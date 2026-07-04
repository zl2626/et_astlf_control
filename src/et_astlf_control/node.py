import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from et_astlf_control.controller import ETASTLFController, ETASTLFParams
from et_astlf_control.path_geometry import (
    ReferencePath,
    heading_error,
    lateral_offset,
    yaw_from_quaternion,
)


class ETASTLFNode(Node):
    def __init__(self) -> None:
        super().__init__("et_astlf_node")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("cmd_topic", "/ackermann_cmd")
        self.declare_parameter("debug_topic", "/et_astlf/debug")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("reference_lookahead_m", 0.0)

        self.declare_parameter("wheelbase_m", 2.5)
        self.declare_parameter("speed_mps", 1.0)
        self.declare_parameter("max_steering_angle_rad", math.radians(27.0))
        self.declare_parameter("c1", 0.3)
        self.declare_parameter("c2", 1.0)
        self.declare_parameter("epsilon", -0.42)
        self.declare_parameter("sigma", 0.4)
        self.declare_parameter("gamma", 0.2)
        self.declare_parameter("super_twisting_c", 0.18)
        self.declare_parameter("mu1", 0.02)
        self.declare_parameter("mu0", 0.02)
        self.declare_parameter("beta1_min", 4.0)
        self.declare_parameter("beta1_initial", 4.48)

        params = ETASTLFParams(
            wheelbase_m=float(self.get_parameter("wheelbase_m").value),
            speed_mps=float(self.get_parameter("speed_mps").value),
            max_steering_angle_rad=float(self.get_parameter("max_steering_angle_rad").value),
            c1=float(self.get_parameter("c1").value),
            c2=float(self.get_parameter("c2").value),
            epsilon=float(self.get_parameter("epsilon").value),
            sigma=float(self.get_parameter("sigma").value),
            gamma=float(self.get_parameter("gamma").value),
            super_twisting_c=float(self.get_parameter("super_twisting_c").value),
            mu1=float(self.get_parameter("mu1").value),
            mu0=float(self.get_parameter("mu0").value),
            beta1_min=float(self.get_parameter("beta1_min").value),
            beta1_initial=float(self.get_parameter("beta1_initial").value),
        )
        self.controller = ETASTLFController(params)
        self.reference_path: Optional[ReferencePath] = None
        self.latest_odom: Optional[Odometry] = None

        odom_topic = str(self.get_parameter("odom_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, debug_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Path, path_topic, self._on_path, 10)

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"ET-ASTLF node ready: odom={odom_topic}, path={path_topic}, cmd={cmd_topic}, speed={params.speed_mps:.3f} m/s"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _on_path(self, msg: Path) -> None:
        xy_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if not xy_points:
            self.reference_path = None
            self.get_logger().warn("Received empty path; controller output paused")
            return
        self.reference_path = ReferencePath.from_xy(xy_points)

    def _on_timer(self) -> None:
        if self.latest_odom is None or self.reference_path is None:
            return

        pose = self.latest_odom.pose.pose
        orientation = pose.orientation
        vehicle_yaw = yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        lookahead = float(self.get_parameter("reference_lookahead_m").value)
        reference = self.reference_path.target_from_pose(pose.position.x, pose.position.y, lookahead)

        x1 = lateral_offset(pose.position.x, pose.position.y, reference)
        x2 = heading_error(vehicle_yaw, reference.yaw)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        output = self.controller.update(x1, x2, now_s)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = output.speed_mps
        cmd.drive.steering_angle = output.steering_angle_rad
        self.cmd_pub.publish(cmd)

        if bool(self.get_parameter("publish_debug").value):
            debug = Float64MultiArray()
            debug.data = [
                x1,
                x2,
                output.sliding,
                output.held_sliding,
                output.steering_angle_rad,
                output.virtual_control,
                output.beta1,
                output.beta2,
                float(output.triggered),
                float(output.trigger_count),
                output.trigger_interval_s,
            ]
            self.debug_pub.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ETASTLFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
