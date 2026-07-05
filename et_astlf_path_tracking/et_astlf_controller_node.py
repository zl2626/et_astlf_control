import math
from pathlib import Path as FilePath
from typing import List, Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from et_astlf_path_tracking.astlf_math import (
    ASTLFController,
    ASTLFParams,
    PathPoint,
    angle_normalize,
    compute_lateral_error,
    compute_reference_heading,
    find_lookahead_target_index,
    is_near_goal,
    steering_to_yaw_rate,
    yaw_from_quaternion,
)
from et_astlf_path_tracking.error_plotting import ErrorHistory, ErrorSample, save_error_plot


class ETASTLFControllerNode(Node):
    """ROS2 adapter for continuous ASTLF path tracking.

    The event-triggered interface is intentionally reserved through the
    use_event_trigger parameter, but this version runs continuous ASTLF control
    when use_event_trigger is false.
    """

    MIN_SPEED = 1.0e-3
    MAX_REASONABLE_DT = 0.5

    def __init__(self) -> None:
        super().__init__("et_astlf_controller_node")

        self._declare_parameters()
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.use_event_trigger = bool(self.get_parameter("use_event_trigger").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.save_error_plot_on_shutdown = bool(self.get_parameter("save_error_plot_on_shutdown").value)
        self.plot_output_dir = FilePath(str(self.get_parameter("plot_output_dir").value)).expanduser()
        self.cmd_output_type = str(self.get_parameter("cmd_output_type").value).lower()
        if self.cmd_output_type not in ("twist", "ackermann", "both"):
            self.get_logger().warn(
                f"Unsupported cmd_output_type={self.cmd_output_type}; falling back to twist output."
            )
            self.cmd_output_type = "twist"

        params = ASTLFParams(
            c1=float(self.get_parameter("c1").value),
            c2=float(self.get_parameter("c2").value),
            eps=float(self.get_parameter("eps").value),
            sigma=float(self.get_parameter("sigma").value),
            gamma=float(self.get_parameter("gamma").value),
            c_gain=float(self.get_parameter("c_gain").value),
            mu1=float(self.get_parameter("mu1").value),
            mu0=float(self.get_parameter("mu0").value),
            beta1_min=float(self.get_parameter("beta1_min").value),
            beta1_init=float(self.get_parameter("beta1_init").value),
            max_steer_angle=float(self.get_parameter("max_steer_angle").value),
        )
        self.controller = ASTLFController(params)

        self.latest_odom: Optional[Odometry] = None
        self.reference_path: List[PathPoint] = []
        self.last_control_time_s: Optional[float] = None
        self.last_debug_log_s = 0.0
        self.last_goal_log_s = 0.0
        self.warned_missing_odom = False
        self.warned_missing_path = False
        self.error_history = ErrorHistory(float(self.get_parameter("plot_window_s").value))

        self.cmd_pub = None
        self.cmd_vel_pub = None
        self.debug_pub = None
        if self.cmd_output_type in ("ackermann", "both"):
            self.cmd_pub = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        if self.cmd_output_type in ("twist", "both"):
            self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        if self.publish_debug:
            self.debug_pub = self.create_publisher(Float64MultiArray, "/et_astlf/debug", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PathMsg, "/reference_path", self._on_reference_path, 10)

        timer_period = 1.0 / max(self.control_rate, 1.0)
        self.create_timer(timer_period, self._control_timer_callback)

        if self.use_event_trigger:
            self.get_logger().warn(
                "use_event_trigger=true was requested, but this version only reserves the ET-ASTLF interface. "
                "Running continuous ASTLF control."
            )

        self.get_logger().info(
            "ASTLF controller started: /odom + /reference_path -> control command, "
            f"cmd_output_type={self.cmd_output_type}, "
            f"target_speed={self.target_speed:.3f} m/s, lookahead={self.lookahead_distance:.3f} m"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("wheelbase", 0.32)
        self.declare_parameter("target_speed", 0.5)
        self.declare_parameter("max_steer_angle", 0.45)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("c1", 0.3)
        self.declare_parameter("c2", 1.0)
        self.declare_parameter("eps", -0.42)
        self.declare_parameter("sigma", 0.4)
        self.declare_parameter("gamma", 0.2)
        self.declare_parameter("c_gain", 0.18)
        self.declare_parameter("mu1", 0.02)
        self.declare_parameter("mu0", 0.02)
        self.declare_parameter("beta1_min", 4.0)
        self.declare_parameter("beta1_init", 4.48)
        self.declare_parameter("use_event_trigger", False)
        self.declare_parameter("lookahead_distance", 0.6)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("save_error_plot_on_shutdown", True)
        self.declare_parameter("plot_output_dir", "/root/astlf_plots")
        self.declare_parameter("plot_window_s", 600.0)
        self.declare_parameter("cmd_output_type", "twist")

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.warned_missing_odom = False

    def _on_reference_path(self, msg: PathMsg) -> None:
        self.reference_path = [PathPoint(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.warned_missing_path = False
        if not self.reference_path:
            self.get_logger().warn("Received empty /reference_path; control output is paused.")

    def _control_timer_callback(self) -> None:
        if self.latest_odom is None:
            if not self.warned_missing_odom:
                self.get_logger().warn("Waiting for /odom before publishing control command.")
                self.warned_missing_odom = True
            return

        if not self.reference_path:
            if not self.warned_missing_path:
                self.get_logger().warn("Waiting for non-empty /reference_path before publishing control command.")
                self.warned_missing_path = True
            return

        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        dt = self._compute_dt(now_s)

        pose = self.latest_odom.pose.pose
        twist = self.latest_odom.twist.twist
        x = pose.position.x
        y = pose.position.y
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        measured_speed = abs(twist.linear.x)
        speed_for_model = measured_speed if measured_speed >= self.MIN_SPEED else self.target_speed

        target_index = find_lookahead_target_index(self.reference_path, x, y, self.lookahead_distance)
        target = self.reference_path[target_index]
        theta_d = compute_reference_heading(self.reference_path, target_index)

        los = compute_lateral_error(x, y, target, theta_d)
        theta_os = angle_normalize(yaw - theta_d)
        output = self.controller.update(los, theta_os, speed_for_model, dt)
        yaw_rate = steering_to_yaw_rate(self.target_speed, self.wheelbase, output.steering_angle)
        self._record_debug_sample(now_s, output, yaw_rate)

        if is_near_goal(self.reference_path, x, y, self.goal_tolerance):
            self._publish_stop_command()
            if now_s - self.last_goal_log_s >= 1.0:
                self.last_goal_log_s = now_s
                self.get_logger().info("Goal reached; publishing zero /cmd_vel.")
            return

        self._publish_control_command(output.steering_angle)

        self._log_debug(now_s, output)

    def _publish_control_command(self, steering_angle: float) -> None:
        if self.cmd_output_type in ("ackermann", "both"):
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.drive.speed = self.target_speed
            cmd.drive.steering_angle = steering_angle
            if self.cmd_pub is not None:
                self.cmd_pub.publish(cmd)

        if self.cmd_output_type in ("twist", "both"):
            twist = Twist()
            twist.linear.x = self.target_speed
            twist.angular.z = steering_to_yaw_rate(self.target_speed, self.wheelbase, steering_angle)
            if self.cmd_vel_pub is not None:
                self.cmd_vel_pub.publish(twist)

    def _publish_stop_command(self) -> None:
        if self.cmd_output_type in ("ackermann", "both") and self.cmd_pub is not None:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_link"
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            self.cmd_pub.publish(cmd)

        if self.cmd_output_type in ("twist", "both") and self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(Twist())

    def _record_debug_sample(self, now_s: float, output, yaw_rate: float) -> None:
        sample = ErrorSample(
            time_s=now_s,
            lateral_error=output.lateral_error,
            heading_error=output.heading_error,
            sliding=output.sliding,
            beta1=output.beta1,
            beta2=output.beta2,
            control_u=output.u,
            steering_angle=output.steering_angle,
            speed=self.target_speed,
            yaw_rate=yaw_rate,
        )
        self.error_history.add(sample)

        if self.debug_pub is None:
            return
        msg = Float64MultiArray()
        msg.data = [
            sample.time_s,
            sample.lateral_error,
            sample.heading_error,
            sample.sliding,
            sample.beta1,
            sample.beta2,
            sample.control_u,
            sample.steering_angle,
            sample.speed,
            sample.yaw_rate,
        ]
        self.debug_pub.publish(msg)

    def save_final_error_plot(self) -> None:
        if not self.save_error_plot_on_shutdown:
            return

        try:
            output_path = save_error_plot(self.error_history, self.plot_output_dir)
        except Exception as exc:  # pragma: no cover - depends on robot image packages
            self.get_logger().error(
                "Failed to save final ASTLF error plot. Install matplotlib with: "
                f"apt install -y python3-matplotlib. Original error: {exc}"
            )
            return

        if output_path is None:
            self.get_logger().warn("No final ASTLF error plot saved because fewer than two samples were recorded.")
            return
        self.get_logger().info(f"Saved final ASTLF error plot: {output_path}")

    def _compute_dt(self, now_s: float) -> float:
        nominal_dt = 1.0 / max(self.control_rate, 1.0)
        if self.last_control_time_s is None:
            self.last_control_time_s = now_s
            return nominal_dt

        dt = now_s - self.last_control_time_s
        self.last_control_time_s = now_s
        if not math.isfinite(dt) or dt <= 0.0 or dt > self.MAX_REASONABLE_DT:
            self.get_logger().warn(f"Abnormal dt={dt:.6f}; using nominal dt={nominal_dt:.6f}.")
            return nominal_dt
        return dt

    def _log_debug(self, now_s: float, output) -> None:
        if now_s - self.last_debug_log_s < 1.0:
            return
        self.last_debug_log_s = now_s
        self.get_logger().info(
            "ASTLF debug: "
            f"Los={output.lateral_error:.4f}, "
            f"theta_os={output.heading_error:.4f}, "
            f"s={output.sliding:.4f}, "
            f"beta1={output.beta1:.4f}, "
            f"beta2={output.beta2:.4f}, "
            f"u={output.u:.4f}, "
            f"delta_f={output.steering_angle:.4f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ETASTLFControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node._publish_stop_command()
        node.save_final_error_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
