import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from et_astlf_path_tracking.astlf_math import generate_u_shape_path, yaw_from_quaternion


class UPathPublisherNode(Node):
    """Publish a U-shaped reference path generated from the first received odom pose."""

    def __init__(self) -> None:
        super().__init__("u_path_publisher_node")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("straight_length", 2.0)
        self.declare_parameter("track_width", 1.0)
        self.declare_parameter("point_spacing", 0.10)
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("regenerate_on_start", True)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.straight_length = float(self.get_parameter("straight_length").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.point_spacing = float(self.get_parameter("point_spacing").value)
        self.path_msg = None

        self.path_pub = self.create_publisher(Path, "/reference_path", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(0.1, publish_rate), self._publish_path)
        self.get_logger().info(
            "U path publisher waiting for /odom: "
            f"straight_length={self.straight_length:.2f} m, "
            f"track_width={self.track_width:.2f} m, spacing={self.point_spacing:.2f} m"
        )

    def _on_odom(self, msg: Odometry) -> None:
        if self.path_msg is not None and bool(self.get_parameter("regenerate_on_start").value):
            return

        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        path_points = generate_u_shape_path(
            start_x=pose.position.x,
            start_y=pose.position.y,
            start_yaw=yaw,
            straight_length=self.straight_length,
            track_width=self.track_width,
            point_spacing=self.point_spacing,
        )

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        for point in path_points:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.frame_id
            pose_msg.pose.position.x = point.x
            pose_msg.pose.position.y = point.y
            pose_msg.pose.orientation.w = 1.0
            path_msg.poses.append(pose_msg)

        self.path_msg = path_msg
        self.get_logger().info(f"Generated U-shaped /reference_path with {len(path_msg.poses)} points.")

    def _publish_path(self) -> None:
        if self.path_msg is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = stamp
        for pose in self.path_msg.poses:
            pose.header.stamp = stamp
        self.path_pub.publish(self.path_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UPathPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
