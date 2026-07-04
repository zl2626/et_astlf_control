import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from et_astlf_control.csv_path import load_xy_csv


def yaw_to_quaternion(yaw_rad: float):
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


class CSVPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("csv_path_publisher")

        self.declare_parameter("csv_file", "")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 1.0)

        self.publisher = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 1)
        self.path_msg = self._load_path_message()

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish_path)
        self.get_logger().info(
            f"CSV path publisher ready: {len(self.path_msg.poses)} points -> {self.get_parameter('path_topic').value}"
        )

    def _load_path_message(self) -> Path:
        csv_file = str(self.get_parameter("csv_file").value)
        if not csv_file:
            raise ValueError("csv_file parameter is required")

        points = load_xy_csv(csv_file)
        frame_id = str(self.get_parameter("frame_id").value)

        path_msg = Path()
        path_msg.header.frame_id = frame_id
        for index, (x, y) in enumerate(points):
            if len(points) == 1:
                yaw = 0.0
            elif index < len(points) - 1:
                next_x, next_y = points[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                prev_x, prev_y = points[index - 1]
                yaw = math.atan2(y - prev_y, x - prev_x)

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)
        return path_msg

    def _publish_path(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = stamp
        for pose in self.path_msg.poses:
            pose.header.stamp = stamp
        self.publisher.publish(self.path_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CSVPathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
