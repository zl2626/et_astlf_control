from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from et_astlf_path_tracking.error_plotting import ErrorHistory, parse_debug_sample, save_error_plot


class ErrorPlotterNode(Node):
    """Subscribe to controller debug data and periodically save error plots as PNG files."""

    def __init__(self) -> None:
        super().__init__("error_plotter_node")

        self.declare_parameter("plot_output_dir", "/root/astlf_plots")
        self.declare_parameter("plot_window_s", 60.0)
        self.declare_parameter("plot_save_period_s", 2.0)

        self.output_dir = Path(str(self.get_parameter("plot_output_dir").value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.history = ErrorHistory(float(self.get_parameter("plot_window_s").value))
        self.warned_plot_failure = False
        self.create_subscription(Float64MultiArray, "/et_astlf/debug", self._on_debug, 50)
        save_period = float(self.get_parameter("plot_save_period_s").value)
        self.create_timer(max(0.5, save_period), self._save_plot)
        self.get_logger().info(f"Error plotter saving PNG curves to {self.output_dir}")

    def _on_debug(self, msg: Float64MultiArray) -> None:
        try:
            self.history.add(parse_debug_sample(msg.data))
        except ValueError as exc:
            self.get_logger().warn(f"Ignoring malformed /et_astlf/debug sample: {exc}")

    def _save_plot(self) -> None:
        try:
            save_error_plot(self.history, self.output_dir)
        except Exception as exc:  # pragma: no cover - depends on robot image packages
            if self.warned_plot_failure:
                return
            self.warned_plot_failure = True
            self.get_logger().error(
                "Failed to save ASTLF error plot. Install matplotlib with: "
                f"apt install -y python3-matplotlib. Original error: {exc}"
            )

    def save_final_plot(self) -> None:
        output_path = save_error_plot(self.history, self.output_dir)
        if output_path is not None:
            self.get_logger().info(f"Saved final ASTLF error plot: {output_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ErrorPlotterNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.save_final_plot()
        except Exception as exc:
            node.get_logger().error(f"Failed to save final ASTLF error plot: {exc}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
