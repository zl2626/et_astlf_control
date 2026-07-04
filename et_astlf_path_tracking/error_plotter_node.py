from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from et_astlf_path_tracking.error_plotting import ErrorHistory, parse_debug_sample


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
        self.plot_available = True

        try:
            import matplotlib

            matplotlib.use("Agg")
            from matplotlib import pyplot as plt

            self.plt = plt
        except Exception as exc:  # pragma: no cover - depends on robot image packages
            self.plot_available = False
            self.plt = None
            self.get_logger().error(
                "matplotlib is not available; install it with: apt install -y python3-matplotlib. "
                f"Original error: {exc}"
            )

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
        if not self.plot_available or len(self.history.samples) < 2:
            return

        times = self.history.series("time_s")
        start_time = times[0]
        t = [item - start_time for item in times]

        fig, axes = self.plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        fig.suptitle("ASTLF path-tracking debug curves")

        axes[0].plot(t, self.history.series("lateral_error"), label="Los (m)")
        axes[0].axhline(0.0, color="black", linewidth=0.7)
        axes[0].set_ylabel("Los (m)")
        axes[0].grid(True)

        axes[1].plot(t, self.history.series("heading_error"), label="theta_os (rad)", color="tab:orange")
        axes[1].axhline(0.0, color="black", linewidth=0.7)
        axes[1].set_ylabel("theta_os")
        axes[1].grid(True)

        axes[2].plot(t, self.history.series("sliding"), label="s", color="tab:green")
        axes[2].axhline(0.0, color="black", linewidth=0.7)
        axes[2].set_ylabel("s")
        axes[2].grid(True)

        axes[3].plot(t, self.history.series("steering_angle"), label="delta_f (rad)", color="tab:red")
        axes[3].plot(t, self.history.series("yaw_rate"), label="cmd angular.z", color="tab:purple")
        axes[3].set_ylabel("command")
        axes[3].set_xlabel("time (s)")
        axes[3].legend(loc="upper right")
        axes[3].grid(True)

        fig.tight_layout()
        output_path = self.output_dir / "astlf_error_curves.png"
        fig.savefig(output_path, dpi=130)
        self.plt.close(fig)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ErrorPlotterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
