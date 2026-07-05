from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional, Sequence


@dataclass(frozen=True)
class ErrorSample:
    time_s: float
    lateral_error: float
    heading_error: float
    sliding: float
    beta1: float
    beta2: float
    control_u: float
    steering_angle: float
    speed: float
    yaw_rate: float


class ErrorHistory:
    def __init__(self, window_s: float) -> None:
        self.window_s = max(1.0, window_s)
        self.samples: List[ErrorSample] = []

    def add(self, sample: ErrorSample) -> None:
        self.samples.append(sample)
        cutoff = sample.time_s - self.window_s
        self.samples = [item for item in self.samples if item.time_s >= cutoff]

    def series(self, field_name: str) -> List[float]:
        return [getattr(sample, field_name) for sample in self.samples]


def parse_debug_sample(values: Sequence[float]) -> ErrorSample:
    if len(values) < 10:
        raise ValueError("debug sample must contain at least 10 values")
    return ErrorSample(
        time_s=float(values[0]),
        lateral_error=float(values[1]),
        heading_error=float(values[2]),
        sliding=float(values[3]),
        beta1=float(values[4]),
        beta2=float(values[5]),
        control_u=float(values[6]),
        steering_angle=float(values[7]),
        speed=float(values[8]),
        yaw_rate=float(values[9]),
    )


def _load_pyplot() -> Any:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    return plt


def save_error_plot(
    history: ErrorHistory,
    output_dir: str | Path,
    filename: str = "astlf_error_curves.png",
    pyplot: Optional[Any] = None,
) -> Optional[Path]:
    """Save ASTLF tracking error curves and return the PNG path.

    The function is kept independent from ROS so the controller can call it
    during shutdown, and the plotter node can call it periodically.
    """

    output_path = Path(output_dir).expanduser() / filename
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt = pyplot if pyplot is not None else _load_pyplot()

    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    fig.suptitle("ASTLF path-tracking debug curves")

    if not history.samples:
        fig.text(
            0.5,
            0.5,
            "No ASTLF samples recorded yet.\nCheck /odom and /reference_path, then run the controller again.",
            ha="center",
            va="center",
        )
        fig.tight_layout()
        fig.savefig(output_path, dpi=130)
        plt.close(fig)
        return output_path

    times = history.series("time_s")
    start_time = times[0]
    t = [item - start_time for item in times]

    axes[0].plot(t, history.series("lateral_error"), label="Los (m)")
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("Los (m)")
    axes[0].grid(True)

    axes[1].plot(t, history.series("heading_error"), label="theta_os (rad)", color="tab:orange")
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("theta_os")
    axes[1].grid(True)

    axes[2].plot(t, history.series("sliding"), label="s", color="tab:green")
    axes[2].axhline(0.0, color="black", linewidth=0.7)
    axes[2].set_ylabel("s")
    axes[2].grid(True)

    axes[3].plot(t, history.series("steering_angle"), label="delta_f (rad)", color="tab:red")
    axes[3].plot(t, history.series("yaw_rate"), label="cmd angular.z", color="tab:purple")
    axes[3].set_ylabel("command")
    axes[3].set_xlabel("time (s)")
    axes[3].legend(loc="upper right")
    axes[3].grid(True)

    fig.tight_layout()
    fig.savefig(output_path, dpi=130)
    plt.close(fig)
    return output_path
