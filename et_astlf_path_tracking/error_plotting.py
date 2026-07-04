from dataclasses import dataclass
from typing import List, Sequence


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
