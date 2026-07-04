import math
from dataclasses import dataclass
from typing import List, Sequence


SMALL_NUMBER = 1.0e-12


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float


@dataclass(frozen=True)
class ASTLFParams:
    c1: float
    c2: float
    eps: float
    sigma: float
    gamma: float
    c_gain: float
    mu1: float
    mu0: float
    beta1_min: float
    beta1_init: float
    max_steer_angle: float

    def __post_init__(self) -> None:
        if self.c1 <= 0.0 or self.c2 <= 0.0:
            raise ValueError("c1 and c2 must be positive")
        if not -0.5 < self.eps < 0.0:
            raise ValueError("eps must be in (-0.5, 0)")
        if not 0.0 < self.sigma < 1.0:
            raise ValueError("sigma must be in (0, 1)")
        if self.gamma <= 0.0:
            raise ValueError("gamma must be positive")
        if self.c_gain <= 0.0:
            raise ValueError("c_gain must be positive")
        if self.mu1 <= 0.0 or self.mu0 <= 0.0:
            raise ValueError("mu1 and mu0 must be positive")
        if self.beta1_min <= 0.0:
            raise ValueError("beta1_min must be positive")
        if self.beta1_init < self.beta1_min:
            raise ValueError("beta1_init must be greater than or equal to beta1_min")
        if not 0.0 < self.max_steer_angle < math.pi / 2.0:
            raise ValueError("max_steer_angle must be in (0, pi/2)")


@dataclass(frozen=True)
class ASTLFOutput:
    lateral_error: float
    heading_error: float
    sliding: float
    beta1: float
    beta2: float
    u0: float
    u0_dot: float
    raw_u: float
    u: float
    steering_angle: float


def signed_power(value: float, exponent: float) -> float:
    """Return abs(value)^exponent * sign(value), matching the paper notation."""
    if abs(value) < SMALL_NUMBER:
        return 0.0
    return math.copysign(abs(value) ** exponent, value)


def angle_normalize(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_lateral_error(vehicle_x: float, vehicle_y: float, target: PathPoint, theta_d: float) -> float:
    """Los = -(x - xd) * sin(theta_d) + (y - yd) * cos(theta_d)."""
    return -(vehicle_x - target.x) * math.sin(theta_d) + (vehicle_y - target.y) * math.cos(theta_d)


def find_nearest_index(path: Sequence[PathPoint], x: float, y: float) -> int:
    if not path:
        raise ValueError("reference path is empty")

    best_index = 0
    best_distance = float("inf")
    for index, point in enumerate(path):
        distance = math.hypot(point.x - x, point.y - y)
        if distance < best_distance:
            best_distance = distance
            best_index = index
    return best_index


def find_lookahead_target_index(
    path: Sequence[PathPoint],
    x: float,
    y: float,
    lookahead_distance: float,
) -> int:
    """Find nearest path point, then walk forward by approximately lookahead_distance."""
    nearest_index = find_nearest_index(path, x, y)
    if lookahead_distance <= 0.0 or nearest_index >= len(path) - 1:
        return nearest_index

    traveled = 0.0
    for index in range(nearest_index, len(path) - 1):
        current = path[index]
        nxt = path[index + 1]
        traveled += math.hypot(nxt.x - current.x, nxt.y - current.y)
        if traveled >= lookahead_distance:
            return index + 1
    return len(path) - 1


def compute_reference_heading(path: Sequence[PathPoint], target_index: int) -> float:
    """Use target point and the following path point; fall back to previous point at path end."""
    if not path:
        raise ValueError("reference path is empty")
    if len(path) == 1:
        return 0.0

    target_index = max(0, min(target_index, len(path) - 1))
    if target_index < len(path) - 1:
        start = path[target_index]
        end = path[target_index + 1]
    else:
        start = path[target_index - 1]
        end = path[target_index]
    return math.atan2(end.y - start.y, end.x - start.x)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def steering_to_yaw_rate(speed: float, wheelbase: float, steering_angle: float) -> float:
    """Convert steering angle to angular velocity with theta_dot = v / L * tan(delta_f)."""
    if wheelbase <= SMALL_NUMBER or not math.isfinite(wheelbase):
        return 0.0
    if not math.isfinite(speed) or not math.isfinite(steering_angle):
        return 0.0
    return speed / wheelbase * math.tan(steering_angle)


def generate_u_shape_path(
    start_x: float,
    start_y: float,
    start_yaw: float,
    straight_length: float,
    track_width: float,
    point_spacing: float,
) -> List[PathPoint]:
    """Generate a left-turn U path in odom/map coordinates from the current vehicle pose."""
    straight_length = max(point_spacing, straight_length)
    track_width = max(point_spacing, track_width)
    point_spacing = max(0.05, point_spacing)
    radius = track_width / 2.0

    local_points: List[PathPoint] = []
    forward_steps = max(1, int(math.ceil(straight_length / point_spacing)))
    for step in range(forward_steps + 1):
        x_local = min(straight_length, step * point_spacing)
        local_points.append(PathPoint(x_local, 0.0))

    arc_length = math.pi * radius
    arc_steps = max(4, int(math.ceil(arc_length / point_spacing)))
    center_x = straight_length
    center_y = radius
    for step in range(1, arc_steps + 1):
        angle = -math.pi / 2.0 + math.pi * step / arc_steps
        x_local = center_x + radius * math.cos(angle)
        y_local = center_y + radius * math.sin(angle)
        local_points.append(PathPoint(x_local, y_local))

    for step in range(1, forward_steps + 1):
        x_local = max(0.0, straight_length - step * point_spacing)
        local_points.append(PathPoint(x_local, track_width))

    cos_yaw = math.cos(start_yaw)
    sin_yaw = math.sin(start_yaw)
    return [
        PathPoint(
            x=start_x + point.x * cos_yaw - point.y * sin_yaw,
            y=start_y + point.x * sin_yaw + point.y * cos_yaw,
        )
        for point in local_points
    ]


def is_near_goal(path: Sequence[PathPoint], x: float, y: float, tolerance: float) -> bool:
    if not path:
        return False
    goal = path[-1]
    return math.hypot(goal.x - x, goal.y - y) <= max(0.0, tolerance)


class ASTLFController:
    """Discrete continuous-time ASTLF controller without event triggering."""

    def __init__(self, params: ASTLFParams) -> None:
        self.params = params
        self.beta1 = params.beta1_init
        self.u0 = 0.0

    def reset(self) -> None:
        self.beta1 = self.params.beta1_init
        self.u0 = 0.0

    def update(self, lateral_error: float, heading_error: float, speed: float, dt: float) -> ASTLFOutput:
        if not math.isfinite(dt) or dt < 0.0:
            dt = 0.0
        if not math.isfinite(speed):
            speed = 0.0

        sliding = self.params.c1 * lateral_error + self.params.c2 * speed * math.sin(heading_error)
        self._update_beta1(sliding, dt)
        beta2 = self._beta2()

        self.u0_dot = -beta2 * signed_power(sliding, 1.0 + 2.0 * self.params.eps)
        self.u0 += self.u0_dot * dt

        raw_u = -self.beta1 * signed_power(sliding, 1.0 + self.params.eps) + self.u0
        max_u = math.tan(self.params.max_steer_angle)
        limited_u = max(-max_u, min(max_u, raw_u))
        steering = max(-self.params.max_steer_angle, min(self.params.max_steer_angle, math.atan(limited_u)))

        return ASTLFOutput(
            lateral_error=lateral_error,
            heading_error=heading_error,
            sliding=sliding,
            beta1=self.beta1,
            beta2=beta2,
            u0=self.u0,
            u0_dot=self.u0_dot,
            raw_u=raw_u,
            u=limited_u,
            steering_angle=steering,
        )

    def _update_beta1(self, sliding: float, dt: float) -> None:
        if dt <= 0.0:
            return

        omega = self.params.sigma * (abs(sliding) ** (1.0 + self.params.eps) + self.params.gamma)
        sliding_power = abs(sliding) ** (2.0 + 2.0 * self.params.eps)

        if self.beta1 <= self.params.beta1_min:
            beta1_dot = self.params.mu0
        elif sliding_power > omega:
            beta1_dot = self.params.mu1
        else:
            beta1_dot = -self.params.mu1

        self.beta1 = max(self.params.beta1_min, self.beta1 + beta1_dot * dt)

    def _beta2(self) -> float:
        return 2.0 * self.params.c_gain * (1.0 + self.params.eps) * self.beta1
