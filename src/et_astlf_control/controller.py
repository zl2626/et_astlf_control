import math
from dataclasses import dataclass
from typing import Optional


def signed_power(value: float, exponent: float) -> float:
    """Paper notation floor(value)^exponent: sign(value) * abs(value)^exponent."""
    if value == 0.0:
        return 0.0
    return math.copysign(abs(value) ** exponent, value)


@dataclass(frozen=True)
class ETASTLFParams:
    wheelbase_m: float
    speed_mps: float
    max_steering_angle_rad: float
    c1: float
    c2: float
    epsilon: float
    sigma: float
    gamma: float
    super_twisting_c: float
    mu1: float
    mu0: float
    beta1_min: float
    beta1_initial: float

    def __post_init__(self) -> None:
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if self.speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")
        if not 0.0 < self.max_steering_angle_rad < math.pi / 2.0:
            raise ValueError("max_steering_angle_rad must be in (0, pi/2)")
        if self.c1 <= 0.0 or self.c2 <= 0.0:
            raise ValueError("c1 and c2 must be positive")
        if not -0.5 < self.epsilon < 0.0:
            raise ValueError("epsilon must be in (-0.5, 0)")
        if not 0.0 < self.sigma < 1.0:
            raise ValueError("sigma must be in (0, 1)")
        if self.gamma <= 0.0:
            raise ValueError("gamma must be positive")
        if self.super_twisting_c <= 0.0:
            raise ValueError("super_twisting_c must be positive")
        if self.mu0 <= 0.0 or self.mu1 <= 0.0:
            raise ValueError("mu0 and mu1 must be positive")
        if self.beta1_min <= 0.0:
            raise ValueError("beta1_min must be positive")
        if self.beta1_initial < self.beta1_min:
            raise ValueError("beta1_initial must be greater than or equal to beta1_min")


@dataclass(frozen=True)
class ETASTLFOutput:
    steering_angle_rad: float
    virtual_control: float
    speed_mps: float
    sliding: float
    held_sliding: float
    beta1: float
    beta2: float
    u0: float
    triggered: bool
    trigger_count: int
    trigger_interval_s: float


class ETASTLFController:
    """Event-triggered ASTLF controller for the path-offset model in the paper."""

    def __init__(self, params: ETASTLFParams) -> None:
        self.params = params
        self.beta1 = params.beta1_initial
        self.u0 = 0.0
        self._last_update_s: Optional[float] = None
        self._last_trigger_s = 0.0
        self._held_sliding = 0.0
        self._held_beta1 = params.beta1_initial
        self._held_beta2 = self._beta2(params.beta1_initial)
        self._has_trigger = False
        self._trigger_count = 0

    def reset(self, now_s: float = 0.0) -> None:
        self.beta1 = self.params.beta1_initial
        self.u0 = 0.0
        self._last_update_s = now_s
        self._last_trigger_s = now_s
        self._held_sliding = 0.0
        self._held_beta1 = self.beta1
        self._held_beta2 = self._beta2(self.beta1)
        self._has_trigger = False
        self._trigger_count = 0

    def sliding_surface(self, lateral_error_m: float, heading_error_rad: float) -> float:
        return (
            self.params.c1 * lateral_error_m
            + self.params.c2 * self.params.speed_mps * math.sin(heading_error_rad)
        )

    def update(self, lateral_error_m: float, heading_error_rad: float, now_s: float) -> ETASTLFOutput:
        dt = 0.0 if self._last_update_s is None else max(0.0, now_s - self._last_update_s)
        sliding = self.sliding_surface(lateral_error_m, heading_error_rad)

        self._adapt_beta(sliding, dt)

        triggered = (not self._has_trigger) or self._should_trigger(sliding)
        trigger_interval = 0.0
        if triggered:
            trigger_interval = 0.0 if not self._has_trigger else max(0.0, now_s - self._last_trigger_s)
            self._held_sliding = sliding
            self._held_beta1 = self.beta1
            self._held_beta2 = self._beta2(self.beta1)
            self._last_trigger_s = now_s
            self._has_trigger = True
            self._trigger_count += 1

        self.u0 += -self._held_beta2 * signed_power(self._held_sliding, 1.0 + 2.0 * self.params.epsilon) * dt
        raw_u = -self._held_beta1 * signed_power(self._held_sliding, 1.0 + self.params.epsilon) + self.u0
        limited_u = self._limit_virtual_control(raw_u)
        steering = math.atan(limited_u)

        self._last_update_s = now_s

        return ETASTLFOutput(
            steering_angle_rad=steering,
            virtual_control=limited_u,
            speed_mps=self.params.speed_mps,
            sliding=sliding,
            held_sliding=self._held_sliding,
            beta1=self.beta1,
            beta2=self._beta2(self.beta1),
            u0=self.u0,
            triggered=triggered,
            trigger_count=self._trigger_count,
            trigger_interval_s=trigger_interval,
        )

    def _adapt_beta(self, sliding: float, dt: float) -> None:
        if dt <= 0.0:
            return

        omega = self.params.sigma * (abs(self._held_sliding) ** (1.0 + self.params.epsilon) + self.params.gamma)
        if self.beta1 <= self.params.beta1_min:
            beta_dot = self.params.mu0
        else:
            beta_dot = self.params.mu1 if abs(sliding) ** (2.0 + 2.0 * self.params.epsilon) >= omega else -self.params.mu1

        self.beta1 = max(self.params.beta1_min, self.beta1 + beta_dot * dt)

    def _should_trigger(self, sliding: float) -> bool:
        threshold = self.params.sigma * (abs(self._held_sliding) ** (1.0 + self.params.epsilon) + self.params.gamma)
        event_error = abs(
            signed_power(self._held_sliding, 2.0 + 2.0 * self.params.epsilon)
            - signed_power(sliding, 2.0 + 2.0 * self.params.epsilon)
        )
        return event_error >= threshold

    def _limit_virtual_control(self, virtual_control: float) -> float:
        max_u = math.tan(self.params.max_steering_angle_rad)
        return min(max(virtual_control, -max_u), max_u)

    def _beta2(self, beta1: float) -> float:
        return 2.0 * self.params.super_twisting_c * (1.0 + self.params.epsilon) * beta1
