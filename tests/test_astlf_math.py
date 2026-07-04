import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from et_astlf_path_tracking.astlf_math import (  # noqa: E402
    ASTLFController,
    ASTLFParams,
    PathPoint,
    angle_normalize,
    compute_lateral_error,
    compute_reference_heading,
    find_lookahead_target_index,
    signed_power,
    steering_to_yaw_rate,
)


def default_params(**overrides):
    values = {
        "c1": 0.3,
        "c2": 1.0,
        "eps": -0.42,
        "sigma": 0.4,
        "gamma": 0.2,
        "c_gain": 0.18,
        "mu1": 0.02,
        "mu0": 0.02,
        "beta1_min": 4.0,
        "beta1_init": 4.48,
        "max_steer_angle": 0.45,
    }
    values.update(overrides)
    return ASTLFParams(**values)


def test_signed_power_preserves_sign_and_handles_zero():
    assert signed_power(-4.0, 0.5) == pytest.approx(-2.0)
    assert signed_power(4.0, 0.5) == pytest.approx(2.0)
    assert signed_power(0.0, 0.16) == 0.0


def test_angle_normalize_returns_minus_pi_to_pi_range():
    assert angle_normalize(3.5) == pytest.approx(3.5 - 2.0 * math.pi)
    assert angle_normalize(-3.5) == pytest.approx(-3.5 + 2.0 * math.pi)


def test_lateral_error_uses_paper_definition():
    target = PathPoint(x=0.0, y=0.0)
    theta_d = 0.0

    assert compute_lateral_error(0.0, 1.0, target, theta_d) == pytest.approx(1.0)
    assert compute_lateral_error(0.0, -1.0, target, theta_d) == pytest.approx(-1.0)


def test_lookahead_target_starts_at_nearest_path_point_and_moves_forward():
    path = [
        PathPoint(0.0, 0.0),
        PathPoint(0.5, 0.0),
        PathPoint(1.0, 0.0),
        PathPoint(1.5, 0.0),
    ]

    index = find_lookahead_target_index(path, x=0.1, y=0.2, lookahead_distance=0.6)

    assert index == 2


def test_reference_heading_uses_target_and_next_point():
    path = [PathPoint(0.0, 0.0), PathPoint(1.0, 0.0), PathPoint(1.0, 1.0)]

    assert compute_reference_heading(path, 1) == pytest.approx(math.pi / 2.0)


def test_astlf_first_update_matches_requested_equations():
    params = default_params()
    controller = ASTLFController(params)

    output = controller.update(lateral_error=0.02, heading_error=0.01, speed=0.5, dt=0.02)

    expected_s = params.c1 * 0.02 + params.c2 * 0.5 * math.sin(0.01)
    expected_beta1 = params.beta1_init - params.mu1 * 0.02
    expected_beta2 = 2.0 * params.c_gain * (1.0 + params.eps) * expected_beta1
    expected_u0 = -expected_beta2 * signed_power(expected_s, 1.0 + 2.0 * params.eps) * 0.02
    expected_u = -expected_beta1 * signed_power(expected_s, 1.0 + params.eps) + expected_u0

    assert output.sliding == pytest.approx(expected_s)
    assert output.beta1 == pytest.approx(expected_beta1)
    assert output.beta2 == pytest.approx(expected_beta2)
    assert output.u0 == pytest.approx(expected_u0)
    assert output.u == pytest.approx(expected_u)
    assert output.steering_angle == pytest.approx(math.atan(expected_u))


def test_astlf_limits_steering_after_atan_conversion():
    params = default_params(max_steer_angle=0.2)
    controller = ASTLFController(params)

    output = controller.update(lateral_error=4.0, heading_error=0.7, speed=0.5, dt=0.02)

    assert output.steering_angle == pytest.approx(-0.2)
    assert output.u == pytest.approx(-math.tan(0.2))


def test_steering_to_yaw_rate_uses_bicycle_model():
    yaw_rate = steering_to_yaw_rate(speed=0.5, wheelbase=0.32, steering_angle=0.2)

    assert yaw_rate == pytest.approx(0.5 / 0.32 * math.tan(0.2))


def test_steering_to_yaw_rate_returns_zero_for_invalid_wheelbase():
    assert steering_to_yaw_rate(speed=0.5, wheelbase=0.0, steering_angle=0.2) == 0.0
    assert steering_to_yaw_rate(speed=0.5, wheelbase=-0.1, steering_angle=0.2) == 0.0


def test_beta1_decreases_only_to_minimum_then_uses_mu0_when_at_minimum():
    params = default_params(beta1_init=4.01, beta1_min=4.0, mu1=0.02, mu0=0.5)
    controller = ASTLFController(params)

    first = controller.update(lateral_error=0.0, heading_error=0.0, speed=0.5, dt=1.0)
    second = controller.update(lateral_error=0.0, heading_error=0.0, speed=0.5, dt=1.0)

    assert first.beta1 == pytest.approx(4.0)
    assert second.beta1 == pytest.approx(4.5)
