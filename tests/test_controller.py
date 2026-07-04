import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from et_astlf_control.controller import (  # noqa: E402
    ETASTLFController,
    ETASTLFParams,
    signed_power,
)


def paper_params(**overrides):
    values = {
        "wheelbase_m": 2.5,
        "speed_mps": 1.0,
        "max_steering_angle_rad": math.radians(27.0),
        "c1": 0.3,
        "c2": 1.0,
        "epsilon": -0.42,
        "sigma": 0.4,
        "gamma": 0.2,
        "super_twisting_c": 0.18,
        "mu1": 0.02,
        "mu0": 0.02,
        "beta1_min": 4.0,
        "beta1_initial": 4.48,
    }
    values.update(overrides)
    return ETASTLFParams(**values)


def test_signed_power_preserves_sign_for_fractional_exponents():
    assert signed_power(-4.0, 0.5) == pytest.approx(-2.0)
    assert signed_power(4.0, 0.5) == pytest.approx(2.0)
    assert signed_power(0.0, 0.5) == 0.0


def test_first_update_uses_paper_sliding_surface_and_virtual_control():
    params = paper_params()
    controller = ETASTLFController(params)

    output = controller.update(lateral_error_m=0.02, heading_error_rad=0.01, now_s=0.0)

    expected_s = params.c1 * 0.02 + params.c2 * params.speed_mps * math.sin(0.01)
    expected_u = -params.beta1_initial * signed_power(expected_s, 1.0 + params.epsilon)

    assert output.triggered is True
    assert output.sliding == pytest.approx(expected_s)
    assert output.virtual_control == pytest.approx(expected_u)
    assert output.steering_angle_rad == pytest.approx(math.atan(expected_u))
    assert output.beta1 == pytest.approx(params.beta1_initial)


def test_first_update_does_not_integrate_from_absolute_clock_time():
    params = paper_params()
    controller = ETASTLFController(params)

    output = controller.update(lateral_error_m=0.02, heading_error_rad=0.01, now_s=1234.0)

    expected_s = params.c1 * 0.02 + params.c2 * params.speed_mps * math.sin(0.01)
    expected_u = -params.beta1_initial * signed_power(expected_s, 1.0 + params.epsilon)

    assert output.virtual_control == pytest.approx(expected_u)
    assert output.u0 == pytest.approx(0.0)
    assert output.beta1 == pytest.approx(params.beta1_initial)


def test_small_sliding_change_does_not_retrigger_event():
    controller = ETASTLFController(paper_params())

    first = controller.update(lateral_error_m=0.02, heading_error_rad=0.01, now_s=0.0)
    second = controller.update(lateral_error_m=0.0201, heading_error_rad=0.0101, now_s=0.02)

    assert first.triggered is True
    assert second.triggered is False
    assert second.held_sliding == pytest.approx(first.held_sliding)
    assert second.trigger_count == 1


def test_large_command_is_limited_by_steering_angle():
    params = paper_params(max_steering_angle_rad=0.25)
    controller = ETASTLFController(params)

    output = controller.update(lateral_error_m=2.0, heading_error_rad=0.7, now_s=0.0)

    assert output.steering_angle_rad == pytest.approx(-0.25)
    assert output.virtual_control == pytest.approx(-math.tan(0.25))


def test_beta1_adapts_but_never_falls_below_minimum():
    params = paper_params(beta1_initial=4.0, beta1_min=4.0, mu0=0.5)
    controller = ETASTLFController(params)

    controller.update(lateral_error_m=0.0, heading_error_rad=0.0, now_s=0.0)
    output = controller.update(lateral_error_m=0.0, heading_error_rad=0.0, now_s=1.0)

    assert output.beta1 >= params.beta1_min
    assert output.beta1 == pytest.approx(4.5)
    assert output.beta2 == pytest.approx(2.0 * params.super_twisting_c * (1.0 + params.epsilon) * output.beta1)
