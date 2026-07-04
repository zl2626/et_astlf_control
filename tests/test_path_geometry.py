import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from et_astlf_control.path_geometry import (  # noqa: E402
    PathPoint,
    ReferencePath,
    heading_error,
    lateral_offset,
    normalize_angle,
    yaw_from_quaternion,
)


def test_normalize_angle_wraps_to_pi_range():
    assert normalize_angle(3.5) == pytest.approx(3.5 - 2.0 * math.pi)
    assert normalize_angle(-3.5) == pytest.approx(-3.5 + 2.0 * math.pi)


def test_lateral_offset_uses_paper_sign_convention():
    reference = PathPoint(x=0.0, y=0.0, yaw=0.0)

    assert lateral_offset(vehicle_x=0.0, vehicle_y=1.0, reference=reference) == pytest.approx(1.0)
    assert lateral_offset(vehicle_x=0.0, vehicle_y=-1.0, reference=reference) == pytest.approx(-1.0)


def test_heading_error_is_vehicle_yaw_minus_reference_yaw():
    assert heading_error(vehicle_yaw=0.4, reference_yaw=0.1) == pytest.approx(0.3)
    assert heading_error(vehicle_yaw=-3.0, reference_yaw=3.0) == pytest.approx(0.28318530717958623)


def test_yaw_from_quaternion_extracts_planar_heading():
    half = math.pi / 4.0
    yaw = yaw_from_quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    assert yaw == pytest.approx(math.pi / 2.0)


def test_reference_path_computes_segment_heading_and_lookahead_target():
    path = ReferencePath.from_xy([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)])

    target = path.target_from_pose(vehicle_x=0.4, vehicle_y=0.8, lookahead_m=1.0)

    assert target.x == pytest.approx(2.0)
    assert target.y == pytest.approx(0.0)
    assert target.yaw == pytest.approx(0.0)
