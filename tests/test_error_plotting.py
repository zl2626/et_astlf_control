import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from et_astlf_path_tracking.error_plotting import ErrorHistory, parse_debug_sample  # noqa: E402


def test_parse_debug_sample_accepts_controller_array_layout():
    sample = parse_debug_sample([1.5, 0.1, -0.2, 0.03, 4.0, 0.8, 0.2, 0.1, 0.5, 0.15])

    assert sample.time_s == pytest.approx(1.5)
    assert sample.lateral_error == pytest.approx(0.1)
    assert sample.heading_error == pytest.approx(-0.2)
    assert sample.sliding == pytest.approx(0.03)
    assert sample.steering_angle == pytest.approx(0.1)
    assert sample.speed == pytest.approx(0.5)
    assert sample.yaw_rate == pytest.approx(0.15)


def test_parse_debug_sample_rejects_short_arrays():
    with pytest.raises(ValueError, match="at least 10"):
        parse_debug_sample([0.0, 1.0])


def test_error_history_prunes_old_samples_by_time_window():
    history = ErrorHistory(window_s=2.0)
    history.add(parse_debug_sample([0.0, 1.0, 0.0, 0.0, 4.0, 0.8, 0.0, 0.0, 0.5, 0.0]))
    history.add(parse_debug_sample([1.0, 2.0, 0.0, 0.0, 4.0, 0.8, 0.0, 0.0, 0.5, 0.0]))
    history.add(parse_debug_sample([3.1, 3.0, 0.0, 0.0, 4.0, 0.8, 0.0, 0.0, 0.5, 0.0]))

    assert [sample.time_s for sample in history.samples] == [3.1]
    assert history.series("lateral_error") == [3.0]
