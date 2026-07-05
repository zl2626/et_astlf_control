import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from et_astlf_path_tracking.error_plotting import ErrorHistory, parse_debug_sample, save_error_plot  # noqa: E402


class _FakeAxis:
    def plot(self, *args, **kwargs):
        return None

    def axhline(self, *args, **kwargs):
        return None

    def set_ylabel(self, *args, **kwargs):
        return None

    def set_xlabel(self, *args, **kwargs):
        return None

    def legend(self, *args, **kwargs):
        return None

    def grid(self, *args, **kwargs):
        return None

    def text(self, *args, **kwargs):
        return None


class _FakeFigure:
    def suptitle(self, *args, **kwargs):
        return None

    def text(self, *args, **kwargs):
        return None

    def tight_layout(self):
        return None

    def savefig(self, output_path, *args, **kwargs):
        Path(output_path).write_bytes(b"fake-png")


class _FakePyplot:
    def subplots(self, *args, **kwargs):
        return _FakeFigure(), [_FakeAxis(), _FakeAxis(), _FakeAxis(), _FakeAxis()]

    def close(self, *args, **kwargs):
        return None


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


def test_save_error_plot_writes_png_to_output_directory(tmp_path):
    history = ErrorHistory(window_s=10.0)
    history.add(parse_debug_sample([0.0, 0.2, 0.1, 0.3, 4.0, 0.8, 0.5, 0.1, 0.5, 0.15]))
    history.add(parse_debug_sample([1.0, 0.1, 0.05, 0.15, 4.1, 0.9, 0.4, 0.08, 0.5, 0.12]))

    output_path = save_error_plot(history, tmp_path / "plots", pyplot=_FakePyplot())

    assert output_path == tmp_path / "plots" / "astlf_error_curves.png"
    assert output_path.read_bytes() == b"fake-png"


def test_save_error_plot_writes_png_with_one_sample(tmp_path):
    history = ErrorHistory(window_s=10.0)
    history.add(parse_debug_sample([0.0, 0.2, 0.1, 0.3, 4.0, 0.8, 0.5, 0.1, 0.5, 0.15]))

    output_path = save_error_plot(history, tmp_path / "plots", pyplot=_FakePyplot())

    assert output_path == tmp_path / "plots" / "astlf_error_curves.png"
    assert output_path.read_bytes() == b"fake-png"


def test_save_error_plot_writes_png_without_samples(tmp_path):
    output_path = save_error_plot(ErrorHistory(window_s=10.0), tmp_path / "plots", pyplot=_FakePyplot())

    assert output_path == tmp_path / "plots" / "astlf_error_curves.png"
    assert output_path.read_bytes() == b"fake-png"
