import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from et_astlf_control.csv_path import load_xy_csv  # noqa: E402


def test_load_xy_csv_accepts_header_comments_and_blank_lines(tmp_path):
    csv_file = tmp_path / "omega_path.csv"
    csv_file.write_text(
        "\n".join(
            [
                "# x,y path exported from planner",
                "x,y",
                "0.0,0.0",
                "",
                "1.5,0.25",
                "3.0,1.0",
            ]
        ),
        encoding="utf-8",
    )

    points = load_xy_csv(csv_file)

    assert points == [(0.0, 0.0), (1.5, 0.25), (3.0, 1.0)]


def test_load_xy_csv_rejects_rows_with_less_than_two_columns(tmp_path):
    csv_file = tmp_path / "bad_path.csv"
    csv_file.write_text("x,y\n0.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="line 2"):
        load_xy_csv(csv_file)


def test_load_xy_csv_rejects_empty_paths(tmp_path):
    csv_file = tmp_path / "empty_path.csv"
    csv_file.write_text("# no points\nx,y\n", encoding="utf-8")

    with pytest.raises(ValueError, match="no path points"):
        load_xy_csv(csv_file)
