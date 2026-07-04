import csv
from pathlib import Path
from typing import List, Tuple, Union


PathLike = Union[str, Path]


def load_xy_csv(path: PathLike) -> List[Tuple[float, float]]:
    csv_path = Path(path)
    points: List[Tuple[float, float]] = []

    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        for line_number, row in enumerate(csv.reader(handle), start=1):
            if not row:
                continue

            first = row[0].strip()
            if not first or first.startswith("#"):
                continue

            if len(row) < 2:
                raise ValueError(f"{csv_path}: line {line_number} must contain at least x,y columns")

            second = row[1].strip()
            if first.lower() == "x" and second.lower() == "y":
                continue

            try:
                points.append((float(first), float(second)))
            except ValueError as exc:
                raise ValueError(f"{csv_path}: line {line_number} contains non-numeric x,y values") from exc

    if not points:
        raise ValueError(f"{csv_path}: no path points found")
    return points
