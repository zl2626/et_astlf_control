import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    yaw: float


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def lateral_offset(vehicle_x: float, vehicle_y: float, reference: PathPoint) -> float:
    return -(vehicle_x - reference.x) * math.sin(reference.yaw) + (vehicle_y - reference.y) * math.cos(reference.yaw)


def heading_error(vehicle_yaw: float, reference_yaw: float) -> float:
    return normalize_angle(vehicle_yaw - reference_yaw)


class ReferencePath:
    def __init__(self, points: Sequence[PathPoint]) -> None:
        if not points:
            raise ValueError("reference path must contain at least one point")
        self.points = list(points)

    @classmethod
    def from_xy(cls, xy_points: Iterable[Tuple[float, float]]) -> "ReferencePath":
        xy = list(xy_points)
        if not xy:
            raise ValueError("reference path must contain at least one point")

        points: List[PathPoint] = []
        for index, (x, y) in enumerate(xy):
            if len(xy) == 1:
                yaw = 0.0
            elif index < len(xy) - 1:
                next_x, next_y = xy[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                prev_x, prev_y = xy[index - 1]
                yaw = math.atan2(y - prev_y, x - prev_x)
            points.append(PathPoint(x=x, y=y, yaw=yaw))
        return cls(points)

    def target_from_pose(self, vehicle_x: float, vehicle_y: float, lookahead_m: float = 0.0) -> PathPoint:
        nearest_index = self._nearest_index(vehicle_x, vehicle_y)
        if lookahead_m <= 0.0:
            return self.points[nearest_index]

        remaining = lookahead_m
        for index in range(nearest_index, len(self.points) - 1):
            start = self.points[index]
            end = self.points[index + 1]
            segment_length = math.hypot(end.x - start.x, end.y - start.y)
            if segment_length >= remaining:
                return end
            remaining -= segment_length
        return self.points[-1]

    def _nearest_index(self, vehicle_x: float, vehicle_y: float) -> int:
        best_index = 0
        best_distance = float("inf")
        for index, point in enumerate(self.points):
            distance = math.hypot(vehicle_x - point.x, vehicle_y - point.y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index
