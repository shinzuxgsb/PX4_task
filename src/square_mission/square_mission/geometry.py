"""Pure geometry helpers for the clockwise square mission."""

from __future__ import annotations

import math
from typing import Iterable, Sequence


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def clockwise_square_offsets(side_cm: float) -> tuple[tuple[float, float], ...]:
    """Return (forward, left) targets for a clockwise square.

    The takeoff point is the lower-left corner when the initial heading points
    toward the top of the square. In body FLU coordinates (forward, left), the
    clockwise order is:
    upper-left, upper-right, lower-right, lower-left.
    """
    side = float(side_cm)
    if not math.isfinite(side) or side <= 0.0:
        raise ValueError("side_cm must be a positive finite number")

    return (
        (side, 0.0),
        (side, -side),
        (0.0, -side),
        (0.0, 0.0),
    )


def body_offset_to_world(
    origin_x_m: float,
    origin_y_m: float,
    origin_yaw_rad: float,
    forward_cm: float,
    left_cm: float,
) -> tuple[float, float]:
    """Convert a takeoff-relative (forward, left) offset to odom-world meters."""
    c = math.cos(origin_yaw_rad)
    s = math.sin(origin_yaw_rad)
    world_x = origin_x_m + (forward_cm * c - left_cm * s) / 100.0
    world_y = origin_y_m + (forward_cm * s + left_cm * c) / 100.0
    return world_x, world_y


def square_waypoints(
    origin_x_m: float,
    origin_y_m: float,
    origin_yaw_rad: float,
    side_cm: float,
) -> tuple[tuple[float, float], ...]:
    return tuple(
        body_offset_to_world(origin_x_m, origin_y_m, origin_yaw_rad, fwd, left)
        for fwd, left in clockwise_square_offsets(side_cm)
    )


def signed_polygon_area_cm2(points: Iterable[Sequence[float]]) -> float:
    """Shoelace area; negative means clockwise in an x-right/y-up frame."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) < 3:
        return 0.0

    twice_area = 0.0
    for current, nxt in zip(pts, pts[1:] + [pts[0]]):
        twice_area += current[0] * nxt[1] - current[1] * nxt[0]
    return twice_area / 2.0
