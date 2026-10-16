import math
from typing import Tuple


def normalize_angle_deg(angle_deg: float) -> float:
    """Normalize an angle to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def body_offset_to_map(
    origin_x_cm: float,
    origin_y_cm: float,
    yaw_deg: float,
    forward_cm: float,
    left_cm: float,
) -> Tuple[float, float]:
    """Convert a body-frame forward/left offset into map coordinates."""
    yaw_rad = math.radians(yaw_deg)
    x_cm = origin_x_cm + forward_cm * math.cos(yaw_rad) - left_cm * math.sin(yaw_rad)
    y_cm = origin_y_cm + forward_cm * math.sin(yaw_rad) + left_cm * math.cos(yaw_rad)
    return x_cm, y_cm


def target_error(
    current_x_cm: float,
    current_y_cm: float,
    current_z_cm: float,
    current_yaw_deg: float,
    target_x_cm: float,
    target_y_cm: float,
    target_z_cm: float,
    target_yaw_deg: float,
) -> Tuple[float, float, float]:
    """Return XY distance, absolute Z error and absolute yaw error."""
    distance_xy_cm = math.hypot(target_x_cm - current_x_cm, target_y_cm - current_y_cm)
    height_error_cm = abs(target_z_cm - current_z_cm)
    yaw_error_deg = abs(normalize_angle_deg(target_yaw_deg - current_yaw_deg))
    return distance_xy_cm, height_error_cm, yaw_error_deg
