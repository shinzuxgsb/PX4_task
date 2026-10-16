"""Coordinate and angle conversions used by the PX4 bridge.

ROS uses a forward-left-up body convention and counter-clockwise yaw. PX4
uses forward-right-down and NED/FRD coordinates. These functions are kept
independent of ROS so they can be unit tested offline.
"""

import math
from typing import Tuple


def normalize_angle_rad(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return ROS/FLU yaw from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def flu_quaternion_to_frd(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float, float]:
    """Convert body-to-world FLU quaternion to body-to-world FRD.

    PX4 VehicleOdometry orders quaternion components as [w, x, y, z].
    """
    return (w, x, -y, -z)


def flu_position_to_frd(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert a vector from FLU world axes to arbitrary world-fixed FRD."""
    return (x, -y, -z)


def map_vector_to_ned(
    map_x: float,
    map_y: float,
    ros_anchor_yaw: float,
    px4_anchor_heading: float,
) -> Tuple[float, float]:
    """Rotate a map-frame vector into the PX4 local NED horizontal frame.

    The two frames are aligned at an anchor pose. No geographic north is
    assumed for the Cartographer frame.
    """
    forward = math.cos(ros_anchor_yaw) * map_x + math.sin(ros_anchor_yaw) * map_y
    left = -math.sin(ros_anchor_yaw) * map_x + math.cos(ros_anchor_yaw) * map_y
    north = math.cos(px4_anchor_heading) * forward + math.sin(px4_anchor_heading) * left
    east = math.sin(px4_anchor_heading) * forward - math.cos(px4_anchor_heading) * left
    return north, east


def map_yaw_to_ned(
    map_yaw: float, ros_anchor_yaw: float, px4_anchor_heading: float
) -> float:
    """Convert a ROS counter-clockwise map yaw to PX4 NED yaw."""
    return normalize_angle_rad(px4_anchor_heading - (map_yaw - ros_anchor_yaw))
