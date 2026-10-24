"""Pure mission-health helpers that can be tested without ROS."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass


@dataclass(frozen=True)
class BridgeStatus:
    mode: str
    armed: bool
    ekf_ok: bool
    link_ok: bool
    battery_pct: int | None


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_bridge_status(text: str) -> BridgeStatus | None:
    mode = re.search(r"PX4模式=([A-Z_]+)", text)
    armed = re.search(r"armed=(True|False)", text)
    ekf = re.search(r"ekf_ok=(True|False)", text)
    battery = re.search(r"/(\d+)%", text)
    if mode is None or armed is None or ekf is None:
        return None
    return BridgeStatus(
        mode=mode.group(1),
        armed=armed.group(1) == "True",
        ekf_ok=ekf.group(1) == "True",
        link_ok="链路=OK" in text,
        battery_pct=int(battery.group(1)) if battery else None,
    )


def stability_stats(samples) -> tuple[float, float, int]:
    """Return maximum horizontal drift (cm), yaw span (deg), sample count."""
    if not samples:
        return math.inf, math.inf, 0
    first = samples[0]
    _stamp0, x0, y0, yaw0 = first
    drift_cm = 0.0
    yaw_span_deg = 0.0
    for _stamp, x, y, yaw in samples:
        drift_cm = max(drift_cm, math.hypot(x - x0, y - y0) * 100.0)
        yaw_span_deg = max(
            yaw_span_deg, abs(math.degrees(wrap_angle(yaw - yaw0)))
        )
    return drift_cm, yaw_span_deg, len(samples)


def sample_rate_hz(stamps) -> float:
    if len(stamps) < 2:
        return 0.0
    duration = float(stamps[-1]) - float(stamps[0])
    return (len(stamps) - 1) / duration if duration > 0.0 else 0.0
