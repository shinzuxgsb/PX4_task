"""ROS-independent bounded velocity estimate from continuous local lidar poses."""

import math
from collections import deque

from .frames import normalize_angle_rad


class LidarVelocityEstimator:
    def __init__(self, window_sec=0.25, min_span_sec=0.12,
                 max_speed_mps=1.5, max_gap_sec=0.5,
                 jump_m=0.75, jump_yaw_rad=math.pi / 4):
        if not 0 < min_span_sec <= window_sec < max_gap_sec:
            raise ValueError("Require 0 < min_span <= window < max_gap")
        if min(max_speed_mps, jump_m, jump_yaw_rad) <= 0:
            raise ValueError("Limits must be positive")
        self.window = window_sec
        self.min_span = min_span_sec
        self.max_speed = max_speed_mps
        self.max_gap = max_gap_sec
        self.jump_m = jump_m
        self.jump_yaw = jump_yaw_rad
        self.samples = deque(maxlen=128)
        self.reset_counter = 0
        self.state = "WARMUP"

    def invalidate(self):
        if self.samples:
            self.reset_counter = (self.reset_counter + 1) % 256
        self.samples.clear()
        self.state = "STALE"

    def update(self, stamp, x, y, yaw):
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            self.invalidate()
            self.state = "INVALID"
            return None
        if self.samples:
            t0, x0, y0, yaw0 = self.samples[-1]
            dt = stamp - t0
            if dt == 0:
                return None  # Never republish a duplicated measurement.
            distance = math.hypot(x - x0, y - y0)
            if (dt < 0 or dt > self.max_gap or distance > self.jump_m
                    or abs(normalize_angle_rad(yaw - yaw0)) > self.jump_yaw
                    or distance / dt > self.max_speed):
                self.invalidate()
                self.state = "RESET"
                self.samples.append((stamp, x, y, yaw))
                return None
        self.samples.append((stamp, x, y, yaw))
        while len(self.samples) > 2 and stamp - self.samples[0][0] > self.window:
            self.samples.popleft()
        if stamp - self.samples[0][0] < self.min_span:
            self.state = "WARMUP"
            return None
        # Windowed least-squares slope reduces scan-to-scan position jitter.
        ts = [s[0] - stamp for s in self.samples]
        mt = sum(ts) / len(ts)
        denom = sum((t - mt) ** 2 for t in ts)
        vx = sum((t - mt) * s[1] for t, s in zip(ts, self.samples)) / denom
        vy = sum((t - mt) * s[2] for t, s in zip(ts, self.samples)) / denom
        if math.hypot(vx, vy) > self.max_speed:
            self.invalidate()
            self.state = "RESET"
            return None
        self.state = "VALID"
        # Slope represents the centre of the window, not its latest endpoint.
        return vx, vy, stamp + mt


def frd_world_velocity_to_body(vx, vy, ros_yaw):
    """Rotate arbitrary world FRD velocity into body FRD (no north assumption)."""
    c, s = math.cos(ros_yaw), math.sin(ros_yaw)
    return c * vx - s * vy, s * vx + c * vy


def frd_world_velocity_to_ned(vx, vy, ros_yaw, px4_heading):
    """Align lidar's arbitrary horizontal world with PX4's current N/E frame.

    ros_yaw is counter-clockwise; PX4 heading is clockwise. This is a yaw-only
    world-frame alignment, not a 3-D body rotation with an invented body VZ.
    """
    theta = px4_heading + ros_yaw
    c, s = math.cos(theta), math.sin(theta)
    return c * vx - s * vy, s * vx + c * vy


def yaw_at_sample(samples, stamp):
    """Interpolate independent lidar yaw at the velocity window centre."""
    if not samples or stamp < samples[0][0] or stamp > samples[-1][0]:
        return None
    for older, newer in zip(samples, list(samples)[1:]):
        if older[0] <= stamp <= newer[0] and newer[0] > older[0]:
            fraction = (stamp - older[0]) / (newer[0] - older[0])
            return normalize_angle_rad(
                older[3] + fraction * normalize_angle_rad(newer[3] - older[3]))
    return samples[-1][3] if stamp == samples[-1][0] else None


def pose_at_sample(samples, stamp):
    """Interpolate planar FRD-world pose at the velocity window centre."""
    if not samples or stamp < samples[0][0] or stamp > samples[-1][0]:
        return None
    if stamp == samples[-1][0]:
        return samples[-1][1], samples[-1][2], samples[-1][3]
    for older, newer in zip(samples, list(samples)[1:]):
        if older[0] <= stamp <= newer[0] and newer[0] > older[0]:
            fraction = (stamp - older[0]) / (newer[0] - older[0])
            return (
                older[1] + fraction * (newer[1] - older[1]),
                older[2] + fraction * (newer[2] - older[2]),
                normalize_angle_rad(
                    older[3] + fraction * normalize_angle_rad(newer[3] - older[3])),
            )
    return None
