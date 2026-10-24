"""Scalar-speed waypoint controller modelled on the reference onboard stack."""

from __future__ import annotations

import math


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def limit_xy_vector(x: float, y: float, limit: float) -> tuple[float, float]:
    """Clamp a 2-D vector by its magnitude instead of clamping each axis."""
    magnitude = math.hypot(x, y)
    if magnitude <= limit or magnitude <= 1e-9:
        return x, y
    scale = limit / magnitude
    return x * scale, y * scale


def touchdown_ready(
    relative_height_cm: float,
    vertical_rate_cm_s: float,
    max_relative_height_cm: float,
    max_vertical_rate_cm_s: float,
) -> bool:
    """Return true when ToF says the vehicle is stationary at the ground."""
    return (
        math.isfinite(relative_height_cm)
        and math.isfinite(vertical_rate_cm_s)
        and math.isfinite(max_relative_height_cm)
        and math.isfinite(max_vertical_rate_cm_s)
        and relative_height_cm <= max_relative_height_cm
        and abs(vertical_rate_cm_s) <= max_vertical_rate_cm_s
    )


class ScalarSpeedPid:
    """Reference-style distance PID producing one non-negative speed value.

    The waypoint direction is applied separately as a unit vector. This avoids
    the diagonal overspeed and axis-wise limit cycling of a per-axis PD loop.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        max_speed_cm_s: float,
        integral_limit_cm_s: float = 5.0,
        deadzone_cm: float = 0.6,
        derivative_filter_alpha: float = 0.8,
    ) -> None:
        for value in (
            kp,
            ki,
            kd,
            max_speed_cm_s,
            integral_limit_cm_s,
            deadzone_cm,
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("speed PID parameters must be finite and non-negative")
        if max_speed_cm_s <= 0.0:
            raise ValueError("max_speed_cm_s must be positive")
        if not 0.0 <= derivative_filter_alpha <= 1.0:
            raise ValueError("derivative_filter_alpha must be within [0, 1]")

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_speed_cm_s = max_speed_cm_s
        self.integral_limit_cm_s = integral_limit_cm_s
        self.deadzone_cm = deadzone_cm
        self.derivative_filter_alpha = derivative_filter_alpha
        self.reset()

    def reset(self) -> None:
        self.previous_error_cm = 0.0
        self.integral_cm_s = 0.0
        self.previous_derivative_cm_s = 0.0
        self.first_call = True

    def update(self, distance_cm: float, dt_s: float) -> float:
        if not math.isfinite(distance_cm):
            self.reset()
            return 0.0

        error_cm = max(0.0, distance_cm)
        if error_cm < self.deadzone_cm:
            error_cm = 0.0

        if self.first_call:
            self.previous_error_cm = error_cm
            self.first_call = False

        dt_s = clamp(dt_s, 0.0, 0.25)
        proportional_cm_s = self.kp * error_cm

        self.integral_cm_s = clamp(
            self.integral_cm_s + error_cm * dt_s,
            -self.integral_limit_cm_s,
            self.integral_limit_cm_s,
        )
        integral_term_cm_s = self.ki * self.integral_cm_s

        derivative_raw_cm_s = (
            (error_cm - self.previous_error_cm) / dt_s if dt_s > 0.0 else 0.0
        )
        derivative_filtered_cm_s = (
            self.derivative_filter_alpha * self.previous_derivative_cm_s
            + (1.0 - self.derivative_filter_alpha) * derivative_raw_cm_s
        )
        derivative_term_cm_s = self.kd * derivative_filtered_cm_s

        output_cm_s = proportional_cm_s + integral_term_cm_s + derivative_term_cm_s
        output_cm_s = clamp(output_cm_s, 0.0, self.max_speed_cm_s)

        if output_cm_s >= self.max_speed_cm_s and error_cm > 0.0:
            self.integral_cm_s -= error_cm * dt_s

        self.previous_error_cm = error_cm
        self.previous_derivative_cm_s = derivative_filtered_cm_s
        return output_cm_s


def target_velocity_world_cm_s(
    current_x_m: float,
    current_y_m: float,
    target_x_m: float,
    target_y_m: float,
    controller: ScalarSpeedPid,
    dt_s: float,
) -> tuple[float, float, float]:
    """Return (world_vx_cm_s, world_vy_cm_s, remaining_distance_cm)."""
    error_x_m = target_x_m - current_x_m
    error_y_m = target_y_m - current_y_m
    distance_cm = math.hypot(error_x_m, error_y_m) * 100.0
    if not math.isfinite(distance_cm) or distance_cm <= 0.1:
        return 0.0, 0.0, max(0.0, distance_cm if math.isfinite(distance_cm) else 0.0)

    speed_cm_s = controller.update(distance_cm, dt_s)
    direction_x = error_x_m * 100.0 / distance_cm
    direction_y = error_y_m * 100.0 / distance_cm
    vx_cm_s = speed_cm_s * direction_x
    vy_cm_s = speed_cm_s * direction_y
    vx_cm_s, vy_cm_s = limit_xy_vector(
        vx_cm_s, vy_cm_s, controller.max_speed_cm_s
    )
    return vx_cm_s, vy_cm_s, distance_cm
