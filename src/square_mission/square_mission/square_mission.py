#!/usr/bin/env python3
"""Arm, take off, fly one clockwise square, return, and land in PX4 offboard.

The node uses the existing mavlink_bridge topic contract:
  SUB /odom_from_slam  nav_msgs/Odometry
  SUB /height          std_msgs/Int16, cm, positive up
  SUB /tofsense/range  sensor_msgs/Range, m, downward AGL
  SUB /is_st_ready     std_msgs/UInt8, 1 means armed and external estimate valid
  PUB /target_velocity std_msgs/Float32MultiArray, body FLU cm/s and deg/s
  PUB /px4_command     std_msgs/String: position/arm/offboard/land/disarm

The takeoff point is the lower-left corner. The route is:
  takeoff -> forward -> right -> back -> left (clockwise) -> return descent
  -> ToF-confirmed touchdown -> direct disarm.
"""

from __future__ import annotations

import argparse
import collections
import math
import os
import re
import signal
import sys
import time
import traceback
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Int16, String, UInt8

from square_mission.geometry import (
    clamp,
    square_waypoints,
    wrap_angle,
    yaw_from_quaternion,
)
from square_mission.velocity_control import (
    ScalarSpeedPid,
    limit_xy_vector,
    target_velocity_world_cm_s,
    touchdown_ready,
)


FLIGHT_STATES = {
    "WAIT_OFFBOARD",
    "TAKEOFF",
    "ROUTE",
    "RETURN_HOVER",
    "DESCEND",
    "FINAL_DESCENT",
    "WAIT_DISARM",
    "LAND",
}

CONTROL_STATES = {
    "TAKEOFF",
    "ROUTE",
    "RETURN_HOVER",
    "DESCEND",
    "FINAL_DESCENT",
}


class SquareMission(Node):
    def __init__(
        self,
        flight_enabled: bool,
        wait_for_go: bool,
        heartbeat_path: str,
    ) -> None:
        super().__init__("square_mission")
        self.flight_enabled = bool(flight_enabled)
        self.wait_for_go = bool(wait_for_go)
        self.heartbeat_path = heartbeat_path
        self.state = "WAIT_TOPICS"
        self.done = False
        self.exit_code = 0
        self.failure_reason = ""
        self.failsafe_requested = ""
        self.failsafe_code = 0
        self.program_abort_requested = False
        self.t0 = time.monotonic()
        self.state_since = self.t0
        self.stable_since = 0.0
        self.last_tick = 0.0
        self.last_log = 0.0
        self.heartbeat_written_at = 0.0
        self.last_preflight_warning = ""
        self.last_preflight_warning_at = 0.0

        self.odom = None
        self.odom_stamp = 0.0
        self.odom_hist = collections.deque(maxlen=120)
        self.good_position = None
        self.good_yaw = 0.0
        self.odom_rejects = 0

        self.height_cm = None
        self.height_stamp = 0.0
        self.height_hist = collections.deque(maxlen=40)
        self.height_zero_cm = 0.0

        self.tof_cm = None
        self.tof_stamp = 0.0
        self.tof_ground_since = 0.0
        self.touchdown_since = 0.0
        self.tof_hist = collections.deque(maxlen=80)
        self.tof_zero_cm = None

        self.ready = 0
        self.ready_stamp = 0.0
        self.ready_seen = False
        self.bridge_status_stamp = 0.0
        self.bridge_link_ok = False
        self.px4_mode = ""
        self.px4_armed = False
        self.px4_ekf_ok = False
        self.battery_pct = None

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.waypoints = []
        self.waypoint_index = 0
        self.arrival_since = 0.0
        self.leg_since = 0.0
        self.hover_until = 0.0
        self.altitude_settled_since = 0.0
        self.land_sent_at = 0.0
        self.disarm_ack_received = False
        self.disarm_rejected = False
        self.return_target_since = 0.0
        self.last_landing_status_at = 0.0
        self.descent_hold_xy = None

        self.i_x_cm_s = 0.0
        self.i_y_cm_s = 0.0
        self.height_i = 0.0
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.cmd_stamp = 0.0

        self._declare_parameters()
        self.side_cm = float(self.get_parameter("side_cm").value)
        self.height_cm_target = float(self.get_parameter("height_cm").value)
        self.vertical_source = str(self.get_parameter("vertical_source").value)
        self.max_px4_ascent_cm = float(
            self.get_parameter("max_px4_ascent_cm").value
        )
        self.hover_s = float(self.get_parameter("hover_s").value)
        self.wp_speed = float(self.get_parameter("waypoint_speed_cm_s").value)
        self.wp_tolerance = float(self.get_parameter("waypoint_tolerance_cm").value)
        self.wp_settle_s = float(self.get_parameter("waypoint_settle_s").value)
        self.wp_timeout_s = float(self.get_parameter("waypoint_timeout_s").value)
        self.vz_max = float(self.get_parameter("vertical_speed_max_cm_s").value)
        self.descend_rate = float(self.get_parameter("descent_rate_cm_s").value)
        self.land_wait_s = float(self.get_parameter("land_wait_s").value)
        self.takeoff_horizontal_speed_cm_s = float(
            self.get_parameter("takeoff_horizontal_speed_cm_s").value
        )
        self.descent_horizontal_speed_cm_s = float(
            self.get_parameter("descent_horizontal_speed_cm_s").value
        )
        self.landing_align_timeout_s = float(
            self.get_parameter("landing_align_timeout_s").value
        )
        self.descent_timeout_s = float(
            self.get_parameter("descent_timeout_s").value
        )
        self.tof_timeout_s = float(self.get_parameter("tof_timeout_s").value)
        self.tof_landing_cm = float(self.get_parameter("tof_landing_cm").value)
        self.tof_landing_hold_s = float(self.get_parameter("tof_landing_hold_s").value)
        self.tof_touchdown_relative_cm = float(
            self.get_parameter("tof_touchdown_relative_cm").value
        )
        self.tof_touchdown_hold_s = float(
            self.get_parameter("tof_touchdown_hold_s").value
        )
        self.tof_touchdown_rate_cm_s = float(
            self.get_parameter("tof_touchdown_rate_cm_s").value
        )
        self.touchdown_final_rate_cm_s = float(
            self.get_parameter("touchdown_final_rate_cm_s").value
        )
        self.touchdown_final_s = float(
            self.get_parameter("touchdown_final_s").value
        )
        self.disarm_ack_timeout_s = float(
            self.get_parameter("disarm_ack_timeout_s").value
        )
        self.arm_timeout_s = float(self.get_parameter("arm_timeout_s").value)
        self.ready_timeout_s = float(self.get_parameter("ready_timeout_s").value)
        self.preflight_timeout_s = float(self.get_parameter("preflight_timeout_s").value)
        self.max_duration_s = float(self.get_parameter("max_duration_s").value)
        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self.height_timeout_s = float(self.get_parameter("height_timeout_s").value)
        self.ready_timeout_live_s = float(self.get_parameter("ready_timeout_live_s").value)
        self.pre_arm_stable_s = float(self.get_parameter("pre_arm_stable_s").value)
        self.pre_arm_stable_cm = float(self.get_parameter("pre_arm_stable_cm").value)
        self.pre_arm_stable_deg = float(self.get_parameter("pre_arm_stable_deg").value)
        self.odom_jump_cm = float(self.get_parameter("odom_jump_cm").value)
        self.yaw_jump_deg = float(self.get_parameter("yaw_jump_deg").value)
        self.odom_reject_limit = int(self.get_parameter("odom_reject_limit").value)
        self.xy_kp = float(self.get_parameter("xy_kp").value)
        self.xy_kd = float(self.get_parameter("xy_kd").value)
        self.xy_ki = float(self.get_parameter("xy_ki").value)
        self.xy_i_limit = float(self.get_parameter("xy_i_limit_cm_s").value)
        self.route_kp = float(self.get_parameter("route_kp").value)
        self.route_kd = float(self.get_parameter("route_kd").value)
        self.route_ki = float(self.get_parameter("route_ki").value)
        self.xy_deadband_cm = float(self.get_parameter("xy_deadband_cm").value)
        self.speed_derivative_filter_alpha = float(
            self.get_parameter("speed_derivative_filter_alpha").value
        )
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.accel_max = float(self.get_parameter("horizontal_accel_max_cm_s2").value)
        self.yaw_kp = float(self.get_parameter("yaw_kp").value)
        self.yaw_rate_max = float(self.get_parameter("yaw_rate_max_deg_s").value)
        self.height_kp = float(self.get_parameter("height_kp").value)
        self.height_kd = float(self.get_parameter("height_kd").value)
        self.height_ki = float(self.get_parameter("height_ki").value)
        self.height_i_limit = float(self.get_parameter("height_i_limit_cm_s").value)
        self.altitude_tolerance = float(self.get_parameter("altitude_tolerance_cm").value)
        self.altitude_settle_s = float(self.get_parameter("altitude_settle_s").value)
        self.command_delay_s = float(self.get_parameter("command_delay_s").value)
        self.offboard_warmup_s = float(self.get_parameter("offboard_warmup_s").value)
        self.abort_file = str(self.get_parameter("abort_file").value)
        self.go_file = str(self.get_parameter("go_file").value)
        self.ready_file = str(self.get_parameter("ready_file").value)
        self.min_battery_pct = int(self.get_parameter("minimum_battery_pct").value)
        self.min_odom_rate_hz = float(self.get_parameter("minimum_odom_rate_hz").value)
        self.min_height_rate_hz = float(
            self.get_parameter("minimum_height_rate_hz").value
        )
        self.offboard_confirm_timeout_s = float(
            self.get_parameter("offboard_confirm_timeout_s").value
        )
        self.go_wait_timeout_s = float(self.get_parameter("go_wait_timeout_s").value)
        self.speed_pid = ScalarSpeedPid(
            kp=self.route_kp,
            ki=self.route_ki,
            kd=self.route_kd,
            max_speed_cm_s=self.wp_speed,
            deadzone_cm=self.xy_deadband_cm,
            derivative_filter_alpha=self.speed_derivative_filter_alpha,
        )

        self._validate_parameters()

        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.velocity_pub = self.create_publisher(
            Float32MultiArray, "/target_velocity", 10
        )
        self.command_pub = self.create_publisher(String, "/px4_command", 10)
        self.create_subscription(Odometry, "/odom_from_slam", self._on_odom, 10)
        self.create_subscription(Int16, "/height", self._on_height, 10)
        self.create_subscription(
            Range, str(self.get_parameter("tof_topic").value), self._on_tof, 10
        )
        self.create_subscription(UInt8, "/is_st_ready", self._on_ready, latched)
        self.create_subscription(Log, "/rosout", self._on_rosout, 20)
        self.create_timer(1.0 / self.control_rate_hz, self._tick)

        mode = "FLIGHT" if self.flight_enabled else "DRY-RUN"
        if self.wait_for_go:
            mode = "PREFLIGHT-WAIT-GO"
        self.log(
            f"{mode}: side={self.side_cm:.0f}cm height={self.height_cm_target:.0f}cm "
            f"vertical_source={self.vertical_source} "
            "route=lower-left -> forward -> right -> back -> left -> land"
        )
        if not self.flight_enabled:
            self.log("未指定 --arm：不会解锁或起飞，只检查话题、定位和就绪状态")
        self._remove_if_exists(self.go_file)
        self._remove_if_exists(self.ready_file)

    def _declare_parameters(self) -> None:
        defaults = {
            "side_cm": 120.0,
            "height_cm": 100.0,
            "vertical_source": "tof",
            "max_px4_ascent_cm": 160.0,
            "hover_s": 3.0,
            "waypoint_speed_cm_s": 24.0,
            "waypoint_tolerance_cm": 8.0,
            "waypoint_settle_s": 0.8,
            "waypoint_timeout_s": 30.0,
            "vertical_speed_max_cm_s": 20.0,
            "descent_rate_cm_s": 25.0,
            "land_wait_s": 30.0,
            "takeoff_horizontal_speed_cm_s": 5.0,
            "descent_horizontal_speed_cm_s": 6.0,
            "landing_align_timeout_s": 15.0,
            "descent_timeout_s": 15.0,
            "tof_topic": "/tofsense/range",
            "tof_timeout_s": 0.30,
            "tof_landing_cm": 8.0,
            "tof_landing_hold_s": 0.50,
            "tof_touchdown_relative_cm": 0.0,
            "tof_touchdown_hold_s": 0.50,
            "tof_touchdown_rate_cm_s": 3.0,
            "touchdown_final_rate_cm_s": 5.0,
            "touchdown_final_s": 1.0,
            "disarm_ack_timeout_s": 1.0,
            "arm_timeout_s": 20.0,
            "ready_timeout_s": 90.0,
            "preflight_timeout_s": 60.0,
            "max_duration_s": 180.0,
            "odom_timeout_s": 0.6,
            "height_timeout_s": 0.6,
            "ready_timeout_live_s": 2.5,
            "pre_arm_stable_s": 1.5,
            "pre_arm_stable_cm": 5.0,
            "pre_arm_stable_deg": 5.0,
            "odom_jump_cm": 15.0,
            "yaw_jump_deg": 10.0,
            "odom_reject_limit": 8,
            "xy_kp": 1.0,
            "xy_kd": 1.2,
            "xy_ki": 0.25,
            "xy_i_limit_cm_s": 15.0,
            # Reference-style scalar-speed PID.  kd acts on distance rate,
            # not directly on delayed SLAM twist feedback.
            "route_kp": 0.8,
            "route_ki": 0.0,
            "route_kd": 0.2,
            "xy_deadband_cm": 0.6,
            "speed_derivative_filter_alpha": 0.8,
            "control_rate_hz": 50.0,
            "horizontal_accel_max_cm_s2": 25.0,
            "yaw_kp": 1.5,
            "yaw_rate_max_deg_s": 6.0,
            "height_kp": 0.6,
            "height_kd": 0.9,
            "height_ki": 0.10,
            "height_i_limit_cm_s": 8.0,
            "altitude_tolerance_cm": 6.0,
            "altitude_settle_s": 0.8,
            "command_delay_s": 0.8,
            "offboard_warmup_s": 1.2,
            "abort_file": "/tmp/square_abort",
            "go_file": "/tmp/square_takeoff_go",
            "ready_file": "/tmp/square_preflight_ready",
            "minimum_battery_pct": 30,
            "minimum_odom_rate_hz": 15.0,
            "minimum_height_rate_hz": 5.0,
            "offboard_confirm_timeout_s": 120.0,
            "go_wait_timeout_s": 300.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _validate_parameters(self) -> None:
        if self.vertical_source not in ("tof", "px4"):
            raise ValueError("vertical_source must be 'tof' or 'px4'")
        if not 20.0 <= self.side_cm <= 500.0:
            raise ValueError("side_cm must be within 20..500 cm")
        if not 30.0 <= self.height_cm_target <= 300.0:
            raise ValueError("height_cm must be within 30..300 cm")
        if not 5.0 <= self.wp_speed <= 50.0:
            raise ValueError("waypoint_speed_cm_s must be within 5..50 cm/s")
        if self.wp_tolerance <= 0.0 or self.wp_tolerance > self.side_cm / 2.0:
            raise ValueError("waypoint_tolerance_cm is invalid")
        for name, value in {
            "takeoff_horizontal_speed_cm_s": self.takeoff_horizontal_speed_cm_s,
            "descent_horizontal_speed_cm_s": self.descent_horizontal_speed_cm_s,
            "landing_align_timeout_s": self.landing_align_timeout_s,
            "descent_timeout_s": self.descent_timeout_s,
            "tof_touchdown_hold_s": self.tof_touchdown_hold_s,
            "tof_touchdown_rate_cm_s": self.tof_touchdown_rate_cm_s,
            "touchdown_final_rate_cm_s": self.touchdown_final_rate_cm_s,
            "touchdown_final_s": self.touchdown_final_s,
            "disarm_ack_timeout_s": self.disarm_ack_timeout_s,
        }.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if (
            not math.isfinite(self.tof_touchdown_relative_cm)
            or not -10.0 <= self.tof_touchdown_relative_cm <= 5.0
        ):
            raise ValueError("tof_touchdown_relative_cm must be within -10..5 cm")
        if self.odom_reject_limit < 1:
            raise ValueError("odom_reject_limit must be positive")

    # ------------------------------------------------------------------ ROS IO
    def _on_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.good_position is not None:
            jump_cm = math.hypot(x - self.good_position[0], y - self.good_position[1]) * 100.0
            yaw_jump_deg = abs(math.degrees(wrap_angle(yaw - self.good_yaw)))
            if jump_cm > self.odom_jump_cm or yaw_jump_deg > self.yaw_jump_deg:
                self.odom_rejects += 1
                self.log(
                    f"reject odometry jump: {jump_cm:.0f}cm / {yaw_jump_deg:.0f}deg "
                    f"(consecutive={self.odom_rejects})",
                    throttle_s=1.0,
                )
                if self.odom_rejects >= self.odom_reject_limit:
                    self.failsafe_requested = "external position estimate is unstable"
                    self.failsafe_code = 7
                return

        self.odom_rejects = 0
        self.good_position = (x, y)
        self.good_yaw = yaw
        self.odom = msg
        self.odom_stamp = time.monotonic()
        self.odom_hist.append((self.odom_stamp, x, y, yaw))

    def _on_height(self, msg: Int16) -> None:
        self.height_cm = float(msg.data)
        self.height_stamp = time.monotonic()
        self.height_hist.append((self.height_stamp, self.height_cm))

    def _on_tof(self, msg: Range) -> None:
        value_cm = float(msg.range) * 100.0
        if not math.isfinite(value_cm) or value_cm <= 0.0:
            return
        self.tof_cm = value_cm
        self.tof_stamp = time.monotonic()
        self.tof_hist.append((self.tof_stamp, self.tof_cm))

    def _on_ready(self, msg: UInt8) -> None:
        self.ready = int(msg.data)
        self.ready_stamp = time.monotonic()
        self.ready_seen = True

    def _on_rosout(self, msg: Log) -> None:
        """Read the bridge's one-line status as a pre-arm EKF/mode/health source."""
        if msg.name != "mavlink_bridge":
            return
        text = str(msg.msg)
        if self.state == "WAIT_DISARM" and "[PX4 ACK] arm/disarm" in text:
            self.disarm_ack_received = True
            self.disarm_rejected = "ACCEPTED" not in text
            return
        mode_match = re.search(r"PX4模式=([A-Z_]+)", text)
        armed_match = re.search(r"armed=(True|False)", text)
        ekf_match = re.search(r"ekf_ok=(True|False)", text)
        battery_match = re.search(r"/\s*(\d+)%", text)
        if not mode_match or not armed_match or not ekf_match:
            return
        self.px4_mode = mode_match.group(1)
        self.px4_armed = armed_match.group(1) == "True"
        self.px4_ekf_ok = ekf_match.group(1) == "True"
        self.bridge_link_ok = "链路=OK" in text
        self.battery_pct = int(battery_match.group(1)) if battery_match else None
        self.bridge_status_stamp = time.monotonic()

    def log(self, text: str, throttle_s: float = 0.0) -> None:
        if throttle_s > 0.0:
            now = time.monotonic()
            if now - self.last_log < throttle_s:
                return
            self.last_log = now
        self.get_logger().info(text)

    def set_state(self, state: str) -> None:
        self.state = state
        self.state_since = time.monotonic()
        self.log(f"PHASE={state}")

    def send_command(self, command: str) -> None:
        self.command_pub.publish(String(data=command))
        self.log(f"PX4 command: {command}")

    def send_velocity(self, vx: float, vy: float, vz: float, vyaw: float = 0.0) -> None:
        now = time.monotonic()
        dt = clamp(now - self.cmd_stamp, 0.005, 0.2) if self.cmd_stamp else 0.05
        self.cmd_stamp = now
        if self.accel_max > 0.0:
            delta_vx = vx - self.last_vx
            delta_vy = vy - self.last_vy
            delta_v = math.hypot(delta_vx, delta_vy)
            max_delta_v = self.accel_max * dt
            if delta_v > max_delta_v and delta_v > 1e-9:
                scale = max_delta_v / delta_v
                vx = self.last_vx + delta_vx * scale
                vy = self.last_vy + delta_vy * scale
        self.last_vx = vx
        self.last_vy = vy
        self.velocity_pub.publish(
            Float32MultiArray(data=[float(vx), float(vy), float(vz), float(vyaw)])
        )

    # ------------------------------------------------------------------ Health
    def odom_fresh(self) -> bool:
        return (
            self.odom is not None
            and time.monotonic() - self.odom_stamp <= self.odom_timeout_s
        )

    def height_fresh(self) -> bool:
        return (
            self.height_cm is not None
            and time.monotonic() - self.height_stamp <= self.height_timeout_s
        )

    def tof_fresh(self) -> bool:
        return (
            self.tof_cm is not None
            and self.tof_stamp > 0.0
            and time.monotonic() - self.tof_stamp <= self.tof_timeout_s
        )

    def ready_fresh(self) -> bool:
        return (
            self.ready_seen
            and time.monotonic() - self.ready_stamp <= self.ready_timeout_live_s
        )

    def tof_ground_reading(self) -> bool:
        """Return true for a fresh downward ToF reading at ground height."""
        now = time.monotonic()
        fresh = self.tof_stamp > 0.0 and now - self.tof_stamp <= self.tof_timeout_s
        return (
            fresh
            and self.tof_cm is not None
            and self.tof_cm <= self.tof_landing_cm
        )

    def tof_landing_detected(self) -> bool:
        """Confirm touchdown after ToF has remained near the ground."""
        now = time.monotonic()
        if not self.tof_ground_reading():
            self.tof_ground_since = 0.0
            return False
        if self.tof_ground_since == 0.0:
            self.tof_ground_since = now
        return now - self.tof_ground_since >= self.tof_landing_hold_s

    def tof_touchdown_detected(self) -> bool:
        """Confirm touchdown using height relative to the takeoff ground."""
        now = time.monotonic()
        if (
            not self.tof_fresh()
            or self.tof_zero_cm is None
            or not touchdown_ready(
                self.height_relative_cm(),
                self.height_rate(),
                self.tof_touchdown_relative_cm,
                self.tof_touchdown_rate_cm_s,
            )
        ):
            self.touchdown_since = 0.0
            return False
        if self.touchdown_since == 0.0:
            self.touchdown_since = now
        return now - self.touchdown_since >= self.tof_touchdown_hold_s

    def return_target_status(self) -> tuple[float, float] | None:
        if (
            self.odom is None
            or not self.odom_fresh()
            or self.start_x is None
            or self.start_y is None
        ):
            return None
        p = self.odom.pose.pose.position
        distance_cm = math.hypot(
            float(p.x) - self.start_x,
            float(p.y) - self.start_y,
        ) * 100.0
        speed = self.recent_xy_speed_cm_s()
        return distance_cm, speed

    def return_target_ready(self) -> bool:
        """Use the normal waypoint tolerance and settle time for the return."""
        status = self.return_target_status()
        if status is None:
            self.return_target_since = 0.0
            return False
        distance_cm, _speed_cm_s = status
        now = time.monotonic()
        if distance_cm > self.wp_tolerance:
            self.return_target_since = 0.0
            return False
        if self.return_target_since == 0.0:
            self.return_target_since = now
        return now - self.return_target_since >= self.wp_settle_s

    def odom_stable(self) -> tuple[bool, float, float, int]:
        now = time.monotonic()
        window = [item for item in self.odom_hist if now - item[0] <= self.pre_arm_stable_s]
        if len(window) < 5:
            return False, 0.0, 0.0, len(window)
        _, x0, y0, yaw0 = window[0]
        drift_cm = max(
            math.hypot(x - x0, y - y0) * 100.0 for _, x, y, _ in window
        )
        yaw_span_deg = max(
            abs(math.degrees(wrap_angle(yaw - yaw0))) for _, _, _, yaw in window
        )
        stable = drift_cm <= self.pre_arm_stable_cm and yaw_span_deg <= self.pre_arm_stable_deg
        return stable, drift_cm, yaw_span_deg, len(window)

    def height_relative_cm(self) -> float:
        if self.vertical_source == "tof":
            if self.tof_cm is not None and self.tof_zero_cm is not None:
                return self.tof_cm - self.tof_zero_cm
            return 0.0
        return (
            self.height_cm - self.height_zero_cm
            if self.height_cm is not None
            else 0.0
        )

    def height_rate(self) -> float:
        history = self.tof_hist if self.vertical_source == "tof" else self.height_hist
        if len(history) < 3:
            return 0.0
        now = time.monotonic()
        old_stamp, old_height = history[0]
        for stamp, height in history:
            if now - stamp <= 0.4:
                old_stamp, old_height = stamp, height
                break
        dt = now - old_stamp
        if dt <= 0.05:
            return 0.0
        return (history[-1][1] - old_height) / dt

    def px4_relative_cm(self) -> float | None:
        if self.height_cm is None or self.height_zero_cm is None:
            return None
        return self.height_cm - self.height_zero_cm

    @staticmethod
    def _sample_rate(samples, window_s: float) -> float:
        now = time.monotonic()
        stamps = [item[0] for item in samples if now - item[0] <= window_s]
        if len(stamps) < 2:
            return 0.0
        duration = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / duration if duration > 0.0 else 0.0

    def odom_rate_hz(self) -> float:
        return self._sample_rate(self.odom_hist, 2.0)

    def height_rate_hz(self) -> float:
        return self._sample_rate(self.height_hist, 2.0)

    def bridge_status_fresh(self) -> bool:
        return (
            self.bridge_status_stamp > 0.0
            and time.monotonic() - self.bridge_status_stamp <= 3.5
        )

    def evaluate_offboard_preflight(self) -> tuple[bool, str]:
        if not self.bridge_status_fresh():
            return False, "mavlink_bridge status is unavailable"
        if not self.bridge_link_ok:
            return False, "MAVLink link is down"
        if self.px4_armed:
            return False, "PX4 is armed before the GO command"
        if not self.px4_ekf_ok:
            return False, "PX4 EKF horizontal/velocity checks are not valid"
        if not self.tof_ground_reading():
            return (
                False,
                f"downward ToF is not reporting ground "
                f"(reading={self.tof_cm if self.tof_cm is not None else '--'}cm)",
            )
        if self.px4_mode != "OFFBOARD":
            return False, f"PX4 mode is {self.px4_mode or 'unknown'}, expected OFFBOARD"
        if self.battery_pct is None:
            return False, "battery percentage is unavailable"
        if self.battery_pct < self.min_battery_pct:
            return (
                False,
                f"battery {self.battery_pct}% is below {self.min_battery_pct}%",
            )
        odom_hz = self.odom_rate_hz()
        if odom_hz < self.min_odom_rate_hz:
            return False, f"odometry rate {odom_hz:.1f}Hz is too low"
        height_hz = self.height_rate_hz()
        if height_hz < self.min_height_rate_hz:
            return False, f"height rate {height_hz:.1f}Hz is too low"
        stable, drift_cm, yaw_deg, samples = self.odom_stable()
        if not stable or samples < 5:
            return (
                False,
                f"position not stable: drift={drift_cm:.1f}cm "
                f"yaw={yaw_deg:.1f}deg samples={samples}",
            )
        return (
            True,
            f"offboard={self.px4_mode} ekf_ok={self.px4_ekf_ok} "
            f"armed={self.px4_armed} battery={self.battery_pct}% "
            f"odom={odom_hz:.1f}Hz height={height_hz:.1f}Hz "
            f"tof={self.tof_cm:.1f}cm drift={drift_cm:.1f}cm yaw={yaw_deg:.1f}deg",
        )

    def write_preflight_ready(self, details: str) -> None:
        content = (
            "READY\n"
            f"time={time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"{details}\n"
        )
        Path(self.ready_file).write_text(content, encoding="utf-8")

    @staticmethod
    def _remove_if_exists(path: str) -> None:
        try:
            os.remove(path)
        except FileNotFoundError:
            pass

    def recent_xy_speed_cm_s(self, window_s: float = 0.8) -> float:
        """Estimate actual XY movement for return-alignment diagnostics."""
        now = time.monotonic()
        window = [sample for sample in self.odom_hist if now - sample[0] <= window_s]
        if len(window) < 5:
            return math.inf
        stamps = [sample[0] for sample in window]
        dt = stamps[-1] - stamps[0]
        if dt <= 0.1:
            return math.inf
        _, x0, y0, _ = window[0]
        _, x1, y1, _ = window[-1]
        return math.hypot(x1 - x0, y1 - y0) * 100.0 / dt

    def request_abort(self, reason: str, code: int = 2) -> None:
        if self.done:
            return
        if not self.failure_reason:
            self.failure_reason = reason
        self.exit_code = self.exit_code or code
        self.log(f"ABORT: {reason}")
        if self.state in FLIGHT_STATES and self.state != "LAND":
            self.state = "LAND"
            self.state_since = time.monotonic()
            self.land_sent_at = self.state_since
            self.send_command("land")
        elif self.state not in FLIGHT_STATES:
            self.finish(code)

    def check_health(self) -> None:
        if self.failsafe_requested:
            reason = self.failsafe_requested
            self.failsafe_requested = ""
            self.request_abort(reason, self.failsafe_code or 2)
            return

        if os.path.exists(self.abort_file) and self.state not in ("LAND", "FINISHED"):
            try:
                os.remove(self.abort_file)
            except OSError:
                pass
            self.request_abort(f"abort file requested: {self.abort_file}", 4)
            return

        if time.monotonic() - self.t0 > self.max_duration_s and self.state != "LAND":
            self.request_abort("mission duration limit reached", 8)
            return

        if self.state not in FLIGHT_STATES or self.state in ("LAND", "WAIT_DISARM"):
            return

        px4_relative = self.px4_relative_cm()
        if not self.odom_fresh():
            self.request_abort("external odometry is stale", 7)
        elif self.vertical_source == "tof" and not self.tof_fresh():
            self.request_abort("ToF altitude is stale", 7)
        elif self.vertical_source == "px4" and not self.height_fresh():
            self.request_abort("height estimate is stale", 7)
        elif not self.ready_fresh():
            self.request_abort("flight-controller readiness is stale", 7)
        elif (
            self.vertical_source == "tof"
            and px4_relative is not None
            and px4_relative > self.max_px4_ascent_cm
        ):
            self.request_abort(
                f"PX4 ascent safety limit exceeded "
                f"({px4_relative:.0f}cm > {self.max_px4_ascent_cm:.0f}cm)",
                8,
            )
        elif self.state == "DESCEND" and self.tof_touchdown_detected():
            self.log(
                f"touchdown candidate by ToF "
                f"({self.height_relative_cm():.1f}cm relative, "
                f"raw={self.tof_cm:.1f}cm); "
                f"final {self.touchdown_final_rate_cm_s:.0f}cm/s descent "
                f"for {self.touchdown_final_s:.1f}s"
            )
            self.begin_final_touchdown_descent()
        elif self.ready == 0:
            if self.tof_touchdown_detected() or self.tof_ground_reading():
                self.log(
                    f"PX4 disarmed while ToF reports ground "
                    f"({self.tof_cm:.1f}cm)"
                )
                self.finish(0)
            else:
                self.request_abort("PX4 is no longer armed or EKF is invalid", 2)

    # ------------------------------------------------------------------ Control
    def control_velocity(
        self,
        target_height_cm: float,
        target_xy: tuple[float, float],
        route_mode: bool,
        dt: float,
    ) -> tuple[float, float, float, float]:
        height_error = target_height_cm - self.height_relative_cm()
        if abs(height_error) <= 15.0:
            self.height_i = clamp(
                self.height_i + height_error * dt,
                -self.height_i_limit / max(self.height_ki, 1e-6),
                self.height_i_limit / max(self.height_ki, 1e-6),
            )
        else:
            self.height_i = 0.0
        vz = clamp(
            self.height_kp * height_error
            - self.height_kd * self.height_rate()
            + self.height_ki * self.height_i,
            -self.vz_max,
            self.vz_max,
        )

        p = self.odom.pose.pose.position
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        cmd_wx, cmd_wy, _distance_cm = target_velocity_world_cm_s(
            float(p.x),
            float(p.y),
            float(target_xy[0]),
            float(target_xy[1]),
            self.speed_pid,
            dt,
        )
        cmd_wx, cmd_wy = limit_xy_vector(cmd_wx, cmd_wy, self.wp_speed)
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * cmd_wx + s * cmd_wy
        vy = -s * cmd_wx + c * cmd_wy

        yaw_error_deg = math.degrees(wrap_angle(self.start_yaw - yaw))
        vyaw = clamp(
            self.yaw_kp * yaw_error_deg,
            -self.yaw_rate_max,
            self.yaw_rate_max,
        )
        return vx, vy, vz, vyaw

    # ------------------------------------------------------------------ Mission
    def arm_now(self) -> None:
        if self.ready != 0:
            self.request_abort("vehicle is already armed before mission start", 2)
            return
        self.send_command("arm")
        self.set_state("WAIT_ARM")

    def record_takeoff_origin(self) -> None:
        self.height_zero_cm = (
            float(self.height_cm) if self.height_cm is not None else None
        )
        self.tof_zero_cm = float(self.tof_cm)
        self.start_x = float(self.odom.pose.pose.position.x)
        self.start_y = float(self.odom.pose.pose.position.y)
        self.start_yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        self.waypoints = list(
            square_waypoints(
                self.start_x,
                self.start_y,
                self.start_yaw,
                self.side_cm,
            )
        )
        self.log(
            f"takeoff origin odom=({self.start_x:+.3f},{self.start_y:+.3f})m "
            f"yaw={math.degrees(self.start_yaw):+.1f}deg "
            f"height zero={self.height_zero_cm if self.height_zero_cm is not None else '--'}cm "
            f"tof zero={self.tof_zero_cm:.1f}cm source={self.vertical_source}"
        )

    def begin_takeoff(self) -> None:
        self.height_i = 0.0
        self.speed_pid.reset()
        self.altitude_settled_since = 0.0
        self.return_target_since = 0.0
        self.set_state("TAKEOFF")

    def begin_route(self) -> None:
        self.waypoint_index = 0
        self.arrival_since = 0.0
        self.leg_since = time.monotonic()
        self.speed_pid.reset()
        self.set_state("ROUTE")
        self.log(
            f"begin clockwise square: side={self.side_cm:.0f}cm, "
            f"waypoint 1/{len(self.waypoints)}"
        )

    def begin_descent(self) -> None:
        self.height_i = 0.0
        self.speed_pid.reset()
        self.return_target_since = 0.0
        self.descent_hold_xy = (self.start_x, self.start_y)
        self.touchdown_since = 0.0
        self.set_state("DESCEND")
        self.log(
            f"target-point descent started toward takeoff point "
            f"({self.descent_hold_xy[0]:+.3f},{self.descent_hold_xy[1]:+.3f})m; "
            f"descending at {self.descend_rate:.0f}cm/s, "
            "direct disarm after low-altitude touchdown confirmation"
        )

    def begin_auto_disarm(self) -> None:
        if self.state == "WAIT_DISARM":
            return
        self.send_velocity(0.0, 0.0, 0.0, 0.0)
        self.disarm_ack_received = False
        self.disarm_rejected = False
        self.set_state("WAIT_DISARM")
        self.send_command("disarm")

    def begin_final_touchdown_descent(self) -> None:
        self.height_i = 0.0
        self.speed_pid.reset()
        self.set_state("FINAL_DESCENT")

    def fallback_to_px4_land(self, reason: str) -> None:
        self.log(f"direct disarm unavailable ({reason}); handing final landing to PX4")
        self.set_state("LAND")
        self.land_sent_at = time.monotonic()
        self.send_command("land")

    def finish(self, code: int) -> None:
        if self.done:
            return
        self._remove_if_exists(self.go_file)
        self._remove_if_exists(self.ready_file)
        self.done = True
        self.exit_code = int(code)
        self.state = "FINISHED"
        if code == 0:
            if self.flight_enabled:
                self.log("mission finished: vehicle disarmed at landing point")
            else:
                self.log("dry-run finished: preflight checks passed; no arm or offboard was sent")
        else:
            self.log(
                f"mission stopped with code {code}: "
                f"{self.failure_reason or 'see log above'}"
            )

    def tick_state(self, dt: float) -> None:
        elapsed = time.monotonic() - self.state_since

        if self.state == "WAIT_TOPICS":
            self.send_velocity(0.0, 0.0, 0.0)
            if self.odom_fresh() and self.height_fresh() and self.ready_seen:
                if self.ready != 0:
                    self.request_abort("vehicle must be disarmed before mission start", 2)
                else:
                    self.set_state("WAIT_STABLE")
            elif time.monotonic() - self.t0 > self.preflight_timeout_s:
                self.request_abort("timed out waiting for odometry, height, or readiness", 3)

        elif self.state == "WAIT_STABLE":
            self.send_velocity(0.0, 0.0, 0.0)
            stable, drift_cm, yaw_deg, samples = self.odom_stable()
            now = time.monotonic()
            if stable:
                if self.stable_since == 0.0:
                    self.stable_since = now
                if now - self.stable_since >= self.command_delay_s:
                    self.log(
                        f"preflight stable: drift={drift_cm:.1f}cm "
                        f"yaw span={yaw_deg:.1f}deg samples={samples}"
                    )
                    if not self.flight_enabled:
                        self.finish(0)
                        return
                    if self.wait_for_go:
                        self.log(
                            "Preflight sensors are stable. Using the RC, select "
                            "OFFBOARD now; the aircraft remains disarmed."
                        )
                        self.set_state("PREFLIGHT_OFFBOARD")
                    else:
                        self.send_command("position")
                        self.set_state("ARM_MODE")
            else:
                self.stable_since = 0.0
            if time.monotonic() - self.t0 > self.preflight_timeout_s:
                self.request_abort("preflight position did not become stable", 3)

        elif self.state == "ARM_MODE":
            self.send_velocity(0.0, 0.0, 0.0)
            if elapsed >= self.command_delay_s:
                self.arm_now()

        elif self.state == "PREFLIGHT_OFFBOARD":
            self.send_velocity(0.0, 0.0, 0.0)
            if elapsed >= 0.5:
                ok, details = self.evaluate_offboard_preflight()
                if ok:
                    self.write_preflight_ready(details)
                    self.log(f"PREFLIGHT PASS: {details}")
                    self.log(
                        "READY FOR GO: run square_takeoff.sh; arming is automatic "
                        "after the GO trigger"
                    )
                    self.set_state("WAIT_GO")
                else:
                    now = time.monotonic()
                    if (
                        details != self.last_preflight_warning
                        or now - self.last_preflight_warning_at >= 2.0
                    ):
                        self.log(f"PREFLIGHT WAIT: {details}")
                        self.last_preflight_warning = details
                        self.last_preflight_warning_at = now
                    if elapsed >= self.offboard_confirm_timeout_s:
                        self.request_abort(f"offboard preflight failed: {details}", 3)

        elif self.state == "WAIT_GO":
            self.send_velocity(0.0, 0.0, 0.0)
            ok, details = self.evaluate_offboard_preflight()
            if not ok:
                self.request_abort(f"preflight degraded before GO: {details}", 3)
                return
            if os.path.exists(self.go_file):
                self._remove_if_exists(self.go_file)
                self._remove_if_exists(self.ready_file)
                self.log("GO received: automatically arming in OFFBOARD mode")
                self.send_command("arm")
                self.set_state("WAIT_ARM")
            elif elapsed > self.go_wait_timeout_s:
                self.request_abort(
                    "GO was not received before the preflight timeout; vehicle remains disarmed",
                    3,
                )

        elif self.state == "WAIT_ARM":
            self.send_velocity(0.0, 0.0, 0.0)
            if self.ready == 1:
                vertical_ready = (
                    self.tof_fresh()
                    if self.vertical_source == "tof"
                    else self.height_fresh()
                )
                if self.odom_fresh() and vertical_ready:
                    self.record_takeoff_origin()
                    self.send_command("offboard")
                    self.set_state("WAIT_OFFBOARD")
                else:
                    self.request_abort("sensor data lost immediately after arming", 7)
            elif elapsed > self.arm_timeout_s:
                self.request_abort(
                    "arming was rejected or did not become ready; inspect PX4 STATUSTEXT",
                    5,
                )
            elif time.monotonic() - self.t0 > self.ready_timeout_s:
                self.request_abort("timed out waiting for armed + EKF-ready", 3)

        elif self.state == "WAIT_OFFBOARD":
            self.send_velocity(0.0, 0.0, 0.0)
            if elapsed >= self.offboard_warmup_s:
                self.begin_takeoff()

        elif self.state == "TAKEOFF":
            relative_height = self.height_relative_cm()
            vx, vy, vz, vyaw = self.control_velocity(
                self.height_cm_target,
                (self.start_x, self.start_y),
                route_mode=False,
                dt=dt,
            )
            vx, vy = limit_xy_vector(
                vx, vy, self.takeoff_horizontal_speed_cm_s
            )
            altitude_error = self.height_cm_target - relative_height
            if altitude_error > self.altitude_tolerance:
                vz = max(vz, 8.0)
            elif altitude_error > 0.0:
                # Do not turn a small remaining climb into a downward command.
                vz = max(vz, 0.0)
            self.send_velocity(vx, vy, vz, vyaw)

            settled = (
                abs(altitude_error) <= self.altitude_tolerance
                and abs(self.height_rate()) <= 8.0
            )
            now = time.monotonic()
            if not settled:
                self.altitude_settled_since = 0.0
            elif self.altitude_settled_since == 0.0:
                self.altitude_settled_since = now
            if settled and now - self.altitude_settled_since >= self.altitude_settle_s:
                self.log(
                    f"takeoff altitude reached: {relative_height:.1f}cm "
                    f"(target {self.height_cm_target:.0f}cm), "
                    f"PX4 relative={self.px4_relative_cm():.1f}cm"
                )
                self.begin_route()
            elif elapsed > 35.0:
                self.request_abort("takeoff altitude timeout", 8)

        elif self.state == "ROUTE":
            if self.waypoint_index >= len(self.waypoints):
                self.hover_until = time.monotonic() + self.hover_s
                self.arrival_since = 0.0
                self.set_state("RETURN_HOVER")
                return

            target_x, target_y = self.waypoints[self.waypoint_index]
            p = self.odom.pose.pose.position
            distance_cm = math.hypot(target_x - p.x, target_y - p.y) * 100.0
            vx, vy, vz, vyaw = self.control_velocity(
                self.height_cm_target,
                (target_x, target_y),
                route_mode=True,
                dt=dt,
            )
            self.send_velocity(vx, vy, vz, vyaw)

            leg_elapsed = time.monotonic() - self.leg_since
            if leg_elapsed > self.wp_timeout_s:
                self.request_abort(
                    f"waypoint {self.waypoint_index + 1} timeout "
                    f"(remaining {distance_cm:.0f}cm)",
                    8,
                )
                return
            if distance_cm <= self.wp_tolerance:
                if self.arrival_since == 0.0:
                    self.arrival_since = time.monotonic()
                elif time.monotonic() - self.arrival_since >= self.wp_settle_s:
                    self.log(
                        f"reached waypoint {self.waypoint_index + 1}/{len(self.waypoints)} "
                        f"(error {distance_cm:.1f}cm)"
                    )
                    self.waypoint_index += 1
                    self.arrival_since = 0.0
                    self.leg_since = time.monotonic()
                    self.speed_pid.reset()
                    if self.waypoint_index < len(self.waypoints):
                        self.log(
                            f"heading to waypoint {self.waypoint_index + 1}/"
                            f"{len(self.waypoints)}"
                        )
            else:
                self.arrival_since = 0.0

        elif self.state == "RETURN_HOVER":
            vx, vy, vz, vyaw = self.control_velocity(
                self.height_cm_target,
                (self.start_x, self.start_y),
                route_mode=False,
                dt=dt,
            )
            self.send_velocity(vx, vy, vz, vyaw)
            target_ready = self.return_target_ready()
            now = time.monotonic()
            if now >= self.hover_until and target_ready:
                status = self.return_target_status()
                detail = (
                    f"XY error={status[0]:.1f}cm speed={status[1]:.1f}cm/s"
                    if status is not None
                    else "XY estimate unavailable"
                )
                self.log(f"high-altitude landing alignment ready: {detail}")
                self.begin_descent()
            elif elapsed > self.landing_align_timeout_s:
                self.request_abort(
                    "high-altitude landing alignment timeout", 8
                )
            elif now >= self.hover_until and now - self.last_landing_status_at >= 2.0:
                status = self.return_target_status()
                detail = (
                    f"XY error={status[0]:.1f}cm speed={status[1]:.1f}cm/s"
                    if status is not None
                    else "XY estimate unavailable"
                )
                self.log(f"WAIT landing alignment: {detail}")
                self.last_landing_status_at = now

        elif self.state == "DESCEND":
            relative_height = self.height_relative_cm()
            target_xy = (
                self.descent_hold_xy
                if self.descent_hold_xy is not None
                else (self.start_x, self.start_y)
            )
            vx, vy, _, vyaw = self.control_velocity(
                self.height_cm_target,
                target_xy,
                route_mode=False,
                dt=dt,
            )
            vx, vy = limit_xy_vector(
                vx, vy, self.descent_horizontal_speed_cm_s
            )
            self.send_velocity(vx, vy, -self.descend_rate, vyaw)
            if elapsed > self.descent_timeout_s:
                self.request_abort("controlled descent timeout", 8)

        elif self.state == "FINAL_DESCENT":
            target_xy = (
                self.descent_hold_xy
                if self.descent_hold_xy is not None
                else (self.start_x, self.start_y)
            )
            vx, vy, _, vyaw = self.control_velocity(
                self.height_cm_target,
                target_xy,
                route_mode=False,
                dt=dt,
            )
            vx, vy = limit_xy_vector(
                vx, vy, self.descent_horizontal_speed_cm_s
            )
            self.send_velocity(
                vx, vy, -self.touchdown_final_rate_cm_s, vyaw
            )
            if elapsed >= self.touchdown_final_s:
                self.begin_auto_disarm()

        elif self.state == "WAIT_DISARM":
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            if self.ready_seen and self.ready == 0:
                self.finish(0)
            elif self.disarm_rejected:
                self.fallback_to_px4_land("PX4 rejected first disarm")
            elif not self.disarm_ack_received and elapsed > self.disarm_ack_timeout_s:
                self.fallback_to_px4_land("PX4 did not confirm disarm")
            elif elapsed > self.disarm_ack_timeout_s + 1.0:
                self.fallback_to_px4_land(
                    "disarm was acknowledged but armed state did not update"
                )

        elif self.state == "LAND":
            self.send_velocity(0.0, 0.0, 0.0)
            if self.ready_seen and self.ready == 0:
                if self.failure_reason:
                    self.finish(self.exit_code or 2)
                else:
                    self.finish(0)
            elif elapsed > self.land_wait_s:
                self.failure_reason = self.failure_reason or "landing confirmation timeout"
                self.finish(self.exit_code or 8)

    def _tick(self) -> None:
        try:
            now = time.monotonic()
            dt = clamp(now - self.last_tick, 0.005, 0.2) if self.last_tick else 0.05
            self.last_tick = now
            if self.done:
                return
            if now - self.heartbeat_written_at >= 1.0:
                try:
                    Path(self.heartbeat_path).touch(exist_ok=True)
                except OSError:
                    pass
                self.heartbeat_written_at = now
            if self.program_abort_requested:
                self.program_abort_requested = False
                self.request_abort("signal received; requesting landing", 4)
            self.check_health()
            if self.done:
                return
            self.tick_state(dt)
        except Exception:  # noqa: BLE001
            self.get_logger().error(
                "unexpected mission exception; requesting PX4 landing\n"
                + traceback.format_exc()
            )
            self.request_abort("unexpected software exception", 6)


def parse_cli() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Clockwise PX4 offboard square with optional two-command GO gate"
    )
    parser.add_argument(
        "--arm",
        action="store_true",
        help=(
            "enable real flight: position -> arm -> offboard -> takeoff -> "
            "square -> target-point descent -> direct disarm"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="explicitly request the default non-flying topic/preflight check",
    )
    parser.add_argument(
        "--wait-go",
        action="store_true",
        help=(
            "switch to OFFBOARD and pass preflight, then wait for the GO file "
            "before automatically arming and taking off"
        ),
    )
    parser.add_argument(
        "--heartbeat-file",
        default="/tmp/square_mission_alive",
        help="touch this file while the mission node is alive",
    )
    args, ros_args = parser.parse_known_args()
    if args.arm and args.dry_run:
        parser.error("--arm and --dry-run cannot be used together")
    if args.wait_go and not args.arm:
        parser.error("--wait-go requires --arm")
    return args, ros_args


def main() -> int:
    args, ros_args = parse_cli()
    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = None
    try:
        node = SquareMission(args.arm, args.wait_go, args.heartbeat_file)

        def _on_signal(_sig, _frame) -> None:
            node.program_abort_requested = True

        signal.signal(signal.SIGINT, _on_signal)
        signal.signal(signal.SIGTERM, _on_signal)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        return node.exit_code
    finally:
        if node is not None:
            try:
                for _ in range(6):
                    node.send_velocity(0.0, 0.0, 0.0)
                    time.sleep(0.02)
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
