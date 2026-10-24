#!/usr/bin/env python3
"""Bridge the D-theme shelf mission lifecycle to the PX4 MAVLink stack.

The original 2024 mission node still produces map-frame position targets. The
copied PID package rotates its output into body FLU before publishing
``/target_velocity``. This node owns only the PX4 lifecycle:

* validate localization, height, ToF, EKF and stable position;
* keep a zero Offboard stream while the vehicle is disarmed;
* automatically arm only after the operator's explicit GO file;
* release the shelf mission through ``/fc_takeoff_if``;
* request PX4 Land on completion, mission abort or failed health.

No arm or Offboard command is sent in dry-run mode.
"""

from __future__ import annotations

import collections
import json
import math
import os
import signal
import time
import traceback
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import (
    Bool,
    Empty,
    Float32MultiArray,
    Int16,
    Int32,
    String,
    UInt8,
)

from shelf_px4_bridge.control_logic import (
    BridgeStatus,
    parse_bridge_status,
    sample_rate_hz,
    stability_stats,
    yaw_from_quaternion,
)


class ShelfPx4Adapter(Node):
    def __init__(self) -> None:
        super().__init__("shelf_px4_adapter")
        self._declare_parameters()
        self.flight_enabled = bool(self.get_parameter("flight_enabled").value)
        self.wait_for_go = bool(self.get_parameter("wait_for_go").value)
        self.ready_file = str(self.get_parameter("ready_file").value)
        self.go_file = str(self.get_parameter("go_file").value)
        self.abort_file = str(self.get_parameter("abort_file").value)
        self.heartbeat_file = str(self.get_parameter("heartbeat_file").value)

        self.state = "WAIT_TOPICS"
        self.state_since = time.monotonic()
        self.t0 = self.state_since
        self.done = False
        self.exit_code = 0
        self.failure_reason = ""
        self.stop_requested = False
        self.failsafe_requested = ""
        self.last_log_s = 0.0
        self.last_heartbeat_s = 0.0
        self.last_zero_s = 0.0
        self.last_land_command_s = 0.0
        self.last_status_s = 0.0
        self.arm_sent_s = 0.0
        self.trigger_s = 0.0
        self.mission_inactive_since = 0.0
        self.bridge_stamp = 0.0
        self.bridge = BridgeStatus("", False, False, False, None)

        self.odom: Odometry | None = None
        self.odom_stamp = 0.0
        self.odom_hist: collections.deque = collections.deque(maxlen=240)
        self.good_position: tuple[float, float] | None = None
        self.good_yaw = 0.0
        self.odom_rejects = 0

        self.height_cm: float | None = None
        self.height_stamp = 0.0
        self.height_hist: collections.deque = collections.deque(maxlen=40)

        self.tof_cm: float | None = None
        self.tof_stamp = 0.0
        self.ready = 0
        self.ready_seen = False
        self.ready_stamp = 0.0
        self.mission_active = False
        self.mission_active_seen = False
        self.mission_complete = False
        self.go_requested = False
        self.gs_message = ""

        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.velocity_pub = self.create_publisher(
            Float32MultiArray, "/target_velocity", 10
        )
        self.command_pub = self.create_publisher(String, "/px4_command", 10)
        self.trigger_pub = self.create_publisher(Bool, "/fc_takeoff_if", latched)
        self.gs_status_pub = self.create_publisher(
            String, "/shelf_scan/gs_out", 50
        )
        self.takeoff_mode_pub = self.create_publisher(
            String, "/shelf_scan/takeoff_mode", 10
        )
        self.target_id_pub = self.create_publisher(
            Int32, "/shelf_scan/target_id", latched
        )
        self.create_subscription(Odometry, "/odom_from_slam", self._on_odom, 10)
        self.create_subscription(Int16, "/height", self._on_height, 10)
        self.create_subscription(
            Range, str(self.get_parameter("tof_topic").value), self._on_tof, 10
        )
        self.create_subscription(UInt8, "/is_st_ready", self._on_ready, latched)
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)
        self.create_subscription(
            Empty, "/mission_complete", self._on_mission_complete, 10
        )
        self.create_subscription(
            Bool, "/mission_active", self._on_mission_active, latched
        )
        self.create_subscription(
            String, "/shelf_scan/gs_in", self._on_gs_command, 50
        )

        self._publish_trigger(False)
        self._remove_if_exists(self.go_file)
        self._remove_if_exists(self.ready_file)
        self._remove_if_exists(self.heartbeat_file)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("control_rate_hz").value), self._tick
        )

        mode = "ARM-ARMED" if self.flight_enabled else "DRY-RUN"
        self._log(
            f"{mode}: waiting for PX4, SLAM, height and ToF. "
            "The aircraft will remain disarmed until the GO file appears."
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "flight_enabled": False,
            "wait_for_go": True,
            "control_rate_hz": 20.0,
            "tof_topic": "/tofsense/range",
            "odom_timeout_s": 0.60,
            "height_timeout_s": 0.60,
            "tof_timeout_s": 0.30,
            "ready_timeout_s": 2.5,
            "bridge_timeout_s": 3.5,
            "preflight_timeout_s": 120.0,
            "go_wait_timeout_s": 300.0,
            "stable_window_s": 1.5,
            "stable_max_drift_cm": 5.0,
            "stable_max_yaw_deg": 5.0,
            "command_delay_s": 0.8,
            "arm_timeout_s": 20.0,
            "trigger_delay_s": 0.5,
            "mission_start_timeout_s": 12.0,
            "mission_max_duration_s": 360.0,
            "mission_inactive_grace_s": 1.0,
            "land_wait_s": 45.0,
            "minimum_battery_pct": 25,
            "require_battery": False,
            "require_tof_ground": True,
            "minimum_odom_rate_hz": 10.0,
            "minimum_height_rate_hz": 5.0,
            "odom_jump_cm": 15.0,
            "yaw_jump_deg": 10.0,
            "odom_reject_limit": 8,
            "ready_file": "/tmp/shelf_preflight_ready",
            "go_file": "/tmp/shelf_takeoff_go",
            "abort_file": "/tmp/shelf_abort",
            "heartbeat_file": "/tmp/shelf_px4_adapter.alive",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self.height_timeout_s = float(self.get_parameter("height_timeout_s").value)
        self.tof_timeout_s = float(self.get_parameter("tof_timeout_s").value)
        self.ready_timeout_s = float(self.get_parameter("ready_timeout_s").value)
        self.bridge_timeout_s = float(self.get_parameter("bridge_timeout_s").value)
        self.preflight_timeout_s = float(
            self.get_parameter("preflight_timeout_s").value
        )
        self.go_wait_timeout_s = float(self.get_parameter("go_wait_timeout_s").value)
        self.stable_window_s = float(self.get_parameter("stable_window_s").value)
        self.stable_max_drift_cm = float(
            self.get_parameter("stable_max_drift_cm").value
        )
        self.stable_max_yaw_deg = float(
            self.get_parameter("stable_max_yaw_deg").value
        )
        self.command_delay_s = float(self.get_parameter("command_delay_s").value)
        self.arm_timeout_s = float(self.get_parameter("arm_timeout_s").value)
        self.trigger_delay_s = float(self.get_parameter("trigger_delay_s").value)
        self.mission_start_timeout_s = float(
            self.get_parameter("mission_start_timeout_s").value
        )
        self.mission_max_duration_s = float(
            self.get_parameter("mission_max_duration_s").value
        )
        self.mission_inactive_grace_s = float(
            self.get_parameter("mission_inactive_grace_s").value
        )
        self.land_wait_s = float(self.get_parameter("land_wait_s").value)
        self.minimum_battery_pct = int(
            self.get_parameter("minimum_battery_pct").value
        )
        self.require_battery = bool(self.get_parameter("require_battery").value)
        self.require_tof_ground = bool(
            self.get_parameter("require_tof_ground").value
        )
        self.minimum_odom_rate_hz = float(
            self.get_parameter("minimum_odom_rate_hz").value
        )
        self.minimum_height_rate_hz = float(
            self.get_parameter("minimum_height_rate_hz").value
        )
        self.odom_jump_cm = float(self.get_parameter("odom_jump_cm").value)
        self.yaw_jump_deg = float(self.get_parameter("yaw_jump_deg").value)
        self.odom_reject_limit = int(
            self.get_parameter("odom_reject_limit").value
        )

    # ------------------------------------------------------------------ ROS IO
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x, y = float(p.x), float(p.y)
        yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        if self.good_position is not None:
            jump_cm = math.hypot(x - self.good_position[0], y - self.good_position[1]) * 100.0
            jump_yaw = abs(
                math.degrees(
                    math.atan2(
                        math.sin(yaw - self.good_yaw),
                        math.cos(yaw - self.good_yaw),
                    )
                )
            )
            if jump_cm > self.odom_jump_cm or jump_yaw > self.yaw_jump_deg:
                self.odom_rejects += 1
                self._log(
                    f"reject SLAM jump {jump_cm:.1f}cm/{jump_yaw:.1f}deg "
                    f"({self.odom_rejects}/{self.odom_reject_limit})",
                    throttle_s=1.0,
                )
                if self.odom_rejects >= self.odom_reject_limit:
                    self._request_land("external position estimate is unstable")
                return
        self.odom_rejects = 0
        self.good_position = (x, y)
        self.good_yaw = yaw
        self.odom = msg
        self.odom_stamp = time.monotonic()
        self.odom_hist.append((self.odom_stamp, x, y, yaw))

    def _on_height(self, msg: Int16) -> None:
        value = float(msg.data)
        if not math.isfinite(value):
            return
        self.height_cm = value
        self.height_stamp = time.monotonic()
        self.height_hist.append((self.height_stamp, value))

    def _on_tof(self, msg: Range) -> None:
        value_cm = float(msg.range) * 100.0
        if not math.isfinite(value_cm) or value_cm <= 0.0:
            return
        self.tof_cm = value_cm
        self.tof_stamp = time.monotonic()

    def _on_ready(self, msg: UInt8) -> None:
        self.ready = int(msg.data)
        self.ready_stamp = time.monotonic()
        self.ready_seen = True

    def _on_mission_complete(self, _msg: Empty) -> None:
        self.mission_complete = True

    def _on_mission_active(self, msg: Bool) -> None:
        self.mission_active_seen = True
        self.mission_active = bool(msg.data)

    def _on_gs_command(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data))
        except (TypeError, ValueError):
            self.get_logger().warn("ignored malformed ground-station command")
            return
        if str(payload.get("type", "")).lower() != "cmd":
            return
        command = str(payload.get("cmd", "")).lower()
        if command in ("px4_start", "start_sweep", "takeoff_start"):
            if not self.flight_enabled:
                self.gs_message = "adapter is in dry-run mode"
                self._publish_gs_status()
                return
            if self.state != "WAIT_GO":
                self.gs_message = f"takeoff request rejected in state {self.state}"
                self._log(f"ground station: {self.gs_message}")
                self._publish_gs_status()
                return
            mode = str(payload.get("mode", "sweep")).lower()
            if command == "start_sweep":
                mode = "sweep"
            if mode not in ("sweep", "target"):
                self.gs_message = f"unsupported mission mode: {mode}"
                self._publish_gs_status()
                return
            target_no = payload.get("no", payload.get("target_no"))
            if mode == "target":
                try:
                    target_no = int(target_no)
                except (TypeError, ValueError):
                    self.gs_message = "target mode requires a cargo number"
                    self._publish_gs_status()
                    return
                self.target_id_pub.publish(Int32(data=target_no))
            self.takeoff_mode_pub.publish(String(data=mode))
            self.go_requested = True
            self.gs_message = f"takeoff accepted: mode={mode}"
            self._log(f"ground station: {self.gs_message}")
            self._publish_gs_status()
        elif command in ("land", "stop_and_land"):
            self.gs_message = "ground station requested landing"
            self._request_land(self.gs_message)
        elif command in ("estop", "emergency_land"):
            self.gs_message = "ground station requested immediate landing"
            self._request_land(self.gs_message)

    def _publish_gs_status(self) -> None:
        payload = {
            "type": "px4_status",
            "state": self.state,
            "mode": self.bridge.mode,
            "armed": bool(self.bridge.armed or self.ready == 1),
            "ekf_ok": bool(self.bridge.ekf_ok),
            "link_ok": bool(self.bridge.link_ok),
            "battery_pct": self.bridge.battery_pct,
            "ready": int(self.ready),
            "mission_active": bool(self.mission_active),
            "error": self.failure_reason,
            "message": self.gs_message,
            "takeoff_available": (
                self.flight_enabled and self.state == "WAIT_GO"
            ),
        }
        self.gs_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.last_status_s = time.monotonic()

    def _on_rosout(self, msg: Log) -> None:
        if msg.name != "mavlink_bridge":
            return
        status = parse_bridge_status(str(msg.msg))
        if status is None:
            return
        self.bridge = status
        self.bridge_stamp = time.monotonic()

    # ------------------------------------------------------------------ Health
    def _fresh(self, stamp: float, timeout_s: float) -> bool:
        return stamp > 0.0 and time.monotonic() - stamp <= timeout_s

    def _odom_fresh(self) -> bool:
        return self.odom is not None and self._fresh(
            self.odom_stamp, self.odom_timeout_s
        )

    def _height_fresh(self) -> bool:
        return self.height_cm is not None and self._fresh(
            self.height_stamp, self.height_timeout_s
        )

    def _tof_fresh(self) -> bool:
        return self.tof_cm is not None and self._fresh(
            self.tof_stamp, self.tof_timeout_s
        )

    def _tof_ground(self) -> bool:
        return self._tof_fresh() and self.tof_cm is not None and self.tof_cm <= 12.0

    def _ready_fresh(self) -> bool:
        # The bridge publishes ready=0 only on state change (latched), while
        # ready=1 is refreshed at 1 Hz. Treat a received zero as the stable
        # disarmed state; require freshness only while armed/ready.
        return self.ready_seen and (
            self.ready == 0
            or self._fresh(self.ready_stamp, self.ready_timeout_s)
        )

    def _bridge_fresh(self) -> bool:
        return self._fresh(self.bridge_stamp, self.bridge_timeout_s)

    def _rate(self, samples, window_s: float) -> float:
        now = time.monotonic()
        return sample_rate_hz(
            [stamp for stamp, *_rest in samples if now - stamp <= window_s]
        )

    def _stability(self) -> tuple[bool, float, float, int]:
        now = time.monotonic()
        window = [item for item in self.odom_hist if now - item[0] <= self.stable_window_s]
        drift_cm, yaw_span_deg, count = stability_stats(window)
        stable = (
            count >= 8
            and drift_cm <= self.stable_max_drift_cm
            and yaw_span_deg <= self.stable_max_yaw_deg
        )
        return stable, drift_cm, yaw_span_deg, count

    def _preflight(self) -> tuple[bool, str]:
        if not self._bridge_fresh():
            return False, "mavlink_bridge status is unavailable"
        if not self.bridge.link_ok:
            return False, "MAVLink link is down"
        if self.bridge.armed:
            return False, "PX4 is armed before GO"
        if not self.bridge.ekf_ok:
            return False, "PX4 EKF is not ready"
        if self.bridge.mode != "OFFBOARD":
            return False, f"PX4 mode is {self.bridge.mode or 'unknown'}, select OFFBOARD"
        if not self._ready_fresh() or self.ready != 0:
            return False, "flight-controller ready state is unavailable or armed"
        if not self._odom_fresh():
            return False, "SLAM odometry is stale"
        if not self._height_fresh():
            return False, "PX4 height is stale"
        if self.require_tof_ground and not self._tof_ground():
            return (
                False,
                f"downward ToF is not at ground ({self.tof_cm if self.tof_cm is not None else '--'}cm)",
            )
        if self.require_battery:
            if self.bridge.battery_pct is None:
                return False, "battery percentage is unavailable"
            if self.bridge.battery_pct < self.minimum_battery_pct:
                return (
                    False,
                    f"battery {self.bridge.battery_pct}% < {self.minimum_battery_pct}%",
                )
        odom_hz = self._rate(self.odom_hist, 2.0)
        height_hz = self._rate(self.height_hist, 2.0)
        if odom_hz < self.minimum_odom_rate_hz:
            return False, f"odometry rate {odom_hz:.1f}Hz is too low"
        if height_hz < self.minimum_height_rate_hz:
            return False, f"height rate {height_hz:.1f}Hz is too low"
        stable, drift_cm, yaw_span_deg, count = self._stability()
        if not stable:
            return (
                False,
                f"position not stable: drift={drift_cm:.1f}cm "
                f"yaw={yaw_span_deg:.1f}deg samples={count}",
            )
        battery = (
            f"{self.bridge.battery_pct}%"
            if self.bridge.battery_pct is not None
            else "not reported"
        )
        return (
            True,
            f"OFFBOARD disarmed EKF=OK battery={battery} "
            f"odom={odom_hz:.1f}Hz height={height_hz:.1f}Hz "
            f"tof={self.tof_cm:.1f}cm drift={drift_cm:.1f}cm "
            f"yaw={yaw_span_deg:.1f}deg",
        )

    # ------------------------------------------------------------------ Output
    def _send_velocity_zero(self) -> None:
        now = time.monotonic()
        if now - self.last_zero_s < 0.05:
            return
        self.last_zero_s = now
        self.velocity_pub.publish(
            Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        )

    def _send_command(self, command: str) -> None:
        self.command_pub.publish(String(data=command))
        self._log(f"PX4 command: {command}")

    def _publish_trigger(self, enabled: bool) -> None:
        self.trigger_pub.publish(Bool(data=bool(enabled)))
        if enabled:
            self._log("shelf mission released: /fc_takeoff_if=true")

    def _write_ready(self, details: str) -> None:
        Path(self.ready_file).write_text(
            "READY\n"
            f"time={time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            f"{details}\n",
            encoding="utf-8",
        )

    @staticmethod
    def _remove_if_exists(path: str) -> None:
        try:
            os.remove(path)
        except FileNotFoundError:
            pass

    def _set_state(self, state: str) -> None:
        self.state = state
        self.state_since = time.monotonic()
        self._log(f"PHASE={state}")

    def _log(self, text: str, throttle_s: float = 0.0) -> None:
        if throttle_s > 0.0:
            now = time.monotonic()
            if now - self.last_log_s < throttle_s:
                return
            self.last_log_s = now
        self.get_logger().info(text)

    def _finish(self, code: int, reason: str = "") -> None:
        if self.done:
            return
        self.done = True
        self.exit_code = int(code)
        self.failure_reason = reason
        self.state = "FINISHED"
        self._remove_if_exists(self.ready_file)
        self._remove_if_exists(self.go_file)
        self._remove_if_exists(self.heartbeat_file)
        self._publish_gs_status()
        if code == 0:
            self._log(f"adapter finished: {reason or 'complete'}")
        else:
            self._log(f"adapter stopped with code {code}: {reason or 'unknown error'}")

    def _request_land(self, reason: str) -> None:
        if self.done or self.state == "LAND":
            return
        if self.bridge.armed or self.ready == 1:
            self.failure_reason = reason
            self.exit_code = self.exit_code or 2
            self._log(f"ABORT: {reason}; requesting PX4 Land")
            self._publish_trigger(False)
            self._send_velocity_zero()
            self._send_command("land")
            self.last_land_command_s = time.monotonic()
            self._set_state("LAND")
        else:
            self._finish(2, reason)

    def request_stop(self) -> None:
        self.stop_requested = True

    # ------------------------------------------------------------------ State machine
    def _tick(self) -> None:
        if self.done:
            return
        try:
            now = time.monotonic()
            if now - self.last_status_s >= 0.5:
                self._publish_gs_status()
            if now - self.last_heartbeat_s >= 1.0:
                self.last_heartbeat_s = now
                try:
                    Path(self.heartbeat_file).touch(exist_ok=True)
                except OSError:
                    pass

            if self.stop_requested:
                if self.bridge.armed or self.ready == 1:
                    self._request_land("operator interrupted the adapter")
                else:
                    self._finish(130, "operator interrupted before arming")
                self.stop_requested = False
                return

            if os.path.exists(self.abort_file):
                self._remove_if_exists(self.abort_file)
                self._request_land("abort file requested")
                return

            if self.state in (
                "WAIT_TOPICS",
                "WAIT_STABLE",
                "WAIT_OFFBOARD",
                "WAIT_GO",
                "ARMING",
                "START_MISSION",
            ):
                self._send_velocity_zero()

            elapsed = now - self.state_since
            if self.state == "WAIT_TOPICS":
                if (
                    self._odom_fresh()
                    and self._height_fresh()
                    and self._ready_fresh()
                    and self._bridge_fresh()
                ):
                    self._set_state("WAIT_STABLE")
                elif now - self.t0 > self.preflight_timeout_s:
                    self._finish(
                        3,
                        "timed out waiting for SLAM, height, PX4 readiness or bridge",
                    )

            elif self.state == "WAIT_STABLE":
                stable, drift_cm, yaw_span_deg, count = self._stability()
                if stable:
                    self._log(
                        f"preflight stable: drift={drift_cm:.1f}cm "
                        f"yaw-span={yaw_span_deg:.1f}deg samples={count}"
                    )
                    self._set_state("WAIT_OFFBOARD")
                elif elapsed > self.preflight_timeout_s:
                    self._finish(3, "SLAM position did not become stable")

            elif self.state == "WAIT_OFFBOARD":
                ok, details = self._preflight()
                if ok:
                    if not self.flight_enabled:
                        self._write_ready(details)
                        self._log(f"DRY-RUN PREFLIGHT PASS: {details}")
                        self._finish(0, "dry-run checks passed; no arm command sent")
                        return
                    self._write_ready(details)
                    self._log(f"PREFLIGHT PASS: {details}")
                    self._log(
                        "Aircraft is disarmed. Run shelf_takeoff.sh to send GO."
                    )
                    self._set_state("WAIT_GO")
                else:
                    self._log(f"PREFLIGHT WAIT: {details}", throttle_s=2.0)

            elif self.state == "WAIT_GO":
                ok, details = self._preflight()
                if not ok:
                    self._remove_if_exists(self.ready_file)
                    self._log(f"PREFLIGHT DEGRADED: {details}; returning to checks")
                    self._set_state("WAIT_STABLE")
                    return
                if (
                    not self.wait_for_go
                    or self.go_requested
                    or os.path.exists(self.go_file)
                ):
                    self._remove_if_exists(self.go_file)
                    self.go_requested = False
                    self._send_command("arm")
                    self.arm_sent_s = now
                    self._set_state("ARMING")
                elif elapsed > self.go_wait_timeout_s:
                    self._finish(3, "GO was not received before timeout; disarmed")

            elif self.state == "ARMING":
                if self.ready == 1 and self.bridge.armed:
                    if now - self.arm_sent_s >= self.command_delay_s:
                        self._publish_trigger(True)
                        self.trigger_s = now
                        self._set_state("START_MISSION")
                elif now - self.arm_sent_s > self.arm_timeout_s:
                    self._request_land("PX4 arm was rejected or did not become ready")

            elif self.state == "START_MISSION":
                if self.mission_active:
                    self._set_state("FLIGHT")
                elif elapsed > self.mission_start_timeout_s:
                    self._request_land("shelf mission did not acknowledge the start trigger")
                elif not self._odom_fresh() or not self._height_fresh():
                    self._request_land("sensor data was lost immediately after arming")
                elif not self._ready_fresh() or self.ready == 0:
                    self._finish(2, "PX4 disarmed before the shelf mission started")

            elif self.state == "FLIGHT":
                if self.mission_complete:
                    self._request_land("shelf route completed; handing final descent to PX4")
                    return
                if self.bridge.mode != "OFFBOARD":
                    self._publish_trigger(False)
                    self._log(
                        f"PX4 left OFFBOARD ({self.bridge.mode}); operator has control"
                    )
                    self._set_state("HANDOVER")
                    return
                if not self._odom_fresh():
                    self._request_land("SLAM odometry became stale")
                    return
                if not self._height_fresh():
                    self._request_land("PX4 height became stale")
                    return
                if self.require_tof_ground and not self._tof_fresh():
                    self._request_land("ToF height became stale")
                    return
                if not self._ready_fresh() or self.ready == 0:
                    self._finish(2, "PX4 disarmed during flight")
                    return
                if self.mission_active:
                    self.mission_inactive_since = 0.0
                else:
                    if self.mission_inactive_since == 0.0:
                        self.mission_inactive_since = now
                    elif (
                        now - self.mission_inactive_since
                        >= self.mission_inactive_grace_s
                    ):
                        self._request_land(
                            "shelf mission stopped without a completion event"
                        )
                        return
                if now - self.trigger_s > self.mission_max_duration_s:
                    self._request_land("shelf mission duration limit reached")

            elif self.state == "LAND":
                if not self._ready_fresh() or self.ready == 0:
                    code = 0 if self.mission_complete and not self.failure_reason else (
                        self.exit_code or 2
                    )
                    reason = (
                        "shelf route completed and PX4 landed/disarmed"
                        if code == 0
                        else self.failure_reason or "landing completed after abort"
                    )
                    self._finish(code, reason)
                    return
                if now - self.last_land_command_s >= 2.0:
                    self._send_command("land")
                    self.last_land_command_s = now
                if elapsed > self.land_wait_s:
                    self._finish(
                        self.exit_code or 8,
                        "landing/disarm confirmation timed out; no forced disarm was sent",
                    )

            elif self.state == "HANDOVER":
                self._remove_if_exists(self.ready_file)
                self._finish(
                    0,
                    "PX4 was switched out of OFFBOARD; mission trigger released",
                )
        except Exception:  # noqa: BLE001
            self.get_logger().error(
                "adapter exception; requesting landing\n" + traceback.format_exc()
            )
            self._request_land("unexpected adapter exception")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShelfPx4Adapter()
    stop_count = 0

    def _on_signal(_signum, _frame) -> None:
        nonlocal stop_count
        stop_count += 1
        node.request_stop()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.2)
            if stop_count >= 2 and not node.done:
                node.get_logger().error("second interrupt: adapter exits without another command")
                break
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
