"""Safety-gated bridge from the existing mission topics to PX4 Offboard.

This node never arms or disarms the aircraft. The pilot remains responsible
for arming and selecting Offboard with the RC transmitter. In bench mode it
does not publish any command to PX4.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, Float32MultiArray, Int16, String, UInt8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .frames import map_vector_to_ned, map_yaw_to_ned, quaternion_to_yaw
from .position_ramp import PositionRamp


PX4_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass
class RosPose:
    x: float
    y: float
    yaw: float


@dataclass
class FrameAnchor:
    ros_x: float
    ros_y: float
    ros_yaw: float
    px4_x: float
    px4_y: float
    px4_z: float
    px4_heading: float
    height_m: float


class Px4OffboardBridge(Node):
    """Translate legacy mission targets into PX4 NED trajectory setpoints."""

    def __init__(self) -> None:
        super().__init__("px4_offboard_bridge")

        self.bench_only = bool(self.declare_parameter("bench_only", True).value)
        self.transport_label = str(self.declare_parameter("transport_label", "DDS").value)
        self.require_native_reset_monitor = bool(
            self.declare_parameter("require_native_reset_monitor", False).value)
        self.native_reset_counter = None
        self.last_native_odometry_monotonic = 0.0
        self.control_mode = str(
            self.declare_parameter("control_mode", "velocity").value
        )
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.body_frame = str(self.declare_parameter("body_frame", "base_link").value)
        self.output_rate_hz = float(
            self.declare_parameter("output_rate_hz", 20.0).value
        )
        self.px4_timeout_sec = float(
            self.declare_parameter("px4_timeout_sec", 0.7).value
        )
        self.pose_timeout_sec = float(
            self.declare_parameter("pose_timeout_sec", 0.5).value
        )
        self.command_timeout_sec = float(
            self.declare_parameter("command_timeout_sec", 0.35).value
        )
        self.mission_start_grace_sec = float(
            self.declare_parameter("mission_start_grace_sec", 1.0).value
        )
        self.minimum_prestream_sec = float(
            self.declare_parameter("minimum_prestream_sec", 1.0).value
        )
        self.max_horizontal_speed_mps = float(
            self.declare_parameter("max_horizontal_speed_mps", 0.30).value
        )
        self.max_vertical_speed_mps = float(
            self.declare_parameter("max_vertical_speed_mps", 0.25).value
        )
        self.max_yaw_rate_rad_s = float(
            self.declare_parameter("max_yaw_rate_rad_s", math.radians(25.0)).value
        )
        self.auto_start_on_offboard = bool(
            self.declare_parameter("auto_start_on_offboard", True).value
        )
        self.allow_mode_command = bool(
            self.declare_parameter("allow_mode_command", False).value
        )
        self.allow_land_command = bool(
            self.declare_parameter("allow_land_command", False).value
        )
        self.land_on_mission_complete = bool(
            self.declare_parameter("land_on_mission_complete", True).value
        )
        self.land_on_fault = bool(self.declare_parameter("land_on_fault", False).value)
        self.require_preflight_checks = bool(
            self.declare_parameter("require_preflight_checks", True).value
        )
        self.target_system = int(self.declare_parameter("target_system", 1).value)
        self.target_component = int(
            self.declare_parameter("target_component", 1).value
        )

        self._validate_parameters()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status: Optional[VehicleStatus] = None
        self.local_position: Optional[VehicleLocalPosition] = None
        self.land_detected: Optional[VehicleLandDetected] = None
        self.ros_pose: Optional[RosPose] = None
        self.anchor: Optional[FrameAnchor] = None
        self.hold_setpoint: Optional[Tuple[float, float, float, float]] = None
        self.target_position: Optional[Tuple[float, float, float, float]] = None
        self.position_ramp = None
        self.target_velocity: Optional[Tuple[float, float, float, float]] = None

        self.last_status_monotonic = 0.0
        self.last_local_position_monotonic = 0.0
        self.last_ros_pose_monotonic = 0.0
        self.last_target_position_monotonic = 0.0
        self.last_target_velocity_monotonic = 0.0
        self.last_tf_warning_monotonic = 0.0
        self.stream_started_monotonic = 0.0
        self.mission_active_since = 0.0
        self.last_state_text = ""
        self.last_offboard = False
        self.offboard_started_once = False
        self.mission_active = False
        self.mission_start_future = None
        self.land_requested = False
        self.pending_land_reason = ""
        self.last_land_command_monotonic = 0.0
        self.land_command_attempts = 0
        self.land_ack_accepted = False
        self.landing_hold_setpoint: Optional[Tuple[float, float, float, float]] = None
        self.faulted = False
        self.fault_reason = ""

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(String, "/px4_bridge/state", state_qos)
        self.height_pub = self.create_publisher(Int16, "/height", 10)

        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._status_callback, PX4_QOS
        )
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._native_odometry_callback, PX4_QOS)
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_position_callback,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self._land_detected_callback,
            PX4_QOS,
        )
        self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self._command_ack_callback,
            PX4_QOS,
        )
        self.create_subscription(
            Float32MultiArray, "/target_position", self._target_position_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/target_velocity", self._target_velocity_callback, 10
        )
        self.create_subscription(UInt8, "/route_choice", self._route_callback, 10)
        self.create_subscription(
            Empty, "/mission_complete", self._mission_complete_callback, 10
        )

        self.mission_start_client = self.create_client(Trigger, "/p_to_p/start")
        self.create_service(Trigger, "/px4/bench_check", self._bench_check_callback)
        self.create_service(Trigger, "/px4/readiness_check", self._readiness_check_callback)
        self.create_service(
            Trigger, "/px4/request_offboard", self._request_offboard_callback
        )
        self.create_service(Trigger, "/px4/request_land", self._request_land_callback)

        self.timer = self.create_timer(1.0 / self.output_rate_hz, self._timer_callback)
        mode = "BENCH/只读" if self.bench_only else self.control_mode
        self.get_logger().info(
            f"PX4 bridge started in {mode} mode. This node never sends arm/disarm commands."
        )
        self._publish_state("WAITING_FOR_PX4")

    def _validate_parameters(self) -> None:
        if self.control_mode not in {"velocity", "position"}:
            raise ValueError("control_mode must be 'velocity' or 'position'")
        positive = {
            "output_rate_hz": self.output_rate_hz,
            "px4_timeout_sec": self.px4_timeout_sec,
            "pose_timeout_sec": self.pose_timeout_sec,
            "command_timeout_sec": self.command_timeout_sec,
            "minimum_prestream_sec": self.minimum_prestream_sec,
            "max_horizontal_speed_mps": self.max_horizontal_speed_mps,
            "max_vertical_speed_mps": self.max_vertical_speed_mps,
            "max_yaw_rate_rad_s": self.max_yaw_rate_rad_s,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")

    def _timestamp_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _native_odometry_callback(self, msg: VehicleOdometry) -> None:
        if (msg.pose_frame != 1 or not all(math.isfinite(v) for v in msg.position)
                or not 0 <= self._timestamp_us() - msg.timestamp_sample <= 300000):
            return
        previous = self.native_reset_counter
        self.native_reset_counter = msg.reset_counter
        self.last_native_odometry_monotonic = time.monotonic()
        if (self.require_native_reset_monitor and previous is not None
                and previous != msg.reset_counter and self.anchor is not None):
            if self._is_armed() and not self.bench_only:
                self._abort_offboard_output(
                    f"Native estimator reset {previous}->{msg.reset_counter}; old frame anchor invalid")
            else:
                self.anchor = None
                self.hold_setpoint = None

    def _status_callback(self, msg: VehicleStatus) -> None:
        previous = self.status
        self.status = msg
        self.last_status_monotonic = time.monotonic()
        if (self.bench_only or previous is None
                or previous.arming_state != VehicleStatus.ARMING_STATE_ARMED
                or msg.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            return
        if (previous.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and msg.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            if self.land_requested and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                return  # Expected, acknowledged/native landing handoff.
            self._abort_offboard_output("Pilot/PX4 left Offboard; lock old mission until disarm")
        elif (self.land_requested
              and previous.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
              and msg.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
            self._abort_offboard_output("Pilot/PX4 cancelled native Land; old mission inhibited")

    def _local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg
        self.last_local_position_monotonic = time.monotonic()
        if msg.dist_bottom_valid and math.isfinite(msg.dist_bottom):
            height_cm = int(round(msg.dist_bottom * 100.0))
            height_msg = Int16()
            height_msg.data = max(-32768, min(32767, height_cm))
            self.height_pub.publish(height_msg)

    def _land_detected_callback(self, msg: VehicleLandDetected) -> None:
        self.land_detected = msg

    def _command_ack_callback(self, msg: VehicleCommandAck) -> None:
        if msg.command not in {
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        }:
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info(f"PX4 accepted command {msg.command}")
            if msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND and self.land_requested:
                self.land_ack_accepted = True
        else:
            self.get_logger().error(
                f"PX4 rejected/failed command {msg.command}, result={msg.result}"
            )
            if msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND and self.land_requested:
                self._abort_offboard_output("PX4 rejected Land command")

    def _target_position_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4 or not all(math.isfinite(v) for v in msg.data[:4]):
            self.get_logger().error("Invalid /target_position; expected four finite values")
            return
        self.target_position = tuple(float(v) for v in msg.data[:4])
        self.last_target_position_monotonic = time.monotonic()

    def _target_velocity_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4 or not all(math.isfinite(v) for v in msg.data[:4]):
            self.get_logger().error("Invalid /target_velocity; expected four finite values")
            return
        self.target_velocity = tuple(float(v) for v in msg.data[:4])
        self.last_target_velocity_monotonic = time.monotonic()

    def _route_callback(self, _msg: UInt8) -> None:
        if (self.bench_only or self.faulted or self.land_requested
                or not self._is_armed() or not self._is_offboard()):
            return
        self.mission_active = True
        self.mission_active_since = time.monotonic()
        self.land_requested = False
        self.land_command_attempts = 0
        self.get_logger().info("Mission command forwarding enabled by /route_choice")

    def _mission_complete_callback(self, _msg: Empty) -> None:
        if not self.mission_active or not self._is_armed() or not self._is_offboard():
            self.get_logger().warning("Ignoring mission completion outside armed Offboard mission")
            return
        self.mission_active = False
        if self.land_on_mission_complete:
            ready, reason = self._ready_for_stream()
            if ready:
                self._begin_land("mission complete at B")
            else:
                self._abort_offboard_output(f"Cannot hand off to PX4 Land: {reason}")

    def _update_ros_pose(self) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time()
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warning_monotonic > 2.0:
                self.get_logger().warning(
                    f"TF {self.map_frame}->{self.body_frame} unavailable: {exc}"
                )
                self.last_tf_warning_monotonic = now
            return False

        stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age < -0.1 or age > self.pose_timeout_sec:
            return False
        q = transform.transform.rotation
        self.ros_pose = RosPose(
            transform.transform.translation.x,
            transform.transform.translation.y,
            quaternion_to_yaw(q.x, q.y, q.z, q.w),
        )
        self.last_ros_pose_monotonic = time.monotonic()
        return True

    def _is_fresh(self, timestamp: float, timeout: float) -> bool:
        return timestamp > 0.0 and time.monotonic() - timestamp <= timeout

    def _is_armed(self) -> bool:
        return (
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def _is_offboard(self) -> bool:
        return (
            self.status is not None
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _ready_for_stream(self) -> Tuple[bool, str]:
        if (self.require_native_reset_monitor and not self._is_fresh(
                self.last_native_odometry_monotonic, self.px4_timeout_sec)):
            return False, "Native estimator reset monitor timeout"
        if not self._is_fresh(self.last_status_monotonic, self.px4_timeout_sec):
            return False, "PX4 vehicle_status timeout"
        if not self._is_fresh(self.last_local_position_monotonic, self.px4_timeout_sec):
            return False, "PX4 local_position timeout"
        if not self._is_fresh(self.last_ros_pose_monotonic, self.pose_timeout_sec):
            return False, "Cartographer TF timeout"
        assert self.status is not None and self.local_position is not None
        if self.require_preflight_checks and not self.status.pre_flight_checks_pass:
            return False, "PX4 preflight checks have not passed"
        if not (
            self.local_position.xy_valid
            and self.local_position.z_valid
            and self.local_position.v_xy_valid
            and self.local_position.v_z_valid
            and math.isfinite(self.local_position.heading)
            and all(math.isfinite(v) for v in (
                self.local_position.x, self.local_position.y, self.local_position.z,
                self.local_position.vx, self.local_position.vy, self.local_position.vz))
        ):
            return False, "PX4 local position/velocity/heading invalid"
        if self.local_position.dead_reckoning:
            return False, "PX4 local position is dead reckoning"
        if not (
            self.local_position.dist_bottom_valid
            and math.isfinite(self.local_position.dist_bottom)
        ):
            return False, "MTF-01 range height invalid"
        return True, "ready"

    def _communication_health(self) -> Tuple[bool, str]:
        """Check the read-only telemetry link without requiring RC or Cartographer."""
        if not self._is_fresh(self.last_status_monotonic, self.px4_timeout_sec):
            return False, "PX4 vehicle_status timeout"
        if not self._is_fresh(self.last_local_position_monotonic, self.px4_timeout_sec):
            return False, "PX4 local_position timeout"
        assert self.local_position is not None
        range_text = (
            f"MTF range={self.local_position.dist_bottom:.3f}m"
            if self.local_position.dist_bottom_valid
            and math.isfinite(self.local_position.dist_bottom)
            else "MTF range not valid"
        )
        return True, f"{self.transport_label} link healthy; {range_text}"

    def _capture_anchor(self) -> None:
        assert self.ros_pose is not None and self.local_position is not None
        self.anchor = FrameAnchor(
            ros_x=self.ros_pose.x,
            ros_y=self.ros_pose.y,
            ros_yaw=self.ros_pose.yaw,
            px4_x=float(self.local_position.x),
            px4_y=float(self.local_position.y),
            px4_z=float(self.local_position.z),
            px4_heading=float(self.local_position.heading),
            height_m=float(self.local_position.dist_bottom),
        )
        self.hold_setpoint = (
            self.anchor.px4_x,
            self.anchor.px4_y,
            self.anchor.px4_z,
            self.anchor.px4_heading,
        )
        self.stream_started_monotonic = time.monotonic()
        self.position_ramp = PositionRamp(self.hold_setpoint, self.stream_started_monotonic)
        self.get_logger().info(
            "Captured ROS<->PX4 frame anchor; pre-streaming a hold setpoint"
        )

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        msg.position = self.control_mode == "position"
        msg.velocity = self.control_mode == "velocity"
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    @staticmethod
    def _limit_xy(x: float, y: float, limit: float) -> Tuple[float, float]:
        magnitude = math.hypot(x, y)
        if magnitude <= limit or magnitude == 0.0:
            return x, y
        scale = limit / magnitude
        return x * scale, y * scale

    def _position_setpoint(self) -> Tuple[float, float, float, float]:
        assert self.anchor is not None and self.hold_setpoint is not None
        if self.land_requested and self.landing_hold_setpoint is not None:
            return self.landing_hold_setpoint
        if not self._is_offboard() or not self.mission_active:
            return self.hold_setpoint
        if not self._is_fresh(
            self.last_target_position_monotonic, self.command_timeout_sec
        ):
            if time.monotonic() - self.mission_active_since > self.mission_start_grace_sec:
                self._handle_fault("position target timeout")
            return self.hold_setpoint
        assert self.target_position is not None
        x_cm, y_cm, height_cm, yaw_deg = self.target_position
        dx = x_cm / 100.0 - self.anchor.ros_x
        dy = y_cm / 100.0 - self.anchor.ros_y
        dn, de = map_vector_to_ned(
            dx, dy, self.anchor.ros_yaw, self.anchor.px4_heading
        )
        px4_z = self.anchor.px4_z - (height_cm / 100.0 - self.anchor.height_m)
        px4_yaw = map_yaw_to_ned(
            math.radians(yaw_deg), self.anchor.ros_yaw, self.anchor.px4_heading
        )
        return (
            self.anchor.px4_x + dn,
            self.anchor.px4_y + de,
            px4_z,
            px4_yaw,
        )

    def _velocity_setpoint(self) -> Tuple[float, float, float, float]:
        assert self.anchor is not None
        if not self._is_offboard() or not self.mission_active:
            return 0.0, 0.0, 0.0, 0.0
        if not self._is_fresh(
            self.last_target_velocity_monotonic, self.command_timeout_sec
        ):
            if time.monotonic() - self.mission_active_since > self.mission_start_grace_sec:
                self._handle_fault("velocity target timeout")
            return 0.0, 0.0, 0.0, 0.0
        assert self.target_velocity is not None
        vx_cm_s, vy_cm_s, vz_cm_s, yaw_rate_deg_s = self.target_velocity
        vn, ve = map_vector_to_ned(
            vx_cm_s / 100.0,
            vy_cm_s / 100.0,
            self.anchor.ros_yaw,
            self.anchor.px4_heading,
        )
        vn, ve = self._limit_xy(vn, ve, self.max_horizontal_speed_mps)
        vd = max(
            -self.max_vertical_speed_mps,
            min(self.max_vertical_speed_mps, -vz_cm_s / 100.0),
        )
        yaw_rate = max(
            -self.max_yaw_rate_rad_s,
            min(self.max_yaw_rate_rad_s, -math.radians(yaw_rate_deg_s)),
        )
        return vn, ve, vd, yaw_rate

    def _publish_trajectory_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        nan = float("nan")
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        if self.control_mode == "position":
            x, y, z, yaw = self._position_setpoint()
            # Shape normal task/hold references only; native Land handoff is unchanged.
            if not self.land_requested:
                x, y, z, yaw = self.position_ramp.step(
                    (x, y, z, yaw), time.monotonic(),
                    self.max_horizontal_speed_mps, self.max_vertical_speed_mps,
                    self.max_yaw_rate_rad_s)
            msg.position = [float(x), float(y), float(z)]
            msg.velocity = [nan, nan, nan]
            msg.yaw = float(yaw)
            msg.yawspeed = nan
        else:
            vx, vy, vz, yaw_rate = self._velocity_setpoint()
            msg.position = [nan, nan, nan]
            msg.velocity = [float(vx), float(vy), float(vz)]
            msg.yaw = nan
            msg.yawspeed = float(yaw_rate)
        self.trajectory_pub.publish(msg)

    def _send_vehicle_command(
        self, command: int, param1: float = 0.0, param2: float = 0.0
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self.command_pub.publish(msg)

    def _begin_land(self, reason: str) -> None:
        if self.land_requested or self.faulted:
            return
        if self.local_position is not None:
            self.landing_hold_setpoint = (
                float(self.local_position.x),
                float(self.local_position.y),
                float(self.local_position.z),
                float(self.local_position.heading),
            )
        self.mission_active = False
        self.land_requested = True
        self.pending_land_reason = reason
        self.land_command_attempts = 0
        self.land_ack_accepted = False
        self.last_land_command_monotonic = 0.0
        self.get_logger().warning(f"PX4 native landing requested: {reason}")

    def _abort_offboard_output(self, reason: str) -> None:
        if self.faulted:
            return
        self.faulted = True
        self.fault_reason = reason
        self.mission_active = False
        self.land_requested = False
        self.get_logger().error(
            f"Offboard output stopped: {reason}; PX4 Offboard-loss failsafe/pilot takeover required"
        )
        self._publish_state(f"FAULT_OUTPUT_STOPPED: {reason}")

    def _handle_fault(self, reason: str) -> None:
        if self.land_on_fault and self._is_armed() and self._is_offboard():
            self._begin_land(reason)
        else:
            self._abort_offboard_output(reason)

    def _service_land_request(self) -> None:
        if not self.land_requested or not self._is_armed():
            return
        if not self.allow_land_command:
            self._publish_state(f"LAND_BLOCKED: {self.pending_land_reason}")
            return
        now = time.monotonic()
        if self.land_ack_accepted:
            if now - self.last_land_command_monotonic >= 2.0 and self._is_offboard():
                self._abort_offboard_output("Land accepted but PX4 did not leave Offboard")
            return
        if now - self.last_land_command_monotonic < 1.0:
            return
        if self.land_command_attempts >= 3:
            self._abort_offboard_output("No confirmation of PX4 Land after three attempts")
            return
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_command_attempts += 1
        self.last_land_command_monotonic = now
        self._publish_state(f"LAND_REQUESTED: {self.pending_land_reason}")

    def _start_mission_if_needed(self) -> None:
        if (
            not self.auto_start_on_offboard
            or self.mission_active
            or self.offboard_started_once
        ):
            return
        if self.mission_start_future is not None:
            return
        if not self.mission_start_client.service_is_ready():
            self.get_logger().error("/p_to_p/start service is not available")
            return
        self.mission_start_future = self.mission_start_client.call_async(Trigger.Request())
        self.offboard_started_once = True
        self.get_logger().info("RC selected Offboard; requesting point-to-point mission start")

    def _check_mission_start_result(self) -> None:
        if self.mission_start_future is None or not self.mission_start_future.done():
            return
        try:
            response = self.mission_start_future.result()
            if response.success:
                self.get_logger().info(f"Mission start accepted: {response.message}")
            else:
                self.get_logger().error(f"Mission start rejected: {response.message}")
                self._handle_fault("mission start rejected")
        except Exception as exc:  # rclpy future exceptions are runtime-specific
            self.get_logger().error(f"Mission start service failed: {exc}")
            self._handle_fault("mission start service failure")
        finally:
            self.mission_start_future = None

    def _health_text(self) -> str:
        ready, reason = self._ready_for_stream()
        status = "ARMED" if self._is_armed() else "DISARMED"
        mode = "OFFBOARD" if self._is_offboard() else "NON_OFFBOARD"
        return f"{'READY' if ready else 'NOT_READY'} {status} {mode}: {reason}"

    def _publish_state(self, text: str) -> None:
        if text == self.last_state_text:
            return
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)
        self.last_state_text = text

    def _bench_check_callback(self, _request, response):
        healthy, detail = self._communication_health()
        response.success = healthy
        response.message = detail
        return response

    def _readiness_check_callback(self, _request, response):
        # Read-only: unlike /request_offboard this never emits a command.
        if self.faulted:
            response.success = False
            response.message = "Bridge output inhibited until disarm: " + self.fault_reason
            return response
        self._update_ros_pose()
        response.success, response.message = self._ready_for_stream()
        return response

    def _request_offboard_callback(self, _request, response):
        if self.faulted:
            response.success = False
            response.message = "Old Offboard session inhibited; disarm before a new mission"
            return response
        if self.bench_only:
            response.success = False
            response.message = "bench_only=true: no PX4 commands are allowed"
            return response
        if not self.allow_mode_command:
            response.success = False
            response.message = "allow_mode_command=false: select Offboard using RC"
            return response
        if not self._is_armed():
            response.success = False
            response.message = "Refusing Offboard request: aircraft is not RC-armed"
            return response
        ready, reason = self._ready_for_stream()
        prestream_age = time.monotonic() - self.stream_started_monotonic
        if not ready or self.anchor is None or prestream_age < self.minimum_prestream_sec:
            response.success = False
            response.message = f"Not ready/prestreamed: {reason}"
            return response
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        response.success = True
        response.message = "Offboard mode command sent; arming was not requested"
        return response

    def _request_land_callback(self, _request, response):
        if self.faulted:
            response.success = False
            response.message = "Old mission inhibited; select landing/takeover using RC"
            return response
        if self.bench_only or not self.allow_land_command:
            response.success = False
            response.message = "Landing command is disabled by configuration"
            return response
        if not self._is_armed():
            response.success = False
            response.message = "Aircraft is not armed"
            return response
        self._begin_land("operator service request")
        response.success = True
        response.message = "PX4 Land requested; monitor mode and descent"
        return response

    def _reset_after_disarm(self) -> None:
        self.position_ramp = None
        self.anchor = None
        self.hold_setpoint = None
        self.stream_started_monotonic = 0.0
        self.mission_active = False
        self.mission_active_since = 0.0
        self.mission_start_future = None
        self.land_requested = False
        self.pending_land_reason = ""
        self.land_command_attempts = 0
        self.land_ack_accepted = False
        self.landing_hold_setpoint = None
        self.faulted = False
        self.fault_reason = ""
        self.offboard_started_once = False
        self.target_position = None
        self.target_velocity = None
        self.last_target_position_monotonic = 0.0
        self.last_target_velocity_monotonic = 0.0

    def _timer_callback(self) -> None:
        self._update_ros_pose()
        self._check_mission_start_result()

        if self.bench_only:
            healthy, detail = self._communication_health()
            prefix = "BENCH_LINK_OK" if healthy else "BENCH_WAITING"
            self._publish_state(f"{prefix}: {detail}")
            return

        if not self._is_armed():
            if self.anchor is not None or self.faulted or self.land_requested:
                self._reset_after_disarm()
            self.last_offboard = False
            self._publish_state(self._health_text())
            return

        if self.faulted:
            self._publish_state(f"FAULT_OUTPUT_STOPPED: {self.fault_reason}")
            return

        if self.land_requested and not self._is_offboard():
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self._publish_state("PX4_LANDING: native Land mode active")
            else:
                self._abort_offboard_output("Land cancelled by pilot/PX4 mode change")
            return

        # Process takeover/re-entry BEFORE publishing any setpoint, including
        # when an async start is pending or no route kick has arrived yet.
        if ((self.last_offboard and not self._is_offboard())
                or (self._is_offboard() and not self.last_offboard
                    and self.offboard_started_once)):
            self._abort_offboard_output("Offboard session ended; disarm before a new mission")
            self.last_offboard = False
            return

        ready, reason = self._ready_for_stream()
        if not ready:
            if self._is_offboard():
                self._handle_fault(reason)
            self._publish_state("NOT_READY: " + reason)
            return

        if self.anchor is None:
            self._capture_anchor()

        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        is_offboard = self._is_offboard()
        if is_offboard and not self.last_offboard:
            self._start_mission_if_needed()
        self.last_offboard = is_offboard

        self._service_land_request()
        if not self.faulted:
            self._publish_state(
                f"LAND_REQUESTED: {self.pending_land_reason}"
                if self.land_requested else self._health_text()
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4OffboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
