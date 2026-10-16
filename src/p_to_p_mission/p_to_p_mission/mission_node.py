import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import rclpy
from px4_msgs.msg import VehicleLandDetected, VehicleStatus
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, Float32MultiArray, Int16, String, UInt8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import body_offset_to_map, normalize_angle_deg, target_error
from .landing import next_descent_setpoint_cm
from .preflight import ground_height_baseline_cm
from .task_guard import envelope_reason, stage_reason


@dataclass
class PoseCm:
    x: float
    y: float
    yaw_deg: float


@dataclass
class Target:
    x: float
    y: float
    z: float
    yaw_deg: float


class PointToPointMission(Node):
    """A -> takeoff -> B -> hold/descend mission coordinator.

    This node publishes map-frame targets.  Position PID converts them to a
    map-frame velocity, and uart_to_stm32 rotates that velocity into the body
    frame before sending protocol frame 0x31.
    """

    ACTIVE_STATES = {
        "TAKEOFF",
        "GOTO_B",
        "HOLD_B",
        "DESCEND_B",
        "ABORT_HOLD",
        "FAULT_HOLD",
        "OBSTACLE_HOLD",
    }

    def __init__(self) -> None:
        super().__init__("p_to_p_mission")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.body_frame = self.declare_parameter("body_frame", "laser_link").value
        self.height_topic = self.declare_parameter(
            "height_topic", "/height"
        ).value
        self.target_mode = self.declare_parameter("target_mode", "relative_body").value
        self.b_offset_x_cm = float(self.declare_parameter("b_offset_x_cm", 250.0).value)
        self.b_offset_y_cm = float(self.declare_parameter("b_offset_y_cm", -250.0).value)
        self.b_x_cm = float(self.declare_parameter("b_x_cm", 250.0).value)
        self.b_y_cm = float(self.declare_parameter("b_y_cm", -250.0).value)
        self.cruise_height_cm = float(
            self.declare_parameter("cruise_height_cm", 150.0).value
        )
        self.landing_height_cm = float(
            self.declare_parameter("landing_height_cm", 20.0).value
        )
        self.target_yaw_deg = float(self.declare_parameter("target_yaw_deg", 0.0).value)
        self.keep_takeoff_yaw = bool(self.declare_parameter("keep_takeoff_yaw", True).value)
        self.terminal_behavior = self.declare_parameter("terminal_behavior", "hold").value
        self.pre_descent_hold_sec = float(
            self.declare_parameter("pre_descent_hold_sec", 3.0).value
        )
        self.position_tolerance_cm = float(
            self.declare_parameter("position_tolerance_cm", 12.0).value
        )
        self.height_tolerance_cm = float(
            self.declare_parameter("height_tolerance_cm", 12.0).value
        )
        self.landing_height_tolerance_cm = float(
            self.declare_parameter("landing_height_tolerance_cm", 3.0).value
        )
        self.active_descent_rate_cm_s = float(
            self.declare_parameter("active_descent_rate_cm_s", 10.0).value
        )
        self.active_descent_max_lead_cm = float(
            self.declare_parameter("active_descent_max_lead_cm", 5.0).value
        )
        self.ground_height_max_cm = float(
            self.declare_parameter("ground_height_max_cm", 20.0).value
        )
        self.native_land_handoff_margin_cm = float(
            self.declare_parameter("native_land_handoff_margin_cm", 4.0).value
        )
        self.yaw_tolerance_deg = float(
            self.declare_parameter("yaw_tolerance_deg", 8.0).value
        )
        self.reached_stable_sec = float(
            self.declare_parameter("reached_stable_sec", 1.0).value
        )
        self.sensor_timeout_sec = float(
            self.declare_parameter("sensor_timeout_sec", 0.5).value
        )
        self.px4_status_timeout_sec = float(
            self.declare_parameter("px4_status_timeout_sec", 1.2).value
        )
        self.require_rc_offboard = bool(
            self.declare_parameter("require_rc_offboard", False).value
        )
        self.stop_output_max_height_cm = float(
            self.declare_parameter("stop_output_max_height_cm", 25.0).value
        )
        self.abort_descent_on_obstacle = bool(
            self.declare_parameter("abort_descent_on_obstacle", True).value
        )
        self.route_choice = int(self.declare_parameter("route_choice", 3).value)
        self.update_rate_hz = float(self.declare_parameter("update_rate_hz", 10.0).value)
        self.task_guard_enabled = bool(self.declare_parameter("task_guard_enabled", False).value)
        self.task_radius_cm = float(self.declare_parameter("task_radius_cm", 150.0).value)
        self.task_max_height_cm = float(self.declare_parameter("task_max_height_cm", 120.0).value)
        self.takeoff_timeout_sec = float(self.declare_parameter("takeoff_timeout_sec", 20.0).value)
        self.travel_timeout_sec = float(self.declare_parameter("travel_timeout_sec", 20.0).value)
        self.hold_timeout_sec = float(self.declare_parameter("hold_timeout_sec", 20.0).value)
        self.descent_timeout_sec = float(self.declare_parameter("descent_timeout_sec", 30.0).value)

        self._validate_parameters()

        durable_qos = QoSProfile(depth=1)
        durable_qos.reliability = ReliabilityPolicy.RELIABLE
        durable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        route_qos = QoSProfile(depth=1)
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        self.target_pub = self.create_publisher(
            Float32MultiArray, "/target_position", durable_qos
        )
        self.state_pub = self.create_publisher(String, "/p_to_p/state", durable_qos)
        self.route_pub = self.create_publisher(UInt8, "/route_choice", route_qos)
        self.mission_complete_pub = self.create_publisher(Empty, "/mission_complete", 10)

        self.height_sub = self.create_subscription(
            Int16, self.height_topic, self._height_callback, 10
        )
        px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._status_callback, px4_qos
        )
        self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected",
            self._land_detected_callback, px4_qos
        )
        self.obstacle_sub = self.create_subscription(
            Bool, "/laser_array/obstacle_below", self._obstacle_callback, 10
        )
        self.start_srv = self.create_service(Trigger, "/p_to_p/start", self._start_callback)
        self.readiness_srv = self.create_service(
            Trigger, "/p_to_p/readiness_check", self._readiness_callback
        )
        self.abort_srv = self.create_service(
            Trigger, "/p_to_p/abort_hold", self._abort_hold_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "/p_to_p/stop_output", self._stop_output_callback
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = "IDLE"
        self.current_pose: Optional[PoseCm] = None
        self.current_height_cm: Optional[float] = None
        self.recent_ground_heights = deque(maxlen=30)
        self.vehicle_status: Optional[VehicleStatus] = None
        self.last_vehicle_status_monotonic = 0.0
        self.land_detected: Optional[VehicleLandDetected] = None
        self.last_land_detected_monotonic = 0.0
        self.flight_session_seen_armed = False
        self.last_pose_monotonic = 0.0
        self.last_height_monotonic = 0.0
        self.last_tf_warning_monotonic = 0.0
        self.obstacle_below = False
        self.a_pose: Optional[PoseCm] = None
        self.b_target: Optional[Target] = None
        self.active_target: Optional[Target] = None
        self.reached_since: Optional[float] = None
        self.state_entered_monotonic = time.monotonic()
        self.route_kick_remaining = 0
        self.ground_height_cm: Optional[float] = None
        self.last_descent_tick_monotonic = 0.0

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self._timer_callback)
        self._publish_state("等待定位、测高与人工启动")
        self.get_logger().info(
            "Point-to-point mission ready. Call /p_to_p/start after preflight checks."
        )

    def _validate_parameters(self) -> None:
        for value in (self.task_radius_cm, self.task_max_height_cm,
                      self.takeoff_timeout_sec, self.travel_timeout_sec,
                      self.hold_timeout_sec, self.descent_timeout_sec):
            if not math.isfinite(value) or value <= 0.:
                raise ValueError('task guard limits must be finite and positive')
        if self.task_guard_enabled and self.cruise_height_cm > self.task_max_height_cm:
            raise ValueError('cruise height exceeds task guard ceiling')
        if self.target_mode not in {"relative_body", "relative_map", "absolute"}:
            raise ValueError(
                "target_mode must be relative_body, relative_map, or absolute"
            )
        if self.terminal_behavior not in {"hold", "land", "px4_land", "guided_land_handoff"}:
            raise ValueError("terminal_behavior must be hold, land, px4_land, or guided_land_handoff")
        if self.cruise_height_cm <= self.landing_height_cm:
            raise ValueError("cruise_height_cm must be greater than landing_height_cm")
        if self.update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        if self.sensor_timeout_sec <= 0.0:
            raise ValueError("sensor_timeout_sec must be positive")
        if self.px4_status_timeout_sec <= 0.0:
            raise ValueError("px4_status_timeout_sec must be positive")
        if self.landing_height_tolerance_cm <= 0.0:
            raise ValueError("landing_height_tolerance_cm must be positive")
        if self.active_descent_rate_cm_s <= 0.0:
            raise ValueError("active_descent_rate_cm_s must be positive")
        if self.active_descent_max_lead_cm <= 0.0:
            raise ValueError("active_descent_max_lead_cm must be positive")
        if self.ground_height_max_cm <= 0.0 or self.native_land_handoff_margin_cm <= 0.0:
            raise ValueError("ground height limit and Land handoff margin must be positive")
        if self.stop_output_max_height_cm < 0.0:
            raise ValueError("stop_output_max_height_cm must not be negative")

    def _height_callback(self, msg: Int16) -> None:
        self.current_height_cm = float(msg.data)
        self.last_height_monotonic = time.monotonic()
        if (
            self.vehicle_status is not None
            and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
            and time.monotonic() - self.last_vehicle_status_monotonic <= self.px4_status_timeout_sec
        ):
            self.recent_ground_heights.append((self.last_height_monotonic, self.current_height_cm))

    def _obstacle_callback(self, msg: Bool) -> None:
        self.obstacle_below = bool(msg.data)

    def _status_callback(self, msg: VehicleStatus) -> None:
        previous = self.vehicle_status
        self.vehicle_status = msg
        self.last_vehicle_status_monotonic = time.monotonic()
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.flight_session_seen_armed = True
            if (self.state in self.ACTIVE_STATES or self.state == "FC_LANDING"):
                expected_land = (self.state == "FC_LANDING"
                                 and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)
                cancelled_land = (self.state == "FC_LANDING" and previous is not None
                                  and previous.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                                  and msg.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND)
                if cancelled_land or (msg.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and not expected_land):
                    self.active_target = None
                    self.route_kick_remaining = 0
                    self._enter_state("PILOT_TAKEOVER", "已切出 Offboard；旧任务取消，着地锁桨前禁止重启")
        self._reset_if_landed_and_disarmed()

    def _land_detected_callback(self, msg: VehicleLandDetected) -> None:
        self.land_detected = msg
        self.last_land_detected_monotonic = time.monotonic()
        self._reset_if_landed_and_disarmed()

    def _reset_if_landed_and_disarmed(self) -> None:
        now = time.monotonic()
        if (not self.flight_session_seen_armed
                or self.vehicle_status is None or self.land_detected is None
                or self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_DISARMED
                or not self.land_detected.landed
                or not 0.0 <= now - self.last_vehicle_status_monotonic <= self.px4_status_timeout_sec
                or not 0.0 <= now - self.last_land_detected_monotonic <= self.px4_status_timeout_sec):
            return
        self.flight_session_seen_armed = False
        if self.state == "IDLE":
            return
        self.active_target = None
        self.a_pose = None
        self.b_target = None
        self.ground_height_cm = None
        self.recent_ground_heights.clear()
        self.route_kick_remaining = 0
        self.last_descent_tick_monotonic = 0.0
        self._enter_state("IDLE", "飞控确认着地且锁桨；旧任务已清理，重新采集地面基线")

    def _rc_offboard_active(self) -> bool:
        return (self.vehicle_status is not None
                and 0.0 <= time.monotonic() - self.last_vehicle_status_monotonic <= self.px4_status_timeout_sec
                and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _update_pose(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, Time()
            )
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_tf_warning_monotonic >= 2.0:
                self.get_logger().warning(
                    f"TF {self.map_frame}->{self.body_frame} unavailable: {exc}"
                )
                self.last_tf_warning_monotonic = now
            return False

        transform_age_sec = (
            self.get_clock().now().nanoseconds
            - Time.from_msg(tf.header.stamp).nanoseconds
        ) / 1e9
        if transform_age_sec > self.sensor_timeout_sec or transform_age_sec < -0.1:
            return False

        q = tf.transform.rotation
        self.current_pose = PoseCm(
            x=tf.transform.translation.x * 100.0,
            y=tf.transform.translation.y * 100.0,
            yaw_deg=self._yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.last_pose_monotonic = time.monotonic()
        return True

    def _sensors_fresh(self) -> bool:
        now = time.monotonic()
        return (
            self.current_pose is not None
            and self.current_height_cm is not None
            and now - self.last_pose_monotonic <= self.sensor_timeout_sec
            and now - self.last_height_monotonic <= self.sensor_timeout_sec
        )

    def _start_readiness(self):
        """Shared start contract; checks only, no state or target changes."""
        if self.state in self.ACTIVE_STATES:
            return False, f"任务已在运行，当前状态={self.state}", None
        if self.state in {"FC_LANDING", "PILOT_TAKEOVER"}:
            return False, "任务已交接降落或人工接管；等待飞控确认着地锁桨后复位", None
        if not self._sensors_fresh():
            return False, "定位或高度数据尚未就绪/已超时", None

        baseline = None
        if self.terminal_behavior == "guided_land_handoff":
            now = time.monotonic()
            if (self.vehicle_status is None
                    or not 0.0 <= now - self.last_vehicle_status_monotonic
                    <= self.px4_status_timeout_sec):
                return False, "任务未收到新鲜的飞控状态，请检查通信话题映射", None
            baseline = ground_height_baseline_cm(
                self.recent_ground_heights, now, self.ground_height_max_cm
            )
            if baseline is None:
                return False, "缺少解锁前稳定的地面测高基线，拒绝启动近地交接任务", None
        return True, "任务启动条件就绪（未启动任务）", baseline

    def _readiness_callback(self, _request, response):
        self._update_pose()
        response.success, response.message, baseline = self._start_readiness()
        if response.success and baseline is not None:
            response.message += f"；地面测高基线={baseline:.1f}cm"
        return response

    def _start_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self._update_pose()
        response.success, response.message, baseline = self._start_readiness()
        if not response.success:
            return response
        if self.require_rc_offboard and not self._rc_offboard_active():
            response.success = False
            response.message = "任务只能在遥控器解锁并切入 Offboard 后启动"
            return response
        if baseline is not None:
            self.ground_height_cm = baseline

        assert self.current_pose is not None
        self.a_pose = PoseCm(
            self.current_pose.x, self.current_pose.y, self.current_pose.yaw_deg
        )
        yaw_target = (
            self.a_pose.yaw_deg if self.keep_takeoff_yaw else self.target_yaw_deg
        )

        if self.target_mode == "relative_body":
            b_x, b_y = body_offset_to_map(
                self.a_pose.x,
                self.a_pose.y,
                self.a_pose.yaw_deg,
                self.b_offset_x_cm,
                self.b_offset_y_cm,
            )
        elif self.target_mode == "relative_map":
            b_x = self.a_pose.x + self.b_offset_x_cm
            b_y = self.a_pose.y + self.b_offset_y_cm
        else:
            b_x, b_y = self.b_x_cm, self.b_y_cm

        if self.task_guard_enabled:
            reason = envelope_reason(b_x, b_y, self.cruise_height_cm,
                                     self.a_pose.x, self.a_pose.y,
                                     self.task_radius_cm, self.task_max_height_cm)
            if reason:
                response.success = False
                response.message = '任务拒绝：' + reason
                return response
        self.b_target = Target(b_x, b_y, self.cruise_height_cm, yaw_target)
        self.active_target = Target(
            self.a_pose.x, self.a_pose.y, self.cruise_height_cm, yaw_target
        )
        self.route_kick_remaining = 1
        self._enter_state("TAKEOFF", "任务启动，保持 A 点水平位置并爬升")

        response.success = True
        response.message = (
            f"启动成功：A=({self.a_pose.x:.1f},{self.a_pose.y:.1f})cm, "
            f"B=({b_x:.1f},{b_y:.1f})cm, 高度={self.cruise_height_cm:.1f}cm"
        )
        return response

    def _abort_hold_callback(self, _request: Trigger.Request, response: Trigger.Response):
        if self.state == "PILOT_TAKEOVER" or (self.require_rc_offboard and not self._rc_offboard_active()):
            response.success = False
            response.message = "人工接管或未进入已解锁 Offboard，禁止恢复旧任务"
            return response
        self._update_pose()
        if not self._sensors_fresh():
            response.success = False
            response.message = "无法取得新鲜位姿/高度，飞控端看门狗将负责速度归零"
            return response

        assert self.current_pose is not None and self.current_height_cm is not None
        self.active_target = Target(
            self.current_pose.x,
            self.current_pose.y,
            self.current_height_cm if self.terminal_behavior == "guided_land_handoff"
            else max(self.current_height_cm, self.landing_height_cm),
            self.current_pose.yaw_deg,
        )
        self.route_kick_remaining = 1
        self._enter_state("ABORT_HOLD", "任务中止：保持当前位置，等待人工处理")
        response.success = True
        response.message = "已切换为当前位置悬停；不会自动锁桨"
        return response

    def _stop_output_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self._update_pose()
        if self.terminal_behavior == "guided_land_handoff":
            response.success = False
            response.message = "近地交接模式禁用 stop_output；请使用遥控器切换模式接管"
            return response
        if not self._sensors_fresh():
            response.success = False
            response.message = "定位或高度数据不新鲜，拒绝停止控制输出"
            return response
        if not 0.0 <= self.current_height_cm <= self.stop_output_max_height_cm:
            response.success = False
            response.message = (
                f"当前高度不在 0~{self.stop_output_max_height_cm:.1f}cm 安全范围，"
                "拒绝停止控制输出"
            )
            return response
        self.mission_complete_pub.publish(Empty())
        self.active_target = None
        self._enter_state("FINISHED", "已在近地高度停止速度输出，请人工锁桨")
        response.success = True
        response.message = "已发送零速度和任务结束帧，请确认落地后人工锁桨"
        return response

    def _enter_state(self, state: str, detail: str) -> None:
        self.state = state
        self.state_entered_monotonic = time.monotonic()
        self.reached_since = None
        self._publish_state(detail)
        self.get_logger().info(f"STATE -> {state}: {detail}")

    def _publish_state(self, detail: str = "") -> None:
        msg = String()
        msg.data = f"{self.state}: {detail}" if detail else self.state
        self.state_pub.publish(msg)

    def _publish_target(self) -> None:
        if self.active_target is None:
            return
        msg = Float32MultiArray()
        msg.data = [
            float(self.active_target.x),
            float(self.active_target.y),
            float(self.active_target.z),
            float(self.active_target.yaw_deg),
        ]
        self.target_pub.publish(msg)

    def _target_reached_stably(self, height_tolerance_cm: Optional[float] = None) -> bool:
        if (
            self.active_target is None
            or self.current_pose is None
            or self.current_height_cm is None
        ):
            self.reached_since = None
            return False

        xy_error, z_error, yaw_error = target_error(
            self.current_pose.x,
            self.current_pose.y,
            self.current_height_cm,
            self.current_pose.yaw_deg,
            self.active_target.x,
            self.active_target.y,
            self.active_target.z,
            self.active_target.yaw_deg,
        )
        inside = (
            xy_error <= self.position_tolerance_cm
            and z_error <= (
                self.height_tolerance_cm
                if height_tolerance_cm is None else height_tolerance_cm
            )
            and yaw_error <= self.yaw_tolerance_deg
        )
        if not inside:
            self.reached_since = None
            return False
        if self.reached_since is None:
            self.reached_since = time.monotonic()
            return False
        return time.monotonic() - self.reached_since >= self.reached_stable_sec

    def _fault_hold(self, reason: str) -> None:
        if self.current_pose is not None and self.current_height_cm is not None:
            self.active_target = Target(
                self.current_pose.x,
                self.current_pose.y,
                self.current_height_cm if self.terminal_behavior == "guided_land_handoff"
                else max(self.current_height_cm, self.landing_height_cm),
                self.current_pose.yaw_deg,
            )
        self._enter_state("FAULT_HOLD", reason)

    def _task_guard_reason(self):
        if not self.task_guard_enabled or self.state == 'FAULT_HOLD':
            return None
        assert self.current_pose is not None and self.current_height_cm is not None
        assert self.a_pose is not None
        reason = envelope_reason(self.current_pose.x, self.current_pose.y,
                                 self.current_height_cm, self.a_pose.x, self.a_pose.y,
                                 self.task_radius_cm, self.task_max_height_cm)
        return reason or stage_reason(
            self.state, time.monotonic()-self.state_entered_monotonic,
            self.takeoff_timeout_sec, self.travel_timeout_sec,
            self.hold_timeout_sec, self.descent_timeout_sec, self.terminal_behavior)

    def _timer_callback(self) -> None:
        self._update_pose()

        if self.state not in self.ACTIVE_STATES:
            return
        if ((self.require_rc_offboard or self.flight_session_seen_armed)
                and not self._rc_offboard_active()):
            return  # No route kick/target while disarmed, taken over or stale.

        if self.route_kick_remaining > 0:
            msg = UInt8()
            msg.data = self.route_choice
            self.route_pub.publish(msg)
            self.route_kick_remaining -= 1

        if not self._sensors_fresh():
            if self.state != "FAULT_HOLD":
                self._fault_hold("定位或高度数据超时；保持最后目标并等待飞控看门狗")
            return

        reason = self._task_guard_reason()
        if reason:
            self.route_kick_remaining = 0
            self._fault_hold(reason + '；取消后续路线，保持当前位置等待人工接管')
            self._publish_target()
            return

        self._publish_target()

        if self.state == "TAKEOFF" and self._target_reached_stably():
            assert self.b_target is not None
            self.active_target = self.b_target
            self._enter_state("GOTO_B", "达到巡航高度，飞向 B 点")
        elif self.state == "GOTO_B" and self._target_reached_stably():
            self._enter_state("HOLD_B", "到达 B 点，持续闭环悬停")
        elif self.state == "HOLD_B":
            if (
                self.terminal_behavior in {"land", "px4_land", "guided_land_handoff"}
                and time.monotonic() - self.state_entered_monotonic
                >= self.pre_descent_hold_sec
                and self._target_reached_stably()
            ):
                if self.terminal_behavior == "px4_land":
                    # Hand off at cruise height. Do not use the legacy external
                    # range/position controller for the near-ground descent.
                    self.mission_complete_pub.publish(Empty())
                    self.active_target = None
                    self._enter_state("FC_LANDING", "B 点稳定悬停后请求 PX4 原生 Land")
                    return
                assert self.b_target is not None
                if self.terminal_behavior == "guided_land_handoff":
                    assert self.ground_height_cm is not None
                    self.active_target = Target(
                        self.b_target.x,
                        self.b_target.y,
                        self.b_target.z,
                        self.b_target.yaw_deg,
                    )
                    self.last_descent_tick_monotonic = time.monotonic()
                    self._enter_state("DESCEND_B", "B 点限速下降至地面附近")
                else:
                    self.active_target = Target(
                        self.b_target.x,
                        self.b_target.y,
                        self.landing_height_cm,
                        self.b_target.yaw_deg,
                    )
                    self._enter_state("DESCEND_B", "B 点垂直下降")
        elif self.state == "DESCEND_B":
            if self.abort_descent_on_obstacle and self.obstacle_below:
                assert self.b_target is not None
                self.active_target = self.b_target
                self._enter_state("OBSTACLE_HOLD", "下方检测到障碍，返回巡航高度悬停")
            elif self.terminal_behavior == "guided_land_handoff":
                assert self.active_target is not None
                assert self.current_height_cm is not None
                assert self.ground_height_cm is not None
                now = time.monotonic()
                dt = now - self.last_descent_tick_monotonic
                self.last_descent_tick_monotonic = now
                self.active_target.z = next_descent_setpoint_cm(
                    self.active_target.z,
                    self.current_height_cm,
                    self.ground_height_cm + self.native_land_handoff_margin_cm,
                    dt,
                    self.active_descent_rate_cm_s,
                    self.active_descent_max_lead_cm,
                )
                if (
                    self.active_target.z
                    <= self.ground_height_cm + self.native_land_handoff_margin_cm
                    and self.current_height_cm
                    <= self.ground_height_cm + self.native_land_handoff_margin_cm + 1.0
                    and self._target_reached_stably(self.landing_height_tolerance_cm)
                ):
                    self.mission_complete_pub.publish(Empty())
                    self.active_target = None
                    self._enter_state("FC_LANDING", "近地目标稳定，交给 PX4 Land 接地锁桨")
            elif self._target_reached_stably(self.landing_height_tolerance_cm):
                # External position control stops at the handoff height.
                # The PX4 bridge requests native Land for final touchdown.
                self.mission_complete_pub.publish(Empty())
                self.active_target = None
                self._enter_state(
                    "FC_LANDING",
                    "已到近地交接高度，飞控接管并持续执行一键降落直至地面",
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointToPointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
