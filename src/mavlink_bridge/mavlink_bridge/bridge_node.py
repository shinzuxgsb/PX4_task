#!/usr/bin/env python3
"""PX4 MAVLink <-> ROS 2 双向桥接节点（替代 uart_to_stm32）。

话题接口与原 uart_to_stm32_node 保持一致，因此下游
pid_control_pkg / activity_control_pkg 无需任何改动：

    发布  /height        std_msgs/Int16   高度（厘米，向上为正）
    发布  /is_st_ready   std_msgs/UInt8   飞控就绪标志（transient_local）
    发布  /mission_step  std_msgs/UInt8   当前航点序号
    订阅  <odom_topic>   nav_msgs/Odometry  SLAM 里程计 -> MAVLink ODOMETRY（外部定位注入）

上行能力（按开关启用）：
    - TIMESYNC 应答（喂 ODOMETRY 的前提，否则 PX4 会丢弃我们的时间戳）
    - ODOMETRY 外部定位注入（SLAM -> PX4 EKF）
    - HEARTBEAT（可选）

Phase 3 再补 /target_velocity -> SET_POSITION_TARGET_LOCAL_NED 与
COMMAND_LONG（解锁/起飞/降落）。
"""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, Int16, String, UInt8

from .jump_guard import classify_step

# ODOMETRY 的 msgid 是 331（>255），MAVLink v1 装不下，必须用 v2 方言。
# pymavlink 默认加载 v10 dialect，这里在导入前强制切到 v20。
os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover
    raise SystemExit(
        "缺少 pymavlink，请先执行: python3 -m pip install pymavlink"
    )

MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_STATE_ACTIVE = 4

# STATUSTEXT 的文本字段长度：MAVLink 2 用它把长消息切块，块长 < 50 表示这条消息结束
STATUSTEXT_CHUNK_LEN = 50
SEVERITY_NAMES = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERROR",
                  4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}


def quat_mul(a, b):
    """Hamilton product of two (w, x, y, z) quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ENU(世界) -> NED(世界) 的固定旋转：180° around (1,1,0)/sqrt(2)
Q_ENU_TO_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
# FLU(机体) -> FRD(机体) 的固定旋转：180° around x
Q_FLU_TO_FRD = (0.0, 1.0, 0.0, 0.0)


class MavlinkBridge(Node):
    def __init__(self) -> None:
        super().__init__("mavlink_bridge")

        # --- 链路参数 ---
        self.declare_parameter("port", "/dev/ttyS6")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("source_system", 255)
        self.declare_parameter("source_component", 191)
        self.declare_parameter("rx_timeout", 3.0)
        self.declare_parameter("reconnect_interval", 2.0)

        # --- 上行开关（Phase 1 关闭）---
        self.declare_parameter("send_heartbeat", False)
        self.declare_parameter("heartbeat_rate", 1.0)
        # 心跳类型：true=MAV_TYPE_GCS。PX4 只有在收到 GCS 类型心跳时才会把
        # gcs_connection_lost 清掉，也才会把"解锁被拒/预检失败"这类 STATUSTEXT
        # 提示正常送到这条链路上（否则飞控侧的原因在地面上完全看不见）。
        self.declare_parameter("heartbeat_type_gcs", True)

        # --- /height ---
        self.declare_parameter("height_topic", "/height")
        self.declare_parameter("height_source", "local_position_ned")
        self.declare_parameter("height_publish_rate", 20.0)
        self.declare_parameter("height_offset_cm", 0.0)

        # --- /is_st_ready ---
        self.declare_parameter("ready_topic", "/is_st_ready")
        self.declare_parameter("ready_mode", "armed_and_ekf")
        self.declare_parameter("ready_publish_rate", 1.0)
        self.declare_parameter("ekf_required_flags", 15)

        # --- /mission_step ---
        self.declare_parameter("mission_topic", "/mission_step")
        self.declare_parameter("publish_mission_step", True)

        # --- TIMESYNC 应答 ---
        self.declare_parameter("timesync_responder", True)

        # --- 外部定位注入（SLAM -> PX4 EKF）---
        self.declare_parameter("odom_uplink_enabled", False)
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("odom_send_rate", 30.0)
        self.declare_parameter("odom_world_frame", "enu")      # enu | ned
        self.declare_parameter("odom_body_frame", "flu")       # flu | frd
        self.declare_parameter("odom_velocity_frame", "body")  # body | world
        self.declare_parameter("odom_use_velocity", True)
        self.declare_parameter("odom_use_orientation", True)
        self.declare_parameter("odom_use_position", True)
        self.declare_parameter("odom_position_noise", 0.01)    # m^2
        self.declare_parameter("odom_velocity_noise", 0.01)    # (m/s)^2
        self.declare_parameter("odom_angle_noise", 0.01)       # rad^2
        # 2D 激光给不了高度：垂直方向的方差要设得极大，让 EKF 直接忽略这一路
        self.declare_parameter("odom_position_noise_z", 1.0e6)  # m^2
        self.declare_parameter("odom_velocity_noise_z", 1.0e6)  # (m/s)^2
        self.declare_parameter("odom_max_position_jump_m", 1.5)
        self.declare_parameter("odom_max_jump_speed_mps", 2.5)
        self.declare_parameter("odom_jump_min_m", 0.25)
        self.declare_parameter("odom_jump_confirm_frames", 2)
        self.declare_parameter("odom_max_data_age_s", 0.5)
        # 147 适配（2026-10-17）：跳变后"位姿稳定多久"才恢复/判定为重定位
        self.declare_parameter("odom_jump_hold_s", 2.0)
        self.declare_parameter("odom_min_send_interval_s", 0.0)

        # --- 测距上行（TOFSense -> MAVLink DISTANCE_SENSOR -> PX4 EKF）---
        self.declare_parameter("range_uplink_enabled", True)
        self.declare_parameter("range_topic", "/tofsense/range")
        self.declare_parameter("range_quality_topic", "/tofsense/quality")
        self.declare_parameter("range_send_rate", 25.0)
        self.declare_parameter("range_min_cm", 2)
        self.declare_parameter("range_max_cm", 400)
        self.declare_parameter("range_covariance_cm2", 4)
        self.declare_parameter("range_orientation", 25)   # 25 = PITCH_270（朝下）
        self.declare_parameter("range_sensor_id", 0)

        # --- Offboard 速度控制（把 /target_velocity 转成 SET_POSITION_TARGET_LOCAL_NED）---
        # 这是对飞控的**写操作**：只有在 PX4 处于 Offboard 模式时才会被执行，
        # 默认关着，确认无误后再打开。
        self.declare_parameter("offboard_enabled", False)
        self.declare_parameter("velocity_topic", "/target_velocity")
        self.declare_parameter("offboard_rate", 20.0)
        self.declare_parameter("offboard_timeout_s", 0.5)
        self.declare_parameter("max_horizontal_speed_cm_s", 24.0)
        self.declare_parameter("max_vertical_speed_cm_s", 20.0)
        self.declare_parameter("max_yaw_rate_deg_s", 6.0)

        # --- 飞控命令通道（解锁/切模式）---
        # 往 /px4_command 发字符串：arm / disarm / offboard / position / altitude / land
        self.declare_parameter("command_topic", "/px4_command")
        self.declare_parameter("command_enabled", True)

        self.declare_parameter("log_rate", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.rx_timeout = float(self.get_parameter("rx_timeout").value)
        self.reconnect_interval = float(self.get_parameter("reconnect_interval").value)
        self.send_heartbeat = bool(self.get_parameter("send_heartbeat").value)
        self.heartbeat_type_gcs = bool(self.get_parameter("heartbeat_type_gcs").value)
        self.height_source = str(self.get_parameter("height_source").value)
        self.height_offset_cm = float(self.get_parameter("height_offset_cm").value)
        self.ready_mode = str(self.get_parameter("ready_mode").value)
        self.ekf_required_flags = int(self.get_parameter("ekf_required_flags").value)
        self.publish_mission_step = bool(self.get_parameter("publish_mission_step").value)
        self.timesync_responder = bool(self.get_parameter("timesync_responder").value)
        self.odom_uplink_enabled = bool(self.get_parameter("odom_uplink_enabled").value)
        self.odom_send_rate = max(1.0, float(self.get_parameter("odom_send_rate").value))
        self.odom_world_frame = str(self.get_parameter("odom_world_frame").value)
        self.odom_body_frame = str(self.get_parameter("odom_body_frame").value)
        self.odom_velocity_frame = str(self.get_parameter("odom_velocity_frame").value)
        self.odom_use_velocity = bool(self.get_parameter("odom_use_velocity").value)
        self.odom_use_orientation = bool(self.get_parameter("odom_use_orientation").value)
        self.odom_use_position = bool(self.get_parameter("odom_use_position").value)
        self.odom_position_noise = float(self.get_parameter("odom_position_noise").value)
        self.odom_velocity_noise = float(self.get_parameter("odom_velocity_noise").value)
        self.odom_angle_noise = float(self.get_parameter("odom_angle_noise").value)
        self.odom_position_noise_z = float(self.get_parameter("odom_position_noise_z").value)
        self.odom_velocity_noise_z = float(self.get_parameter("odom_velocity_noise_z").value)
        self.odom_max_position_jump_m = float(self.get_parameter("odom_max_position_jump_m").value)
        self.odom_max_jump_speed_mps = float(
            self.get_parameter("odom_max_jump_speed_mps").value
        )
        self.odom_jump_min_m = float(self.get_parameter("odom_jump_min_m").value)
        self.odom_jump_confirm_frames = max(
            1, int(self.get_parameter("odom_jump_confirm_frames").value)
        )
        self.odom_max_data_age_s = float(self.get_parameter("odom_max_data_age_s").value)
        self.odom_jump_hold_s = float(self.get_parameter("odom_jump_hold_s").value)
        self.odom_min_send_interval_s = max(
            1.0 / self.odom_send_rate, float(self.get_parameter("odom_min_send_interval_s").value)
        )
        self.range_uplink_enabled = bool(self.get_parameter("range_uplink_enabled").value)
        self.range_send_interval_s = 1.0 / max(
            1.0, float(self.get_parameter("range_send_rate").value)
        )
        self.range_min_cm = int(self.get_parameter("range_min_cm").value)
        self.range_max_cm = int(self.get_parameter("range_max_cm").value)
        self.range_covariance_cm2 = int(self.get_parameter("range_covariance_cm2").value)
        self.range_orientation = int(self.get_parameter("range_orientation").value)
        self.range_sensor_id = int(self.get_parameter("range_sensor_id").value)
        self.offboard_enabled = bool(self.get_parameter("offboard_enabled").value)
        self.offboard_rate = max(1.0, float(self.get_parameter("offboard_rate").value))
        self.offboard_timeout_s = float(self.get_parameter("offboard_timeout_s").value)

        depth10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        ready_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.height_pub = self.create_publisher(
            Int16, self.get_parameter("height_topic").value, depth10
        )
        self.ready_pub = self.create_publisher(
            UInt8, self.get_parameter("ready_topic").value, ready_qos
        )
        self.mission_pub = self.create_publisher(
            UInt8, self.get_parameter("mission_topic").value, depth10
        )

        # --- 外部定位注入 ---
        self._odom_lock = threading.Lock()
        self._odom_count = 0
        self._odom_rejected_jump = 0
        self._odom_last_send = 0.0
        self._odom_last_pos = None
        self._odom_last_accept_recv = None
        self._odom_reject_streak = 0
        self._odom_reset_counter = 0
        # --- 147 适配（2026-10-17）：跳变 fail-closed 状态 ---
        self._odom_warm_done = False      # 启动预热是否完成（稳定后才开始发 EV）
        self._odom_warm_since = None
        self._odom_hold_active = False    # 跳变后是否处于"停发"状态
        self._odom_hold_ok_since = None   # 位姿回到原坐标系后的稳定计时
        self._odom_hold_away_since = None # 位姿停在新位置后的稳定计时
        self._odom_hold_away_logged = False
        self._odom_sub = None
        if self.odom_uplink_enabled:
            if not hasattr(mavutil.mavlink, "MAVLINK_MSG_ID_ODOMETRY"):
                self.get_logger().error(
                    "当前 pymavlink 方言不含 ODOMETRY（需要 MAVLink v2，"
                    "请确认 MAVLINK20=1）——里程计注入不会生效"
                )
            odom_topic = str(self.get_parameter("odom_topic").value)
            self._odom_sub = self.create_subscription(
                Odometry, odom_topic, self._odom_callback, 20
            )

        # --- 测距上行 ---
        self._range_lock = threading.Lock()
        self._range_m: float | None = None
        self._range_quality = 100
        self._range_stamp = 0.0
        self._range_last_send = 0.0
        self._range_sent = 0
        self._range_sub = None
        self._range_quality_sub = None
        if self.range_uplink_enabled:
            self._range_sub = self.create_subscription(
                Range,
                str(self.get_parameter("range_topic").value),
                self._range_callback,
                10,
            )
            self._range_quality_sub = self.create_subscription(
                UInt8,
                str(self.get_parameter("range_quality_topic").value),
                self._range_quality_callback,
                10,
            )

        # --- Offboard 速度控制 ---
        self._sp_lock = threading.Lock()
        self._setpoint = None
        self._setpoint_time = 0.0
        self._offboard_sent = 0
        self._px4_target = None
        self._vel_sub = None
        if self.offboard_enabled:
            self._vel_sub = self.create_subscription(
                Float32MultiArray,
                str(self.get_parameter("velocity_topic").value),
                self._velocity_callback,
                10,
            )
            self.create_timer(1.0 / self.offboard_rate, self._send_setpoint)

        # --- 命令通道 ---
        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self._cmd_sub = None
        if self.command_enabled:
            self._cmd_sub = self.create_subscription(
                String,
                str(self.get_parameter("command_topic").value),
                self._command_callback,
                10,
            )

        # --- 状态缓存（读线程写，定时器读）---
        self._lock = threading.Lock()
        self._log_lock = threading.Lock()
        self._statustext_pending = ""
        self._statustext_severity = 6
        self._state = {
            "height_m": None,
            "px": None,
            "py": None,
            "vx": None,
            "vy": None,
            "px4_sp": None,
            "px4_mode": None,
            "px4_sub_mode": None,
            "alt_rel_m": None,
            "alt_amsl_m": None,
            "ekf_flags": 0,
            "armed": False,
            "mission_seq": 0,
            "battery_v": None,
            "battery_pct": None,
            "last_rx": 0.0,
            "msg_count": 0,
            "bad_data": 0,
        }
        self._last_ready_pub: int | None = None
        self._last_mission_pub: int | None = None

        self._stop = threading.Event()
        self._conn = None

        self._reader = threading.Thread(target=self._read_loop, name="mavlink-rx", daemon=True)
        self._reader.start()

        hz = max(0.1, float(self.get_parameter("height_publish_rate").value))
        self.create_timer(1.0 / hz, self._publish_height)
        rz = max(0.1, float(self.get_parameter("ready_publish_rate").value))
        self.create_timer(1.0 / rz, self._publish_ready)
        lz = max(0.1, float(self.get_parameter("log_rate").value))
        self.create_timer(1.0 / lz, self._log_status)

        if self.send_heartbeat:
            self.create_timer(
                1.0 / max(0.2, float(self.get_parameter("heartbeat_rate").value)),
                self._send_heartbeat,
            )

        self.get_logger().info(
            f"mavlink_bridge 启动: {self.port} @ {self.baud} | "
            f"心跳={'开' if self.send_heartbeat else '关'} | "
            f"TIMESYNC应答={'开' if self.timesync_responder else '关'} | "
            f"里程计注入={'开: ' + str(self.get_parameter('odom_topic').value) if self.odom_uplink_enabled else '关'}"
        )

    # ------------------------------------------------------------------ 链路
    def _connect(self) -> bool:
        try:
            self._conn = mavutil.mavlink_connection(
                self.port,
                baud=self.baud,
                source_system=int(self.get_parameter("source_system").value),
                source_component=int(self.get_parameter("source_component").value),
                autoreconnect=False,
            )
            self.get_logger().info(f"串口已打开: {self.port} @ {self.baud}")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"打开 {self.port} 失败: {exc}")
            self._conn = None
            return False

    def _read_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            if self._conn is None and not self._connect():
                time.sleep(self.reconnect_interval)
                continue

            try:
                msg = self._conn.recv_match(blocking=True, timeout=1.0)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"串口异常，重连中: {exc}")
                self._close()
                time.sleep(self.reconnect_interval)
                continue

            if msg is None:
                continue
            self._handle(msg)

    def _close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001
                pass
            self._conn = None

    # -------------------------------------------------------------- 报文处理
    def _handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            with self._lock:
                self._state["bad_data"] += 1
            return

        if mtype == "TIMESYNC":
            with self._lock:
                self._state["last_rx"] = time.monotonic()
                self._state["msg_count"] += 1
            self._handle_timesync(msg)
            return

        if mtype == "STATUSTEXT":
            # 把飞控的提示原样打到日志里（网页控制站会显示这段日志流）。
            # 解锁被拒（"Arming denied: ..."）、预检失败（"Preflight Fail: ..."）
            # 都会在这里现身，否则这类原因只存在于飞控侧、陆地上完全看不见。
            self._handle_statustext(msg)
            with self._lock:
                self._state["last_rx"] = time.monotonic()
                self._state["msg_count"] += 1
            return

        if mtype in ("COMMAND_ACK", "COMMAND_LONG_ACK"):
            # 飞控对我们下发命令的应答。arm 被拒时这里会给出明确结果
            # （TEMPORARILY_REJECTED=被预检/模式要求挡下，DENIED=直接拒绝），
            # 具体原因由飞控侧同时发出的 STATUSTEXT 给出。
            command = int(getattr(msg, "command", -1))
            result = int(getattr(msg, "result", -1))
            mv = mavutil.mavlink
            result_names = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
                            4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
            command_names = {
                mv.MAV_CMD_COMPONENT_ARM_DISARM: "arm/disarm",
                mv.MAV_CMD_NAV_LAND: "land",
                mv.MAV_CMD_NAV_TAKEOFF: "takeoff",
                mv.MAV_CMD_DO_SET_MODE: "set_mode",
            }
            cmd_name = command_names.get(command, f"cmd{command}")
            res_name = result_names.get(result, f"result{result}")
            text = f"[PX4 ACK] {cmd_name} -> {res_name}"

            if result == 0:
                self.get_logger().info(text)
            else:
                self.get_logger().warn(f"{text}（命令未被执行；原因见飞控 [PX4 ...] 提示）")

            with self._lock:
                self._state["last_rx"] = time.monotonic()
                self._state["msg_count"] += 1
            return

        with self._lock:
            self._state["last_rx"] = time.monotonic()
            self._state["msg_count"] += 1

            if mtype == "HEARTBEAT":
                self._state["armed"] = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                self._state["px4_mode"] = (int(msg.custom_mode) >> 16) & 0xFF
                self._state["px4_sub_mode"] = (int(msg.custom_mode) >> 24) & 0xFF

            elif mtype == "LOCAL_POSITION_NED":
                # NED: z 向下为正 -> 高度取负
                self._state["height_m"] = -float(msg.z)
                self._state["px"] = float(msg.x)
                self._state["py"] = float(msg.y)
                self._state["vx"] = float(msg.vx)
                self._state["vy"] = float(msg.vy)

            elif mtype == "ALTITUDE":
                self._state["alt_rel_m"] = float(getattr(msg, "altitude_relative", math.nan))
                self._state["alt_amsl_m"] = float(getattr(msg, "altitude", math.nan))

            elif mtype == "ESTIMATOR_STATUS":
                self._state["ekf_flags"] = int(msg.flags)

            elif mtype == "MISSION_CURRENT":
                self._state["mission_seq"] = int(msg.seq)

            elif mtype == "POSITION_TARGET_LOCAL_NED":
                # PX4 回传的当前 setpoint，用来验证 Offboard 指令是否被采纳
                self._state["px4_sp"] = (float(msg.vx), float(msg.vy), float(msg.vz))

            elif mtype == "SYS_STATUS":
                if getattr(msg, "voltage_battery", 65535) != 65535:
                    self._state["battery_v"] = msg.voltage_battery / 1000.0
                self._state["battery_pct"] = int(getattr(msg, "battery_remaining", -1))

    def _handle_statustext(self, msg) -> None:
        """飞控提示 → 日志。MAVLink 2 会把长文本拆成多条 STATUSTEXT，这里先拼回一条再打印。"""
        text = msg.text

        if isinstance(text, (bytes, bytearray)):
            text = bytes(text).decode("utf-8", "replace")

        text = str(text).rstrip("\x00")
        severity = int(getattr(msg, "severity", 6))
        chunk_seq = int(getattr(msg, "chunk_seq", 0) or 0)

        if not text:
            return

        with self._log_lock:
            if chunk_seq == 0 and self._statustext_pending:
                # 上一条正好是 50 的整数倍、尾巴还挂着：先把它打完，避免两条粘一起
                self._emit_statustext(self._statustext_severity, self._statustext_pending)
                self._statustext_pending = ""

            self._statustext_pending += text
            self._statustext_severity = severity

            if len(text) >= STATUSTEXT_CHUNK_LEN:
                return          # 这一块是满的，后面还有

            full = self._statustext_pending
            self._statustext_pending = ""

        self._emit_statustext(severity, full)

    def _emit_statustext(self, severity: int, text: str) -> None:
        if not text:
            return

        sev_name = SEVERITY_NAMES.get(severity, f"S{severity}")
        message = f"[PX4 {sev_name}] {text}"

        if severity <= 4:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def flush_statustext(self) -> None:
        """把还没打完的飞控提示吐出来（用于长度正好是 50 整数倍、等不到后续块的消息）。"""
        with self._log_lock:
            pending = self._statustext_pending
            severity = self._statustext_severity
            self._statustext_pending = ""

        if pending:
            self._emit_statustext(severity, pending)

    def _handle_timesync(self, msg) -> None:
        """应答 PX4 的 TIMESYNC 请求（PX4 是请求方：tc1 == 0）。

        不回应答的话 PX4 的 Timesync 永远不收敛，而 `Timesync::sync_stamp()`
        在未收敛时会把时间戳直接换成「到达时刻」——注入的里程计就失去延迟补偿。
        """
        if not self.timesync_responder or self._conn is None:
            return
        try:
            if int(msg.tc1) == 0:
                # tc1 填本机时间，ts1 原样回显 PX4 的时间
                self._conn.mav.timesync_send(int(time.time_ns()), int(msg.ts1))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"TIMESYNC 应答失败: {exc}", throttle_duration_sec=5.0
            )

    # ------------------------------------------------------------------ 计算
    def _height_cm(self) -> int | None:
        with self._lock:
            if self.height_source == "altitude_relative":
                h = self._state["alt_rel_m"]
            elif self.height_source == "altitude_amsl":
                h = self._state["alt_amsl_m"]
            else:
                h = self._state["height_m"]
        if h is None or not math.isfinite(h):
            return None
        cm = int(round(h * 100.0 + self.height_offset_cm))
        return max(-32768, min(32767, cm))

    def _is_ready(self) -> tuple[int, str]:
        with self._lock:
            armed = self._state["armed"]
            flags = self._state["ekf_flags"]
        ekf_ok = (flags & self.ekf_required_flags) == self.ekf_required_flags

        if self.ready_mode == "always":
            return 1, "always"
        if self.ready_mode == "armed":
            return int(armed), f"armed={armed}"
        if self.ready_mode == "ekf":
            return int(ekf_ok), f"ekf_ok={ekf_ok} flags=0x{flags:x}"
        ready = armed and ekf_ok
        return int(ready), f"armed={armed} ekf_ok={ekf_ok} flags=0x{flags:x}"

    def _link_alive(self) -> bool:
        with self._lock:
            last = self._state["last_rx"]
        return last > 0 and (time.monotonic() - last) < self.rx_timeout

    # ------------------------------------------------------------------ 发布
    def _publish_height(self) -> None:
        cm = self._height_cm()
        if cm is None:
            return
        msg = Int16()
        msg.data = cm
        self.height_pub.publish(msg)

    def _publish_ready(self) -> None:
        ready, _ = self._is_ready()
        if not self._link_alive():
            ready = 0
        if self._last_ready_pub != ready or ready:
            msg = UInt8()
            msg.data = ready
            self.ready_pub.publish(msg)
            self._last_ready_pub = ready

        if self.publish_mission_step:
            with self._lock:
                seq = self._state["mission_seq"]
            if self._last_mission_pub != seq:
                m = UInt8()
                m.data = max(0, min(255, seq))
                self.mission_pub.publish(m)
                self._last_mission_pub = seq

    def _send_heartbeat(self) -> None:
        if self._conn is None:
            return
        try:
            self._conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS if self.heartbeat_type_gcs
                else mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                MAV_STATE_ACTIVE,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"发送心跳失败: {exc}")

    # ------------------------------------------------------ 外部定位注入
    def _command_callback(self, msg: String) -> None:
        """飞控命令通道：arm / disarm / offboard / position / altitude / land。"""
        if self._conn is None:
            self.get_logger().warn("链路未连接，命令忽略")
            return
        cmd = msg.data.strip().lower()
        mv = mavutil.mavlink
        try:
            if cmd in ("arm", "disarm"):
                self._conn.mav.command_long_send(
                    1, 1,
                    mv.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1.0 if cmd == "arm" else 0.0,
                    0, 0, 0, 0, 0, 0,
                )
                self.get_logger().warn(f"已发送 {cmd} 命令")
                return
            if cmd == "land":
                self._conn.mav.command_long_send(
                    1, 1, mv.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
                )
                self.get_logger().warn("已发送 land 命令")
                return

            main_modes = {
                "offboard": 6,
                "position": 3,
                "altitude": 2,
                "manual": 1,
            }
            if cmd in main_modes:
                base_mode = mv.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                custom_mode = main_modes[cmd] << 16
                self._conn.mav.set_mode_send(1, base_mode, custom_mode)
                self.get_logger().warn(
                    f"已请求切换到 PX4 模式: {cmd} (main_mode={main_modes[cmd]})"
                )
                return
            self.get_logger().warn(f"未知命令: {cmd}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"发送命令失败: {exc}")

    def _velocity_callback(self, msg: Float32MultiArray) -> None:
        """接收 pid_controller 的 /target_velocity: [vx,vy,vz cm/s, vyaw deg/s]（机体系 FLU）。"""
        if len(msg.data) < 4:
            self.get_logger().warn(
                "目标速度需要 4 个元素 [vx,vy,vz cm/s, vyaw deg/s]",
                throttle_duration_sec=5.0,
            )
            return
        with self._sp_lock:
            self._setpoint = (msg.data[0], msg.data[1], msg.data[2], msg.data[3])
            self._setpoint_time = time.monotonic()

    def _send_setpoint(self) -> None:
        """按固定频率把目标速度发给 PX4（Offboard setpoint）。

        坐标系用 MAV_FRAME_BODY_NED —— PX4 收到后会用当前姿态把机体系速度
        转到 NED，所以这里只需要做单位换算和 FLU->FRD 的轴向翻转。
        """
        if self._conn is None:
            return
        with self._sp_lock:
            sp = self._setpoint
            stamp = self._setpoint_time
        if sp is None:
            return
        if (time.monotonic() - stamp) > self.offboard_timeout_s:
            # 指令过期就停发，PX4 会因为收不到 setpoint 自动退出 Offboard
            return

        vx, vy, vz, vyaw = sp
        horizontal_limit = max(
            1.0, float(self.get_parameter("max_horizontal_speed_cm_s").value)
        )
        horizontal_speed = math.hypot(vx, vy)
        if horizontal_speed > horizontal_limit and horizontal_speed > 1e-9:
            scale = horizontal_limit / horizontal_speed
            vx *= scale
            vy *= scale
        vertical_limit = max(
            1.0, float(self.get_parameter("max_vertical_speed_cm_s").value)
        )
        yaw_limit = max(0.1, float(self.get_parameter("max_yaw_rate_deg_s").value))
        vz = max(-vertical_limit, min(vertical_limit, vz))
        vyaw = max(-yaw_limit, min(yaw_limit, vyaw))
        vx_m = vx / 100.0            # cm/s -> m/s
        vy_m = -vy / 100.0           # FLU(左) -> FRD(右)
        vz_m = -vz / 100.0           # FLU(上) -> FRD(下)
        yaw_rate = -math.radians(vyaw)   # 逆时针为正 -> 顺时针为正

        mv = mavutil.mavlink
        type_mask = (
            mv.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mv.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mv.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mv.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mv.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mv.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mv.POSITION_TARGET_TYPEMASK_FORCE_SET
            | mv.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        try:
            self._conn.mav.set_position_target_local_ned_send(
                0,                                  # time_boot_ms（PX4 不校验）
                1, 1,                               # target_system / component
                mv.MAV_FRAME_BODY_NED,              # 机体系，PX4 内部转到 NED
                type_mask,                          # 只用心速度和偏航角速度
                0.0, 0.0, 0.0,                      # 位置（忽略）
                vx_m, vy_m, vz_m,                   # 速度
                0.0, 0.0, 0.0,                      # 加速度（忽略）
                0.0, yaw_rate,                      # 偏航角（忽略）/ 偏航角速度
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"发送 Offboard setpoint 失败: {exc}", throttle_duration_sec=5.0
            )
            return
        self._offboard_sent += 1

    def _range_callback(self, msg: Range) -> None:
        """测距 -> MAVLink DISTANCE_SENSOR，PX4 会发布到 distance_sensor 话题供 EKF 使用。"""
        if self._conn is None:
            return
        now = time.monotonic()
        if (now - self._range_last_send) < self.range_send_interval_s:
            return

        dist_cm = int(round(msg.range * 100.0))
        if dist_cm <= 0:
            return
        dist_cm = max(self.range_min_cm, min(self.range_max_cm, dist_cm))
        with self._range_lock:
            signal_quality = max(1, min(100, int(self._range_quality)))

        try:
            self._conn.mav.distance_sensor_send(
                int(time.monotonic() * 1000) & 0xFFFFFFFF,
                self.range_min_cm,
                self.range_max_cm,
                dist_cm,
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                self.range_sensor_id,
                self.range_orientation,
                self.range_covariance_cm2,
                0.35,   # 水平 FOV (rad)
                0.35,   # 垂直 FOV (rad)
                (0.0, 0.0, 0.0, 0.0),   # quaternion（扩展字段，不用）
                signal_quality,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"发送 DISTANCE_SENSOR 失败: {exc}", throttle_duration_sec=2.0
            )
            return

        self._range_last_send = now
        with self._range_lock:
            self._range_m = msg.range
            self._range_stamp = now
            self._range_sent += 1

    def _range_quality_callback(self, msg: UInt8) -> None:
        with self._range_lock:
            self._range_quality = int(msg.data)

    def _odom_callback(self, msg: Odometry) -> None:
        """SLAM 里程计 -> MAVLink ODOMETRY，喂给 PX4 的 EKF。"""
        if self._conn is None:
            return

        now = time.monotonic()
        if (now - self._odom_last_send) < self.odom_min_send_interval_s:
            return

        stamp = msg.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1e-9
        now_epoch = time.time()
        age = now_epoch - stamp_s
        clock_skew = abs(age) > 1.0
        if age > self.odom_max_data_age_s and not clock_skew:
            self.get_logger().warn(
                f"里程计数据过期 {age:.2f}s，丢弃", throttle_duration_sec=2.0
            )
            return
        if clock_skew:
            self.get_logger().warn(
                f"里程计时间戳偏差 {age:+.2f}s（疑似时钟步进），按当前时间发送",
                throttle_duration_sec=2.0,
            )

        p = msg.pose.pose.position
        q_msg = msg.pose.pose.orientation
        lin = msg.twist.twist.linear

        # 1) 位置：世界系 -> NED
        #    enu      : 全局 ENU（东-北-天），如 FAST-LIO / Cartographer 的 map 系
        #    flu      : 机体对齐的局部 FLU（前-左-上），即 Cartographer 的 odom 系
        #    ned      : 已经是 NED，直通
        if self.odom_world_frame == "enu":
            px, py, pz = p.y, p.x, -p.z
        elif self.odom_world_frame == "flu":
            px, py, pz = p.x, -p.y, -p.z
        else:
            px, py, pz = p.x, p.y, p.z

        # 2) 姿态：q_out = q_world修正 ⊗ q_in ⊗ q_body修正
        q_in = quat_normalize((q_msg.w, q_msg.x, q_msg.y, q_msg.z))
        q_world_fix = (1.0, 0.0, 0.0, 0.0)
        q_body_fix = (1.0, 0.0, 0.0, 0.0)
        if self.odom_world_frame == "enu":
            q_world_fix = Q_ENU_TO_NED        # 世界系 ENU -> NED
        elif self.odom_world_frame == "flu":
            q_world_fix = Q_FLU_TO_FRD        # 世界系 FLU -> FRD（与机体同一套修正）
        if self.odom_body_frame == "flu":
            q_body_fix = Q_FLU_TO_FRD         # 机体系 FLU -> FRD
        q_out = quat_normalize(quat_mul(q_world_fix, quat_mul(q_in, q_body_fix)))

        # 3) 速度：ROS 约定 twist 在机体系；PX4 的 child_frame_id 也是机体系
        R = quat_to_matrix(q_in)
        if self.odom_use_velocity:
            v_in = np.array([lin.x, lin.y, lin.z])
            v_world = R @ v_in if self.odom_velocity_frame == "body" else v_in
            if self.odom_world_frame == "enu":
                vx, vy, vz = v_world[1], v_world[0], -v_world[2]
            elif self.odom_world_frame == "flu":
                vx, vy, vz = v_world[0], -v_world[1], -v_world[2]
            else:
                vx, vy, vz = v_world[0], v_world[1], v_world[2]
        else:
            vx = vy = vz = float("nan")

        # 4) 跳变检测（回环/重定位会产生假的巨大速度）
        # ------------------------------------------------------------------
        # 2026-10-20：原来的 0.5 m 单帧阈值在飞行中过于敏感。实测 0.51 m /
        # 0.43 s = 1.19 m/s，是正常飞行；原逻辑会直接 fail-closed 停发 EV，
        # PX4 随后 EV 超时 failsafe。现在改为：
        #   · 先用单帧硬限幅挡住米级/几百米级重定位；
        #   · 再用接收时间归一化速度判断，允许正常飞行在 odom 间隙里的位移；
        #   · 单帧可疑只丢这一帧，连续 odom_jump_confirm_frames 帧才进入
        #     fail-closed 停发，避免一次毛刺直接触发飞控保护。
        # 真正的重定位仍会停发 EV，让飞控走位置失效保护。
        # ------------------------------------------------------------------
        jump = 0.0
        bad_step = False
        if self._odom_last_pos is not None:
            jump = math.dist((px, py, pz), self._odom_last_pos)
            dt_s = (
                now - self._odom_last_accept_recv
                if self._odom_last_accept_recv is not None
                else 0.0
            )
            bad_step = classify_step(
                jump,
                dt_s,
                self.odom_max_position_jump_m,
                self.odom_jump_min_m,
                self.odom_max_jump_speed_mps,
            )

        if not self._odom_warm_done:
            if bad_step:
                self._odom_last_pos = (px, py, pz)
                self._odom_last_accept_recv = now
                self._odom_warm_since = None
                return
            if self._odom_warm_since is None:
                self._odom_warm_since = now
            if (now - self._odom_warm_since) < self.odom_jump_hold_s:
                self._odom_last_pos = (px, py, pz)
                self._odom_last_accept_recv = now
                return
            self._odom_warm_done = True
            self.get_logger().info(
                f"EV 预热完成（位置稳定 {self.odom_jump_hold_s:.1f}s），开始发送里程计"
            )

        if self._odom_hold_active:
            off = math.dist((px, py, pz), self._odom_last_pos)
            if off <= self.odom_max_position_jump_m:
                if self._odom_hold_ok_since is None:
                    self._odom_hold_ok_since = now
                if (now - self._odom_hold_ok_since) >= self.odom_jump_hold_s:
                    self._odom_hold_active = False
                    self._odom_hold_ok_since = None
                    self._odom_hold_away_since = None
                    self._odom_hold_away_logged = False
                    self.get_logger().warn(
                        "位姿已回到原坐标系，恢复发 EV（reset_counter 不变）"
                    )
                else:
                    return
            else:
                self._odom_hold_ok_since = None
                if self._odom_hold_away_since is None:
                    self._odom_hold_away_since = now
                if (now - self._odom_hold_away_since) >= self.odom_jump_hold_s:
                    if not self._odom_hold_away_logged:
                        self._odom_hold_away_logged = True
                        self.get_logger().error(
                            f"位姿稳定在新位置（偏离 {off:.2f}m）——判定为 SLAM 重定位，"
                            "EV 保持停发；请停链重启（切勿在飞行中重启）"
                        )
                return
        else:
            if bad_step:
                self._odom_rejected_jump += 1
                self._odom_reject_streak += 1
                if self._odom_reject_streak >= self.odom_jump_confirm_frames:
                    self._odom_hold_active = True
                    self._odom_hold_ok_since = None
                    self._odom_hold_away_since = None
                    self._odom_hold_away_logged = False
                    self.get_logger().error(
                        f"连续 {self._odom_reject_streak} 帧里程计跳变"
                        f"（最近 {jump:.2f}m），停发 EV；"
                        "请停链重启（切勿在飞行中重启）"
                    )
                else:
                    self.get_logger().warn(
                        f"里程计单帧跳变 {jump:.2f}m，丢弃该帧并等待下一帧确认",
                        throttle_duration_sec=1.0,
                    )
                return
            self._odom_reject_streak = 0

        self._odom_last_pos = (px, py, pz)
        self._odom_last_accept_recv = now

        # 5) 协方差（对角），ENU->NED 时位置/速度的 x/y 方差互换
        pos_var = self.odom_position_noise
        vel_var = self.odom_velocity_noise
        ang_var = self.odom_angle_noise
        pose_cov = [0.0] * 21
        pose_cov[0] = 0.0 if not self.odom_use_position else pos_var
        pose_cov[6] = pose_cov[0]
        # z：2D 激光没有高度观测量，给极大方差让 EKF 忽略
        pose_cov[11] = 0.0 if not self.odom_use_position else self.odom_position_noise_z
        if self.odom_use_orientation:
            pose_cov[15] = ang_var
            pose_cov[18] = ang_var
            pose_cov[20] = ang_var
        vel_cov = [0.0] * 21
        vel_cov[0] = 0.0 if not self.odom_use_velocity else vel_var
        vel_cov[6] = vel_cov[0]
        vel_cov[11] = 0.0 if not self.odom_use_velocity else self.odom_velocity_noise_z

        # 6) 时间戳：与 TIMESYNC 应答用同一个时钟（epoch 纳秒 -> 微秒）
        if abs(stamp_s - now_epoch) > 1.0:
            time_usec = int(now_epoch * 1e6)
        else:
            time_usec = int(stamp_s * 1e6) if stamp_s > 0 else int(now_epoch * 1e6)

        # 我们已在本地统一转成 NED/FRD，所以固定声明 LOCAL_NED，
        # PX4 的 LOCAL_NED 分支是「直接拷贝」，不做任何二次换算。
        frame_id = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        try:
            self._conn.mav.odometry_send(
                time_usec,
                frame_id,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,   # child_frame_id：速度在局部 NED 系
                float(px), float(py), float(pz),
                [float(c) for c in q_out],
                float(vx), float(vy), float(vz),
                float("nan"), float("nan"), float("nan"),   # 角速度不给
                pose_cov,
                vel_cov,
                self._odom_reset_counter,
                mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
                100,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"发送 ODOMETRY 失败: {exc}", throttle_duration_sec=2.0)
            return

        self._odom_last_send = now
        with self._odom_lock:
            self._odom_count += 1

    def _log_status(self) -> None:
        self.flush_statustext()      # 长度正好 50 整数倍的提示，靠这里兜底打出来
        with self._lock:
            st = dict(self._state)
        mode_names = {
            1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
            5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE",
        }
        mode = st.get("px4_mode")
        mode_txt = mode_names.get(mode, f"mode{mode}") if mode else "--"
        with self._odom_lock:
            odom_count = self._odom_count
        with self._range_lock:
            range_m = self._range_m
            range_quality = self._range_quality
            range_stamp = self._range_stamp
            range_sent = self._range_sent
        cm = self._height_cm()
        ready, reason = self._is_ready()
        alive = self._link_alive()
        text = (
            f"链路={'OK' if alive else '失联'} "
            f"PX4模式={mode_txt} 高度={cm if cm is not None else '--'}cm "
        )
        if st.get("px") is not None:
            text += f"PX4位置=({st['px']:+.2f},{st['py']:+.2f})m "
        text += (
            f"就绪={ready} ({reason}) "
            f"航点={st['mission_seq']} "
            f"电池={st['battery_v'] if st['battery_v'] is not None else '--'}V/"
            f"{st['battery_pct']}% "
            f"共{st['msg_count']}条 坏帧{st['bad_data']}"
        )
        if self.odom_uplink_enabled:
            text += f" | ODOMETRY 已发 {odom_count} 帧, 跳变丢弃 {self._odom_rejected_jump}"
        if self.range_uplink_enabled:
            range_age_ms = (time.monotonic() - range_stamp) * 1000.0 if range_stamp else None
            text += (
                f" | 测距={range_m * 100:.1f}cm q={range_quality} "
                f"age={range_age_ms:.0f}ms DISTANCE_SENSOR 已发 {range_sent} 帧"
                if range_m is not None
                else f" | 测距=-- (未收到 {self.get_parameter('range_topic').value})"
            )
        if self.offboard_enabled:
            sp = st.get("px4_sp")
            text += f" | Offboard 已发 {self._offboard_sent} 帧"
            if sp is not None:
                text += f" PX4当前setpoint=({sp[0]:+.2f},{sp[1]:+.2f},{sp[2]:+.2f})m/s"
        self.get_logger().info(text)

    def destroy_node(self) -> bool:
        self._stop.set()
        self._close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MavlinkBridge()
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
