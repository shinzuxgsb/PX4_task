"""Publish continuous Cartographer odometry to PX4 as external vision data."""

import math
import time
from typing import Optional, Tuple

import rclpy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener

from .frames import (
    flu_position_to_frd,
    flu_quaternion_to_frd,
    normalize_angle_rad,
    quaternion_to_yaw,
)
from .lidar_velocity import (LidarVelocityEstimator, frd_world_velocity_to_body,
                             frd_world_velocity_to_ned, pose_at_sample)


PX4_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CartographerOdometryBridge(Node):
    """Convert odom->base_link FLU pose into PX4 world-fixed FRD odometry."""

    def __init__(self) -> None:
        super().__init__("cartographer_odometry_bridge")
        self.enabled = bool(self.declare_parameter("enabled", False).value)
        self.publish_to_px4 = bool(self.declare_parameter("publish_to_px4", True).value)
        self.body_velocity = bool(self.declare_parameter("body_velocity", False).value)
        self.horizontal_only = bool(self.declare_parameter("horizontal_only", False).value)
        self.horizontal_firmware_confirmed = bool(self.declare_parameter("horizontal_firmware_confirmed", False).value)
        if self.horizontal_only and self.publish_to_px4 and not self.horizontal_firmware_confirmed:
            raise ValueError("Horizontal-only output requires tested custom PX4 firmware; preview is safe")
        self.publish_yaw = bool(self.declare_parameter("publish_yaw", True).value)
        self.scan_timeout_sec = float(self.declare_parameter("scan_timeout_sec", 0.3).value)
        window = float(self.declare_parameter("velocity_window_sec", 0.25).value)
        max_speed = float(self.declare_parameter("max_velocity_mps", 1.5).value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.body_frame = str(self.declare_parameter("body_frame", "base_link").value)
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 30.0).value
        )
        self.pose_timeout_sec = float(
            self.declare_parameter("pose_timeout_sec", 0.25).value
        )
        self.startup_timeout_sec = float(
            self.declare_parameter("startup_timeout_sec", 45.0).value
        )
        self.unhealthy_timeout_sec = float(
            self.declare_parameter("unhealthy_timeout_sec", 10.0).value
        )
        self.publish_velocity = bool(
            self.declare_parameter("publish_velocity", True).value
        )
        # ==== 2026-10-13 本机改动（相对参考件的**唯一**偏离）====
        # 参考件的 EV 只给水平位置+航向，垂直分量交给飞控自带测距（派 ttyS1→MAVLink）。
        # 本机不接那根线（用户 2026-10-13 决定"保留我们的 EV 高度"），因此把本机定高器
        # /tofsense/height（Float32, m）并进 EV 的垂直分量：
        #   z_frd = -height_m   （FRD/NED 向下为正）
        #   高度新鲜 → position_variance[2] = height_sigma_m²
        #   高度超时/非法 → z=NaN 且方差=100（明确"不提供"，绝不谎报 0）
        # 对应飞控参数维持我们原来的：EKF2_EV_CTRL=15（含垂直位）、EKF2_HGT_REF=3(Vision)。
        self.use_height = bool(self.declare_parameter("use_height", True).value)
        self.height_topic = str(
            self.declare_parameter("height_topic", "/tofsense/height").value
        )
        self.height_timeout_sec = float(
            self.declare_parameter("height_timeout_sec", 0.25).value
        )
        self.height_sigma_m = float(
            self.declare_parameter("height_sigma_m", 0.15).value
        )
        # 2026-10-13 本机改动：位姿帧可选 ned|frd（默认 ned）。
        # 为什么默认改成 NED：PX4 v1.15.4 `EKF/ev_yaw_control.cpp` 里
        #   NED (L141-155)：把偏航重置到 EV 并 **`yaw_align = true`**；
        #   FRD (L157-167)：**显式 `yaw_align = false`**（"用 EV 自己的任意航向基准"）。
        # ⇒ 用 FRD 时 `cs_yaw_align=false`、`heading_good_for_control=false`
        #   （EKF 的偏航不被认为是"可用于控制的航向"）；NED 才是我们旧 EV 桥
        #   验证过能飞的约定（位置 (y,x) 映射 + yaw = pi/2 - ros_yaw）。
        # 想回到参考件原样：-p pose_frame:=frd
        self.pose_frame_mode = str(
            self.declare_parameter("pose_frame", "ned").value
        ).lower()
        if self.pose_frame_mode not in ("ned", "frd"):
            raise ValueError("pose_frame must be 'ned' or 'frd'")
        self.velocity_filter_alpha = float(
            self.declare_parameter("velocity_filter_alpha", 0.35).value
        )
        self.xy_stddev_m = float(self.declare_parameter("xy_stddev_m", 0.08).value)
        self.yaw_stddev_rad = float(
            self.declare_parameter("yaw_stddev_rad", math.radians(8.0)).value
        )
        self.velocity_stddev_mps = float(
            self.declare_parameter("velocity_stddev_mps", 0.15).value
        )
        self.reset_jump_m = float(
            self.declare_parameter("reset_jump_m", 0.75).value
        )
        self.reset_jump_yaw_rad = float(
            self.declare_parameter("reset_jump_yaw_rad", math.radians(45.0)).value
        )
        # ==== 2026-10-14 本机改动：数据短时异常时"沿用上一次正常数据" ====
        # 动机（16:23 炸机复盘）：SLAM/TOF 数据一异常，EV 就整帧停发；而本机 EV 是
        # 飞控唯一的位置+高度来源，瞬间失去参考就是炸机。
        # 现在的策略（有界，仍然让真故障能被飞控察觉）：
        #   · 异常 ≤ hold_extrapolate_sec：用上一次正常速度外推
        #   · 之后到 hold_max_sec：冻结上一次正常位姿，方差随缺失时间增大
        #     （PX4 EKF2 会用 position/orientation/velocity_variance 作为量测协方差，
        #      见 ekf2/EKF/ev_pos_control.cpp: pos_cov = diag(ev_sample.position_var)）
        #   · 超过 hold_max_sec：仍然停发，由飞控侧 COM_POSCTL_NAVL / EKF 失效动作兜底
        #   · 高度同理：TOF 单点跳变先丢弃并沿用上一次正常高度（见 _height_callback）
        # 注意：quality 必须保持 100 —— 低于 EKF2_EV_QUAL_MIN 会让飞控永久停掉 EV
        # 高度融合（见下方 msg.quality 注释），"可信度下降"要用方差表达。
        self.hold_last_good = bool(
            self.declare_parameter("hold_last_good", True).value
        )
        self.hold_extrapolate_sec = float(
            self.declare_parameter("hold_extrapolate_sec", 0.4).value
        )
        self.hold_max_sec = float(
            self.declare_parameter("hold_max_sec", 2.0).value
        )
        self.hold_sigma_rate_mps = float(
            self.declare_parameter("hold_sigma_rate_mps", 0.4).value
        )
        self.height_hold_max_sec = float(
            self.declare_parameter("height_hold_max_sec", 1.0).value
        )
        self.height_jump_max_m = float(
            self.declare_parameter("height_jump_max_m", 0.5).value
        )
        self.height_jump_confirm_n = int(
            self.declare_parameter("height_jump_confirm_n", 5).value
        )

        if (self.publish_rate_hz <= 0.0 or self.pose_timeout_sec <= 0.0
                or self.startup_timeout_sec <= 0.0 or self.unhealthy_timeout_sec <= 0.0):
            raise ValueError("publish rate and timeout must be positive")
        if (self.hold_extrapolate_sec < 0.0 or self.hold_max_sec < 0.0
                or self.hold_sigma_rate_mps < 0.0 or self.height_hold_max_sec < 0.0
                or self.height_jump_max_m <= 0.0 or self.height_jump_confirm_n < 1):
            raise ValueError("hold parameters must be non-negative (jump threshold positive)")
        if not 0.0 <= self.velocity_filter_alpha <= 1.0:
            raise ValueError("velocity_filter_alpha must be within [0, 1]")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.velocity_estimator = LidarVelocityEstimator(
            window_sec=window, max_speed_mps=max_speed,
            jump_m=self.reset_jump_m, jump_yaw_rad=self.reset_jump_yaw_rad)
        self.last_scan_monotonic = 0.0
        self.px4_heading = None
        self.heading_reset_counter = None
        self.last_heading_monotonic = 0.0
        self.height_m: Optional[float] = None
        self.last_height_monotonic = 0.0
        self.last_good_height_m: Optional[float] = None
        self.height_candidate_m: Optional[float] = None
        self.height_candidate_count = 0
        self.last_good_pose_monotonic: Optional[float] = None
        self.last_good_fields: Optional[dict] = None
        self.hold_start_monotonic: Optional[float] = None
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                                 self._heading_callback, PX4_QOS)
        self.create_subscription(LaserScan, "/scan", self._scan_callback,
                                 qos_profile_sensor_data)
        if self.use_height:
            self.create_subscription(Float32, self.height_topic,
                                     self._height_callback, 10)
        output_topic = ("/fmu/in/vehicle_visual_odometry" if self.publish_to_px4
                        else "/px4_bridge/visual_odometry_preview")
        self.publisher = self.create_publisher(
            VehicleOdometry, output_topic, PX4_QOS
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, "/px4_bridge/odometry_state", state_qos
        )

        self.previous_position: Optional[Tuple[float, float]] = None
        self.previous_yaw: Optional[float] = None
        self.previous_sample_time = 0.0
        self.filtered_velocity = (0.0, 0.0)
        self.reset_counter = 0
        self.last_warning_monotonic = 0.0
        self.last_state = ""
        self.startup_monotonic = time.monotonic()
        self.last_publishing_monotonic: Optional[float] = None

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)
        self._publish_state("DISABLED" if not self.enabled else "WAITING_FOR_ODOM")
        enabled_text = "enabled" if self.enabled else "disabled"
        self.get_logger().info(
            f"Cartographer odometry bridge {enabled_text}; "
            f"source={self.odom_frame}->{self.body_frame}"
        )

    def _publish_state(self, value: str) -> None:
        now = time.monotonic()
        # "沿用上一次正常数据"也算"还在发"，否则看门狗会误判成链路故障（见下面 raise）
        if value == "PUBLISHING" or value.startswith("HOLD_LAST_GOOD"):
            self.last_publishing_monotonic = now
        elif self.enabled:
            reference = (self.last_publishing_monotonic if self.last_publishing_monotonic is not None
                         else self.startup_monotonic)
            limit = (self.unhealthy_timeout_sec if self.last_publishing_monotonic is not None
                     else self.startup_timeout_sec)
            if now - reference > limit:
                raise RuntimeError(
                    f"Lidar odometry unhealthy for {now - reference:.1f}s: {value}"
                )
        if value == self.last_state:
            return
        msg = String()
        msg.data = value
        self.state_pub.publish(msg)
        self.last_state = value

    def _scan_callback(self, _msg) -> None:
        self.last_scan_monotonic = time.monotonic()

    def _height_callback(self, msg: Float32) -> None:
        """本机改动：定高器读数（m）。

        非法值直接丢弃；**单点跳变**（16:23 那次出现 0.85 m→2.33 m）先不信，
        继续沿用上一次正常高度，除非连续 height_jump_confirm_n 个样本都在新电平上
        （≈90 ms，真实爬升/下降照样能跟上）。"""
        value = float(msg.data)
        if not math.isfinite(value) or value <= 0.0:
            return
        if (self.last_good_height_m is not None
                and abs(value - self.last_good_height_m) > self.height_jump_max_m):
            if (self.height_candidate_m is not None
                    and abs(value - self.height_candidate_m) <= self.height_jump_max_m):
                self.height_candidate_count += 1
            else:
                self.height_candidate_m = value
                self.height_candidate_count = 1
            if self.height_candidate_count < self.height_jump_confirm_n:
                self._warn_throttled(
                    f"Height sample rejected as jump: {value:.3f} m "
                    f"(last good {self.last_good_height_m:.3f} m) - holding last good")
                return
            self.get_logger().warning(
                f"Height step confirmed: {self.last_good_height_m:.3f} -> {value:.3f} m")
        self.height_candidate_m = None
        self.height_candidate_count = 0
        self.height_m = value
        self.last_good_height_m = value
        self.last_height_monotonic = time.monotonic()

    def _heading_callback(self, msg) -> None:
        # Read heading only for frame alignment, never feed back EKF velocity/height.
        if not math.isfinite(msg.heading):
            self.px4_heading = None
            return
        # BODY_FRD velocity uses Cartographer yaw, not PX4 heading. Preserve
        # its independent history across estimator selection/heading resets.
        # NED-aligned horizontal-only output does depend on PX4 heading.
        if (self.horizontal_only and self.heading_reset_counter is not None
                and self.heading_reset_counter != msg.heading_reset_counter):
            self.velocity_estimator.invalidate()
        self.heading_reset_counter = msg.heading_reset_counter
        self.px4_heading = float(msg.heading)
        self.last_heading_monotonic = time.monotonic()

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_monotonic >= 2.0:
            self.get_logger().warning(message)
            self.last_warning_monotonic = now

    def _height_for_output(self, now: float) -> Tuple[float, float]:
        """返回 (z_value, z_var)。

        · 读数新鲜 → 用它（FRD：向下为正）
        · 超时但在 height_hold_max_sec 内 → **沿用上一次正常高度**，方差随缺失增大
        · 再久 → NaN + 大方差（明确"本帧不提供"，绝不谎报 0）
        """
        if not self.use_height or self.height_m is None:
            return float("nan"), 100.0
        age = now - self.last_height_monotonic
        if 0.0 <= age <= self.height_timeout_sec:
            sigma = self.height_sigma_m
            return -float(self.height_m), float(sigma * sigma)
        if 0.0 <= age <= self.height_hold_max_sec:
            self._warn_throttled(
                f"Height stale ({age:.2f}s): holding last good {self.height_m:.3f} m")
            sigma = self.height_sigma_m + self.hold_sigma_rate_mps * age
            return -float(self.height_m), float(sigma * sigma)
        return float("nan"), 100.0

    def _publish_hold(self, reason: str) -> bool:
        """数据短时异常时沿用上一次正常位姿（有界），避免飞控瞬间失去唯一位置/高度来源。

        返回 True 表示已经发过一帧（调用方不要再发状态）；超过 hold_max_sec 或还没有
        任何"上一次正常数据"时返回 False，保持原来的 fail-closed 语义。
        """
        if not self.hold_last_good or self.last_good_fields is None:
            return False
        if self.last_good_pose_monotonic is None:
            return False
        now = time.monotonic()
        age = now - self.last_good_pose_monotonic
        if age < 0.0 or age > self.hold_max_sec:
            self.hold_start_monotonic = None
            return False
        fields = self.last_good_fields
        if self.hold_start_monotonic is None:
            self.hold_start_monotonic = now
            self.get_logger().warning(
                f"Data problem ({reason}) {age:.2f}s after last good sample: "
                f"holding last good odometry for up to {self.hold_max_sec:.1f}s")
        # 位置：短时用上一次正常速度外推，之后冻结；方差随信息缺失时间增大。
        dt = min(age, self.hold_extrapolate_sec)
        px = fields["px"] + (fields["vx"] * dt if math.isfinite(fields["vx"]) else 0.0)
        py = fields["py"] + (fields["vy"] * dt if math.isfinite(fields["vy"]) else 0.0)
        xy_sigma = fields["xy_stddev"] + self.hold_sigma_rate_mps * age
        yaw_sigma = fields["yaw_stddev"] + self.hold_sigma_rate_mps * age
        vel_sigma = math.sqrt(max(fields["vel_var"], 0.0)) + self.hold_sigma_rate_mps * age
        z_value, z_var = self._height_for_output(now)
        if not math.isfinite(z_value):
            z_value = fields["z"]
            z_var = max(fields["z_var"], 100.0)
        nan = float("nan")
        msg = VehicleOdometry()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.timestamp_sample = msg.timestamp
        msg.pose_frame = fields["pose_frame"]
        msg.position = [float(px), float(py), float(z_value)]
        msg.q = [float(value) for value in fields["q"]]
        msg.velocity_frame = fields["velocity_frame"]
        msg.velocity = ([float(fields["vx"]), float(fields["vy"]), nan]
                        if math.isfinite(fields["vx"]) and math.isfinite(fields["vy"])
                        else [nan, nan, nan])
        msg.angular_velocity = [nan, nan, nan]
        msg.position_variance = [xy_sigma * xy_sigma, xy_sigma * xy_sigma, float(z_var)]
        msg.orientation_variance = [100.0, 100.0, yaw_sigma * yaw_sigma]
        msg.velocity_variance = [vel_sigma * vel_sigma, vel_sigma * vel_sigma, 100.0]
        msg.reset_counter = fields["reset_counter"]
        msg.quality = 100        # 绝不低于 EKF2_EV_QUAL_MIN（见下方 msg.quality 注释）
        self.publisher.publish(msg)
        self._publish_state(f"HOLD_LAST_GOOD({reason},age={age:.2f}s)")
        return True

    def _timer_callback(self) -> None:
        if not self.enabled:
            return
        if time.monotonic() - self.last_scan_monotonic > self.scan_timeout_sec:
            self.velocity_estimator.invalidate()
            if not self._publish_hold("SCAN_STALE"):
                self._publish_state("SCAN_STALE")
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.body_frame, Time()
            )
        except TransformException as exc:
            self.velocity_estimator.invalidate()
            self._warn_throttled(
                f"TF {self.odom_frame}->{self.body_frame} unavailable: {exc}"
            )
            if not self._publish_hold("TF_UNAVAILABLE"):
                self._publish_state("STALE")
            return

        stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp_ns) / 1e9
        if age_sec < -0.1 or age_sec > self.pose_timeout_sec:
            self.velocity_estimator.invalidate()
            self._warn_throttled(f"Cartographer odometry is stale ({age_sec:.3f}s)")
            if not self._publish_hold("POSE_STALE"):
                self._publish_state("STALE")
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        px, py, _ = flu_position_to_frd(
            translation.x, translation.y, translation.z
        )
        ros_yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        q_frd = flu_quaternion_to_frd(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        # 2026-10-13 本机改动：NED 模式（默认）—— 位置 (y,x) 映射 + yaw = pi/2 - ros_yaw，
        # 与旧 px4_ev_bridge 的约定一致；FRD 模式保持参考件原样。
        if self.pose_frame_mode == "ned":
            px = float(translation.y)
            py = float(translation.x)
            yaw_ned = normalize_angle_rad(math.pi / 2.0 - ros_yaw)
            q_out = [math.cos(yaw_ned / 2.0), 0.0, 0.0, math.sin(yaw_ned / 2.0)]
            pose_frame_out = VehicleOdometry.POSE_FRAME_NED
            vel_frame_out = VehicleOdometry.VELOCITY_FRAME_NED
        else:
            q_out = q_frd
            pose_frame_out = VehicleOdometry.POSE_FRAME_FRD
            vel_frame_out = VehicleOdometry.VELOCITY_FRAME_FRD

        sample_time = stamp_ns / 1e9 if stamp_ns > 0 else now_ns / 1e9
        result = self.velocity_estimator.update(sample_time, px, py, ros_yaw)
        if result is None:
            if not self._publish_hold("VELOCITY_PENDING"):
                self._publish_state(self.velocity_estimator.state)
            return
        vx, vy, velocity_sample_time = result
        lidar_pose = pose_at_sample(self.velocity_estimator.samples, velocity_sample_time)
        if self.body_velocity and lidar_pose is None:
            self._publish_state("POSE_NOT_ALIGNED")
            return
        lidar_x, lidar_y, lidar_yaw = lidar_pose if lidar_pose is not None else (nan, nan, nan)
        if self.horizontal_only:
            if self.px4_heading is None or time.monotonic() - self.last_heading_monotonic > self.scan_timeout_sec:
                self._publish_state("WAITING_FOR_PX4_HEADING")
                return
            vx, vy = frd_world_velocity_to_ned(vx, vy, ros_yaw, self.px4_heading)
        elif self.body_velocity:
            vx, vy = frd_world_velocity_to_body(vx, vy, lidar_yaw)
            if self.publish_to_px4:
                self._publish_state("BODY_PREVIEW_ONLY")
                return

        nan = float("nan")
        msg = VehicleOdometry()
        msg.timestamp = now_ns // 1000
        msg.timestamp_sample = int((velocity_sample_time if self.body_velocity or self.horizontal_only else sample_time) * 1e6)
        msg.pose_frame = pose_frame_out
        # Cartographer is 2-D。本机改动：垂直分量并进本机定高器（见上面 use_height 段）。
        # BODY mode uses pose interpolated at the velocity window centre so
        # position, velocity and yaw share one timestamp and reset counter.
        # 本机改动：高度超时后先沿用上一次正常高度（有界），再久才是 NaN（不提供）
        monotonic_now = time.monotonic()
        z_value, z_var = self._height_for_output(monotonic_now)
        msg.position = ([float(lidar_x), float(lidar_y), z_value] if self.body_velocity
                        else ([nan, nan, nan] if self.horizontal_only
                              else [float(px), float(py), z_value]))
        if self.publish_yaw and self.body_velocity and not self.horizontal_only:
            # Cartographer estimates planar yaw in its arbitrary odom frame.
            # FRD yaw is clockwise; do not use PX4's heading as this input.
            msg.q = [math.cos(lidar_yaw / 2), 0.0, 0.0,
                     -math.sin(lidar_yaw / 2)]
        else:
            msg.q = ([float(value) for value in q_out] if self.publish_yaw and not self.horizontal_only
                     else [nan, nan, nan, nan])
        msg.velocity_frame = (VehicleOdometry.VELOCITY_FRAME_BODY_FRD if self.body_velocity
                              else vel_frame_out)
        if self.horizontal_only:
            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        if self.publish_velocity and math.isfinite(vx) and math.isfinite(vy):
            msg.velocity = [float(vx), float(vy), nan]
        else:
            msg.velocity = [nan, nan, nan]
        msg.angular_velocity = [nan, nan, nan]
        msg.position_variance = [
            self.xy_stddev_m**2,
            self.xy_stddev_m**2,
            z_var,
        ]
        msg.orientation_variance = [100.0, 100.0, self.yaw_stddev_rad**2]
        msg.velocity_variance = [
            self.velocity_stddev_mps**2,
            self.velocity_stddev_mps**2,
            100.0,
        ]
        msg.reset_counter = self.velocity_estimator.reset_counter
        # 2026-10-13 本机改动：参考件这里填 0（它的高度不走 EV，所以无所谓）。
        # 但 PX4 的 EV 融合有质量门（ev_control.cpp: quality_sufficient 要求
        # quality >= EKF2_EV_QUAL_MIN，默认 50），且 ev_height_control.cpp 在
        # "EV 重置 + quality 不足" 时会**停掉 EV 高度融合**且不再启动。
        # 本机方案的垂直分量靠 EV，所以必须给足质量（与我们旧 EV 桥一致：100）。
        msg.quality = 100
        self.publisher.publish(msg)
        # 记录"上一次正常数据"，供 _publish_hold 在短时异常时沿用
        self.last_good_pose_monotonic = monotonic_now
        self.last_good_fields = {
            "px": float(msg.position[0]),
            "py": float(msg.position[1]),
            "vx": float(msg.velocity[0]) if math.isfinite(msg.velocity[0]) else nan,
            "vy": float(msg.velocity[1]) if math.isfinite(msg.velocity[1]) else nan,
            "z": float(msg.position[2]) if math.isfinite(msg.position[2]) else nan,
            "z_var": float(msg.position_variance[2]),
            "q": [float(value) for value in msg.q],
            "pose_frame": msg.pose_frame,
            "velocity_frame": msg.velocity_frame,
            "xy_stddev": float(self.xy_stddev_m),
            "yaw_stddev": float(self.yaw_stddev_rad),
            "vel_var": float(msg.velocity_variance[0]),
            "reset_counter": int(msg.reset_counter),
        }
        self.hold_start_monotonic = None
        self._publish_state("PUBLISHING")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartographerOdometryBridge()
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
