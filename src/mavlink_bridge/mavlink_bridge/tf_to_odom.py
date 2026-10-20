#!/usr/bin/env python3
"""TF -> nav_msgs/Odometry：把 2D 激光 SLAM 的位姿变成带速度的里程计。

原始激光扫描没有速度，必须靠位姿时间差分。台架实测（静止时位姿抖动约
1 mm/帧）表明直接对相邻帧差分会产生 20 cm/s 量级的假速度，所以这里做了
三层抗噪：

    1. 时间窗差分（默认 0.2 s），而不是相邻帧
    2. 一阶低通滤波
    3. 位移死区 + 速度跳变门限

输出按 ROS 约定：header.frame_id = 父坐标系，child_frame_id = 子坐标系，
twist 表达在子坐标系（机体）下。
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class TfToOdom(Node):
    def __init__(self) -> None:
        super().__init__("tf_to_odom")

        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("output_topic", "/odom_from_slam")
        self.declare_parameter("sample_rate", 50.0)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("velocity_window_s", 0.2)
        self.declare_parameter("deadband_mps", 0.05)
        self.declare_parameter("max_speed_mps", 1.0)
        self.declare_parameter("lowpass_alpha", 0.4)
        self.declare_parameter("max_tf_age_s", 0.3)

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.window = float(self.get_parameter("velocity_window_s").value)
        self.deadband = float(self.get_parameter("deadband_mps").value)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.alpha = min(1.0, max(0.01, float(self.get_parameter("lowpass_alpha").value)))
        self.max_tf_age = float(self.get_parameter("max_tf_age_s").value)
        sample_rate = max(5.0, float(self.get_parameter("sample_rate").value))
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.hist: deque[tuple[float, float, float, float]] = deque(maxlen=int(sample_rate * 3))
        self.v_filt = (0.0, 0.0)
        self.jump_count = 0
        self.static_count = 0

        self.pub = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 20
        )
        self.create_timer(1.0 / sample_rate, self._sample)
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f"tf_to_odom: {self.parent_frame} -> {self.child_frame} | "
            f"窗={self.window:.2f}s 死区={self.deadband * 100:.1f}cm/s "
            f"限速={self.max_speed * 100:.0f}cm/s α={self.alpha:.2f}"
        )

    # ------------------------------------------------------------------ 采样
    def _sample(self) -> None:
        try:
            tr = self.buf.lookup_transform(
                self.parent_frame, self.child_frame, rclpy.time.Time()
            )
        except Exception:  # noqa: BLE001
            return
        st = tr.header.stamp
        t = st.sec + st.nanosec * 1e-9
        if self.hist and t <= self.hist[-1][0]:
            return  # 同一帧或时间倒流
        q = tr.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.hist.append(
            (t, tr.transform.translation.x, tr.transform.translation.y, yaw)
        )

    # ------------------------------------------------------------------ 发布
    def _publish(self) -> None:
        if len(self.hist) < 2:
            return
        t_now, x_now, y_now, yaw_now = self.hist[-1]
        if (self.get_clock().now().nanoseconds * 1e-9 - t_now) > self.max_tf_age:
            return

        # 找窗口起点
        target = t_now - self.window
        older = None
        for s in self.hist:
            if s[0] >= target:
                older = s
                break
        if older is None:
            return

        t_old, x_old, y_old, yaw_old = older
        dt = t_now - t_old
        if dt < self.window * 0.5:
            return

        dx = x_now - x_old
        dy = y_now - y_old
        dist = math.hypot(dx, dy)
        vx_w = dx / dt
        vy_w = dy / dt
        yaw_rate = wrap_pi(yaw_now - yaw_old) / dt

        # 跳变门限：超过合理速度判为回环/重定位跳变，本周期丢弃并重置滤波
        if dist / dt > self.max_speed:
            self.jump_count += 1
            self.v_filt = (0.0, 0.0)
            self.get_logger().warn(
                f"位姿跳变 {dist / dt * 100:.0f}cm/s > 门限，丢弃（累计 {self.jump_count} 次）",
                throttle_duration_sec=1.0,
            )
            return

        # 死区：窗口内位移太小直接判静止，避免把抖动当速度
        if dist < self.deadband * dt:
            self.static_count += 1
            self.v_filt = (0.0, 0.0)
            vx_w = vy_w = yaw_rate = 0.0
        else:
            # 一阶低通
            self.v_filt = (
                self.alpha * vx_w + (1.0 - self.alpha) * self.v_filt[0],
                self.alpha * vy_w + (1.0 - self.alpha) * self.v_filt[1],
            )
            vx_w, vy_w = self.v_filt

        # 世界系速度 -> 机体系（ROS 约定：twist 在 child frame）
        cos_y = math.cos(yaw_now)
        sin_y = math.sin(yaw_now)
        vx_b = vx_w * cos_y + vy_w * sin_y
        vy_b = -vx_w * sin_y + vy_w * cos_y

        msg = Odometry()
        msg.header.stamp = rclpy.time.Time(seconds=int(t_now), nanoseconds=int((t_now % 1) * 1e9)).to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.pose.pose.position.x = x_now
        msg.pose.pose.position.y = y_now
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(yaw_now)
        msg.twist.twist.linear.x = vx_b
        msg.twist.twist.linear.y = vy_b
        msg.twist.twist.angular.z = yaw_rate
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TfToOdom()
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
