#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""盯 /tracked_pose：每秒打印一次位姿 + 这一秒里的变化量，并标出跳变。

用途：**地面静置时看 SLAM 稳不稳**（这是判断 SLAM 健康最直接的办法），
以及手推测试时看 EV 的标度对不对。

    python3 ~/px4_ws/tools/watch_pose.py
    # 同时看 TOF 高度（默认就带）
    python3 ~/px4_ws/tools/watch_pose.py --ros-args -p show_tof:=true

判读：
  * 静置时  Δxy 应该 < 1~2 cm、Δyaw < 0.5°，合计漂移越接近 0 越好
  * 出现 Δxy > 0.2m 或 Δyaw > 10° 的整秒  ->  SLAM 在跳，别飞
  * Ctrl-C 退出时给整段会话的累计漂移
"""

import _ros_env  # noqa: F401  —— 不 source 也能跑（见该文件注释）

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                 reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WatchPose(Node):
    def __init__(self):
        super().__init__('watch_pose')
        self.topic = self.declare_parameter('topic', '/tracked_pose').value
        self.show_tof = bool(self.declare_parameter('show_tof', True).value)
        self.rate_hz = float(self.declare_parameter('print_hz', 1.0).value)

        self.pose = None
        self.h = None
        self.create_subscription(PoseStamped, self.topic, self._on_pose, QOS)
        if self.show_tof:
            self.create_subscription(Float32, '/tofsense/height', self._on_h, QOS)

        self.last = None          # (t, x, y, z, yaw)
        self.n = 0
        self.worst_xy = 0.0
        self.worst_yaw = 0.0
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / max(0.2, self.rate_hz), self._tick)
        self.get_logger().warn('开始盯 %s（静置时 Δ 应接近 0；Ctrl-C 看汇总）' % self.topic)

    def _on_pose(self, m):
        self.pose = m
        self.n += 1

    def _on_h(self, m):
        self.h = float(m.data)

    def _tick(self):
        if self.pose is None:
            self.get_logger().warn('还没收到 %s' % self.topic)
            return
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        p = self.pose.pose.position
        yaw = math.degrees(yaw_of(self.pose.pose.orientation))

        dxy = dyaw = 0.0
        if self.last is not None:
            _, x0, y0, _, yaw0 = self.last
            dxy = math.hypot(p.x - x0, p.y - y0)
            dyaw = abs((yaw - yaw0 + 180.0) % 360.0 - 180.0)
            self.worst_xy = max(self.worst_xy, dxy)
            self.worst_yaw = max(self.worst_yaw, dyaw)

        flag = ''
        if dxy > 0.2:
            flag += '  <<< 位置跳变'
        if dyaw > 10.0:
            flag += '  <<< 航向跳变'

        tof = ('  ToF=%.3fm' % self.h) if (self.show_tof and self.h is not None) else ''
        self.get_logger().info(
            't=%6.1fs  x=%+.3f y=%+.3f z=%+.3f  yaw=%+7.2f°  |  Δxy=%.3fm Δyaw=%.2f°%s%s'
            % (t, p.x, p.y, p.z, yaw, dxy, dyaw, tof, flag))
        self.last = (t, p.x, p.y, p.z, yaw)

    def summary(self):
        self.get_logger().warn(
            '汇总：收到 %d 帧；单秒最大 Δxy=%.3fm、Δyaw=%.2f°（静置时应 <0.02m / <0.5°）'
            % (self.n, self.worst_xy, self.worst_yaw))


def main():
    rclpy.init()
    node = WatchPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        try:
            node.summary()
        except Exception:
            pass
        node.destroy_node()
        # 2026-10-10：SIGTERM/Ctrl-C 时 rclpy 已经关过一次上下文，再关会抛
        # "RCLError: failed to shutdown: rcl_shutdown already called" 刷一屏红字 →
        # 加个 rclpy.ok() 守卫（timeout/Ctrl-C 退出时不再有假报错）
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
