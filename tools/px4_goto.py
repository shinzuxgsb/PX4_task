#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定点导航：把飞机飞到指定点并保持（Offboard 本地 NED 位置控制）。

为什么不用 QGC 的 "Go to location"
----------------------------------
那条走的是 MAVLink VEHICLE_CMD_DO_REPOSITION，参数是**经纬度**。
你们没有 GPS（estimator_status_flags.cs_gps = false），所以用不了。
无 GPS 的"定点导航"只能走 Offboard 的本地位置设定值 —— 就是本脚本做的事。

用法（在香橙派上；前提：飞机上电、px4-ev-chain 在跑、check_ev.sh 第 5 节订阅者 >=1）
------------------------------------------------------------------
    # ① 先干跑：只订阅并打印，不发任何指令、不解锁（默认就是干跑）
    python3 ~/px4_ws/tools/px4_goto.py --ros-args -p x:=1.0 -p y:=0.5 -p alt:=1.2

    # ② 真跑：起飞到「北 1.0m / 东 0.5m / 高 1.2m」并一直保持
    python3 ~/px4_ws/tools/px4_goto.py --ros-args \
        -p dry_run:=false -p x:=1.0 -p y:=0.5 -p alt:=1.2 \
        -p auto_arm:=true -p auto_offboard:=true

    # Ctrl-C 退出后脚本停止发 Offboard 流，PX4 会触发 offboard 失效保护（默认切定点/降落）。

参数
----
    x, y        目标点水平坐标，NED（x=北, y=东；单位 m）
    alt         目标高度（正数向上，m）。给了 alt 就用它算 z
    z           NED 的 z（负数向上，m）。alt 与 z 都不给时默认 -1.0（即高 1 m）
    yaw_deg     目标航向（度，NED 从北顺时针）。不给 = 保持当前机头（yawspeed=0）
    rate        Offboard 流频率 Hz（默认 50；PX4 要求 > 2 Hz）
    start_delay 启动后等多久开始（s，默认 5，给你 Ctrl-C 的机会）
    auto_arm    true 才自动解锁（默认 false）
    auto_offboard true 才自动切 Offboard（默认 false）
    dry_run     默认 true —— 不发任何东西。必须显式 dry_run:=false 才会真控制
    hold_s      到达后保持多少秒，0 = 一直保持（默认 0）

⚠️ 真跑会真的让飞机起飞。确认桨叶周围无人、目标点在安全空间内再用。
"""

import _ros_env  # noqa: F401  —— 不 source 也能跑（见该文件注释）

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

CMD_ARM_DISARM = 400      # MAV_CMD_COMPONENT_ARM_DISARM
CMD_DO_SET_MODE = 176     # MAV_CMD_DO_SET_MODE
MAIN_MODE_OFFBOARD = 6.0  # px4_custom_mode.h: PX4_CUSTOM_MAIN_MODE_OFFBOARD

QOS_PUB = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
QOS_SUB = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)


class Px4Goto(Node):
    def __init__(self):
        super().__init__('px4_goto')

        self.x = float(self.declare_parameter('x', 0.0).value)
        self.y = float(self.declare_parameter('y', 0.0).value)
        alt = self.declare_parameter('alt', float('nan')).value
        z = self.declare_parameter('z', float('nan')).value
        if not math.isnan(float(alt)):
            self.z = -abs(float(alt))
        elif not math.isnan(float(z)):
            self.z = float(z)
        else:
            self.z = -1.0
        self.yaw_deg = float(self.declare_parameter('yaw_deg', float('nan')).value)

        self.rate = float(self.declare_parameter('rate', 50.0).value)
        self.start_delay = float(self.declare_parameter('start_delay', 5.0).value)
        self.auto_arm = bool(self.declare_parameter('auto_arm', False).value)
        self.auto_offboard = bool(self.declare_parameter('auto_offboard', False).value)
        self.dry_run = bool(self.declare_parameter('dry_run', True).value)
        self.hold_s = float(self.declare_parameter('hold_s', 0.0).value)

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', QOS_PUB)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', QOS_PUB)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', QOS_PUB)

        self.lpos = None
        self.status = None
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self._on_lpos, QOS_SUB)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self._on_status, QOS_SUB)

        self.t0 = self.get_clock().now()
        self.arm_sent = False
        self.offboard_sent = False
        self.reached_at = None
        self.last_log = self.t0

        yaw_txt = '保持当前机头' if math.isnan(self.yaw_deg) else '%.1f°' % self.yaw_deg
        self.get_logger().warn(
            '目标点（NED）：x=%.2f(N) y=%.2f(E) z=%.2f(D, 负=向上) | 航向 %s | %s'
            % (self.x, self.y, self.z, yaw_txt,
               '【干跑 dry_run，不发任何指令】' if self.dry_run else '【真跑，会真的起飞】'))

        self.create_timer(1.0 / max(1.0, self.rate), self._tick)

    # ---------- 回调 ----------
    def _on_lpos(self, m):
        self.lpos = m

    def _on_status(self, m):
        self.status = m

    # ---------- 工具 ----------
    def _now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def _command(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self._now_us()
        m.command = cmd
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.command_pub.publish(m)

    def _elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def _armed(self):
        return (self.status is not None
                and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _in_offboard(self):
        return (self.status is not None
                and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    # ---------- 主循环 ----------
    def _tick(self):
        if self.dry_run:
            self._log_progress(dry=True)
            return

        t = self._elapsed()

        # 1) 持续发 Offboard 流（必须先有流，PX4 才允许切 Offboard）
        ocm = OffboardControlMode()
        ocm.timestamp = self._now_us()
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self.offboard_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        nan = float('nan')
        sp.position = [self.x, self.y, self.z]
        sp.velocity = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        if math.isnan(self.yaw_deg):
            sp.yaw = nan
            sp.yawspeed = 0.0        # 保持当前机头
        else:
            sp.yaw = math.radians(self.yaw_deg)
            sp.yawspeed = nan
        self.setpoint_pub.publish(sp)

        # 2) 解锁 / 切 Offboard
        if t > self.start_delay:
            if self.auto_arm and not self.arm_sent:
                self._command(CMD_ARM_DISARM, 1.0, 0.0)
                self.arm_sent = True
                self.get_logger().warn('已发送解锁命令')
            if self.auto_offboard and self._armed() and not self.offboard_sent:
                self._command(CMD_DO_SET_MODE, 1.0, MAIN_MODE_OFFBOARD)
                self.offboard_sent = True
                self.get_logger().warn('已发送切换 Offboard 命令')

        # 3) 到点判断 + 保持
        if self.lpos is not None:
            d = math.sqrt((self.lpos.x - self.x) ** 2 + (self.lpos.y - self.y) ** 2)
            dh = abs(self.lpos.z - self.z)
            if d < 0.15 and dh < 0.10:
                if self.reached_at is None:
                    self.reached_at = self.get_clock().now()
                    self.get_logger().warn('已到达目标点，保持中…')
                elif self.hold_s > 0 and (self.get_clock().now() - self.reached_at).nanoseconds * 1e-9 > self.hold_s:
                    self.get_logger().warn('保持时间到，停止发流（PX4 将触发 offboard 失效保护）')
                    raise SystemExit(0)
            else:
                self.reached_at = None

        self._log_progress()

    def _log_progress(self, dry=False):
        now = self.get_clock().now()
        if (now - self.last_log).nanoseconds * 1e-9 < 1.0:
            return
        self.last_log = now
        if self.lpos is None:
            self.get_logger().warn('还没收到 /fmu/out/vehicle_local_position —— DDS 通了吗？')
            return
        d = math.sqrt((self.lpos.x - self.x) ** 2 + (self.lpos.y - self.y) ** 2)
        self.get_logger().info(
            '当前 NED(%.2f, %.2f, %.2f)  目标(%.2f, %.2f, %.2f)  水平差 %.2fm  %s%s'
            % (self.lpos.x, self.lpos.y, self.lpos.z, self.x, self.y, self.z, d,
               'armed' if self._armed() else 'disarmed',
               '  [OFFBOARD]' if self._in_offboard() else '') + ('  [dry-run]' if dry else ''))


def main():
    rclpy.init()
    node = Px4Goto()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # 2026-10-10：同 watch_pose.py —— 避免 SIGTERM/Ctrl-C 时重复 shutdown 刷红字
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
