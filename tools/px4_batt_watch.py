#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""低电看门狗：电压掉到阈值就命令 PX4 降落（NAV_LAND），防止"飞到没电直接掉下来"。

为什么要自己做一个
------------------
飞控现在的电池参数**没配容量**（实测 battery_status: capacity=0、full_charge_capacity_wh=0、
nominal_voltage=0、time_remaining_s=nan）。而 PX4 的低电告警/failsafe 是基于
`remaining`（剩余比例）判定的 —— 容量为 0 时 remaining 恒为 1.0，
**低电保护永远不会触发**，于是一路飞到断电。

本脚本只看**电压**（voltage_filtered_v，这个是真值），所以不依赖容量参数。

行为
----
  * 每 check_hz 检查一次；只在**已解锁**时动作
  * 电压低于 warn_cell_v       -> 打警告（不动作）
  * 电压低于 land_cell_v 持续 confirm_sec -> 发 MAV_CMD_NAV_LAND（PX4 进 Land 模式自动降落）
  * 低于 emergency_cell_v      -> 立刻发（不等 confirm）
  * max_flight_s>0 时，起飞后超过这个时长也强制降落（容量信息不可靠时的兜底）

用法（在香橙派上）
------------------
    python3 ~/px4_ws/tools/px4_batt_watch.py                 # 默认 dry_run=true，只打印
    python3 ~/px4_ws/tools/px4_batt_watch.py --ros-args \
        -p dry_run:=false -p cells:=4 -p land_cell_v:=3.5

⚠️ 阈值务必按你们的电池核对（默认 4S：warn 3.6V/cell、land 3.5V/cell、紧急 3.3V/cell）。
   带负载时电压会下垂，所以 land 阈值取得比较低；confirm_sec 用来滤掉瞬时下垂。
   建议先用 dry_run 飞一次，看日志里飞行中的电压曲线，再定阈值。
"""

import _ros_env  # noqa: F401  —— 不 source 也能跑（见该文件注释）

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import BatteryStatus, VehicleCommand, VehicleStatus

QOS_IN = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)
QOS_OUT = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)

CMD_NAV_LAND = 21


class BattWatch(Node):
    def __init__(self):
        super().__init__('px4_batt_watch')
        # 注意：命令行 `-p max_flight_s:=0` 会被 YAML 解析成 **INTEGER**，而 rclpy 声明成
        # DOUBLE 时类型不匹配会直接抛 InvalidParameterTypeException 把节点搞崩
        # （和 px4_takeoff_land 的 launch 里记的坑一模一样）。
        # 所以这里用 dynamic_typing，参数写 `0` 还是 `0.0` 都收得下。
        self.cells = int(self._num('cells', 4))
        self.warn_cell_v = self._num('warn_cell_v', 3.60)
        self.land_cell_v = self._num('land_cell_v', 3.50)
        self.emergency_cell_v = self._num('emergency_cell_v', 3.30)
        self.confirm_sec = self._num('confirm_sec', 3.0)
        self.max_flight_s = self._num('max_flight_s', 0.0)
        self.dry_run = self._flag('dry_run', True)
        self.check_hz = self._num('check_hz', 5.0)

        self.batt = None
        self.status = None
        self.bad_since = None
        self.armed_since = None
        self.land_sent = False
        self.last_log = 0.0
        self.last_resend = 0.0

        self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self._on_batt, QOS_IN)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_status, QOS_IN)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', QOS_OUT)

        self.get_logger().warn(
            '低电看门狗启动：%dS，警告 %.2fV/cell，降落 %.2fV/cell，紧急 %.2fV/cell，'
            '确认 %.1fs，%s'
            % (self.cells, self.warn_cell_v, self.land_cell_v, self.emergency_cell_v,
               self.confirm_sec, '【dry_run：只打印，不发指令】' if self.dry_run else '【真动作】'))
        self.create_timer(1.0 / max(1.0, self.check_hz), self._tick)

    def _num(self, name, default):
        d = ParameterDescriptor()
        d.dynamic_typing = True
        return float(self.declare_parameter(name, default, d).value)

    def _flag(self, name, default):
        d = ParameterDescriptor()
        d.dynamic_typing = True
        v = self.declare_parameter(name, default, d).value
        if isinstance(v, str):
            return v.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(v)

    def _on_batt(self, m):
        self.batt = m

    def _on_status(self, m):
        self.status = m

    def _armed(self):
        return (self.status is not None
                and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _send_land(self, why):
        if self.dry_run:
            self.get_logger().error('[dry_run] 本应发送 MAV_CMD_NAV_LAND（%s）' % why)
            return
        m = VehicleCommand()
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        m.command = CMD_NAV_LAND
        m.param1 = 0.0
        m.param2 = 0.0
        m.param4 = float('nan')
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)
        self.get_logger().error('已发送 NAV_LAND 强制降落：%s' % why)

    def _tick(self):
        now = time.time()
        if self.batt is None or self.status is None:
            return

        armed = self._armed()
        if not armed:
            self.armed_since = None
            self.bad_since = None
            self.land_sent = False
            # 未解锁时也每 10 秒报一次电压 —— 起飞前就能核对阈值对不对
            if self.batt.connected and now - self.last_log > 10.0:
                self.last_log = now
                self.get_logger().info(
                    '电压 %.2fV（%.3fV/cell）—— 未解锁，看门狗待命'
                    % (self.batt.voltage_filtered_v,
                       float(self.batt.voltage_filtered_v) / self.cells))
            return

        if self.armed_since is None:
            self.armed_since = now

        if not self.batt.connected:
            if now - self.last_log > 5.0:
                self.last_log = now
                self.get_logger().warn('电池未连接/无数据，看门狗不动作')
            return

        if self.batt.cell_count and self.batt.cell_count != self.cells:
            self.get_logger().error(
                '飞控报告 %d 芯，但本脚本按 %d 芯算阈值！改 -p cells:=%d 再跑'
                % (self.batt.cell_count, self.cells, self.batt.cell_count))
            return

        v_cell = float(self.batt.voltage_filtered_v) / self.cells

        # 电压趋势（每 5 秒一行，供事后定阈值）
        if now - self.last_log > 5.0:
            self.last_log = now
            self.get_logger().info(
                '电压 %.2fV（%.3fV/cell）电流 %.1fA 剩余(飞控算的，容量没配，不可信) %.2f'
                % (self.batt.voltage_filtered_v, v_cell, self.batt.current_filtered_a,
                   self.batt.remaining))

        # 兜底：飞行时长上限
        if self.max_flight_s > 0.0 and (now - self.armed_since) > self.max_flight_s:
            if not self.land_sent:
                self.land_sent = True
                self._send_land('飞行时长超过 %.0fs 的兜底上限' % self.max_flight_s)
            return

        if v_cell < self.emergency_cell_v:
            self._send_land('电压 %.3fV/cell < 紧急阈值 %.2f' % (v_cell, self.emergency_cell_v))
            self.land_sent = True
            return

        if v_cell < self.land_cell_v:
            if self.bad_since is None:
                self.bad_since = now
            elif (now - self.bad_since) >= self.confirm_sec and not self.land_sent:
                self.land_sent = True
                self._send_land('电压 %.3fV/cell 低于 %.2f 且持续 %.1fs'
                                % (v_cell, self.land_cell_v, self.confirm_sec))
        else:
            self.bad_since = None
            if v_cell < self.warn_cell_v and now - self.last_log > 5.0:
                self.get_logger().warn('电压偏低：%.3fV/cell（警告阈值 %.2f）'
                                       % (v_cell, self.warn_cell_v))


def main():
    rclpy.init()
    node = BattWatch()
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
    main()
