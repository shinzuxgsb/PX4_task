#!/usr/bin/env python3
"""ev_drift_check.py —— 看"漂"到底出在哪：SLAM(EV) vs 飞控(EKF) vs 摇杆。

用法（VS Code 终端里跑，Ctrl-C 停）：
    source /opt/ros/humble/setup.bash; source ~/px4_ws/install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml
    python3 ~/px4_ws/tools/ev_drift_check.py

判据（台架：桨拆掉，用手慢慢推机身 50 cm / 转 45°）：
  · 摇杆回中、机身真静止，而 EV 的 v1s 持续 >0.03 m/s  → **SLAM 在漂**（估计自己在动）
  · EV 稳、EKF 却在走                                  → 融合/延迟问题（标 EKF2_EV_DELAY）
  · 出现 !JUMP!（EV 一帧跳 >0.30 m）                    → **跳变型漂移**，最危险：飞控会猛冲去追
"""
import math
import time

import rclpy
from px4_msgs.msg import (ManualControlSetpoint, VehicleLocalPosition,
                          VehicleOdometry, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

PX4_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Monitor(Node):
    def __init__(self) -> None:
        super().__init__('ev_drift_check')
        self.ev = None
        self.ekf = None
        self.man = None
        self.st = None
        self.hist = []          # 最近 1 s 的 (monotonic, x, y)
        self.jump = 0.0
        self.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry',
                                 self.on_ev, PX4_QOS)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.on_ekf, PX4_QOS)
        self.create_subscription(ManualControlSetpoint, '/fmu/out/manual_control_setpoint',
                                 self.on_man, PX4_QOS)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.on_st, PX4_QOS)
        self.create_timer(0.25, self.tick)
        print('时间  | EV=SLAM 报的位置 / v1s=最近 1 秒的 EV 速度 | EKF=飞控融出来的位置'
              ' | 杆=遥控器输入 | 解锁/模式\n' + '-' * 100, flush=True)

    def on_ev(self, m) -> None:
        x, y = float(m.position[0]), float(m.position[1])
        now = time.monotonic()
        if self.ev is not None:
            d = math.hypot(x - self.ev[0], y - self.ev[1])
            if d > 0.30:
                self.jump = d
        self.hist.append((now, x, y))
        self.hist = [h for h in self.hist if now - h[0] <= 1.0]
        self.ev = (x, y, float(m.position[2]))

    def on_ekf(self, m) -> None:
        self.ekf = (float(m.x), float(m.y), bool(getattr(m, 'xy_valid', False)),
                    bool(getattr(m, 'z_valid', False)))

    def on_man(self, m) -> None:
        self.man = (float(getattr(m, 'roll', 0.0)), float(getattr(m, 'pitch', 0.0)),
                    float(getattr(m, 'yaw', 0.0)), float(getattr(m, 'throttle', 0.0)))

    def on_st(self, m) -> None:
        self.st = (int(m.arming_state), int(m.nav_state),
                   bool(getattr(m, 'pre_flight_checks_pass', False)))

    def tick(self) -> None:
        if self.ev is None:
            print('等 EV 数据…（EV 桥起来了吗）', flush=True)
            return
        now = time.monotonic()
        spd = float('nan')
        if len(self.hist) >= 2 and now - self.hist[0][0] > 0.2:
            t0, x0, y0 = self.hist[0]
            spd = math.hypot(self.ev[0] - x0, self.ev[1] - y0) / (now - t0)
        s = '%s EV=(%+.2f,%+.2f) z=%+.2f v1s=%.2f' % (
            time.strftime('%H:%M:%S'), self.ev[0], self.ev[1], self.ev[2], spd)
        if self.ekf is not None:
            s += ' | EKF=(%+.2f,%+.2f) xy/z=%d%d dEV-EKF=(%+.2f,%+.2f)' % (
                self.ekf[0], self.ekf[1], self.ekf[2], self.ekf[3],
                self.ev[0] - self.ekf[0], self.ev[1] - self.ekf[1])
        if self.man is not None:
            s += ' | 杆 r=%+.2f p=%+.2f y=%+.2f t=%+.2f' % self.man
        if self.st is not None:
            s += ' | %s nav=%d pfc=%d' % ('已解锁' if self.st[0] == 2 else '未解锁',
                                          self.st[1], self.st[2])
        if self.jump > 0.0:
            s += '   !JUMP %.2fm!' % self.jump
            self.jump = 0.0
        print(s, flush=True)


def main() -> None:
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
