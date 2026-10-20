#!/usr/bin/env python3
"""离线验证 hover_test.py 的控制律符号（不接 ROS、不接飞控，只在笔记本上跑）。

    python3 test_hover_control.py

重点检查水平环：世界系 PD + 按当前偏航转到机体系。符号写反会让飞机朝反方向飞，
所以这里把几种典型位姿/速度下的指令方向都断言一遍。
"""

from __future__ import annotations

import collections
import math
import os
import sys
import time
import types


def _stub(name, **attrs):
    mod = types.ModuleType(name)

    for key, value in attrs.items():
        setattr(mod, key, value)

    sys.modules[name] = mod
    return mod


class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Node:
    def __init__(self, *args, **kwargs):
        pass

    def create_publisher(self, *args, **kwargs):
        return _Msg()

    def create_subscription(self, *args, **kwargs):
        return _Msg()

    def create_timer(self, *args, **kwargs):
        return _Msg()


_stub("rclpy", init=lambda *a, **k: None, shutdown=lambda: None,
      spin_once=lambda *a, **k: None, ok=lambda: True)
_stub("rclpy.node", Node=_Node)
_stub("rclpy.qos", DurabilityPolicy=_Msg(TRANSIENT_LOCAL=1),
      ReliabilityPolicy=_Msg(RELIABLE=1), QoSProfile=lambda **k: None)
_stub("nav_msgs")
_stub("nav_msgs.msg", Odometry=_Msg)
_stub("std_msgs")
_stub("std_msgs.msg", Float32MultiArray=_Msg, Int16=_Msg, String=_Msg, UInt8=_Msg)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hover_test  # noqa: E402


def make_odom(x=0.0, y=0.0, yaw_deg=0.0, vx=0.0, vy=0.0):
    """vx/vy 是按 ROS 约定表达在机体（子）坐标系的速度，单位 m/s。"""
    yaw = math.radians(yaw_deg)

    return _Msg(
        pose=_Msg(pose=_Msg(
            position=_Msg(x=x, y=y, z=0.0),
            orientation=_Msg(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        )),
        twist=_Msg(twist=_Msg(linear=_Msg(x=vx, y=vy, z=0.0), angular=_Msg(z=0.0))),
    )


def make_node(odom, start=(0.0, 0.0, 0.0), height_cm=50.0, phase="HOVER", **over):
    """造一个只够 hold_velocity 用的假节点。"""
    node = hover_test.HoverTask.__new__(hover_test.HoverTask)
    args = types.SimpleNamespace(
        height_kp=0.8, height_kd=0.6, vz_max=20.0,
        height_ki=0.10, height_i_limit=8.0, height_i_band=15.0,
        xy_kp=1.5, xy_kd=0.8, xy_deadband_cm=1.5, vxy_max=30.0,
        xy_ki=0.30, xy_i_limit=15.0, xy_i_band=30.0,
        xy_kp_climb=2.5, xy_kd_climb=1.0,
        yaw_kp=1.5, hold_xy=True, kalman_free=True,
    )

    for key, value in over.items():
        setattr(args, key, value)

    node.args = args
    node.odom = odom
    node.phase = phase
    node.start_x, node.start_y, node.start_yaw = start
    node.height_cm = height_cm
    node.height_zero = 0.0
    node.i_x = node.i_y = 0.0
    node.h_i = 0.0
    node.height_hist = collections.deque([(time.time() - 0.3, height_cm),
                                          (time.time() - 0.15, height_cm),
                                          (time.time(), height_cm)], maxlen=40)
    return node


def approx(value, want, tol=0.6):
    return abs(value - want) <= tol


def main() -> int:
    checks = []

    # 1) 朝 +X 方向（偏航 0），飞机在起点前方 20cm → 应该往回（-X，机体 -x）
    vx, vy, _, _ = make_node(make_odom(x=0.20, y=0.0, yaw_deg=0.0)).hold_velocity(50.0, 0.05)
    checks.append(("偏航0 · 偏移+x20cm → 机体 -x", f"({vx:+.1f},{vy:+.1f})",
                   vx < -20.0 and approx(vy, 0.0)))

    # 2) 同一位置，但机头朝 +Y（偏航 +90°）：世界 -X 在机体里是 +y（左侧）
    vx, vy, _, _ = make_node(make_odom(x=0.20, y=0.0, yaw_deg=90.0)).hold_velocity(50.0, 0.05)
    checks.append(("偏航90 · 偏移+x20cm → 机体 +y", f"({vx:+.1f},{vy:+.1f})",
                   vy > 20.0 and approx(vx, 0.0)))

    # 3) 机头朝 +Y，世界 +Y 偏移在机体里是 -x（后退）
    vx, vy, _, _ = make_node(make_odom(x=0.0, y=0.20, yaw_deg=90.0)).hold_velocity(50.0, 0.05)
    checks.append(("偏航90 · 偏移+y20cm → 机体 -x", f"({vx:+.1f},{vy:+.1f})",
                   vx < -20.0 and approx(vy, 0.0)))

    # 4) 位置在起点但机体正以 +x 5cm/s 漂移 → 阻尼项应该给出反方向
    vx, vy, _, _ = make_node(make_odom(vx=0.05)).hold_velocity(50.0, 0.05)
    checks.append(("静止位置 + 漂移5cm/s → 反向指令", f"({vx:+.1f},{vy:+.1f})", vx < -2.0))

    # 5) 位置误差在死区内 → 只有阻尼项，没有 P 项
    vx, _, _, _ = make_node(make_odom(x=0.01)).hold_velocity(50.0, 0.05)
    checks.append(("死区 1cm < 1.5cm → 无 P 项输出", f"{vx:+.2f}", approx(vx, 0.0)))

    # 6) 大偏差时被 vxy_max 限幅
    vx, _, _, _ = make_node(make_odom(x=1.0)).hold_velocity(50.0, 0.05)
    checks.append(("偏移 1m → 限幅 30cm/s", f"{vx:+.2f}", approx(vx, -30.0)))

    # 7) 偏航误差 → 偏航角速度指令方向：实际偏航 +10°，起点偏航 0 → 应给负的角速度
    _, _, _, vyaw = make_node(make_odom(yaw_deg=10.0)).hold_velocity(50.0, 0.05)
    checks.append(("偏航超 +10° → 负角速度", f"{vyaw:+.2f}", vyaw < 0.0))

    # 8) 高度环：低于目标 → 上升；高于目标 → 下降
    low = make_node(make_odom(), height_cm=30.0)
    _, _, vz_low, _ = low.hold_velocity(50.0, 0.05)
    high = make_node(make_odom(), height_cm=80.0)
    _, _, vz_high, _ = high.hold_velocity(50.0, 0.05)
    checks.append(("高度30<50 → 上升", f"{vz_low:+.2f}", vz_low > 0.0))
    checks.append(("高度80>50 → 下降", f"{vz_high:+.2f}", vz_high < 0.0))

    # 9) kalman_free=False 时不做坐标变换（调试路径）
    vx, vy, _, _ = make_node(make_odom(x=0.20, yaw_deg=90.0), kalman_free=False).hold_velocity(50.0, 0.05)
    checks.append(("kalman_free=False · 世界系直给", f"({vx:+.1f},{vy:+.1f})",
                   vx < -20.0 and approx(vy, 0.0)))

    # 10) 水平积分：持续小偏差 → 指令随时间变大（用来顶掉恒定推力偏置）
    node = make_node(make_odom(x=0.05))
    v_first = node.hold_velocity(50.0, 0.05)[0]
    v_last = v_first

    for _ in range(60):                                   # 3 秒
        v_last = node.hold_velocity(50.0, 0.05)[0]

    checks.append(("水平 I：持续小偏差 → 指令变大", f"{v_first:+.1f}→{v_last:+.1f}",
                   v_last < v_first - 2.0))

    # 11) 水平积分的抗饱和：偏差超出作用带 → 不累积
    node = make_node(make_odom(x=0.50))                   # 50cm > 30cm 作用带

    for _ in range(60):
        node.hold_velocity(50.0, 0.05)

    checks.append(("水平 I：偏差超带外 → 积分清零", f"i_x={node.i_x:.2f}", abs(node.i_x) < 1e-9))

    # 12) 起飞段增益更硬
    v_hover = make_node(make_odom(x=0.10), phase="HOVER").hold_velocity(50.0, 0.05)[0]
    v_climb = make_node(make_odom(x=0.10), phase="TAKEOFF").hold_velocity(50.0, 0.05)[0]
    checks.append(("起飞段增益更硬", f"{v_hover:+.1f} vs {v_climb:+.1f}",
                   v_climb < v_hover - 5.0))

    # 13) 高度积分：持续 5cm 静差 → 上升指令变大
    node = make_node(make_odom(), height_cm=45.0)         # 目标 50 → 差 5cm
    vz_first = node.hold_velocity(50.0, 0.05)[2]
    vz_last = vz_first

    for _ in range(100):                                  # 5 秒
        vz_last = node.hold_velocity(50.0, 0.05)[2]

    checks.append(("高度 I：持续静差 → 上升指令变大", f"{vz_first:+.1f}→{vz_last:+.1f}",
                   vz_last > vz_first + 1.0))

    bad = 0

    for name, value, ok in checks:
        print(f"{'PASS' if ok else 'FAIL'}  {name}   (值={value})")
        bad += 0 if ok else 1

    print(f"\n{len(checks) - bad}/{len(checks)} 通过")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
