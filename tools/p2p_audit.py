#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p2p_audit.py —— A→B 任务「到点判定 / 超时 / 包线」逐段对账（**只读**）

对应 CHANGELOG 2026-10-12「还没做」第 4 条：
    任务的"到点判定/超时/包线"用 /p_to_p/state 与实际飞行轨迹逐段对账。

做法：用**同一把尺子**（直接 import 任务节点的 geometry / task_guard / landing），
把同一时刻的**实际轨迹**（TF map→laser_link + /height + 飞控 local_position）记下来，
按段（TAKEOFF / GOTO_B / HOLD_B / DESCEND_B / FC_LANDING / 各 FAULT）算：
    目标 vs 实际 —— 三个误差 vs 三个容差 —— 持续时间 vs 该段超时 —— 包线(半径/高度)
并把「节点判定 vs 审计判定」不一致处直接标出：假到位 / 漏切 / 超时未生效 /
包线未生效 / 段内传感器超时 / 下降段目标回抬 / 下降超速 / 水平误差远超容差。

用法（**必须与链路用同一条 FastDDS 传输**，否则 TF 查不到 —— 本机已知的 SHM/UDP 坑）
    source /opt/ros/humble/setup.bash; source ~/px4_ws/install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml
    python3 ~/px4_ws/tools/p2p_audit.py                # 记录；Ctrl-C 结束并打印对账表
    python3 ~/px4_ws/tools/p2p_audit.py --self-test    # 不连 ROS，跑内置自检

输出（默认落 SD 卡 ~/flight_records/，**不要放 /tmp**，重启会丢）
    <out>/samples.csv   逐拍原始记录
    <out>/report.txt    逐段对账表 + 汇总

⚠️ 只订阅：不发布、不调服务、不改参数、不解锁。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

_PKG_SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", "src", "p_to_p_mission"))
if _PKG_SRC not in sys.path:
    sys.path.insert(0, _PKG_SRC)

try:
    from p_to_p_mission.geometry import normalize_angle_deg, target_error
    from p_to_p_mission.task_guard import envelope_reason, stage_reason
    from p_to_p_mission.landing import next_descent_setpoint_cm
    _CRITERIA_SOURCE = "p_to_p_mission（与节点同一份）"
except Exception:
    _CRITERIA_SOURCE = "内联等价实现（未能 import 任务节点）"

    def normalize_angle_deg(a: float) -> float:
        return (a + 180.0) % 360.0 - 180.0

    def target_error(cx, cy, cz, cyaw, tx, ty, tz, tyaw):
        return (math.hypot(tx - cx, ty - cy), abs(tz - cz),
                abs(normalize_angle_deg(tyaw - cyaw)))

    def envelope_reason(x, y, h, ox, oy, radius, max_h):
        if not all(math.isfinite(v) for v in (x, y, h, ox, oy)):
            return "nonfinite task observation"
        if math.hypot(x - ox, y - oy) > radius:
            return "task horizontal radius exceeded"
        if h > max_h:
            return "task height ceiling exceeded"
        return None

    def stage_reason(state, elapsed, takeoff, travel, hold, descent, terminal):
        limits = {"TAKEOFF": takeoff, "GOTO_B": travel, "DESCEND_B": descent}
        if terminal != "hold":
            limits["HOLD_B"] = hold
        lim = limits.get(state)
        if lim is not None and elapsed >= lim:
            return "%s timeout (%gs)" % (state, lim)
        return None

    def next_descent_setpoint_cm(prev, measured, ground, dt, rate, lead):
        dt = min(max(dt, 0.0), 0.25)
        return max(ground, prev - rate * dt, min(prev, measured - lead))


FLIGHT_STATES = ("TAKEOFF", "GOTO_B", "HOLD_B", "DESCEND_B")


@dataclass
class Limits:
    position_tol_cm: float = 15.0
    height_tol_cm: float = 12.0
    yaw_tol_deg: float = 8.0
    reached_stable_sec: float = 1.0
    landing_tol_cm: float = 3.0
    sensor_timeout_sec: float = 0.5
    task_radius_cm: float = 150.0
    task_max_height_cm: float = 120.0
    takeoff_timeout_sec: float = 20.0
    travel_timeout_sec: float = 20.0
    hold_timeout_sec: float = 20.0
    descent_timeout_sec: float = 30.0
    terminal_behavior: str = "hold"
    active_descent_rate_cm_s: float = 10.0

    def timeout_of(self, state: str) -> Optional[float]:
        table = {
            "TAKEOFF": self.takeoff_timeout_sec,
            "GOTO_B": self.travel_timeout_sec,
            "HOLD_B": None if self.terminal_behavior == "hold" else self.hold_timeout_sec,
            "DESCEND_B": self.descent_timeout_sec,
        }
        return table.get(state)


@dataclass
class Sample:
    """一拍记录：位置/目标 cm，角度 度，时间 s"""
    t: float
    state: str
    x: float = float("nan")
    y: float = float("nan")
    z: float = float("nan")        # ★ 与任务节点同源：/height（cm，AGL）；**不是** TF 的 z
    yaw: float = float("nan")
    slz_cm: float = float("nan")   # 仅诊断用：SLAM(TF) 的 z（2D SLAM 里是常数偏移）
    tx: float = float("nan")
    ty: float = float("nan")
    tz: float = float("nan")
    tyaw: float = float("nan")
    pose_age: float = float("nan")
    height_age: float = float("nan")
    fc_x: float = float("nan")
    fc_y: float = float("nan")
    fc_z: float = float("nan")
    fc_vz: float = float("nan")
    nav_state: float = float("nan")
    arming_state: float = float("nan")

    def errors(self) -> Tuple[float, float, float]:
        return target_error(self.x, self.y, self.z, self.yaw,
                            self.tx, self.ty, self.tz, self.tyaw)


@dataclass
class Segment:
    state: str
    t_start: float
    t_end: Optional[float] = None
    samples: List[Sample] = field(default_factory=list)
    end_target: Optional[Tuple[float, float, float, float]] = None


@dataclass
class Verdict:
    state: str = ""
    t_start: float = 0.0
    t_end: float = 0.0
    duration: float = 0.0
    n: int = 0
    target: Optional[Tuple[float, float, float, float]] = None
    last: Optional[Tuple[float, float, float, float]] = None
    err_xy_max: float = 0.0
    err_z_max: float = 0.0
    err_yaw_max: float = 0.0
    err_xy_last: float = 0.0
    err_z_last: float = 0.0
    err_yaw_last: float = 0.0
    inside_ratio: float = 0.0
    inside_longest: float = 0.0
    audit_reached: bool = False
    timeout_limit: Optional[float] = None
    timeout_reached: bool = False
    env_max_dist: float = 0.0
    env_max_height: float = 0.0
    env_violation: bool = False
    sensor_gap_samples: int = 0
    descent_monotonic: bool = True
    descent_rate_max_cm_s: float = 0.0
    fc_gap_xy_max_cm: float = 0.0
    flags: List[str] = field(default_factory=list)


def tolerances(state: str, lim: Limits) -> Tuple[float, float, float]:
    if state == "DESCEND_B":
        return lim.position_tol_cm, lim.landing_tol_cm, lim.yaw_tol_deg
    return lim.position_tol_cm, lim.height_tol_cm, lim.yaw_tol_deg


def evaluate_segment(seg: Segment, lim: Limits,
                     origin: Optional[Tuple[float, float]],
                     node_advanced: bool) -> Verdict:
    """按节点同款判据把一段算一遍，并标出与节点判定不一致之处。"""
    v = Verdict(state=seg.state, t_start=seg.t_start)
    if seg.samples:
        v.t_end = seg.t_end if seg.t_end is not None else seg.samples[-1].t
    else:
        v.t_end = seg.t_end if seg.t_end is not None else seg.t_start
    v.duration = max(0.0, v.t_end - v.t_start)
    v.n = len(seg.samples)
    v.target = seg.end_target
    if not seg.samples:
        v.flags.append("段内没有任何采样（审计看不到过程）")
        return v

    last = seg.samples[-1]
    v.last = (last.x, last.y, last.z, last.yaw)
    tol_xy, tol_z, tol_yaw = tolerances(seg.state, lim)
    inside_since: Optional[float] = None
    inside_count = 0
    prev: Optional[Sample] = None
    for s in seg.samples:
        exy, ez, eyaw = s.errors()
        if math.isfinite(exy):
            v.err_xy_max = max(v.err_xy_max, exy)
            v.err_z_max = max(v.err_z_max, ez)
            v.err_yaw_max = max(v.err_yaw_max, eyaw)
            if exy <= tol_xy and ez <= tol_z and eyaw <= tol_yaw:
                inside_count += 1
                if inside_since is None:
                    inside_since = s.t
                v.inside_longest = max(v.inside_longest, s.t - inside_since)
            else:
                inside_since = None
        if origin is not None and math.isfinite(s.x) and math.isfinite(s.y):
            v.env_max_dist = max(v.env_max_dist,
                                 math.hypot(s.x - origin[0], s.y - origin[1]))
            v.env_max_height = max(v.env_max_height, s.z)
            if envelope_reason(s.x, s.y, s.z, origin[0], origin[1],
                               lim.task_radius_cm, lim.task_max_height_cm):
                v.env_violation = True
        if ((math.isfinite(s.pose_age) and s.pose_age > lim.sensor_timeout_sec)
                or (math.isfinite(s.height_age) and s.height_age > lim.sensor_timeout_sec)):
            v.sensor_gap_samples += 1
        if prev is not None:
            dt = s.t - prev.t
            if dt > 1e-6 and math.isfinite(s.tz) and math.isfinite(prev.tz):
                v.descent_rate_max_cm_s = max(v.descent_rate_max_cm_s,
                                              (prev.tz - s.tz) / dt)
                if seg.state == "DESCEND_B" and s.tz > prev.tz + 0.5:
                    v.descent_monotonic = False
            if (math.isfinite(s.fc_x) and math.isfinite(prev.fc_x)
                    and math.isfinite(s.x) and math.isfinite(s.fc_y)):
                v.fc_gap_xy_max_cm = max(v.fc_gap_xy_max_cm,
                                         math.hypot(s.fc_x - s.x, s.fc_y - s.y))
        prev = s

    v.err_xy_last, v.err_z_last, v.err_yaw_last = last.errors()
    v.inside_ratio = inside_count / float(len(seg.samples))
    v.audit_reached = v.inside_longest >= lim.reached_stable_sec
    v.timeout_limit = lim.timeout_of(seg.state)
    v.timeout_reached = (v.timeout_limit is not None and v.duration >= v.timeout_limit)

    # ---- 差异标记：对账的重点 ----
    if seg.state in FLIGHT_STATES and node_advanced and not v.audit_reached:
        v.flags.append("节点切段了，但按同一容差本段从未连续 %.1fs 满足容差（假到位？）"
                       % lim.reached_stable_sec)
    if seg.state in FLIGHT_STATES and (not node_advanced) and v.audit_reached:
        v.flags.append("审计已连续 %.1fs 满足容差，节点却没切段（漏切？）" % v.inside_longest)
    if v.timeout_reached and not node_advanced:
        v.flags.append("本段 %.1fs ≥ 超时 %.0fs，节点却没进 FAULT_HOLD（超时未生效？）"
                       % (v.duration, v.timeout_limit or 0.0))
    if v.env_violation and not node_advanced:
        v.flags.append("包线超限（离 A 最远 %.0f cm / 最高 %.0f cm）节点却没进 FAULT_HOLD"
                       % (v.env_max_dist, v.env_max_height))
    if v.sensor_gap_samples:
        v.flags.append("段内 %d 拍定位/高度超过 sensor_timeout(%.2fs)"
                       % (v.sensor_gap_samples, lim.sensor_timeout_sec))
    if seg.state == "DESCEND_B":
        if not v.descent_monotonic:
            v.flags.append("下降段目标回抬（不单调）——与 landing.next_descent_setpoint_cm 约定不符")
        if v.descent_rate_max_cm_s > lim.active_descent_rate_cm_s * 1.5 + 1.0:
            v.flags.append("下降速率峰值 %.1f cm/s 远超设定 %.1f cm/s"
                           % (v.descent_rate_max_cm_s, lim.active_descent_rate_cm_s))
    if (seg.state in FLIGHT_STATES and math.isfinite(v.err_xy_max)
            and v.err_xy_max > tol_xy * 3):
        v.flags.append("水平误差峰值 %.0f cm = 容差 %.0f cm 的 %.1f 倍"
                       % (v.err_xy_max, tol_xy, v.err_xy_max / max(tol_xy, 1e-9)))
    return v


def format_report(verdicts: Sequence[Verdict], lim: Limits,
                  origin: Optional[Tuple[float, float]]) -> str:
    out: List[str] = []
    out.append("=" * 118)
    out.append("A→B 任务逐段对账 | 判据来源：%s | 容差 xy %.0f cm / z %.0f cm / yaw %.0f° | 稳定 %.1fs"
               % (_CRITERIA_SOURCE, lim.position_tol_cm, lim.height_tol_cm,
                  lim.yaw_tol_deg, lim.reached_stable_sec))
    out.append("包线：半径 %.0f cm / 高度 %.0f cm | A 点锚定 = %s"
               % (lim.task_radius_cm, lim.task_max_height_cm,
                  "首段实测位置" if origin else "未锚定（未经历 TAKEOFF）"))
    out.append("=" * 118)
    out.append("%-11s %7s %5s | %-25s | %-25s | %6s %6s %7s | %5s %5s | %s"
               % ("段", "时长s", "采样", "目标(x,y,z,yaw)", "实际(x,y,z,yaw)",
                  "XY", "Z", "Yaw", "内比", "超时", "判定 / 标记"))
    out.append("-" * 118)
    for v in verdicts:
        tg = "(%.0f,%.0f,%.0f,%.0f)" % v.target if v.target else "-"
        ac = "(%.0f,%.0f,%.0f,%.0f)" % v.last if v.last else "-"
        txt = "到位" if v.audit_reached else "未到位"
        if v.timeout_limit is not None:
            txt += " / 限%.0fs" % v.timeout_limit
        if v.env_violation:
            txt += " [包线超限]"
        if v.flags:
            txt += "  ⚠ " + "；".join(v.flags)
        out.append("%-11s %7.1f %5d | %-25s | %-25s | %6.1f %6.1f %7.1f | %4.0f%% %5s | %s"
                   % (v.state, v.duration, v.n, tg, ac,
                      v.err_xy_last, v.err_z_last, v.err_yaw_last,
                      v.inside_ratio * 100.0,
                      "是" if v.timeout_reached else "否", txt))
    out.append("-" * 118)
    bad = [v for v in verdicts if v.flags]
    out.append("汇总：共 %d 段，其中 %d 段有差异标记。" % (len(verdicts), len(bad)))
    for v in bad:
        out.append("  ⚠ %-11s %s" % (v.state, "；".join(v.flags)))
    if not bad:
        out.append("  未发现「节点判定 vs 审计判定」不一致之处。")
    out.append("说明：本表只对账任务侧判据，不含飞控 EKF/EV 质量；水平误差含 SLAM 漂移，")
    out.append("      不能当成控制精度。")
    return "\n".join(out)


def _self_test() -> int:
    """内置自检：不连 ROS，合成数据验证判据与差异标记。"""
    lim = Limits()
    ok = True

    def mk(state, rows, t_end=None):
        return Segment(state=state, t_start=rows[0].t,
                       t_end=(rows[-1].t if t_end is None else t_end),
                       samples=list(rows))

    def s(t, x, y, z, yaw, tx, ty, tz, tyaw, pose_age=0.0, height_age=0.0):
        return Sample(t=t, state="X", x=x, y=y, z=z, yaw=yaw, tx=tx, ty=ty, tz=tz,
                      tyaw=tyaw, pose_age=pose_age, height_age=height_age)

    rows = [s(i * 0.1, 100.0 + 0.2 * i, 0.0, 50.0, 0.0, 100.0, 0.0, 50.0, 0.0)
            for i in range(20)]
    v = evaluate_segment(mk("GOTO_B", rows), lim, (0.0, 0.0), True)
    if not v.audit_reached or v.flags:
        ok = False
        print("[1] 失败 reached=%s flags=%s" % (v.audit_reached, v.flags))

    rows = [s(i * 0.1, 120.0, 0.0, 50.0, 0.0, 100.0, 0.0, 50.0, 0.0) for i in range(20)]
    v = evaluate_segment(mk("GOTO_B", rows), lim, (0.0, 0.0), True)
    if v.audit_reached or not any("假到位" in f for f in v.flags):
        ok = False
        print("[2] 失败 reached=%s flags=%s" % (v.audit_reached, v.flags))

    rows = [s(i * 0.5, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 50.0, 0.0) for i in range(50)]
    v = evaluate_segment(mk("TAKEOFF", rows), lim, (0.0, 0.0), False)
    if not v.timeout_reached or not any("超时未生效" in f for f in v.flags):
        ok = False
        print("[3] 失败 timeout=%s flags=%s" % (v.timeout_reached, v.flags))

    rows = [s(i * 0.1, 200.0, 0.0, 50.0, 0.0, 200.0, 0.0, 50.0, 0.0) for i in range(10)]
    v = evaluate_segment(mk("GOTO_B", rows), lim, (0.0, 0.0), False)
    if not v.env_violation or not any("包线超限" in f for f in v.flags):
        ok = False
        print("[4] 失败 env=%s flags=%s" % (v.env_violation, v.flags))

    rows = []
    for i in range(10):
        tz = 50.0 - i * 2.0 if i < 6 else 38.0 + (i - 6) * 4.0
        rows.append(s(i * 0.2, 0.0, 0.0, tz, 0.0, 0.0, 0.0, tz, 0.0))
    v = evaluate_segment(mk("DESCEND_B", rows), lim, None, False)
    if v.descent_monotonic or not any("回抬" in f for f in v.flags):
        ok = False
        print("[5] 失败 monotonic=%s flags=%s" % (v.descent_monotonic, v.flags))

    rows = [s(i * 0.1, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 50.0, 0.0, pose_age=0.9)
            for i in range(10)]
    v = evaluate_segment(mk("TAKEOFF", rows), lim, (0.0, 0.0), False)
    if v.sensor_gap_samples == 0 or not any("sensor_timeout" in f for f in v.flags):
        ok = False
        print("[6] 失败 gap=%s flags=%s" % (v.sensor_gap_samples, v.flags))

    if next_descent_setpoint_cm(50.0, 100.0, 4.0, 0.1, 10.0, 5.0) < 4.0:
        ok = False
        print("[7] 失败：目标穿地")
    if next_descent_setpoint_cm(50.0, 900.0, 4.0, 0.1, 10.0, 5.0) > 50.0:
        ok = False
        print("[7] 失败：追了向上的假读数")

    for st in ("TAKEOFF", "GOTO_B", "DESCEND_B"):
        if not stage_reason(st, lim.timeout_of(st) + 0.1, lim.takeoff_timeout_sec,
                            lim.travel_timeout_sec, lim.hold_timeout_sec,
                            lim.descent_timeout_sec, lim.terminal_behavior):
            ok = False
            print("[8] 失败：%s 超时未被 stage_reason 判出" % st)

    print("p2p_audit 自检：%s（判据来源：%s）"
          % ("全部通过" if ok else "有不通过项", _CRITERIA_SOURCE))
    return 0 if ok else 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="A→B 任务逐段对账（只读）")
    ap.add_argument("--self-test", action="store_true", help="不连 ROS，跑内置自检")
    ap.add_argument("--out-dir", default=None,
                    help="输出目录（默认 ~/flight_records/p2p_audit_<时间戳>）")
    ap.add_argument("--rate", type=float, default=10.0, help="采样频率 Hz（默认 10）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="记录多少秒后自动结束并打印对账表（0 = 一直记到 Ctrl-C）")
    args = ap.parse_args(list(argv) if argv is not None else None)
    if args.self_test:
        return _self_test()

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from rclpy.time import Time
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
    from std_msgs.msg import Float32MultiArray, Int16, String
    from tf2_ros import Buffer, TransformException, TransformListener

    class P2PAudit(Node):
        def __init__(self) -> None:
            super().__init__("p2p_audit")
            dp = self.declare_parameter
            self.map_frame = dp("map_frame", "map").value
            self.body_frame = dp("body_frame", "laser_link").value
            self.height_topic = dp("height_topic", "/height").value
            self.lim = Limits(
                position_tol_cm=float(dp("position_tolerance_cm", 15.0).value),
                height_tol_cm=float(dp("height_tolerance_cm", 12.0).value),
                yaw_tol_deg=float(dp("yaw_tolerance_deg", 8.0).value),
                reached_stable_sec=float(dp("reached_stable_sec", 1.0).value),
                landing_tol_cm=float(dp("landing_height_tolerance_cm", 3.0).value),
                sensor_timeout_sec=float(dp("sensor_timeout_sec", 0.5).value),
                task_radius_cm=float(dp("task_radius_cm", 150.0).value),
                task_max_height_cm=float(dp("task_max_height_cm", 120.0).value),
                takeoff_timeout_sec=float(dp("takeoff_timeout_sec", 20.0).value),
                travel_timeout_sec=float(dp("travel_timeout_sec", 20.0).value),
                hold_timeout_sec=float(dp("hold_timeout_sec", 20.0).value),
                descent_timeout_sec=float(dp("descent_timeout_sec", 30.0).value),
                terminal_behavior=str(dp("terminal_behavior", "hold").value),
                active_descent_rate_cm_s=float(dp("active_descent_rate_cm_s", 10.0).value),
            )
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.out_dir = args.out_dir or os.path.join(
                os.path.expanduser("~"), "flight_records", "p2p_audit_%s" % stamp)
            os.makedirs(self.out_dir, exist_ok=True)
            self.csv_path = os.path.join(self.out_dir, "samples.csv")
            self.report_path = os.path.join(self.out_dir, "report.txt")
            self._fh = open(self.csv_path, "w", newline="")
            self._wr = csv.writer(self._fh)
            self._wr.writerow(["t", "state", "x_cm", "y_cm", "z_cm", "yaw_deg",
                               "tx_cm", "ty_cm", "tz_cm", "tyaw_deg",
                               "slz_cm_slam",
                               "err_xy_cm", "err_z_cm", "err_yaw_deg",
                               "pose_age_s", "height_age_s",
                               "fc_x_m", "fc_y_m", "fc_z_m", "fc_vz_mps",
                               "nav_state", "arming_state"])
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            px4q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.target: Optional[Tuple[float, float, float, float]] = None
            self.height_cm: Optional[float] = None
            self.height_t = 0.0
            self.pose: Optional[Tuple[float, float, float, float]] = None
            self.fc: Optional[Tuple[float, float, float, float]] = None
            self.nav_state = float("nan")
            self.arming_state = float("nan")
            self.origin: Optional[Tuple[float, float]] = None
            self.seg: Optional[Segment] = None
            self.done: List[Verdict] = []
            self._finished = False
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(String, "/p_to_p/state", self._on_state, latched)
            self.create_subscription(Float32MultiArray, "/target_position",
                                     self._on_target, latched)
            self.create_subscription(Int16, self.height_topic, self._on_height, 10)
            self.create_subscription(VehicleLocalPosition,
                                     "/fmu/out/vehicle_local_position",
                                     self._on_fc_pos, px4q)
            self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status",
                                     self._on_status, px4q)
            self.create_timer(1.0 / max(1.0, args.rate), self._tick)
            if args.duration > 0.0:
                self.create_timer(args.duration, self._finish_and_stop)
            self.get_logger().warn("p2p_audit（只读）开始记录 → %s"
                                   "；Ctrl-C 结束并打印对账表" % self.csv_path)

        def _finish_and_stop(self) -> None:
            self.get_logger().warn("p2p_audit：记录时长到（%.0fs），结束并输出对账表"
                                   % args.duration)
            self.finish()
            raise SystemExit(0)

        def _on_state(self, msg: String) -> None:
            raw = (msg.data or "").strip()
            state = raw.split(":")[0].split()[0] if raw else "UNKNOWN"
            if self.seg is None or state != self.seg.state:
                advanced = (self.seg is not None and self.seg.state in FLIGHT_STATES
                            and state != self.seg.state)
                self._close_segment(advanced)
                self.seg = Segment(state=state, t_start=self._now())
                self.seg.end_target = self.target
                if state == "TAKEOFF" and self.pose is not None:
                    self.origin = (self.pose[0], self.pose[1])

        def _on_target(self, msg: Float32MultiArray) -> None:
            vals = list(msg.data)
            if len(vals) >= 4:
                self.target = (float(vals[0]), float(vals[1]),
                               float(vals[2]), float(vals[3]))
                if self.seg is not None:
                    self.seg.end_target = self.target

        def _on_height(self, msg: Int16) -> None:
            self.height_cm = float(msg.data)
            self.height_t = self._now()

        def _on_fc_pos(self, msg: VehicleLocalPosition) -> None:
            self.fc = (msg.x * 100.0, msg.y * 100.0, msg.z * 100.0, float(msg.vz))

        def _on_status(self, msg: VehicleStatus) -> None:
            self.nav_state = float(msg.nav_state)
            self.arming_state = float(msg.arming_state)

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9 - self._t0

        def _pose_now(self):
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame,
                                                     self.body_frame, Time())
            except TransformException:
                return None, float("inf")
            age = (self.get_clock().now().nanoseconds
                   - Time.from_msg(tf.header.stamp).nanoseconds) * 1e-9
            q = tf.transform.rotation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            return (tf.transform.translation.x * 100.0,
                    tf.transform.translation.y * 100.0,
                    tf.transform.translation.z * 100.0, yaw), age

        def _tick(self) -> None:
            pose, pose_age = self._pose_now()
            if pose is not None:
                self.pose = pose
            t = self._now()
            if self.seg is None:
                return
            p = self.pose if self.pose is not None else (float("nan"),) * 4
            tg = self.target if self.target is not None else (float("nan"),) * 4
            fc = self.fc if self.fc is not None else (float("nan"),) * 4
            s = Sample(t=t, state=self.seg.state, x=p[0], y=p[1], yaw=p[3],
                       z=(self.height_cm if self.height_cm is not None
                          else float("nan")),
                       slz_cm=p[2],
                       tx=tg[0], ty=tg[1], tz=tg[2], tyaw=tg[3], pose_age=pose_age,
                       height_age=((t - self.height_t) if self.height_cm is not None
                                   else float("inf")),
                       fc_x=fc[0], fc_y=fc[1], fc_z=fc[2], fc_vz=fc[3],
                       nav_state=self.nav_state, arming_state=self.arming_state)
            self.seg.samples.append(s)
            e = s.errors()
            row = ["%.3f" % t, self.seg.state]
            row += ["%.2f" % v if math.isfinite(v) else "nan"
                    for v in (p[0], p[1], s.z, p[3])]
            row += ["%.2f" % v if math.isfinite(v) else "nan" for v in tuple(tg)]
            row += ["%.2f" % s.slz_cm if math.isfinite(s.slz_cm) else "nan"]
            row += ["%.1f" % v if math.isfinite(v) else "nan" for v in e]
            row += ["%.3f" % pose_age, "%.3f" % s.height_age]
            row += ["%.3f" % (v / 100.0) if math.isfinite(v) else "nan"
                    for v in (s.fc_x, s.fc_y, s.fc_z, s.fc_vz)]
            row += [self.nav_state, self.arming_state]
            self._wr.writerow(row)
            self._fh.flush()

        def _close_segment(self, advanced: bool) -> None:
            if self.seg is None:
                return
            self.seg.t_end = self._now()
            self.done.append(evaluate_segment(self.seg, self.lim, self.origin, advanced))
            self.seg = None

        def finish(self) -> None:
            if self._finished:
                return
            self._finished = True
            self._close_segment(False)
            try:
                self._fh.close()
            except Exception:
                pass
            text = format_report(self.done, self.lim, self.origin)
            print(text)
            with open(self.report_path, "w") as fh:
                fh.write(text + "\n")
            print("原始采样：%s" % self.csv_path)
            print("对账报告：%s" % self.report_path)

    rclpy.init()
    node = P2PAudit()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.finish()
        except Exception as exc:
            print("收尾失败：%s" % exc)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
