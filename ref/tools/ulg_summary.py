#!/usr/bin/env python3
"""把 PX4 的 .ulg 日志摘成"这次飞行发生了什么"的摘要。

用法：python3 ulg_summary.py <log.ulg>

输出：时长、模式/解锁时间线、位置与速度统计、遥控输入幅度、
      EKF 融合标志与视觉创新量（判断速度到底是谁提供的）。
"""

from __future__ import annotations

import sys

import numpy as np
from pyulog import ULog

NAV_STATE = {
    0: "MANUAL", 1: "ALTITUDE", 2: "POSITION", 3: "MISSION", 4: "HOLD", 5: "RETURN",
    6: "ACRO", 7: "OFFBOARD", 8: "STABILIZED", 9: "POSITION_SLOW", 10: "TAKEOFF",
    11: "LAND", 12: "FOLLOW", 13: "PREC_LAND", 14: "ORBIT", 15: "AUTO_READY",
    16: "AUTO_TAKEOFF", 17: "AUTO_LOITER", 18: "AUTO_MISSION", 19: "AUTO_RTL",
    20: "AUTO_LAND", 21: "AUTO_PREC_LAND", 22: "AUTO_VTOL_TAKEOFF", 23: "EXTERNAL1",
}

ARM_STATE = {0: "INIT", 1: "DISARMED", 2: "ARMED", 3: "STANDBY_ERROR", 4: "SHUTDOWN"}


def topic(log, name):
    for d in log.data_list:
        if d.name == name:
            return d.data

    return None


def timeline(log, name, fields, formatter):
    data = topic(log, name)

    if data is None:
        print(f"  (日志里没有 {name})")
        return

    t = data["timestamp"]
    t0 = t[0]
    prev = None

    for i in range(len(t)):
        key = tuple(int(data[f][i]) for f in fields)

        if key != prev:
            print(f"    +{ (t[i]-t0)/1e6:7.2f}s  {formatter(key)}")
            prev = key


def stats(log, name, fields):
    data = topic(log, name)

    if data is None:
        print(f"  (日志里没有 {name})")
        return None

    print(f"  {name}: {len(data['timestamp'])} 采样")

    for f in fields:
        if f not in data:
            continue

        a = np.asarray(data[f], dtype=float)

        if a.size == 0:
            continue

        print(f"    {f:<12} min={np.nanmin(a):+9.3f}  max={np.nanmax(a):+9.3f}  mean={np.nanmean(a):+9.3f}")

    return data


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    log = ULog(sys.argv[1])
    dur = (log.start_timestamp and 0) or 0
    t_all = [d.data["timestamp"] for d in log.data_list if "timestamp" in d.data and len(d.data["timestamp"])]
    t_min = min(t.min() for t in t_all)
    t_max = max(t.max() for t in t_all)
    print(f"日志: {sys.argv[1]}")
    print(f"时长: {(t_max-t_min)/1e6:.1f} s   话题数: {len(log.data_list)}")
    print(f"里面的话题: {', '.join(sorted(d.name for d in log.data_list))[:600]}")

    print("\n== 解锁 / 模式时间线 ==")
    timeline(log, "vehicle_status", ("arming_state", "nav_state"),
             lambda k: f"arming={ARM_STATE.get(k[0], k[0])}  nav_state={NAV_STATE.get(k[1], k[1])}")

    print("\n== 估计位置 / 速度（EKF 输出）==")
    vlp = stats(log, "vehicle_local_position", ("x", "y", "z", "vx", "vy", "vz", "dist_bottom"))

    if vlp is not None:
        for f in ("xy_valid", "v_xy_valid", "z_valid", "v_z_valid", "dist_bottom_valid"):
            if f in vlp:
                a = np.asarray(vlp[f])
                print(f"    {f:<20} True 占比 {100.0*np.sum(a)/len(a):5.1f}%")

    print("\n== 电机 / 舵机输出（看有没有真的转）==")
    am = topic(log, "actuator_motors")

    if am is not None:
        for i in range(4):
            k = f"control[{i}]"

            if k in am:
                a = np.asarray(am[k], dtype=float)
                print(f"    motor{i}  max={np.nanmax(a):+.3f}  mean={np.nanmean(a):+.3f}")

    ao = topic(log, "actuator_outputs")

    if ao is not None:
        for i in range(4):
            k = f"output[{i}]"

            if k in ao:
                a = np.asarray(ao[k], dtype=float)
                print(f"    out{i}    max={np.nanmax(a):7.1f}  mean={np.nanmean(a):7.1f}")

    print("\n== PX4 收到的外部里程计 ==")
    stats(log, "vehicle_odometry", ("x", "y", "z", "vx", "vy", "vz"))

    print("\n== 遥控摇杆输入（绝对值最大值）==")
    stats(log, "manual_control_setpoint", ("roll", "pitch", "yaw", "throttle"))

    print("\n== 激光测距 ==")
    stats(log, "distance_sensor", ("current_distance",))

    print("\n== EKF 融合标志 ==")
    est = topic(log, "estimator_status")

    if est is not None:
        for f in ("control_mode_flags", "solution_status_flags", "innovation_check_flags", "filter_fault_flags"):
            if f in est:
                a = np.asarray(est[f])
                uniq, counts = np.unique(a, return_counts=True)
                top = sorted(zip(uniq, counts), key=lambda x: -x[1])[:3]
                print(f"    {f}: " + "  ".join(f"{int(v)}({c}次)" for v, c in top))

        # 水平速度/位置方差
        for f in ("vel_test_ratio", "pos_test_ratio", "hgt_test_ratio"):
            if f in est:
                a = np.asarray(est[f], dtype=float)
                print(f"    {f}: max={np.nanmax(a):.2f}")

    print("\n== 视觉创新量（证明外部里程计是否真被融合）==")
    inn = topic(log, "estimator_innovations")

    if inn is None:
        print("  (日志里没有 estimator_innovations)")
    else:
        keys = [k for k in inn.keys() if k.startswith("ev_")]
        print(f"    ev_* 字段: {keys if keys else '(无 ev_ 字段)'}")

        for k in keys:
            a = np.asarray(inn[k], dtype=float)
            print(f"    {k:<16} max|.|={np.nanmax(np.abs(a)):.4f}  mean={np.nanmean(a):+.4f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
