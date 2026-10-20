#!/usr/bin/env python3
"""把 .ulg 里的飞行过程打印成一张紧凑的时间表：模式 / 高度 / 速度 / 指令速度 / 电机。

用法（香橙派上，pyulog + numpy 已装）：
    python3 ulg_flight_trace.py <log.ulg> [每行间隔秒数，默认 0.5]

列含义：
    t      距日志开始的时间（s）
    nav    vehicle_status.nav_state（2=POSCTL，14=OFFBOARD，18=AUTO_LAND …）
    armed  解锁状态（1=已解锁）
    z      高度（m，由 NED 的 -z 得到，机腹朝下为正）
    vz     垂直速度（m/s，正=上升）
    vh     水平速度（m/s）
    sp_vz  飞控正在跟踪的垂直速度指令（m/s）——非零说明外部 setpoint 真的到了
    sp_vh  飞控正在跟踪的水平速度指令（m/s）
    mot    电机输出最大值（0~1）
    ds     测距模块读数（m）
"""

from __future__ import annotations

import bisect
import math
import sys

import numpy as np
from pyulog import ULog

NAV_NAMES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION", 4: "LOITER", 5: "RTL",
    6: "POSCTL_SLOW", 10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD",
    15: "STAB", 17: "TAKEOFF", 18: "LAND", 19: "FOLLOW", 20: "PRECLAND", 21: "ORBIT",
    22: "VTOL_TAKEOFF",
}


def dataset(log: ULog, name: str) -> dict | None:
    for d in log.data_list:
        if d.name == name:
            return d.data

    return None


def column(data: dict | None, field: str) -> np.ndarray | None:
    if data is None or field not in data:
        return None

    return np.asarray(data[field], dtype=float)


class Series:
    """按时间戳查询某话题的最近一条记录。"""

    def __init__(self, data: dict | None):
        self.data = data
        ts = column(data, "timestamp")
        self.ts = ts if ts is not None else np.zeros(0)

    def at(self, t: float, field: str):
        if self.data is None or field not in self.data or len(self.ts) == 0:
            return None

        i = bisect.bisect_left(self.ts, t)
        i = 0 if i == 0 else min(i, len(self.ts) - 1)
        v = self.data[field][i]

        try:
            v = float(v)
        except (TypeError, ValueError):
            return None

        return None if math.isnan(v) else v


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    path = sys.argv[1]
    step_s = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    log = ULog(path)

    lp = dataset(log, "vehicle_local_position")
    if lp is None:
        print("日志里没有 vehicle_local_position，无法分析")
        return 1

    lp_s = Series(lp)
    t0 = float(lp_s.ts[0])
    t_end = float(lp_s.ts[-1])
    status = Series(dataset(log, "vehicle_status"))
    sp = Series(dataset(log, "vehicle_local_position_setpoint"))
    motors = Series(dataset(log, "actuator_motors"))
    ds = Series(dataset(log, "distance_sensor"))

    def height(i: int) -> float:
        return -float(lp["z"][i])

    def motors_max(t: float):
        if motors.data is None or "actuator" not in motors.data:
            return None

        i = bisect.bisect_left(motors.ts, t)
        i = 0 if i == 0 else min(i, len(motors.ts) - 1)
        row = np.asarray(motors.data["actuator"][i][:4], dtype=float)

        return float(np.nanmax(row)) if row.size else None

    zs = np.asarray([height(i) for i in range(len(lp_s.ts))])
    i_peak = int(np.nanargmax(zs))
    xy = np.column_stack([np.asarray(lp["x"], dtype=float), np.asarray(lp["y"], dtype=float)])
    travel = float(np.nansum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    ds_col = column(ds.data, "current_distance")
    ds_vals = [float(v) for v in ds_col if not math.isnan(v)] if ds_col is not None else []

    print(f"== {path} ==")
    print(f"日志时长 {(t_end - t0) / 1e6:.1f}s | 水平位移(累计) {travel:.2f}m | "
          f"最高 {zs[i_peak]:.2f}m (t={(lp_s.ts[i_peak] - t0) / 1e6:.1f}s) | "
          f"结束高度 {zs[-1]:.2f}m")
    if ds_vals:
        print(f"测距模块: {min(ds_vals):.3f}~{max(ds_vals):.3f}m")
    print()
    print(f"{'t':>6} {'nav':>10} {'armd':>4} {'z':>7} {'vz':>7} {'vh':>7} "
          f"{'sp_vz':>7} {'sp_vh':>7} {'mot':>5} {'ds':>6}")

    next_row = t0
    prev_key = None

    for i in range(len(lp_s.ts)):
        t = float(lp_s.ts[i])
        nav = status.at(t, "nav_state")
        armed = status.at(t, "arming_state")
        key = (nav, armed)
        # 每 step_s 打一行；模式/解锁状态一变立刻打一行
        if t < next_row and key == prev_key and i != i_peak:
            continue

        next_row = t + step_s * 1e6
        prev_key = key
        vz = float(lp["vz"][i]) if "vz" in lp else float("nan")
        vx = float(lp["vx"][i]) if "vx" in lp else 0.0
        vy = float(lp["vy"][i]) if "vy" in lp else 0.0
        s_vz = sp.at(t, "vz")
        s_vx = sp.at(t, "vx") or 0.0
        s_vy = sp.at(t, "vy") or 0.0
        mot = motors_max(t)
        d = ds.at(t, "current_distance")
        nav_txt = NAV_NAMES.get(int(nav), f"nav{int(nav)}") if nav is not None else "--"
        print(f"{(t - t0) / 1e6:6.1f} {nav_txt:>10} "
              f"{('1' if armed == 2 else '0') if armed is not None else '--':>4} "
              f"{height(i):7.2f} {vz:7.2f} {math.hypot(vx, vy):7.2f} "
              f"{(s_vz if s_vz is not None else float('nan')):7.2f} "
              f"{math.hypot(s_vx, s_vy):7.2f} "
              f"{(mot if mot is not None else float('nan')):5.2f} "
              f"{(d if d is not None else float('nan')):6.2f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
