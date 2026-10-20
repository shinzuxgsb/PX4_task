#!/usr/bin/env python3
"""把 .ulg 里的关键状态变化按"飞控开机时间(ms)"排成事件表。

用法：python3 ulg_events.py <log.ulg>
"""

from __future__ import annotations

import sys

import numpy as np
from pyulog import ULog

NAV = {
    # vehicle_status.nav_state（见 PX4 msg/VehicleStatus.msg）
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER",
    5: "AUTO_RTL", 6: "POSITION_SLOW", 10: "ACRO", 12: "DESCEND", 13: "TERMINATION",
    14: "OFFBOARD", 15: "STABILIZED", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
    19: "AUTO_FOLLOW", 20: "AUTO_PRECLAND", 21: "ORBIT", 22: "AUTO_VTOL_TAKEOFF",
}


def series(log, name, field):
    for d in log.data_list:
        if d.name == name and field in d.data:
            return np.asarray(d.data["timestamp"]), np.asarray(d.data[field])

    return None, None


def report(log, name, field, label, fmt=str):
    t, v = series(log, name, field)

    if t is None:
        print(f"  (没有 {name}.{field})")
        return

    prev = None

    for i in range(len(t)):
        cur = v[i]

        if prev is None or cur != prev:
            print(f"  t={t[i]/1000.0:9.1f}ms ({(t[i]-t[0])/1e6:6.2f}s from log start)  {label}={fmt(cur)}")
            prev = cur


def main() -> int:
    log = ULog(sys.argv[1])
    print("== 事件表 ==")
    report(log, "vehicle_status", "arming_state", "arming_state")
    report(log, "vehicle_status", "nav_state", "nav_state", lambda x: NAV.get(int(x), x))
    report(log, "actuator_armed", "armed", "armed")
    report(log, "vehicle_land_detected", "landed", "landed")

    t, v = series(log, "actuator_motors", "control[0]")

    if t is not None:
        print(f"  电机: 首次>0.05 在 t={(t[np.argmax(v > 0.05)]/1000.0):.1f}ms, "
              f"最大 {v.max():.3f}, 归零在 t={(t[len(v)-1-int(np.argmax(v[::-1] > 0.05))]/1000.0):.1f}ms")

    t, v = series(log, "vehicle_local_position", "z")

    if t is not None:
        print(f"  高度: 起始 {v[0]:+.2f}m  最高 {-v.min():+.2f}m  结束 {v[-1]:+.2f}m (NED, 负=升高)")

    t, v = series(log, "vehicle_local_position", "xy_valid")

    if t is not None:
        print(f"  水平位置有效: {int(v.sum())}/{len(v)} 采样为 True")

    t, v = series(log, "vehicle_local_position", "v_xy_valid")

    if t is not None:
        print(f"  水平速度有效: {int(v.sum())}/{len(v)} 采样为 True")

    print(f"\n  日志覆盖: {(t[-1]-t[0])/1e6:.1f}s（飞控开机后 {t[0]/1e6:.1f}s ~ {t[-1]/1e6:.1f}s）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
