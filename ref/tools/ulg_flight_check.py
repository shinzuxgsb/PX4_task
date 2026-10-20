#!/usr/bin/env python3
"""逐时间片看一份 .ulg：到底飞没飞、靠什么保持水平位置/速度。

用法：python3 ulg_flight_check.py <log.ulg> [采样间隔]
"""

from __future__ import annotations

import sys

import numpy as np
from pyulog import ULog


def get(log, name):
    for d in log.data_list:
        if d.name == name:
            return d.data

    return None


def main() -> int:
    path = sys.argv[1]
    step = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    log = ULog(path)

    stamps = [np.asarray(d.data["timestamp"]) for d in log.data_list
              if "timestamp" in d.data and len(d.data["timestamp"])]
    t0 = min(s.min() for s in stamps)

    vs = get(log, "vehicle_status")

    if vs is not None:
        print(f"armed_time    = {vs['armed_time'][0] or vs['armed_time'][-1]}")
        print(f"takeoff_time  = {vs['takeoff_time'][0] or vs['takeoff_time'][-1]}")

    vlp = get(log, "vehicle_local_position")

    if vlp is not None:
        print("\n== 位置/速度时间线 ==")
        print("     t(s)      z      vz   xy_valid  v_xy_valid  dist_bottom  bottom_ok")

        for i in range(0, len(vlp["timestamp"]), step):
            print(f"  {(vlp['timestamp'][i]-t0)/1e6:7.2f}  {vlp['z'][i]:+6.2f}  {vlp['vz'][i]:+6.2f}"
                  f"  {str(bool(vlp['xy_valid'][i])):>8}  {str(bool(vlp['v_xy_valid'][i])):>10}"
                  f"  {vlp['dist_bottom'][i]:10.2f}  {bool(vlp['dist_bottom_valid'][i])}")

    ev = get(log, "estimator_event_flags")

    if ev is not None:
        print("\n== 视觉（外部里程计）融合事件 ==")

        for k in ("starting_vision_pos_fusion", "starting_vision_vel_fusion", "starting_vision_yaw_fusion",
                  "reset_pos_to_vision", "reset_vel_to_vision", "reset_pos_to_last_known", "reset_hgt_to_baro"):
            if k in ev:
                a = np.asarray(ev[k])
                print(f"  {k:<28} True {int(np.sum(a)):>4} / {len(a)}")

    am = get(log, "actuator_motors")

    if am is not None:
        print("\n== 电机输出时间线（控制量 0~1）==")

        for i in range(0, len(am["timestamp"]), step * 2):
            vals = " ".join(f"{am[f'control[{k}]'][i]:+.3f}" for k in range(4))
            print(f"  {(am['timestamp'][i]-t0)/1e6:7.2f}s  {vals}")

    print(f"\n话题清单: {', '.join(sorted(d.name for d in log.data_list))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
