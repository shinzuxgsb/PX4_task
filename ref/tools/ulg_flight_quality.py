#!/usr/bin/env python3
"""从 .ulg 里量化"定点飞得稳不稳"，输出可直接写进设计报告的指标。

    python3 ulg_flight_quality.py <log.ulg>

给出：
  * 模式时间线（确认这次是不是 POSCTL 定点）与解锁窗口
  * 外部定位是否真的融合（xy_valid 占比、ev 创新量）
  * 悬停段的水平保持：晃动 RMS、漂移速率(cm/s)、总位移(cm)
  * 高度保持：均值/标准差/最大偏差(cm)
"""

from __future__ import annotations

import sys

import numpy as np
from pyulog import ULog

# nav_state 枚举取自本机固件 msg/VehicleStatus.msg（PX4 v1.15.4）
NAV = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
       6: "POSITION_SLOW", 10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD",
       15: "STABILIZED", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND", 19: "AUTO_FOLLOW",
       20: "AUTO_PRECLAND", 21: "ORBIT"}


def load(log: ULog, name: str):
    for d in log.data_list:
        if d.name == name and "timestamp" in d.data:
            return d.data

    return None


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    log = ULog(sys.argv[1])
    vs = load(log, "vehicle_status")
    vlp = load(log, "vehicle_local_position")
    inn = load(log, "estimator_innovations")

    print("=" * 62)
    print(f"飞行质量报告  {sys.argv[1]}")
    print("=" * 62)

    # ---- 模式时间线 -------------------------------------------------------
    if vs is not None:
        t = np.asarray(vs["timestamp"])
        nav = np.asarray(vs["nav_state"])
        arm = np.asarray(vs["arming_state"])
        print("\n[模式时间线]")
        prev = None

        for i in range(len(t)):
            key = (int(nav[i]), int(arm[i]))

            if key != prev:
                print(f"  t+{(t[i]-t[0])/1e6:7.2f}s  模式={NAV.get(key[0], key[0]):<16} "
                      f"{'已解锁' if key[1] == 2 else '未解锁'}")
                prev = key

    # ---- 外部定位是否融合 -------------------------------------------------
    if vlp is not None:
        xy = np.asarray(vlp.get("xy_valid", []))
        vxy = np.asarray(vlp.get("v_xy_valid", []))
        print("\n[外部定位（雷达SLAM）融合]")
        print(f"  xy_valid   : {100.0*xy.sum()/max(1,len(xy)):5.1f}% 采样有效")
        print(f"  v_xy_valid : {100.0*vxy.sum()/max(1,len(vxy)):5.1f}% 采样有效")

    if inn is not None:
        parts = []

        for key in ("ev_hpos[0]", "ev_hpos[1]", "ev_hvel[0]", "ev_hvel[1]"):
            if key in inn:
                parts.append(f"{key.split('[')[0]}{key[-2] if '[' in key else ''}"
                             f"={np.nanmax(np.abs(np.asarray(inn[key], dtype=float))):.3f}")

        print("  ev 创新量峰值: " + ("  ".join(parts) if parts else "(无)"))

    # ---- 悬停段保持精度 ---------------------------------------------------
    if vlp is not None:
        t = np.asarray(vlp["timestamp"], dtype=float)
        x = np.asarray(vlp["x"], dtype=float)
        y = np.asarray(vlp["y"], dtype=float)
        z = np.asarray(vlp["z"], dtype=float)
        h = -z                                    # 高度（向上为正）
        mask = h > 0.3                            # 取离地 0.3m 以上的部分当作在空中

        if mask.sum() > 20:
            th, xh, yh, hh = t[mask], x[mask], y[mask], h[mask]
            dur = (th[-1] - th[0]) / 1e6
            mx, my = float(np.mean(xh)), float(np.mean(yh))
            rms = float(np.sqrt(np.mean((xh - mx) ** 2 + (yh - my) ** 2)))
            sx = np.polyfit((th - th[0]) / 1e6, xh, 1)[0]
            sy = np.polyfit((th - th[0]) / 1e6, yh, 1)[0]
            drift_rate = float(np.hypot(sx, sy))
            total_move = float(np.hypot(xh[-1] - xh[0], yh[-1] - yh[0]))
            print("\n[悬停保持精度]（离地 >0.3m 的时段）")
            print(f"  悬停时长     : {dur:.1f} s")
            print(f"  水平晃动 RMS : {rms*100:.1f} cm（相对平均水平位置）")
            print(f"  水平漂移速率 : {drift_rate*100:.2f} cm/s"
                  f"  (x={sx*100:+.2f}, y={sy*100:+.2f})")
            print(f"  起止总位移   : {total_move*100:.1f} cm")
            print(f"  高度 mean±std: {np.mean(hh)*100:.1f} ± {np.std(hh)*100:.1f} cm"
                  f"   最大偏差 {np.max(np.abs(hh-np.mean(hh)))*100:.1f} cm")
            verdict = "稳" if (rms < 0.15 and drift_rate < 0.05 and np.std(hh) < 0.15) else "需改进"
            print(f"  结论         : {verdict}"
                  f"（判据：晃动RMS<15cm 且 漂移<5cm/s 且 高度σ<15cm）")
        else:
            print("\n[悬停保持精度] 该日志里没有离地 >0.3m 的时段（可能是台架会话）")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
