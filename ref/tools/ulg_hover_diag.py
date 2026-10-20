#!/usr/bin/env python3
"""悬停质量诊断：把"能不能再稳一点"拆成可量的几项。

用法：python3 ulg_hover_diag.py <log.ulg>

输出：
  1) 悬停油门估计 vs MPC_THR_HOVER（偏差大 → 高度环一直顶着固定偏置）
  2) 速度指令 → 实际速度 的响应延迟（内环延迟，用互相关找最佳滞后）
  3) 测距/激光高度 vs EKF 高度（延迟 + 差值 RMS；两者不一致说明定高源有问题）
  4) 水平环工作点：悬停段指令速度与实际速度的均值/标准差
"""

from __future__ import annotations

import math
import sys

import numpy as np
from pyulog import ULog


def get(log: ULog, name: str):
    for d in log.data_list:
        if d.name == name:
            return d.data

    return None


def col(data, field):
    if data is None or field not in data:
        return None

    return np.asarray(data[field], dtype=float)


def lag_scan(t, a, b, max_lag_ms=600, step_ms=20):
    """找让 b 最贴近 a（同一起始时刻）的滞后：返回 (lag_ms, 残差相关系数)。"""
    best = (0.0, -2.0)
    t0 = max(t.min(), t.min())

    for lag in np.arange(0.0, max_lag_ms + 1.0, step_ms):
        ti = np.arange(t0, t.max() - lag * 1e-3, 0.02)
        ai = np.interp(ti, t, a)
        bi = np.interp(ti + lag * 1e-3, t, b)

        if ai.std() < 1e-6 or bi.std() < 1e-6:
            continue

        r = float(np.corrcoef(ai, bi)[0, 1])

        if r > best[1]:
            best = (float(lag), r)

    return best


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    log = ULog(sys.argv[1])
    # 可选用时间窗限定"稳态段"：python3 ulg_hover_diag.py <log> [t0秒] [t1秒]
    win = None

    if len(sys.argv) > 3:
        win = (float(sys.argv[2]), float(sys.argv[3]))

    lp = get(log, "vehicle_local_position")

    if lp is None:
        print("日志里没有 vehicle_local_position")
        return 1

    t = col(lp, "timestamp") / 1e6
    z = -col(lp, "z")                      # 高度，机腹朝上为正
    vx, vy, vz = col(lp, "vx"), col(lp, "vy"), col(lp, "vz")
    armed = None

    # 悬停段：高度 > 0.3m 且 |vz| 不大（排除起降）
    hover = (z > 0.30) & (np.abs(vz) < 0.15)

    if win is not None:
        tb = t - t[0]
        hover &= (tb >= win[0]) & (tb <= win[1])

    print(f"日志 {sys.argv[1]}")
    print(f"分析窗口：{'稳态段 %.1f~%.1fs' % win if win else '自动（高度>0.3m 且垂直速度<0.15m/s）'}"
          f"，采样 {int(hover.sum())} 个\n")

    # 1) 悬停油门
    hte = get(log, "hover_thrust_estimate")
    thr = col(hte, "hover_thrust") if hte is not None else None
    params = {k: v for k, v in (log.initial_parameters or {}).items()}
    mpc_thr = params.get("MPC_THR_HOVER")

    if thr is not None:
        print(f"1) 悬停油门估计 mean={float(np.nanmean(thr)):.3f} "
              f"std={float(np.nanstd(thr)):.3f}  | MPC_THR_HOVER={mpc_thr}")
        if mpc_thr:
            print(f"   偏差 {float(np.nanmean(thr)) - mpc_thr:+.3f}"
                  f"（偏大=参数偏低，飞控需要更多油门才能悬停；超过 ±0.05 值得重新标定）\n")
    else:
        print("1) 日志里没有 hover_thrust_estimate\n")

    # 2) 内环延迟：指令速度 → 实际速度
    sp = get(log, "vehicle_local_position_setpoint")

    if sp is not None:
        ts = col(sp, "timestamp") / 1e6
        spvx, spvy = col(sp, "vx"), col(sp, "vy")
        # 只在 offboard 且悬停的时间段内比较
        m = (ts > t[hover].min()) & (ts < t[hover].max())
        lagx, rx = lag_scan(ts[m], spvx[m], np.interp(ts[m], t, vx))
        lagy, ry = lag_scan(ts[m], spvy[m], np.interp(ts[m], t, vy))
        print(f"2) 指令→实际 速度响应延迟：x {lagx:.0f}ms(r={rx:.2f})  "
              f"y {lagy:.0f}ms(r={ry:.2f})")
        print("   （这套链路的总延迟还包括 雷达→SLAM→odom→EKF；内环就有几百 ms 的话，"
              "位置环增益不能再加大）\n")
    else:
        print("2) 日志里没有 vehicle_local_position_setpoint\n")

    # 3) 激光高度 vs EKF 高度
    ds = get(log, "distance_sensor")

    if ds is not None:
        td = col(ds, "timestamp") / 1e6
        d = col(ds, "current_distance")
        ok = (d > 0.02) & (d < 15.0)
        td, d = td[ok], d[ok]

        if td.size > 10:
            ti = np.arange(max(td.min(), t.min()), min(td.max(), t.max()), 0.05)
            zz = np.interp(ti, t, z)
            dd = np.interp(ti, td, d)

            if ti.size > 10:
                # 高度零点：取整段最小高度作地面参考
                floor = float(np.percentile(zz, 2))
                lagd, rd = lag_scan(ti, zz - floor, dd)      # 两者都在 ti 上，别再按掩码截断
                # 把激光按最佳滞后对齐后再比差值
                sel = (zz - floor > 0.30) & (dd < 15.0)
                diff = (zz - floor - np.interp(ti + lagd * 1e-3, td, d))[sel]
                print(f"3) 激光/测距 vs EKF 高度：最佳滞后 {lagd:.0f}ms(r={rd:.2f})，"
                      f"差值 mean={float(np.nanmean(diff)):+.3f}m std={float(np.nanstd(diff)):.3f}m")
                print("   （std 大于 2~3cm 说明定高源不一致；滞后大则高度环会有相位滞后）\n")
    else:
        print("3) 日志里没有 distance_sensor\n")

    # 4) 水平环工作点
    if sp is not None:
        ts = col(sp, "timestamp") / 1e6
        m = (ts > t[hover].min()) & (ts < t[hover].max())
        spvx, spvy = col(sp, "vx")[m], col(sp, "vy")[m]
        avx = np.interp(ts[m], t, vx)
        avy = np.interp(ts[m], t, vy)
        print("4) 悬停段水平速度（m/s）")
        print(f"   指令 : mean=({np.nanmean(spvx):+.3f},{np.nanmean(spvy):+.3f}) "
              f"std=({np.nanstd(spvx):.3f},{np.nanstd(spvy):.3f})")
        print(f"   实际 : mean=({np.nanmean(avx):+.3f},{np.nanmean(avy):+.3f}) "
              f"std=({np.nanstd(avx):.3f},{np.nanstd(avy):.3f})")
        print(f"   实际速度模长 mean={float(np.nanmean(np.hypot(avx, avy))):.3f} m/s")

    # 姿态：稳态悬停时若有恒定倾角，说明存在真实水平力偏置（重心/推力不平衡/风）
    att = get(log, "vehicle_attitude")

    if att is not None:
        ta = col(att, "timestamp") / 1e6
        w, x, y, zq = [col(att, f"q[{i}]") for i in range(4)]
        roll = np.degrees(np.arctan2(2 * (w * x + y * zq), 1 - 2 * (x * x + y * y)))
        pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - zq * x), -1, 1)))
        tb = t[hover] - t[0]

        if tb.size:
            m = (ta - ta[0] >= tb.min()) & (ta - ta[0] <= tb.max())

            if m.sum() > 5:
                print(f"4b) 悬停姿态：roll mean={np.nanmean(roll[m]):+.2f}° "
                      f"std={np.nanstd(roll[m]):.2f}° | pitch mean={np.nanmean(pitch[m]):+.2f}° "
                      f"std={np.nanstd(pitch[m]):.2f}°")
                print("   （恒定倾角 ≈ arctan(水平加速度/g)：1° ≈ 0.17m/s² 的等效水平力；"
                      "std 大 = 在来回摆）\n")

    # 5) 相关飞控参数（从日志头部参数表读，免去停链路）
    keys = ["MPC_THR_HOVER", "MPC_THR_MIN", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC",
            "MPC_XY_VEL_D_ACC", "MPC_XY_VEL_MAX", "MPC_ACC_HOR", "MPC_Z_VEL_P_ACC",
            "MPC_Z_VEL_I_ACC", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_XY_P",
            "EKF2_EV_CTRL", "EKF2_EV_DELAY", "EKF2_EV_NOISE_MD", "EKF2_EV_POS_X",
            "EKF2_EV_POS_Y", "EKF2_EV_POS_Z", "EKF2_EV_VEL", "EKF2_EV_ANG",
            "EKF2_HGT_REF", "EKF2_RNG_CTRL", "EK2_RNG_NOISE"]

    print("\n5) 相关参数")

    for k in keys:
        if k in params:
            print(f"   {k:<20} = {params[k]:g}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
