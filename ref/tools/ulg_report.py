#!/usr/bin/env python3
"""把 PX4 的 .ulg 日志汇总成 JSON，供笔记本网页控制站展示「飞行记录」。

注意：PX4 把外部定位源统一叫 "External Vision"（日志字段 `estimator_innovations.ev_*`），
本项目的外部定位源是 **2D 雷达 + Cartographer SLAM**，飞机上没有摄像头。
因此这里输出 `external_pose_fused` 表示「外部定位（雷达）的测量是否被 EKF 实际使用」。

用法：
    python3 ulg_report.py <file.ulg>            # 输出 JSON
    python3 ulg_report.py --list <目录>          # 列出目录下所有 .ulg 的汇总
"""

from __future__ import annotations

import json
import os
import sys

# nav_state 枚举取自本机固件源码 msg/VehicleStatus.msg（PX4 v1.15.4），别用记忆里的旧表
NAV = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
    6: "POSITION_SLOW", 10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD",
    15: "STABILIZED", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND", 19: "AUTO_FOLLOW",
    20: "AUTO_PRECLAND", 21: "ORBIT",
}


def report(path: str) -> dict:
    import numpy as np
    from pyulog import ULog

    log = ULog(path)
    data = {}

    for d in log.data_list:
        if "timestamp" in d.data and len(d.data["timestamp"]):
            data.setdefault(d.name, d.data)

    # 时长按核心话题算：有些话题会带一条时间戳为 0（或极早）的样本，
    # 用它做 min 会把 11 秒的飞行算成 22 分钟。
    t0 = t1 = None

    for name in ("vehicle_status", "vehicle_local_position", "sensor_combined"):
        core = data.get(name)

        if core is not None:
            s = np.asarray(core["timestamp"], dtype=float)
            s = s[s > 0]

            if s.size:
                t0, t1 = float(s.min()), float(s.max())
                break

    if t0 is None:
        stamps = [np.asarray(d["timestamp"], dtype=float) for d in data.values()]
        stamps = [s[s > 0] for s in stamps]
        stamps = [s for s in stamps if s.size]

        if stamps:
            t0, t1 = min(s.min() for s in stamps), max(s.max() for s in stamps)
        else:
            t0 = t1 = 0.0
    out = {
        "file": os.path.basename(path),
        "size_mb": round(os.path.getsize(path) / 1e6, 2),
        "duration_s": round((t1 - t0) / 1e6, 1),
        "mtime": int(os.path.getmtime(path)),
        "topics": len(data),
    }

    # 模式 / 解锁
    vs = data.get("vehicle_status")

    if vs is not None:
        nav = np.asarray(vs.get("nav_state", []))
        arm = np.asarray(vs.get("arming_state", []))
        modes = {}

        for value in nav:
            name = NAV.get(int(value), str(int(value)))
            modes[name] = modes.get(name, 0) + 1

        out["modes"] = dict(sorted(modes.items(), key=lambda kv: -kv[1])[:4])
        out["armed_pct"] = round(100.0 * float(np.sum(arm == 2)) / max(1, len(arm)), 1)

    # 高度 / 水平位置 / 外部定位（雷达）是否被融合
    vlp = data.get("vehicle_local_position")

    if vlp is not None:
        z = np.asarray(vlp["z"], dtype=float)
        out["max_height_m"] = round(float(-np.nanmin(z)), 2) if len(z) else 0.0

        for key in ("xy_valid", "v_xy_valid"):
            if key in vlp:
                a = np.asarray(vlp[key])
                out[key + "_pct"] = round(100.0 * float(np.sum(a)) / max(1, len(a)), 1)

        if "eph" in vlp:
            out["max_eph_m"] = round(float(np.nanmax(np.asarray(vlp["eph"], dtype=float))), 2)

        x = np.asarray(vlp["x"], dtype=float)
        y = np.asarray(vlp["y"], dtype=float)

        if len(x):
            out["travel_m"] = round(float(np.hypot(x.max() - x.min(), y.max() - y.min())), 2)

    inn = data.get("estimator_innovations")

    if inn is not None:
        fused = 0.0

        for key in ("ev_hpos[0]", "ev_hvel[0]"):
            if key in inn:
                fused = max(fused, float(np.nanmax(np.abs(np.asarray(inn[key], dtype=float)))))

        out["external_pose_fused"] = fused > 1e-9
        out["ev_max_innovation"] = round(fused, 4)

    out["verdict"] = _verdict(out)
    return out


def _verdict(info: dict) -> str:
    if info.get("armed_pct", 0) <= 0:
        return "未解锁（台架会话）"

    if info.get("xy_valid_pct", 0) < 50 or info.get("external_pose_fused") is False:
        return "⚠ 该次飞行没有有效外部定位（雷达），水平位置靠惯导推算"

    if info.get("max_height_m", 0) > 0.3:
        return "正常飞行（外部定位在线）"

    return "解锁但未离地"


def main() -> int:
    args = sys.argv[1:]

    if not args:
        print(__doc__)
        return 1

    if args[0] == "--list":
        root = args[1] if len(args) > 1 else "."
        files = sorted(
            (os.path.join(root, f) for f in os.listdir(root) if f.endswith(".ulg")),
            key=os.path.getmtime, reverse=True)
        out = []

        for path in files:
            try:
                out.append(report(path))
            except Exception as exc:                       # noqa: BLE001
                out.append({"file": os.path.basename(path), "error": str(exc)})

        print(json.dumps(out, ensure_ascii=False))
        return 0

    try:
        print(json.dumps(report(args[0]), ensure_ascii=False))
    except Exception as exc:                               # noqa: BLE001
        print(json.dumps({"file": os.path.basename(args[0]), "error": str(exc)}, ensure_ascii=False))
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
