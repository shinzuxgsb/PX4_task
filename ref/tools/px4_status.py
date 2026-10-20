#!/usr/bin/env python3
"""读取 PX4 的实时状态（位置/速度/估计器标志），用于台架诊断。

    python3 px4_status.py /dev/ttyS6 --seconds 8
"""

from __future__ import annotations

import argparse
import os
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

EST_FLAGS = {
    1: "ATT", 2: "VEL_H", 4: "VEL_V", 8: "POS_H_REL", 16: "POS_H_ABS",
    32: "POS_V_ABS", 64: "POS_V_AGL", 128: "CONST_POS", 256: "PRED_H_REL",
    512: "PRED_H_ABS",
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyS6")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=255, source_component=191, autoreconnect=False
    )
    print(f"等待心跳 ... ({args.port} @ {args.baud})")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print("没有心跳：链路未通或串口被占用")
        return 1
    print(f"链路 OK: sysid={hb.get_srcSystem()}\n")

    deadline = time.time() + args.seconds
    last_print = 0.0
    while time.time() < deadline:
        msg = conn.recv_match(
            type=["LOCAL_POSITION_NED", "ESTIMATOR_STATUS", "SYS_STATUS", "ALTITUDE", "VFR_HUD"],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue
        now = time.time()
        if now - last_print < 1.0:
            continue
        last_print = now
        if msg.get_type() == "LOCAL_POSITION_NED":
            print(
                f"POS  x={msg.x:+8.3f} y={msg.y:+8.3f} z={msg.z:+9.3f} m  |  "
                f"V  x={msg.vx:+7.3f} y={msg.vy:+7.3f} z={msg.vz:+7.3f} m/s"
            )
        elif msg.get_type() == "ESTIMATOR_STATUS":
            flags = int(msg.flags)
            names = " ".join(n for b, n in EST_FLAGS.items() if flags & b)
            print(f"EKF  flags=0x{flags:x}  [{names}]")
        elif msg.get_type() == "ALTITUDE":
            print(
                f"ALT  amsl={msg.altitude:+8.2f} rel={msg.altitude_relative:+8.2f} "
                f"local={msg.altitude_local:+8.2f} terrain={msg.altitude_terrain:+8.2f} "
                f"clearance={msg.bottom_clearance:+6.2f} m"
            )
        elif msg.get_type() == "VFR_HUD":
            print(f"HUD  alt={msg.alt:+8.2f} m  climb={msg.climb:+6.2f} m/s")

    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
