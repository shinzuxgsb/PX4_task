#!/usr/bin/env python3
"""打印 MAVLink 上所有心跳来源（sysid/compid/自动驾驶仪类型/主模式/子模式）。

用途：排查「飞控实际模式」与「地面站/桥接显示的模式」不一致——看是不是有别的心跳来源
（非自驾仪组件）在覆盖显示值。

    python3 heartbeat_dump.py [串口] [秒数]
"""

from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD",
        7: "STABILIZED", 8: "RATTITUDE"}
AUTO_SUB = {1: "AUTO_READY", 2: "AUTO_TAKEOFF", 3: "AUTO_LOITER", 4: "AUTO_MISSION",
            5: "AUTO_RTL", 6: "AUTO_LAND", 8: "AUTO_FOLLOW", 9: "AUTO_PRECLAND"}


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS6"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    conn = mavutil.mavlink_connection(port, baud=115200, source_system=255,
                                      source_component=191, autoreconnect=False)
    seen = {}
    t0 = time.time()

    while time.time() - t0 < seconds:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)

        if msg is None:
            continue

        key = (msg.get_srcSystem(), msg.get_srcComponent())
        main_mode = (int(msg.custom_mode) >> 16) & 0xFF
        sub_mode = (int(msg.custom_mode) >> 24) & 0xFF
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        state = (main_mode, sub_mode, msg.autopilot, msg.type, armed)

        if seen.get(key) == state:
            continue

        seen[key] = state
        name = MAIN.get(main_mode, f"main{main_mode}")

        if main_mode == 4:
            name += "/" + AUTO_SUB.get(sub_mode, f"sub{sub_mode}")

        print(f"sysid={key[0]:3d} compid={key[1]:3d} autopilot={msg.autopilot} mavtype={msg.type:3d} "
              f"armed={armed!s:5} 主模式={main_mode}({name}) 子模式={sub_mode}")

    print(f"共 {len(seen)} 个心跳来源")
    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
