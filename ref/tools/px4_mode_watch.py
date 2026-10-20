#!/usr/bin/env python3
"""实时观察遥控模式档位：打印心跳的模式 + RC_MAP_FLTMODE 通道的原始值 + 推算档位。

用法（在香橙派上，先停 drone-link）：
    python3 px4_mode_watch.py [串口] [秒数]

用途：验证三段开关是否真的能切出三个模式（档位算法见 PX4 rc_update.cpp：
先用 RCx_MIN/MAX/TRIM 归一化到 -1..1，再按 6 档切分）。
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
            5: "AUTO_RTL", 6: "AUTO_LAND"}


def slot_of(us: float, rc_min: float, rc_max: float, trim: float) -> int:
    """按 PX4 rc_update.cpp 的算法复算档位（1~6），value 是归一化后的 -1..1。"""
    mid = (rc_max + rc_min) / 2.0

    if rc_max <= rc_min:
        return 0

    value = (us - trim) / max(1.0, (rc_max - mid)) if (us >= trim) else (us - trim) / max(1.0, (mid - rc_min))
    value = max(-1.0, min(1.0, value))
    raw = (((value + 1.05) * 6 + 0.5) / 2.1) + 0.5 + 1
    return max(1, min(6, int(raw)))


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS6"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
    conn = mavutil.mavlink_connection(port, baud=115200, source_system=255,
                                      source_component=191, autoreconnect=False)
    conn.wait_heartbeat(timeout=10)
    conn.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    # 读一下 ch5 的标定（通过参数协议）
    for name in ("RC5_MIN", "RC5_MAX", "RC5_TRIM"):
        conn.mav.param_request_read_send(1, 1, name.encode(), -1)

    cal = {}
    mode = None
    ch5 = None
    t0 = time.time()
    last = 0.0

    while time.time() - t0 < seconds:
        msg = conn.recv_match(blocking=True, timeout=1)

        if msg is None:
            continue

        mtype = msg.get_type()

        if mtype == "PARAM_VALUE":
            cal[msg.param_id.strip("\x00")] = round(float(msg.param_value), 1)
        elif mtype == "HEARTBEAT":
            mm = (int(msg.custom_mode) >> 16) & 0xFF
            sm = (int(msg.custom_mode) >> 24) & 0xFF
            mode = MAIN.get(mm, str(mm)) + (("/" + AUTO_SUB.get(sm, str(sm))) if mm == 4 else "")
        elif mtype == "RC_CHANNELS":
            values = [getattr(msg, f"chan{i}_raw") for i in range(1, 15)]
            ch5 = values[4]

        if time.time() - last >= 1.0 and mode:
            last = time.time()
            print(f"t+{last - t0:4.0f}s  模式={mode:<18} ch5={ch5}µs  "
                  f"标定 MIN={cal.get('RC5_MIN')} MAX={cal.get('RC5_MAX')} TRIM={cal.get('RC5_TRIM')}  "
                  f"→ 第 {slot_of(ch5, cal.get('RC5_MIN', 1000), cal.get('RC5_MAX', 2000), cal.get('RC5_TRIM', 1500)) if ch5 else '?'} 档")

    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
