#!/usr/bin/env python3
"""导出 PX4 全部参数（刷固件前的备份）。

用法（在香橙派上）：
    python3 px4_param_dump.py [串口] [输出文件]
    python3 px4_param_dump.py /dev/ttyS6 /tmp/px4_params_backup.txt

注意：PX4 把非 REAL32 参数按位塞进 MAVLink 的 float 字段，需要还原（与 px4_param.py 同一套逻辑）。
运行前先停掉占用串口的 mavlink_bridge（systemctl stop drone-link）。
"""

from __future__ import annotations

import os
import struct
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402


def decode_value(msg) -> str:
    raw = msg.param_value
    ptype = getattr(msg, "param_type", 9)

    if ptype == 9:  # MAV_PARAM_TYPE_REAL32
        return f"{raw:g}"

    bits = struct.unpack("<i", struct.pack("<f", raw))[0]

    if ptype == 8:  # 8 位整数，PX4 仍以 int32 位模式给出
        return f"{bits}"

    return f"{bits}"


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS6"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/px4_params_backup.txt"
    baud = int(os.environ.get("PX4_BAUD", "115200"))

    conn = mavutil.mavlink_connection(
        port, baud=baud, source_system=255, source_component=191, autoreconnect=False
    )
    print(f"等待心跳 ... ({port} @ {baud})")

    hb = conn.wait_heartbeat(timeout=10)

    if hb is None:
        print("没有收到心跳：链路未通，或串口被其他进程占用")
        conn.close()
        return 1

    sysid = hb.get_srcSystem()
    conn.mav.param_request_list_send(sysid, hb.get_srcComponent())
    print("已请求全部参数，收集中 ...")

    params: dict[str, str] = {}
    expected = 0
    deadline = time.time() + 60

    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)

        if msg is None:
            if params:
                break
            continue

        name = msg.param_id.strip("\x00")
        params[name] = decode_value(msg)
        expected = max(expected, msg.param_count)

        if expected and len(params) >= expected:
            break

    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(f"# PX4 参数备份  {time.strftime('%Y-%m-%d %H:%M:%S')}  来源={port}\n")

        for name in sorted(params):
            fh.write(f"{name}\t{params[name]}\n")

    print(f"已保存 {len(params)} / {expected} 个参数到 {out_path}")
    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
