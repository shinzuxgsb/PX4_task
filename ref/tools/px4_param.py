#!/usr/bin/env python3
"""从香橙派侧读取 PX4 参数（只读，不改动飞控状态）。

用法：
    python3 px4_param.py [串口] 参数名1 参数名2 ...
    python3 px4_param.py /dev/ttyS6 EKF2_EV_CTRL EKF2_OF_CTRL

注意：同一个串口同时只能被一个进程占用，运行前需要先停掉 mavlink_bridge。
"""

from __future__ import annotations

import os
import struct
import sys
import time

# ODOMETRY 等消息需要 MAVLink v2 方言，且必须在导入前设置
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402


def decode_value(msg) -> str:
    """PX4 把非 REAL32 参数按位塞进 float 字段，需要按 param_type 还原。"""
    raw = msg.param_value
    ptype = getattr(msg, "param_type", 9)
    # MAV_PARAM_TYPE_REAL32 = 9，其余按整数位模式解释
    if ptype == 9:
        return f"{raw:g}"
    bits = struct.unpack("<i", struct.pack("<f", raw))[0]
    if ptype == 8:  # INT8/UINT8 等仍按 int32 位模式给出即可
        pass
    return f"{bits}"


def read_params(port: str, baud: int, names: list[str], timeout_s: float = 3.0) -> int:
    conn = mavutil.mavlink_connection(
        port, baud=baud, source_system=255, source_component=191, autoreconnect=False
    )
    print(f"等待心跳 ... ({port} @ {baud})")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print("没有收到心跳：链路未通，或串口被其他进程占用")
        conn.close()
        return 1
    print(f"链路 OK: sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}\n")

    failures = 0
    for name in names:
        conn.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
        deadline = time.time() + timeout_s
        found = None
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", "replace")
            pid = pid.rstrip("\x00")
            if pid == name:
                found = msg
                break
        if found is None:
            print(f"{name:<24} = <无响应>")
            failures += 1
        else:
            ptype = getattr(found, "param_type", 9)
            suffix = "" if ptype == 9 else f"   [int32 位模式, type={ptype}]"
            print(f"{name:<24} = {decode_value(found)}{suffix}")

    conn.close()
    return 0 if failures == 0 else 2


def main() -> int:
    args = sys.argv[1:]
    port = args[0] if args and args[0].startswith("/") else "/dev/ttyS6"
    names = args[1:] if args and args[0].startswith("/") else args
    if not names:
        names = ["EKF2_EV_CTRL"]
    baud = int(os.environ.get("PX4_BAUD", "115200"))
    return read_params(port, baud, names)


if __name__ == "__main__":
    raise SystemExit(main())
