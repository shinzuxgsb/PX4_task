#!/usr/bin/env python3
"""通过 MAVLink shell 在飞控上执行 nsh 命令（只读诊断用）。

    python3 px4_shell.py /dev/ttyS6 "dmesg | grep -i dist" "status"
"""

from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

DEV_SHELL = mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL
FLAG_EXCLUSIVE = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
FLAG_RESPOND = mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND
FLAG_MULTI = mavutil.mavlink.SERIAL_CONTROL_FLAG_MULTI
DATA_LEN = 70


def send_shell(conn, text: str) -> None:
    payload = (text + "\n").encode("ascii", "replace")
    buf = list(payload[:DATA_LEN]) + [0] * max(0, DATA_LEN - len(payload))
    conn.mav.serial_control_send(
        DEV_SHELL, FLAG_EXCLUSIVE | FLAG_RESPOND | FLAG_MULTI, 0, 0, len(payload[:DATA_LEN]), buf[:DATA_LEN]
    )


def drain(conn, seconds: float) -> str:
    out = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        msg = conn.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.device != DEV_SHELL or msg.count == 0:
            continue
        raw = bytes(msg.data[: msg.count])
        out.append(raw.decode("utf-8", "replace"))
    return "".join(out)


def main() -> int:
    args = sys.argv[1:]
    port = args[0] if args and args[0].startswith("/") else "/dev/ttyS6"
    cmds = args[1:] if args and args[0].startswith("/") else args
    if not cmds:
        cmds = ["status"]
    baud = int(os.environ.get("PX4_BAUD", "115200"))

    conn = mavutil.mavlink_connection(
        port, baud=baud, source_system=255, source_component=191, autoreconnect=False
    )
    print(f"等待心跳 ... ({port} @ {baud})")
    if conn.wait_heartbeat(timeout=10) is None:
        print("没有心跳：链路未通或串口被占用")
        return 1

    # 先敲个回车唤醒 shell
    send_shell(conn, "")
    drain(conn, 1.0)

    for cmd in cmds:
        print(f"\n$ {cmd}")
        send_shell(conn, cmd)
        text = drain(conn, 3.0)
        for line in text.replace("\r", "").splitlines():
            line = line.rstrip()
            if line and not line.startswith("nsh>"):
                print(line)

    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
