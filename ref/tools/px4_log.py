#!/usr/bin/env python3
"""通过 MAVLink 在飞控上列日志 / 下载日志（PX4 的 LOG 协议，QGC 用的同一套）。

用法（在香橙派上，先停 drone-link 释放串口）：
    python3 px4_log.py list [串口]
    python3 px4_log.py download <日志ID> [输出文件] [串口]
    python3 px4_log.py info <ulg文件>          # 用 pyulog 打印日志里的话题

注意：TELEM3 是 115200，约 11 KB/s，1 MB 日志 ≈ 1.5 分钟。
"""

from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

CHUNK = 0xEF  # 单次请求字节数（QGC 用的值）


def connect(port: str, baud: int = 115200):
    conn = mavutil.mavlink_connection(
        port, baud=baud, source_system=255, source_component=191, autoreconnect=False
    )
    print(f"等待心跳 ... ({port} @ {baud})")
    hb = conn.wait_heartbeat(timeout=10)

    if hb is None:
        print("没有心跳：链路未通或串口被占用（先 systemctl stop drone-link）")
        return None

    return conn


def list_logs(conn) -> list:
    conn.mav.log_request_list_send(conn.target_system, conn.target_component, 0, 0xFFFF)
    entries = {}
    deadline = time.time() + 15

    while time.time() < deadline:
        msg = conn.recv_match(type="LOG_ENTRY", blocking=True, timeout=1)

        if msg is None:
            if entries:
                break
            continue

        entries[msg.id] = msg
        print(f"收到 {len(entries)}/{msg.num_logs}", end="\r")

        if len(entries) >= msg.num_logs:
            break

    print()
    return [entries[k] for k in sorted(entries)]


def cmd_list(port: str) -> int:
    conn = connect(port)

    if conn is None:
        return 1

    entries = list_logs(conn)
    conn.close()

    if not entries:
        print("飞控上没有日志（SD 卡没插？logger 没开？）")
        return 1

    print(f"{'ID':>4}  {'UTC 时间':<20} {'大小':>10}  预计下载")

    for e in entries:
        utc = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(e.time_utc)) if e.time_utc else "?"
        est_min = e.size / 11000.0 / 60.0
        print(f"{e.id:>4}  {utc:<20} {e.size/1024/1024:>7.2f} MB  {est_min:>5.1f} 分钟")

    return 0


def cmd_download(port: str, log_id: int, out_path: str) -> int:
    conn = connect(port)

    if conn is None:
        return 1

    entries = {e.id: e for e in list_logs(conn)}
    entry = entries.get(log_id)

    if entry is None:
        print(f"没有 ID={log_id} 的日志")
        conn.close()
        return 1

    total = entry.size
    print(f"下载 log {log_id}：{total/1024/1024:.2f} MB -> {out_path}")
    buf = bytearray()
    offset = 0
    last_report = time.time()
    start = time.time()

    with open(out_path, "wb") as fh:
        while offset < total:
            count = min(CHUNK, total - offset)
            conn.mav.log_request_data_send(
                conn.target_system, conn.target_component, log_id, offset, count
            )

            got = None
            deadline = time.time() + 3

            while time.time() < deadline:
                msg = conn.recv_match(type="LOG_DATA", blocking=True, timeout=0.5)

                if msg is not None and msg.id == log_id and msg.ofs == offset:
                    got = msg
                    break

            if got is None:
                print(f"\noffset {offset} 超时，重试")
                time.sleep(0.5)
                continue

            fh.write(bytes(got.data[:got.count]))
            offset += got.count

            if time.time() - last_report > 5:
                pct = 100.0 * offset / total
                speed = offset / max(1e-6, time.time() - start)
                print(f"  {pct:5.1f}%  {offset/1024/1024:.2f}/{total/1024/1024:.2f} MB  {speed/1024:.1f} KB/s", end="\r")
                last_report = time.time()

    conn.mav.log_request_end_send(conn.target_system, conn.target_component)
    conn.close()
    print(f"\n完成：{out_path}（{os.path.getsize(out_path)/1024/1024:.2f} MB）")
    return 0


def cmd_info(path: str) -> int:
    from pyulog import ULog  # noqa: PLC0415

    log = ULog(path)
    print(f"{path}: {len(log.data_list)} 个话题，时长 {log.start_timestamp/1e6:.1f}s")
    interesting = ("vehicle_status", "vehicle_local_position", "vehicle_attitude", "vehicle_odometry",
                   "manual_control_setpoint", "estimator_status", "estimator_innovations",
                   "sensor_combined", "distance_sensor", "actuator_controls", "rc_channels")

    for d in sorted(log.data_list, key=lambda x: x.name):
        if d.name in interesting or "position" in d.name or "odometry" in d.name:
            print(f"  {d.name:<38} 采样 {len(d.data['timestamp'])} 次")

    return 0


def main() -> int:
    args = sys.argv[1:]

    if not args:
        print(__doc__)
        return 1

    what = args[0]

    if what == "list":
        return cmd_list(args[1] if len(args) > 1 else "/dev/ttyS6")

    if what == "download":
        if len(args) < 2:
            print("用法: px4_log.py download <日志ID> [输出文件] [串口]")
            return 1

        log_id = int(args[1])
        out = args[2] if len(args) > 2 else f"/root/px4/logs/log_{log_id:03d}.ulg"
        port = args[3] if len(args) > 3 else "/dev/ttyS6"
        os.makedirs(os.path.dirname(out), exist_ok=True)
        return cmd_download(port, log_id, out)

    if what == "info":
        if len(args) < 2:
            print("用法: px4_log.py info <ulg文件>")
            return 1

        return cmd_info(args[1])

    print(__doc__)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
