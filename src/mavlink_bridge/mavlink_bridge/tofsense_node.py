#!/usr/bin/env python3
"""TOFSense-M 读取节点：解析 NLink 帧，发布 sensor_msgs/Range。

NLink 帧（112 字节，4x4 模式）：
  0    : 0x57 帧头      1: 0x01 帧标记(M 系列)   3: id
  4-7  : system_time uint32 小端 (ms)           8: zone_map
  9+6i : distance 3 字节 **小端**, 单位 µm -> /1e6 = 米
  12+6i: status        13+6i: signal strength
  111  : checksum（前 111 字节之和 & 0xFF）
"""

from __future__ import annotations

import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import serial
except ImportError:  # pragma: no cover
    raise SystemExit("缺少 pyserial，请执行: python3 -m pip install pyserial")

HEADER = 0x57
MARK_M = 0x01
FRAME_LEN = 112
ZONES = 16


def find_frames(buf: bytearray) -> list[bytes]:
    """从缓冲区里切出所有校验通过的帧（就地删除已消费数据）。"""
    frames = []
    while len(buf) >= FRAME_LEN:
        idx = buf.find(bytes([HEADER, MARK_M]))
        if idx < 0:
            del buf[: max(0, len(buf) - 1)]
            break
        if idx > len(buf) - FRAME_LEN:
            del buf[:idx]
            break
        frame = bytes(buf[idx : idx + FRAME_LEN])
        del buf[: idx + FRAME_LEN]
        if (sum(frame[:-1]) & 0xFF) == frame[-1]:
            frames.append(frame)
    return frames


def zone_distances(frame: bytes) -> list[float]:
    out = []
    for i in range(ZONES):
        base = 9 + i * 6
        raw = int.from_bytes(frame[base : base + 3], "little")   # µm
        out.append(raw / 1e6)
    return out


class TofsenseNode(Node):
    def __init__(self) -> None:
        super().__init__("tofsense")

        self.declare_parameter("port", "/dev/ttyS3")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("topic", "/tofsense/range")
        self.declare_parameter("frame_id", "tofsense")
        self.declare_parameter("zone_select", "median")   # median | min | center
        self.declare_parameter("min_range", 0.02)         # m
        self.declare_parameter("max_range", 5.0)          # m
        self.declare_parameter("field_of_view", 0.35)     # rad
        self.declare_parameter("publish_rate", 50.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.zone_select = str(self.get_parameter("zone_select").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)

        self.pub = self.create_publisher(Range, str(self.get_parameter("topic").value), 10)
        self._lock = threading.Lock()
        self._latest: float | None = None
        self._frames = 0
        self._bad = 0

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, name="tofsense-rx", daemon=True)
        self._thread.start()

        rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(5.0, self._log)
        self.get_logger().info(
            f"tofsense: {self.port} @ {self.baud}, zone_select={self.zone_select}"
        )

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set() and rclpy.ok():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                    self.get_logger().info(f"串口已打开: {self.port}")
                    while not self._stop.is_set() and rclpy.ok():
                        chunk = ser.read(4096)
                        if chunk:
                            buf.extend(chunk)
                            for frame in find_frames(buf):
                                self._on_frame(frame)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"串口异常，2 秒后重试: {exc}")
                time.sleep(2.0)

    def _on_frame(self, frame: bytes) -> None:
        dists = [d for d in zone_distances(frame) if self.min_range <= d <= self.max_range]
        if not dists:
            self._bad += 1
            return
        self._frames += 1
        if self.zone_select == "min":
            value = min(dists)
        elif self.zone_select == "center":
            value = sum(dists) / len(dists)
        else:  # median
            s = sorted(dists)
            n = len(s)
            value = s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])
        with self._lock:
            self._latest = value

    def _publish(self) -> None:
        with self._lock:
            value = self._latest
        if value is None:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(self.get_parameter("field_of_view").value)
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = float(value)
        self.pub.publish(msg)

    def _log(self) -> None:
        with self._lock:
            value = self._latest
        self.get_logger().info(
            f"距离={value * 100:.1f}cm  (有效帧 {self._frames}, 无有效分区 {self._bad})"
            if value is not None
            else f"还没有数据（有效帧 {self._frames}）"
        )

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TofsenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
