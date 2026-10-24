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
from std_msgs.msg import UInt8

try:
    import serial
except ImportError:  # pragma: no cover
    raise SystemExit("缺少 pyserial，请执行: python3 -m pip install pyserial")

HEADER = 0x57
MARK_M = 0x01
FRAME_LEN = 112
ZONES = 16
CENTER_4X4_ZONES = (5, 6, 9, 10)


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


def zone_samples(frame: bytes) -> list[tuple[float, int, int]]:
    """Return (distance_m, dis_status, signal_strength) for every zone."""
    out = []
    for i in range(ZONES):
        base = 9 + i * 6
        # NLink stores distance * 1000 in a signed int24.
        raw = int.from_bytes(frame[base : base + 3], "little", signed=True)
        status = int(frame[base + 3])
        signal = int.from_bytes(frame[base + 4 : base + 6], "little")
        out.append((raw / 1e6, status, signal))
    return out


def zone_distances(frame: bytes) -> list[float]:
    """Backward-compatible helper returning every raw zone distance."""
    return [distance for distance, _status, _signal in zone_samples(frame)]


class TofsenseNode(Node):
    def __init__(self) -> None:
        super().__init__("tofsense")

        self.declare_parameter("port", "/dev/ttyS3")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("topic", "/tofsense/range")
        self.declare_parameter("quality_topic", "/tofsense/quality")
        self.declare_parameter("frame_id", "tofsense")
        self.declare_parameter("zone_select", "center4_median")
        self.declare_parameter("valid_statuses", "0,10")
        self.declare_parameter("min_signal", 0)
        self.declare_parameter("min_valid_zones", 3)
        self.declare_parameter("max_zone_spread_m", 0.15)
        self.declare_parameter("max_sample_age_s", 0.20)
        self.declare_parameter("min_range", 0.02)         # m
        self.declare_parameter("max_range", 4.0)          # m
        self.declare_parameter("field_of_view", 0.35)     # rad
        self.declare_parameter("publish_rate", 50.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.zone_select = str(self.get_parameter("zone_select").value)
        self.valid_statuses = {
            int(value.strip())
            for value in str(self.get_parameter("valid_statuses").value).split(",")
            if value.strip()
        }
        self.min_signal = int(self.get_parameter("min_signal").value)
        self.min_valid_zones = int(self.get_parameter("min_valid_zones").value)
        self.max_zone_spread = float(self.get_parameter("max_zone_spread_m").value)
        self.max_sample_age = float(self.get_parameter("max_sample_age_s").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)

        self.pub = self.create_publisher(Range, str(self.get_parameter("topic").value), 10)
        self.quality_pub = self.create_publisher(
            UInt8, str(self.get_parameter("quality_topic").value), 10
        )
        self._lock = threading.Lock()
        self._latest: float | None = None
        self._latest_stamp = 0.0
        self._latest_quality = 0
        self._frames = 0
        self._bad = 0
        self._status_rejected = 0
        self._signal_rejected = 0
        self._incoherent = 0

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
        valid: list[tuple[int, float, int]] = []
        for zone_index, (distance, status, signal) in enumerate(zone_samples(frame)):
            if status not in self.valid_statuses:
                self._status_rejected += 1
                continue
            if not self.min_range <= distance <= self.max_range:
                self._bad += 1
                continue
            if signal < self.min_signal:
                self._signal_rejected += 1
                continue
            valid.append((zone_index, distance, signal))

        if self.zone_select == "center4_median":
            selected = [sample for sample in valid if sample[0] in CENTER_4X4_ZONES]
            if len(selected) < self.min_valid_zones:
                self._bad += 1
                return
            distances = [distance for _index, distance, _signal in selected]
            if max(distances) - min(distances) > self.max_zone_spread:
                self._incoherent += 1
                return
        else:
            if len(valid) < self.min_valid_zones:
                self._bad += 1
                return
            selected = valid
            distances = [distance for _index, distance, _signal in selected]

        if self.zone_select == "min":
            value = min(distances)
        elif self.zone_select == "center":
            value = sum(distances) / len(distances)
        else:  # median / center4_median
            distances.sort()
            middle = len(distances) // 2
            value = (
                distances[middle]
                if len(distances) % 2
                else 0.5 * (distances[middle - 1] + distances[middle])
            )

        signals = sorted(signal for _index, _distance, signal in selected)
        signal = signals[len(signals) // 2]
        self._frames += 1
        with self._lock:
            self._latest = value
            self._latest_stamp = time.monotonic()
            self._latest_quality = max(1, min(100, int(round(signal * 100.0 / 255.0))))

    def _publish(self) -> None:
        with self._lock:
            value = self._latest
            stamp = self._latest_stamp
            quality = self._latest_quality
        if value is None or time.monotonic() - stamp > self.max_sample_age:
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
        quality_msg = UInt8()
        quality_msg.data = quality
        self.quality_pub.publish(quality_msg)

    def _log(self) -> None:
        with self._lock:
            value = self._latest
            stamp = self._latest_stamp
            quality = self._latest_quality
        age_ms = (time.monotonic() - stamp) * 1000.0 if stamp else float("inf")
        self.get_logger().info(
            f"距离={value * 100:.1f}cm quality={quality} age={age_ms:.0f}ms "
            f"(有效帧 {self._frames}, 无有效分区 {self._bad}, "
            f"status拒绝 {self._status_rejected}, 信号拒绝 {self._signal_rejected}, "
            f"分区不一致 {self._incoherent})"
            if value is not None
            else (
                f"还没有数据（有效帧 {self._frames}, 无有效分区 {self._bad}, "
                f"status拒绝 {self._status_rejected}, "
                f"信号拒绝 {self._signal_rejected}, 分区不一致 {self._incoherent}）"
            )
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
