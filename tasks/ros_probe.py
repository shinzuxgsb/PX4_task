#!/usr/bin/env python3
"""ROS 2 话题探针：打印发布者数量，并统计 N 秒内收到的消息条数。

    python3 ros_probe.py [秒数]

用来定位"任务脚本收不到话题"到底是发现（discovery）问题还是代码问题。
"""

from __future__ import annotations

import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16, UInt8


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("ros_probe")
        self.counts = {"/odom_from_slam": 0, "/height": 0, "/is_st_ready": 0, "/target_velocity": 0}
        self.last = {}
        self.create_subscription(Odometry, "/odom_from_slam", lambda m: self._hit("/odom_from_slam", m), 10)
        self.create_subscription(Int16, "/height", lambda m: self._hit("/height", m), 10)
        self.create_subscription(UInt8, "/is_st_ready", lambda m: self._hit("/is_st_ready", m), 10)

    def _hit(self, topic: str, msg) -> None:
        self.counts[topic] += 1
        self.last[topic] = msg


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = Probe()
    t0 = time.time()
    reported = 0.0

    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

        if time.time() - reported >= 2.0:
            reported = time.time()
            line = " | ".join(
                f"{t}: pubs={node.count_publishers(t)} msgs={node.counts[t]}" for t in node.counts)
            print(f"[probe] {line}", flush=True)

    print("\n[probe] 小结：")

    for topic, count in node.counts.items():
        pubs = node.count_publishers(topic)
        value = ""

        if topic == "/height" and count:
            value = f"  最新值={node.last[topic].data}cm"
        elif topic == "/is_st_ready" and count:
            value = f"  最新值={node.last[topic].data}"
        elif topic == "/odom_from_slam" and count:
            p = node.last[topic].pose.pose.position
            value = f"  最新位置=({p.x:+.2f},{p.y:+.2f})m"

        print(f"  {topic:<18} 发布者={pubs}  收到={count} 条{value}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
