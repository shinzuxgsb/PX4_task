#!/usr/bin/env python3
# =====================================================================
#  tf_to_pose.py  ——  Cartographer 的 TF  ->  /tracked_pose
#
#  背景：这版 cartographer_ros(Humble apt 包)没有 publish_tracked_pose
#        参数，所以不会自己发 /tracked_pose；但它一定会发 TF
#        （map -> odom -> base_link）。
#
#  本脚本把 TF 里的 map -> base_link 变换转成 geometry_msgs/PoseStamped
#  发到 /tracked_pose，px4_ev_bridge 就能照常订阅（一行都不用改）。
#
#  用法：
#      python3 tf_to_pose.py
#      python3 tf_to_pose.py --ros-args -p parent_frame:=map -p child_frame:=base_link
# =====================================================================

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class TfToPose(Node):
    def __init__(self):
        super().__init__('tf_to_pose')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('out_topic', '/tracked_pose')
        self.declare_parameter('rate', 30.0)

        self.parent = self.get_parameter('parent_frame').value
        self.child = self.get_parameter('child_frame').value
        self.declare_parameter('stale_timeout', 0.3)
        rate = float(self.get_parameter('rate').value)
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter('out_topic').value, 10)

        self.warned = False
        # —— 防冻结（2026-10-08 事故修复）——
        self.last_tf_key = None          # 上一帧 TF 的时间戳 (sec, nanosec)
        self.last_tf_change = self.get_clock().now()
        self.stale = False
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.tick)
        self.get_logger().info(
            f'等待 TF {self.parent} -> {self.child}，转成 {self.get_parameter("out_topic").value}')

    def tick(self):
        try:
            tf = self.buffer.lookup_transform(self.parent, self.child, Time())
        except Exception as exc:  # noqa: BLE001
            if not self.warned:
                self.get_logger().warn(f'还没有 {self.parent} -> {self.child} 的 TF：{exc}')
                self.warned = True
            return

        if self.warned:
            self.get_logger().info('TF 已就绪，开始转发位姿')
            self.warned = False

        # —— 防冻结（2026-10-08 事故修复）——
        # lookup_transform(..., Time()) 取的是"缓冲区里最新的一帧"。上游 cartographer
        # 挂掉后这个调用**依然会成功**，只是反复返回同一帧旧变换 —— 于是本脚本会以
        # 30Hz 无限重发一个冻结的位姿，飞控拿到后以为飞机一直停在原地（= 2026-10-08
        # 定点模式右飘事故的根因）。这里只看"时间戳有没有在变"：
        # 注意 tf.header.stamp 与本机时钟存在固定偏移（实测约 +1.1s），
        # 所以不能用它跟 now() 比大小，只能比较"是否变化"。
        key = (tf.header.stamp.sec, tf.header.stamp.nanosec)
        now = self.get_clock().now()
        if key != self.last_tf_key:
            self.last_tf_key = key
            self.last_tf_change = now
            if self.stale:
                self.get_logger().info('上游 TF 恢复更新，继续转发位姿')
                self.stale = False
        else:
            age = (now - self.last_tf_change).nanoseconds * 1e-9
            if age > self.stale_timeout:
                if not self.stale:
                    self.get_logger().error(
                        f'上游 TF 已 {age:.1f}s 没有更新（cartographer 挂了？）'
                        f'—— 停止发布位姿，让飞控进入位置失效保护')
                    self.stale = True
                return

        msg = PoseStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.parent
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TfToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
