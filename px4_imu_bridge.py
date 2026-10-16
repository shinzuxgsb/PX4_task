#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PX4 飞控 IMU -> sensor_msgs/Imu（喂给 cartographer 2D 做旋转预测）。

背景（2026-10-09 新增）
----------------------
cartographer 的 2D SLAM 之前**完全不用 IMU**（`TRAJECTORY_BUILDER_2D.use_imu_data = false`），
位姿只能靠 scan matching 猜。机身一倾斜，激光扫描平面跟着倾斜，扫到的不再是建图时的水平切片，
匹配会跑到错误的角度/位置上去（`CHANGELOG.md` 2026-10-08 实测：倾斜 30° → yaw 摆 29°、
水平位置摆 1.26 m）。而**陀螺的角速度不受"扫描平面倾斜"影响**，把它喂给 cartographer 后，
旋转预测由陀螺提供、scan matching 只在很小的角度窗口里找 → 航向不再被倾斜带跑。

数据源：飞控通过 uXRCE-DDS 自己发布的 `/fmu/out/sensor_combined`
        （`px4_msgs/SensorCombined`：`gyro_rad[3]` rad/s、`accelerometer_m_s2[3]` m/s²，
          两者都是 **FRD 机体系**：x 前 / y 右 / z 下）。
        该话题在 PX4 1.15.4 的默认 DDS 发布列表里（`dds_topics.yaml`），**不需要改固件**。

输出：`sensor_msgs/Imu`，默认话题 `/imu`
      —— cartographer_ros 订阅的相对话题名就是 `imu`，节点命名空间是 `/` 时即 `/imu`。
      `frame_id = laser_link` —— **必须和 cartographer 的 tracking_frame 一致**！
      （2026-10-10 修：原来发 imu_link，而 URDF 里 imu_link 在 base 上方 0.05m、
        laser_link 在 0.1m，两者差 5cm → cartographer 的 sensor_bridge.cpp:136
        直接 CHECK 失败：
          Check failed: sensor_to_tracking->translation().norm() < 1e-5
          The IMU frame must be colocated with the tracking frame
        2D 只拿 IMU 测转速，5cm 平移可忽略；标成 laser_link 后 translation=0，不再崩。）

坐标系（关键，错了航向会反）
--------------------------
  PX4 机体系是 **FRD**（x 前、y 右、z 下）；ROS / cartographer 用 **FLU**（x 前、y 左、z 上）。

      FLU = (x, -y, -z)        ← 陀螺和加速度都要这么转

  转对之后：机体绕 z 轴逆时针（俯视）转时 `angular_velocity.z` 为正，
  与 cartographer 2D 的 yaw 方向一致；水平静止时 `linear_acceleration.z ≈ +9.81`。

时间戳
------
默认用**收到消息的时刻**（本机 ROS 时钟），不用 PX4 自带的 timestamp。
原因：PX4 timestamp 是"开机后微秒"，要换算成 ROS 时间必须依赖 timesync 的偏移估计，
估偏一次就可能让时间**倒流** —— 而 cartographer 对 IMU 时间非单调是直接 CHECK 崩溃的
（二进制里有 `Check failed: timed_pose_queue_.empty() || imu_data.time >= ...`）。
代价只是几毫秒的传输延迟，对 2D 的"旋转预测"没有任何影响。

用法
----
    source /opt/ros/humble/setup.bash
    source ~/px4_ws/install/setup.bash
    python3 ~/px4_imu_bridge.py
    # 可选参数（--ros-args -p 名:=值）：
    #   input_topic  output_topic  frame_id  publish_hz  frd_to_flu  log_period_sec

自检（飞控必须上电、DDS 通）：
    ros2 topic hz /imu                 # 期望 ~100 Hz（publish_hz 上限）
    手动把机身绕竖直轴转，看 /tracked_pose 的 yaw 是否同向变化、且不再乱跳
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu


class Px4ImuBridge(Node):
    def __init__(self):
        super().__init__('px4_imu_bridge')

        self.in_topic = self.declare_parameter(
            'input_topic', '/fmu/out/sensor_combined').value
        self.out_topic = self.declare_parameter('output_topic', '/imu').value
        self.frame_id = self.declare_parameter('frame_id', 'laser_link').value
        self.publish_hz = float(self.declare_parameter('publish_hz', 100.0).value)
        self.frd_to_flu = bool(self.declare_parameter('frd_to_flu', True).value)
        self.log_period = float(self.declare_parameter('log_period_sec', 5.0).value)

        # 和 px4_bridge / px4_ev_bridge 里收 /fmu/out 的写法保持一致：
        # BEST_EFFORT + VOLATILE（飞控侧就是这么发的）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)

        self.pub = self.create_publisher(Imu, self.out_topic, qos)
        self.sub = self.create_subscription(
            SensorCombined, self.in_topic, self.on_msg, qos)

        self.min_dt = 0.0 if self.publish_hz <= 0.0 else 1.0 / self.publish_hz
        self.last_pub = 0.0
        self.n_in = 0
        self.n_out = 0
        self.last_msg_time = 0.0
        self.last_gz = 0.0
        self.last_az = 0.0
        self.t_last_log = time.monotonic()

        self.create_timer(1.0, self.on_timer)

        self.get_logger().info(
            'IMU 桥已启动：%s -> %s（frame_id=%s，最多 %.0f Hz，FRD->FLU=%s）'
            % (self.in_topic, self.out_topic, self.frame_id,
               self.publish_hz, self.frd_to_flu))

    def on_msg(self, msg):
        self.n_in += 1
        self.last_msg_time = time.monotonic()

        if self.min_dt > 0.0 and (self.last_msg_time - self.last_pub) < self.min_dt:
            return  # 限流：飞控的 sensor_combined 通常 250 Hz，没必要全转发

        gx = float(msg.gyro_rad[0])
        gy = float(msg.gyro_rad[1])
        gz = float(msg.gyro_rad[2])
        ax = float(msg.accelerometer_m_s2[0])
        ay = float(msg.accelerometer_m_s2[1])
        az = float(msg.accelerometer_m_s2[2])

        if not all(math.isfinite(v) for v in (gx, gy, gz, ax, ay, az)):
            return

        if self.frd_to_flu:
            # FRD(x前/y右/z下) -> FLU(x前/y左/z上)
            gy, gz = -gy, -gz
            ay, az = -ay, -az

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id

        # 不提供姿态（cartographer 只用角速度和线加速度）；-1 表示"该字段无效"
        out.orientation.w = 1.0
        out.orientation_covariance[0] = -1.0

        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz
        # ⚠️ 2026-10-10 修：这里原来是 -1.0（"没有这条数据"）。但 **cartographer 需要
        # 角速度和线加速度**，它的 sensor_bridge.cpp:119 有一条硬 CHECK：
        #     Check failed: msg->linear_acceleration_covariance[0] != -1
        # 一旦收到 covariance=-1 的 IMU 就 **直接 abort（exit -6）**，
        # launch 再把它拉起来、再崩 —— 死循环，于是"永远没有 SLAM"。
        # 这正是先前只能用 fly_no_imu.lua 的真正原因。填成真实的小方差即可。
        out.angular_velocity_covariance[0] = 1e-4      # (rad/s)^2，约 0.01 rad/s 的噪声
        out.angular_velocity_covariance[4] = 1e-4
        out.angular_velocity_covariance[8] = 1e-4

        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance[0] = 1e-2   # (m/s^2)^2，约 0.1 m/s^2
        out.linear_acceleration_covariance[4] = 1e-2
        out.linear_acceleration_covariance[8] = 1e-2

        self.pub.publish(out)
        self.n_out += 1
        self.last_pub = self.last_msg_time
        self.last_gz = gz
        self.last_az = az

    def on_timer(self):
        now = time.monotonic()
        if (now - self.t_last_log) < self.log_period:
            return
        dt = now - self.t_last_log
        self.t_last_log = now

        if self.last_msg_time == 0.0:
            self.get_logger().warn(
                '还没收到 %s —— 飞控上电了吗？DDS(Agent/ttyS6) 通了吗？'
                '（IMU 数据来自飞控自身，飞控不上电这里就没有数据）' % self.in_topic)
            return

        age = now - self.last_msg_time
        if age > 3.0:
            self.get_logger().warn('%.1f 秒没收到 IMU（飞控断了？）' % age)
            return

        self.get_logger().info(
            'IMU: 收 %.0f Hz -> 发 %.0f Hz | gyro(z)=%+.3f rad/s  accel(z)=%+.2f m/s^2'
            % (self.n_in / dt, self.n_out / dt, self.last_gz, self.last_az))
        self.n_in = 0
        self.n_out = 0


def main():
    rclpy.init()
    node = Px4ImuBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 被 SIGTERM 停止（run.sh stop / systemctl stop）时 rclpy 会抛这个，
        # 不当错误处理，免得日志里出现一大段 traceback
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
