#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""盯「飞控 ↔ 香橙派」链路稳不稳（地面长时间静置用；排查飞行中飞控自己重启）。

2026-10-10 为什么写它
---------------------
那天飞行中飞控在 17:52:42 自己重启了一次，飞机掉下来。事后是三条独立证据拼出来的：
  ① 任务节点报 /fmu/out/vehicle_local_position 停了 > 0.5s；
  ② agent 日志里出现 `session re-established`（17:52:48）+ 重创全部 DDS 实体；
  ③ 飞控 hrt 反推的开机时刻 = 17:52:50，且 latest_arming_reason 被清零。
这个工具就是把这三条同时盯着，**在地面静置时**提前发现同类问题。

看三件事
  1. /fmu/out/vehicle_local_position 的到达间隔（正常 ~100Hz、最大间隔 ~0.02s）
     · > 0.5s：警告一次；> 1.0s：计一次"重大中断"（任务节点的看门狗阈值就是 1.0s）
  2. /fmu/out/vehicle_status 的 arming_state / nav_state 变化（重启后 arming_state 回 1）
  3. agent 日志里新增的 `session re-established` / `create_client`
     → 飞控重启或掉线重连的直接证据

用法（香橙派上，建议先 export FASTRTPS_DEFAULT_PROFILES_FILE，见 ~/.bashrc）
    python3 ~/px4_ws/tools/watch_fc_link.py                  # 一直跑，每 30s 一行小结
    python3 ~/px4_ws/tools/watch_fc_link.py --minutes 20     # 跑 20 分钟自动退出
    python3 ~/px4_ws/tools/watch_fc_link.py --agent-log /tmp/某份agent.log
"""

import _ros_env  # noqa: F401  —— 不 source 也能跑（见该文件注释）

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

DEFAULT_AGENT_LOG = '/tmp/microxrce_agent.log'
# agent 日志里出现这些就说明飞控那边重新握手了（= 飞控重启 / 串口掉线重连）
SESSION_MARKERS = ('session re-established', 'session established', 'create_client')


class WatchFcLink(Node):
    def __init__(self, agent_log: str, minutes: float):
        super().__init__('watch_fc_link')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self.on_lpos, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',
                                 self.on_status, qos)

        self.agent_log = agent_log
        self.deadline = (time.monotonic() + minutes * 60.0) if minutes > 0 else None

        self.t0 = time.monotonic()
        self.last_lpos_t = None
        self.n_lpos = 0
        self.max_gap = 0.0
        self.n_gap_warn = 0      # > 0.5s
        self.n_gap_big = 0       # > 1.0s
        self.warned_this_gap = False

        self.armed = None
        self.nav_state = None
        self.n_reset = 0         # agent 日志里的重连次数
        self.final_reported = False
        self.log_offset = self._init_offset(agent_log)

        self.create_timer(0.5, self.tick)
        self.create_timer(30.0, self.report)

        self.get_logger().info(
            '开始盯飞控链路：local_position 间隔 / 状态变化 / agent 会话重连')
        if self.log_offset < 0:
            self.get_logger().warn(f'读不到 agent 日志 {agent_log}（只能看 ROS 那两条线索）')

    # ---------------- agent 日志尾巴 ----------------
    @staticmethod
    def _init_offset(path: str) -> int:
        try:
            return os.path.getsize(path)      # 只看启动之后新增的部分
        except OSError:
            return -1

    def _tail_agent_log(self):
        if self.log_offset < 0:
            return
        try:
            size = os.path.getsize(self.agent_log)
            if size < self.log_offset:        # 文件被重建（重启链路）
                self.log_offset = 0
            if size == self.log_offset:
                return
            with open(self.agent_log, 'rb') as f:
                f.seek(self.log_offset)
                chunk = f.read().decode('utf-8', 'replace')
                self.log_offset = f.tell()
        except OSError:
            return
        for line in chunk.splitlines():
            if any(m in line for m in SESSION_MARKERS):
                self.n_reset += 1
                clean = line.replace('\x1b', '').strip()
                print(f'\n[{time.strftime("%H:%M:%S")}] ⚠️  飞控重连/重启迹象：{clean}\n',
                      flush=True)
            elif 'create_participant' in line:
                print(f'[{time.strftime("%H:%M:%S")}]     （agent 重新创建了 DDS 参与者)\n',
                      flush=True)

    # ---------------- ROS 回调 ----------------
    def on_lpos(self, _msg):
        now = time.monotonic()
        if self.last_lpos_t is not None:
            gap = now - self.last_lpos_t
            self.max_gap = max(self.max_gap, gap)
            if gap > 1.0:
                self.n_gap_big += 1
                self.n_gap_warn += 1
                self.warned_this_gap = True
                print(f'\n[{time.strftime("%H:%M:%S")}] ‼️  vehicle_local_position 断了 {gap:.2f}s'
                      f'（>1.0s：任务节点这时会主动中止任务）\n', flush=True)
            elif gap > 0.5 and not self.warned_this_gap:
                self.n_gap_warn += 1
                self.warned_this_gap = True
                print(f'\n[{time.strftime("%H:%M:%S")}] ⚠️  vehicle_local_position 停了 {gap:.2f}s'
                      f'（未到 1.0s 阈值）\n', flush=True)
            elif gap < 0.2:
                self.warned_this_gap = False
        self.last_lpos_t = now
        self.n_lpos += 1

    def on_status(self, msg: VehicleStatus):
        if self.armed is not None and msg.arming_state != self.armed:
            print(f'[{time.strftime("%H:%M:%S")}] 解锁状态变化：{self.armed} → {msg.arming_state}',
                  flush=True)
        if self.nav_state is not None and msg.nav_state != self.nav_state:
            print(f'[{time.strftime("%H:%M:%S")}] 飞行模式变化：nav_state {self.nav_state} → {msg.nav_state}',
                  flush=True)
        self.armed = msg.arming_state
        self.nav_state = msg.nav_state

    # ---------------- 周期任务 ----------------
    def tick(self):
        self._tail_agent_log()
        if self.deadline is not None and time.monotonic() >= self.deadline:
            self.report(final=True)
            raise SystemExit(0)

    def report(self, final: bool = False):
        if final:
            if self.final_reported:
                return
            self.final_reported = True
        mins = (time.monotonic() - self.t0) / 60.0
        rate = self.n_lpos / max(1e-6, time.monotonic() - self.t0)
        state = ('未解锁' if self.armed == 1 else
                 '已解锁' if self.armed == 2 else f'arming_state={self.armed}')
        tag = '结束小结' if final else '小结'
        print(f'[{time.strftime("%H:%M:%S")}] {tag}：已跑 {mins:.1f} 分钟 | lpos {self.n_lpos} 帧'
              f'（{rate:.1f} Hz）最大间隔 {self.max_gap*1000:.0f} ms'
              f' | >0.5s {self.n_gap_warn} 次、>1.0s {self.n_gap_big} 次'
              f' | nav_state={self.nav_state} {state} | 飞控重连 {self.n_reset} 次',
              flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent-log', default=DEFAULT_AGENT_LOG)
    ap.add_argument('--minutes', type=float, default=0.0,
                    help='跑多久自动退出（分钟），0=一直跑')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = WatchFcLink(args.agent_log, args.minutes)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.report(final=True)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
