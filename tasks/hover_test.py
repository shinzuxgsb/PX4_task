#!/usr/bin/env python3
"""自启动悬停测试任务（跑在香橙派上，通过 ROS 2 话题指挥飞控）。

流程（自启动，一条命令跑完）：

    INIT → （从这一秒起持续喂零速度 setpoint）→ 切定点(可选) → 解锁(arm)
         → 等「外部定位就绪」(/is_st_ready==1) → 切 OFFBOARD
         → 起飞到指定高度（默认离起飞地面 50cm，PD 速度闭环 + 到高稳定判据）
         → 悬停 N 秒（默认 5s，同时抑制水平漂移）
         → 下降 → 触发 PX4 自动降落(land) → 收尾退出

高度口径：以「解锁后、起飞前」的 /height 读数为零点（飞控 local z 的原点不在脚下，
上次实测地面上就读 10cm），所以 50cm 指的是离起飞地面 50cm。

安全设计：
  * **先喂 setpoint、再解锁**：PX4 的 `offboard_control_signal_lost` 只看"最近有没有收到
    offboard setpoint"。飞机如果停在 OFFBOARD 模式（上一次测试残留），而解锁时又没有 setpoint 流，
    PX4 会以 "No offboard signal" 直接拒绝 arm——所以本任务从 INIT 起就按 20Hz 发零速度 setpoint
    （PX4 只在 OFFBOARD 模式下才把 setpoint 送进控制器，其他模式下收到只是缓存，不会动飞机）。
  * **解锁前先切定点**：默认先下发 `position`，把飞机从可能残留的 OFFBOARD 模式切回 POSCTL，
    让解锁门禁只依赖"外部定位有效"这一项。
  * **先解锁、后等就绪**：只有 /is_st_ready==1（解锁 且 EKF 位置/速度有效）才开始 OFFBOARD 与爬升，
    所以"外部定位没起来就飞"这种情况不可能发生。
  * 解锁超时（默认 20s）→ 打印排查提示、下发 disarm 并退出（不再无限等下去）。
  * 就绪等待超时（默认 90s）→ 自动上锁退出。
  * 全程监视 /is_st_ready 掉线（上锁/失稳/遥控接管）→ 立即停止发 setpoint 并退出。
  * 触碰 /tmp/hover_abort（网页"停止"按钮）→ 立即转降落。
  * 总时长上限（默认 120s）→ 强制转降落。
  * 主循环任何未预期异常都被兜住：打印回溯、停止发指令、退出（不会像以前那样静默死掉）。
  * 遥控器 ch5 中档=POSCTL，随时可人工接管。

话题约定（与 mavlink_bridge 一致）：
  订阅 /odom_from_slam (nav_msgs/Odometry) 水平位置与偏航
  订阅 /height        (std_msgs/Int16)     高度 cm
  订阅 /is_st_ready   (std_msgs/UInt8)     1 = 已解锁且 EKF 有效
  发布 /target_velocity (std_msgs/Float32MultiArray) [vx,vy,vz cm/s, vyaw deg/s]（机体系 FLU）
  发布 /px4_command     (std_msgs/String)  arm / offboard / land / disarm

水平环（抑制漂移）：
   世界系 PID，再按当前偏航转到机体系：
       cmd = -kp·(位置偏差) - kd·(里程计实测速度) - i
   * 里程计速度来自 tf_to_odom 的 twist（机体系，时间窗差分 + 低通），D 项是真实速度反馈；
   * I 项只在离起点 30cm 内累积、限幅 ±15cm/s，用来顶掉恒定的推力/气动水平偏置
     （实测偏置约 0.13m/s²，纯 PD 只能靠 5~7cm 位置偏差硬顶，会留下 1.4cm/s 残漂）；
   * 1.5cm 位置死区挡 SLAM 毫米级抖动；起飞段自动切换到更硬的一组增益（--xy-kp-climb）。
   调参建议：还飘就先把 --xy-kd 加到 1.0~1.2；出现低频来回摆就把 --xy-kp 降到 1.0。

起飞分两段：
   先爬到 --climb-hold-cm（默认 12cm）稳住 --climb-hold-s（默认 1.2s），让水平环把
   地效与推力偏置吃掉，再继续爬到目标高度——起飞瞬间那 20~30cm 带偏就出在这里。
"""

from __future__ import annotations

import argparse
import collections
import math
import os
import signal
import sys
import traceback
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int16, String, UInt8


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_of(q) -> float:
    # 四元数 -> 偏航角（rad）
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HoverTask(Node):
    def __init__(self, args) -> None:
        super().__init__("hover_test_task")
        self.args = args
        self.phase = "INIT"
        self.abort = False
        self.done = False
        self.exit_code = 0

        self.odom = None
        self.odom_stamp = 0.0
        self.height_cm = None
        self.height_stamp = 0.0
        self.height_hist = collections.deque(maxlen=40)   # (时间, 高度cm)，用于算垂直速率
        self.height_zero = 0.0                            # 起飞点地面读数（高度全部按"离起飞地面"算）
        self.ready = 0
        self.ready_stamp = 0.0
        self.ready_seen = False
        self.arm_sent_at = 0.0
        self.arm_skipped = False
        self.pre_arm_done = False
        self.pre_arm_sent_at = 0.0
        self.settled_since = None
        self.land_sent = False
        self.land_sent_at = 0.0
        self.stabilize_until = None        # 起飞中途"先稳住"的截止时刻
        self.i_x = self.i_y = 0.0          # 水平积分项（cm/s）
        self.h_i = 0.0                     # 高度积分项（cm·s）
        self.last_tick_t = 0.0             # 上拍时间，用来算 dt
        self.hover_max_off_cm = 0.0        # 悬停段最大水平偏移
        self.hover_speed_sum = 0.0         # 悬停段水平速度累计（算平均用）
        self.hover_speed_n = 0

        self.start_x = self.start_y = self.start_yaw = None
        self.t0 = time.time()
        self.keepalive_sent = 0

        self.vel_pub = self.create_publisher(Float32MultiArray, "/target_velocity", 10)
        self.cmd_pub = self.create_publisher(String, "/px4_command", 10)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/odom_from_slam", self._on_odom, 10)
        self.create_subscription(Int16, "/height", self._on_height, 10)
        self.create_subscription(UInt8, "/is_st_ready", self._on_ready, latched)
        self.create_timer(0.05, self._tick)          # 20 Hz 主循环
        self.log("PHASE=INIT 悬停测试任务启动"
                 f"（目标 {args.height}cm，悬停 {args.hover}s，{'干跑(不解锁)' if args.dry_run else '实飞'}）")

    # ------------------------------------------------------------------ 回调
    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_stamp = time.time()

    def _on_height(self, msg: Int16) -> None:
        self.height_cm = float(msg.data)
        self.height_stamp = time.time()
        self.height_hist.append((self.height_stamp, self.height_cm))

    def height_rel(self) -> float:
        """离"起飞地面"的高度（cm）。飞控的 local z 原点并不在脚下，所以按起飞点做零点。"""
        return (self.height_cm if self.height_cm is not None else 0.0) - self.height_zero

    def height_rate(self) -> float:
        """最近约 0.4s 的高度变化率（cm/s，正=升高）；样本不足时返回 0。"""
        hist = list(self.height_hist)

        if len(hist) < 3:
            return 0.0

        now = time.time()
        t0, h0 = hist[0]

        for ts, hv in hist:                       # 取 0.4s 窗内最早的一条
            if now - ts <= 0.4:
                t0, h0 = ts, hv
                break

        dt = now - t0

        return (hist[-1][1] - h0) / dt if dt > 0.05 else 0.0

    def xy_offset(self) -> tuple:
        """相对起飞点的水平偏移（cm，世界系）。"""
        if self.odom is None or self.start_x is None:
            return (0.0, 0.0)

        return ((self.odom.pose.pose.position.x - self.start_x) * 100.0,
                (self.odom.pose.pose.position.y - self.start_y) * 100.0)

    def xy_speed(self) -> tuple:
        """里程计实测水平速度（cm/s，机体系）。"""
        if self.odom is None:
            return (0.0, 0.0)

        return (float(self.odom.twist.twist.linear.x) * 100.0,
                float(self.odom.twist.twist.linear.y) * 100.0)

    def _on_ready(self, msg: UInt8) -> None:
        self.ready = int(msg.data)
        self.ready_stamp = time.time()
        self.ready_seen = True

    # ------------------------------------------------------------------ 工具
    def log(self, text: str) -> None:
        print(f"[task] {text}", flush=True)

    def set_phase(self, phase: str) -> None:
        self.phase = phase
        self.log(f"PHASE={phase}")

    def send_cmd(self, cmd: str) -> None:
        self.cmd_pub.publish(String(data=cmd))
        self.log(f"下发飞控命令: {cmd}")

    def send_vel(self, vx=0.0, vy=0.0, vz=0.0, vyaw=0.0) -> None:
        self.vel_pub.publish(Float32MultiArray(data=[float(vx), float(vy), float(vz), float(vyaw)]))

    def odom_fresh(self) -> bool:
        return self.odom is not None and (time.time() - self.odom_stamp) < 0.5

    def height_fresh(self) -> bool:
        return self.height_cm is not None and (time.time() - self.height_stamp) < 0.5

    def check_abort(self) -> bool:
        if os.path.exists(self.args.abort_file):
            self.log(f"检测到 {self.args.abort_file}，转为降落")
            self.abort = True
            return True

        return self.abort

    def hold_velocity(self, target_h: float, dt: float) -> tuple:
        """按当前高度/位置误差算 (vx,vy,vz,vyaw)：机体系 FLU，单位 cm/s 与 deg/s。

        三个环都做了抗饱和的条件积分：

        * 高度：PD + 小积分（消掉 5~10cm 的慢静差；大误差时清零，避免起飞段灌爆）
        * 水平：PD + 小积分（顶掉恒定的推力/气动力偏置——实测这个偏置约 0.13m/s²，
          纯 P 环只能靠 5~7cm 的位置偏差硬顶，还会留下 1.4cm/s 的残漂）
        * 偏航：P
        """
        height = self.height_rel()
        err_h = target_h - height
        h_i_limit = self.args.height_i_limit / max(self.args.height_ki, 1e-6)

        if abs(err_h) <= self.args.height_i_band:
            self.h_i = clamp(self.h_i + err_h * dt, -h_i_limit, h_i_limit)
        else:
            self.h_i = 0.0

        # PD：P 顶着误差，D 用实测垂直速率刹车；I 把慢静差抠掉
        vz = clamp(self.args.height_kp * err_h
                   - self.args.height_kd * self.height_rate()
                   + self.args.height_ki * self.h_i,
                   -self.args.vz_max, self.args.vz_max)
        vx = vy = 0.0
        vyaw = 0.0

        if self.args.hold_xy and self.odom is not None and self.start_x is not None:
            yaw_now = yaw_of(self.odom.pose.pose.orientation)
            c, s = math.cos(yaw_now), math.sin(yaw_now)              # 世界 -> 机体：R(-yaw)

            # 位置误差（世界系，m）。小误差直接当 0，免得被 SLAM 的毫米级抖动来回拨。
            dxw = self.odom.pose.pose.position.x - self.start_x
            dyw = self.odom.pose.pose.position.y - self.start_y
            db = self.args.xy_deadband_cm / 100.0
            dxw = 0.0 if abs(dxw) < db else dxw
            dyw = 0.0 if abs(dyw) < db else dyw

            # 条件积分：只在离起点近时累积（起飞/降落的大误差不参与），并限幅
            if math.hypot(dxw, dyw) <= self.args.xy_i_band / 100.0:
                self.i_x = clamp(self.i_x + self.args.xy_ki * dxw * 100.0 * dt,
                                 -self.args.xy_i_limit, self.args.xy_i_limit)
                self.i_y = clamp(self.i_y + self.args.xy_ki * dyw * 100.0 * dt,
                                 -self.args.xy_i_limit, self.args.xy_i_limit)
            else:
                self.i_x = self.i_y = 0.0

            # 实测速度（cm/s）。nav_msgs/Odometry 的 twist 表达在子坐标系（机体），
            # 转到世界系后和位置误差同一个坐标系里做 PD。
            vb_x = float(self.odom.twist.twist.linear.x) * 100.0
            vb_y = float(self.odom.twist.twist.linear.y) * 100.0
            vwx = c * vb_x - s * vb_y
            vwy = s * vb_x + c * vb_y

            # 起飞段用更硬的一组水平增益：这一段的带偏最需要纠回来
            kp = self.args.xy_kp_climb if self.phase == "TAKEOFF" else self.args.xy_kp
            kd = self.args.xy_kd_climb if self.phase == "TAKEOFF" else self.args.xy_kd

            # 世界系速度指令：P 拉回起点 + D 用实测速度刹车 + I 顶掉恒定偏置
            cmd_wx = -kp * dxw * 100.0 - kd * vwx - self.i_x
            cmd_wy = -kp * dyw * 100.0 - kd * vwy - self.i_y

            if self.args.kalman_free:                                # 世界 -> 机体系（当前偏航）
                vx = c * cmd_wx + s * cmd_wy
                vy = -s * cmd_wx + c * cmd_wy
            else:
                vx, vy = cmd_wx, cmd_wy                              # 调试用：不做坐标变换

            vx = clamp(vx, -self.args.vxy_max, self.args.vxy_max)
            vy = clamp(vy, -self.args.vxy_max, self.args.vxy_max)

            err = math.degrees(math.atan2(math.sin(self.start_yaw - yaw_now),
                                          math.cos(self.start_yaw - yaw_now)))
            vyaw = clamp(self.args.yaw_kp * err, -10.0, 10.0)

        return vx, vy, vz, vyaw

    # ------------------------------------------------------------------ 主循环
    def _tick(self) -> None:
        """定时器入口：兜住任何未预期异常，不让任务在飞行中悄悄死掉（上次就是这么断流的）。"""
        try:
            self._tick_impl()
        except Exception:                                          # noqa: BLE001
            self.log("⚠ 任务内部异常 → 立即停止发指令并退出；PX4 会按 offboard 丢失保护"
                     "把模式交回 Position（遥控器此时可以接管）")
            traceback.print_exc()
            self.done = True
            self.exit_code = self.exit_code or 6

    def _tick_impl(self) -> None:
        if self.done:
            return

        now = time.time()
        dt = clamp(now - self.last_tick_t, 0.005, 0.2) if self.last_tick_t else 0.05
        self.last_tick_t = now

        elapsed = time.time() - self.t0
        self.check_abort()

        if elapsed > self.args.max_duration:
            self.log(f"总时长超过 {self.args.max_duration}s，强制降落")
            self.abort = True

        # 起飞后若掉线（上锁/EKF 失效/遥控接管），立即停发 setpoint。
        # 例外：LAND 阶段已经交给 PX4 自动降落后，PX4 落地自动上锁是**正常收尾**，不能当异常。
        handover_done = self.phase == "LAND" and self.land_sent

        if (not handover_done and self.phase in ("TAKEOFF", "HOVER", "LAND")
                and self.ready_seen and self.ready == 0):
            self.log("⚠ /is_st_ready=0（上锁/EKF 失效/被接管）→ 停止发指令并退出")
            self.finish(2)
            return

        # 解锁之前就按 20Hz 喂零速度 setpoint（"保活"）。
        # PX4 判断 offboard 信号是否有效只看"最近有没有收到 setpoint"：
        # 飞机若停在 OFFBOARD 模式，没有这条流就会以 "No offboard signal" 拒绝解锁。
        # PX4 只会在 OFFBOARD 模式下把 setpoint 送进控制器，其余模式下收到仅缓存，不会动飞机。
        if self.phase in ("INIT", "WAIT_TOPICS", "ARM", "WAIT_READY"):
            self.send_vel(0.0, 0.0, 0.0, 0.0)
            self.keepalive_sent += 1

        if self.phase == "INIT":
            self.set_phase("WAIT_TOPICS" if not (self.odom_fresh() and self.height_fresh()) else "ARM")

        elif self.phase == "WAIT_TOPICS":
            if self.odom_fresh() and self.height_fresh():
                self.set_phase("ARM")

        elif self.phase == "ARM":
            if self.args.dry_run:
                self.log("干跑模式：不发送 arm，直接进入就绪等待（只验证话题与逻辑）")
                self.set_phase("WAIT_READY")
                return

            if self.ready == 1:
                self.arm_skipped = True
                self.log("飞控已处于「已解锁 + 定位就绪」状态 → 跳过解锁，直接进入 OFFBOARD 阶段")
                self.set_phase("WAIT_READY")
                return

            if not self.pre_arm_done and self.args.pre_arm_mode not in ("", "none"):
                # 上一次测试可能把飞机留在 OFFBOARD 模式；OFFBOARD 的解锁门禁要求
                # "已有 setpoint 流"，先切回定点（POSCTL）让解锁只依赖外部定位是否有效。
                self.send_cmd(self.args.pre_arm_mode)
                self.pre_arm_sent_at = time.time()
                self.pre_arm_done = True
                return                      # 下一 tick 再解锁，避免两条命令挤在同一飞控循环里

            if self.pre_arm_done and (time.time() - self.pre_arm_sent_at) < self.args.pre_arm_delay:
                return

            self.send_cmd("arm")
            self.arm_sent_at = time.time()
            self.set_phase("WAIT_READY")

        elif self.phase == "WAIT_READY":
            if self.args.dry_run:
                self.log(f"干跑结论：odom {'ok' if self.odom_fresh() else '无'} | "
                         f"height={self.height_cm}cm | /is_st_ready={self.ready}"
                         f" | 已发保活 setpoint {self.keepalive_sent} 帧"
                         f"{'（未解锁时为 0，正常）' if self.ready == 0 else ''}")
                self.log("干跑模式：到此为止（未发 arm、未切 OFFBOARD、未起飞）")
                self.finish(0)
                return

            if self.ready == 1:
                # 起飞点地面读数当作高度零点：飞控 local z 的原点不在脚下（上次地面上读数是 10cm），
                # 不做这一步的话"目标 50cm"实际只有离地 40cm。
                self.height_zero = self.height_cm if self.height_cm is not None else 0.0
                self.log(f"外部定位就绪，记录起飞点：地面读数 {self.height_zero:.0f}cm 作为高度零点")

                if self.height_zero > 30.0:
                    self.log(f"⚠ 起飞前高度读数 {self.height_zero:.0f}cm 偏大（飞机可能不在水平地面上），"
                             f"目标高度将以此为基准")

                self.start_x = self.odom.pose.pose.position.x
                self.start_y = self.odom.pose.pose.position.y
                self.start_yaw = yaw_of(self.odom.pose.pose.orientation)

                if self.args.dry_run:
                    self.log("干跑模式：到此为止（未切 OFFBOARD、未起飞）")
                    self.finish(0)
                    return

                self.send_cmd("offboard")
                self.set_phase("TAKEOFF")

            elif self.abort:
                self.set_phase("LAND")

            elif (not self.arm_skipped and self.arm_sent_at > 0
                  and (time.time() - self.arm_sent_at) > self.args.arm_timeout):
                self.log(f"⚠ 解锁超时：{self.args.arm_timeout:.0f}s 内飞控没有进入解锁状态 → "
                         f"多半是 arm 被飞控拒绝。看上面 [PX4 ...] 开头的提示（例如 "
                         f"\"Arming denied: ...\" / \"Preflight Fail: ...\"）就能知道具体原因。")
                self.log(f"  当前：/is_st_ready={self.ready}（1=已解锁且定位有效）"
                         f" height={self.height_cm}cm odom={self.odom_fresh()}"
                         f" 保活 setpoint={self.keepalive_sent} 帧")
                self.log("  常见原因：① 飞机停在 OFFBOARD 模式且没有 setpoint 流（本任务已用"
                         "「先喂 setpoint + 先切定点」规避）② EKF 外部定位还没有效 ③ 遥控器"
                         "未连接/未校准 ④ 预检项未过（磁罗盘、电调等）")

                if not self.args.dry_run:
                    self.send_cmd("disarm")

                self.finish(5)

            elif time.time() - self.t0 > self.args.ready_timeout:
                self.log(f"⚠ 等待就绪超时（{self.args.ready_timeout}s）：检查雷达/SLAM/链路，以及是否已解锁")

                if not self.args.dry_run:
                    self.send_cmd("disarm")

                self.finish(3)

        elif self.phase == "TAKEOFF":
            # 分两段：先爬到 climb_hold_cm 稳住 climb_hold_s（让水平环/积分把地效与推力偏置吃掉），
            # 再继续爬到目标高度——起飞瞬间那 20~30cm 带偏就发生在这里。
            if (self.stabilize_until is None
                    and self.height_rel() >= self.args.climb_hold_cm - 4.0):
                self.stabilize_until = time.time() + self.args.climb_hold_s
                self.log(f"离地 {self.height_rel():.1f}cm → 先稳 {self.args.climb_hold_s:.1f}s "
                         f"再继续爬升（压起飞水平带偏）")

            climbing = self.stabilize_until is not None and time.time() >= self.stabilize_until
            target = self.args.height if climbing else self.args.climb_hold_cm

            vx, vy, vz, vyaw = self.hold_velocity(target, dt)
            h = self.height_rel()
            err = self.args.height - h

            if climbing and err > self.args.tol:                      # 还差一截：保证有最小上升速度
                vz = max(vz, self.args.climb_min)

            self.send_vel(vx, vy, vz, vyaw)

            # 到高判据：高度进带 + 垂直速度已收敛，并连续保持 settle_s（防止在爬升途中就判"到高"）
            settled = (climbing and abs(err) <= self.args.tol
                       and abs(self.height_rate()) <= self.args.settle_rate)

            if not settled:
                self.settled_since = None
            elif self.settled_since is None:
                self.settled_since = time.time()

            if settled and (time.time() - self.settled_since) >= self.args.settle_s:
                self.log(f"稳定在 {h:.1f}cm（目标 {self.args.height:.0f}cm，"
                         f"速率 {self.height_rate():+.1f}cm/s）→ 开始悬停 {self.args.hover}s")
                self.hover_until = time.time() + self.args.hover
                self.hover_max_off_cm = 0.0
                self.hover_speed_sum = 0.0
                self.hover_speed_n = 0
                self.set_phase("HOVER")
            elif self.abort:
                self.set_phase("LAND")

        elif self.phase == "HOVER":
            vx, vy, vz, vyaw = self.hold_velocity(self.args.height, dt)
            self.send_vel(vx, vy, vz, vyaw)

            ox, oy = self.xy_offset()
            sx, sy = self.xy_speed()
            self.hover_max_off_cm = max(self.hover_max_off_cm, math.hypot(ox, oy))
            self.hover_speed_sum += math.hypot(sx, sy)
            self.hover_speed_n += 1

            if int(time.time() * 2) % 2 == 0:                          # 约 2Hz 打点
                self.log(f"悬停中 h={self.height_rel():.1f}cm "
                         f"剩余 {max(0.0, self.hover_until - time.time()):.1f}s "
                         f"指令=({vx:+.0f},{vy:+.0f},{vz:+.0f})cm/s "
                         f"偏移=({ox:+.1f},{oy:+.1f})cm 实测速度=({sx:+.1f},{sy:+.1f})cm/s")

            if self.abort or time.time() >= self.hover_until:
                mean_speed = self.hover_speed_sum / self.hover_speed_n if self.hover_speed_n else 0.0
                self.log(f"悬停结束：最大水平偏移 {self.hover_max_off_cm:.1f}cm，"
                         f"平均水平速度 {mean_speed:.1f}cm/s → 开始降落")
                self.set_phase("LAND")

        elif self.phase == "LAND":
            if self.args.dry_run:
                self.finish(0)
                return

            h = self.height_rel()

            if h > self.args.land_handover_cm and not self.land_sent:
                self.send_vel(0.0, 0.0, -self.args.descend_rate, 0.0)   # 先自己下降到交接高度
            else:
                if not self.land_sent:
                    self.land_sent = True
                    self.land_sent_at = time.time()
                    self.send_cmd("land")                               # 交给 PX4 自动降落（只发一次）

                self.send_vel(0.0, 0.0, 0.0, 0.0)

                if self.ready_seen and self.ready == 0:
                    # 交给 PX4 自动降落后，/is_st_ready 变 0 = 已落地并自动上锁 → 正常结束
                    self.log("已落地并自动上锁，任务正常结束")
                    self.finish(0)
                    return

                if time.time() - self.land_sent_at > self.args.land_wait:
                    self.log("降落等待结束，收尾退出")
                    self.finish(0)

        elif self.phase == "DONE":
            return

    def finish(self, code: int) -> None:
        self.done = True
        self.exit_code = code
        self.set_phase("DONE")
        self.log(f"任务结束，退出码 {code}"
                 f"（0=正常，2=异常中断，3=等待就绪超时，4=被强制结束，5=解锁被拒/解锁超时）")


def main() -> int:
    ap = argparse.ArgumentParser(description="自启动悬停测试任务")
    ap.add_argument("--height", type=float, default=50.0, help="目标高度 cm（默认 50）")
    ap.add_argument("--hover", type=float, default=5.0, help="悬停时长 s（默认 5）")
    ap.add_argument("--tol", type=float, default=6.0, help="到高判据 cm")
    ap.add_argument("--climb-rate", type=float, default=30.0, help="(保留参数)")
    ap.add_argument("--descend-rate", type=float, default=25.0, help="下降速度 cm/s")
    ap.add_argument("--height-kp", type=float, default=0.8, help="高度 P 增益 (cm/s per cm)")
    ap.add_argument("--height-kd", type=float, default=0.6, help="高度 D 增益（用实测垂直速率刹车）")
    ap.add_argument("--height-ki", type=float, default=0.10,
                    help="高度 I 增益（1/s²，顶掉慢静差）")
    ap.add_argument("--height-i-limit", type=float, default=8.0,
                    help="高度 I 项最大贡献 cm/s（抗饱和）")
    ap.add_argument("--height-i-band", type=float, default=15.0,
                    help="只有高度误差小于该值 cm 时才累积积分")
    ap.add_argument("--climb-min", type=float, default=8.0, help="离目标还差很远时的最小上升速度 cm/s")
    ap.add_argument("--climb-hold-cm", type=float, default=12.0,
                    help="起飞途中先稳住的离地高度 cm")
    ap.add_argument("--climb-hold-s", type=float, default=1.2,
                    help="该高度稳住多久再继续爬升 s")
    ap.add_argument("--settle-s", type=float, default=0.6, help="进带且速率收敛需保持多久才算到高 s")
    ap.add_argument("--settle-rate", type=float, default=8.0,
                    help="判定「垂直速度已收敛」的门限 cm/s")
    ap.add_argument("--xy-kp", type=float, default=1.5,
                    help="水平 P 增益 (速度 cm/s per 位置偏差 m)")
    ap.add_argument("--xy-kd", type=float, default=0.8,
                    help="水平 D 增益（用里程计实测速度刹车，压漂移）")
    ap.add_argument("--xy-ki", type=float, default=0.30,
                    help="水平 I 增益（1/s²，顶掉恒定推力/气动偏置）")
    ap.add_argument("--xy-i-limit", type=float, default=15.0,
                    help="水平 I 项最大贡献 cm/s（抗饱和）")
    ap.add_argument("--xy-i-band", type=float, default=30.0,
                    help="只有离起点小于该值 cm 时才累积水平积分")
    ap.add_argument("--xy-kp-climb", type=float, default=2.5,
                    help="起飞段专用水平 P 增益（更硬，压起飞带偏）")
    ap.add_argument("--xy-kd-climb", type=float, default=1.0,
                    help="起飞段专用水平 D 增益")
    ap.add_argument("--xy-deadband-cm", type=float, default=1.5,
                    help="位置误差死区 cm（小于它当作 0，避免被 SLAM 抖动来回拨）")
    ap.add_argument("--yaw-kp", type=float, default=1.5, help="偏航 P 增益 (deg/s per deg)")
    ap.add_argument("--vz-max", type=float, default=20.0, help="垂直速度上限 cm/s")
    ap.add_argument("--vxy-max", type=float, default=30.0, help="水平速度上限 cm/s")
    ap.add_argument("--hold-xy", dest="hold_xy", action="store_true", default=True,
                    help="悬停时抑制水平漂移（默认开）")
    ap.add_argument("--no-hold-xy", dest="hold_xy", action="store_false")
    ap.add_argument("--kalman-free", dest="kalman_free", action="store_true", default=True,
                    help="把世界系误差转到机体系（默认开）")
    ap.add_argument("--land-handover-cm", type=float, default=12.0,
                    help="降到该高度后交给 PX4 自动降落")
    ap.add_argument("--land-wait", type=float, default=25.0, help="自动降落最长等待 s")
    ap.add_argument("--arm-timeout", type=float, default=20.0,
                    help="解锁后等待「已解锁+定位就绪」的上限 s，超时按「解锁被拒」处理")
    ap.add_argument("--pre-arm-mode", default="position",
                    help="解锁前先切到的 PX4 主模式（position/altitude/manual/none）")
    ap.add_argument("--pre-arm-delay", type=float, default=0.5,
                    help="切模式与解锁之间的等待 s")
    ap.add_argument("--ready-timeout", type=float, default=90.0, help="等待「外部定位就绪」上限 s")
    ap.add_argument("--max-duration", type=float, default=120.0, help="任务总时长上限 s")
    ap.add_argument("--abort-file", default="/tmp/hover_abort", help="出现该文件即转降落")
    ap.add_argument("--dry-run", action="store_true", help="只验证话题与逻辑，不解锁不飞")
    args = ap.parse_args()

    if os.path.exists(args.abort_file) and not args.dry_run:
        os.remove(args.abort_file)

    rclpy.init()
    node = HoverTask(args)

    def _on_signal(_sig, _frm):
        # 收到 SIGTERM/SIGINT：立即硬停（否则 timeout 杀掉不了进程）
        node.abort = True
        node.done = True
        node.exit_code = node.exit_code or 4

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if not args.dry_run:
            node.send_vel(0.0, 0.0, 0.0, 0.0)

        node.destroy_node()
        rclpy.shutdown()

    return node.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
