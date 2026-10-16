# px4_takeoff_land —— 原地起飞 / 悬停 / 原地降落

补上 `px4_bridge` 缺的任务状态机。`px4_bridge` 只做「速度透传」，
本节点负责：**解锁 → 切 Offboard → 起飞 → 悬停 → 下降 → 交还 PX4 自动降落**。

> 本节点和 `px4_bridge` 都会往 `/fmu/in/offboard_control_mode` 发消息，
> **不要同时运行**。

## 编译 / 运行

```bash
cd ~/px4_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select px4_takeoff_land
source install/setup.bash

# 先空跑确认话题链路（不解锁、不切换模式）
ros2 launch px4_takeoff_land takeoff_land.launch.py dry_run:=true

# 真跑
ros2 launch px4_takeoff_land takeoff_land.launch.py target_alt:=2.0 hover_time:=5.0
```

监控任务阶段：

```bash
ros2 topic echo /mission_state
```

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `target_alt` | 2.0 | 目标高度（相对起飞点，m） |
| `hover_time` | 5.0 | 到顶后悬停时间（s） |
| `climb_vel` / `descend_vel` | 0.5 / 0.5 | 升降速度（m/s，`velocity` 模式用） |
| `land_switch_alt` | 0.3 | 低于此高度交还 PX4 自动降落（m） |
| `control_mode` | position | `position` = 位置设定值（抗漂移，推荐）；`velocity` = 速度设定值 |
| `dry_run` | false | true 时只跑流程、不发任何指令 |
| `start_delay` | 3.0 | 启动后等待时间（s） |
| `max_alt` | 0.0 | >0 时作为高度硬上限保护 |

## 小任务航线（2026-10-09 新增）

`mission_enable:=true` 时，起飞到 `target_alt` 后自动执行：

```
起飞 → (沿起飞时刻机头方向)前进 forward_dist → 原地掉头 turn_deg
     → 保持新航向飞回起飞点正上方 → 悬停 hover_time → 交还 PX4 自动降落
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `mission_enable` | false | 打开任务航线（false = 原来的"原地起降"） |
| `forward_dist` | 0.5 | 前进距离（m），方向 = **起飞时刻的机头方向**（不是世界北） |
| `turn_deg` | 180 | 掉头角度（deg），正 = 顺时针 / 向右 |
| `pos_tol` | 0.15 | 水平到位容差（m） |
| `yaw_tol_deg` | 10 | 航向到位容差（deg） |
| `settle_time` | 0.5 | 连续在容差内多久才算"到位"（s） |
| `leg_timeout` | 20 | 每段超时（s），超时即 ABORT → 自动降落 |

- 任务段全部用 **位置 + 绝对航向**设定值（NED），所以要求 `control_mode=position`
  （写成 velocity 会被自动改回并打警告）。
- "前进方向"取 `vehicle_local_position.heading`（就是 EV/SLAM 给的航向）→
  **起飞前航向估计必须可信**；节点在拿到有效 heading 之前不会往下走。
- 阶段可以用 `ros2 topic echo /mission_state` 盯：
  `TAKEOFF → MISSION_FORWARD → MISSION_TURN → MISSION_RETURN → HOVER → DESCEND →`
  `HANDOVER_TO_PX4_LAND → DONE`。
- 每段到位/超时都会打日志（含实际误差），飞完可以照着复盘。

### 验证顺序（务必）

1. **拆桨**跑 `dry_run:=true`：确认话题链路与 `/mission_state` 正常。
   （dry_run 不解锁，所以会停在 `ARMING` 然后超时 `ABORT` —— 这是预期，用来验证"命令确实被拦住"。）
2. 拆桨、装电池，按下面清单确认飞控参数（尤其 `COM_DISARM_PRFLT`、`COM_OF_LOSS_T`、`MPC_TKO_SPEED`），
   并确认 `ros2 topic info /fmu/in/vehicle_visual_odometry` 的 **Subscription count ≥ 1**。
3. 真机首飞：`target_alt` 先用 0.3~0.5 m、`forward_dist` 先用 0.3 m，场地空旷、遥控器在手
   （拨杆切回 Position 就能接管），全程盯 `/mission_state` 与 `/fmu/out/vehicle_local_position`。
4. 一切正常再放大到 0.5 m / 掉头 180°。

> ⚠️ 本节点和 `px4_bridge` 都会发 `/fmu/in/offboard_control_mode`，**不要同时运行**；
> 也不要在别人用遥控器飞的时候启动本节点（它会切 Offboard 抢控制权）。

## 飞控侧需要「准备」的参数（在 QGC 或 `param set` 里配）

**这些是参数配置，不是写固件**——PX4 1.15.4 标准板级固件本身就够用，
不需要改 PX4 源码重编（`uxrce_dds_client` 默认已编进所有 FMU 板级固件）。

| 参数 | 建议值 | 作用 |
| --- | --- | --- |
| `COM_RC_IN_MODE` | 1 | 无遥控器、纯上位机控制时用「Joystick only」（会关掉 RC 相关检查与失效保护） |
| `COM_RCL_EXCEPT` | 4 | 有遥控器时：Offboard 模式下忽略 RC 丢失（bit2 = Offboard） |
| `COM_OF_LOSS_T` | 1.0 | Offboard 信号丢失判定时间（默认值，节点必须小于该周期持续发流） |
| `COM_OBL_RC_ACT` | 0 | 离开 Offboard 后的动作（0 = Position 悬停，适合室内首测） |
| `COM_ARM_WO_GPS` | 1 | 允许无 GPS 解锁 |
| `COM_DISARM_PRFLT` | 10 | **解锁后 10s 不起飞就自动上锁**——节点必须马上接手起飞 |
| `MPC_TKO_RAMP_T` | 3.0 | 起飞推力斜坡时间（s） |
| `MPC_TKO_SPEED` | 1.5 | 起飞爬升速度（m/s） |
| `MPC_LAND_SPEED` | 0.7 | 降落末段速度（m/s） |
| `MPC_THR_HOVER` | 实测值 | 悬停油门，必须与整机重量匹配（否则起降不稳） |
| `UXRCE_DDS_CFG` | 接香橙派的那路串口 | 把 uXRCE-DDS 绑到该串口，开机自启 |
| `EKF2_GPS_CTRL` / `EKF2_OF_CTRL` / `EKF2_EV_CTRL` | 按定位方案 | 室内无 GPS 时打开光流 / VIO |

## 硬性前提：必须有有效的本地位置/速度估计

`offboardCheck.cpp` 里 PX4 1.15.4 的判定：

- 发 `OffboardControlMode` 的周期必须 < `COM_OF_LOSS_T`（默认 1s）
- 声明 `velocity` → `local_velocity_invalid` 必须为 false
- 声明 `position` → `local_position_invalid` 必须为 false

所以：

- **室外**：GPS 定位即可
- **室内无 GPS**：必须加光流 + 测距（或 VIO/mocap）喂给 EKF2，
  否则 PX4 会拒绝进入 Offboard（日志会报 "No valid local position estimate"）

## 建议验证顺序

1. **SITL**（香橙派或 PC 上）先跑通整个流程，确认状态机、话题、参数都对
2. 真机 **拆桨** 跑一次 `dry_run`，看 `/mission_state` 阶段是否正常推进
3. 真机装桨、系留 / 空旷场地首飞，`target_alt` 先给 0.5~1.0m
