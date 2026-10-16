# px4_ev_bridge —— 雷达 + 定高器 合成外部视觉里程计

把雷达 SLAM 的水平位姿和 TOFSense-M 的垂直高度合成一条
`vehicle_visual_odometry`，喂给 PX4 EKF2。

```
雷达 SLAM  --/tracked_pose-----+
                               +--> px4_ev_bridge --> /fmu/in/vehicle_visual_odometry --> EKF2
TOFSense-M --/tofsense/height--+
```

分工：

| 维度 | 来源 |
| --- | --- |
| 水平位置 x,y / 水平速度 / 航向 | 雷达 2D SLAM |
| 垂直位置 z | TOFSense-M 定高器 |

为什么高度必须走 EV：PX4 的 uXRCE-DDS 输入话题里没有 distance_sensor，
定高器接在香橙派上时无法作为“测距”喂给 EKF2，只能塞进 EV 的垂直分量。

## 编译 / 运行

```bash
cd ~/px4_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select px4_ev_bridge
source install/setup.bash
ros2 launch px4_ev_bridge ev_bridge.launch.py
```

slam_toolbox 的话用 `pose_topic:=/pose pose_msg_type:=pose_with_covariance`。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| pose_topic | /tracked_pose | SLAM 位姿话题 |
| pose_msg_type | pose_stamped | 或 pose_with_covariance |
| velocity_topic | 空 | 可选 TwistStamped，空则用位置差分求速度 |
| enu_to_ned | true | SLAM 是 ENU，PX4 要 NED |
| yaw_offset_deg | 0.0 | 航向额外偏置 |
| use_height | true | 是否用定高器提供垂直位置 |
| height_topic | /tofsense/height | 定高器高度（米） |
| height_offset | 0.0 | 高度标定偏置（米） |
| height_sigma | 0.02 | 高度噪声（m） |
| height_timeout | 0.25 | 超时判为缺失（s） |
| position_sigma / velocity_sigma / yaw_sigma_deg | 0.05 / 0.10 / 2.0 | 雷达侧噪声 |
| publish_rate_hz | 30.0 | **固定发布频率（必须 > 5）**，见下面「发布节奏」 |

## 发布节奏（必须 ≥5 Hz，否则 EV 融合根本不启动）

EKF2 的硬约束（`ekf2/EKF/common.h`）：

```c
static constexpr uint64_t EV_MAX_INTERVAL = 200e3;  // 200 ms
```

`ev_control.cpp` 判断 `starting_conditions_passing` 时要求：相邻两帧 EV 的时间间隔
**小于 200 ms**，且最新一帧必须是近 200 ms 内的。否则 EV 的位置/速度/航向融合
**永远不会启动**；已经启动的，超过 400 ms 没数据还会被强制关掉。

cartographer 的 `/tracked_pose` 远达不到 5 Hz，所以本节点**不再"收到新位姿才发"**，
而是按 `publish_rate_hz`（默认 30 Hz）**固定频率重发最近一帧位姿**。
SLAM 超过 `pose_timeout` 没更新时**停止发布**，让 EKF2 自己超时退出 EV 融合
—— 绝不能把冻结的旧位置继续喂给 EKF2。

## 飞控侧参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| EKF2_EV_CTRL | 15 | bit0 水平位置 + bit1 垂直位置 + bit2 速度 + bit3 航向 |
| EKF2_HGT_REF | 3 | 高度参考 = Vision（不再是 Range） |
| EKF2_EV_DELAY | 实测 | SLAM+传输延迟（ms） |
| EKF2_EV_POS_X/Y/Z | 实测 | 雷达相对机体重心的安装位置 |
| EKF2_MAG_TYPE / SYS_HAS_MAG | **5** / 0 | 不用磁力计，航向由 EV 提供。**1.15.4 只有 0/1/5 三个值，`2` 是无效值** |
| EKF2_BARO_CTRL / SYS_HAS_BARO | 0 / 0 | 不用气压计 |
| SYS_HAS_GPS | 0 | 没装 GPS 时置 0，避免一堆 GPS 检查 |
| UXRCE_DDS_CFG | = 接香橙派的那路串口 | **不设的话飞控侧 `uxrce_dds_client` 根本不启动，EV 一个字节都进不来** |
| EKF2_EV_NOISE_MD | 0 | 用报文里报的方差（参数是下限），默认值即可 |

## 起飞前必须确认

**飞控端（nsh / QGC 的 MAVLink 控制台）**：

```bash
# 1) EV 数据有没有进来（不是 "never published"）
listener vehicle_visual_odometry

# 2) EKF 有没有真的在融合 EV：cs_ev_pos / cs_ev_hgt / cs_ev_yaw 要 True
listener estimator_status_flags

# 3) 位置/高度是否有效：xy_valid / z_valid 要 True
listener vehicle_local_position
```

**香橙派端**：

```bash
ros2 topic hz /fmu/in/vehicle_visual_odometry        # 应接近 publish_rate_hz
ros2 topic echo /fmu/out/vehicle_local_position --once
```

`xy_valid` / `v_xy_valid` / `z_valid` 三个都要是 true，才允许起飞。
注意 EKF2 的 `checkPosVelValidity()` 有 **1 秒观察期**（从无效变有效要连续通过 1 s），
链路刚通时别急着解，等一两秒。
同时看节点日志里那行 `EV: NED pos(x, y, z) yaw ... | 高度有效`。
