# drone_d_ws 建图与定位（SLAM）链路全面解析

> 对象：`drone_d_ws` 工作区里"雷达 → cartographer → TF → 任务节点/PID"这一整条链路。
> 相关文件：`src/bluesea2/`、`src/my_carto_pkg/`、`src/pid_control_pkg/`、
> `src/uart_to_stm32/`、`src/shelf_scan_pkg/src/mission_node.cpp`。

---

## 0. 三句话总览

1. **不是"先建图再飞"，而是边飞边建图（online 2D SLAM）**：用机载 2D 激光雷达 +
   cartographer 2D 做实时建图与定位，**不加载任何先验地图（没有 pbstream）、不使用 IMU、
   不使用轮式/光流里程计**，是纯激光 scan matching。
2. **地图坐标系（`map`）的原点是随机的**：cartographer 的 map 原点 = 建图起点
   （开机时雷达所在位置），map 的朝向 = 雷达自己的 0° 方向。所以任务节点**必须**在起飞前
   用"飞机摆在起飞点"这一件事做一次刚体标定（`FrameTransform`），把 map 对齐到题目场地坐标。
3. **建图的产物只有一个**：TF 树里的 `map→odom→base_link` 位姿（外加一张没人看的
   占用栅格图 `/map`）。任务节点、位置 PID、STM32 桥三个消费者都靠这个位姿工作，
   其中 PID 和 STM32 桥对**位姿新鲜度**有硬保护（>0.5s 陈旧就发零速度）。

---

## 1. 整条链路的鸟瞰

```
  ┌──────────────────────── 香橙派 (ROS 2 Humble) ────────────────────────────┐
  │                                                                          │
  │  2D 激光雷达 ──UART 921600(/dev/ttyS4)──▶ bluesea_node                    │
  │  (雷神 BlueSea2 系列, 900rpm)                    │  /scan  LaserScan        │
  │                                                  │  frame_id = laser_link   │
  │                                                  ▼                          │
  │   urdf/fly.urdf ──▶ robot_state_publisher ──▶ TF: base_link→laser_link      │
  │                                                  │                          │
  │                                                  ▼                          │
  │                                       cartographer_node (amphi.lua, 2D)     │
  │                                                  │                          │
  │                          TF: map→odom→base_link   │   /submap_list          │
  │                                                  ▼                          │
  │                              cartographer_occupancy_grid_node → /map        │
  └──────────────────────────────────────────────────┬───────────────────────┘
                                                     │ TF 查询
        ┌────────────────────────────┬───────────────┴────────────────┐
        ▼                            ▼                                ▼
 shelf_mission_node           pid_control_pkg                  uart_to_stm32
  20 Hz 查位姿                 50 Hz 查位姿                      100 Hz 查位姿
  → 场地坐标 + 航点判定        → 位置 PID → /target_velocity     → 差分算速度
  → /target_position           （位姿>0.5s 陈旧 → 零速度）      → /velocity_map + 0x32 帧
                                                                 → 飞控（速度源）
```

三个消费者都查**同一个变换**：`map → laser_link`。

---

## 2. 第一环：雷达驱动 `bluesea2`

### 2.1 参数（实际生效的是 `params/uart_lidar.yaml`）

`my_carto_pkg/launch/fly_carto.launch.py` 在 include 雷达 launch 时**显式**传了
`params_file=bluesea2/params/uart_lidar.yaml`（不传会被外层的 `params_file` 顶掉，
雷达拿不到串口号，报 `DEVICE_OFFLINE(-114)`，README 里记着这个坑）。

| 参数 | 值 | 作用 |
|---|---|---|
| `type` / `port` / `baud_rate` | `uart` / `/dev/ttyS4` / `921600` | 雷达接在香橙派 ttyS4，与飞控用的 ttyS6 分开 |
| `frame_id` | `laser_link` | **必须与 cartographer 的 `tracking_frame` 一致** |
| `scan_topic` | `scan` | 发 `sensor_msgs/LaserScan` |
| `output_360` | `true` | 累积一整圈（360°）才发一帧，而不是每 36° 扇区发一次 |
| `output_scan` / `output_cloud2` | `true` / `false` | 只发 LaserScan，不发点云 |
| `rpm` | `900` | 15 圈/s → **`/scan` 约 15 Hz** |
| `hard_resample` / `soft_resample` / `resample_res` | `true` / `true` / `0.3` | 重采样到 **0.3°/点 ≈ 1200 点/圈**（原始更粗时插值补齐） |
| `min_dist` / `max_dist` | `0.1` / `8.0` m | 与 cartographer 的 `min_range/max_range` 对齐，超出记 `inf` |
| `with_checksum` | `true` | 串口帧带校验和 |
| `with_confidence` / `with_smooth` / `with_deshadow` | `1` / `1` / `1` | 让雷达回传置信度并开内部平滑/去遮挡 |
| `error_circle` / `error_scale` | `2` / `0.8` | 一整圈里 >80% 是零点距离就判为无效帧；连续 2 次报警 |
| `with_angle_filter` | `false` | 不做角度裁剪，全 360° 保留 |
| `filter_open`（未设置） | `false` | SDK 里那个"邻域一致性中值滤波"**没开** |

### 2.2 驱动内部是怎么把数据变成 `/scan` 的

```
UartThreadProc 线程          Parser 线程/回调            PubHub 队列         主循环(100Hz)
   read(fd)  ──▶  按帧头/校验和拆 RawData(扇区)  ──▶  fans[]  ──▶  checkIsRun()
                                                                   GetAllFans()
                                                                      │
                                                          PublishLaserScan() → /scan
```

关键处理（`sdk/src/bluesea.cpp`、`src/node.cpp`）：

1. **找整圈起点 `checkIsRun()`**：以 `from_zero ? 0° : 180°` 为目标角度找第一个扇区，
   记下 `offsetangle/offsetidx` 作为"一圈的起始角"。
2. **拼一整圈 `GetAllFans()`**：从起点角往后累到下一次出现同一角度为止，
   并做**两个完整性校验**：
   - 角度连续：`(angle + span) % 3600 == 下一扇区.angle`（单位 0.1°）；
   - 总跨度：`total == 3600`（±起点扇区偏移）。
   任一不满足 → **整圈丢弃**（宁可少一帧，也不要半圈数据污染 scan matching）。
3. **重采样**：把每个扇区插值到 `resample_res = 0.3°` 一点。
4. **整帧有效性**：`checkZeroDistance()` 判定零点占比 >80% 视为无效；连续 2 帧触发告警。
5. **发布**（`PublishLaserScan`）：

```cpp
msg.header.frame_id = "laser_link";
msg.angle_min = -M_PI;  msg.angle_max = M_PI - 2π/N;  msg.angle_increment = 2π/N;
msg.range_min = 0.1;    msg.range_max = 8.0;
msg.scan_time = ts_end - ts_beg;         // ≈ 1/15 s
msg.time_increment = scan_time / N;      // 每点时间偏移（cartographer 做去畸变用）
msg.intensities[i] = 置信度;
// 距离为 0 / 超范围 / 命中 mask 的点写 inf
```

> **时间戳用的是雷达自己给的时间**（`fan->ts`），不是 ROS 时钟。
> 对 SLAM 没问题（同一套时间源、只做相对匹配），但意味着 scan 时间与 `/height`
> 之间没有严格同步——好在高度不参与建图。

6. **日志 `no answer` / `chksum error` 是正常的**：驱动启动时会给雷达下发配置命令
   （`LSRPM` 等），这型雷达不回应；只要 `ros2 topic hz /scan` 稳定在 10~15Hz，
   说明数据链路是好的。README 已把这条写进常见问题。

7. 附带的 `scan_motor` 服务（`base_lidar/srv/Control`）可远程启停电机 / 调速，
   D 题不用。launch 里用 `LifecycleNode` 启动，但驱动源码其实是**普通 rclcpp 节点**
   （没有 lifecycle 接口），只是沿用了厂商模板；不影响 `/scan`。

---

## 3. 第二环：TF 树是怎么搭起来的

三处分别负责一段，缺一段 `map→laser_link` 就断，任务节点会一直打
`定位数据不可用（map -> laser_link）`：

| 变换 | 谁发布 | 来源 |
|---|---|---|
| `base_link → laser_link`（z = **0.1 m**） | `robot_state_publisher` | `my_carto_pkg/urdf/fly.urdf` 的 fixed joint |
| `base_link → imu_link`（z = 0.05 m） | `robot_state_publisher` | 同上（**建图用不到 IMU**，见第 7 节） |
| `map → odom` | `cartographer_node` | 位姿图全局优化结果 |
| `odom → base_link` | `cartographer_node` | 局部 SLAM 外推位姿（`provide_odom_frame=true`） |

要点：

- URDF 里 `laser_link` 有 **z=0.1 的偏移、没有 x/y 偏移**。这意味着代码认定的"飞机位置"
  其实是**雷达在机体平面上的投影点**。标定会把这一点直接当成起飞点，
  所以**只要雷达不挪位置**，这个固定偏置是被吸收掉的；但如果雷达不在机体的旋转中心，
  **转向时机体实际位置会绕着雷达画弧**——这是后文 P1 里的一条。
- `publish_frame_projected_to_2d = true`：发布的位姿被投影到 2D（z 归零、只留 yaw），
  所以 `map→laser_link` 的 z 基本恒为 0.1，任务节点只取 x/y/yaw，高度一律用 `/height`。

---

## 4. 第三环：cartographer 2D 的配置与算法

### 4.1 实际加载的是哪个 lua

`fly_carto.launch.py` 里写死：

```
-configuration_directory <my_carto_pkg>/configuration_files
-configuration_basename amphi.lua        ← 实际生效
```

包里的 `fly.lua`、`map_builder.lua`、`trajectory_builder*.lua`、`pose_graph.lua` 中，
只有被 include 的才是"底座"，**`amphi.lua` 的覆盖项才是真正在跑的参数**
（`fly.lua` 目前是没人用的备份）。

### 4.2 `amphi.lua` 关键参数逐条解释

| 参数 | 值 | 为什么 |
|---|---|---|
| `map_frame` / `tracking_frame` / `published_frame` | `map` / `laser_link` / `base_link` | 跟踪雷达、对外发布机体 |
| `provide_odom_frame` | `true` | 插一层 odom，供 PID/STM32 桥查询 |
| `use_pose_extrapolator` | `true` | 用匀速模型在两帧 scan 之间外推位姿（使 200Hz 的 TF 平滑） |
| `pose_extrapolator.use_imu_based` | `false`（默认） | **纯匀速外推，不依赖 IMU** |
| `use_odometry` / `use_nav_sat` / `use_landmarks` | 全 `false` | 没有任何外部里程计输入 |
| `num_laser_scans` | `1` | 单线雷达 |
| `TRAJECTORY_BUILDER_2D.use_imu_data` | **`false`** | 关键：整个 SLAM 不用 IMU |
| `min_range` / `max_range` | `0.1` / `8.` | 与雷达 `min_dist/max_dist` 一致；8m 覆盖 5m×4m 场地 |
| `submaps.num_range_data` | **`35`** | 每 35 帧（≈2.3s）切一个子图 —— 比默认 90 短得多 |
| `num_accumulated_range_data` | `1` | 不做多帧累积去畸变 |
| `missing_data_ray_length` | `1.0` | `inf` 点按 1m 处理（安全网穿透/无回波时别画太长） |
| `use_online_correlative_scan_matching` | **`true`** | 先用相关性搜索做粗匹配，再交给 Ceres 精配（比默认纯 Ceres 稳） |
| `real_time_correlative_scan_matcher.linear_search_window` | **`0.1` m** | 每帧最多搜索 ±10cm 平移 |
| `...translation_delta_cost_weight` | **`10.`** | 重罚"平移突跳" → 抑制定位在悬停时漂移 |
| `...rotation_delta_cost_weight` | `0.1` | 旋转相对宽松，允许转向 |
| `POSE_GRAPH.optimize_every_n_nodes` | **`35`** | 回环优化频率提高（默认 90） |
| `POSE_GRAPH.constraint_builder.min_score` | **`0.65`** | 回环匹配阈值抬高（默认 0.55）→ 拒绝可疑回环 |
| `POSE_GRAPH.optimization_problem.huber_scale` | `1e2` | 更软的鲁棒核，抑制离群约束拉偏地图 |
| `MAP_BUILDER.num_background_threads` | `4` | 后台优化线程数（pose_graph.lua 里） |
| `POSE_GRAPH.constraint_builder.sampling_ratio` | `0.3` | 抽 30% 节点建立约束（计算量与精度折中） |
| `global_constraint_search_after_n_seconds` | `10.` | 跑够 10s 后开始全局回环搜索 |
| `pose_publish_period_sec` | `5e-3` | TF 以 ~200Hz 发布 |

**这套参数的调参意图很清楚**：场景是 5m×4m 小房间、飞行速度只有 25~45cm/s，
所以把"搜索窗口缩小 + 平移跳变重罚 + 回环阈值提高"，换来的是**悬停时位姿稳、不容易被
相似结构骗出错误回环**；代价是快速移动时容易跟丢（但飞行速度本来就被 PID 限制住了）。

### 4.3 两个阶段在干什么

**前端（局部 SLAM，每帧都跑）**

```
新 scan ──▶ RTCSM 在 ±10cm/（角度窗口由 pose_graph 配置）内做相关性粗匹配
        ──▶ Ceres scan matcher 精配（相对上一帧的位姿）
        ──▶ 插入当前子图（每 35 帧关一个子图）
        ──▶ 结果 → odom→base_link（经匀速外推平滑到 200Hz）
```

**后端（位姿图，每 35 个节点跑一次）**

```
节点+子图 ──▶ 回环检测（采样率 0.3，min_score 0.65，10s 后允许全局搜索）
          ──▶ Ceres 全局优化（huber 1e2 鲁棒核）──▶ 修正 map→odom
```

**占用栅格**：`cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0`
一直把 `/map` 用 5cm 栅格发出来——但 D 题没有任何节点消费它（RViz 也没启动），
属于"顺手开着"的产物。

### 4.4 量化一下"够不够用"

| 量 | 值 | 说明 |
|---|---|---|
| 雷达帧率 | ~15 Hz（900 rpm） | 每帧间隔 66.7 ms |
| 每帧点数 | ~1200（0.3°） | 一圈 360° |
| 巡航速度 | 25 cm/s（`drone_config.yaml` 的 `max_linear_velocity`） | field.json 里 `speeds.horizontal=45` 只用于地面站估时 |
| 帧间位移 | **1.7 cm**（25cm/s） / 3.0 cm（45cm/s） | 远小于 RTCSM 的 10cm 搜索窗口 → **匹配有余量** |
| 一个子图的时长/距离 | 2.33 s / 58 cm | 35 帧 |
| 场地尺度 | 500×400 cm，雷达 8m 量程 | 一圈能同时看到 4 面安全网，特征充足 |

---

## 5. 第四环：标定 —— 把随机的 `map` 对齐到题目场地坐标

### 5.1 为什么必须标定

cartographer 建图时，**第一帧 scan 的位置就是 map 原点**，map 的 X 轴由**雷达自己的 0° 方向**
决定（不是机头方向，也不是场地方向）。所以上电后 `map` 与题目场地坐标系之间差一个
**平面刚体变换**（旋转 + 平移），必须解出来：

```
field = R(yaw_offset) · map + t          （两套坐标只差旋转+平移，单位 cm）
```

### 5.2 标定怎么做（`mission_node.cpp` + `gs_protocol.cpp`）

起飞前把飞机**准确放在起飞点 (75,75) 的中心、机头正对货架**，然后：

```cpp
// 连续 5 帧位姿漂移 < 2cm（确认飞机已经放稳、SLAM 已收敛）
if (calibration_samples_ >= 5) calibrateFromCurrentPose("启动自动标定");

// 解算
yaw_offset_deg = normalizeDeg(field_home_yaw_deg - map_yaw_deg);   // 场地朝向 - 地图朝向
t = field_home - R(yaw_offset) · map_home;

// 之后任何航点：fieldToMap  = R(-yaw_offset) · (field - t) → 发 /target_position
// 反向：mapToField = R(yaw_offset) · map + t            → 遥测回报场地坐标
```

参数：`field_home_x_cm=75 / field_home_y_cm=75 / field_home_yaw_deg=0`（2026-09-13 起
起飞朝向改成"正对货架"= 场地 0°，**必须与摆放方向一致**）。

### 5.3 标定能吸收什么、不能吸收什么

| 能吸收 | 说明 |
|---|---|
| 雷达的安装朝向（相对机头转了多少度） | 因为 `yaw_offset` 直接由"机头朝向 0°"反解 |
| 雷达相对机体中心的固定平移 | 标定把"雷达当前所在点"直接认定为起飞点 |
| 建图起点在场地中的任意位置 | 平移 `t` 就是干这个的 |

| 不能吸收 | 后果 |
|---|---|
| 雷达不在机体旋转中心 | 原地转向时机体实际位置会画弧，画弧半径 = 偏置距离 |
| 摆飞机时摆歪的角度 | 整个场地坐标系跟着转同样角度（摆歪 90° → 全盘偏 90°） |
| 建图过程中的累积漂移 / 错误回环 | 全场点位一起被拉偏 |
| 标定之后挪动机体 | 标定失效（必须重启或发 `calibrate` 重标） |

调试手段：`ros2 topic echo /shelf_scan/status` 看 `calibrated: true`；
`ros2 run tf2_ros tf2_echo map laser_link` 看位姿是否连续、是否跳动。

---

## 6. 第五环：建图结果被谁消费、怎么用

三个消费者查的都是 `map → laser_link`，但频率和保护策略不同：

| 消费者 | 频率 | 查位姿后做什么 | 位姿异常时的行为 |
|---|---|---|---|
| `shelf_mission_node` | 20 Hz | `mapToField()` 得场地坐标 → 算到位偏差、发遥测、判定航段切换 | 仅 `has_pose_=false` + 3s 限流告警，**任务不中断**（继续按时间超时推进） |
| `pid_control_pkg` | 50 Hz | 位置 PID：`map` 系目标 − 当前位姿 → `/target_velocity` | **`pose_data_timeout_sec=0.5` 陈旧检查**：超时或查询失败 → 直接发 `[0,0,0,0]`（就地悬停） |
| `uart_to_stm32` | 100 Hz | 对位姿做**差分**得速度 → `/velocity_map` (Twist) 并按 0x32 帧发给飞控 | 查询失败只打 DEBUG 日志，不发新帧（飞控保持上一次速度） |

这里有三个值得注意的工程细节：

1. **建图输出同时是飞控的速度源**。`/velocity_map` + 0x32 帧给了 STM32 一个"SLAM 实测速度"，
   飞控把它当光流/速度反馈用。**所以建图质量直接决定悬停稳不稳**，不只是"航点准不准"。
2. **安全网两层**：PID 侧 0.5s 陈旧保护会主动刹车；任务节点侧没有保护，会在定位丢失后
   继续按超时逻辑往下走。也就是说定位丢失时飞机**会被 PID 按住悬停**，但任务状态机会
   以为"这个点超时了"继续推进到下一个点——恢复定位后会突然飞向更远的目标。
3. 速度是**用 TF 差分算的**，位姿跳变（回环优化）会在那一瞬间产生一个虚假的大速度。
   cartographer 的 `pose_publish_period_sec=5ms`  + 匀速外推能减弱这点，
   但**回环优化发生的瞬间仍可能在 `/velocity_map` 上看到尖峰**——这是这套方案的固有特性。

---

## 7. 误差来源全面梳理

### 7.1 传感器与场景层

| 来源 | 影响 | 现有对策 |
|---|---|---|
| **纯激光、无 IMU** | 急加减速/快速偏航时匀速外推失准，转弯处易跟丢 | 飞行速度被限制在 25cm/s；RTCSM 每帧重匹配；`translation_delta_cost_weight=10` 抑制突跳 |
| 无轮式/光流里程计 | 没有第二路运动约束 | 同上（这是"没有选择"的选择） |
| **安全网是网面** | 激光可能穿透、部分回波、多径 → 产生稀疏/虚假点 | `missing_data_ray_length=1.0` 限制 inf 射线的长度；`min_score 0.65` 拒绝坏匹配 |
| 白色 PVC 板 + 白墙 + 现场光照 | 回波强度不稳定 | 雷达带 `with_confidence`/`with_smooth`/`with_deshadow` |
| 2D 雷达只能扫一个高度面 | 100~150cm 高度上，两个货架都表现为"细线 + 两根红柱" | 无（结构如此）；靠 4 面安全网 + 地面/墙面提供全局特征 |
| 两个货架**结构完全相同** | 回环误匹配（把货架 2 认成货架 1）风险高 | `min_score` 抬到 0.65、`huber_scale=1e2`、`sampling_ratio=0.3` |
| 雷达最小距离 0.1m | 极近处可能扫到机身/桨叶 | `min_range` 与之一致；雷达装在机体上方 0.1m |

### 7.2 系统与工程层

| 来源 | 影响 | 现状 |
|---|---|---|
| **每次上电重新建图**（没有 pbstream 保存/加载） | 每次的 map 都不同，绝对基准全靠起飞点标定 | 已用 `pure_localization_trimmer` 注释掉了纯定位模式；`/write_state` 服务还在，可手动存图 |
| 子图短（35 帧）、回环频繁 | 香橙派 CPU 压力大（4 个后台线程） | 未做优化；实测能跑就先用着 |
| 雷达时间戳来自雷达自身 | 与 `/height` 无严格同步 | 高度不参与建图，影响可接受 |
| 标定依赖"人把飞机摆正" | 摆放误差直接变成全场旋转/平移误差 | 靠"连续 5 帧漂移<2cm"+ 现场用卷尺核对 (75,75)/(150)/(350) |
| 回环优化瞬间的 TF 跳变 | `/velocity_map` 速度尖峰 | 未特殊处理 |
| 任务节点无位姿陈旧保护 | 定位丢失时任务继续推进 | PID 侧有 0.5s 保护兜底 |

---

## 8. 与题目要求的匹配度

| 题目相关要求 | 建图链路的支撑情况 |
|---|---|
| "辅助定位方式不限" | 选用机载 2D 激光 + cartographer，**不改场地、不贴标记**，符合规则 |
| 起飞点垂直起飞、150±10cm | 高度不靠地图，靠 STM32 的 `/height` 闭环 |
| 24 个点位精确悬停 | 依赖 `map→field` 标定精度 + SLAM 全程漂移量 |
| 不得人为干预、不得降落 | 建图/定位全自动，标定在起飞前完成 |
| 用时（10 分 ×2） | SLAM 是纯机载计算，不占空中时间；子图短 → 回环快 → 位姿收敛快 |

**定位精度的量级判断**：这类"小房间 + 双雷达 + 短子图 + 高回环率"的配置，
在 5m×4m 场地里通常能做到**几厘米级**的相对精度（局部），
绝对精度主要吃两件事：① 起飞点摆放；② 全程是否有一次错误回环。
任务节点留了 8cm 的到位容差和"没扫到沿法线微调"的兜底，是合适的裕量。

---

## 9. 问题清单与改进建议

### P0　把飞控的 IMU 接进 cartographer（收益最大）

飞控**已经在往香橙派发 0x01（IMU/四元数）帧**，只是 `uart_to_stm32` 里
`protocolDataHandler` 没处理它（只处理 0x05 高度、0x40 遥控、0xB1）。
而 `amphi.lua` 里 `use_imu_data=false` + `pose_extrapolator.use_imu_based=false`，
整套 SLAM 完全没用姿态信息。

**做法**：在 `uart_to_stm32` 里把 0x01 解析出来发一个 `sensor_msgs/Imu`（约 200Hz），
cartographer 配置改 `TRAJECTORY_BUILDER_2D.use_imu_data=true` +
`pose_extrapolator.use_imu_based=true`（并确认 IMU 的 frame/安装方向）。
预期收益：转弯/加减速时位姿明显更稳，跟丢概率大幅下降，`/velocity_map` 更干净。
**代价**：需要先验证飞控 0x01 帧的实际内容（四元数坐标系、单位）与时间戳。

### P1　考虑"先建图存图 + 纯定位"

场地是固定的（比赛现场不会变），完全可以在赛前跑一遍存下 `.pbstream`，
比赛时用 `pure_localization = true` 加载，省掉建图的收敛过程、也避免误回环。
代码里 `trajectory_builder.lua` 的 `pure_localization_trimmer` 已被注释掉，
说明这条路当初考虑过但没启用。
**风险**：如果现场货架位置与赛前不一致，纯定位会整体偏；而且题目不允许改场地，
但货架位置每场可能微调——所以**建议保留"现场重新建图+标定"作为主线**，
把纯定位当备选。

### P1　确认雷达的安装位置是否在机体旋转中心

`fly.urdf` 只给了 `laser_link` 的 z=0.1，没有 x/y 偏移。若雷达装在机头前方 10cm，
原地转 90° 时"雷达轨迹"和"机体轨迹"差 10cm，视觉对准/激光打点就会偏。
**做法**：量出雷达相对机体中心的 x/y 偏置，写进 URDF（`origin xyz="x y 0.1"`），
这样 PID、任务节点、速度差分都自动变准。

### P1　给 `shelf_mission_node` 加位姿陈旧保护

现在它只把 `has_pose_` 置 false 就继续推进（靠航段超时）。建议：
定位丢失超过 N 秒 → 进入"暂停"（冻结 leg 计时、可选发一次 `land` 或保持悬停），
恢复后再继续，避免"定位丢失期间按超时跳点"。

### P2　清理 launch 里的死代码

- `my_carto_pkg/launch/fly_carto.launch.py` 里 `rviz_node` **被注释掉了**，
  但 `use_rviz` 参数还在声明 —— 传 `use_rviz:=true` 不会有任何效果。
  （外层 `shelf_scan.launch.py` 传的 `use_rviz:=false` 也就无所谓了。）
- `cartographer_occupancy_grid_node` 一直在发 `/map`（1Hz、5cm），
  但机上没有任何消费者。要省 CPU 可以去掉，或改成只在调试时启动。
- `fly.lua` 是没被使用的备份配置，建议加注释说明或删掉，避免改错文件。

### P2　启动时序是硬编码的 TimerAction

`fly_carto.launch.py`：雷达 0s → robot_state_publisher 1s → cartographer **10s**；
外层：uart 2s → PID 6s → D 题 4 个节点 8s。
也就是说**任务节点（8s）比 cartographer（10s）先起 2 秒**——
任务节点会先打两秒"定位数据不可用"，然后等 TF 出现后再做 5 帧稳定判定才标定。
功能上没问题，但把 10s 硬编码在文件里容易踩坑（换机器/换雷达要改源码），
建议做成 launch 参数。

### P3　雷达驱动层的两个小点

- `filter_open`（邻域一致性中值滤波）默认 false，没有被启用；
  如果现场噪点多（网面多径），可以试用它，但会牺牲 5~10% 的点。
- 一圈完整性校验很严格（`total != 3600` 直接丢整圈）。15Hz 下丢一帧影响很小，
  但在串口误码率高的环境下会表现为 `/scan` 频率掉到 10Hz 以下——
  这时优先查 ttyS4 的线材/共地，而不是改校验。

---

## 10. 调试命令速查

```bash
source env.sh                    # 或 source install/setup.bash

# ① 雷达数据是否正常（应为 ~15Hz）
ros2 topic hz /scan
ros2 topic echo --once /scan | head -20

# ② TF 链是否完整（这是"定位好不好"的唯一判据）
ros2 run tf2_ros tf2_echo map laser_link        # 位姿应连续、小幅抖动，不该跳变
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link laser_link  # 应恒为 (0,0,0.1)

# ③ 图与占用栅格（1Hz 发布，5cm 分辨率）
ros2 topic hz /map

# ④ 任务节点的定位/标定状态
ros2 topic echo /shelf_scan/status              # calibrated / map_pose_cm / stage

# ⑤ PID 与飞控链路的诊断日志（每 0.5s 一条 FLIGHT_DIAG / VELOCITY_MAP）
ros2 topic echo /velocity_map
ros2 topic echo /target_velocity

# ⑥ 想看建图效果（香橙派上没接屏幕时，在电脑上开 RViz，ROS_DOMAIN_ID 要一致）
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix my_carto_pkg)/share/my_carto_pkg/rviz/cartographer.rviz

# ⑦ 存图 / 载图（当前未使用，纯定位方案要用到）
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/orangepi/map.pbstream', include_unfinished_submaps: true}"

# ⑧ 重新标定（飞机必须静止在起飞点）
ros2 topic pub --once /shelf_scan/gs_in std_msgs/msg/String \
  "{data: '{\"type\":\"cmd\",\"cmd\":\"calibrate\"}'}"
```

---

## 11. 一句话总结这套建图方案

**"机载 2D 激光 + cartographer 纯激光 SLAM，边飞边建图，地图原点随开机位置随机，
用起飞点做一次刚体标定把 map 对齐到题目场地坐标；位姿同时喂给位置 PID 和飞控速度环，
整条链路没有 IMU、没有先验地图、也没有里程计。"**

优点是**不依赖任何场地改造、不需要外部设备**，完全满足题目"辅助定位方式不限、
不得改变场地"的约束；短板是**没有 IMU 融合**、**每次上电都要重新建图并重新标定**，
精度的天花板主要由"起飞点摆放"和"回环是否出错"这两件事决定。

---

## 12. 是自启动建图，还是手动操控飞行建图？

**答：全自动在线建图，不需要也不支持"手动飞一圈建图"。** 建图与任务飞行是同一件事，
边飞边建；人只负责"摆位 + 一键启动 + 收尾"，全程不碰摇杆。

### 12.1 建图是自动拉起来的

- `bash run.sh` → `shelf_scan.launch.py` 里 `start_localization` 默认 `true`
  → 自动 include `my_carto_pkg/fly_carto.launch.py`；
- launch 用 `TimerAction` 自动编排时序，**没有任何一步需要人工执行**：

  | 时刻 | 自动启动的东西 |
  |---|---|
  | 0 s | 激光雷达驱动 `bluesea_node`（`/scan` 开始出数据） |
  | 1 s | `robot_state_publisher`（发布 `base_link→laser_link`） |
  | 2 s | `uart_to_stm32`（STM32 串口 + TF 差分速度） |
  | 6 s | `pid_control_pkg` 位置 PID |
  | 8 s | D 题 4 个节点（激光/识别/链路/任务） |
  | 10 s | `cartographer_node` + `cartographer_occupancy_grid_node`（开始建图） |

- 飞机停在起飞点**静止**时就已经在建图了：任务节点的"启动自动标定"要求
  **连续 5 帧位姿漂移 < 2cm**，本质上就是在等 SLAM 收敛。
- 地图是**边飞边扩展**的：起飞、巡航、盘点、返航的整个过程都在往里加子图，
  不是"先飞一圈把图建好，再飞第二圈干活"。

### 12.2 飞行本身也不是手动操控

- 代码里**没有任何订阅遥控器/手柄的入口**。`shelf_mission_node` 只接受
  地面站下发的 `start_route`（或本地兜底航线），航点写死在 `route.legs` 里；
- `uart_to_stm32` 里的 `/rc_channels` 只是把飞控**上传**的遥控通道打印出来做调试，
  不参与控制；
- 实际飞行由 `pid_control_pkg` 位置闭环完成：`/target_position`（程序给的）→
  PID → `/target_velocity` → 0x31 帧 → STM32；
- `tools/flight_test.py` 虽然是给调试用的，但它发的也是**自主小航线**
  （起飞 → 前飞 100cm → 返航降落），同样不是摇杆操控。

### 12.3 人到底要做哪几步

| # | 动作 | 性质 |
|---|---|---|
| 1 | 把飞机放在起飞点中心、机头正对货架（场地 0°） | 摆位（决定整个坐标系是否正确） |
| 2 | 上电、`bash run.sh` | 启动程序（雷达+cartographer 自动建图） |
| 3 | 等日志出现 `坐标标定完成（启动自动标定）` | 等待 |
| 4 | 地面站点「开始遍历盘点」（或发 `start_route`） | **一键启动** |
| 5 | —— 之后全程无人干预，飞机自主飞完并降落到降落点 —— | 自主 |
| 6 | 日志 `landed` 后遥控器接管、上锁 | 收尾（固件原因，见下） |

### 12.4 为什么"一键启动"这一步绕不开

按你们这台飞机的 `ANO_FC_407` 固件，**香橙派发速度 ≠ 飞机起飞**，中间有两道门控：

```c
// User_Task.c :: UserTask_OneKeyCmd()
CH6 拨到高位 1700~2200µs → 切模式2
   → if (m_speed_z > 10)  { 解锁 }        // 必须等机载端先发出上升速度
   → 等 1 s
   → takeoff_if = 1                       // 从这一刻起才采用香橙派的速度

// ANO_LX.c
if (takeoff_if) { rt_tar.st_data.vel_x = m_speed_x; ... }   // 香橙派的速度生效
else            { rt_tar.st_data.vel_x = 0;        ... }   // 一律清零
```

所以现场有两条等效路线：**地面站点按钮下发 `start_route`（机载端开始发 vz>10）+
遥控器 CH6 拨高位**，两者配合完成"一键起飞"；之后整个盘点过程完全自主。

顺带两个固件相关的事实（都写在 `docs/飞控端STM32对接香橙派改造清单.md`）：

- **遥控失控分支会清零速度并自动返航**（`RC_Data_Task()` 的 `fail_safe` 分支），
  所以比赛时必须保持遥控链路常开，或按文档打补丁让 `fail_safe` 时改用香橙派速度；
- **落地最后一步要人收尾**：`Set_m_speed()` 没有超时清零、固件也不解析 `0x66`，
  所以"机载端降到 8cm 之后由遥控器上锁落地"是这份固件的正常流程。

### 12.5 唯一一处"人工让飞机运动起来"的场合

`docs/现场联调与验收清单.md` 里有一条赛前检查：

> 手动把飞机沿通道挪 100cm，`gs_out` 遥测里的场地坐标随之变化约 100cm

这是**用手把飞机搬着走**来验证 `map→场地` 标定是否线性、是否正确——属于地面联调手段，
不是"手动飞行建图"，也不在正式流程里。

### 12.6 结论

```
摆位 → 上电（自动建图开始）→ 等自动标定 → 一键启动
      → 自主起飞 / 巡航 / 盘点 / 返航 / 降落（全程边飞边建图，无人干预）
      → 遥控接管上锁
```

没有"手动飞一圈建图"这个阶段，也没有这个功能；如果确实想让建图更稳，
只能靠**开机后多等一会儿（让 SLAM 收敛再标定）**，或者按第 9 节 P0 把飞控 IMU 接进来。
