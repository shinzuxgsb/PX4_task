# 无人机定位/控制栈（香橙派 + PX4）打包说明

适用机型：**MicoAir743 V2**（PX4 固件 1.15.4）+ **香橙派 5 Max**（Ubuntu 22.04 / ROS 2 Humble）
+ **LDS-50C-E 2D 雷达**（网口 UDP）+ **TOFSense-M v2.0**（串口测高）。

本包 = 我这台飞机**实际在跑**的代码（位置环、任务状态机、SLAM 配置、桥接、启动脚本）
+ 原工程里残留的**位置 PID / 航点**代码 + **飞控全参数快照** + 日志分析工具 + 一份样例飞行日志。
打包时间：2026-10-16。

---

## 0. 先看这个：定点漂移排查清单（按可能性排序）

### ① 先确认「外部定位真的进了 EKF」，而不是只看话题在跑

```bash
# 在香橙派上（先停链路释放串口）
sudo systemctl stop drone-link
python3 06_tools/px4_log.py list /dev/ttyS6
python3 06_tools/px4_log.py download <最新ID> /root/px4/logs/log_new.ulg /dev/ttyS6
sudo systemctl start drone-link

python3 06_tools/ulg_report.py /root/px4/logs/log_new.ulg      # 摘要 + 结论
python3 06_tools/ulg_flight_quality.py /root/px4/logs/log_new.ulg  # 晃动/漂移/高度
python3 06_tools/ulg_events.py /root/px4/logs/log_new.ulg      # 模式与解锁时间线
python3 06_tools/ulg_flight_trace.py /root/px4/logs/log_new.ulg 1.0 # 高度/速度/指令 时序表
```

**判据**：`xy_valid_pct ≈ 100%`、`v_xy_valid_pct ≈ 100%`、`max_eph_m` 厘米级、
`external_pose_fused = true`、`ev_max_innovation` 不超过 0.2。

如果 `xy_valid = 0%` / `max_eph_m` 涨到米级 → **问题在 SLAM→EKF 这一段，不在位置环**。
我们最早那次"定点起飞就漂"就是这个：eph 从 3cm 涨到 14m，水平位置全靠惯导推算。
同时对照 `05_px4_params/` 里的 `EKF2_EV_CTRL=15`（水平位置+速度融合）、
`EKF2_EV_DELAY`、`EKF2_EV_NOISE_MD=0`、以及 `EKF2_EV_POS/VEL` 噪声设置。

### ② 看 SLAM 位姿是否在跳（这个最容易被忽略）

```bash
journalctl -u drone-link -f | grep -E "位姿跳变|里程计跳变"
```

我们这台曾出现大量 `位姿跳变 100~150cm/s > 门限，丢弃`。
那是雷达/里程计的抖动，虽然被 `tf_to_odom` 丢掉了速度，但**位置照样会带着抖动进去**，
直接让位置保持变差。频繁出现请先查：

- 雷达是否随机身振动、安装是否松动（外参变了）；
- 雷达转速 / 网口丢包（`ip -s link show enP3p49s0`）；
- 地面纹理、玻璃/反光面导致的扫描匹配退化（2D SLAM 的通病）。

### ③ 再看水平环本身

现在的位置环在 `01_control_task/hover_test.py` 的 `hold_velocity()`：

```
世界系 PID：cmd = -kp·(位置偏差) - kd·(里程计实测速度) - i
再按【当前偏航】转到机体系；位置死区 1.5cm；限幅 30cm/s
I 项只在离起点 30cm 内累积、限幅 ±15cm/s（抗饱和）
```

**为什么必须要有 I 项**：实测这台飞机存在约 0.13m/s² 的恒定水平力偏置（悬停时
机身持续低头 ~0.75°，两架次可重复），相当于"小信号几乎没有平移权限"。纯 PD 只能靠
5~7cm 的位置偏差硬顶它，会留下 1.4cm/s 的残漂；加上条件积分后这部分静差被抠掉。
如果队友的飞机也是这个症状（悬停时能明显看出机身歪着、而位置环一直在给速度指令
却不怎么动），优先查机械：4 个电机座/桨的推力轴是否垂直、机架有没有扭、桨有没有变形。

起飞分两段：先爬到 `--climb-hold-cm`（12cm）稳住 `--climb-hold-s`（1.2s），
让水平环把地效/偏置吃掉，再继续爬到目标高度——起飞瞬间的 20~30cm 带偏就出在这里。

估计链（`02_bridge/config/tf_to_odom.yaml` + `mavlink_bridge.yaml`）也调过：
实测"指令→实际速度"滞后 300~400ms，主因是速度窗 0.2s + α0.25 低通、以及注入 EKF 的
观测噪声设成了 σ=10cm/10cm·s⁻¹（比雷达实际精度松一个数量级）。现在窗 0.14s、α0.40、
噪声 σ≈5cm/5cm·s⁻¹。若改完 `ev` 创新量变大或 `xy_valid` 掉，就往回放一档。

改完**先跑离线单测**（不需要飞机，直接验控制律符号）：

```bash
python3 01_control_task/test_hover_control.py     # 10 项，全过才算符号没错
```

调参顺序：

- 还飘 → `--xy-kd` 从 0.8 加到 1.0~1.2（加强速度阻尼）；
- 出现低频来回摆 → `--xy-kp` 从 1.5 降到 1.0；
- 速度反馈滞后太明显 → 把 `02_bridge/config/tf_to_odom.yaml` 的
  `velocity_window_s` 从 0.20 降到 0.15。

### ④ 对照基线（我们这台实测，可直接比）

`07_sample_log/flight_quality.txt`（2026-10-16 那次定点悬停）：

```
外部定位:  xy_valid 100% (113/113)，v_xy_valid 100%，eph 峰值 3cm，ev 创新量最大 7.1cm
悬停段:    高度 68.0 ± 12.0 cm，最大偏差 33cm
          水平晃动 RMS 11.9cm，漂移速率 4.75cm/s，累计位移 44cm
判定:      "稳"（判据：晃动 RMS<15cm 且 漂移<5cm/s 且 高度σ<15cm）
```

队友那台如果漂移明显大于这个量级，按 ①②③ 的顺序查，不要先动 PID 参数。

### ⑤ 确认它真的是「定点」模式

```bash
python3 06_tools/ulg_events.py <log.ulg>     # 看模式时间线，确认是 POSCTL 而不是 STABILIZED
```

遥控器 ch5 三段开关会被 PX4 映射到 **6 档里的 1 / 4 / 6 档**（`rc_update.cpp` 的档位公式），
所以 `COM_FLTMODE1 / 4 / 6` 才对应三段开关的三个位置。
我们这台：`COM_FLTMODE1=8`(Stabilized)、`COM_FLTMODE4=2`(Position)、`COM_FLTMODE6=7`(Offboard)。
如果 4 档没设成 `2`(Position)，中间档就根本不是定点。

### ⑥ 供电与振动

电池电压低时 EKF 与 SLAM 都会退化。看控制站页面的 `battery_v/pct`，或 `06_tools/px4_status.py`。

---

## 1. 目录说明

| 目录 | 内容 | 板上路径 |
|---|---|---|
| `01_control_task/` | **位置环 + 任务状态机**（`hover_test.py` 解锁→OFFBOARD→起飞→悬停→降落）、离线单测、`run_hover.sh`/`stop_hover.sh`、`drone-link.service`、`start_link.sh` | `/home/orangepi/drone_link/tasks/` |
| `02_bridge/` | **MAVLink 桥接 ROS 2 包**：`bridge_node.py`（里程计注入/offboard setpoint/高度/就绪/命令/飞控提示透传）、`tf_to_odom.py`（SLAM→带速度里程计）、`tofsense_node.py`、launch、config | `/home/orangepi/ros2_ws/src/mavlink_bridge/` |
| `03_slam/` | Cartographer 配置（`amphi.lua` 等）、URDF（`fly.urdf`，雷达安装外参）、雷达驱动配置（`lidar_net.yaml`） | `~/install/my_carto_pkg/share/my_carto_pkg/` |
| `04_original_pid_waypoints/` | **原工程**的位置 PID（`pid_control_pkg`：`pid_controller.cpp` + `bspline_trajectory_tracker.cpp`）与航点/任务（`activity_control_pkg`：`route_target_publisher`、`waypoint_navigator`、`pure_waypoints.yaml`） | `/home/orangepi/ros2_ws/src/` |
| `05_px4_params/` | 飞控**全参数快照**（1046 项，2026-10-16 导出） | — |
| `06_tools/` | 日志/参数/命令工具：`px4_log.py`（MAVLink 拉 SD 卡日志）、`ulg_report/events/flight_quality/flight_trace.py`（解析 ulog）、`px4_param.py`、`px4_param_dump.py`、`px4_shell.py`（nsh 只读诊断）、`px4_status.py` | `/root/px4/tools/` |
| `07_sample_log/` | 样例飞行日志 `log_035.ulg` + 4 份分析输出（摘要/事件/品质/时序） | `/root/px4/logs/` |
| `08_gs_web/` | 网页控制站（笔记本侧服务 + 香橙派侧任务脚本）：链路启停/重置原点/拉起悬停任务/飞控命令/飞行记录。启动：`launch_web_gs.bat` 或 `start_web_gs.cmd` | `web_gs/` |
| `09_gs_desktop_src/` | D 题货架盘点桌面站源码（PySide6，含 24 格航线/几何校验；不含打包好的 exe）。`USAGE.txt` 是原使用说明 | `gs_d/` |
| `10_notes/` | 我们整理的解析文档：`slam_localization_notes.md`（建图与定位链路，**先看这份**）、`px4_firmware_notes.md`（飞控代码）、`problem_d_notes.md` / `problem_d_gs_notes.md` / `drone_d_ws_notes.md`（D 题思路与地面站）、`px4_reference_index.md`（资料索引）、`field_topview_waypoints.png`（场地俯视图与航点） | — |

---

## 2. 数据流（这台飞机实际链路）

```
雷达 LDS-50C-E (UDP, 192.168.158.98:6543)
  → bluesea2 驱动 → /scan @15Hz
  → Cartographer 2D（amphi.lua，odom→base_link）
  → tf_to_odom（0.2s 窗差分 + α=0.25 低通 + 2cm/s 死区 + 100cm/s 跳变门限）→ /odom_from_slam @30Hz（位姿+速度）
  → mavlink_bridge：MAVLink ODOMETRY（MAV_ESTIMATOR_TYPE_VIO）→ PX4 EKF2
       水平位置/速度融合；垂直方向方差设 1e6（2D 雷达没有高度，直接忽略）
TOFSense-M (ttyS3 @921600) → /tofsense/range → MAVLink DISTANCE_SENSOR → EKF 定高

飞控 → 桥接：HEARTBEAT / LOCAL_POSITION_NED / ESTIMATOR_STATUS / STATUSTEXT / COMMAND_ACK
  → /height(cm)、/is_st_ready(1=已解锁且 EKF 有效)、/mission_step
控制任务 → /target_velocity [vx,vy,vz cm/s, vyaw deg/s]（机体系 FLU）
  → 桥接 SET_POSITION_TARGET_LOCAL_NED（BODY_NED, 20Hz）→ PX4 OFFBOARD 速度控制
控制任务 → /px4_command → arm / disarm / position / offboard / altitude / manual / land
```

---

## 3. 部署步骤（换到队友那台相同结构的飞机）

1. **雷达网络**：板上网口（我们这里是 `enP3p49s0`）设 `192.168.158.100/24`；
   改 `03_slam/lidar_net.yaml` 里的 `lidar_ip` / `local_port` 为对方雷达的地址。
2. **ROS 2 包**：把 `02_bridge/` 放进 `<ws>/src/`，需要航点/PID 参考就把 `04_original_pid_waypoints/*` 也放进去；
   `colcon build --packages-select mavlink_bridge`。
3. **控制任务**：`01_control_task/` 下的东西按表放到 `/home/orangepi/drone_link/`；
   `drone-link.service` 放 `/etc/systemd/system/`，然后 `systemctl daemon-reload && systemctl enable --now drone-link`。
4. **SLAM 配置**：`03_slam/{configuration_files,urdf,launch}` 放到 `my_carto_pkg` 的 share 目录，
   或改 `02_bridge/launch/drone_stack.launch.py` 里的 `DEFAULT_MY_CARTO_SHARE`。
5. **飞控参数**：对照 `05_px4_params/px4_params_2026-10-16.txt`，至少核对
   `EKF2_EV_CTRL=15`、`EKF2_EV_DELAY`、`EKF2_EV_NOISE_MD=0`、
   `COM_RC_IN_MODE`、`COM_FLTMODE1/4/6`、`COM_OBL_RC_ACT`、`RC_MAP_ARM_SW`、`NAV_DLL_ACT`。
6. **先干跑再实飞**：
   ```bash
   sudo -u orangepi bash /home/orangepi/drone_link/tasks/run_hover.sh --dry-run
   sudo -u orangepi bash /home/orangepi/drone_link/tasks/run_hover.sh --height 50 --hover 5
   ```

---

## 4. 已知坑（我们都踩过，别重复踩）

1. **OFFBOARD 模式的解锁门禁**：飞机停在 OFFBOARD 而当时没有 offboard setpoint 流时，
   PX4 会直接拒绝 arm（`Arming denied: Resolve system health failures first` + 事件 `No offboard signal`）。
   OFFBOARD 是唯一一个"要求已有 setpoint 流才让解锁"的模式（`modeCheck` + `offboardCheck`）。
   我们的任务因此改成：**从 INIT 起就 20Hz 喂零速度 setpoint**，并在解锁前先切回 Position。
2. **ROS 2 DDS 跨用户**：任务必须以 `orangepi` 用户运行（`sudo -u orangepi bash ...`）。
   用 root 跑会"发现得了发布者但收不到数据"。
3. **PX4 会在上锁时自动保存被改过的参数**：临时改参数做实验后，记得改回去并 `param save`，
   否则重启后会带着实验值（我们被 `COM_ARMABLE=0` 坑过一次）。
4. **桥接必须发 GCS 类型心跳**：不发心跳时 PX4 认为地面站未连接，
   而且 `STATUSTEXT`（拒解锁原因）根本不会送到这条链路上，出问题只能靠猜。
   `02_bridge/config/mavlink_bridge.yaml` 里 `send_heartbeat: true` + `heartbeat_type_gcs: true` 就是为此。
5. **journald 易失**：`journalctl` 只有当前那次启动的日志。要历史必须靠 SD 卡 ulog；
   而 `SDLOG_MODE=0` 表示**只在解锁期间记录**——没解锁就没有日志，别以为记录丢了。
6. **长连接 SSH 会半开**：笔记本侧控制站曾因为香橙派掉线导致 `ssh journalctl -f` 卡死 15 分钟，
   页面一直显示旧数字（页面看着一切正常，数字其实是十几分钟前的）。解决办法已写进
   `08_gs_web/server.py`：SSH keepalive + 陈旧看门狗 + 页面"数据已停更"红条。
7. **给 Windows 写 `.bat/.cmd`**：必须纯 ASCII + CRLF，中文或 LF 会让 cmd 解析错乱。

---

## 5. 距离第一次实飞的推荐顺序

1. 台架：手推飞机，看 `/odom_from_slam` 位置与 PX4 `LOCAL_POSITION_NED` 是否一致跟随；
2. `run_hover.sh --dry-run`：只看话题/逻辑，不解锁；
3. 无桨上电：确认 arm、切 OFFBOARD、`is_st_ready=1` 都正常；
4. 装桨实飞前，先跑一次 `01_control_task/test_hover_control.py` 确认控制律没被改坏。

有问题随时找我，我这边有这套链路的完整调试记录。
