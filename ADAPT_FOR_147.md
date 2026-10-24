# 队友那套工作空间 → 放到第一架（147）上的适配说明

> 来源：队友交接包 `新建文件夹/`（原机：香橙派 5 Max，10.23.87.150，跑 `drone-link.service`）。
> 搬运时间：2026-10-16。**目的：在 147 上原样试这套 MAVLink 定位链路**，与 147 现有
> `~/px4_ws`（DDS EV 链）互不干扰（不同目录、不改系统配置）。

## 1. 包内容对照

| 新工作空间里的路径 | 来自交接包 | 说明 |
| --- | --- | --- |
| `src/mavlink_bridge/` | `02_bridge/` | 原样（只改了 `launch/drone_stack.launch.py` 两行，见 §2） |
| `src/my_carto_pkg/` | `03_slam/` | 配置/launch/urdf **逐字节未改**；只新增了 `package.xml`+`CMakeLists.txt`（原机上这个包以 `/home/orangepi/install/my_carto_pkg` 形式存在，交接包没带包骨架） |
| `src/bluesea2/` | 取自 147 的 10-09 留档 | 雷达驱动（原机在它自己的 `~/ros2_ws/src`）。**注意**：这是 10-09 那版，不含 10-13 的"串口打不开就退出"补丁 |
| `config/lidar_uart.yaml` | 147 现有 `uart_lidar.yaml` | **147 的雷达是串口款 LDS-50C-R（/dev/ttyS4 @921600）**，故默认用它 |
| `config/lidar_net.yaml` | 交接包 `03_slam/lidar_net.yaml` | 队友原机的网口款参数（备着，`lidar_params_file:=` 可切） |
| `tasks/` | `01_control_task/` | 悬停测试（`hover_test.py`）等脚本，原样保留作参考 |
| `optional_pkgs/` | `04_original_pid_waypoints/` | 原工程残留的 PID/航点 C++ 包；**放在 src 之外、不参与编译** |
| `ref/` | `README.md`、`05_px4_params/`、`07_sample_log/`、`10_notes/` | 参考材料（飞控参数快照、样例 ulog、笔记） |

## 2. 为 147 改的两处（都在 `src/mavlink_bridge/launch/drone_stack.launch.py`）

1. `DEFAULT_MY_CARTO_SHARE`：原来写死 `/home/orangepi/install/my_carto_pkg/share/my_carto_pkg`
   → 改成 `get_package_share_directory("my_carto_pkg")`（跟着 overlay 走）。
2. `DEFAULT_LIDAR_PARAMS`：原来写死 `/home/orangepi/lidar_net.yaml`（网口）
   → 改成包内 `config/lidar_uart.yaml`（147 的串口雷达）；要回网口加 `lidar_params_file:=`。

另外 `setup.py` 的 `data_files` 里补了两行，把 `lidar_uart.yaml` / `lidar_net.yaml` 装进 share。

## 3. 跑之前必须满足的条件（否则要么起不来，要么和现有链路打架）

1. **先停旧链**：`sudo systemctl stop px4-ev-chain`
   —— 新链要独占 `/dev/ttyS4`(雷达) / `/dev/ttyS3`(定高) / `/dev/ttyS6`(飞控)，和 `px4_ws` 那条链完全冲突。
2. **飞控参数要切到 MAVLink**（QGC 里改，只有人工能做）：
   - `UXRCE_DDS_CFG = 0`（关掉 DDS；147 现在是 TELEM3 上跑 uXRCE-DDS）
   - `SER_TEL3_BAUD = 115200`（147 现在是 1500000）
   - 这套是靠 **MAVLink ODOMETRY + TIMESYNC** 把里程计喂给 EKF，并用 `EKF2_HGT_REF`（队友那台是 2=Range）取高度；
     147 现在是 `EKF2_HGT_REF=3(Vision)`，想完全对齐队友那台就把这项也改成 2（并在飞控端接测距）——**改之前想清楚**。
3. `python3 -c "import pymavlink"` 必须能过；不过就 `pip3 install --user pymavlink`（只装到 `~/.local`）。
4. 依赖：`ros-humble-cartographer-ros`（147 已有 ✓）、`robot_state_publisher` ✓。

## 4. 怎么跑

```bash
bash ~/mavlink_ws/start_mavlink_ws.sh          # 起整套（串口雷达）
bash ~/mavlink_ws/start_mavlink_ws.sh --net    # 用网口雷达参数
bash ~/mavlink_ws/start_mavlink_ws.sh stop     # 停
```

判据（队友 README 里那套）：
```bash
ros2 topic hz /scan                 # 15 Hz 左右
ros2 topic hz /odom_from_slam       # 30 Hz 左右
ros2 topic echo /tofsense/range --once
```
飞控侧再确认：`listener vehicle_odometry`/`estimator_status_flags` 里 `cs_ev_*` 是否 true、`xy_valid` 是否 true。

## 5. 与 147 现有方案的差别（做对比时要记住）

| | 147 现有 `~/px4_ws`（10-09 留档） | 本工作空间（队友那套） |
| --- | --- | --- |
| 定位注入 | uXRCE-DDS（Agent + FastDDS，1.5 Mbaud） | **MAVLink ODOMETRY**（115200） |
| 高度 | EV 桥带 TOF 高度一起发 | 里程计 + `EKF2_HGT_REF`（队友设 2=Range） |
| SLAM | cartographer `fly_no_imu.lua`（纯激光） | cartographer **`amphi.lua`**（与 10-09 留档里那份逐字节相同） |
| 控制 | PX4 Position 模式 + 遥控器 | 队友自带位置 PID（`tasks/hover_test.py`，走 `/target_velocity` → SET_POSITION_TARGET_LOCAL_NED） |

## 6. pymavlink 怎么解决的（不动系统包）
香橙派上没有 pip（`pip3: command not found`），所以从 PyPI 取了 aarch64/cp310 的 wheel
（`pymavlink 2.4.50` + 依赖 `fastcrc 0.5.0`），解包后放在 `mavlink_ws/third_party/`，
启动脚本里用 `export PYTHONPATH="$WS/third_party"` 挂载。系统里**没有**装任何包。
验证：`PYTHONPATH=~/mavlink_ws/third_party python3 -c "from pymavlink import mavutil; print('ok')"`

## 7. 最终状态（2026-10-17，已飞通）
- 开机自启：`mavlink-ws-chain.service`（enabled + active）；`px4-ev-chain` / `px4-p2p-chain` 均 disabled 但保留。
- 依赖：147 上没有 pip，pymavlink 2.4.50 / fastcrc 0.5.0 / pyserial 3.5 用 wheel 解包挂在 `third_party/`，启动脚本 export PYTHONPATH；系统里没装任何包。
- 启动脚本：删掉 `set -u`（否则 `source /opt/ros/humble/setup.bash` 报 `AMENT_TRACE_SETUP_FILES`）；导出 `FASTRTPS_DEFAULT_PROFILES_FILE=$WS/config/fastdds_udp_only.xml`。
- 飞控参数（QGC 改，改完必须重启飞控）：`UXRCE_DDS_CFG=0`、`SER_TEL3_BAUD=115200`、`MAV_1_CONFIG=103`、`EKF2_HGT_REF=2`、`EKF2_RNG_CTRL=2`、`EKF2_RNG_POS_Z=0.044` + 照抄队友的 `EKF2_RNG_*` / `EKF2_GND_*` / `EKF2_BARO_CTRL=1`（147 固件没有 `MAV_1_MODE`/`MAV_1_FORWARD`）。
- 实测：`链路=OK`、`ekf_ok=True`、`坏帧=0`；`listener vehicle_visual_odometry` 与 `listener distance_sensor` 均有数据；高度稳定 9~10cm；实际飞行成功且优于原方案。
- 回滚：`sudo systemctl disable --now mavlink-ws-chain && sudo systemctl enable --now px4-ev-chain`，并把飞控参数改回（`UXRCE_DDS_CFG=TELEM3`、`SER_TEL3_BAUD=1500000`、`MAV_1_CONFIG` 复原）。

## 8. 跳变保护（2026-10-17，**刻意偏离队友原版**）
- 事故：2026-10-17 22:07，SLAM 单帧跳 **272.35 m**。原版逻辑（丢一帧 + reset_counter+1 + 基准挪到新位置）让下一帧以新 reset_counter 变成"合法重定位"，PX4 把 EKF 位置锚到 273 m 外，位置环追这个假目标 → 翻机。
- 改法（`bridge_node.py` 的 `_odom_callback` 第 4 段）：
  · 启动**预热**：位置连续稳定 `odom_jump_hold_s`（默认 2 s）才开始发 EV；
  · 运行中跳变 > `odom_max_position_jump_m`：**停发 EV、不递增 reset_counter**；
    - 位姿回到"最后一个可信位置"附近并稳定 2 s → 恢复发（EKF 不重置）；
    - 位姿在新位置稳定 2 s → 判定为真重定位，**继续停发**并打 ERROR（需人工停链重启）。
- 新增参数：`odom_jump_hold_s: 2.0`（`config/mavlink_bridge.yaml`）。
- 备份：`src/mavlink_bridge/mavlink_bridge/bridge_node.py.bak-20261017`（原版 41187 B）、`config/mavlink_bridge.yaml.bak-20261017`。
- 回滚：把两个 `.bak-20261017` 覆盖回去 → `colcon build --symlink-install --packages-select mavlink_bridge` → `sudo systemctl restart mavlink-ws-chain`。

## 9. Cartographer 2.0.9003 源码编译并切换（2026-10-19）
- 背景：147 原先用的是 apt 混装版本（`cartographer` 2.0.9004 + `cartographer_ros` 2.0.9002），队友那台是源码编译的 2.0.9003/2.0.9003。apt 源里没有 9003，因此只能在 147 上源码编译。
- 源码：队友的 `cartographer_ws`，`cartographer` 提交 `ddfd4414ef2907b6c46a195f1d4e380beedce05d`，`cartographer_ros` 提交 `1bbf9af3d250ba7d17f9b2340e7fe01ac22cf7a7`，均为 `ros2` 分支、版本 2.0.9003。
- 编译位置：`/home/orangepi/cartographer_ws`；补装了 `liblua5.3-dev`，其余依赖系统已有。
- 编译命令：
  ```bash
  cd /home/orangepi/cartographer_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --parallel-workers 2 \
    --packages-select cartographer cartographer_ros cartographer_ros_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
  结果：3 个包全部成功，总耗时 10 min 10 s；生成 `install/cartographer/lib/libcartographer.a` 和 `install/cartographer_ros/lib/cartographer_ros/cartographer_node`。
- 启动切换：只改 `start_mavlink_ws.sh`，在 `source /opt/ros/humble/setup.bash` 之后、`source "$WS/install/setup.bash"` 之前加一行：
  ```bash
  source /home/orangepi/cartographer_ws/install/setup.bash
  ```
  这样 `cartographer_ros`、`cartographer_ros_msgs` 和核心库都优先用新编译的 9003；`mavlink_bridge` 等原工作空间包仍来自 `mavlink_ws/install`。
- 备份：`start_mavlink_ws.sh.bak-20261019`。
- 验证：
  · `ros2 pkg prefix cartographer_ros` → `/home/orangepi/cartographer_ws/install/cartographer_ros`；
  · `ros2 pkg xml cartographer_ros` / `cartographer_ros_msgs` → `<version>2.0.9003`；
  · `cartographer_ros/CMakeCache.txt` 的 `cartographer_DIR` 指向新安装目录，`link.txt` 链接新 `libcartographer.a`；
  · 进程路径为 `/home/orangepi/cartographer_ws/install/cartographer_ros/lib/cartographer_ros/cartographer_node`；
  · 服务 `mavlink-ws-chain` active，`ekf_ok=True`，`/scan` 约 15.1 Hz，ODOMETRY 持续递增，跳变丢弃 0。
- 未改动：`my_carto_pkg` 的 7 个 `.lua`、launch、URDF、`tf_to_odom.yaml` 均未改，仍与队友文件逐字节一致。
- 回滚：恢复 `start_mavlink_ws.sh.bak-20261019`（或删掉新增的 `source` 行）→ `sudo systemctl restart mavlink-ws-chain`，即回到 apt 的 9004/9002。

### 9.1 补充结论（2026-10-19 21:40）
- 代码对比：`2.0.9004` 相对 `2.0.9003` 只改了 `package.xml` 和 CMake 查找 Boost；
  `2.0.9002 → 2.0.9003` 只改了 Ceres 2.1 的 `Manifold` API 和 rosbag2 Rolling 编译兼容。
  147 的系统 Ceres 是 **2.0.0**，`Manifold` 分支根本不会被编译，所以 9003/9004 的 2D SLAM 算法基本一致。
- 源码里的未提交补丁：landmark 清理（`use_landmarks=false`，不生效）、velocity/tracked_pose 发布
  （`publish_tracked_pose` 已关闭，`/velocity_map` 没有订阅者，不生效）。
- 因此“不炸机”主要不是 Cartographer 版本变了，而是 `bridge_node.py` 的 fail-closed 跳变保护：
  跳变 > `odom_max_position_jump_m`（0.5 m）就停发 EV、不递增 `reset_counter`。
- 2026-10-19 21:28:47 仍出现 **4.95 m** 跳变，桥停发 EV、`ekf_ok=False`；
  21:39:41 地面重启后预热恢复，ODOMETRY 正常、跳变 0。说明跳变根因仍在，只是被桥拦住了。

## 10. 启动前等待 NTP 同步（2026-10-20）
- 事故证据：20:16:40.523 `systemd-timesyncd` 首次校时；20:16:40.887 桥检测到
  **180.34 m** SLAM 跳变；20:16:47 Cartographer 报 `scan rate: 0.03 Hz`、
  时间跨度 `38s ± 569s`，20:17:02 才恢复 15.12 Hz。
- 根因：`lidar_uart.yaml` 未设置 `time_mode`，`bluesea2` 串口驱动默认 `time_mode=0`
  （系统时间），`DecTimestamp()` 用 `gettimeofday()`；链路先启动、NTP 后校时，
  雷达扫描时间戳被拉跳，Cartographer 输出数百米假位姿。
  - 2026-10-21: NTP disabled. start_mavlink_ws.sh no longer waits; systemd-timesyncd disabled; no time-sync dependency.
