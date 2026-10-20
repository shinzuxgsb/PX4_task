# PX4 飞控资料索引（官方手册 / 开源代码 / 生态工具）

> 整理日期：2026-10-06
> 所有链接均已实测可访问（HTTP 200）。
> 面向场景：室内无 GPS 全自主多旋翼（立体货架盘点，见 `D题_解析与实现思路.md`）。

---

## 1. 官方文档（第一优先级）

PX4 文档是 VuePress 静态站，同一页面通过语言/版本前缀切换：

| 入口 | 地址 |
|---|---|
| 最新开发版（main，英文） | https://docs.px4.io/main/en/ |
| 最新开发版（main，**中文**） | https://docs.px4.io/main/zh/ |
| 稳定版 v1.17（英文） | https://docs.px4.io/v1.17/en/ |
| 官网 | https://px4.io |

> 中文翻译通常滞后于英文，遇到"页面只有英文"时属正常，接口/参数以英文 main 为准。

### 1.1 入门与基础

| 内容 | 地址 |
|---|---|
| 快速上手（Basic Concepts） | https://docs.px4.io/main/en/getting_started/px4_basic_concepts.html |
| 飞控选型 | https://docs.px4.io/main/en/getting_started/flight_controller_selection.html |
| 飞行模式总览 | https://docs.px4.io/main/en/getting_started/flight_modes.html |
| 飞行模式（多旋翼） | https://docs.px4.io/main/en/flight_modes_mc/ |
| 飞行模式（概念/实现） | https://docs.px4.io/main/en/concept/flight_modes.html |

### 1.2 架构与中间件（读源码前必看）

| 内容 | 地址 |
|---|---|
| 系统架构总览 | https://docs.px4.io/main/en/concept/px4_systems_architecture.html |
| 飞行栈架构 | https://docs.px4.io/main/en/concept/architecture.html |
| 控制分配 Control Allocation | https://docs.px4.io/main/en/concept/control_allocation.html |
| uORB 消息总线 | https://docs.px4.io/main/en/middleware/uorb.html |
| uORB 消息图（可视化） | https://docs.px4.io/main/en/middleware/uorb_graph.html |
| uXRCE-DDS（ROS 2 桥接，现用方案） | https://docs.px4.io/main/en/middleware/uxrce_dds.html |
| MAVLink 中间件 | https://docs.px4.io/main/en/middleware/mavlink.html |
| 启动流程 system_startup | https://docs.px4.io/main/en/concept/system_startup.html |

### 1.3 模块与消息参考（写代码时的字典）

| 内容 | 地址 | 说明 |
|---|---|---|
| 模块参考（应用/驱动） | https://docs.px4.io/main/en/modules/modules_main.html | 25 个模块页，含各 module 的启动参数 |
| uORB 消息参考 | https://docs.px4.io/main/en/msg_docs/ | 298 个消息定义，随 main 分支自动生成 |
| 参数参考 | https://docs.px4.io/main/en/advanced_config/parameter_reference.html | 全部参数默认值与说明 |

### 1.4 调试与日志

| 内容 | 地址 |
|---|---|
| 调试总目录 | https://docs.px4.io/main/en/debug/ |
| ULog 日志格式 | https://docs.px4.io/main/en/log/ulog_file_format.html |
| 日志分析（Flight Review） | https://docs.px4.io/main/en/log/flight_review.html |
| 系统控制台 / MAVLink Shell | https://docs.px4.io/main/en/debug/system_console.html |

---

## 2. 针对室内自主盘点最相关的官方章节

这几节与 D 题（无 GPS、全自主、需要定位+航线）直接对应，建议优先精读：

### 2.1 室内定位 / 状态估计（核心难点）

| 内容 | 地址 |
|---|---|
| 视觉惯性里程计 VIO | https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry.html |
| 动捕系统 Motion Capture | https://docs.px4.io/main/en/computer_vision/motion_capture.html |
| 外部位置估计（ROS 融合） | https://docs.px4.io/main/en/ros/external_position_estimation.html |
| 光流 Optical Flow | https://docs.px4.io/main/en/sensor/optical_flow.html |
| RTK / 高精度定位（备选） | https://docs.px4.io/main/en/advanced_features/rtk-gps.html |

### 2.2 Offboard 外部控制（实现"自主飞航线"）

| 内容 | 地址 |
|---|---|
| ROS 2 Offboard 控制（推荐） | https://docs.px4.io/main/en/ros/ros2_offboard_control.html |
| ROS 2 通信总览 | https://docs.px4.io/main/en/ros/ros2_comm.html |
| ROS 2 用户指南 | https://docs.px4.io/main/en/ros2/user_guide.html |
| Offboard 控制概念 | https://docs.px4.io/main/en/ros/offboard_control.html |
| MAVROS Offboard（C++） | https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html |
| MAVROS Offboard（Python） | https://docs.px4.io/main/en/ros/mavros_offboard_python.html |

### 2.3 避障 / 碰撞预防 / 路径规划

| 内容 | 地址 |
|---|---|
| 障碍物避让 Obstacle Avoidance | https://docs.px4.io/main/en/computer_vision/obstacle_avoidance.html |
| 碰撞预防 Collision Prevention | https://docs.px4.io/main/en/computer_vision/collision_prevention.html |
| 路径规划接口 | https://docs.px4.io/main/en/computer_vision/path_planning_interface.html |
| 安全降落 Safe Landing | https://docs.px4.io/main/en/computer_vision/safe_landing.html |
| 视觉目标估计 | https://docs.px4.io/main/en/advanced_features/vision_target_estimator.html |

### 2.4 仿真（先仿真跑通再上真机）

| 内容 | 地址 |
|---|---|
| 仿真总览 / SITL 快速开始 | https://docs.px4.io/main/en/simulation/px4_simulation_quickstart.html |
| SITL 预编译包 | https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages.html |
| HITL 硬件在环 | https://docs.px4.io/main/en/simulation/hitl.html |
| Gazebo 仿真（sim_gazebo_gz） | https://docs.px4.io/main/en/sim_gazebo_gz/ |
| ROS 接口仿真 | https://docs.px4.io/main/en/simulation/ros_interface.html |
| 多机仿真 | https://docs.px4.io/main/en/simulation/multi-vehicle-simulation.html |

---

## 3. 开源代码仓库（PX4 GitHub 组织）

| 仓库 | 星标 | 说明 | 地址 |
|---|---|---|---|
| PX4-Autopilot | 12.7k | **主固件仓库**，C++，BSD-3-Clause | https://github.com/PX4/PX4-Autopilot |
| PX4-ECL | 543 | 估计与控制库（EKF2、姿态/位置控制核心） | https://github.com/PX4/PX4-ECL |
| PX4-Matrix | 193 | 轻量矩阵库（源码里大量 `matrix::` 的来源） | https://github.com/PX4/PX4-Matrix |
| NuttX | 173 | PX4 定制的 NuttX 实时内核 | https://github.com/PX4/NuttX |
| PX4-Bootloader | 319 | Bootloader（PX4FMU/IO/FLOW） | https://github.com/PX4/PX4-Bootloader |
| px4_msgs | 160 | ROS 2 消息定义（与 uORB 对应） | https://github.com/PX4/px4_msgs |
| px4_ros_com | 226 | ROS2/ROS 桥接（旧方案，现多用 uXRCE-DDS） | https://github.com/PX4/px4_ros_com |
| PX4-Avoidance | 752 | ROS 避障节点（含局部/全局规划器） | https://github.com/PX4/PX4-Avoidance |
| PX4-OpticalFlow | 201 | 光流算法（室内定速/定位用） | https://github.com/PX4/PX4-OpticalFlow |
| PX4-Flow | 254 | PX4FLOW 板上固件 | https://github.com/PX4/PX4-Flow |
| PX4-GPSDrivers | 108 | 平台无关 GPS 驱动 | https://github.com/PX4/PX4-GPSDrivers |
| jMAVSim | 107 | 轻量多旋翼仿真器（MAVLink） | https://github.com/PX4/jMAVSim |
| PX4-SITL_gazebo-classic | 466 | Gazebo Classic 的 SITL 插件与模型 | https://github.com/PX4/PX4-SITL_gazebo-classic |
| PX4-gazebo-models | 106 | Gazebo 模型库 | https://github.com/PX4/PX4-gazebo-models |
| pyulog | 171 | Python 解析 ULog 日志 | https://github.com/PX4/pyulog |
| ulog_cpp | 27 | C++ 读写 ULog | https://github.com/PX4/ulog_cpp |
| FlightPlot | 74 | 飞行日志绘图（Java） | https://github.com/PX4/FlightPlot |
| flight_review | 276 | 在线日志分析 Web（Flight Review） | https://github.com/PX4/flight_review |
| DriverFramework | 86 | OS 无关驱动框架（POSIX/NuttX） | https://github.com/PX4/DriverFramework |

### 3.1 文档源码仓库

| 仓库 | 说明 | 地址 |
|---|---|---|
| PX4-Autopilot `docs/` | **官方手册源码目录**（main 分支即最新文档） | https://github.com/PX4/PX4-Autopilot/tree/main/docs |
| docs.px4.io | 生成后的 HTML 文档 | https://github.com/PX4/docs.px4.io |
| PX4-user_guide | 旧版用户指南源（已归档） | https://github.com/PX4/PX4-user_guide |
| PX4-Devguide | 旧版开发者指南源（GitBook，已归档） | https://github.com/PX4/PX4-Devguide |

---

## 4. 配套生态工具

| 工具 | 用途 | 地址 |
|---|---|---|
| QGroundControl | 地面站（参数、航线、日志、校准） | https://github.com/mavlink/qgroundcontrol |
| MAVSDK | 跨语言 SDK（C++/Python/Swift…）做 Offboard 与任务 | https://github.com/mavlink/MAVSDK |
| MAVLink | 通信协议与消息定义 | https://github.com/mavlink/mavlink |
| Micro-XRCE-DDS-Agent | ROS 2 ↔ PX4 的 DDS 桥（配合 uXRCE-DDS） | https://github.com/eProsima/Micro-XRCE-DDS-Agent |
| PX4 ROS 2 接口库 | 官方 ROS 2 控制/导航/航点接口 | https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html |
| PX4 ROS 2 控制接口 | 直接接管控制的官方接口 | https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html |

---

## 5. 版本与许可

| 项目 | 值 |
|---|---|
| 主仓库默认分支 | `main` |
| 最新稳定版 | **v1.17.0**（2026-05-13） |
| 在测版本 | v1.18.0-rc1（2026-09-10） |
| 许可证 | BSD-3-Clause |
| 最近提交 | 2026-10-05（活跃开发中） |

> 生产/比赛建议锁 **v1.17.0 稳定 tag**，不要用 `main`；文档对应看 `https://docs.px4.io/v1.17/en/`。

---

## 5.1 QGroundControl 安装（版本选择）

**结论：装 stable v5.1.5。** 与上一条「PX4 锁 v1.17.0 稳定版」配套。

| 平台 | 文件 | 直链 |
|---|---|---|
| Windows x86_64（常规 PC） | `QGroundControl-installer-AMD64.exe`（约 129 MB） | https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl-installer-AMD64.exe |
| Windows ARM64（骁龙 X 等） | `QGroundControl-installer-ARM64.exe` | https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl-installer-ARM64.exe |
| macOS（通用二进制） | `QGroundControl.dmg` | https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl.dmg |
| Linux x86_64 | `QGroundControl-x86_64.AppImage` | https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl-x86_64.AppImage |
| Android | `QGroundControl.apk` | https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl.apk |

- 全部版本列表：https://github.com/mavlink/qgroundcontrol/releases
- v5.1 稳定版文档：https://docs.qgroundcontrol.com/Stable_V5.1/en/
- 系统要求：Windows 10 (1809+) / Windows 11；建议 8 GB RAM、SSD、i5 及以上、NVIDIA/AMD 独显
- 安装后会生成 3 个快捷方式：`QGroundControl`、`GPU Compatibility Mode`、`GPU Safe Mode`，**默认用第一个**，花屏/黑屏再试后两个
- 只有跑 PX4 `main` 分支时才需要 Daily Build：https://docs.px4.io/main/en/dev_setup/qgc_daily_build.html
- 注意：官方文档里的下载 CDN 域名 `d176tv9ibo4jno.cloudfront.net` 在部分国内网络下**无法解析**，改用上面的 GitHub 直链

---

## 6. 社区与求助

| 渠道 | 地址 |
|---|---|
| 官方论坛（Discourse） | https://discuss.px4.io |
| GitHub Issues | https://github.com/PX4/PX4-Autopilot/issues |
| 官方 Discord / Slack | 见 https://docs.px4.io/main/en/contribute/ 页面入口 |
| 贡献指南 | https://docs.px4.io/main/en/contribute/ |

---

## 7. 建议阅读顺序（针对本 D 题）

1. 系统架构 + uORB（`concept/px4_systems_architecture`、`middleware/uorb`）——建立全局心智模型。
2. 室内定位三选一（VIO / 动捕 / 光流）+ 外部位置估计——解决"没有 GPS 怎么定位"。
3. ROS 2 Offboard 控制 + uXRCE-DDS——解决"代码怎么指挥飞机飞航线"。
4. 避障与路径规划接口——解决"货架间怎么不撞"。
5. SITL + Gazebo——先把航线逻辑在仿真里跑通。
6. ULog / Flight Review——上真机后定位问题。

---

## 8. PX4 固件下载（预编译二进制）

**关键前提：PX4 固件是按飞控板（board target）分别编译的，不存在"一个通用固件"。**
必须先确定手上的飞控型号，再下对应 target 的 `.px4` 文件。

### 8.1 官方发布地址（实测可用）

官方预编译固件托管在 AWS S3 上，QGC 刷固件时也是从这里拉：

```
https://px4-travis.s3.amazonaws.com/Firmware/<版本>/<板子target>_default.px4
```

`<版本>` 可用值（实测枚举）：

| 别名 | 含义 | 实测构建时间 |
|---|---|---|
| `stable` | 当前稳定版（= v1.17.0） | 2026-05-13 |
| `beta` | 当前 beta/RC（= v1.18.0-rc1，45 个 target） | 2026-09 |
| `main` | **已废弃**，停留在 2024-10-04 | 2024-10-04 |
| `master` | 旧的持续构建别名 | 2026-08-18 |
| `v1.17.0` / `v1.16.2` / … | 指定精确版本 | 对应发布日期 |

可用精确版本：v1.4.4 → v1.18.0-rc1（含各 beta/rc），共 60+ 个版本目录。
`stable` 目录下共 **250 个 `.px4` 文件**，其中 **111 个**是 `_default` 飞行固件。

每个 target 还配套一个同名目录，内含 `parameters.json`、`airframes.xml`、
`actuators.json`、`all_events.json.xz`（QGC 用来填充参数与机型列表）；
同目录下另有 `.elf`（带调试符号）和 `_bootloader.px4`（引导程序）。

### 8.2 常用飞控板 target 对照（v1.17.0 实测全部存在）

| 飞控产品 | target（文件名前缀） | 固件大小 |
|---|---|---|
| Pixhawk 6X / CUAV 同源 | `px4_fmu-v6x` | 1.82 MB |
| Pixhawk 6C | `px4_fmu-v6c` | 1.80 MB |
| Pixhawk 6U | `px4_fmu-v6u` | 1.78 MB |
| Pixhawk 4 / FMUv5 | `px4_fmu-v5` | 1.88 MB |
| Pixhawk 5X | `px4_fmu-v5x` | 1.90 MB |
| Pixhawk 4 Mini / FMUv4 | `px4_fmu-v4` | 1.82 MB |
| mRo Pixhawk / FMUv3 | `px4_fmu-v3` | 1.87 MB |
| Holybro Kakute H7 | `holybro_kakuteh7` | 1.67 MB |
| Holybro Durandal | `holybro_durandal-v1` | 1.81 MB |
| Holybro Pix32 v5 | `holybro_pix32v5` | 1.90 MB |
| CUAV X7 Pro | `cuav_x7pro` | 1.80 MB |
| CUAV Nora | `cuav_nora` | 1.78 MB |
| CUAV FMU v6X | `cuav_fmu-v6x` | 1.76 MB |
| Matek H743 | `matek_h743` | 1.65 MB |
| mRo Control Zero H7 | `mro_ctrl-zero-h7` | 1.80 MB |
| CubePilot Cube Orange | `cubepilot_cubeorange` | 1.81 MB |
| ARK FMU v6X | `ark_fmu-v6x` | 1.82 MB |
| SIYI N7 | `siyi_n7` | 1.78 MB |

示例直链（Pixhawk 6X，v1.17.0）：
```
https://px4-travis.s3.amazonaws.com/Firmware/v1.17.0/px4_fmu-v6x_default.px4
```

列出全部 target：
```
https://px4-travis.s3.amazonaws.com/?list-type=2&prefix=Firmware/stable/&max-keys=1000
```

### 8.3 刷写方式

| 方式 | 适用 | 说明 |
|---|---|---|
| **QGC 自动刷写**（首选） | 所有官方支持板 | 设置 → 固件 → 选 PX4 → 自动识别板型并下载对应固件，全程无需手找文件 |
| QGC 自定义固件 | 自编译 / 特殊 target | 设置 → 固件 → 自定义固件文件 → 选本地 `.px4` |
| 引导程序恢复 | 板子变砖 / bootloader 丢失 | 用 `_bootloader.px4`，QGC 的"重新安装引导程序"流程 |

> 注意：本机 DNS 无法解析 `firmware.px4.io`，QGC 用的
> `px4-travis.s3.amazonaws.com` 可正常解析，国内网络下以 S3 地址为准。
