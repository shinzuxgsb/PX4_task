# PX4 无人机上位机项目 —— 交接说明（Codex 新会话请先读这里）

> 更新：2026-10-12　｜　配套详细档案：**[`SSH_ACCESS.md`](SSH_ACCESS.md)**（含口令、雷达参数、踩坑记录）
> 安全提醒：`SSH_ACCESS.md` 含明文口令，**不要提交 Git、不要外发**。
>
> ⚠️ **2026-10-17 状态变更（最重要，先看这条）**：147 上**开机自启已换成新链 `mavlink-ws-chain.service`**
> → `~/mavlink_ws/start_mavlink_ws.sh` → **队友那套**：bluesea2（串口雷达 `/dev/ttyS4`）+ Cartographer（`amphi.lua`）
> + `mavlink_bridge`（**MAVLink ODOMETRY + DISTANCE_SENSOR**，不是 DDS；定高 `/dev/ttyS3`，飞控 `/dev/ttyS6` @115200）。
> 飞控侧配套：`UXRCE_DDS_CFG=0`、`SER_TEL3_BAUD=115200`、`MAV_1_CONFIG=103`、
> `EKF2_HGT_REF=2`、`EKF2_RNG_CTRL=2`、`EKF2_RNG_POS_Z=0.044`。
> ⇒ 本文档里凡说「当前跑的是 xx 那一套」的地方，与本节冲突的**一律以本节为准**。
> 旧的两条链都还在、只是 **disabled**：`px4-ev-chain`（10-09 的 DDS 链，`~/px4_ws`）、`px4-p2p-chain`（参考件 p_to_p，`~/px4_ws.p2p-20261016`）。
> 切换/回滚、以及 10-16 那次「回退到 10-09 版」「飞控空中重启复盘」，见 `CHANGELOG.md` 的同名小节和 `~/mavlink_ws/ADAPT_FOR_147.md`。
>
> **2026-10-24 D 题移植**：`mavlink_ws` 新增 `shelf_scan_pkg`、`pid_control_pkg`、
> `shelf_px4_bridge`，匿名 STM32 串口层换为 PX4 MAVLink/Offboard；入口为
> `~/mavlink_ws/shelf_preflight.sh` → `~/mavlink_ws/shelf_takeoff.sh`。
> 已在板上编译通过、自测 23/23 + 适配器测试 5/5；未解锁未实飞。
> 详情、备份哈希和实飞硬门禁见 `CHANGELOG.md` 2026-10-24 一节。
> 网页地面站位于 `mavlink_ws/groundstation_px4/`，支持相机实时画面和
> `WAIT_GO` 后一键起飞；实机链路已验证到预检就绪，未点击起飞。
>
> **2026-10-09 变更**：雷达由网口款 **`LDS-50C-E`** 换成串口款 **`LDS-50C-R`**
> （香橙派 UART4 → `/dev/ttyS4`，需 u-boot overlay `rk3588-uart4-m2.dtbo`）。
> 详见 `CHANGELOG.md` 的 2026-10-09 一节与 [`px4_ws/docs/串口雷达改造.md`](px4_ws/docs/串口雷达改造.md)。

## 1. 项目目标

给一台 **PX4 无人机**做上位机（机载电脑）软件：

- 上位机：**Orange Pi 5 Max**（RK3588，8G，232G NVMe）
- 飞控：**PX4 1.15.4**（本地源码在 `D:\px4无人机\PX4-Autopilot-1.15.4`）
- 系统：**Ubuntu 22.04 + ROS 2 Humble**（用户已明确选 22.04）
- 传感器：**蓝海光电 LDS-50C-E-R 网口雷达**（2D 360°）+ 摄像头
- 飞控通信：PX4 走 **uXRCE-DDS**（PX4 1.15.4 内置 `uxrce_dds_client`，
  `dds_topics.yaml` 里 `OffboardControlMode / TrajectorySetpoint / VehicleCommand /
  VehicleOdometry / VehicleLocalPosition / VehicleStatus` 都在）

## 2. 硬件与网络（已实测确认）

| 项 | 值 |
| --- | --- |
| 香橙派 IP（WiFi） | `10.23.87.147`，用户/口令 `orangepi` / `orangepi` |
| 有线网口 | `enP3p49s0`，**Realtek RTL8125 2.5GbE（PCIe）**，驱动 `r8125 9.009.00` |
| 有线网口地址 | 静态 **`192.168.158.100/24`**（NetworkManager 连接 `Wired connection 1`） |
| 雷达（现用，2026-10-09 起） | **`LDS-50C-R`（串口款）**：香橙派 **UART4 → `/dev/ttyS4`**，921600 8N1；40pin 排针 19=TX / 23=RX（pinmux `UART4_M2`） |
| 雷达（旧，备用） | `LDS-50C-E`（网口款），IP `192.168.158.98`，命令端口 6543，本机监听 6668 |
| SSH | 只能用**口令**登录；本机没有 sshpass/plink，需开伪终端喂密码 |

> 注：有线网口 `enP3p49s0` **当前 DOWN / NO-CARRIER（没插网线）** —— 雷达改串口后网口不再需要，
> `192.168.158.100` 那套静态配置暂时用不到；要切回网口款雷达时再把网线接上、确认该网卡起来。

> **不要**用 `D:\24电赛d题\.codex_ssh\id_ed25519` —— 那把钥匙没授权在这台板子上。

## 2.1 整机（飞机侧）现状 —— 2026-10-07 首飞前排查

| 项 | 值 | 证据 |
| --- | --- | --- |
| 飞控 | **MicoAir743 v2**（STM32H743） | Windows 设备安装记录：USB `VID_3162:004B`(bootloader) → `VID_1B8C:0036`(App)，2026-10-06 13:39 插机，与 PX4 板级 ID 表吻合 |
| 飞控固件 | **PX4 1.15.4**，构建目标 **`micoair_h743-v2_default`** | QGC 元数据缓存 `%LOCALAPPDATA%\QGroundControl\QGroundControl Daily\cache\QGCCompInfoCache\421e15a6_00_0.cache`（含 `px4-travis.s3.amazonaws.com/Firmware/v1.15.4/micoair_h743-v2_default/...` 回退 URL） |
| 地面站 | QGC 已装：`D:\QGroundControl\QGroundControl.exe`；已连过飞控（参数缓存 970 项） | `%APPDATA%\QGroundControl\QGroundControl Daily\ParamCache\1_1.v2` |
| 本地 PX4 源码 | `PX4-Autopilot-1.15.4` **未编译**（无 `build/`，本机无交叉工具链）→ 刷固件/调参数都走 QGC | 目录扫描 |
| 机架 / 遥控器 / 接收机 / 电池 | ❓ **本地没有任何记录，待用户确认** | — |

> 注意：`px4_ws` 里的上位机包（雷达 / SLAM / `px4_bridge` / `px4_takeoff_land` /
> `px4_ev_bridge` / `tofsense_m`）**遥控器首飞时一律用不到**。首飞时香橙派建议断电，
> 或至少不运行 `px4_bridge`、`px4_takeoff_land`、`px4_ev_bridge` ——
> 它们会发 `/fmu/in/offboard_control_mode`，和遥控器抢控制权。
> 另：本文 §5.1 里那张包的清单已过期（现在是 9 个包，见下表）。

## 3. 已确认的关键结论（别再重复踩）

1. **（网口款 `LDS-50C-E` 适用，历史经验）雷达的数据回发机制**：雷达不按 EEPROM 里的 `srv_ip/srv_port` 单播，
   而是**回发给「最后一个给它发命令的 IP + 源端口」**；没人跟它说话时发 `0.0.0.0`
   （等于没人收得到）。所以必须先绑定 `local_port(6668)`，**再从同一个 socket**
   发指令（厂家用 `LGCPSH` / cmd `0x0043`，每秒保活），之后才会持续收到数据。
   → 只监听不说话，一个包都收不到。
2. **板子真正的网口是 PCIe 上的 RTL8125**，不是 RK3588 的 GMAC
   （`ethernet@fe1b0000` 在设备树里是 `disabled` 且本来就没接网口）。
   曾经花三轮改 GMAC 的设备树，方向是错的，已复原。
3. **网口枚举可能和冷启动有关**：早期热重启时 PCIe 链路上不来、看不到这张网卡，
   断电冷启动后才出现。**待验证：热重启后网口是否还在**（关系到开机自启可靠性）。
4. 雷达走网口后，**PX4 链路建议走串口**（TELEM2 → 香橙派 `/dev/ttyS*`），
   否则一块网口要同时接雷达和飞控就得加交换机。
5. **外部定位（EV/SLAM）必须有"防冻结"守护**（2026-10-08 血的教训）：
   上游节点挂掉后，"话题仍在 30Hz 发布"**不等于**数据有效——
   `tf_to_pose` 会拿"最新可用 TF"无限重发最后一帧旧变换，飞控会以为飞机静止而自由飞走。
   所以守护必须看**消息自带时间戳是否推进**，不能只看"有没有新消息到达"。
   同一个坑在 EV 桥里也有过一次（原实现只看到达时间）。
6. **香橙派开机时间必须是对的**（2026-10-08 定点右飘事故的真根因）：
   该板 RTC 曾在开机时停留在上次关机时刻（实测慢了 18h41m），NTP 校正时产生**时间跳变**，
   cartographer 遇到大时间跳变会抛 `std::length_error` 崩掉。
   → `run.sh fly` 的 `[0/7]` 步骤会先等 NTP 同步再拉起 SLAM（最多 60 s）。
   若再出现开机时间停在旧时刻，检查板载 hym8563 RTC 的纽扣电池。
7. **排查进程别用 `pgrep -x`**：Linux `comm` 截断到 15 字符，
   `pgrep -x cartographer_node` / `-x px4_ev_bridge_node` 永远匹配不上（会误判"节点没跑"），
   要用 `pgrep -af` 匹配完整命令行。
8. **串口雷达 `LDS-50C-R` 走 UART4（2026-10-09 起）**：设备树里 `serial4=serial@feb70000` 默认
   `disabled`，要 u-boot 叠 `rk3588-uart4-m2.dtbo` 才有 `/dev/ttyS4`
   （`sudo bash ~/px4_ws/tools/enable_ttyS4.sh --reboot`）。雷达参数用
   `params/uart_lidar.yaml`（`dev_id: -1`、`reversed: false`、`max_dist: 8.0`）。
   话题/坐标系不变（`/scan`、`laser_link`），`carto_pkg / slam_pkg / px4_ev_bridge` **不用改**。
   与网口款的关键差异：`reversed` 相反、串口款开角度重采样、`/scan` 是 1200 点（网口款 2000 点）。
9. **新开的终端会话可能看不到已有 ROS 话题（2026-10-09 发现，未定性）**：`ros2 topic list`
   只列出自己的 `/parameter_events`、`/rosout`，`ros2 node list` 空。链路本身是好的
   （cartographer / 定高器 / EV 桥日志都在推进），坏的只是「新参与者 ↔ 开机时那批参与者」的发现。
   已排除 ROS_DOMAIN_ID、`ROS_LOCALHOST_ONLY`、网络命名空间、daemon 缓存。可疑点是开机时
   `enP3p49s0` 还 UP、之后变成 DOWN。
   **2026-10-09 18:00 更新：重启 `px4-ev-chain` 之后，新会话的 `ros2 topic list` / `ros2 node list`
   完全正常**（能列出全部话题和节点）→ 那是**那次开机状态下的瞬态**，不是代码问题。
   结论：改了机载配置后**重启一次链路**，ROS 2 的发现就正常。
10. **cartographer 开 `use_imu_data` 就"没有 IMU 就不干活"（2026-10-09）**：开了之后它的多队列
   会一直刷 `Queue waiting for data: (0, imu)`，而且**一个位姿都不输出**（飞控没上电时实测
   `/tracked_pose` 完全没有数据）。所以配置拆成两份：`fly.lua`（带 IMU）/ `fly_no_imu.lua`
   （`include "fly.lua"` 后把开关关掉），`run.sh fly` 会先探 `/imu` 有没有数据再二选一。
   反过来也要记住：**飞行中 IMU 流断了，cartographer 会停出位姿** → EV 超时 → 飞控进入位置失效保护。
11. **判断"EV 到底有没有送进飞控"用订阅数，别看进程**（2026-10-09 晚）：
    `ros2 topic info /fmu/in/vehicle_visual_odometry` 的 **`Subscription count`** ——
    `0` = uXRCE-DDS Agent 没在桥接（报文根本没出香橙派，飞控什么都收不到）；
    `>=1` = 已经在往飞控送。这条比"进程都在跑"有意义得多，`tools/check_ev.sh` 第 5 步就是它。
    另外：EV 现在**有**上报"位置差分速度"（`report_velocity` 默认 true，`velocity_sigma=1.0` 保守），
    想关掉就 `report_velocity:=false` 或飞控侧 `EKF2_EV_CTRL` 去掉 bit2。
12. **`grep -q` + `set -o pipefail` 是个坑（2026-10-09 晚踩到）**：`cmd | grep -qm1 关键词`
    在 `pipefail` 下**永远判为失败**（grep 提前退出 → cmd 收 SIGPIPE 非零 → 管道非零）。
    探针类写法要用 `out="$(cmd | grep -m1 关键词 || true)"; [ -n "$out" ]`。
13. **ROS 2 参数类型不能"看起来像就行"（2026-10-09 晚踩到）**：命令行/launch 传
    `turn_deg:=180` 会被 YAML 解析成 **int**，而节点声明的是 `double` →
    `rclcpp::exceptions::InvalidParameterTypeException`，**节点直接崩**。同类：`hover_time:=5`。
    写 launch 文件时数值参数一律用
    `ParameterValue(LaunchConfiguration('x'), value_type=float)`，bool 用 `value_type=bool`。
14. **Offboard 任务航线已经能跑**（2026-10-09 晚，`px4_takeoff_land` 扩展）：`mission_enable:=true`
    后走 `起飞 → 沿起飞机头方向前进 forward_dist → 掉头 turn_deg → 回起飞点正上方 → 悬停 → 降落`，
    用「位置 + 绝对航向」设定值；`/mission_state` 看阶段。用法与验证清单见
    `px4_takeoff_land/README.md` 的「小任务航线」一节和 `CHANGELOG.md` 同日最后一节。
15. **飞控串口链路只剩 ~17% 余量 —— 这是现在最脆弱的一环**（2026-10-09 晚炸机根因）：
    `ttyS6` 接收方向实测 **74.7 KB/s / 921600 上限 ≈90 KB/s ≈ 83%**（PX4 按 `dds_topics.yaml`
    全速发 `sensor_combined` 100Hz、`vehicle_local_position` 95Hz…），驱动计数已有 `fe/brk/oe`。
    只要链路一忙到会话超时重建，**PX4 会在 `COM_OF_LOSS_T`(1s) 后直接掉出 Offboard** ——
    第一次任务试飞就是这样炸的（Agent 日志 `session re-established`，IMU 桥报 13.8 秒收不到数据）。
    → 飞行前先做：飞控 `SER_TEL3_BAUD` + Agent 都提到 **1500000**；线束远离动力线/双绞/缩短。
16. **两处"看起来正常其实在骗人"的地方**（2026-10-09 晚，已加固）：
    ① TOF 高度会偶发离谱读数（贴地读到 2.55m），而它**直接就是 EV 的 z** → 已加
    `height_jump_max`/`height_jump_confirm`（单帧跳变先丢、连续多帧才认账）；
    ② 任务节点原来不知道自己链路断了 → 已加 `link_timeout`(0.5s) 看门狗，ABORT 时还会打印
    `armed/nav_state/inOffboard/vehicle_status 未更新时间`。**排查这类问题先看 `/tmp/px4_ws_*.log`
    的时间戳有没有在推进 + Agent 日志有没有 `session re-established`。**
17. **起飞前一律先跑 `bash ~/px4_ws/tools/preflight.sh`**（2026-10-09 晚新增，只读）：
    它会依次查：链路进程 / `/scan`·`/tracked_pose`·`/tofsense/height`·EV 的频率 /
    **EV 有没有送进飞控（Subscription count）** / 飞控四个 valid + `dead_reckoning` + `eph/evh` /
    **EKF 是否真在融合 EV（`cs_ev_*`）** / 串口占用 / 有没有 `px4_bridge` 抢 offboard。
    任何 FAIL 都别飞。第 6 段要串口计数，先 `sudo -v` 或单独 `set_dds_baud.sh status`。
18. **新会话"看得见话题、收不到数据"是 FastDDS 同机 SHM 的坑**（2026-10-09 晚）：
    `ros2 topic list` 有、`Subscription count>=1`，但 `ros2 topic hz/echo` 一个数据都没有。
    → `export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml`（只用 UDP）就好，
    实测 `/scan` 96 帧/8 秒。`preflight.sh` 已自动带上这个变量。
19. **同一时间只能有一条链路**（2026-10-09 晚）：重复跑 `bash run.sh fly` 会和已在跑的那条互相
    `pkill`，留下"进程都在但没数据"的半死状态。`run.sh fly` 现在会检测并拒绝；
    正确做法是 `bash run.sh stop` 后重起，或 `sudo systemctl restart px4-ev-chain`。
20. **DDS 波特率已双边提到 1500000**（2026-10-09 晚，占用 83% → **50%**）：
    `/etc/default/microxrce-agent` 里 `XRCE_BAUD=1500000`，飞控侧 `SER_TEL3_BAUD=1500000`。
    ⚠️ **改波特率必须两边同时改，只改一边 = 链路全断**（现象：`0 KB/s`、
    `fe/brk` 帧错暴涨，因为收到的是乱码）。用 `tools/set_dds_baud.sh <baud> [--restart]` 改香橙派侧，
    它会同时打印 QGC 侧的步骤；`status` 子命令会实测流量并诊断 0 KB/s 的原因。
    当前链路 30 秒内 `fe/brk` 零增长，干净。
21. **阈值/上限不能写死**（2026-10-09 晚自己踩的）：`preflight.sh` 第 6 段曾把串口上限
    写死成"921600 的 90 KB/s"，波特率提到 1500000 后就报出假 FAIL（73 KB/s 被算成 82%）。
    已改成从 Agent 命令行读实际 `--dev`/`-b` 再算。**凡是"链路参数会变"的地方都要动态取值。**
    （另：`sudo bash tools/preflight.sh` 可整份跑，脚本内部强制 UDP 传输，root 下 ROS 检查正常。）
22. **看门狗一定要盯"高频话题"**（2026-10-09 晚自己踩的）：`px4_takeoff_land` 的链路看门狗
       一开始盯 `/fmu/out/vehicle_status`，而它**只有 1.3 Hz**（间隔 0.77s），配 0.5s 超时 →
       飞行中频繁误判"链路中断"，两次任务都在**返航段中途被自己掐断**（表现为"只返回了 60cm"）。
       现已改成盯 `/fmu/out/vehicle_local_position`（**95 Hz**）。
       真要怀疑链路断，先看 `/tmp/microxrce_agent.log` 有没有 `session re-established`，
       以及中止那一刻的 `inOffboard / nav_state` —— 这次两者都说明链路其实是好的。
23. **起飞时向右偏 0.15~0.35 m 是实测存在的**（2026-10-09 晚，两次运行一致；航向 86~89° 时
       "右" ≈ -x）。同一次里还发现**飞控的 EKF 位置与 SLAM 位置差 0.4m 量级**（任务节点读 EKF 说
       "前进到位"，SLAM 说几乎没动）→ 位置估计在起飞瞬间被带偏，EKF 和 SLAM 同时发生。
       可选动作：IMU 路径台架试验（`SLAM_USE_IMU=1`）、标定 `EKF2_EV_DELAY`、起飞更慢更缓。
24. **飞控的 `dist_bottom` 不是定高器直读**：`vehicle_local_position.dist_bottom_sensor_bitfield=0
       (NONE)` 时它是 **EKF 的地形估计**（实测与定高器差 0.3m：EKF 说 0.68m、定高器说 0.99m）。
       所以"起飞到位"这类高度判据不能只看 `dist_bottom`。
25. **雷达"开机自启不成功"的真相与加固**（2026-10-10）：驱动偶发在打开串口时报它自己的
       `-106 Failure to modify serial port baud rate` 然后卡死（不握手、`/scan` 无数据、
       驱动进程连 `ttyS4` 句柄都没有），**但雷达本体是好的** —— 手动裸读能收到完整帧
       （`cf fa 3c 00 ...`）。`run.sh start_lidar()` 已加固：起驱动前 `stty` 预置端口 +
       起完校验 `/scan` 没数据就重试一次。**判"雷达死没死"的正确姿势**：
       `sudo timeout 6 cat /dev/ttyS4 | wc -c` → 正常应有几十万字节（连续流）；
       一句话恢复：`bash ~/px4_ws/run.sh lidar`。
26. **看板上 `tools/preflight.sh` 时先确认是哪一版**（2026-10-10）：队友 10:24 改过一版，
       它没设 `FASTRTPS_DEFAULT_PROFILES_FILE` → 会误报 `/scan`、`/tracked_pose`、
       `/tofsense/height` "无数据"（FastDDS 同机 SHM 的坑）。跑之前 export 一下就对了：
       `export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml`。
27. **`/scan` 没有频率 ≠ 雷达坏了，但也可能是真坏**（2026-10-10 一天里两种都遇到）：
        · **假报警**：新会话没设 `FASTRTPS_DEFAULT_PROFILES_FILE` → FastDDS 同机 SHM 坑，
          大消息（`/scan`）尤其明显；加 profile 立刻能读到。
        · **真故障**：驱动活着、握着 `ttyS4` 句柄、握手也过了，**就是不发布 `/scan`**。
          判据：`ros2 topic info /scan` 的 **Publisher count**（0 = 真没人发），
          以及 cartographer 日志里的 `scan rate:` 行是否还在推进（地面真值）。
        → 已上线**雷达看门狗**：`tools/lidar_watchdog.sh` + `px4-lidar-watch.service`（开机自启），
          每 20s 查一次、连续 3 次没数据就重启驱动，**解锁时只报警不动手**。
28. **飞控电池参数没配 → PX4 低电 failsafe 形同虚设**（2026-10-10，队友 `tools/px4_batt_watch.py` 发现）：
        `battery_status` 里 `capacity=0 / nominal_voltage=0 / time_remaining_s=nan`，
        PX4 的低电告警基于 `remaining` 比例 → 恒为 1.0，永不触发。**要么在 QGC 里把电池参数
        （容量/电芯数）配好，要么用 `px4_batt_watch.py` 按电压兜底。**
29. **`run.sh` 已重构成"事件驱动 + 并行启动"**（2026-10-10 晚）：不再用固定 sleep 猜"起好没"，
        改轮询就绪条件；Agent/IMU 桥/定高器/雷达/SLAM/tf_to_pose/EV 桥**一起起**。
        `systemctl restart px4-ev-chain` 实测 **28~34 秒**（原来 50~101 秒）。
        单组件恢复：`bash ~/px4_ws/run.sh restart lidar|slam|tf|tof|ev|imu|agent`。
        **新写检查/启动逻辑时的三条铁律**：
        ① 时限用**墙钟**（`$SECONDS`）不能用循环次数（判据本身可能要 4~5 秒）；
        ② 就绪判据不能依赖"还没启动的组件"（曾让 SLAM 等 tf_to_pose 发布的 `/tracked_pose`）；
        ③ 别用裸 `wait`（本脚本已用 `nohup ... &` 起了整条链，裸 wait 会等它们全部退出）。
30. **`ros2 launch` 包装进程 + `respawn=True` 是个陷阱**（2026-10-10 晚）：
        `pkill -f cartographer` 只杀节点，**launch 包装还活着** → 它按 `respawn=True`
        把节点再拉起来 → **两份实例**打架（现象：TF 抖动、`Requested submap … does not exist`、
        `/tracked_pose` 断流）。停/重启带 launch 的组件时，**必须连 `ros2 launch <pkg>` 一起杀**。
31. **链路看门狗 `px4-chain-watch`（2026-10-10 晚新增，开机自启）**：每 15 秒查
        雷达/SLAM/EV 桥/定高器/IMU 桥/Agent 六段，**连续 2 轮不健康**才动手，
        且**只在能确认未解锁（`arming_state=1`）时才重启**（读不到就保守不动手；
        "IMU 桥也收不到飞控数据"说明飞控没通信，可豁免）。
        **健康判据全部基于"日志时间戳新鲜度"**（不看 `ros2 topic` —— DDS 发现层会骗人）：
        SLAM=日志 60 秒内有新行；EV 桥=最近 40 秒内出现 `EV: NED pos`；定高器=40 秒内有 `帧` 行；
        雷达=进程在 + 日志有握手标记 +（ `/scan` 有频率 或 SLAM 日志新鲜 ）。
        还要查 `cartographer_node` **实例数必须为 1**（`pgrep -f '^/opt/.../cartograph[e]r_node'` 锚定路径，
        否则会把"正在看日志的人自己那条命令"也数进去）。
32. **"上电后要等一分多钟才收到 EV 数据"的真根因 = 链路被"等 NTP"卡在起跑线**（2026-10-14）：
        用户的现象描述是"雷达传输数据很慢"，**但雷达一点不慢**（驱动 0.46 秒握手、`/scan` 稳定
        15 Hz、EV 30 Hz）。真凶是 `px4-p2p-chain.service` 里的 `After=/Wants=time-sync.target`：
        `systemd-time-wait-sync` 要等 `systemd-timesyncd` 报"内核时钟已同步"，而 NTP 源是编译默认的
        `ntp.ubuntu.com`（journal 里 17 分钟后还在 `Timed out waiting for reply`）。
        **证据（单调时钟，同一天四次上电）**：链路启动只比 "Finished Wait Until Kernel Time
        Synchronized" 晚 0.02~0.03 秒，而那次等待分别是 **35.3 s / 35.6 s / 172.9 s / 283.1 s**；
        单次 boot 内"上电 → EV"≈ 46 s，其中 35 s 干等 NTP、链路自身约 10 s。
        **已修**：单元去掉该硬依赖（`run_p2p.sh` 改成有界等待 `NTP_WAIT_MAX`，默认 15 s、设 0 关掉）
        + NTP 换源（drop-in `px4_ws/systemd/timesyncd.conf.d/10-ntp-cn.conf`，实测重启后 0.07 秒同步）。
        改完实测：**服务起跑 → EV ≈ 8 秒**，`list-dependencies --after` 里 `time-sync` 计数 = 0。
        **两个连带发现（未改）**：① `systemctl restart px4-p2p-chain` 的**停机阶段会卡满 60 秒**
        （`State 'stop-sigterm' timed out. Killing.`，python 节点不理 SIGTERM；建议
        `KillSignal=SIGINT` + `KillMode=mixed` + `TimeoutStopSec=15`）；② `stop_all` 的 `pkill -f`
        模式很宽 —— **远程命令里只要出现 `cartographer_odometry_bridge` 之类字面量，重启链路就会
        把你自己那条 SSH 会话一起杀掉**（"sshd 莫名失联"多半就是它）。
33. **启动形态只有一套**（2026-10-14，用户要求"不要搞那么多种模式，飞行逻辑也不该有变化"）：
        `run_p2p.sh` 原来 `start/bench/pos` 三种形态，差别**只在"任务状态机 + Offboard 桥起不起"**，
        感知链完全一样。已合并成**只留感知链那套**（原 `pos`）：`bash run_p2p.sh`（默认）起
        agent+雷达+cartographer+定高+EV 桥，`start_mission:=false start_offboard:=false`
        ⇒ **一条 /fmu/in/offboard_* 都不会发**。理由：手动飞时"飞不飞/往哪飞"必须只由遥控器 +
        PX4 位置环决定；`start` 里多出来的 `p_to_p_mission_node` + `px4_offboard_bridge` 是另一套
        飞行逻辑（切到 Offboard 就自动接管，launch 里 `allow_land_command:=True` 还能自己落）。
        `pos`/`bench` 保留为等价别名（会打"已废弃"提示）。`tools/preflight.sh` 不再按模式分支：
        任务/Offboard 未运行一律按"预期如此"报 `[!!]`，真在跑才报 NG。板端 drop-in
        `10-mode.conf`（`P2P_MODE=pos`）已 mv 到 `~/backups/20261014_2158_mode_collapse/`。
        ⚠️ **代价**：没有 Offboard 桥 ⇒ **飞控不能停在 OFFBOARD 模式**（`pre_flight_checks_pass`
        会一直 false、解不了锁）——起飞前用遥控器拨到**定点(Position)** 再解锁。
        以后真要跑 A→B：先 `bash run_p2p.sh stop`，再手敲
        `ros2 launch p_to_p_px4 final_position.launch.py`（红线：EV 只能有一个发布者）。
34. **黑匣子（`record_flight.sh`）曾因"目录名撞车"整架次没录像**（2026-10-15 修）：目录名是
        `date +%Y%m%d_%H%M%S`，而 **RTC 停走 + timesyncd 把多次开机恢复到几乎同一时刻** ⇒ 两次开机同名
        ⇒ `ros2 bag record -o` 报 "Output folder ... already exists" 并 exit=1（今天几次飞行因此零数据）。
        现在名字 = 墙钟 + 本次开机 `boot_id` 前 8 位 + `_a<n>`，撞名自动加后缀，开录即失败自动重试，
        跑起来再崩自动重起一份。**判据**：`~/flights/<stamp>_<boot_id>_a<n>/`。
        **另一条同样致命的**：journald 只保留最近 8 次开机 —— 一天多的测试记录会被冲掉（今天 13:00 前后
        那三次飞行的日志就是这么没的）。重要飞行后立刻把 `~/flights` 与 `journalctl -b -1` 拉回本机，
        或把 `/etc/systemd/journald.conf` 的保留量调大。

## 4. 参考工程（很有价值的模板）

`D:\24电赛d题\drone_d_ws` —— 2024 电赛 D 题那套 **ROS 2 Humble 工作区**，
同样是 Orange Pi 5 Max，里面有现成可复用的：

- `src/bluesea2/src/bluesea-ros2/` —— 厂家 ROS 2 驱动，**已支持 UDP**，
  `params/udp_lidar.yaml` 就是网口配置（改 IP/端口即可）
- `my_carto_pkg/`（cartographer ROS2 + `fly.lua` + urdf）、`pid_control_pkg/`、
  `my_launch/`（参数体系）、`autostart/`（systemd 自启）、`tools/`（校准/自检脚本）
- 注意它的 `uart_to_stm32` 是给自研 STM32 飞控用的**自定义串口协议**，
  本项目换 PX4 后要替换成 px4_msgs 桥接

另：`D:\px4无人机\ros1_basic_ws` 是更早的 ROS 1 遗留工程（串口雷达 LDS-50C-R），
只作参考，不要再基于它开发。

## 5. 本机（Windows）工具

- `D:\px4无人机\tools\lidar_probe.cpp` —— 雷达 IP/端口探测工具，
  单文件、不依赖 ROS，`g++ -O2 -std=c++17 -o lidar_probe lidar_probe.cpp`
  （已在香橙派上编译通过；功能：sniff / query / bcast / sweep）
- 本机**没有** C++ 编译器，语法验证要在香橙派上做

## 5.1 工作区 `px4_ws`（本机与板子上各一份）

| 位置 | 路径 |
| --- | --- |
| 本机（编辑用） | `D:\px4无人机\px4_ws` |
| 香橙派（运行用） | `~/px4_ws` |

含 9 个包（`src/`）：

| 包 | 用途 | 状态 |
| --- | --- | --- |
| `bluesea2` | 雷达驱动（`base_lidar` + `bluesea-ros2`），**串口 `/dev/ttyS4`**（2026-10-09 前是 UDP 网口） | ✅ `/scan` 15.2Hz / 1200 点 |
| `carto_pkg` | cartographer 2D 建图（`fly.lua` + urdf） | ✅ 已适配（雷达由 run.sh 单独起） |
| `slam_pkg` | slam_toolbox 2D 建图（另一套方案） | ✅ 已适配 |
| `px4_msgs` | PX4 uORB 消息，**从 PX4 1.15.4 源码生成**（213 msg + 1 srv） | ✅ 编译通过 |
| `px4_bridge` | 上位机 ↔ PX4 Offboard 桥接 | ✅ 编译通过 + 转换已验证 |
| `px4_takeoff_land` | Offboard 起飞/悬停/降落状态机 | ✅ 编译通过；⚠️ 不能和 `px4_bridge` 同跑 |
| `px4_ev_bridge` | 雷达 SLAM 位姿 + TOFsense 高度 → 合成 `vehicle_visual_odometry` 喂 EKF2 | ✅ 编译通过 |
| `tofsense_m` | TOFsense-M 定高器驱动 | ✅ 编译通过 |
| `serial_comm` | boost 串口库 | 备用 |

> **2026-10-12 新增两个包**：p_to_p_mission（A→B 点对点任务状态机）、p_to_p_px4（PX4 uXRCE-DDS Offboard 桥，只走 DDS）。来源、移植改动、验证记录见 CHANGELOG.md 2026-10-12「补记 3」与本文 §11。

顶层脚本：`build.sh` / `run.sh` / `env.sh` / `stop.sh` / `README.md` / **`agent.sh`**（新增）。
`run.sh` 支持 `init|lidar|carto|slam|agent|px4|fly|status|stop`；雷达参数由 `LIDAR_PARAMS` 决定，
**默认已指向 `uart_lidar.yaml`**（串口款），可 `LIDAR_PARAMS=.../udp_lidar.yaml bash run.sh lidar` 临时切回网口款。
另新增 `docs/串口雷达改造.md`、`tools/enable_ttyS4.sh`（开 UART4）、`tools/check_ev.sh`（整链路体检，只读）、
`tools/fc_link_probe.sh`（不依赖 DDS、裸探飞控那路串口有没有信号）、
`px4_imu_bridge.py`（板上 `~/px4_imu_bridge.py`：飞控 IMU → `/imu`，给 cartographer 做旋转预测）。
`systemd/microxrce-agent.{service,default}` 是 uXRCE-DDS Agent 的自启单元
（已装到香橙派 `/etc/systemd/system/`，**还没 enable**，等飞控接线确认串口）。

**已实测**（2026-10-07）：

```
/scan                 15.065 Hz（一帧 2000 点，360°）
px4_bridge            ENU[10, 20]cm/s -> NED velocity[0.200, 0.100] m/s  ✓
全工作区编译           6 个包全部通过（px4_msgs 生成耗时最长，约 5 分钟）
```

**踩过的坑**：

1. `px4_bridge` 里 `now_us()` 不能声明成 `const` —— const 版 `get_clock()`
   返回 `const Clock`，而 `Clock::now()` 非 const，会编译失败。
2. 雷达驱动启动要 **约 20~25 秒**才出数据（先跟雷达握手，日志里会有几轮
   `udp_talk_GS_PACK: lidar connection abnormality` 重试，属正常），
   别用 8 秒就判定"没数据"。
3. 自带的 `udp_lidar.launch` 是 Python 内容但扩展名不对，已补了
   `udp_lidar.launch.py`；且原文件用的是 `LifecycleNode`（该节点不是
   生命周期节点），新的那份已改成 `Node`。

## 6. 当前进度

- [x] 香橙派换到 Ubuntu 22.04，SSH 可用
- [x] 有线网口打通（RTL8125），静态 IP 配好
- [x] 雷达握手成功，`/scan` 数据链路已验证（6 秒 3538 包）
- [x] **雷达换成串口款 `LDS-50C-R`（UART4 `/dev/ttyS4`）**：overlay 已启用、`/scan` 15.2Hz / 1200 点、
      `/tofsense/height` 55.6Hz、`/tracked_pose` 30.1Hz、`/fmu/in/vehicle_visual_odometry` 15.0Hz
      （2026-10-09 队友完成并实测；本会话登机复核一致）
- [x] **定点模式遥控垂直起降已可用**（2026-10-09 晚）：飞控已接上；EV 的
      `cs_ev_pos / cs_ev_yaw / cs_ev_hgt / cs_ev_vel` 全 true（无 reject），
      `xy_valid / v_xy_valid / z_valid / v_z_valid` 全 true，`heading` 与 EV yaw 一致。
      遗留：起飞瞬间会"往前冲一下"，机制待查（见 `CHANGELOG.md` 同日晚间一节的 4 条假设 + 录包方法）
- [~] **Offboard 小任务航线**（起飞→前进→掉头→回原位）：节点已就绪、拆桨 dry_run 通过；
      **2026-10-09 晚首次试飞炸机** —— 前进段本身成功（1.2m/2s），但飞行中飞控↔香橙派
      DDS 会话断开（串口 RX 已占 83%）→ PX4 掉出 Offboard → 飞机外飞 5.7m 后撞毁。
      复盘与加固见 `CHANGELOG.md` 同日最后一节；**下次飞行前必须先解决串口余量与 `COM_OBL_RC_ACT`**。
- [x] **安装 ROS 2 Humble**（2026-10-07 完成）
- [x] **建工作区 + `bluesea-ros2` 驱动跑通 `/scan`**（2026-10-07 完成）
- [x] **基础通用功能：carto 建图 / slam 建图 / PX4 通讯桥** 已适配并编译通过
- [~] 建 PX4 uXRCE-DDS 实际链路：**香橙派侧已就绪**（Agent 已装 + `agent.sh` 可启停 +
      systemd 单元已装未启用），**卡在飞控侧**：TELEM2 未接线、`UXRCE_DDS_CFG` 未配
      （实测 ttyS1/S3/S6 全 `tx:0 rx:0`）。详见 `CHANGELOG.md` 2026-10-07 修复记录 #1
- [ ] 实跑 cartographer / slam_toolbox 建图（目前只编译通过，还没跑过图）
- [ ] 移植 PID / 任务节点（`pid_control_pkg`、`my_launch`，需单位换算）

### 香橙派当前状态（可直接接手）

| 项 | 值 |
| --- | --- |
| ROS 2 | **已装好**：`/opt/ros/humble`，191 个 `ros-humble-*` 包 |
| shell 环境 | `~/.bashrc` 末尾已加 `source /opt/ros/humble/setup.bash`（新登录自动生效） |
| 安装日志 | `~/ros2_install.log` |
| colcon | `/usr/bin/colcon`（随 `ros-dev-tools` 安装） |
| 已确认可用包 | rclcpp / rclpy / sensor_msgs / geometry_msgs / visualization_msgs /
std_srvs / tf2_ros / robot_state_publisher / ament_cmake |
| ROS_DISTRO | `humble` |

> 手工开终端时记得 `source /opt/ros/humble/setup.bash`（或重开 shell）。

### 雷达驱动工作区（已跑通）

| 项 | 值 |
| --- | --- |
| 工作区 | **`~/px4_ws`**（colcon，`--symlink-install` 编过） |
| 包 | `~/px4_ws/src/bluesea2/src/base_lidar` 和 `.../src/bluesea-ros2` |
| 参数文件 | `~/px4_ws/src/bluesea2/src/bluesea-ros2/params/udp_lidar.yaml` |
| 驱动日志 | `~/lidar_run.log` |

**实测结果（2026-10-07）**：

```
话题 /scan 存在，频率 15.06 Hz，一帧 2000 点，角分辨率 0.18°（360°）
frame_id=laser_link，range_max=50.0，scan_time=0.066s
数据有效：1431 个有效测距点，0.27 m ~ 8.99 m
驱动日志：UdpThreadProc: LDS-50C-E-R
```

**编译 / 运行命令**：

```bash
# 编译
cd ~/px4_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# 运行（关键：--params-file 必须指到那个 udp_lidar.yaml）
cd ~/px4_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run bluesea2 bluesea2_node --ros-args \
  --params-file ~/px4_ws/src/bluesea2/src/bluesea-ros2/params/udp_lidar.yaml

# 验证
ros2 topic hz /scan

# 停止
pkill -f bluesea2_node
```

> 注意：`launch/udp_lidar.launch` 里装的是 **Python 内容但扩展名是 `.launch`**，
> `ros2 launch` 可能因扩展名不识别而报错。**直接用上面 `ros2 run` 最稳**，
> 或者把该文件复制一份改名成 `udp_lidar.launch.py`。
> 另外 `params/` 里留了个 `udp_lidar.yaml.bak`（改参数前的备份），可删。

## 6.1 下一步怎么做（给第二个会话）

1. ~~建工作区~~ / ~~跑通 `/scan`~~ —— **已完成**，见上一节。
2. **接 cartographer**（2D SLAM）：把 `D:\24电赛d题\drone_d_ws\src\my_carto_pkg`
   拷到 `~/px4_ws/src/`，`colcon build` 后用它的 `fly_carto.launch.py`。
   注意 `tracking_frame` 用 `laser_link`，`published_frame` 用 `base_link`。
   该包需要额外装 `ros-humble-cartographer-ros`。
3. **PX4 链路**：先定物理链路（串口 TELEM2 ↔ 香橙派 `/dev/ttyS*`，
   和雷达的网口不冲突），再装 `Micro-XRCE-DDS-Agent` + `px4_msgs`，
   验证 `ros2 topic echo /fmu/out/vehicle_status`。
4. **移植 PID / 任务节点**：从 `drone_d_ws` 拷 `pid_control_pkg`、`my_launch`，
   把 `uart_to_stm32` 那层换成 px4_msgs 桥（单位 cm/deg → m/rad，ENU → NED）。

## 7. 约定

### 7.1 文档同步与备份（2026-10-10 用户明确要求「日志同步是双端的，你备份好了吗」）

- **正本（canonical）只在本机**：`D:\px4无人机\CHANGELOG.md` 与 `AGENTS.md`。
- **板上 `~/px4_ws/docs/` 放只读镜像**（方便队友在板上直接翻），文件名相同：
  - 推过去：`scp CHANGELOG.md AGENTS.md orangepi@10.23.87.147:~/px4_ws/docs/`
  - ⚠️ **不要在板上编辑这两个文件**：它们只是镜像，下次从本机推会覆盖。
    队友若在板上改了，必须先在板上 `md5sum` 确认，再拉回本机合并（`scp` 反向）。
- `SSH_ACCESS.md` **只留本机、绝不推到板上**（含明文口令）。
- **备份/产物放在哪，必须在日志里写明路径（2026-10-12 用户要求）**：
  板端备份统一 `~/backups/<时间戳>_<用途>/`（内含 `px4_ws-full-src.tar.gz` + `etc/` + `home/`）；
  本机快照统一"同目录同名 + `.bak-<YYYYMMDD>`"；用户给的参考资料先固化到 `D:\px4无人机\ref_<名字>\`；
  板上文档镜像快照放 `~/px4_ws/docs/<文件>.bak-<YYYYMMDD>`。
  **临时目录（`%TEMP%`、板上 `/tmp`）不算留档**，要留的必须拷到上述固定位置。
  每条改动都要在 `CHANGELOG.md` 里写明**本轮产物的确切路径**。
- 改文档前后各留一份日期快照（与代码的 `.bak-YYYYMMDD` 习惯一致）：
  `cp CHANGELOG.md CHANGELOG.md.bak-$(date +%Y%m%d)`（PC 端同样留一份）。
- 同步完**两端必对哈希**：`md5sum CHANGELOG.md AGENTS.md`（板）vs `Get-FileHash`（本机）。

- 改动机载端前**先备份**，改完在板子上实测
- 不要用热重启来验证网口，优先断电冷启动
- 文档、结论写回本文件和 `SSH_ACCESS.md`，方便下一个会话接手
- **关键性改动必须记录（2026-10-07 用户明确要求）**：任何改动飞控参数、机载软件、
  板子/网络/系统配置、本机代码行为或本目录文档结论的动作，改完都要在
  `CHANGELOG.md` 加一条（改了什么 / 为什么 / 怎么验证 / 怎么回滚），
  并把结论回写本文件。**新会话开始前先读 `CHANGELOG.md`。**

## 8. 2026-10-07 会话记录（垂直起降链路）

详细见 [`CHANGELOG_2026-10-07_垂直起降链路.md`](CHANGELOG_2026-10-07_垂直起降链路.md)。

- **成果**：PX4 ↔ ROS 2 的 uXRCE-DDS 已打通，EKF2 接受了雷达+定高器的位姿 ——
  `/fmu/out/vehicle_local_position` 的 `xy_valid / z_valid / v_xy_valid / v_z_valid` 四个全为 true
- **关键结论①**：Agent 必须跑 **`/dev/ttyS6`**（飞控 UA5/TELEM3 那条线**双向都通**）；
  `/dev/ttyS1` 那条只通一半（收得到、发不回）—— 这是之前握手失败整整半天的根因
- **关键结论②**：DTB 只启用 uart1/3/6/9；**uart4/5/8 绝不能启用**（会分别打掉 PWM 风扇 / HDMI / SPI flash）
  - ⚠️ **2026-10-09 更正**：这条是当时「手工乱改 GMAC 设备树」那轮的结论。串口雷达改造后
    **uart4 已按官方 overlay（`rk3588-uart4-m2.dtbo`）正式启用并实测无害**；
    **uart5 / uart8 仍然不要动。**
- **新增**：`px4_takeoff_land`、`px4_ev_bridge`(v0.2)、`tofsense_m` 三个 ROS 2 包 + `~/tf_to_pose.py`
- **待办**：QGC 补 `COM_RCL_EXCEPT=4`；首飞先 `dry_run` 再 `target_alt=0.5`

### 8.1 修正约定（2026-10-07 用户明确要求）

- **以后每次改动都要补进 `CHANGELOG.md`**（不只"大事"，任何改动都要）。
  写清：改了什么 / 为什么 / 怎么验证 / 怎么回滚。
- 补充记录：`px4_takeoff_land` 曾漏上传到香橙派导致 `launch` 找不到包；
  且 `now_us()` 声明成 `const` 会编译失败（rclcpp 的 `Clock::now()` 非 const）——
  这个坑在 `px4_bridge` / `px4_ev_bridge` / `px4_takeoff_land` 里**各踩了一次**，
  以后新写发布/订阅类节点时先检查这一点。

## 9. 2026-10-09 会话记录（登机核对队友改动 + 同步）

**本次是「核对 + 同步」——机上改动由队友完成，本会话没动机载端。**

- 队友的改动（2026-10-09 15:58–16:35，香橙派上）：
  1. 雷达由网口款 `LDS-50C-E` 换成串口款 `LDS-50C-R`，接 **UART4 → `/dev/ttyS4`**
     （921600 8N1；40pin 排针 19=TX / 23=RX，pinmux `UART4_M2`）；用 `tools/enable_ttyS4.sh`
     往 u-boot 叠 `rk3588-uart4-m2.dtbo` 打开 UART4（`/etc/default/u-boot` 已备份为 `.bak-ttyS4`）。
  2. 新增 `src/bluesea2/src/bluesea-ros2/params/uart_lidar.yaml`（`type=uart`、`port=/dev/ttyS4`、
     `baud_rate=921600`、`dev_id=-1`、`reversed=false`、`max_dist=8.0`、开角度重采样）。
  3. `run.sh` 的 `LIDAR_PARAMS` 默认改 uart 版（可环境变量覆盖）；`fly_carto.launch.py` 注释同步；
     `README.md` 重写；新增 `docs/串口雷达改造.md`、`tools/{enable_ttyS4.sh,check_ev.sh}`、
     `.gitattributes`、`.gitignore`。
  4. 修 `systemd/microxrce-agent.service`：`Environment=XRCE_DEV` 原来写死 `/dev/ttyS3`
     且写在 `EnvironmentFile=` **之后**，会把 `/etc/default/microxrce-agent` 的 `ttyS6` 顶掉
     （ttyS3 是定高器的口）→ 改成默认 `ttyS6`、调整顺序、并注明**保持 disabled**。
  5. `px4_takeoff_land_node.cpp` 用的是修好 `now_us()` 非 const 的版本。
- 本会话做的：
  - 登机核对：dmesg 有 `feb70000.serial: ttyS4 at MMIO 0xfeb70000 ... 16550A`、`/dev/ttyS4` 存在、
    `px4-ev-chain.service` **active**、`MicroXRCEAgent(serial ttyS6)` + 雷达驱动 + cartographer +
    `tf_to_pose.py` + 定高器 + EV 桥**全部在跑**，且日志都在推进。
  - 逐文件哈希比对后，把队友的改动**同步回本机** `D:\px4无人机\px4_ws`（覆盖
    `README.md / run.sh / uart_lidar.yaml / udp_lidar.yaml / fly_carto.launch.py /
    px4_takeoff_land_node.cpp / microxrce-agent.service`，新增 `docs/`、`tools/`、
    `.gitattributes`、`.gitignore`）。
  - 记录一个**未定性现象**：从新开的会话看不到已有 ROS 话题
    （详见 `CHANGELOG.md` 同日「⚠️ 本次发现、尚未定性」小节）。
- **2026-10-09 晚（本会话第二轮改动）**：
  - EV 桥**删掉"自己算的速度"**（位置差分 + 低通），速度字段改发 NaN（= 不提供速度）；
  - 新增 `px4_ws/px4_imu_bridge.py`（板上 `~/px4_imu_bridge.py`）：把飞控
    `/fmu/out/sensor_combined` 转成 `sensor_msgs/Imu` 发到 `/imu`（FRD→FLU，默认 100 Hz 限流）；
  - cartographer 开 `TRAJECTORY_BUILDER_2D.use_imu_data = true`（配置拆成 `fly.lua` / `fly_no_imu.lua`）；
  - `run.sh` 的 `fly` 流程改成 **7 步**（新增 `[2/7]` IMU 桥），并按"`/imu` 有没有数据"自动选配置；
  - `stop.sh` 补齐以前漏掉的进程（`px4_ev_bridge_node / tofsense_m_node / tf_to_pose / MicroXRCEAgent`）。
  - 详见 `CHANGELOG.md` 同日「机载端大改」一节（含 IMU 转换的隔离自检结果与飞控上电后的待验证步骤）。
  - **当晚 20:00 更新**：队友把"EV 上报速度"改回来了（新参数 `report_velocity` 默认 true，
    方差仍保守 `velocity_sigma=1.0`，且跳变当帧不参与差分）→ 飞控已接上、`cs_ev_*` 全 true，
    **定点遥控垂直飞行已可用**。cartographer 目前实际跑的是 `fly_no_imu.lua`（纯激光），
    IMU 那条路还没在真机验证过 —— 要用得显式 `SLAM_USE_IMU=1 bash run.sh fly`。
    本会话还修了 `run.sh` 里"探 `/imu` 永远失败"的 pipefail bug。详见 `CHANGELOG.md` 同日晚间一节。
- 仍未解决的老问题（同 10-07/10-08）：飞控侧接线 / `UXRCE_DDS_CFG` 未定，以及
  「定点模式起飞后失控」的根因（10-08 第二次飞行航向估计空中崩掉）——**先别急着再飞**。

## 10. 2026-10-10 深夜会话记录（第二次炸机复盘 + 香橙派侧加固 + 新建「任务.sh」）

详细见 [`CHANGELOG.md`](CHANGELOG.md) 同日「深夜」一节。要点：

- **「任务.sh」原本不存在**（本机 `D:\` 与板上都搜过）。跑任务一直是手敲
  `ros2 launch px4_takeoff_land takeoff_land.launch.py ...`。本会话新建了
  **`tools/mission.sh`**，以后用它：
  `bash ~/px4_ws/tools/mission.sh [高度 前进 掉头角 悬停秒]`，`DRY=1` 干跑、`FORCE=1` 跳过自检拦截。
  它负责 source 环境 + **强制 FastDDS 走 UDP** + 跑 `tools/preflight.sh`（没过就拒飞）+ 起 launch。
- **「解锁超时」= 误报**：今天 5 次里有 4 次是 `dry_run:=true`（干跑不解锁却傻等 8s）；
  唯一一次真跑（17:52）解锁是成功的（`命令 400 回执 result=0`）。节点已改成干跑到 PRESTREAM 就收尾。
- **⚠️ 关键结论：PX4 的 `pre_flight_checks_pass` 不是"飞机健康度"，而是 `canArm(当前 nav_state)`**
  （`Commander.cpp:1834`，按模式分组）。中止一次任务后飞控停在 OFFBOARD 时，若"此刻没有 Offboard 流"
  就会被拒解锁（`modeCheck.cpp` 的 `mode_req_offboard_signal`）；AUTO_MISSION 无任务、AUTO_LAND 等同理。
  实测：OFFBOARD+无流 false → 切 POSCTL true → 切回 OFFBOARD false。
  节点已加 `SetArmMode` 阶段：先等飞控说可解锁，不行就切一次 POSCTL，并把原因翻成人话打日志
  （订阅了 `/fmu/out/failsafe_flags`）。
- **⚠️ 上次炸机的真凶：飞控自己在飞行中重启**（17:52:42，正悬停 0.37m 做掉头）。
  三条独立证据：agent 日志 17:52:48 `session re-established` + 重创全部 DDS 实体；
  `nav_state_timestamp`(hrt) 反推飞控开机 = **17:52:51**；`latest_arming_reason` 被清零。
  同时排除串口噪声（24s 采样 `fe/brk` 一个都没涨）。**这不是香橙派/DDS 的问题。**
  另：10:27 那次真机也是在 `MISSION_TURN` 处日志戛然而止（无中止、无 SIGINT）。
  → **两次真机都死在掉头段（电流最大时刻），最大嫌疑是供电瞬断（brownout）/物理碰撞，属硬件。**
- **飞之前先做**：① 用户查供电（飞控取电方式/接头/独立供电+电容）；② 从飞控 SD 卡拉 ulog 看复位原因；
  ③ `python3 ~/px4_ws/tools/watch_fc_link.py --minutes 15` 静置观察（飞控重连应为 0 次）；
  ④ 复飞从 `mission.sh 0.5 0 0 3`（只起降）开始，再 0.5m 直线，最后才上 180° 掉头。
- **新增工具**：`tools/watch_fc_link.py`（盯 lpos 间隔 / 状态变化 / agent 会话重连，用来发现"飞控偷偷重启"）；
  同步新增 `tools/mission.sh`。
- **板上环境改动**：`~/.bashrc` 的 UDP 导出**移到"非交互就 return"之前**（原来在末尾，`bash -lc` 不生效），
  另加 `/etc/profile.d/fastdds-udp.sh`。备份 `~/.bashrc.bak-20261010`。
- **流程失误（已记进 CHANGELOG）**：这次改 `px4_takeoff_land_node.cpp` 前没在板上留 `.bak`，被直接覆盖。
  **以后改任何板上文件前先 `cp <文件> <文件>.bak-$(date +%Y%m%d)`。**
- 自检现状（18:0x）：`preflight.sh` **22 项全绿**；链路各话题正常（lpos ~100Hz、最大间隔 46ms）；
  飞控当前停在 **POSCTL**（这是最好歇的状态：可解锁）。

### 10.1 补充（同日稍后）：`tools/` 里的 python 工具现在「不 source 也能跑」

- 用户实测：新 shell 里直接 `python3 ~/px4_ws/tools/watch_fc_link.py` →
  `ModuleNotFoundError: No module named 'px4_msgs'`（本机 `.bashrc` 不会自动 source ROS）。
- 已新增 `tools/_ros_env.py`，并在 `watch_fc_link.py / px4_goto.py / px4_batt_watch.py / watch_pose.py`
  首行加 `import _ros_env`：环境不对就用 bash 把自己重启一次（参数保留），
  没设 FastDDS profile 时自动设成 UDP。四个工具都在「干净 shell」里实测通过。
- 顺手修了三个工具退出时的 `RCLError: failed to shutdown` 红字（加 `if rclpy.ok():` 守卫）。
- **流程提醒**：以后写/测板上 python 工具，**必须在"没 source 过"的干净 shell 里试一遍**
  （`ssh orangepi@… 'python3 …'` 天然就是这种环境）；不然就会像这次一样，用户一跑就报错。
- 但 `bash run.sh` / `mission.sh` 这类 shell 脚本本来就是自己 source 的，不受影响。

### 10.2 同日：ROS 环境写进 `~/.bashrc`（用户要求「不用每次都先 source」）

- `~/.bashrc` 现在在**「非交互就 return」之前**自动 source `/opt/ros/humble/setup.bash`
  与 `~/px4_ws/install/setup.bash`（用 `[ -z "${AMENT_PREFIX_PATH:-}" ]` 防重复）。
  实测：交互 shell / `bash -lc` 都直接有 `ros2`、`px4_msgs`；
  **bash 脚本与 systemd 单元不受影响**（它们不读 `.bashrc`，`BASH_ENV` 为空）。
  备份 `~/.bashrc.bak-20261010b`。
- 因此新会话**不需要**再手敲 source；`tools/_ros_env.py` 保留，兜住 `ssh host 'python3 …'`、
  systemd/cron 这类不读 `.bashrc` 的场合。
- ⚠️ 19:21 曾看到 `px4-ev-chain` 在 `activating`、`px4-chain-watch` 在 `inactive`：
  查清是**香橙派 19:19:12 被重启**（上一次运行无 systemd 关机序列＝断电/硬复位式），
  run.sh 于 19:21:54 恢复正常，与 `.bashrc` 改动无关。**如果是用户自己拔的电就没问题；
  若不是，这就是今天第二次"不干净下电"（第一次是 17:52 飞控飞行中重启）→ 指向供电问题。**

### 10.3 同日深夜第二轮：任务改成「0.5m 高度前进 1.2m（不掉头）」+ 两道交叉校验

- **任务形态（新默认）**：起飞 0.5m → 沿机头方向前进 **1.2m** → 就地悬停 → 交还 PX4 降落。
  **不掉头、不返航**（`turn_deg` 默认 0、新参数 `return_home` 默认 false）。
  恢复老任务：`RETURN=1 bash ~/px4_ws/tools/mission.sh 0.5 0.5 180 5`。
- 新增参数（launch 可调）：`return_home` / `turn_step_deg`(30°，掉头分段缓转) /
  `yaw_xcheck_deg`(20°) / `alt_xcheck_m`(0.5) / `alt_xcheck_abort_m`(0.9) / `xcheck_hold`(1.0s) /
  `tilt_gate_deg`(25°)；阈值设 0 = 关掉那道校验。
- **航向交叉校验**：飞控 `heading` vs **EV 桥发给飞控的 NED 位姿**（`/fmu/in/vehicle_visual_odometry`）
  的 yaw，**比的是相对地面基准的变化量**。
  ⚠️ **千万别用 `/tracked_pose`**：它是 SLAM 的 map 系，实测 yaw≈0° 而飞控 heading≈90.4°（差 90°），
  直接比会一解锁就误报。地面基准实测 −0.4° ✔
- **高度交叉校验**：飞控 `z` vs 定高器 `/tofsense/height`，同样比变化量；倾斜 >25° 时不判
  （TOF 斜射会让读数变长 —— 10-10 的"1.5m/2.07m"就是这么来的，实际只有 60cm）。
- 另外：飞控"已解锁→未解锁"现在会单独报一条 ERROR（空中出现≈飞控重启/掉电/触发保护）。
- 验证：编译通过；两条干跑（新默认 / `RETURN=1` 老形态）正常；两个地面基准合理
  （高度 0.05m、航向 −0.4°）；五个文件板机 md5 一致。

### 10.4 夜间：ulog 复盘 + 三条硬结论（SW 侧）

- 新增 `tools/analyze_ulog.py`（用 PyPI wheel 解到 `/tmp/pyulog_pkg`，不装 pip、不动系统包；
  香橙派能上 PyPI，本机 Windows 只有商店版 python 占位符，解析得在板上做）。
- **①「下午的日志都没有」= 文件没来得及关闭，不是配置问题**：`SDLOG_MODE=0`（解锁开始、上锁结束并关闭）、
  `COM_ARM_SDCARD=1`（没卡不给解锁，而下午两次都解锁成功了）。早上那份可见，正因为它是唯一一次
  **正常上锁**的飞行。⇒ **"日志不见"本身就是硬掉电/硬复位的证据**。
  抢救：卡做 `chkdsk /f`（或 `fsck.vfat -a`）→ `.CHK` 文件 → 日志开头有它自己的文件名可辨认。
  **飞控重启后没再飞过 → 那些块大概率还在，能捞回来。**
- **② 用户说的「PX4 自动断电保护」= 失效检测器 FD 的 Terminate**（`FD_FAIL_R/P=60°`、`TTRI=0.3s`
  → `failsafe.cpp:530` → 立即 `Action::Terminate` = 切动力）。**是结果不是原因**。
  `COM_OBL_RC_ACT=4` 是 **Land**（枚举 …3=Return/4=Land/5=Hold/6=Terminate/7=Disarm），不是断电。
- **③ 这条飞机的位置/高度/速度/航向全靠 SLAM**：`EKF2_EV_CTRL=15`、`EKF2_HGT_REF=3`(Vision)、
  `EKF2_MAG_TYPE=5`(无磁罗盘)、`SYS_HAS_MAG=0`；早上手飞实测 `cs_ev_*` 全 true。
  ⇒ EV 一断/一跳，飞控的航向与高度就没有第二来源 → **航向交叉校验这道新保护是对症的**。
- 早上那份 ulog 是**正常手飞**（RC 解锁→POSCTL 悬停 7s→落地时侧翻 −53°→RC 上锁，无 failsafe），
  **不覆盖 Offboard 路径**，不能单独定案；但它给出了硬件/EV 悬停可用的基线
  （悬停 3~5A；爬升峰值 25.9A、电池 15.7→15.0V，电源扛得住）。

### 10.5 深夜第三次炸机（20:42，"前进 1.2m 不掉头"任务）—— **根因抓到现行**

- **事故点仍然是"前进段刚到位"那一刻**（与掉头无关）。20:42:04.64 前进到位 → **20:42:05.24**
  我新加的航向校验报 `飞控 88.8° / EV 142.8°（偏差 53.9°）` → EV 桥 `20:42:05.57` 起
  **每秒 20+ 次** `SLAM 位姿跳变：航向 45~137°`，直到 20:42:39 → 飞控 20:42:05.44 断链 →
  agent 20:42:10 `session re-established` → 20:42:17 飞控（新开机）报 disarmed。
- **根因**：前进段结束时飞机减速俯仰 → **2D 激光 SLAM（`fly_no_imu.lua`，无 IMU）航向崩掉**；
  而 EV 桥对跳变的反应是**"暂停发 EV 1.0s"** —— 对这条 **位置/高度/航向全靠 EV** 的飞机
  （`EKF2_EV_CTRL=15`、无磁罗盘）等于**让飞控失明**，这次还是每秒钟触发好几次。
  同时 TOF 显示飞机真被推着往上蹿（0.5→1.29→2.10m，倾斜仅 7° 所以不是伪值）→ 电机猛推 → 失控坠落。
- **飞控那份 ulog 没用上**：`log_79_UnknownDate.ulg` 解析出来 `file_corruption=True`、`data_list=0`、
  只干净解析到 56 字节 ⇒ **数据没刷盘就被掉电**（又一次"硬掉电"旁证）；
  路径 `log/sess129/log100.ulg`（sessNNN）= 那次飞控时钟没同步。
- **下一步（优先级）**：① SLAM 带 IMU（治本）——但**先修 `/imu`**：实测 `orientation` 是单位四元数、
  `orientation_covariance[0]=-1`（= 不提供姿态），cartographer 靠它做重力对齐/去俯仰，所以必须让 IMU 桥
  从 `/fmu/out/vehicle_attitude` 发有效姿态并按 body→laser_link 换算；② EV 桥：TOF **按倾斜补偿**
  （`h=tof·cos(tilt)`）取代"跳变丢弃"，并**取消"停发 EV 1s"**（改成只拒绝/降权跳变那一帧）；
  ③ 任务节点加**硬阈值**：航向偏差 >40° 或高度差 >1.5m **立即**中止，另加 EV 停发监控；
  ④ 供电仍要查（猛推时电流裕度）。

## 11. 2026-10-12 会话记录（登机核对 10-11 改动 + 板端备份 + 双端同步；p_to_p 移植待源）

- **10-11 板上改动（本会话只读核对发现，详情见 `CHANGELOG.md` 同日条）**：
  1. `px4_ev_bridge` **回滚成"只查位置跳变"**（删掉 10-10 加的航向跳变判据），
     板上 mtime 2026-10-11 20:39。理由：对称环境 2D SLAM 航向频繁 180° 跳，带航向判据时
     30 秒触发 223 次 reset + 停发 EV 1 s，EKF2 连高度一起丢。
     ⚠️ 根因（**2D 激光 SLAM 没 IMU 时航向会崩**）仍未解决；两版差异见本机 `*.bak-20261012`。
  2. `tools/chain_watchdog.sh` 于 10-11 20:40 改名停用（`px4-chain-watch` 现 disabled/inactive）
     ⇒ `run.sh` 的"分段自愈"目前不生效。
  3. 10-11 17:57 又坠机一次：现场 `~/flight_records/20261011_crash_1757/`，
     回滚前快照 `~/backups/20261011_203907_before_rollback/`。
- **板端备份（新格式，以后每轮大改前照做）**：`~/backups/20261012_203137_before_p2p_port/`
  = `px4_ws-full-src.tar.gz`（src + tools + systemd + docs + 顶层脚本）+ `etc/`（已装单元与 /etc 配置）
  + `home/`（.bashrc、两个 py 工具）；sha256 见 `CHANGELOG.md` 同日条。
- **本机正本已同步**：EV 两个文件 ← 板上版（旧版 `.bak-20261012`）；补回
  `tools/lidar_watchdog.sh`、`systemd/px4-lidar-watch.service`、`docs/README_镜像说明.md`。
- **机载现状**：`px4-ev-chain` active；DDS 走 `ttyS6` @1500000；cartographer 跑 `fly_no_imu.lua`
  （纯激光，**没有** IMU 融合）；IMU 那条路仍未在真机验证。
- **下一步（用户要的方向，口径待确认）**：把 `onboard_ws` 的 p_to_p 点对点逻辑
  （A→B 目标 + 到点判定 + 限速下降到地面基线上方约 4 cm + 交还 PX4 Land + 包线守卫 +
  默认 bench-only 不解锁）移植到 `px4_ws`，用现有 PX4 uXRCE-DDS + EV 链路跑，
  替换它原来那层 STM32 `0x31/0x32` 串口控制。
  **卡点：`onboard_ws` 源文件已随临时目录被清理，需要用户重发**；
  收到后第一步先固化进 `D:\px4无人机\ref_onboard_ws`，再动代码。
- **约束重申（用户 2026-10-12）**：**每次改动都要双端记录**（§7.1 / §8.1），保留。

- **参考件已固化（2026-10-12）**：`onboard_ws`（p_to_p 那套）重发收到 →
  `D:\px4无人机\ref_onboard_ws\`（原件 `onboard_ws.zip` md5 `f0fb2448…` + 解压源码
  `onboard_ws\src\` 8 包 253 文件 + `说明.md`）。**移植从这份取源**；早先那份临时副本作废。- **p_to_p 移植落地（2026-10-12 晚，第一步完成）**：
  - 新增 `px4_ws/src/p_to_p_mission`（任务状态机，代码原样）+ `px4_ws/src/p_to_p_px4`
    （精简到 `frames` / `position_ramp` / `offboard_bridge`）+ `p_to_p_mission/launch/p_to_p.launch.py`
    （只起任务+桥，雷达/SLAM/EV 仍由 `run.sh` 负责）；
  - **不搬** USB MAVLink / TOF / EV 支路（本机 `px4_ev_bridge` 已独占 `/fmu/in/vehicle_visual_odometry`）、
    **不搬** STM32 `0x31/0x32` 那层；
  - 默认 `bench_only=true`、`allow_mode_command=false`、`allow_land_command=false`、
    `land_on_mission_complete=false`；任务默认 `terminal_behavior=hold`、包线 1.5 m / 1.2 m / 各段 20 s；
  - 验证：`py_compile` + YAML 通过、`colcon build` 2 包成功、30 秒 bench 预演（节点起来、
    `readiness_check` 成功、`/fmu/in/*` 无消息）均已实测；**未做真机/未解锁**；
  - 改前备份 = `~/backups/20261012_203137_before_p2p_port/`（回滚：删这两个包目录即可）。
  - 下一步：飞控上电后核 `/height` → 拆桨演练 `bench_only:=false` 的 Offboard 流程 →
    再验 `guided_land_handoff`。详见 `CHANGELOG.md`「补记 3」§4。
### 11.1 p_to_p 移植：下次照着做的操作清单（runbook，2026-10-12 写入）

板上日志 `~/px4_ws/docs/CHANGELOG.md` 的「补记 4」是**可直接复制执行**的完整清单；要点：

1. **先只读体检**：`bash ~/px4_ws/tools/preflight.sh` + `python3 ~/px4_ws/tools/watch_fc_link.py --minutes 5`，
   `/scan` ~15 Hz、`/tofsense/height` ~55 Hz、`/fmu/in/vehicle_visual_odometry` ~15 Hz；
2. **只预演**：`export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml`（必须！）→
   `ros2 launch p_to_p_mission p_to_p.launch.py`（`bench_only` 默认 true），
   看 `/p_to_p/state` = IDLE…、`/px4_bridge/state` = BENCH_LINK_OK、`tf2_echo map base_link` 查得到；
3. **飞控上电后先验 `/height`**（厘米、有数据）—— 没数据不许往下走；
4. **拆桨**地面演练：`bench_only:=false cruise_height_cm:=50 b_offset_x_cm:=0 terminal_behavior:=hold`；
   遥控器先解锁再切 Offboard；看 `/fmu/in/trajectory_setpoint` ~20 Hz；
5. **一次只放开一个变量**：前进距离 0→50→100 cm，高度 50→80→120 cm，最后才试 `guided_land_handoff`；
   每改一次都要在 `CHANGELOG.md` 记一条并推板上镜像；
6. **回滚**：`rm -rf ~/px4_ws/src/{p_to_p_mission,p_to_p_px4}` + 重编；
   或整机 `tar -xzf ~/backups/20261012_203137_before_p2p_port/px4_ws-full-src.tar.gz -C ~/px4_ws`；
7. **红线**：不再引入第二个 `/fmu/in/vehicle_visual_odometry` 发布者；不同跑 `px4_bridge`/`px4_takeoff_land`；
   不跳过 preflight；包线是节点本地限制、不是 geofence。
