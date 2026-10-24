# 关键改动记录（Key Changes Log）

> 建立于 **2026-10-07**。目的：新会话接手时不用重新整理资料。
>
> **规则**：凡属于「关键性改动」——改飞控参数、改/装板子上的软件、改网络或系统配置、
> 改本机代码行为、改本目录文档里的结论——**改完必须在下表加一条**，写清
> 「改了什么 / 为什么 / 怎么验证 / 怎么回滚」。纯阅读、查询、报错排查不用记。
>
> **用户原话（2026-10-07）**：
> 「你后面做的所有关键性改动都要记录下来，把这句话也记录下来，
> 方便后面我开新对话的时候减少资料整理时间。」

## 变更历史

| 日期 | 改了什么 | 为什么 | 怎么验证 | 怎么回滚 |
| --- | --- | --- | --- | --- |
| 2026-10-07 | 建立本文件；在 `AGENTS.md` §7 增加「关键性改动必须记录」约定 | 用户明确要求 | 本文件存在；`AGENTS.md` §7 有该条 | 删除本文件，回退 `AGENTS.md` §7 |
| 2026-10-07 | 首飞前**只读**排查（未改任何东西），结论写入 `AGENTS.md` §2.1：飞控 MicoAir743 v2、固件 PX4 1.15.4（构建目标 `micoair_h743-v2_default`）、本机 QGC 位置与参数缓存证据 | 用户问「现在有什么能起飞、怎么用遥控器起飞」 | 见 `AGENTS.md` §2.1 的「证据」列 | 无需回滚（只读） |
| 2026-10-09 | **队友改动（本会话登机核对并同步）**：雷达由网口款 **LDS-50C-E** 换成串口款 **LDS-50C-R**（接 UART4 `/dev/ttyS4`，需 u-boot overlay `rk3588-uart4-m2.dtbo`）；新增 `px4_ws/docs/串口雷达改造.md`、`px4_ws/tools/{enable_ttyS4.sh,check_ev.sh}`、`px4_ws/{.gitattributes,.gitignore}`；`run.sh` 的 `LIDAR_PARAMS` 默认改 `uart_lidar.yaml`；修 `systemd/microxrce-agent.service` 串口默认值（ttyS3→ttyS6）；本机 `px4_ws` 已同步 | 硬件换成了串口款雷达；原 service 默认值会把定高器的 ttyS3 顶掉 | 详见本文件「2026-10-09 队友改动同步」一节 | 见该节「回滚」 |
| 2026-10-09 | **香橙派侧配套改动**：`px4_ev_bridge/launch/ev_bridge.launch.py` 新增并显式传入 `position_sigma=0.25`、`velocity_sigma=1.0`（原来是节点默认 0.05 / 0.10，launch 里看不到），`yaw_sigma_deg` 保持 25；本机改 + 推送香橙派 + `systemctl restart px4-ev-chain` 生效 | 飞控侧把 `EKF2_EVV/EVA/EVP_NOISE` 调高后，香橙派这侧要同步"少信自己的数据"；其中 `velocity_sigma` 最关键 —— EV 的"速度"是位置差分算出来的，不是传感器 | 见本文件「2026-10-09 香橙派侧配套改动」一节 | 见该节「回滚」 |
| 2026-10-09 | **① EV 桥彻底删掉"假速度"**（速度字段改发 NaN = 不提供速度）；**② 新增 IMU 桥** `px4_imu_bridge.py`（飞控 `/fmu/out/sensor_combined` → `sensor_msgs/Imu`，FRD→FLU，发 `/imu`）；**③ cartographer 开 `use_imu_data = true`**，并新增 `fly_no_imu.lua` 变体，`run.sh` 按"有没有 /imu 数据"自动二选一；**④ `run.sh` 加 [2/7] IMU 桥步骤（重编号 x/7）**、`stop.sh` 补齐漏掉的进程 | 位置差分出的"速度"不是独立信息却被 EKF 当独立速度测量融合；2D 激光倾斜会污染航向，需要陀螺提供旋转预测 | 见本节「验证」 | 见本节「回滚」 |
| 2026-10-09 晚 | **队友第二轮改动（已同步）+ 本会话修的一个 bug**：EV 桥**恢复**上报位置差分速度（新参数 `report_velocity` 默认 true、跳变当帧不参与、`velocity_sigma` 仍保守 1.0）；`check_ev.sh` 增加"EV 有没有订阅者"判据；新增 `tools/fc_link_probe.sh`（裸探飞控串口）；`.bashrc` 加 `ROS_DOMAIN_ID=0`。本会话修掉 `run.sh` 里"探 `/imu` 在 pipefail 下永远失败"的 bug，并把 IMU 模式改成显式开关 `SLAM_USE_IMU=1` | 用户要求飞控能收到速度；**飞控已接上，定点遥控垂直飞行已可用**；IMU 模式尚未在真机验证，不该默默启用 | 见本文件「2026-10-09 晚（20:00–20:30）」一节 | 见该节「回滚」 |
| 2026-10-09 晚 | **`px4_takeoff_land` 新增 Offboard 小任务航线**：`mission_enable:=true` 时走「起飞 → 沿起飞机头方向前进 `forward_dist` → 原地掉头 `turn_deg` → 保持新航向回起飞点正上方 → 悬停 → 交还 PX4 自动降落」；新增 3 个阶段与 7 个参数；launch 里数值参数改用 `ParameterValue(..., value_type=float/bool)`（修掉"`turn_deg:=180` 整数导致节点崩溃"） | 用户要试 offboard 任务（起飞50cm→前进50cm→掉头→回原位） | 拆桨 dry_run 实测：任务点几何与手算一致、阶段推进正常（见本节） | `mission_enable:=false` 即回到原行为；文件回滚见本节「回滚」 |
| 2026-10-09 晚 | **Offboard 任务首飞炸机复盘 + 加固**：根因 = 飞控↔香橙派 uXRCE-DDS 会话中途断开（串口接收方向实测 **74.7 KB/s / 上限 90 KB/s ≈ 83%**，驱动有 `fe/brk/oe`），PX4 因 `COM_OF_LOSS_T` 掉出 Offboard；次因 = TOF 高度偶发离谱值（贴地读到 **2.55 m**）直接就是 EV 的 z、且任务节点不知道自己链路断了。加固：① EV 桥加高度跳变保护；② 任务节点 offboard 流 50→**10 Hz**；③ 任务节点加**飞控链路看门狗**；④ ABORT 日志打印现场状态；⑤ launch 数值参数类型强制 | 用户第一次跑 offboard 任务时炸机，要把原因查清并防复发 | 见本节时间线 + 对照实验（停 IMU 桥流量不变，证明是 FC 自己发满） | 见本节「回滚」（`.bak-20261009b`） |
| 2026-10-16 | **机载代码整体回退到 2026-10-09 晚那一版**（老方案：`run.sh fly` + `px4_ev_bridge` + `px4_takeoff_land`）；`px4-p2p-chain`→disabled、`px4-ev-chain`→enable；板上加 systemd drop-in（`XRCE_BAUD=1500000` + 强制 FastDDS UDP profile） | 用户 2026-10-16 明确要求「回退香橙派代码到刚给你发的那版 10.9 号的版本」（微信 `restore_1009_evening.zip`） | 见本文件「2026-10-16 回退」一节：五项话题频率全绿 + EV「高度有效」 | 见该节 §6「怎么回退这次回退」 |
| 2026-10-16 | **本机 `D:\px4无人机\px4_ws` 建 git 留档**：`git init` + commit `4f4fa18`、tag `snapshot-20261016-board`（= 当时板上那版；逐文件 md5 对齐：442 个相同、2 份 `docs/*.md` 按本机正本刷新、板上多出的只有 `__pycache__`/`.pytest_cache`/`*.bak-*`）；`.gitignore` 增 `.pytest_cache/` | 用户要求「先在本地 git 现在香橙派这版代码」 | 见本文件「2026-10-16 px4_ws git 留档」一节 | `git -C "D:\px4无人机\px4_ws" reset --hard 4f4fa18`（或删掉 `.git`） |
| 2026-10-16 | **炸机复盘（只读排查，未改任何东西）**：结论 = **飞控在空中自己重启**（IMU 流中断 **7.8 s** + DDS `session re-established` + `latest_arming_reason` 被清零）；同时发现 **SLAM 位置跑飞**（0 → −15.3 m / 25 s）与 **TOF 高度乱跳**（0.19~0.83 m）。排除项：机载侧没有任何看门狗能重启飞控、PX4 在 armed 时拒绝 246/PREFLIGHT_STORAGE、失败检测器动作是 Disarm 而非 reboot、FMU 无硬件看门狗、`CBRK_FLIGHTTERM=121212` | 用户问「低空飞行 20 多秒后像断电一样掉下来，是硬件还是软件」 | 见本文件「2026-10-16 炸机复盘」一节 | 无需回滚（只读排查） |
| 2026-10-17 | **机载端新增独立工程 `~/mavlink_ws`（队友那套 MAVLink 定位链）并切换为开机自启**（新单元 `mavlink-ws-chain.service`；`px4-ev-chain` / `px4-p2p-chain` 全部 disabled 但保留）；飞控参数改为 `UXRCE_DDS_CFG=0` / `SER_TEL3_BAUD=115200` / `MAV_1_CONFIG=103` / `EKF2_HGT_REF=2` / `EKF2_RNG_CTRL=2` / `EKF2_RNG_POS_Z=0.044` | 用户要求「把这个工作空间放到最初那架（147）上试试能不能用」，并明确「一开机就用新链而不是旧链」 | 见本文件「2026-10-17 mavlink_ws 落地」一节：飞控 uORB 上 `vehicle_visual_odometry` 与 `distance_sensor` 双向确认、高度稳定 9~10 cm、无 preflight 报错，**已实际飞行且比原方案更好** | 见该节 §6（切回旧链 = 两条 systemctl + 飞控参数复原） |
| 2026-10-17 | **给 `mavlink_bridge` 加"跳变保护"（刻意偏离队友原版）**：跳变 > `odom_max_position_jump_m`(0.5m) 时**停发 EV、不递增 reset_counter**；回到原坐标系并稳定 2s 才恢复；稳定在新位置则判定真重定位、保持停发并报 ERROR。另加启动**预热**（稳定 2s 才发第一帧）。新增参数 `odom_jump_hold_s: 2.0` | 2026-10-17 22:07 SLAM 单帧跳 **272.35 m**，原版"丢一帧+reset_counter+1"让下一帧变成"合法重定位"，PX4 把 EKF 位置锚到 273 m 外，位置环追假目标 → 翻机 | 注入 5.01 m 假位姿实测：`停止发送 EV（不递增 reset_counter）` ✓、`判定为 SLAM 重定位，EV 保持停发` ✓；恢复后 `EV 预热完成` ✓ | 覆盖 `bridge_node.py.bak-20261017` + `mavlink_bridge.yaml.bak-20261017` → 重编译 `mavlink_bridge` → 重启 `mavlink-ws-chain` |

## 首飞相关待确认（下次接着做）

- **机架类型 / 遥控器 / 接收机型号 / 电池**：本机无任何记录，需向用户确认后补进 `AGENTS.md` §2.1。
- **首飞前必查 `COM_RC_IN_MODE`**：若被改成 `1`（Joystick only）则遥控杆完全失效；
  `px4_ws/src/px4_takeoff_land/README.md` 里的建议参数表曾建议改成 1。
- 遥控首飞时**不要**运行 `px4_bridge` / `px4_takeoff_land` / `px4_ev_bridge`
  （它们会发 `/fmu/in/offboard_control_mode`，和遥控器抢控制权），香橙派最好断电。

## 2026-10-07 追加：定点模式解锁不了的机制（已查源码确认）

- **现象**：上电后在 **Position（定点）** 模式下解锁被拒（用户 2026-10-07 反馈）。
- **机制**（源码 `src/modules/commander/ModeUtil/mode_requirements.cpp`，PX4 1.15.4）：
  - **Position** 需要：角速度 + 姿态 + **水平位置 `local_position_relaxed`** + 高度 `local_alt` + 遥控输入
  - **Stabilized** 需要：角速度 + 姿态 + 遥控输入
  - **Altitude** 需要：角速度 + 姿态 + **高度 `local_alt`** + 遥控输入
  → 室内没有 GPS / 光流 / VIO 时，EKF 给不出 xy 位置，**定点模式必然解不了锁**
    （事件 `check_modes_local_pos`，消息 "No valid local position estimate"）；
    Stabilized / Altitude 则能解锁。检查逻辑在
    `src/modules/commander/HealthAndArmingChecks/checks/modeCheck.cpp`。
- **排查命令**（QGC → 分析工具 → MAVLink 控制台）：
  - `commander status` → 看 `Preflight check: OK/FAILED`、`navigation mode: ...`
  - `listener failsafe_flags` → 看 `local_position_invalid_relaxed` / `local_altitude_invalid` /
    `attitude_invalid` / `manual_control_signal_lost` 哪个为 true
- **状态**：机制已确认；**具体被哪一条拒绝，待用户在实机上跑上面两条命令确认**。

## 2026-10-07 追加：用 EV（雷达+测高器）在定点模式起飞 —— 卡点清单

**用户目标（2026-10-07 澄清）**：关闭气压计 / 磁力计，位姿由雷达 SLAM + TOFsense-M
定高经 `px4_ev_bridge` 送给 EKF2，**要在 Position（定点）模式解锁起飞**，
不是改用 Stabilized。

### 定点模式解锁的必要条件（源码已确认）

- `EstimatorChecks.cpp: setModeRequirementFlags()`：
  - `local_position_invalid_relaxed` ← 需要 `lpos.xy_valid == true`、`lpos.eph < COM_POS_FS_EPH`、
    数据不超时（`COM_POS_FS_DELAY`），且**从无效变有效要连续通过 1 s**（`checkPosVelValidity` 观察期）
  - 未解锁时 `xy_valid` 还会被 `pre_flt_fail_innov_heading` / `pre_flt_fail_innov_vel_horiz` 强制置 false
  - Position 还要求 `lpos.z_valid == true`（`mode_req_local_alt`）

### 关键参数默认值（PX4 1.15.4 实测查源码）

| 参数 | 默认 | 备注 |
| --- | --- | --- |
| `EKF2_EV_CTRL` | **15** | EV 水平/垂直/速度/航向全开，不是 0 |
| `EKF2_HGT_REF` | **1 = GPS** | 没装 GPS 时必须改 **3(Vision)** 或 2(Range)，否则 z 永无效 |
| `EKF2_EV_DELAY` | 0 | cartographer 延迟大，通常要按实测加 |
| `EKF2_MAG_TYPE` | 0 | **只有 0/1/5 三个值；`=2` 是无效值**（`px4_ev_bridge/README.md` 里写的 2 是错的） |
| `UXRCE_DDS_SYNCT` | 1 | 时间同步默认开 |

### 时间戳机制（澄清，不用改代码）

PX4 的 XRCE-DDS 桥对 ROS→PX4 的消息会做
`topic.timestamp = min(timestamp - time_offset, hrt_absolute_time())`
（`Tools/msg/templates/ucdr/msg.h.em`），即自动换算到飞控时间基并 clamp 到当前。
所以 `px4_ev_bridge` 里用 ROS 系统时钟（`now_us()`）是**可以**的。

### 待用户实测确认

QGC → 分析工具 → MAVLink 控制台跑：
`listener vehicle_visual_odometry`、`listener estimator_status_flags`、
`listener vehicle_local_position`、`listener estimator_status`，
看 `cs_ev_pos/cs_ev_hgt/cs_ev_yaw`、`xy_valid/v_xy_valid/z_valid/eph`、`pre_flt_fail_innov_*`。

## 2026-10-07 追加：飞控实测日志（用户提供）—— 根因 = EV 从未进飞控

**实测输出**（PX4 1.15.4 nsh，MicoAir743 v2）：

- `listener vehicle_visual_odometry` → **`never published`**
  → `px4_ev_bridge` 的数据**从来没到过飞控**（uXRCE-DDS 链路未建立）。
- `estimator_status_flags`（2 个 EKF 实例，内容一致）：
  `cs_ev_pos / cs_ev_hgt / cs_ev_yaw / cs_ev_vel` **全 False**；
  `cs_inertial_dead_reckoning = True`、`cs_fake_pos = True`、`cs_fake_hgt = True`；
  `cs_gps / cs_mag / cs_baro_hgt / cs_rng_hgt / cs_opt_flow` 全 False；
  `cs_tilt_align = True`、`cs_yaw_align = False`、`cs_vehicle_at_rest = True`。
- `vehicle_local_position`：`xy_valid / z_valid / v_xy_valid / v_z_valid` **全 False**，
  `dead_reckoning = True`，`xy_reset_counter = 9`，`heading_good_for_control = False`，
  `ref_lat/ref_lon/ref_alt = nan`，`eph=0.0136`，`evh=0.0413`。
- `dist_bottom = 0.0986 / valid=True` 但 `dist_bottom_sensor_bitfield = 0 (NONE)`
  → 那是 EKF 地形估计的假值，**没有测距传感器在喂 EKF**
  （依据 `ekf2/EKF2.cpp:1622-1624`）。

**结论**：EKF 处于「纯惯导 + 假位置」状态，Position 和 Altitude 都必然解不了锁。
根因是 **EV 没进飞控**，不是参数调错。

**下一步（按顺序验证）**：
1. 建 uXRCE-DDS 链路（`AGENTS.md` §6 里仍未打勾的那项）
2. `listener vehicle_visual_odometry` 能看到数据
3. `cs_ev_pos / cs_ev_hgt / cs_ev_yaw` 变 True
4. `xy_valid / z_valid` 变 True（约 1 s 观察期）→ 定点模式可解锁
5. 另需确认 `EKF2_HGT_REF` 是否已设为 **3（Vision）**（默认是 1 = GPS，没 GPS 时高度参考无效）

**遗留风险**：EV 航向是目前唯一的 yaw 源（`cs_yaw_align=False`，磁力计已关），
起飞前必须核对 `heading` 与实际机头方向一致，否则一起飞就自旋。

## 2026-10-07 修复记录 #1：EV 链路（香橙派侧已就绪，剩接线 + 飞控侧配置）

### 改了什么

1. `px4_ws/src/px4_ev_bridge/src/px4_ev_bridge_node.cpp`（本机 + 香橙派均已更新）
   - 新增参数 `publish_rate_hz`（默认 **30 Hz**，低于 5 会被强制改成 5）
   - 新增 `publishTick()`：改成**固定频率重发最近一帧位姿**。原来是"收到新位姿才发"，
     而 EKF2 要求 EV 间隔 < `EV_MAX_INTERVAL`(=200 ms)，cartographer 达不到 →
     `starting_conditions_passing` 永远为假、EV 融合根本不启动
   - SLAM 位姿超过 `pose_timeout` 时**停止发布**（绝不发冻结的旧位置）
   - 文件头补上 `EV_MAX_INTERVAL` 的源码依据
2. `px4_ws/src/px4_ev_bridge/README.md`
   - `EKF2_MAG_TYPE` **2 → 5**（1.15.4 只有 0/1/5，2 是无效值）
   - 新增「发布节奏」章节；参数表补 `publish_rate_hz`、`SYS_HAS_GPS`、
     `UXRCE_DDS_CFG`、`EKF2_EV_NOISE_MD`
   - 「起飞前必须确认」换成飞控端 3 条 listener + 香橙派端 2 条命令
3. 新增 `px4_ws/agent.sh`（`start|stop|status|detect`，`detect` 用
   `/proc/tty/driver/serial` 的 rx 计数判断飞控接在哪路 UART）
4. 新增 `px4_ws/systemd/microxrce-agent.{service,default}`（用 `--dev`/`-b` 起 Agent）
5. `px4_ws/run.sh` 新增 `agent` 模式；`px4` 模式会检查并提示 Agent 没在跑

### 验证（2026-10-07 实测）

- 香橙派 `colcon build --symlink-install --packages-select px4_ev_bridge` → `Finished [31.4s]` ✅
- `bash agent.sh start` 能起（日志 `TermiosAgentLinux ... fd: 3`，
  实际命令行 `serial --dev /dev/ttyS3 -b 921600`）✅，已 stop，无残留进程
- `MicroXRCEAgent serial --help` 参数确认：`-D/--dev`、`-b/--baudrate`（默认 115200）✅
- systemd 文件已装到 `/etc/systemd/system/microxrce-agent.service` 与
  `/etc/default/microxrce-agent`，**尚未 enable**（等接线确认端口）
- ⚠️ 串口实测：`ttyS1 / ttyS3 / ttyS6` 的 `tx:0 rx:0`（**飞控没接**），
  `ttyS7` 是板载蓝牙（`brcm_patchram_plus` 在用），别选它

### 回滚

- 香橙派：`~/px4_ws/src/px4_ev_bridge/src/px4_ev_bridge_node.cpp.bak-20261007`、
  `~/px4_ws/src/px4_ev_bridge/README.md.bak-20261007`
- 本机：对本文件的 apply_patch 反向回退

### 还没做（必须人工，我碰不到硬件/飞控）

1. 飞控 TELEM2 ↔ 香橙派空闲 UART 接线（TX↔RX 交叉、共地）
2. 飞控侧 `UXRCE_DDS_CFG` = 那路串口，`SER_TELx_BAUD=921600`，重启
3. 飞控参数：`EKF2_HGT_REF=3`、`EKF2_MAG_TYPE=5`、`SYS_HAS_GPS=0`
4. 接线后用 `sudo bash ~/px4_ws/agent.sh detect` 确认端口，再
   `sudo systemctl enable --now microxrce-agent`

## 2026-10-07 修复记录 #2：`run.sh` 新增 `fly` 一键起飞链路

- 新增 `bash run.sh fly [carto|slam]`：后台依次拉起
  Agent → 雷达（等 25 s 握手）→ SLAM（等 15 s）→ EV 桥，
  最后打印进程列表、EV 日志尾部和飞控端验收命令。
- `stop_all()` 补上 `MicroXRCEAgent` 与 `px4_ev_bridge_node`（原来漏了，`stop` 停不干净）。
- 实测：香橙派上 `bash -n run.sh` 通过，用法行变为
  `{init|lidar|carto|slam|agent|px4|fly|status|stop}`。
- 为什么需要：定点模式依赖香橙派**持续**供位姿，EV 断 > 400 ms
  （2×`EV_MAX_INTERVAL`）EKF2 就会关掉 EV 融合 → `xy_valid=false` →
  空中触发位置失效 failsafe。所以起飞前必须整条链路都拉起来，飞行中不能停。

## 2026-10-07 修复记录 #3：**香橙派重启 = EV 全丢** → 改成 systemd 开机自启

### 现象与根因

用户反馈「（飞控端）还是 never published」。上板实测：

| 证据 | 结果 |
| --- | --- |
| 相关进程（Agent/雷达/SLAM/EV/定高器） | **一个都没有** |
| `/tmp/*.log` | **全部不存在**（/tmp 重启即清空）→ **香橙派重启过** |
| `/proc/tty/driver/serial` | ttyS1/3/6 全 `tx:0 rx:0`（当时） |

根因：此前整条链路是**手工 nohup 一次性启动**（见
`CHANGELOG_2026-10-07_垂直起降链路.md` §7），**没有任何自启机制**。
香橙派一重启，Agent/雷达/SLAM/EV 全没了 → 飞控侧 `vehicle_visual_odometry`
自飞控开机起一个包都没收到 → `listener` 显示 `never published`。
**链路本身没问题，是"没跑"。**

### 同时修正的两个错误（我上一轮引入/未覆盖的）

1. `agent.sh` / `microxrce-agent.default` 默认串口原是 `/dev/ttyS3` —— 那是
   **定高器 TOFsense-M 的口**，占用它高度就没了。已改为 **`/dev/ttyS6`**（飞控 DDS）。
2. `run.sh fly` 只起了 Agent+雷达+SLAM+EV，**漏了另外两个必需环节**：
   - `tf_to_pose.py`（这版 cartographer 不发 `/tracked_pose`，必须由它转发 TF）
   - `tofsense_m`（定高器，`/dev/ttyS3`）
   已重写为 6 步完整链路，并在末尾打印 4 个关键话题的频率。

### 改动清单

- `px4_ws/run.sh`：`start_fly()` 重写为 6 步；`stop_all()` 补 `tofsense_m_node`、`tf_to_pose`
- `px4_ws/agent.sh`：默认 `XRCE_DEV=/dev/ttyS6`；`detect` 提示已知占用
- `px4_ws/systemd/microxrce-agent.default`：`XRCE_DEV=/dev/ttyS6`
- **新增 `px4_ws/systemd/px4-ev-chain.service`**（oneshot + RemainAfterExit=yes，
  `ExecStart=run.sh fly`、`ExecStop=run.sh stop`、TimeoutStartSec=180）
- 把板子上的 `~/tf_to_pose.py` 取回本机存一份 `px4_ws/tf_to_pose.py`（原来只在板子上）

### 验证（2026-10-07 20:48 实测，服务 active）

```
MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600      ← Agent 会话已建立
/scan                          average rate: 15.086
/tofsense/height               average rate: 55.623
/tracked_pose                  average rate: 29.966
/fmu/in/vehicle_visual_odometry average rate: 30.013   ← EV 以 30Hz 发出（新固定频率生效）
EV: NED pos(0.00, -0.00, -0.04) yaw 89.8° | 高度有效
ttyS6: tx:438175 rx:9767759    ← 飞控会话活着
ttyS3: rx:631008                ← 定高器在发
```

`px4-ev-chain.service` 已 **enable**（`multi-user.target.wants` 里有软链），
以后香橙派上电即自动拉起整条链路。
停止/禁用：`sudo systemctl stop|disable px4-ev-chain`。

## 2026-10-07 待排查：解锁推油门后原地顺时针自旋（未改动任何代码）

**用户现象**：解锁正常；推油门后无人机**原地顺时针自旋并上升**。
**关键**：早在**自稳模式**下就是这个现象，且现在 EV 链路已通仍然是 →
**与雷达/SLAM/EKF/定点模式无关**，问题在偏航控制链路或机体本身。

**源码依据（PX4 1.15.4）**：
- `mc_att_control_main.cpp:127`：`attitude_setpoint.yaw_sp_move_rate = manual.yaw * MPC_MAN_Y_MAX`
  （`MPC_MAN_Y_MAX` 默认 **150 deg/s**）→ 自稳模式下偏航杆给的是**角速度指令**，
  所以**任何"持续的偏航杆输入"都会变成持续自旋**（速度 ≈ 偏移×150°/s）。
- `control_allocator`：`CA_ROTOR${i}_KM` 注释明确「正 = CCW，负 = CW」；
  `4001_quad_x` 里 rotor0/1 为正、rotor2/3 为 -0.05；
  `CA_R_REV` 是逐电机反向的位掩码（QGC 执行器页的 Reverse 勾选）。

**两个最可能的原因（待用户实测确认）**：
1. 遥控器 yaw 通道未真正回中 / 有微调 / 映射到非回中通道 → 持续角速度指令。
   查：QGC 遥控器页回中应显示 1500；nsh `listener manual_control_setpoint` → `yaw` 应为 0.000。
2. 电机转向与 PX4 期望的「对角线同向、相邻反向」不符（例如两条对角线的
   CW/CCW 整体对调）→ 姿态正常、能爬升，但**偏航反号 → 持续自旋**。
   查：拆桨，QGC 执行器页逐个点电机，核对物理位置与转向。

**辅助判据**：nsh `listener actuator_outputs` / `actuator_motors`，慢慢推油门——
四路出现**持续差动** = 飞控在主动命令偏航（指向遥控器或航向估计）；
**对称** = 机械/转向问题。
再手动把机身顺时针转，`listener vehicle_angular_velocity` 的 `xyz[2]` 应为**正**（NED 约定）。

**另记**：EV 桥静止时打出 `yaw 89.8°`，说明 SLAM 的 ENU→NED 换算带约 90° 偏置。
这不会导致自旋，但会让定点模式下的「前推杆」方向与机头差 90°，起飞前要对核。

## 2026-10-08 事故复盘：定点模式起飞后持续右飘、无法控制 —— 根因 = cartographer 崩溃后喂了冻结位姿

**现象**：定点模式正常起飞，油门不动时持续向右飞，遥控器停不下来。

**根因链（实测证据）**：

1. `cartographer_node` **崩溃**：
   `terminate called after throwing an instance of 'std::length_error'`
   `what(): cannot create std::vector larger than max_size()` → `process has died ... exit code -6`。
   `ros2 node list` 里已无 `/cartographer_node`（`occupancy_grid_node` 还活着，所以外表看不出）。
2. `tf_to_pose.py` **继续以 30 Hz 重发最后那一次成功查到的 TF**，
   `/tracked_pose` 的 `header.stamp` 永远停在崩溃那一刻（`sec: 1791448428` = 2026-10-08 16:33:48）。
3. `px4_ev_bridge` 的守护只看**消息到达时间**，不看**消息自带的时间戳** →
   以为"位姿一直很新鲜"，继续以 30 Hz 把**冻结的、错的**位置发给 EKF2：
   `EV: NED pos(114.86, -1073.44, -0.05) yaw 126.7°`（跳变 1079.56 m、航向跳 36° 后定住）。
4. EKF2 收到"位置恒定 + 速度≈0"的假测量 → 认为飞机**静止在原地** →
   位置环不纠偏、不减速 → 飞机随着固有倾斜偏差一路飘走，遥控也"停不下来"
   （且航向跳了 36°，推杆方向也被带偏）。

**结论**：不是 EKF、也不是飞控参数问题；是**上游 SLAM 挂了以后，整条链路没人发现，把冻结数据当有效数据喂给了飞控**。

**必须补的三道防线（待实施，用户尚未授权改代码）**：

1. `tf_to_pose.py`：用 **TF 自带的 stamp** 发布；TF 停止更新（或 lookup 失败）时**停止发布**，
   不能拿旧变换无限重发。
2. `px4_ev_bridge`：守护条件从"有没有新消息"改成 **"`header.stamp` 是否在推进"**；
   stamp 不推进超过阈值 → 停发 EV。这样 EKF2 会在 400 ms 后自动关闭 EV 融合、
   `xy_valid` 变 false，飞控进入位置失效 failsafe（Hold/Land），远好于"带着冻结位置自由飞"。
3. `cartographer` 崩溃要能自愈：`ros2 launch` 默认不 respawn，需要加 `respawn=True`
   或 systemd 看护；另外查 `std::length_error` 的触发（常见是异常扫描数据/时间跳变，
   可给 `/scan` 过滤 inf/0 距离、在 `fly.lua` 设 `max_range` 上限）。

**首飞纪律（务必）**：只要定位来自外部 SLAM，起飞后就必须有人在飞控端盯
`listener vehicle_local_position` 的 `xy_valid`；一旦变 false 立刻切自稳手动接管。

## 2026-10-08 修复记录 #4：给 EV 定位链路加三道防冻结守护（已落地 + 已验证）

> 用户指令原话：「改，并记录下来」

### 改了什么（本机 + 香橙派各一份，已同步）

1. **`px4_ws/tf_to_pose.py`**（板子上是 `~/tf_to_pose.py`）
   - 新增参数 `stale_timeout`（默认 0.3 s）
   - `tick()` 里记录 TF 的 `(sec, nanosec)`，判断**该时间戳有没有在变**；
     连续超过 `stale_timeout` 不推进 → 打 error 并**停止发布** `/tracked_pose`
   - 关键点：`tf.header.stamp` 与本机时钟有固定偏移（实测约 +0.3~1.1 s 且会浮动），
     所以**不能**用它与 `now()` 比大小，只能比较"是否变化"
2. **`px4_ws/src/px4_ev_bridge/src/px4_ev_bridge_node.cpp`**
   - 新增参数 `stamp_timeout`（默认 0.3 s）
   - `onPose()` 记录位姿自带的 `stamp`（us）；`publishTick()` 增加守护：
     `now() - last_stamp_change_ > stamp_timeout_` → 打 error 并**停发 EV**
   - 上游没给时间戳（stamp==0）时退回"按到达时间"判断，不影响别的用法
   - 效果：一旦停发，EKF2 会在 2×`EV_MAX_INTERVAL`(400 ms) 后关闭 EV 融合 →
     `local_position_invalid` → 飞控进入位置失效 failsafe（Hold/Land）
3. **`px4_ws/src/carto_pkg/launch/fly_carto.launch.py`**
   - `cartographer_node` 与 `cartographer_occupancy_grid_node` 都加
     `respawn=True, respawn_delay=2.0`（原来崩了就永久没位姿）

### 为什么（对应 2026-10-08 定点右飘事故）

cartographer 在起飞瞬间抛 `std::length_error` 崩掉 → `tf_to_pose` 用
`lookup_transform(..., Time())`（= 取"缓冲区最新一帧"）继续拿到**最后一帧旧变换**并以
30 Hz 无限重发 → 新守护把这种"消息在到、内容冻住"的情况识别出来了。

### 验证（2026-10-08 17:36-17:40 实测）

| 检查 | 结果 |
| --- | --- |
| `colcon build --packages-select carto_pkg px4_ev_bridge` | ✅ 0.33 s / 32.5 s |
| `python3 -m py_compile` 两个 py 文件 | ✅ 语法 OK |
| 板子上新代码 | ✅ `tf_to_pose` 防冻结段 2 处、launch `respawn=` 2 行、EV 桥 `stamp_timeout` 3 处 |
| 编译产物 | ✅ `build/px4_ev_bridge/px4_ev_bridge_node` 10-08 17:36 |
| 整链重启后 | ✅ 进程齐全、`/tracked_pose` 时间戳在推进、EV 话题 30.0 Hz、**飞控 `xy_valid=true`** |
| 安装目录 launch | ✅ 是符号链接，指向源文件，`respawn=` 已生效 |

> **冻结测试已于 2026-10-08 17:55 执行并通过**（见本节末尾「冻结测试结果」）。

### 回滚

- 板子上：`~/tf_to_pose.py.bak-20261008`、
  `~/px4_ws/src/carto_pkg/launch/fly_carto.launch.py.bak-20261008`、
  `~/px4_ws/src/px4_ev_bridge/src/px4_ev_bridge_node.cpp.bak-20261008`
  （覆盖回去后 `colcon build --packages-select carto_pkg px4_ev_bridge` + 重启服务）
- 本机：对这三个文件反向 apply_patch

### 遗留问题（本次未解决）

1. ~~`std::length_error` 的根因没查~~ → **已于当日查清并修复，见下面的「修复记录 #5」**。
   当时的猜测（2D 激光倾斜时给出异常扫描）**是错的**：本驱动把无效点填成 `inf`
   （`node.cpp:139`），`fly.lua` 的 `max_range=8` 也会滤掉，扫描数据是干净的。
2. ~~respawn 的副作用~~ → 已在 #5 里补了"跳变后静默期"处理。
3. ~~解锁后自旋（偏航）问题~~ → **用户 2026-10-08 确认已解决**。
4. ~~`ros2 node list` 偶发不列 `/cartographer_node`~~ → 复查两次都正常显示，
   判断是重启后 ROS 发现层的瞬态显示问题，功能无影响。
   **附带发现自己的排查陷阱**：`pgrep -x cartographer_node` 永远匹配不上
   （Linux `comm` 截断到 15 字符："cartographer_no"），**必须用 `pgrep -af` 匹配完整命令行**。
   同理 `pgrep -x px4_ev_bridge_node` 也永远返回空。

## 2026-10-08 修复记录 #5：**cartographer 崩溃的真正根因 = 开机时 RTC 时间不准 → NTP 校正造成时间跳变**

### 真根因（证据确凿）

```
journalctl --list-boots:
  0  b7712d68...  Wed 2026-10-07 21:47:31 CST — Thu 2026-10-08 17:40:28 CST
uptime -s  →  2026-10-08 16:28:12        # 系统实际是 16:28 开机的
```

**同一次开机，日志开头却记成 10-07 21:47:31** —— 说明香橙派开机时 RTC 时间停留在
上次关机时刻，**慢了 18 小时 41 分**。整条链路（含 cartographer）用这个错误时钟启动，
几分钟后 NTP 把时钟校正过来，产生一次 **~19 小时的时间跳变**；
cartographer 遇到巨大时间跳变时抛 `std::length_error` 崩掉
（它的日志停在旧时钟 21:51:52，紧跟着就是 terminate）。

时间线上完全吻合：崩溃点正好是用户起飞那一刻（16:33:48），纯属"开机后约 5 分钟 NTP 才同步"的巧合。

### 改了什么

1. **`px4_ws/run.sh`**：`start_fly()` 新增 **[0/6] 等待系统时间同步** ——
   轮询 `timedatectl show -p NTPSynchronized --value`，最多等 60 秒；
   等到才拉起 SLAM，超时也继续但打警告（避免没网时永远起不来）。
   这样从结构上杜绝"链路运行中途发生时间跳变"。
2. **`px4_ev_bridge`**：新增参数 `jump_pause_sec`（默认 1.0 s）。
   检测到位姿跳变（> `reset_jump_m`）时，除原有的 `reset_counter` +1 之外，
   再**暂停发布 EV 1 秒**，让 EKF2 先超时关掉 EV 融合、再以干净的
   `reset_counter` 重新接入，避免两段不连续的位姿被混在一起（对应 respawn 后新地图的跳变）。

### 验证（2026-10-08 17:48 实测）

```
journalctl -u px4-ev-chain:  [0/6] 等待系统时间同步… → 时间已同步（等待 0s） ✓
/scan 15.042 Hz  /tracked_pose 30.039 Hz  /fmu/in/vehicle_visual_odometry 29.994 Hz
飞控端 vehicle_local_position: xy_valid=True  z_valid=True
timedatectl: System clock synchronized: yes / NTP service: active / RTC time 正确
ros2 topic info /scan -v → Subscription: cartographer_node ✓
```

### 仍未做 / 待观察

- **RTC 电池**：这次 RTC 是旧的，但 `timedatectl` 现在显示 RTC 已正确。
  若再次出现"开机时间停在上次关机时刻"，应检查板载 hym8563 RTC 的纽扣电池。
- ~~两道守护只做了静态检查、没有动态验证~~ → **已于 10-08 17:55 完成 SIGSTOP 冻结测试，
  端到端通过**，详见本节末尾。
- `std::length_error` 是 cartographer 的固有脆弱点：只要运行中发生时间跳变就可能复现。
  [0/6] 只保证"启动时时钟已同步"，运行中若再发生大跳变仍会崩（此时由 respawn + 两道守护兜住）。

### 冻结测试结果（2026-10-08 17:55，动态验证，用户授权执行）

方法：`kill -STOP` 冻住 `cartographer_node`（进程还在、但完全停止产出 —— 正是上次事故的形态），
观察下游与飞控反应，再 `kill -CONT` 恢复。测试前先确认 `arming_state=1`（未解锁）才动手。

| 阶段 | `/tracked_pose` | `/fmu/in/vehicle_visual_odometry` | 飞控 `dead_reckoning` | 飞控 `xy_valid` |
| --- | --- | --- | --- | --- |
| A 基线 | 29.999 Hz | 29.970 Hz | False | — |
| B 冻住上游 | **已停发** | **已停发** | **True** | **False** |
| C SIGCONT 恢复 | 29.965 Hz | 29.999 Hz | **False** | **True** |

关键日志：

```
[tf_to_pose]   [ERROR] 上游 TF 已 0.3s 没有更新（cartographer 挂了？）—— 停止发布位姿，让飞控进入位置失效保护
[px4_ev_bridge][WARN]  位姿已过期，暂停发 EV（等待 SLAM 恢复）
[px4_ev_bridge][INFO]  EV: NED pos(...) yaw 90.1° | 高度有效      ← SIGCONT 后自动续上
```

**结论**：两道守护（tf_to_pose 的时间戳防冻结、EV 桥的守护）均在 0.3 s 内触发并停发；
飞控在 ~15 s 内 `dead_reckoning=True`、`xy_valid=False`（进入位置失效保护方向）；
恢复后全链路自动续上，无需人工干预。**"上游挂死 → 飞控被喂冻结位姿 → 自由飞走"这条路径已被切断。**

**注意（重要，修正早先说法）**：EV 停发后**快速变化的是 `dead_reckoning`**；
`xy_valid` 要等 EKF2 自己的 dead-reckoning 超时（本次实测十秒级）才翻 false。
也就是说停发 EV 后到飞控真正进入 failsafe 之间有**约十几秒的惯性推算窗口** —— 这段时间飞机会按
惯导漂移。所以守护是"防止灾难性飞走"，不是"立刻让飞机停住"。

**本次未覆盖**：`jump_pause_sec`（跳变后静默期）未被触发 —— 恢复时位姿只跳了约 0.1 m，
没超过 `reset_jump_m`(1 m)。该项仍只有静态检查。

## 2026-10-08 诊断：**2D SLAM 对机身倾斜敏感** → 疑似"定点模式往右飘"的机制

### 现象与试验

用户把无人机**拿在手里**做动作，看 EV/SLAM 输出（地面、不装桨、飞机断电状态由用户确认）。

| 手持动作 | EV 记录到的变化 |
| --- | --- |
| 前移约 80cm + 原地转 180°（含抬起） | y 从 0.01 → **+1.32m**，yaw 90.0° → **-121°**；放下后回到 (0.06,-0.07,-0.05) |
| 平移 + **倾斜约 30°**（含抬起 0.72m） | **yaw -121.9° → -92.7°（摆了 +29°）→ -127.4° → -122.9°**；y 摆到 **+1.26m** 后回到 0.02；z 如实跟随抬升/放下（-0.05↔-0.72） |

### 关键推断

1. **平移和高度是可信的**：z 完全跟得上抬升/放下；平移幅度也被记录（回位后能回到原点）。
2. **但倾斜会污染航向估计**：第二次试验里机身只是"倾斜约 30°"，
   **纯倾斜（横滚/俯仰）在物理上不该改变航向**，而 SLAM 报出的 yaw 摆了 **+29°**
   （-121.9° → -92.7°），恢复水平后又回到 -118° 附近。
   同时水平位置也被带着摆了约 1m。
3. 物理原因：2D 激光的**扫描平面跟着机身一起倾斜**，扫到的不再是建图时的那个水平切片，
   scan matching 会匹配到错误的角度/位置。

### 为什么这能解释飞行中的"往右飘、停不下来"

飞行中每次加速/修正都会让机身倾斜 10~30° → SLAM 的**航向和位置估计被打偏** →
位置环用的"前后左右"参考系歪了、位置反馈也歪了 → 飞机朝一个方向持续漂移，
而飞控自己"以为一切正常"（`xy_valid` 仍为 true）。

### 还没定论 / 待做

- 上面第二次试验里"平移"的实际距离未核实。若实际只平移了 30~50cm，
  那 +1.26m 就是**倾斜造成的假位移**（更严重）；若确实搬了 1.3m，则平移是准的。
- **需要一次联合试验**：同时记录**飞控自己的姿态**（`/fmu/out/vehicle_attitude` 的
  roll/pitch，来自 IMU，是独立真值）和 **EV 的 yaw/位置**。让用户把机身倾斜固定角度并保持，
  就能量化"倾斜 X° → SLAM 航向错 Y°"。这才是干净证据。

### 对策（按优先级）

1. **限制飞行倾斜**：`MPC_TILTMAX_AIR` 默认 45° → 降到 **15~20°**（QGC 改，我碰不到飞控）。
   倾斜小 → 扫描平面变化小 → 估计稳。室内首飞代价可忽略。
2. **考虑恢复磁罗盘做航向**：`EKF2_EV_CTRL` 改成 **7**（去掉 EV 航向 bit3），
   `SYS_HAS_MAG=1` + `EKF2_MAG_TYPE=0/1` → 航向由磁力计给（不受激光倾斜影响），
   EV 只负责位置。缺点：室内磁干扰需实测。
3. **给 cartographer 喂 IMU**（`use_imu_data = true` + 把飞控 IMU 转成 sensor_msgs/Imu）——
   治本，但工作量大，作为后续项。

## 2026-10-08 重大修复 #6：**cartographer 的 motion_filter 从未被覆盖 → 位姿只有 1.5Hz**

> 用户提示：「细读代码自己看看是不是哪里有漏洞或者是哪里的通信协议之间没导通」
> —— 确实是一处**配置契约未打通**：SLAM 的输出率与 EKF 需要的输入率不匹配。

### 发现过程（量化证据）

用已录制的 30Hz EV 原始数据做统计（`/tmp/probe_ev.log`，10061 条消息 / 约 5 分钟）：

```
连续重复次数分布： 10045 组 × 连续重复 20 次
→ EV 位姿的**真实刷新率只有 30/20 = 1.5 Hz**
（20 条消息的 position 是 bit 级完全相同的）
```

### 根因

`carto_pkg/configuration_files/trajectory_builder_2d.lua`（cartographer 原版默认）：

```lua
motion_filter = {
  max_time_seconds = 5.,
  max_distance_meters = 0.2,
  max_angle_radians = math.rad(1.),
}
```

**`motion_filter` 是"位姿输出"的总闸门**（只有过了 5s / 移动>0.2m / 转动>1° 才产生新位姿），
而 `fly.lua` **只覆盖了 min_range/max_range/num_range_data 等，从没覆盖 motion_filter**
→ 位姿输出率被压到约 1.5 Hz。

后果有三层：

1. **位置反馈最多滞后 ~0.67 秒，而且是台阶** → 位置环结构性地控不住：
   飞机带着固有倾斜偏差慢慢飘，飞控纠得太晚 → 表现为"定点模式一直往一边飘、停不下来"。
2. **同一个位置被复读 20 次**（桥按 30Hz 发）→ EKF2 把每次当独立测量 →
   位置置信度虚增 20 倍 → 死信过期位置、速度估计被带歪。
3. EKF2 要求 EV 间隔 < 200ms（>5Hz），原本只是**靠复读"骗过"**这条检查，
   真正的新信息只有 1.5Hz。

### 改动

1. **`fly.lua`**（cartographer 配置）
   ```lua
   TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds  = 0.1
   TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
   TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians   = 0.002
   TRAJECTORY_BUILDER_2D.submaps.num_range_data = 150   -- 原 35，避免 submap 过密
   POSE_GRAPH.optimize_every_n_nodes = 150              -- 原 35，避免优化过频
   ```
2. **`px4_ev_bridge`**：默认发布率 `30.0 → 15.0` Hz（与上游真实率一致，避免复读）。

### 验收（2026-10-08 20:52 实测）

| 指标 | 改之前 | 改之后 |
| --- | --- | --- |
| EV 位姿真实刷新率 | **1.5 Hz**（每 20 条完全相同） | **14.6 Hz**（658 条**零重复**） |
| EV 话题频率 | 30 Hz（其中 95% 是复读） | 14.99 Hz（基本 1:1） |
| cartographer 扫描率 | 15.09 Hz / 100% real time | **15.07 Hz / 100.00% real time** ✓ |
| cartographer CPU | — | 53%（单核）、负载 1.18、温度 48℃ ✓ |
| 飞控 `xy_valid` | true | true ✓ |

### 回滚

- `fly.lua`：删掉 motion_filter 三行并把 num_range_data/optimize_every_n_nodes 改回 35
- `px4_ev_bridge`：`publish_rate_hz` 默认改回 30.0 后重编译
- 板子上 `fly_carto.launch.py.bak-20261008` 等备份仍在

### 仍未确证 / 待办

- 这是**目前最强的"往右飘"解释**，但还没在真机飞行中验证过（需要下次飞行观察）。
- `EKF2_EV_DELAY` 仍是默认 0：位姿率提高后影响变小，但实测延迟后仍应设置。
- 倾斜敏感（见上一节）依然存在，`MPC_TILTMAX_AIR` 限倾斜的建议仍然有效。

## 2026-10-08 第二次飞行：仍然飞走 19 米 —— 航向估计空中崩掉

**飞行记录（EV 侧，21:04:40-21:05:05）**：

```
21:04:40  (0.10,  0.06, -0.17)  yaw  90.1°    起飞
21:04:45  (6.24, -0.06, -0.96)  yaw -13.7°    5 秒内位置跑 6m，航向跳 -104°
21:04:50  (12.01, 0.32, -1.01)  yaw -149.2°   又跳 -135°
21:04:55  (14.22,-2.38, -1.03)  yaw -130.0°
21:05:00  (16.29,-6.04, -0.85)  yaw -118.2°
21:05:05  (17.63,-7.39, -0.04)  yaw  -79.2°   落地，总位移 ~19m
```

**判据**：yaw 序列**非单调**（-104°、-135°、+19°、+12°、+39°），真实机体转动不可能这样
→ 结论是 **SLAM 的航向估计在空中崩掉**，不是飞机真转。
航向错 → 位置环把位置误差旋转到错误方向 → 正反馈 → 飞走；推杆也按错误航向解释 → 拉不回来。

### 责任划分（用户直接提问：飞控端还是香橙派）

| 问题 | 在哪一侧 | 性质 | 状态 |
| --- | --- | --- | --- |
| `motion_filter` 未覆盖 → 位姿只有 1.5Hz | 香橙派（配置） | 真 bug | ✅ 已修（实测 14.6Hz） |
| `tf_to_pose` 无限重发冻结位姿 | 香橙派（代码） | 真 bug | ✅ 已修（含冻结测试） |
| 开机 RTC 时间跳变 → cartographer 崩 | 香橙派（系统） | 真 bug | ✅ 已修（[0/6] 等 NTP） |
| 崩了不自愈 | 香橙派（服务） | 缺陷 | ✅ 已修（respawn） |
| 2D 激光倾斜时航向必崩（实测 30°→29°） | 传感器物理 + SLAM | **固有局限** | ⚠️ 只能缓解 |
| EV 航向是唯一航向源、却高权重且**无创新门限** | **飞控参数** | **配置失衡** | ❌ 待改 |
| 磁力计被关（业务要求） | 飞控参数 | 约束 | 需接受 |
| `EKF2_EV_DELAY` 未标定 | 飞控参数 | 待优化 | 待实测 |

**判断**：触发方是香橙派（SLAM 输出错误航向），但**香橙派侧的硬 bug 已全部修完**，
剩下的是 2D SLAM 的物理局限；而把它放大成"飞走 19 米"的是飞控侧的配置失衡
（唯一航向源被高权重信任、磁力计又关着）。**下一步该动的杠杆在飞控参数上。**

### 建议的飞控参数改动（磁力计保持关闭的前提下；QGC 里改）

| 参数 | 现在 | 改成 | 依据 |
| --- | --- | --- | --- |
| `EKF2_EVA_NOISE` | 0.1 rad (5.7°) | **0.3 ~ 0.5 rad** | EV 航向噪声下限；且 EV 航向**没有创新门限**（只有 `EKF2_EVV_GATE`/`EKF2_EVP_GATE`）→ 提高它可让陀螺主导航向 |
| `EKF2_EVP_NOISE` | 0.1 m | 0.2 ~ 0.3 m | 位置更依赖 IMU，削弱 SLAM 台阶/抖动 |
| `EKF2_EV_CTRL` | 15 | **保持 15** | **不可改成 7**：`ev_pos_control.cpp` 里 EV 位置的坐标系对齐依赖 `yaw_align`，去掉 EV 航向会导致位置反馈被转一个未知角度 |
| `MPC_TILTMAX_AIR` | 20（用户已改） | 20 | 倾斜小 → 激光平面变化小 |

增益估算：EV 航向方差 R=(5.7°)² 时，100° 跳变会带走约 22° 航向；
改成 R=(17°)² 后只带走约 3°，航向实际由 100Hz 陀螺积分主导
（陀螺零偏校准后约 0.01~0.05°/s，30 秒漂 1~2°）。

### 尚未证实 / 流程短板

- "航向崩掉导致飞走"是**强推断**，不是铁证：**缺飞控侧 ULog**（飞行全程的姿态/位置/模式/输出都没有记录）。
  → 强烈建议给飞控插 SD 卡并确认 QGC 能下载日志。
- 预热时间：**20 秒足够**（雷达 15Hz → 300 帧），此前"30~60 秒"的说法已作废。

---

## 2026-10-07 —— 打通「雷达+定高器 → PX4 垂直起降」链路

**改了什么**
- 新增 3 个 ROS 2 包（本机 `D:\px4无人机\px4_ws\src\` 与香橙派 `~/px4_ws/src/` 各一份，均编译通过）：
  - `px4_takeoff_land` —— 原地起飞/悬停/降落任务状态机（Offboard）
  - `px4_ev_bridge` (v0.2) —— 雷达水平位姿 + 定高器垂直高度 → `vehicle_visual_odometry`
  - `tofsense_m` —— TOFSense-M 定高器驱动（NLink 协议，921600）
- 新增香橙派脚本 `~/tf_to_pose.py` —— 把 Cartographer 的 TF(`map→base_link`) 转成 `/tracked_pose`
  （这版 cartographer_ros 没有 `publish_tracked_pose` 参数，不会自己发位姿）
- `carto_pkg/launch/fly_carto.launch.py` 加了 `publish_tracked_pose: True`（实测**无效**，保留无害）
- 香橙派 DTB：**只启用 uart1/3/6/9**（`rk3588-orangepi-5-max.dtb`）
- 香橙派用户加入 `dialout` 组
- 装 Micro-XRCE-DDS-Agent（源码 superbuild，装到 `/usr/local/bin`）
- 飞控（MicoAir743 v2.0）QGC 参数：见下

**为什么**
- 目标：让 PX4 用雷达 SLAM 提供水平位置/速度、定高器提供高度，做原地起飞降落
- PX4 的 uXRCE-DDS 输入里**没有 distance_sensor**，定高器接在香橙派上时只能把高度
  塞进 `vehicle_visual_odometry` 的垂直分量 → `EKF2_EV_CTRL=15` + `EKF2_HGT_REF=3`

**关键结论（务必记住）**
1. **Agent 必须跑在香橙派 `/dev/ttyS6`** —— 飞控那条线（UA5/TELEM3）**双向都通**；
   另一条 `/dev/ttyS1`（UA1）**只通一半**（收得到、发不回），是之前握手失败的根因
2. **uart4/5/8 绝不能启用**：uart5 打掉 HDMI、uart4 打掉 PWM 风扇、uart8 打掉 SPI flash
3. DTB 改动后建议**断电冷启动**：热重启会让有线网卡 `enP3p49s0` 消失（雷达就断了）
4. `pkill -f <名字>` 会误杀自己的 shell（命令行里含同样字符串）

**怎么验证**
- `/fmu/out/vehicle_local_position` 的 `xy_valid / z_valid / v_xy_valid / v_z_valid` 四个**全为 true**
- Agent 日志出现 `session established`；`ros2 topic list | grep fmu` 有完整话题表
- `/tofsense/height` 有读数；`/tracked_pose` ~30Hz

**怎么回滚**
- DTB：`sudo cp .../rk3588-orangepi-5-max.dtb.bak-before-uart .../rk3588-orangepi-5-max.dtb && sudo reboot`
- Agent：`sudo rm /usr/local/bin/MicroXRCEAgent`（源码留在 `~/Micro-XRCE-DDS-Agent`）
- 代码包：直接删 `~/px4_ws/src/{px4_takeoff_land,px4_ev_bridge,tofsense_m}` 后重新 colcon build
- 用户组：`sudo gpasswd -d orangepi dialout`

**详细记录**：见 `CHANGELOG_2026-10-07_垂直起降链路.md`

---

## 2026-10-07 修复记录 #2：px4_takeoff_land 未上传 + 编译错误

**改了什么**
- `px4_takeoff_land` 包此前**从未上传到香橙派**（早期只传了 `px4_ev_bridge` 和
  `tofsense_m`），导致 `ros2 launch px4_takeoff_land ...` 报
  `package 'px4_takeoff_land' not found`。已补传并编译通过。
- 修编译错误：`now_us()` 被声明成 `const` → const 版 `get_clock()` 返回 const Clock，
  而 rclcpp 的 `Clock::now()` 非 const，编译失败。
  （和 `px4_bridge`、`px4_ev_bridge` 是**同一个坑**，第三次踩到了，代码里已加注释。）

**验证（2026-10-07 实测）**
- `colcon build --packages-select px4_takeoff_land` → Finished [36.0s]
- 跑 `dry_run:=true` 全程通过：
  `四个 *_valid 全 true` → 参考点 x=-0.01 y=0.00 z=-0.05 → `PRESTREAM` → `ARMING`
  → `[dry_run] 本应发送命令 400` → `解锁超时` → `ABORT` → `[dry_run] 本应发送命令 21`
- 说明：末尾的 ABORT **是 dry_run 的预期行为**（dry_run 拦截所有指令，飞机永远不会
  真解锁，8s 后必然解锁超时），不是故障。它恰好证明了状态机的正常路径和异常路径都对。

**回滚**
- 无需回滚；`now_us()` 那处改动是必修项

**约定（用户 2026-10-07 明确要求，务必遵守）**
- **以后每次改动都要补进本文件 `CHANGELOG.md`。**

---

## 2026-10-09 队友改动同步：**网口雷达 → 串口雷达（LDS-50C-E → LDS-50C-R）**

> 本次改动**由队友在香橙派上完成**（2026-10-09 15:58–16:35），本会话只是登机核对、
> 并把结论与文件同步回本机。队友自己的原始记录在 `px4_ws/docs/串口雷达改造.md`（已同步到本地）。

### 改了什么

| | 改造前 | 改造后 |
| --- | --- | --- |
| 雷达 | 蓝海光电 **LDS-50C-E**（网口 / UDP） | 蓝海光电 **LDS-50C-R**（串口） |
| 连接 | 网口 `192.168.158.98:6543`，本机监听 UDP `6668` | 香橙派 **UART4** → `/dev/ttyS4`，921600 8N1 |
| 驱动参数 | `params/udp_lidar.yaml` | `params/uart_lidar.yaml`（`dev_id: -1`、`reversed: false`、`max_dist: 8.0`、开启角度重采样） |
| 设备树 | — | u-boot 追加 overlay `rk3588-uart4-m2.dtbo` 打开 UART4 |
| 启动脚本 | `run.sh` 里 `LIDAR_PARAMS` 写死 udp 版 | 默认 uart 版，可用环境变量 `LIDAR_PARAMS=...` 临时切回网口版 |

- **接线（40pin 排针）**：pin19 = `UART4_M2_TX` → 雷达 RX；pin23 = `UART4_M2_RX` → 雷达 TX；共地。
  pinmux 走 **UART4_M2**（与 `/dev/ttyS1`(27/28)、`ttyS3`(31/33)、`ttyS6`(11/13)、`ttyS7`(蓝牙) 都不冲突）。
  设备树里 `serial4 = serial@feb70000`，默认 `status="disabled"`，这就是改造前 `/dev/ttyS4` 不存在的原因。
- **新增文件**：`px4_ws/docs/串口雷达改造.md`、`px4_ws/tools/enable_ttyS4.sh`
  （幂等：备份 `/etc/default/u-boot` → 追加 overlay → `u-boot-update`）、
  `px4_ws/tools/check_ev.sh`（「雷达→SLAM→EV→飞控」整链路体检，只读）、
  `px4_ws/.gitattributes`、`px4_ws/.gitignore`。
- **顺带修复**：`systemd/microxrce-agent.service` 里 `Environment=XRCE_DEV=/dev/ttyS3`
  写在 `EnvironmentFile=` **之后**，会把 `/etc/default/microxrce-agent` 的 `ttyS6` 顶掉 ——
  而 **ttyS3 是定高器 TOFsense-M 的口**，一旦 `systemctl enable` 这个单元就会踩。
  已改成默认 `ttyS6`，并把 `EnvironmentFile=` 挪到 `Environment=` 之后；单元内注明
  **保持 disabled**（整条链路由 `px4-ev-chain.service → run.sh fly → agent.sh` 拉起，
  两个都启用会抢同一路串口）。
- **`px4_takeoff_land_node.cpp`**：队友那份是修好的版本（`now_us()` 非 `const`），已同步覆盖本机。
- **重要纠正**：本文件 2026-10-07 那条「uart4 绝不能启用」是**当时 GMAC 设备树那一轮**的经验，
  和本次无关 —— 本次是**按官方 overlay 正经打开 UART4 给雷达用**，已实测无害；**uart5 / uart8 依然不要动**。

### 为什么

网口款 LDS-50C-E 被换成串口款 LDS-50C-R（同系列 2D 360°，话题名 `/scan`、`frame_id=laser_link`
都不变），所以雷达侧连接方式整体从 UDP 改成 UART。

### 验证（2026-10-09 实测）

| 项 | 实测 |
| --- | --- |
| u-boot overlay | `/etc/default/u-boot` 的 `U_BOOT_FDT_OVERLAYS` 已含 `device-tree/rockchip/overlay/rk3588-uart4-m2.dtbo`；dmesg 有 `feb70000.serial: ttyS4 at MMIO 0xfeb70000 (irq = 110 ...) is a 16550A` |
| `/dev/ttyS4` | `crw-rw---- 1 root dialout 4, 68` ✅ |
| 串口握手 | `version: V4` / `LSRPM:900H OK`；`LSRES:300H NG`（该固件不支持下发分辨率，驱动在本机重采样，无害） |
| `/scan` | 15.2 Hz，**每帧 1200 点**（雷达原生 0.3°；网口款是 2000 点 / 0.18°） |
| `/tofsense/height` | 55.6 Hz |
| `/tracked_pose` | 30.1 Hz |
| `/fmu/in/vehicle_visual_odometry` | 15.0 Hz |

结论：`carto_pkg / slam_pkg / px4_ev_bridge` **一行都不用改**，串口雷达完整顶替了网口雷达。

**本会话（2026-10-09 17:00 前后）登机复核到的状态**：

- `systemctl is-active px4-ev-chain` = **active**；相关进程都在跑：
  `MicroXRCEAgent serial --dev /dev/ttyS6 -b 921600`、
  `bluesea2_node --params-file .../uart_lidar.yaml`、`cartographer_node`、
  `tofsense_m_node`、`px4_ev_bridge_node`、`python3 ~/tf_to_pose.py`。
- 这些节点在 DDS 端口 7400 上都有参与者（`ss -uapn` 可见），并且输出都在推进：
  cartographer 持续做约束搜索、定高器在出帧（`高度 0.048 m`）、EV 桥每 5 秒一行
  （`EV: NED pos(0.02,-0.01,-0.05) yaw 90.5° | 高度有效`，飞机静止在桌上，数值不跳是正常的）。
- 有线网口 `enP3p49s0`（RTL8125，`lspci` 还在）**当前是 DOWN / NO-CARRIER、没有 IPv4** ——
  雷达已改串口，网口不再需要；`SSH_ACCESS.md` 里那套 `192.168.158.100` 静态配置**暂时用不到**。
- 飞控侧：香橙派上没有 `/fmu/out/*` 话题，说明**飞控当前没上电 / 没接**（Agent 仍在 ttyS6 等着），
  与 10-08 的结论一致，不影响本次雷达改造的验证。

### ⚠️ 本次发现、尚未定性：新会话看不到已有 ROS 话题

**现象**：从**新开的** SSH 会话里 `source env` 后，`ros2 topic list` 只列出自己的
`/parameter_events`、`/rosout`；`ros2 node list` 为空；`ros2 topic hz /scan` /
`ros2 topic echo /tracked_pose` 都报 "does not appear to be published yet"。

**已排除**：ROS_DOMAIN_ID（链路服务与我的会话都是 0）、`ROS_LOCALHOST_ONLY=0`、
网络命名空间（`/proc/self/ns/net` 与节点进程相同）、daemon 缓存（`ros2 daemon stop/start`、
`--no-daemon`、`--spin-time 20` 都试过）。
**对照组**：同一条会话里自己起的 `ros2 topic pub /codex_probe` 能被自己列出来 ——
说明「自己人互相发现」正常，坏的是「新参与者 ↔ 开机时那批参与者」的发现。

**可疑点**：那批参与者是**开机时建的**，当时 `enP3p49s0` 还是 UP（带 `192.168.158.100`），
之后该网卡变成 DOWN / NO-CARRIER，而 FastDDS 会把各网卡的 locator 缓存下来。

**待办**：下次有机会时 `bash ~/px4_ws/run.sh stop && bash ~/px4_ws/run.sh fly`（或整机重启），
然后立刻在**新会话**里 `ros2 topic list`：
- 若恢复 → 就是「网卡状态变化导致 DDS 发现失效」，以后**不要在飞行链路跑着的时候拔网线 / 改网卡状态**；
- 若不恢复 → 再查 FastDDS 配置（`ROS_AUTOMATIC_DISCOVERY_RANGE` / 自定义 profile）。

> 现状不影响飞行本身（机载链路内部是通的），只影响「人在香橙派上开终端看话题」。

### 回滚

- 系统层（关掉 UART4 overlay）：
  ```bash
  sudo cp -a /etc/default/u-boot.bak-ttyS4 /etc/default/u-boot
  sudo u-boot-update && sudo reboot
  ```
- 只回滚雷达参数（需网口雷达 + `enP3p49s0` 静态 IP 都在）：
  ```bash
  LIDAR_PARAMS=~/px4_ws/src/bluesea2/src/bluesea-ros2/params/udp_lidar.yaml bash ~/px4_ws/run.sh lidar
  ```
- 本机文件：本次同步**覆盖**了 `px4_ws/README.md`、`px4_ws/run.sh`、
  `src/bluesea2/src/bluesea-ros2/params/{uart_lidar,udp_lidar}.yaml`、
  `src/carto_pkg/launch/fly_carto.launch.py`、
  `src/px4_takeoff_land/src/px4_takeoff_land_node.cpp`、
  `systemd/microxrce-agent.service`，**新增**了
  `px4_ws/{docs/串口雷达改造.md,tools/enable_ttyS4.sh,tools/check_ev.sh,.gitattributes,.gitignore}`。
  香橙派侧改动前的原始文件在 `~/px4_ws/.backup_serial_radar_20261009_160120/`。
- `microxrce-agent.service` 若想恢复旧值：把 `Environment=XRCE_DEV=/dev/ttyS6` 改回 `/dev/ttyS3`
  （**不建议**：那会抢定高器的串口）。

---

## 2026-10-09 分析：**D 题（自研 STM32）与本项目（PX4）的定位链路对比** —— "起飞后一直往右偏"的架构性根因

> 用户提问：「起飞之后一直往右偏，你之前说是速度环有问题，再通读一下 D 题那套融合比较好的上位机代码对比一下；
> 另外 PX4 飞控收到的不是雷达给的速度和位置数据，而是经过香橙派处理过的数据，是这样吗？」
> 本节只做对比与结论，**没有改动任何代码或飞控参数**。

### 1. 现在 PX4 到底收到什么（逐行核对 `px4_ev_bridge_node.cpp`）

香橙派发给飞控的只有一帧 `vehicle_visual_odometry`（`/fmu/in/vehicle_visual_odometry`）：

| EV 字段 | 真实来源 | 备注 |
| --- | --- | --- |
| position x/y | cartographer 的 SLAM 位姿（`/scan` → TF → `tf_to_pose.py` → `/tracked_pose`） | ENU→NED：`x_ned=y_enu, y_ned=x_enu` |
| position z | **TOFsense-M 定高器** | `z_ned=-(height+offset)`，**跟雷达无关** |
| velocity x/y | **没有任何传感器测它** —— EV 桥对 SLAM 位置做差分 + 一阶低通（`vel_filter_alpha=0.7`，约 0.2 s 滞后） | 却按 `velocity_variance=0.01`（σ=0.10 m/s）报出去 |
| orientation | SLAM 的 yaw（`π/2 − yaw_enu`），roll/pitch 恒 0 | EKF2 只取 yaw |

**结论：PX4 从来没有见过雷达数据**，连 cartographer 的原始输出也没见过 —— 它拿到的是
"SLAM → tf_to_pose → EV 桥"三道加工后的结果，而且是当作**测量值**喂进 EKF2 的。

### 2. D 题（能飞稳）的链路

- `pid_control_pkg`：**50 Hz** 位置环（`control_frequency: 50`），用 TF 取 SLAM 位姿 → 算误差(cm) →
  输出 `/target_velocity` = `[vx, vy, vz cm/s, vyaw deg/s]`，**地图/世界系**。
- `uart_to_stm32`：把地图系速度用 `Rz(yaw)` **旋转到机体系**，再通过 `0x31` 帧发 cm/s
  （`vel_x` = 头向速度、`vel_y` = 左向速度）给 STM32。
- 飞控收到的**只有速度指令**；姿态/速度内环完全靠自家 IMU。
  **SLAM 只决定"往哪飞"，不参与飞控的状态估计。**
- （D 题自己也踩过这个坑：`targetVelocityCallback()` 里那段注释，就是"世界系当机体系 →
  绕着目标点画圈 / 冲进货架"的修复记录。）

### 3. 为什么同一套 SLAM，D 题能用、PX4 会飞走

| | D 题 | 本项目（PX4） |
| --- | --- | --- |
| 闭环在哪 | 上位机 50 Hz 位置环 | 飞控（EKF2 + 位置控制器） |
| SLAM 的角色 | **指令来源** | **状态估计的测量输入** |
| 航向来源 | 飞控自家 IMU（SLAM yaw 只用来旋转指令） | **SLAM yaw 直接融进 EKF**（`EKF2_EV_CTRL=15` 含 yaw bit） |
| SLAM 出错时 | 方向飞偏，但姿态稳、可控 | 飞控自己的航向/位置估计被污染 → 位置环朝错方向修 → 正反馈飞走 |

正好对上 10-08 实测：空中 yaw 序列 `90° → -13.7° → -149.2°` **非单调跳变**、19 m 飞走。

### 4. "速度环有问题"这句话的修正

- 上一轮找到的 `motion_filter` 未覆盖（位姿其实只有 1.5 Hz）**确实是真 bug**，已修到约 15 Hz ——
  但它只解释了"反馈迟 + 台阶"，不是全部。
- 这次通读代码后，真正结构性的是**把"差分出来的速度"当成速度测量**：
  1. 它是 `Δpos/Δt` 再经 α=0.7 低通（约 0.2 s 滞后），不是任何传感器测出来的；
  2. 权重还给得高：EKF2 取 `max(EKF2_EVV_NOISE², 报文方差)`
     （`EKF2.cpp:2216-2223`；参数说明原文 "lower bound or replace the uncertainty included
     in the message"），默认 `EKF2_EVV_NOISE=0.1` → 有效就是 **0.1 m/s** 的自信度；
     桥里报的 σ（位置 0.05 m / 速度 0.10 m/s / 航向 2°）**基本都低于飞控参数的底**，
     所以调桥不管用，要调的是飞控参数。
  3. 再叠加 2D 激光的**倾斜污染**（10-08 手持实测：倾斜 30° → yaw 摆 29°、y 摆 1.26 m）→
     倾斜一下就能注入好几 m/s 的假速度。
- 合成机理：**倾斜 → SLAM 报假位移/假航向 → EKF 相信 → 位置环朝错方向修 → 机身更倾斜 → 更假**，
  这就是"起飞后一直往右偏、停不下来"的形态（比单纯"带宽不够"更贴合现象）。

### 5. 建议（飞控参数需用户在 QGC 改；本会话没碰飞控）

| 优先级 | 动作 | 理由 |
| --- | --- | --- |
| ⭐⭐⭐ | `EKF2_EVV_NOISE` 0.1 → **0.5~1.0**（或 `EKF2_EV_CTRL` 15 → **11**，直接去掉速度 bit） | EV 速度是差分的，不该给 0.1 m/s 的信任 |
| ⭐⭐⭐ | `EKF2_EVA_NOISE` 0.1 → **0.3~0.5**（min=0.05） | 让 100 Hz 陀螺主导航向；100° 跳变从带走约 22° 降到约 3° |
| ⭐⭐ | `EKF2_EVP_NOISE` 0.1 → **0.2~0.3** | 削弱 SLAM 位置的台阶/抖动 |
| ⭐⭐ | 标定 `EKF2_EV_DELAY`（现为 0） | TF 链路实测 stamp 偏差 0.3~1.1 s，这条链时延不小 |
| ⭐ | `MPC_TILTMAX_AIR` 保持 20° | 倾斜小 → 激光扫描平面变化小（治标，但有效） |
| ⭐ | 给 cartographer 喂 IMU（`use_imu_data=true`，把飞控 IMU 转成 `sensor_msgs/Imu`） | 治本：2D SLAM 有 IMU 后倾斜污染小一个量级 |
| 备选 | 把位置环搬回香橙派（`pid_control_pkg` → `/target_velocity` → `px4_bridge` 的 Offboard **速度**模式），照 D 题的架构 | 闭环回到 50 Hz、SLAM 跳变不再进飞控估计器；但 PX4 仍要求有效的位置/航向，故上面 ⭐⭐⭐ 两条仍要做 |

> ⚠️ 搬 D 题代码时的坑：`px4_bridge` 期望 `/target_velocity` 是**地图系(ENU)**、自己再做 ENU→NED；
> 而 D 题的 `uart_to_stm32` 是**先转机体系**再发飞控。两者不能叠加，否则会"转两次"。

---

## 2026-10-09 香橙派侧配套改动：**EV 桥上报的"不确定度"与飞控参数对齐**

> 用户指令原话：「参数按你给的改了，你再改香橙派那侧的」

### 改了什么

`px4_ws/src/px4_ev_bridge/launch/ev_bridge.launch.py`（本机改 + 推送香橙派 + 已重启生效）：

| 参数 | 原值 | 现值 | 说明 |
| --- | --- | --- | --- |
| `position_sigma` | 0.05 m（仅节点默认，launch 里没传） | **0.25 m** | SLAM 水平位置有台阶、会被机身倾斜污染，不该按 2.5 cm 精度上报 |
| `velocity_sigma` | 0.10 m/s（仅节点默认） | **1.0 m/s** | ← 本次最关键：EV 的"速度"**没有任何传感器测它**，是 EV 桥对 SLAM 位置做 `Δpos/Δt + α=0.7 低通` 算的。方差 0.01 → 1.0，等于**降权 100 倍**，基本退出融合 |
| `yaw_sigma_deg` | 25.0 | 25.0（保持） | 2026-10-08 已调，让 100 Hz 陀螺主导航向 |

同时把 `position_sigma` / `velocity_sigma` 显式加进 `DeclareLaunchArgument` 并传进节点
（原来这两个只存在于节点内部的默认值里，launch 里看不到，容易被误以为"没配"）。

### 为什么是这一侧说了算（PX4 1.15.4 实测机制）

- `EKF2_EV_NOISE_MD` 默认 **0** = "噪声取报文里的方差，飞控参数只作下限"
  （`module.yaml` 原文 *the measurement noise is taken from the vision message and the EV noise
  parameters are used as a lower bound*；`EKF2.cpp:2216-2223` 就是 `fmaxf(param², msg_var)`）。
- 即 **有效噪声 = max(飞控 EKF2_EVP/EVV/EVA_NOISE, 报文里的 σ)** —— 谁大听谁的。
  原来我们报的 σ(0.05 m / 0.10 m/s / 25°) 里前两个都低于飞控参数底线，**所以调桥没用**；
  现在把桥报大，香橙派这侧就成了"信多少"的决定方。
- **更正一条旧注释**：launch 文件里原先写"`EKF2_EVA_NOISE` 范围只有 0.05~0.1 rad"是**错的** ——
  1.15.4 的 `module.yaml` 里它只有 `min: 0.05`、**没有 max**，所以飞控侧也能调大
  （上一轮建议的 0.3~0.5 rad 是可行的）。

### 验证（2026-10-09 17:36 实测）

```
systemctl is-active px4-ev-chain   → active（ExecMainStatus=0）
进程                               → MicroXRCEAgent(ttyS6) / bluesea2_node(uart_lidar.yaml) /
                                     cartographer_node / tf_to_pose.py / tofsense_m_node /
                                     px4_ev_bridge_node 全部在跑
/scan 15.088 Hz   /tofsense/height 55.680 Hz   /tracked_pose 30.022 Hz
/fmu/in/vehicle_visual_odometry 14.996 Hz
EV 桥实际加载参数（/tmp/launch_params_0macrf8f）:
    position_sigma: 0.25     velocity_sigma: 1.0     yaw_sigma_deg: 25.0
EV 日志：EV: NED pos(0.00, -0.01, -0.05) yaw 90.0° | vel(0.00, 0.00) | 高度有效
```

### 副作用 / 注意

- `EKF2_EVP_NOISE` 是 EV 位置 **x/y/z 三者共用的下限**，所以降权"水平位置"的同时，
  **定高器的高度也被一起降权**（现在等效 ±0.25 m）。若之后发现高度保持变差：
  把 `EVP_NOISE` 调回 0.15 左右，或把高度改走 `distance_sensor`（`EKF2_RNG_*`）单独融合。
- 本次只动了"上报的不确定度"（即 EV 数据的权重），**没有改控制结构**；
  「位置环搬回香橙派、照 D 题的架构」那条备选路径仍未做。

### 回滚

- 香橙派：`~/px4_ws/src/px4_ev_bridge/launch/ev_bridge.launch.py.bak-20261009` 覆盖回去
  + `sudo systemctl restart px4-ev-chain`
- 本机：把 `position_sigma` 改回 0.05、`velocity_sigma` 改回 0.10（或删掉这两行，回到节点默认值）

### 补充：为什么当初要在桥里"自己算"速度（2026-10-09 回答提问）

- **历史原因**：`vehicle_visual_odometry` 这个接口自带 velocity 字段，而 cartographer 只给位姿、
  不给速度；为了让这帧消息"填满"，就补了一路位置差分速度。当时还有个想当然的担心 ——
  "EKF 要有速度源，`xy_valid` 才会 true"。
- **这个担心是多余的（源码证据）**：
  `vehicle_local_position.xy_valid / v_xy_valid` 都来自 `_ekf.local_position_is_valid()`
  = `!_horizontal_deadreckon_time_exceeded && !fake_pos`（`EKF2.cpp:1585-1586`、`EKF/ekf.h:288`）；
  而水平 dead-reckoning 的判定在 `EKF/ekf_helper.cpp:586`：

  ```
  velPosAiding = (gps || ev_pos || ev_vel || aux_gpos)
                 && (最近融合过水平位置 || 最近融合过水平速度);
  ```

  —— **"位置"和"速度"二者有其一就够**。所以只发 EV 位置（`EKF2_EV_CTRL` 去掉 bit2）完全可行。
- **更本质的一点**：位置差分得到的"速度"不是新信息，而是对同一份信息的**重复计票** ——
  它的噪声还会因为做差分而被放大（`Δpos/Δt` 对高频噪声增益 ~2/dt）。把它当独立测量喂进卡尔曼滤波，
  会让滤波器高估速度的置信度（10-08 已经实测过同类的坑：位姿真实率只有 1.5 Hz、被复读 20 次时，
  位置置信度虚增 20 倍）。而**真正的速度本来就在飞控自己的 IMU 里**（100 Hz+、无量化、无传输延迟），
  15 Hz 的位置差分不可能比它好。
- **结论**：源只有位姿时，正确做法是**只报位置**，速度交给飞控的 IMU + 滤波器自己算；
  只有当源本身能给出真实速度（VIO / 光流）时才该报速度。本次把 `velocity_sigma` 抬到 1.0
  是"达到同一效果"的折中；想更干净可以直接把 EV 的 velocity 删掉，或把 `EKF2_EV_CTRL` 15 → 11。

---

## 2026-10-09 机载端大改：**EV 桥删掉假速度 + 把飞控 IMU 接进 cartographer**

> 用户指令原话：「velocity删掉，然后融合imu到香橙派上」

### 改了什么（4 处代码 + 3 个脚本/配置）

| # | 文件 | 改动 |
| --- | --- | --- |
| 1 | `px4_ws/src/px4_ev_bridge/src/px4_ev_bridge_node.cpp` | 删除 `onPose()` 里"位置差分求速度"的整段（含 `vel_filter_alpha` 低通）；`publishEv()` 里速度字段改发 **NaN**、`velocity_variance` 填"不可信"；日志改成 `速度不上报(交给飞控IMU)`。保留 `velocity_topic` 参数 —— 以后真接了 VIO/光流可以从那一路发真速度 |
| 2 | `px4_ws/px4_imu_bridge.py`（**新增**，板上放 `~/px4_imu_bridge.py`） | 订阅飞控 `/fmu/out/sensor_combined`，转成 `sensor_msgs/Imu` 发到 `/imu`；**FRD→FLU**（`gyro/accel` 的 y、z 取反）；时间戳用**收到时刻**（避免 PX4 时间换算误差导致时间倒流，cartographer 对 IMU 时间非单调会直接 CHECK 崩溃）；默认限流 100 Hz |
| 3 | `px4_ws/src/carto_pkg/configuration_files/fly.lua` | `TRAJECTORY_BUILDER_2D.use_imu_data = true` |
| 4 | `px4_ws/src/carto_pkg/configuration_files/fly_no_imu.lua`（**新增**） | `include "fly.lua"` + `use_imu_data = false`（官方式写法，配置只有一个来源） |
| 5 | `px4_ws/src/carto_pkg/launch/fly_carto.launch.py` | remap `imu → /imu` |
| 6 | `px4_ws/run.sh` | `fly` 流程新增 **[2/7] 起 IMU 桥**，步骤重编号为 `x/7`；SLAM 前**探测 `/imu` 有没有数据**，自动选 `fly.lua`（有 IMU）或 `fly_no_imu.lua`（没有）；自检话题表加 `/imu` |
| 7 | `px4_ws/stop.sh` | 停止列表补上 `px4_imu_bridge`，以及以前漏掉的 `px4_ev_bridge_node / tofsense_m_node / tf_to_pose / MicroXRCEAgent` |

### 为什么

1. **删速度**：EV 的"速度"没有任何传感器测它，是拿 SLAM 位置差分 + 低通造出来的；
   它和位置不独立、还会放大量化噪声，却被 EKF2 当成一次独立的"速度测量"融合
   （EKF2.cpp:2167 用 `if (ev_odom_vel.isAllFinite())` 判定，所以**填 NaN 就是干净的"不提供"**）。
   真实速度在飞控自己的 IMU 里，不需要机载电脑代劳。
2. **接 IMU**：2D 激光的扫描平面跟着机身倾斜 → scan matching 匹配到错误切片 → 航向/位置被带偏
   （10-08 实测：倾斜 30° → yaw 摆 29°、y 摆 1.26 m）。陀螺的角速度不受扫描平面倾斜影响，
   接进来后旋转预测由陀螺提供，scan matching 只在很小的角度窗口里找。
   IMU 来源用飞控的 `/fmu/out/sensor_combined`（**该话题在 PX4 1.15.4 默认 DDS 发布列表里，
   不需要改固件**）。

### ⚠️ 过程中踩到、必须记住的坑

**cartographer 一旦开 `use_imu_data = true`，就"没有 IMU 就不干活"**：
它的多队列按时间排序，会一直刷
```
W ordered_multi_queue.cc:155] Queue waiting for data: (0, imu)
```
而且**一个位姿都不输出**（实测：飞控没上电 → `/tracked_pose` 完全没有数据 → EV 桥也跟着
`还没收到 /tracked_pose 的位姿`）。所以：

- **飞控不在时绝不能开**。这就是 `fly_no_imu.lua` 存在的原因，`run.sh fly` 会自动二选一；
- 反过来，**飞行中 IMU 流断了，cartographer 也会停出位姿** → EV 超时 → 飞控进入位置失效保护
  （这个失败方向是安全的，但要知道会发生）。

### 验证（2026-10-09 18:00 左右实测）

| 检查 | 结果 |
| --- | --- |
| 编译 | `colcon build --symlink-install --packages-select px4_ev_bridge` → Finished [33.3s]；`carto_pkg` → 0.95s（新 lua 需要重编才会进 install 目录） |
| 语法 | `bash -n run.sh / stop.sh` ✔、两个 launch 与 IMU 桥 `py_compile` ✔ |
| 配置选择（飞控**没**上电） | 日志 `[提示] /imu 没数据（飞控没上电？）→ 用 fly_no_imu.lua` ✔；进程参数确认是 `-configuration_basename fly_no_imu.lua` |
| `Queue waiting for data` | slam 日志里 **0 次** ✔（退回无 IMU 配置后不再阻塞） |
| 关键话题 | `/scan` 15.17 Hz、`/tracked_pose` 29.99 Hz、`/tofsense/height` 55.0 Hz、`/fmu/in/vehicle_visual_odometry` 15.0 Hz ✔ |
| EV 桥日志 | `EV: NED pos(...) yaw 90.0° \| 速度不上报(交给飞控IMU) \| 高度有效` ✔ |
| IMU 桥（飞控没上电） | 每 5 s 一条 `还没收到 /fmu/out/sensor_combined —— 飞控上电了吗？` ✔（预期行为） |
| **IMU 转换隔离自检** | 用另一个实例（`input_topic:=/test/imu_in`、`output_topic:=/test/imu`，不干扰正式链路）注入假数据：<br>FRD `gyro=(0.1,0.2,0.3) accel=(1.0,2.0,-9.81)` → 输出 FLU `angular_velocity=(0.1,-0.2,-0.3)`、`linear_acceleration=(1.0,-2.0,+9.81)`、`frame_id=imu_link` ✔ **完全正确** |

### ⏳ 飞控上电后仍需验证（本次做不到，飞控不在）

1. `ros2 topic hz /imu` → 期望 ~100 Hz（`publish_hz` 上限）；
2. `bash ~/px4_ws/run.sh status` 或重启服务，确认日志选了 `fly.lua`（有 IMU）；
3. **倾斜试验**：手拿着把机身倾斜 20~30°，同时看 `/tracked_pose` 的 yaw —— 接 IMU 后
   应该不再像 10-08 那样摆 29°；
4. 顺带确认 `/fmu/in/vehicle_visual_odometry` 里 `velocity` 是 nan（飞控侧 `listener vehicle_odometry`
   或看 ULog），且 `xy_valid / v_xy_valid` 仍为 true。

### 回滚

- 香橙派上（都留了 `.bak-20261009`）：`run.sh`、`stop.sh`、
  `src/px4_ev_bridge/src/px4_ev_bridge_node.cpp`、`src/carto_pkg/configuration_files/fly.lua`、
  `src/carto_pkg/launch/fly_carto.launch.py`；`ev_bridge.launch.py.bak-20261009` 是**今早那版**
  （σ 对齐之前的原始版本）。
  ```bash
  cd ~/px4_ws && for f in run.sh stop.sh src/px4_ev_bridge/src/px4_ev_bridge_node.cpp \
      src/carto_pkg/configuration_files/fly.lua src/carto_pkg/launch/fly_carto.launch.py; do
      cp -a $f.bak-20261009 $f; done
  colcon build --symlink-install --packages-select px4_ev_bridge carto_pkg
  sudo systemctl restart px4-ev-chain
  ```
- 只回滚 IMU（保留"删速度"）：把 `fly.lua` 的 `use_imu_data` 改回 `false`；
  `run.sh` 里 `fly` 流程会继续自动选（或者干脆用 `fly_no_imu.lua`）。
- 只回滚"删速度"：`ev_bridge_node.cpp.bak-20261009` 覆盖回去重编。

---

## 2026-10-09 19:00 现场小事故：**雷达驱动报 `Failure to modify serial port baud rate` → 一个位姿都没有**

**现象**：17:58 那次重启（不是本会话发起的）之后，
`/tmp/px4_ws_lidar.log` 只打印到
```
[INFO] [UartThreadProc 594]: Connection info:uartname /dev/ttyS4 baudrate 921600
[ERROR] [open_serial_port 269]: code:-106 value:Failure to modify serial port baud rate
```
就再没有任何输出（没有 `LVERSH` 握手、没有 `lidar start work`），`/scan` 无数据 →
`/tracked_pose` 无数据 → EV 桥一直报"还没收到位姿"。服务本身显示 `active`（oneshot 不监控子进程），
**外表看不出来**。

**处理（已恢复）**：
```bash
sudo systemctl stop px4-ev-chain        # ExecStop 会跑 run.sh stop
# 确认没有残留进程、也没有别的程序占着 /dev/ttyS4
sudo systemctl start px4-ev-chain
```
重启后握手正常（`LSRPM:900H OK` → `lidar start work`），`/scan` ~15 Hz、`/tracked_pose` ~30 Hz。
`sudo stty -F /dev/ttyS4 921600` 单独试是**成功**的，说明串口本身没坏。

**怀疑原因**：`run.sh start_lidar()` 是 `pkill -f bluesea2_node` → `sleep 1` → 起新实例，
旧实例可能还没完全释放串口（或那一刻被别人打开着），新实例的 `tcsetattr` 就失败了。
**待办**：把 `start_lidar` 的等待改成"轮询 `/dev/ttyS4` 确实没人占用再起"，
或者失败时自动重试一次。

**排查提示（以后再遇到）**：服务 `active` ≠ 链路正常。看 `/tmp/px4_ws_*.log` 的**时间戳有没有在推进**，
以及 `/scan` 有没有频率 —— 这两个才是判据。

---

## 2026-10-09 晚（20:00–20:30）：**队友第二轮改动 —— 定点遥控垂直飞行已可用**

> 用户原话：「队友又进行了改动，这次能够比较正常的进行定点遥控垂直飞行了，就是开始的时候会
> 往前冲一下然后垂直起飞，你看看改动日志并同步一下」

### 队友改了什么（19:29 / 19:57，香橙派上；本会话已同步回本机）

| 文件 | 改动 |
| --- | --- |
| `src/px4_ev_bridge/src/px4_ev_bridge_node.cpp` | **恢复上报"位置差分速度"**（推翻我白天那版"改发 NaN"）：新增参数 `report_velocity`（**默认 true**）、`vel_filter_alpha`（0.7）；差分前加了"跳变当帧不参与"保护（`have_prev_ = false`），避免 SLAM 重定位那一帧算出巨大的假速度；日志里速度栏现在有 `有(位置差分)` / `有(外部源)` / `不上报(...)` 三种 |
| `src/px4_ev_bridge/launch/ev_bridge.launch.py` | 新增 `report_velocity:=true`、`vel_filter_alpha:=0.7` 两个 launch 参数并传进节点 |
| `tools/check_ev.sh` | 新增**最关键的一步**：`ros2 topic info /fmu/in/vehicle_visual_odometry` 看 Publisher / Subscription count（**0 = uXRCE-DDS Agent 没在桥接，报文根本没出香橙派**）；原步骤重新编号为 1~9 |
| `tools/fc_link_probe.sh`（**新增**） | 直接探「飞控 ↔ 香橙派」那条串口：临时停掉 MicroXRCEAgent → 按 921600/57600/115200/230400 逐个裸读 10 秒 → 打印收到多少字节 → 自动把 Agent 恢复到 ttyS6。判读：有字节=线通、飞控在发；全 0=飞控没在驱动这根线（没上电 / `UXRCE_DDS_CFG` 不对 / TX-RX 没交叉） |
| `~/.bashrc`（板子上，18:07） | 末尾补 `export ROS_DOMAIN_ID=0`（第 119 行），避免新终端与会话域不一致 |

**关于"速度又回来了"**：队友的注释写明是「按用户要求恢复上报（用户要求飞控能收到速度）」。
现在这版比我白天担心的那版**风险小得多**，因为有两条护栏：①速度方差仍是保守的
`velocity_sigma = 1.0`（EKF 不会像原来 σ=0.1 那样过度信任差分速度）；②跳变当帧不参与差分。
要彻底关掉：`report_velocity:=false`，或飞控侧 `EKF2_EV_CTRL` 去掉 bit2。

### 现在的实测状态（2026-10-09 20:30，本会话复核）

| 项 | 值 |
| --- | --- |
| 飞控 | **已连接**（`/fmu/out/*` 齐全，含 `sensor_combined / vehicle_attitude / estimator_status_flags`） |
| EV 有没有送到飞控 | `Publisher count: 1`、**`Subscription count: 1`** ✔（队友新加的那个判据） |
| EKF 是否真在用 EV | `cs_ev_pos / cs_ev_yaw / cs_ev_hgt / cs_ev_vel` **全 true**；`reject_hor_pos / reject_hor_vel / reject_yaw` 全 false ✔ |
| 飞控本地位置 | `xy_valid / z_valid / v_xy_valid / v_z_valid` **全 true**；`heading = 1.7649 rad (101.1°)`，与 EV 报的 yaw 101.1° 一致 ✔ |
| EV 桥 | 15.0 Hz，`速度有(位置差分)` / `高度有效`；启动 16 秒时触发过一次"时间戳 0.3 s 没推进 → 停发 EV"守护，随后自愈 ✔ |
| `/imu` | **有数据**（≈64~65 Hz，来自飞控 `sensor_combined`） |
| cartographer 实际配置 | ⚠️ **`fly_no_imu.lua`（纯激光）** —— 这次飞行**没用上 IMU**，原因见下 |

### 本会话顺手修掉的 bug（我自己昨晚引入的）

`run.sh` 探 `/imu` 那句写成
```bash
if timeout 6 ros2 topic hz /imu 2>/dev/null | grep -qm1 'average rate'; then
```
在 `set -Eeo pipefail` 下**永远判为失败**：`grep -q` 一匹配上就立刻退出 → `ros2` 收 SIGPIPE
非零退出 → 整条管道非零 → 走 else 分支。结果**明明 `/imu` 有 65 Hz，日志却一直报"没数据"**，
于是永远选 `fly_no_imu.lua`。已改成用变量接住（和自检那段同样的写法），实测探针返回
`average rate: 63.987` ✔。

同时把"用不用 IMU"改成**显式开关**：

- 默认 `fly_no_imu.lua` —— 就是**现在能正常定点起降**的那份配置，行为不变；
- 想试 IMU 旋转预测：`SLAM_USE_IMU=1 bash run.sh fly`，它会先确认 `/imu` 真有数据才切，
  否则退回纯激光并打警告（**因为 `fly.lua` 在没有 IMU 时一个位姿都不出，这是硬约束**）。

> 也就是说：目前"能飞"和"有 IMU 保护"是两条路，**IMU 那条还没在真机上验证过**，
> 建议先在台架上做倾斜试验（手拿机身倾斜 20~30°，看 `/tracked_pose` 的 yaw 是否还乱摆），
> 再决定要不要在飞行时启用。

### "起飞时往前冲一下、然后垂直起飞" —— 现有分析（**待用数据确认，不是定论**）

没有录包、也没有飞控 ULog，只能给机制假设，按可能性排：

1. **起飞瞬间 EV 位置出现"台阶"，位置环去追它**：定点模式要把位置钉在起飞点；起飞瞬间螺旋桨
   振动/地效让 2D 激光的 scan matching 抖一下 → SLAM 位置平移几厘米~十几厘米 → EKF 跟着变 →
   位置环为此把机身往前倾一下（就是那个"冲"）→ 稳态后回到垂直。间接证据：EV 桥启动 16 秒时
   **触发过一次"位姿时间戳 0.3 s 没推进 → 停发 EV"**，说明这条链路本来就会短暂停顿/抖动。
2. **EV 暂停 → EKF 掉出 EV 融合 → 惯导外推**：守护停发 EV 后 EKF2 要 400 ms 才关 EV、十几秒后才
   置 `xy_valid=false`；这期间位置靠惯导漂，EV 恢复时有个"回正"台阶 → 同样表现为先冲一下。
3. **`EKF2_EV_DELAY` 仍是 0**：整条视觉链路（雷达 15 Hz + SLAM + tf + 桥）本来就有 0.1~0.3 s 延迟，
   不做延迟补偿时位置环在**动态过程**中会过冲；稳态悬停时是准的，所以只在起飞/机动时明显。
4. **`EKF2_EV_POS_X/Y/Z`（EV 安装偏移）没标定**：SLAM/激光原点与飞控原点不重合时，机身一倾斜
   就把倾斜换算成一段平移 → 也是"一起飞就冲"，偏移越大越明显。

**怎么定论（建议按顺序）**：

```bash
# a) 起飞时录一段（在香橙派上）
ros2 bag record -o takeoff_test /fmu/out/vehicle_local_position /fmu/out/vehicle_attitude \
    /fmu/in/vehicle_visual_odometry /tracked_pose /tofsense/height /scan
# 录完把包拿回本机/虚拟机分析：看"冲"的那一瞬间
#   vehicle_local_position 的 x/y 与 vehicle_visual_odometry 的 position 是否同步台阶
```

- b) 同时看 EV 桥日志有没有 `SLAM 位姿跳变 x.xx m`、`位姿时间戳已 0.3 秒没有推进` —— 有就落在 1/2；
- c) 若没有台阶、只在机身倾斜时才冲 → 大概率是 4（安装偏移）或 3（延迟）；
- d) 想先见效：把 `EKF2_EV_DELAY` 按实测填上（QGC），并确认 `EKF2_EV_POS_X/Y/Z` 与实际安装一致。

### 回滚

- 本机/香橙派：队友那 4 个文件已同步；我改的 `run.sh` 可用 `run.sh.bak-20261009` 回滚
  （那是今天 15:59 的原始版本）。想把 IMU 模式改回"自动"：把 `run.sh` 里
  `SLAM_USE_IMU` 那个默认值从 `0` 改成 `1` 即可。

---

## 2026-10-09 晚（20:30–）：**新增 Offboard 小任务航线（起飞 → 前进 → 掉头 → 回原位 → 降落）**

> 用户要求：「我现在想试试 offboard 模式走个任务，就设起飞50cm，往前走50cm，掉头，回原位」

### 改了什么（在 `px4_takeoff_land` 上扩展，没有新建包）

| 文件 | 改动 |
| --- | --- |
| `src/px4_takeoff_land/src/px4_takeoff_land_node.cpp` | 新增 3 个阶段 `MISSION_FORWARD / MISSION_TURN / MISSION_RETURN`（插在 `TAKEOFF` 与 `HOVER` 之间；`mission_enable=false` 时行为与原来**完全一致**）；新增 `publishMissionSetpoint()`（**位置 + 绝对航向**：`yaw` 有效、`yawspeed=NaN`）；新增 `posSettled() / yawSettled()`（要求**连续 `settle_time_` 都待在容差内**才算到位，避免"路过"就算到）；`WaitReady` 里记录起飞点航向并打印任务点；每段到位/超时都打实际误差 |
| `src/px4_takeoff_land/launch/takeoff_land.launch.py` | 新增 7 个任务参数；**所有数值参数改成 `ParameterValue(..., value_type=float/bool)`**（原因见下面的坑） |
| `src/px4_takeoff_land/README.md` | 新增「小任务航线」章节：参数表 / 阶段名 / 验证顺序 / 警告 |

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `mission_enable` | false | 打开任务航线（false = 原来的原地起降） |
| `forward_dist` | 0.5 | 前进距离（m），方向 = **起飞时刻机头方向**（不是世界北） |
| `turn_deg` | 180 | 掉头角度（deg），正 = 顺时针 / 向右 |
| `pos_tol` / `yaw_tol_deg` / `settle_time` | 0.15 / 10 / 0.5 | 到位判定（m / deg / s） |
| `leg_timeout` | 20 | 每段超时（s）→ 超时即 ABORT（交还 PX4 自动降落） |

阶段序列（`/mission_state` 可订阅）：
`WAIT_READY → PRESTREAM → ARMING → OFFBOARD → TAKEOFF → MISSION_FORWARD → MISSION_TURN →`
`MISSION_RETURN → HOVER → DESCEND → HANDOVER_TO_PX4_LAND → DONE`

### 怎么跑（起飞 0.5m / 前进 0.5m / 掉头 180°）

```bash
ros2 launch px4_takeoff_land takeoff_land.launch.py \
    target_alt:=0.5 hover_time:=5.0 \
    mission_enable:=true forward_dist:=0.5 turn_deg:=180
```

### 踩到的坑（必须记住）：ROS 2 参数类型

`turn_deg:=180`（整数）会被 YAML 解析成 **int**，而节点声明的是 double →
`rclcpp::exceptions::InvalidParameterTypeException`，**节点直接崩溃退出**：

```
parameter 'turn_deg' has invalid type: ... is of type {double}, setting it to {integer} is not allowed
```

同类坑：`hover_time:=5`、`leg_timeout:=20`、`target_alt:=1`。
**解决**：launch 文件里用 `ParameterValue(LaunchConfiguration('x'), value_type=float)` 强制类型
（已对所有数值参数生效；bool 同理 `value_type=bool`）。**以后写新 launch 文件都要这么写。**

### 验证（2026-10-09 20:34，**拆桨** dry_run 实测）

```
起飞-降落任务节点已就绪：目标高度 0.50m / 悬停 2.0s / 控制模式 position / dry_run=on
任务航线已启用：前进 0.50m → 掉头 180° → 回原位（到位容差 0.15m / 10°，每段超时 20s，settle 0.5s）
参考点 x=0.03 y=-0.01 z=-0.05 航向=95.2°
任务航线（NED）：起飞 → z=-0.55 | 前进 0.50m → (-0.01, 0.48) 航向 95.2° |
                掉头 180° → 航向 -84.8° | 回原位 (0.03, -0.01) 同航向 | 悬停 2.0s 后降落
阶段 -> PRESTREAM -> ARMING -> [dry_run] 本应发送命令 400 -> ABORT -> [dry_run] 本应发送命令 21
```

- **几何核对通过**：航向 95.2° → `cos=-0.091, sin=0.996` → 目标点
  `x = 0.03 + 0.5×(-0.091) = -0.015 ≈ -0.01`、`y = -0.01 + 0.5×0.996 = 0.488 ≈ 0.48` ✔
  与日志完全一致（说明"前进方向 = 机头方向"算对了）。
- dry_run 里 ARMING 8 秒后必然超时 ABORT —— **这是预期**（它不解锁），用来证明"命令确实被拦住"。

### 飞行前必须确认（清单）

1. `ros2 topic info /fmu/in/vehicle_visual_odometry` 的 **Subscription count ≥ 1**；
   `/fmu/out/vehicle_local_position` 的 `xy_valid / v_xy_valid / z_valid / v_z_valid` 全 true；
2. **拆桨**跑一次上面那条 dry_run，确认 `参考点 / 任务航线` 打印与预期一致（航向数值、正负号）；
3. 真机首飞参数：`target_alt:=0.3~0.5`、`forward_dist:=0.3~0.5`、`hover_time:=5`，
   场地空旷、**遥控器在手**（拨回 Position 就能接管），全程盯 `/mission_state`；
4. 飞控侧按 README 核对：`COM_DISARM_PRFLT`（解锁后不起飞会自动上锁）、`COM_OF_LOSS_T`、
   `COM_OBL_RC_ACT`（离开 Offboard 时的动作）、`MPC_TKO_SPEED`、`MPC_THR_HOVER`；
5. **不要和 `px4_bridge` 同时跑**（两者都发 `/fmu/in/offboard_control_mode`）；
   **也不要在别人用遥控器飞的时候启动它**（它会切 Offboard 抢控制权）。

### 已知风险 / 待观察

- 任务段用**位置 + 绝对航向**，所以必须持续有有效位置估计；一旦 EV 掉、或飞控退出 Offboard，
  节点会 ABORT 并交还 PX4 自动降落（那一刻的行为由 `COM_OBL_RC_ACT` 决定）。
- "起飞时往前冲一下"那个现象会**直接影响前进段的起点精度**（先冲再走），
  上一节"定位录包分析"对这次任务同样重要。
- 掉头段只判定"航向到位"，不判定转向路径；`turn_deg` 给 ±180 都行（正 = 顺时针）。

### 回滚

- 板上 `src/px4_takeoff_land/{src/px4_takeoff_land_node.cpp,launch/takeoff_land.launch.py,README.md}.bak-20261009`
  拷回去 + `colcon build --symlink-install --packages-select px4_takeoff_land`
- 或者不动文件，直接 `mission_enable:=false`（= 原来的原地起降）。

---

## 2026-10-09 晚 20:48：**第一次 Offboard 任务试飞炸机 —— 复盘（根因：飞控↔香橙派 DDS 会话中途断开）**

> 用户原话：「刚才试飞了一次炸机了…」
> 命令：`ros2 launch px4_takeoff_land takeoff_land.launch.py target_alt:=0.5 hover_time:=5.0 mission_enable:=true forward_dist:=1.2 turn_deg:=180`
> 飞行前检查（`Subscription count: 1`、四个 valid 全 true、`dead_reckoning: false`）**都通过了**。

### 时间线（全部来自板上日志，不是推测）

| 时刻 | 事件 | 证据文件 |
| --- | --- | --- |
| 20:48:37 | 任务节点启动（前进 **1.2m**、掉头 180°） | `~/.ros/log/px4_takeoff_land_node_2068_1791550117022.log` |
| 20:48:40 | 参考点 x=0.08 y=-0.05 z=-0.04 航向 93.8° | 同上 |
| 20:48:42 | 解锁成功 → 进入 Offboard（命令回执 result=0） | 同上 |
| 20:48:46.9 | 「到达目标高度 **0.35m**」→ 开始前进段（`alt_tol=0.15`，0.35m 就算到位） | 同上 |
| 20:48:48.9 | 「前进段到位：水平误差 0.124m」→ **前进段本身是成功的**（2 秒走了 1.2m） | 同上 |
| 20:48:51 | IMU 桥：收 100 Hz → **44 Hz**（链路开始异常） | `/tmp/px4_ws_imu.log` |
| 20:48:50→52 | TOF 高度读数 0.047 → 0.103 → **2.550 m** → 0.046 → 0.567 → 0.046（贴地时的离谱值）；EV 的 z 跟着跳到 **-2.55** | `/tmp/px4_ws_tof.log` + `/tmp/px4_ws_ev.log` |
| **20:48:55** | **Agent：`establish_session` → `session re-established`**，随后 PX4 重建 participant 与全部订阅者 | `/tmp/microxrce_agent.log:133` |
| 20:48:57–20:49:08 | IMU 桥连续报「8.8 / 13.8 秒没收到 IMU」→ **飞控↔派断了约 14 s** | `/tmp/px4_ws_imu.log` |
| 20:49:03 | 任务节点 ABORT：「任务(掉头)过程中离开 Offboard」→ 发 NAV_LAND | 任务节点日志 |
| 20:49:08 起 | EV 位置 1.4 → 2.4 → 4.9 → **5.8 m**（飞机真的在往外飞），yaw 在 ±150° 间乱摆；最后停在 (2.3~3.0, 5.7) | `/tmp/px4_ws_ev.log` |
| 20:49:06 | 用户 Ctrl-C 结束任务节点（SIGINT） | 任务节点日志 |

### 根因

**PX4 掉出 Offboard 的直接原因 = 它收不到 offboard 流了，而收不到是因为 uXRCE-DDS 会话断开了。**
`COM_OF_LOSS_T` 默认 1 s：1 秒没有 `OffboardControlMode`/`TrajectorySetpoint`，PX4 就退出 Offboard，
按 `COM_OBL_RC_ACT` 交给对应模式。之后飞控既没有 EV（链路断着）又只能靠惯导 → 位置估计漂 → 飞机往外飞 → 炸机。

**为什么会话会断？链路已经跑在极限上 —— 这是本次新测到的硬数据：**

```
ttyS6（飞控 DDS）10 秒内： tx +20.8 KB (2.1 KB/s)   rx +747.1 KB (74.7 KB/s)
921600 8N1 理论上限 ≈ 90 KB/s   →   接收方向占用 74.7/90 ≈ 83%
驱动计数： fe:1 brk:1 oe:2   （帧错 / break / 溢出）
```

对照实验：**把 IMU 桥停掉再测，流量几乎没变**（74.4 → 75.3 KB/s）→ 占链路不是"我们订阅了谁"，
而是 PX4 按 `dds_topics.yaml` 全速往外发：
`/fmu/out/sensor_combined` **100 Hz**、`/fmu/out/vehicle_local_position` **95 Hz**、`vehicle_attitude` …

→ **余量只剩 ~17%。任务一开始额外多发 50 Hz offboard 流，就把 FC 侧串口处理顶到超时，会话重建。**
触发时机（刚起飞、刚要前进）与"链路刚上量"完全吻合。

### 次生问题（不解决还会再炸）

1. **TOF 高度会偶发离谱读数**（贴地读到 2.55 m），而它**直接就是 EV 的 z** → 2 m 的 z 台阶灌进 EKF2
   足以让位置估计失效（同样能触发掉出 Offboard）。原跳变检测只看水平 (x,y)，**完全没管 z**。
2. **任务节点不知道链路断了**：它只在下一秒 tick 里发现 `!inOffboard()`，日志还写"离开 Offboard 或掉解锁"，
   分不清原因 —— 这次复盘被迫翻三个日志才拼出来。
3. 试飞参数偏激进：`forward_dist=1.2`（建议首飞 0.3），且 `alt_tol=0.15` 让它在 **0.35 m** 就开始前进；
   0.5 m 高度 + 1.2 m 快速平移，是 2D 激光最难受的工况。
4. 飞的是**纯激光**（`fly_no_imu.lua`）—— IMU 那条路可用但尚未验证，没敢开。
5. `COM_OBL_RC_ACT = 0`（Position）：估计坏掉时它会"追着错误位置飞"，比直接 Land 危险。

### 本会话做的加固（已编译 + 部署 + 验证）

| # | 改动 | 作用 |
| --- | --- | --- |
| 1 | `px4_ev_bridge` 新增 `height_jump_max`(0.5m) / `height_jump_confirm`(5 帧)：高度单帧跳变先丢弃，连续多帧才认账 | 挡掉 2.55 m 这类假 z，别再把 EKF 打崩 |
| 2 | `px4_takeoff_land` 的 `control_freq` 50 → **10 Hz** | offboard 流少发 80%，给 83% 占用的串口让路（PX4 只要求周期 <1s） |
| 3 | `px4_takeoff_land` 新增**飞控链路看门狗** `link_timeout`(0.5s)：`/fmu/out/vehicle_status` 超时即报错 + 中止 | 链路一断立刻点名，不再靠猜 |
| 4 | ABORT 日志补上 `armed / nav_state / inOffboard / vehicle_status 未更新时间` | 下次一眼分清"掉解锁"还是"掉模式"、链路断没断 |
| 5 | 两个 launch 的数值参数统一 `ParameterValue(..., value_type=float/int/bool)` | 防 `velocity_sigma:=1` 这类整数参数把节点打崩 |

验证：`colcon build` 两个包通过（31.2s / 32.2s）；重启链路后 `/scan 15.18Hz`、`/tracked_pose 29.99Hz`、
`/tofsense/height 55.73Hz`、`/fmu/in/vehicle_visual_odometry 15.02Hz`；EV 桥实际加载参数含
`height_jump_max: 0.5`、`height_jump_confirm: 5` ✔

### 下次飞行前必须做的（按优先级）

1. **先解决串口余量**（根因）：
   - 飞控 `SER_TEL3_BAUD` 与 Agent 都提到 **1500000**（香橙派 ttyS6 的 `base_baud` 就是 1500000，
     dmesg 有依据）→ 占用从 83% 降到 ~40%；
   - 检查 TELEM3 ↔ 香橙派那条线：尽量短、远离动力线、必要时双绞/屏蔽 —— `fe/brk/oe` 计数说明线上有噪声。
2. **`COM_OBL_RC_ACT` 改成 1（Land）**：室内测试时 Offboard 一丢就落地，比"追着坏位置飞"安全得多。
3. **取 ULog**：飞控插 SD 卡 → QGC → 分析工具 → 下载日志。ULog 里有 PX4 自己的模式切换 / failsafe 原因 /
   估计器状态，是这次唯一缺的证据。**以后每次飞行都留 ULog。**
4. 首飞参数收敛：`target_alt:=0.8~1.0`（高一点对 2D 激光友好）、`forward_dist:=0.3`、先 `turn_deg:=0` 只测直线，
   直线稳定后再加掉头；拆桨 dry_run 先过一遍。
5. 先做 IMU 那条路的**台架倾斜试验**，再决定飞行时是否 `SLAM_USE_IMU=1`。
6. 操作纪律：遥控器在手，一有异常立刻拨回 **Stabilized/Position** 接管，不要等它自己落。

### 回滚

- 板上 `.bak-20261009b` = 本轮 4 个文件（炸机时那一版）；
- `.bak-20261009` = 更早版本（队友那版 / 15:58 原始版）。

---

## 2026-10-09 晚 21:20–21:40：**串口占用没根治（只能两边一起调波特率）+ 新增起飞前自检/波特率工具**

> 用户问：「你改好了吗，那个占用量过高的情况…告诉我飞行前要检查什么，给出代码和任务进行代码」

### 1. 占用量：**没有根治，只能两边一起改**（诚实结论）

- 我之前做的（已生效）：把**我们自己发出去的** offboard 流从 50 Hz 降到 **10 Hz**（`px4_takeoff_land`）。
  但实测**占满链路的是"飞控 → 香橙派"这个方向**，由 PX4 按固件里的 `dds_topics.yaml` 全速发布决定：
  `sensor_combined` 100 Hz、`vehicle_local_position` 95 Hz、`vehicle_attitude` …（停掉 IMU 桥流量不变，已验证）。
- **真正的解法**：把这个串口提速到 **1500000**（香橙派 ttyS6 的 `base_baud` 就是 1500000，dmesg 有依据），
  占用会从 83% 降到 ~40%。**必须两边同时改**，否则连 DDS 都不通：

  1. 香橙派侧（一条命令，已做成脚本）：
     ```bash
     sudo bash ~/px4_ws/tools/set_dds_baud.sh 1500000 --restart
     ```
  2. 飞控侧（**只能你在 QGC 里改**，我碰不到飞控参数）：
     QGC → 参数 → `SER_TEL3_BAUD` = `1500000` → 重启飞控
  3. 验证：`sudo bash ~/px4_ws/tools/set_dds_baud.sh status` 看实测占用；
     `bash ~/px4_ws/tools/preflight.sh` 看链路是否全绿。
  4. 也顺手检查 TELEM3 那根线（短、远离动力线、双绞/屏蔽）—— 计数里有 `fe/brk/oe` 说明有噪声。

  > 备选（不改波特率）：只能靠"少发+线好"，余量仍然只有 17%，不建议继续飞任务。

### 2. 顺带解决/发现的另外三件事

| 事 | 现象 | 处理 |
| --- | --- | --- |
| **新会话收不到数据** | `ros2 topic list` 看得见 `/scan`、`Subscription count=1`，但 `ros2 topic hz` 一个数据都没有（今晚 21:20 又遇到） | FastDDS **同机共享内存传输**在多次重启后建立不起数据通路（发现是好的）。用 `tools/fastdds_udp_only.xml` 强制只用 UDP → 实测 `/scan` 96 帧/8 秒、定高器 281 条 ✔。`preflight.sh` 已自动设置 |
| **重复启动链路** | 今晚 21:18 有人又跑了一次 `bash run.sh fly`（服务里那条还在跑），两份互相 pkill → 留下"进程都在、/scan 与 /tracked_pose 却收不到数据"的半死状态 | `run.sh fly` 加防重复启动：检测到已有 cartographer/Agent 就直接报错并提示用 `run.sh stop` 或 `systemctl restart`（强开用 `ALLOW_DOUBLE_START=1`） |
| **EV 高度跳变** | 上一条复盘里的 TOF 2.55m 假读数 | 已加 `height_jump_max/height_jump_confirm`（见上一节） |

### 3. 新增的两个工具脚本（都在 `px4_ws/tools/`）

```bash
# 起飞前自检（只读）：进程 / 定位链路频率 / EV 是否送进飞控 / 四个 valid / EKF 是否用 EV /
# 串口占用 / 有没有 px4_bridge 抢 offboard。任何 FAIL 都不要飞。
bash ~/px4_ws/tools/preflight.sh

# 切换 DDS 串口波特率（香橙派侧），并打印飞控侧 QGC 要改什么
sudo bash ~/px4_ws/tools/set_dds_baud.sh 1500000 --restart
bash ~/px4_ws/tools/set_dds_baud.sh status      # 看配置 + 实测占用
```

另外 `agent.sh` 现在支持 `/etc/default/microxrce-agent` 里的 `XRCE_DEV/XRCE_BAUD`（配置文件优先、环境变量最高），
所以 `set_dds_baud.sh` 改一处就够。

### 验证（2026-10-09 21:38 实测）

重启链路后 `tools/preflight.sh` 输出 **READY（0 FAIL）**：
`/scan 15.17Hz`、`/tracked_pose 29.98Hz`、`/tofsense/height 56.46Hz`、`EV 14.99Hz`、
`Subscription count=1`、四个 valid 全 true、`dead_reckoning=false`、`cs_ev_*` 全 true、`eph 0.108 / evh 0.099`。
（第 6 段串口占用需要先 `sudo -v` 缓存密码；否则脚本只给 WARN 并提示用 `set_dds_baud.sh status` 看。）

### 回滚

- `run.sh`：`.bak-20261009`（15:59 原始版）；`agent.sh`：`.bak-20261009b`；
- 新增的 `tools/preflight.sh`、`tools/set_dds_baud.sh`、`tools/fastdds_udp_only.xml` 直接删掉即可，不影响链路。

---

## 2026-10-09 晚 21:50：**DDS 波特率已双边提到 1500000（占用 83% → 50%）**

用户反馈：在 QGC 改了 `SER_TEL3_BAUD=1500000` 之后，`set_dds_baud.sh status` 仍显示
`XRCE_BAUD=921600`、Agent `-b 921600`，而且**实测流量 0 KB/s、`fe:189 brk:81` 帧错暴涨**。

**原因**：只改了飞控侧。两边波特率不一致 → 收到的是乱码 → 帧错/break 计数飞涨、一个字节都用不了。
（`status` 当时那行 "上限约 92160 KB/s" 是我脚本的显示 bug，已修。）

**处理（已完成）**：

```bash
sudo bash ~/px4_ws/tools/set_dds_baud.sh 1500000 --restart
```

- `/etc/default/microxrce-agent` → `XRCE_BAUD=1500000`，Agent 实际命令行 `-b 1500000` ✔
- **实测占用：飞控→派 73 KB/s / 上限 146 KB/s @1500000 = 50%**（原 921600 时 74.7/90 = 83%）
- **30 秒内 `fe/brk` 零增长**（581/249 保持不变）→ 帧错都是波特率不匹配那段时间攒的，当前链路干净
- `tools/preflight.sh` → **READY**（`/scan`/`/tracked_pose`/`/tofsense`/EV 频率正常、
  `Subscription count=1`、四个 valid 全 true、`dead_reckoning=false`、`cs_ev_*` 全 true）

**工具改进**：`set_dds_baud.sh status` 现在会
① 分别打印"配置里写的"和"Agent 实际在跑的"波特率并指出不一致；
② 按真实波特率算占用上限；
③ 0 KB/s 时直接给出诊断 "两边波特率不一致 / 飞控没上电 / 线没接 / UXRCE_DDS_CFG 不对"。

**教训（重要）**：**改波特率必须两边同时改**，只改一边 = 链路全断。步骤见
`tools/set_dds_baud.sh` 的输出（它会把 QGC 那几步也打出来）。

### 补记：`preflight.sh` 第 6 段的假 FAIL（同日 22:0x 修）

用户反馈：链路已经提到 1500000、实测 73 KB/s，但自检仍报
`[FAIL] 占用 82% —— 已经在危险区`，因此怀疑"到底有没有真的在验证"。

**这是真 bug**：第 6 段把上限**写死成 921600 的 90 KB/s**（`pct = rx*100/92160`），
换波特率之后分母就错了 → 73 KB/s 被算成 82%、报假 FAIL。
**已修**：改为从 Agent 命令行读实际设备与波特率（`--dev` / `-b`），
`上限 = baud/10/1024`，并新增"0 字节 = 两边波特率不一致/没接线"的诊断。

修复后实测（`sudo bash tools/preflight.sh`）：

```
== 6. 飞控串口（ttyS6）占用 ==
      Agent：/dev/ttyS6 @ 1500000（上限按这个算）
      飞控→派 实测 74 KB/s（1500000 8N1 上限约 146 KB/s，占用 50%）
  [PASS] 占用 50%

结论：READY（0 个 WARN）
```

（顺便确认：**`sudo bash preflight.sh` 可以整份跑**，脚本内部强制走 UDP 传输，
root 身份下 ROS 检查一样正常 —— 第 6 段要读 `/proc/tty/driver/serial` 就必须 root。）

> 这个 bug 的教训写进 `AGENTS.md` §3.21：**任何"阈值/上限"都不能写死，
> 尤其是链路参数会变的地方。**

---

## 2026-10-09 晚 21:49 / 21:50 两次任务试飞复盘：**返航被自己的看门狗掐断（我的 bug）**

用户反馈：「最后一个，飞机一起飞就先向右偏，然后往前飞了1.2m，最后返回的时候只返回了60cm」

### 两次运行的日志（任务节点）

| 运行 | 前进距离 | 关键过程 | 结束原因 |
| --- | --- | --- | --- |
| 21:49:12 | 0.60m | 起飞到 0.66m → 前进段到位（误差 0.064m）→ 掉头 0° → 进 MISSION_RETURN | **21:49:27 中止：飞控 DDS 链路中断（vehicle_status 0.50s 未更新）** |
| 21:50:08 | 1.20m | 起飞到 0.68m → 前进段到位（误差 0.025m）→ 掉头 0° → 进 MISSION_RETURN | **21:50:22.7 中止：同上（返航段只走了 0.7s）** |

两次中止信息里都写着 `armed=1 nav_state=14 inOffboard=1` —— **其实还在 Offboard**；
`/tmp/microxrce_agent.log` 里这段时间**没有任何 `session re-established`**。

### 根因：**看门狗的超时对象选错了（我的 bug）**

- 我加的链路看门狗盯的是 `/fmu/out/vehicle_status`，但它实测**只有 1.3 Hz**（平均间隔 0.77 s），
  而我给的超时是 **0.5 s** → 一多半时间都会误判成"链路断了"。
- 后果：两次任务都在**返航段中途**被自己的看门狗掐断 → ABORT → NAV_LAND 就地落下
  → 用户看到"只返回了 60cm"（EV 轨迹也印证：1.2m 那次从 y=1.11 回到 y=0.60 后落地）。

**修法**：看门狗改成盯 **`/fmu/out/vehicle_local_position`（实测 95 Hz）** →
0.5s 收不到才真是链路断（相当于连丢 47 帧）。已编译部署（`px4_takeoff_land` 33.4s）。

### 第二次观察：**起飞后确实向右偏**（数据）

EV（SLAM）轨迹（航向 ≈86~89°，所以"右" ≈ -x 方向）：

| 运行 | 起飞前 x | 起飞后 2~5 秒的 x | 横向偏移 |
| --- | --- | --- | --- |
| 21:49 | +0.03 | -0.13 → -0.26 → **-0.32** | **向右 0.35m（并且一直偏在那里）** |
| 21:50 | +0.04 | **-0.11** | 向右 0.15m |

两次都偏右、量级 0.15~0.35 m，与之前"起飞瞬间位置估计有台阶 → 位置环去追"的判断一致。
注意 21:49 那次：任务节点（读飞控的 EKF 位置）说"前进段到位、误差 0.064m"，
而 EV（SLAM）同时显示 y≈0.03（几乎没前进）→ **飞控的估计和 SLAM 差出 0.4m 量级**，
说明"位置估计在起飞阶段被带偏"这件事同时在 EKF 和 SLAM 里发生。

**下一步可选动作**：① 先做 IMU 那条路的台架倾斜试验（`SLAM_USE_IMU=1` 之后旋转时的航向更稳）；
② 标定 `EKF2_EV_DELAY`；③ 起飞阶段别做大动作（`MPC_TKO_SPEED` 调小、目标高度给 0.8~1.0m）。

### 第三次发现：**飞控的"高度"和定高器不一致**

21:50 那次：任务节点用飞控的 `dist_bottom` 判定"到达目标高度 0.68m"，
而同一时刻定高器读数 **0.99 m**（21:49 那次是 1.156 m）。`dist_bottom_sensor_bitfield = 0 (NONE)`
—— 也就是说 `dist_bottom` 是 **EKF 的地形估计**，不是定高器直读。起飞阈值判定因此偏早。
（任务段的高度用的是 EV/定高器那一套，所以航段本身没问题；只是"起飞到位"的判据不精确。）

### 当前链路状态（21:55）

- 板子重新上电过（串口计数归零），**这个 boot 到现在 `fe/brk` 都是 0** —— 飞行前后都没涨，
  1.5 Mbaud 下链路干净 ✔（占用 50%）
- `/tracked_pose`、EV、`cs_ev_*`、四个 valid 全部正常（`tools/preflight.sh` 输出 READY）

### 回滚

- `src/px4_takeoff_land/src/px4_takeoff_land_node.cpp.bak-20261009c` = 本次修改前（看门狗盯错话题那版）

---

## 2026-10-10 16:00：**"雷达好像坏了/开机自启动不成功"—— 雷达没坏，是驱动启动偶发失败 + 自检工具误报**

用户反馈：「我的雷达好像坏了，一直没有自启动成功」

### 证据：雷达本体是好的

驱动在开机时（15:56）又报了昨天 17:58 那次同样的错：
```
[ INFO] [ProfileInit 518]: 921600
[ERROR] [open_serial_port 269]: code:-106 value:Failure to modify serial port baud rate
[ INFO] [UartThreadProc 594]: Connection info:uartname /dev/ttyS4 baudrate 921600
```
之后就卡住：**没有握手、`/scan` 没数据**，而且**驱动进程一个 `ttyS4` 句柄都没有**（说明它自己把口关了），
`ttyS4 rx:0 tx:0` 六秒不动。但**手动把口打开裸读能收到完整数据帧**：
```
cf fa 3c 00 fa 05 b4 00 12 ee 00 12 ed 00 11 ee 00 10 00 00 10 ...
```
→ 说明**供电/接线/UART/pinmux 全都正常，雷达在发数据**；坏的是驱动的启动流程（-106，间歇性）。

### 加固：`run.sh start_lidar()`（已用开机路径验证）

1. 起驱动前先用 `stty -F /dev/ttyS4 921600 cs8 -cstopb -parenb -crtscts raw -echo` **预置串口**；
2. 起完 25 秒后**校验 `/scan` 是否真有数据**（用"变量接住"的写法，避免 `grep -q`+pipefail 的老坑），
   没有就**重试一次**；两次都不行才大声报错并列出排查步骤。

`systemctl restart px4-ev-chain`（= 开机自启的同一条路径）实测：
```
[3/7] 起雷达（要等 ~25 秒握手）...
      已预置 /dev/ttyS4 = 921600 8N1 raw
      雷达驱动已启动（第 1 次，日志：/tmp/px4_ws_lidar.log），等 25 秒完成握手...
      ✓ 雷达正常：/scan average rate: 15.178
```
驱动日志 **0 报错**；链路 `/scan 15.09Hz`、`/tracked_pose 30.00Hz`、`/tofsense/height 55.73Hz`、
`/fmu/in/vehicle_visual_odometry 14.99Hz`。

### 另一件事：**板上的 `tools/preflight.sh` 会误报"无数据"**（很可能是"以为雷达坏了"的来源）

板上那份脚本是 **10-10 10:24 由队友改的**（不是本会话推的那版），它没有设
`FASTRTPS_DEFAULT_PROFILES_FILE`，于是撞上 FastDDS 同机共享内存的坑：
`ros2 topic list` 看得见话题、`ros2 topic hz` 却一个数据都收不到 → 脚本判 `/scan`、`/tracked_pose`、
`/tofsense/height` 全部"无数据"（而链路的自检明明是 15.09 / 30.0 / 55.7 ✔）。

**验证**：同一条 `ros2 topic hz /scan` 加上 profile 后读到 97 帧/8 秒、`/tracked_pose` 184 帧/8 秒 ✔。
**一行修法**（在脚本开头加）：
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml
```
本会话**没有覆盖**队友那份脚本（怕他正在改）。

> 它报的另外两项 `cs_ev_vel != true` / `pre_flight_checks_pass != true` 是**飞控侧**状态，
> 它自己的提示也说了：多半飞控还停在上次任务中止时的 Offboard（`nav_state=14`）→
> QGC 里切回 **Position**、等 EV 重新融合后再跑一遍。

### 回滚

- `run.sh.bak-20261010` = 本次加固前的版本

---

## 2026-10-10：**队友又一批改动 + 雷达"静默失败"的三种形态 + 雷达看门狗上线**

> 用户原话：「你先把今天变动的文件同步一下日志，然后就是 `bash ~/px4_ws/tools/preflight.sh` 之后有一连串报错」

### 1. 今天（10-10）板上改动的文件（已全部同步回本机 `px4_ws`）

| 时间 | 文件 | 改动要点 |
| --- | --- | --- |
| 10:05 | `tools/px4_goto.py`（**新**） | Offboard 本地 NED 定点导航，**默认干跑**；注释说明"没有 GPS → QGC 的 Go to location（走经纬度）用不了，只能发本地位置设定值" |
| 10:24 | `tools/preflight.sh`（**重写**） | 队友版起飞前自检：进程 / 数据 / DDS / EKF / localPosValid / 解锁条件 / failsafe 信息，最后给结论清单 |
| 13:08 | `src/px4_ev_bridge/src/px4_ev_bridge_node.cpp` + `launch/ev_bridge.launch.py` | ① `report_velocity` **默认改回 false**（注释："先按最保守的配置来"，并写了"越飞越倾"的观察）② 新增 `yaw_jump_deg=45` 航向跳变检测 ③ **保留并加强**了我加的高度保护（`height_jump_max=0.2`、新增 `height_rate_max=2.0` / `height_stable_band=0.1`、`height_jump_confirm=25`）④ launch 里 σ 仍为 `position 0.25 / velocity 1.0 / yaw 25` |
| 13:09 | `src/px4_takeoff_land/src/px4_takeoff_land_node.cpp` | 任务节点继续演进 |
| 13:15 | `tools/px4_batt_watch.py`（**新**） | 低电看门狗：电压低就发 NAV_LAND。**重要发现**：飞控电池参数没配容量（`capacity=0 / full_charge_capacity_wh=0 / nominal_voltage=0 / time_remaining_s=nan`）→ **PX4 自带的低电 failsafe 永远不触发**，所以必须自己看电压 |
| 13:49 | `px4_imu_bridge.py` | IMU 桥演进 |
| 13:57 | `tools/watch_pose.py`（**新**） | 每秒打印 `/tracked_pose` 与这一秒的变化量、标出跳变 → **地面静置时判断 SLAM 稳不稳的最直接办法** |
| 13:58 | `src/tofsense_m/{src/tofsense_m_node.cpp,launch/tofsense_m.launch.py}` | 定高器驱动与 launch 改动 |
| 16:12 | `run.sh`（本会话） | 雷达启动加固：起驱动前 `stty` 预置端口 + 起完校验 `/scan`、没数据自动重试一次 |
| 16:2x | `tools/preflight.sh`（本会话） | 修了它两个 bug（见 §2） |
| 16:3x | `tools/lidar_watchdog.sh` + `systemd/px4-lidar-watch.service`（**新**，本会话） | 雷达看门狗（见 §3） |

### 2. `bash preflight.sh` 那"一连串报错"逐条解释

| 报错 | 真相 | 处理 |
| --- | --- | --- |
| `/scan 无数据` | **今天两种都遇到了**：<br>① **假报警**（16:2x）—— 脚本没设 `FASTRTPS_DEFAULT_PROFILES_FILE`，撞 FastDDS 同机 SHM 的坑（大消息 `/scan` 尤其明显）：同一条命令加上 profile 就能读到 **97 帧/8 秒**；<br>② **真故障**（16:19 之后）—— 驱动活着、握着 `ttyS4` 句柄、握手也过了，**就是不发布 `/scan`**（地面真值：cartographer 自己的 `scan rate:` 日志停在 16:19:15） | ① 已给脚本加 profile；② 看门狗自动重启驱动（已上线） |
| `cs_ev_vel != true` | **脚本和自家配置矛盾**：EV 桥现在默认 `report_velocity=false`（日志打「速度不上报(交给飞控IMU)」），这种配置下 `cs_ev_vel` 必然是 false，不该算 NG | 已改成：按 EV 桥实际配置判定，`report_velocity=false` 时只打 `[!!]` 提示 |
| `pre_flight_checks_pass != true` | **飞控侧真实状态**（未解锁 `arming_state=1`、`nav_state=4 AUTO_MISSION`）。电池正常（16.39 V / remaining 0.91 / warning 0），四项 valid 全 true。最可能：停在 AUTO_MISSION 而没有任务（`auto_mission_missing: true`），或**没有遥控器信号**（`manual_control_signal_lost: true`）等解锁检查项 | 先在 QGC 切到**定点(Position)** 再重跑；仍不过就看 QGC 的「Ready to Fly」列表，或 MAVLink 控制台 `commander check` |

### 3. 新增：雷达看门狗（自愈）

`tools/lidar_watchdog.sh` + `systemd/px4-lidar-watch.service`（已 `enable`，**开机自启**）：

- 每 20 秒查一次 `/scan`，**连续 3 次没数据**就重启雷达驱动；
- **解锁状态下只报警、绝不动手**（半空中丢雷达 → EV 断 → 飞控进 failsafe，不能让它自己乱重启）。

实测（16:29–16:31）它自己把当时那个"驱动活着却不发布 /scan"的故障救回来了：

```
[16:29:41] [/scan 没数据] 连续第 1/3 次
[16:30:10] [/scan 没数据] 连续第 2/3 次
[16:30:38] [/scan 没数据] 连续第 3/3 次
[16:30:39] 未解锁，且 /scan 连续 3 次无数据 → 重启雷达驱动（日志 /tmp/px4_lidar_watchdog.log）
（重启后 /scan 恢复，window 97）
```

### 4. 雷达"静默失败"的三种形态（务必记住）

1. 开机 `open` 报 `-106 Failure to modify serial port baud rate` 后卡死 → `run.sh` 已加"预置 stty + 校验 /scan + 重试"；
2. 起来了、握手也过了，却**不发布 `/scan`**（进程活着、握着串口句柄）→ 现在由看门狗救；
3. 与 1 同类：日志只有一行、之后再无输出。

**判据**：看 `/scan` **有没有频率**（不是"进程在不在"）；地面真值可以看 cartographer 日志里的 `scan rate:` 行。
**注意**：新会话直接 `ros2 topic hz` 可能因 FastDDS SHM 的坑**假报无数据** —— 先
`export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml`。

### 回滚

- `run.sh.bak-20261010`、`tools/preflight.sh.bak-20261010`（队友原版）；
- 看门狗：`sudo systemctl disable --now px4-lidar-watch`，再删 `tools/lidar_watchdog.sh` 与 `/etc/systemd/system/px4-lidar-watch.service`。

---

## 2026-10-10 晚：**启动链路大改（事件驱动 + 并行 + 分段看门狗）—— 自启动 101 秒 → 30 秒左右，并能自愈**

> 用户指令：「除了硬件你都开始弄吧，记得记录，不用太快，要保证质量」

### 1. 先量化：原来"慢 + 脆"在哪（实测时间线）

| 阶段 | 旧版耗时 | 真相 |
| --- | --- | --- |
| [0/7] 等 NTP | **29 s**（开机那次） | RTC 不准（硬件根因），脚本里死等 60 秒上限 |
| [1/7] Agent | 3 s | 固定 `sleep 3` |
| [3/7] 雷达 | **26 s** | 写死"等 25 秒握手"——**实测真实握手只要 ~1.7 秒**（16:19:27.9→16:19:28.7 日志） |
| [4/7] SLAM | 18.5 s | 固定 `sleep 15` |
| 末尾自检 | ~20 s | 5 个话题串行各等 4~8 s，而且**检查得太早**（"启动完成"≠"就绪"） |
| **合计** | **~101 s（开机）/ 50 s（重启）** | 而且服务 `active` 时链子可能还是坏的 |

### 2. 改了什么

| 改动 | 内容 |
| --- | --- |
| **事件驱动** | 固定 sleep → 轮询就绪：雷达=驱动日志 `lidar start work` + `/scan` 有频率；SLAM=日志 `scan rate:` 或 `/scan_matched_points2` 有频率；tf_to_pose=`/tracked_pose`；EV 桥=日志出现 `EV: NED pos` |
| **并行启动** | Agent / IMU 桥 / 定高器 / 雷达 / SLAM / tf_to_pose / EV 桥**全部一起拉起来**（各节点自己会轮询输入），启动时间从"各步相加"变成"流水线关键路径" |
| **判据以日志为准** | FastDDS 同机发现层偶发"看得见收不到"，只看 `ros2 topic` 会误判（今天误判过两次）；健康判定改用**日志时间戳新鲜度** |
| **失败不中断** | 任何组件失败只报警、继续起其余（原来 `set -e` 会让整条链停在半启动） |
| **`restart <组件>`** | `bash run.sh restart lidar\|slam\|tf\|tof\|ev\|imu\|agent`：看门狗与人工恢复走同一条路径 |
| **链路看门狗** | `tools/chain_watchdog.sh` + `systemd/px4-chain-watch.service`（开机自启、`KillMode=process`）：每 15 s 查六段，**连续 2 轮不健康才动手**，且**只在确认未解锁时才重启**（读不到 `arming_state` 时保守不动手；"飞控确实没在通信"可豁免） |
| **systemd** | 链路单元加 `After/Wants=time-sync.target`（配合 `systemd-time-wait-sync` 真正等 NTP）；**整条链强制 UDP 传输**；旧的 `px4-lidar-watch` 下线（被 `px4-chain-watch` 取代） |

### 3. 实测结果

| 项 | 之前 | 现在 |
| --- | --- | --- |
| `systemctl restart px4-ev-chain` | 50 s（还曾因 180 s 超时被 systemd 杀掉整条链） | **28 s / 34 s**（两次实测） |
| 开机启动 | 101 s | 预计 ~30 s（NTP 29→0、雷达 26→~10、其余并行） |
| 组件故障 | 靠人发现 | **看门狗 30 秒内自动重启对应组件**（实测两次：SLAM 卡死、EV 桥停发，都自动恢复 ✔） |

### 4. 过程中修掉的 4 个**我自己的** bug（已写进代码注释）

1. `wait_until` 用"循环次数"当时限 → 判据本身要 4~5 秒，30 次 = 150 秒（实测重启 182 秒被判超时）→ 改**墙钟计时**。
2. SLAM 就绪判据等 `/tracked_pose`，而它由**后面才启动**的 tf_to_pose 发布 → 永远等不到 → 改用 cartographer 自己的日志与 `/scan_matched_points2`。
3. `status_summary` 里裸 `wait` → 等的是**整条链的所有后台任务**（永不返回）→ 启动脚本被 systemd 超时杀掉 → 改为只等采样子进程。
4. **重复实例**：`pkill -f cartographer` 只杀节点，**旧的 `ros2 launch` 包装进程还活着**，它的 `respawn=True` 又把节点拉起来 → 两份 cartographer 打架（TF 抖动、`map_builder_bridge.cpp:180 Requested submap … does not exist`）→ 现在连 launch 包装一起杀，看门狗还会查"实例数=1"（锚定真实路径，避免把"正在看日志的人自己那条命令"数进去）。

### 5. 还没解决 / 待观察

- **组件偶发"进程活着但输出停住"**（实测 cartographer 25 秒后卡死、EV 桥停发）→ 已由看门狗兜住；已把整条链切成 UDP 传输做观察（怀疑是 FastDDS 共享内存通路把发布线程卡住）。
- **RTC 纽扣电池**（硬件）：换掉就能删掉 NTP 等待与时间跳变风险。
- **飞控电池参数没配容量**（QGC）：配好才能让 PX4 自带低电 failsafe 生效。
- 飞控串口余量已从 83% 降到 50%；`COM_OBL_RC_ACT` 建议改 Land。

### 相关文件 / 回滚

- 改：`run.sh`（重构）、`systemd/px4-ev-chain.service`
- 新增：`tools/chain_watchdog.sh`、`systemd/px4-chain-watch.service`
- 下线：`tools/lidar_watchdog.sh`（移到板上 `tools/.old/`）、`px4-lidar-watch.service`（已 disable 并删除）
- 回滚：`run.sh.bak-20261010b`、板上 `/etc/systemd/system/px4-ev-chain.service.bak-20261010`

---

## 2026-10-10 深夜：**第二次炸机复盘（飞控自己重启）+「解锁超时」误报根治 + 香橙派侧四处加固 + 新增「任务.sh」**

> 用户问：「那现在跑一下任务，原来的那个任务.sh还能用吗」；同时报「现在解锁超时」。

### 0. 结论速览（先看这里）

| 问题 | 结论 |
| --- | --- |
| 「任务.sh」 | **从来就不存在**（本机 `D:\` 与板上 `~`、`~/px4_ws` 都搜过）。"跑任务"一直是手敲那条 `ros2 launch px4_takeoff_land ...`。本会话**新建了 `tools/mission.sh`**，以后就用它 |
| 「解锁超时」 | **误报，不是故障**。今天的 5 次「解锁超时」全是 `dry_run:=true`（干跑本来就不会解锁，节点却傻等 8s 再报错）。真跑那次（17:52）**解锁是成功的**（`命令 400 回执 result=0`） |
| 上次炸机的真凶 | **飞控在飞行中自己重启了**（17:52:42，当时正悬停 0.37m 做掉头）。三条独立证据见 §2。**这不是香橙派/DDS 的问题**，香橙派侧只能做到"发现并如实报告" |
| 两次真机都死在同一位置 | 10:27 那次也是"前进段跑完 → 进 `MISSION_TURN` → 日志戛然而止"（没有中止、没有 SIGINT）。**掉头是电流最大的时刻**，最大嫌疑是供电瞬断（brownout）或物理碰撞，属硬件 |
| 现在能飞吗 | **不建议**。飞控会随机重启这件事没查清之前，起飞就是拿飞机赌。见 §5 清单 |

### 1. 「解锁超时」到底怎么回事

#### 1.1 干跑误报（今天 5 次前 4 次都是这个）

节点原来在 `dry_run` 下也会进 `ARMING`，而干跑**不会**真发解锁命令（`sendVehicleCommand` 里遇到 dry_run 只打印）→ 等 `arm_timeout`(8s) 到点 → 报 `解锁超时`。用户看到的吓人日志就是这么来的：

```
[13:16] nav_state=4 inOffboard=0  解锁超时   ← dry_run=on
[13:50] nav_state=2               解锁超时   ← dry_run=on
[10:20] nav_state=14              解锁超时   ← dry_run=on
[17:38] nav_state=4               解锁超时   ← dry_run=on
[17:48] nav_state=14              解锁超时   ← dry_run=on
[17:52] nav_state=14 → ARM 回执 result=0 → 已解锁 → 进入 Offboard → 起飞 0.37m → 前进 0.5m ✔  ← 这是唯一一次真跑
```

#### 1.2 真跑时确实存在"按模式判定"的解锁门槛（已实测）

读 PX4 源码确认：`pre_flight_checks_pass` **不是飞机健康度**，而是

```cpp
// src/modules/commander/Commander.cpp:1834
bool pre_flight_checks_pass = _health_and_arming_checks.canArm(_vehicle_status.nav_state);
```

而 `canArm(nav_state)` = `can_arm & getModeGroup(nav_state)`（`HealthAndArmingChecks/Common.hpp`），
`modeCheck.cpp` 会把"当前模式的模式需求没满足"变成**针对该模式族**的解锁禁令：

```cpp
if (offboard_control_signal_lost && mode_req_offboard_signal != 0)   // mode_req_offboard_signal = 1<<14 → 只针对 OFFBOARD
    reporter.armingCheckFailure(..., "No offboard signal");           // → 锁掉 OFFBOARD 族的解锁位
if (auto_mission_missing && mode_req_mission != 0) ...                // 同理：AUTO_MISSION 族
if (mode_req_prevent_arming & (1u << nav_state)) ...                  // AUTO_LAND/TERMINATION 等：直接不许解锁
```

**所以：中止一次任务后飞控停在 OFFBOARD，下一次任务若在"没有 Offboard 流"的瞬间来解锁，就会被拒。**
实测 A/B（在不解锁的前提下用 `ros2 topic pub` 发 `DO_SET_MODE`）：

| 飞控模式 | `pre_flight_checks_pass` |
| --- | --- |
| OFFBOARD(14) + 没有 Offboard 流 | **false** → 拒解锁 |
| 切到 POSCTL(2) | **true** → 飞机完全健康、可解锁 |
| 再切回 OFFBOARD(14) | false |

> 附注：我第一次做这个测试时发的 offboard 报里 `timestamp: 0`，PX4 判定"太旧"→ 不算有效流，
> 所以那一栏结果不算数（`offboardCheck.cpp` 判据是 `hrt < ocm.timestamp + COM_OF_LOSS_T`）。
> 真正的证据是 17:52 那次真跑：OFFBOARD 下 ARM 回执 `result=0`，说明"有流就能解锁"这条路是通的。

### 2. 上次炸机的真凶：飞控自己重启（三条证据）

任务节点 17:52 那次的完整日志（`~/.ros/log/px4_takeoff_land_node_4672_*.log`）：

```
17:52:30.87 节点就绪 → 参考点 (-0.38,-0.09,-0.04) 航向 91.6°
17:52:33.97 PRESTREAM → 17:52:36.06 ARMING
17:52:36.38 已发送解锁命令  → 17:52:36.38「命令 400 回执：result=0」→ 17:52:36.46 已解锁
17:52:36.56 进入 Offboard → 17:52:40.96 到达目标高度 0.37m
17:52:42.85 前进段到位（水平误差 0.094m）→ MISSION_TURN
17:52:43.55 [ERROR] 任务中止：飞控 DDS 链路中断（/fmu/out/vehicle_local_position 超时未更新）
             （armed=1 nav_state=14 inOffboard=1，vehicle_status 已 0.61s 未更新）
17:52:48.43 SIGINT（用户 Ctrl-C）
```

三条互相独立的证据指向同一件事——**飞控整机重启，不是 DDS 抖动**：

1. **agent 日志**（`/tmp/microxrce_agent.log`）：`[1791625968.53] = 17:52:48` `session re-established`，
   紧接着 17:52:55 重新创建**全部** DDS 实体（participant/topic/publisher/datawriter 一整套）。
   而 agent 进程本身从 17:44:16 起就没重启过（`etim` 连续）→ 重连是**飞控那一侧**发起的。
2. **飞控自己的 hrt 时钟反推开机时刻**：`vehicle_status.nav_state_timestamp` 是 hrt（不被 DDS 桥转成 epoch）。
   18:01:03 我发 `DO_SET_MODE`，返回值 `492180703 µs = 492.18s` → **飞控开机 = 17:52:51**。
3. **飞控 RAM 状态被清零**：`latest_arming_reason` / `latest_disarming_reason` 由 `1` 变成 `0`（复位值）。

同时排除"串口噪声"这个嫌疑：24 秒采样里 `tx +41460 / rx +1.4M`，而 **`fe`/`brk` 一个都没涨**
（`/proc/tty/driver/serial` 第 6 行：`fe:10 brk:3 oe:3`，自 17:44 起就这么点）→ 1.5 Mbaud 这条线是干净的。

**两次真机都死在 `MISSION_TURN`**：10:27 那次（`dry_run=off`，同样是 0.5m 任务）日志在
`== 阶段 -> MISSION_TURN` 之后**一行都没有**（既没中止也没 SIGINT）→ 说明当时不是"节点正常退出"。
掉头是四轴电流最大、机身角速度最大的时刻。**最大嫌疑：供电瞬断（brownout）/ 电池或电源模块带不动 /
掉头时机身/桨叶撞到东西导致接触不良。** 属硬件，我没有替用户动。

### 3. 本次改了香橙派侧的 5 处（全部已实测）

#### 3.1 `takeoff_land.launch.py`：launch 自己补上 FastDDS UDP profile

```python
_udp_profile = os.path.expanduser('~/px4_ws/tools/fastdds_udp_only.xml')
_env_actions = ([SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', _udp_profile)]
                if os.path.exists(_udp_profile) else [])
```

为什么：systemd 那条链有 env，**手敲 launch 的交互 shell 没有** → FastDDS 同机共享内存的坑
（话题看得见、数据收不到/停住）。
实测：用 `env -u FASTRTPS_DEFAULT_PROFILES_FILE` 起 launch，
节点进程 `/proc/<pid>/environ` 里仍然有 `FASTRTPS_DEFAULT_PROFILES_FILE=.../fastdds_udp_only.xml` ✔

#### 3.2 `px4_takeoff_land_node.cpp`：四处改动

1. **新增 `SetArmMode` 阶段**（在 PRESTREAM 之后、ARMING 之前）：
   等飞控自己说"可以解锁"（`pre_flight_checks_pass`）；不行就明确切一次 **POSCTL**
   （`DO_SET_MODE param1=1 param2=3`）再解锁；4 秒还不行就照旧进 ARMING 试一次。
   → 从此不会再被"残留模式"卡住（OFFBOARD 无流 / AUTO_MISSION 无任务 / AUTO_LAND …）。
2. **新增 `/fmu/out/failsafe_flags` 订阅 + `armBlockReason()`**：解锁被拒时把原因翻成人话打出来
   （无 Offboard 信号 / 本地位置无效 / 无 home / 无任务 / 无手动输入 / 电池告警 / 当前模式不适合解锁 …）。
3. **链路看门狗 `link_timeout` 0.5s → 1.0s**，并加"0.5s 级停顿只提示不动手"的日志；
   超时中止时的文案改成"飞控也可能刚重启过——去 agent 日志找 session re-established"（这次就是这么回事）。
   依据：实测该话题 ~100 Hz、最大间隔 0.021s（`watch_fc_link.py` 实测 99.9 Hz / 46 ms），1.0s ≈ 连丢 100 帧；
   而飞控自己的 Offboard 失效保护是 `COM_OF_LOSS_T`（默认 1s），放宽到 1s 不会更危险。
4. **`dry_run` 不再假报「解锁超时」**：干跑跑到 PRESTREAM 结束就打印
   "干跑结束：Offboard 流已连发 2.0s。真跑时接着会：切模式 → 解锁 → …" 然后进 DONE。

#### 3.3 新增 `tools/mission.sh`（就是用户要的「任务.sh」）

```bash
bash ~/px4_ws/tools/mission.sh                 # 默认 0.5m / 前进 0.5m / 掉头 180° / 悬停 5s
bash ~/px4_ws/tools/mission.sh 0.8 0.3 0 5     # 高度 前进 掉头角 悬停秒
DRY=1 bash ~/px4_ws/tools/mission.sh           # 干跑
FORCE=1 ...  跳过"自检没过"的拦截；SKIP_PREFLIGHT=1 不做自检
```

负责：source 环境 → **强制 UDP** → 跑 `tools/preflight.sh`（没过就拒绝起飞）→ 打印参数 →
（交互式时问一次 yes 确认）→ `exec ros2 launch px4_takeoff_land takeoff_land.launch.py ...`。

#### 3.4 新增 `tools/watch_fc_link.py`：地检"飞控会不会自己重启/链路会不会停"

同时盯三条线索：`/fmu/out/vehicle_local_position` 的到达间隔（>0.5s 警告 / >1.0s 记重大中断）、
`vehicle_status` 的解锁/模式变化、agent 日志里新增的 `session re-established`。
实测：99.9 Hz、最大间隔 46 ms、0 次中断；伪造一条 `session re-established` 能立刻报警 ✔。
**用途**：飞机在地面静置时开着它跑十几分钟，就能知道"飞控会不会无缘无故重启"。

#### 3.5 板上环境：让所有 shell 都强制 FastDDS 走 UDP

- `/home/orangepi/.bashrc`：把导出**插到"非交互就 return"之前**
  （原来放在文件末尾 → `bash -lc` 这类非交互 shell 根本不执行；这里是第一次踩这个坑）。
  备份：`/home/orangepi/.bashrc.bak-20261010`。
- 新增 `/etc/profile.d/fastdds-udp.sh`（登录 shell 兜底）。
  实测：`bash -lc 'echo $FASTRTPS_DEFAULT_PROFILES_FILE'` → 有值 ✔

### 4. 本会话自己踩的三个坑（记下来免得再犯）

1. **`set -u` + `source /opt/ros/humble/setup.bash`**：ROS 的 setup 里有未定义变量，
   非交互 shell 在 `set -u` 下会**直接退出**（`mission.sh` 打完 "1/4 环境" 就没了）。→ 已改成 `set +u`。
2. **README 式的 `\~/.bashrc` 末尾导出对非交互 shell 无效**（Ubuntu 的 .bashrc 开头就 `return`）。
3. **C++ 里少一行 `using px4_msgs::msg::FailsafeFlags;`** → `error: 'FailsafeFlags' does not name a type`。
   这个文件不用 `using namespace`，每个消息类型都要单独 using（编译报错已修）。

### 5. 下一次飞行前的动作清单

香橙派侧我已经做完的（可直接复核）：

```bash
bash ~/px4_ws/tools/preflight.sh              # 期望「全部通过」——今天 18:0x 实测 22 项全绿
python3 ~/px4_ws/tools/watch_fc_link.py --minutes 15   # 静置 15 分钟：飞控重连次数应为 0，中断 0 次
DRY=1 bash ~/px4_ws/tools/mission.sh          # 干跑一遍，确认航线与链路
```

**只能用户做的（硬件/飞控侧）**，按优先级：

1. **查供电**（最要紧）：这是当前唯一"已知能导致炸机"的原因。
   确认飞控是怎么取电的（电源模块/BEC/USB？香橙派是不是也吃同一条 5V？），
   看接头/焊点/XT60 是否发黑松动，必要时给飞控独立供电 + 并一颗大电容。
2. **从飞控 SD 卡拉 ulog 看复位原因**（或用 QGC 看开机消息）：
   PX4 开机时会打印复位原因（watchdog / hardfault / 掉电），这是唯一能区分
   "掉电" 与 "固件跑飞" 的直接证据。
3. 飞控串口线整理（`fe/brk` 现在还很小，属预防）。
4. QGC：`COM_OBL_RC_ACT` 建议设 Land；电池容量参数补上（否则 PX4 自带低电 failsafe 永不触发）。

**首次复飞的保守打法**（等上面第 1 项有结论后）：

```bash
bash ~/px4_ws/tools/mission.sh 0.5 0 0 3      # 只起飞+悬停 3s+降落，不走任务段
# 稳了再走"不 掉头"的航线：
bash ~/px4_ws/tools/mission.sh 0.5 0.5 0 5
# 最后才上带 180° 掉头的完整任务：
bash ~/px4_ws/tools/mission.sh 0.5 0.5 180 5
```

飞完看：`~/.ros/log/px4_takeoff_land_node_*.log`（最新）、
`grep session /tmp/microxrce_agent.log`、`sudo journalctl -u px4-chain-watch --no-pager | tail -30`。

### 6. 改动文件 / 回滚

| 文件 | 说明 |
| --- | --- |
| `src/px4_takeoff_land/src/px4_takeoff_land_node.cpp` | 4 处改动（都带 `2026-10-10` 注释）；已 `colcon build` 通过 |
| `src/px4_takeoff_land/launch/takeoff_land.launch.py` | 加 `SetEnvironmentVariable` |
| `tools/mission.sh`（新） | 任务.sh |
| `tools/watch_fc_link.py`（新） | 链路/重启地检 |
| `tools/preflight.sh` | 结论里的"下一步"改成推荐 `mission.sh` 的三种打法（手敲写法仍列出） |
| 板上 `~/.bashrc`、`/etc/profile.d/fastdds-udp.sh`（新） | 强制 UDP |

五个文件的板上/本机 md5 已逐一比对，**完全一致**：`mission.sh 1fa7f5b9…`、
`watch_fc_link.py 0214c370…`、`preflight.sh c6e95f68…`、
`px4_takeoff_land_node.cpp 360d5488…`、`takeoff_land.launch.py d6438d1f…`。

回滚：

- 新文件：`rm tools/mission.sh tools/watch_fc_link.py /etc/profile.d/fastdds-udp.sh`（后者需 sudo）；
- `.bashrc`：`cp ~/.bashrc.bak-20261010 ~/.bashrc`；
- 节点/launch：删掉所有标了 `2026-10-10` 的块（四处 + launch 的 env 两行）。
- ⚠️ **流程失误**：这次改节点前**没有**在板上留 `px4_takeoff_land_node.cpp.bak-20261010`
  （同一个文件被直接覆盖）。以后改前先 `cp <文件> <文件>.bak-$(date +%Y%m%d)`。

### 7. 补丁（同日稍后）：`tools/` 下的 python 工具「不用先 source 也能跑」

**用户实测踩到**：

```
orangepi@era-desktop:~/px4_ws$ python3 ~/px4_ws/tools/watch_fc_link.py --minutes 15
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
ModuleNotFoundError: No module named 'px4_msgs'
```

原因：本机 `~/.bashrc` 只设了 FastDDS profile 与 `ROS_DOMAIN_ID`，**不会**自动 source ROS；
必须自己先 `source /opt/ros/humble/setup.bash` + `source ~/px4_ws/install/setup.bash`。
（我开发时是在 source 过的 shell 里测的，所以没暴露——这条已记进 AGENTS 的流程提醒。）

做法：

- 新增 `tools/_ros_env.py`：环境 OK 就什么都不做；不 OK 就用 `bash -c 'source …; exec python3 <本脚本> "$@"'`
  **重新起一次自己**（参数原样保留，环境变量 `_ROS_ENV_WRAPPED=1` 防递归），
  并在没设 FastDDS profile 时自动设成 UDP（顺手把那类坑也堵上）。
- 在 4 个工具最前面各加一行 `import _ros_env  # noqa: F401`：
  `watch_fc_link.py`、`px4_goto.py`、`px4_batt_watch.py`、`watch_pose.py`。
- 顺手修三个工具原有的退出毛病：`finally: rclpy.shutdown()` 在 SIGTERM/Ctrl-C 下会二次关闭，
  抛 `RCLError: failed to shutdown: rcl_shutdown already called` 刷一屏红字 → 加 `if rclpy.ok():` 守卫。

实测（全部在 `AMENT_PREFIX_PATH` 为空的干净 shell 里跑）：

| 工具 | 结果 |
| --- | --- |
| `watch_fc_link.py --minutes 0.4` | 自举成功；93.9 Hz，最大间隔 29 ms，>0.5s 0 次，飞控重连 0 次 |
| `watch_pose.py` | 静置 Δxy ≤ 0.011 m、Δyaw ≤ 0.21°（判据 <0.02m / <0.5° 达标） |
| `px4_goto.py -p x:=0 -p y:=0` | 干跑正常打印当前位置/目标 |
| `px4_batt_watch.py` | `dry_run` 默认 true；16.83 V（4.209 V/cell）健康 |

残留一行无害提示：`watch_pose.py` 退出时可能打 `Failed to publish log message to rosout`
（SIGTERM 之后上下文已关，摘要日志发不出去，纯粹影响观感）。

**回滚**：删 `tools/_ros_env.py`，并去掉 4 个脚本里那行 `import _ros_env`（`rclpy.ok()` 守卫可以留着）。

> 顺带观察：18:25 复查时飞控 `nav_state=4`（AUTO_MISSION）——不是我这几个工具改的（它们没发任何指令），
> 应该是用户/遥控器/QGC 那边切的。这正好是"老逻辑会直接解锁超时、新逻辑先切 POSCTL"的场景。

### 8. 用户要求：ROS 环境写进 `~/.bashrc`（不用每次先 source）

用户原话：「你写进bashrc里，不用我每次都要先source」。

在 `~/.bashrc` 里、**「非交互就 return」之前**（紧接着上一节的 FastDDS UDP 导出）加了：

```bash
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    [ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
    [ -f "$HOME/px4_ws/install/setup.bash" ] && . "$HOME/px4_ws/install/setup.bash"
fi
```

要点与实测：

| 场合 | 结果 |
| --- | --- |
| 交互式新终端（`bash -i`） | `ROS_DISTRO=humble`、`which ros2` 有、`import px4_msgs` 成功、`ros2 topic list` 直接能看到话题 ✔ |
| 登录 shell（`bash -lc`） | 同上（`~/.profile` 里本来就有 `source ~/.bashrc`）✔ |
| **bash 脚本 / systemd 单元** | **不受影响**（它们根本不读 `.bashrc`，`BASH_ENV` 也是空的 → `run.sh` / 看门狗行为不变）✔ |
| `ssh orangepi@… 'cmd'` | sshd 把 shell 当登录 shell 起 → 也会带上（对我做诊断反而方便） |

备份：`/home/orangepi/.bashrc.bak-20261010b`；回滚 = 删掉上面那个 `if` 块（或直接 `cp` 备份回来）。

> ⚠️ 顺手发现 + 已排除疑点：19:21 复查时看到 `px4-ev-chain` 一度是 `activating`、
> `px4-chain-watch` 是 `inactive` —— 查下来是**香橙派在 19:19:12 被重启过**
> （单调时钟 `uptime -s` 确认；上一次运行的日志**没有 systemd 关机序列**，是"断电/硬复位"式），
> run.sh 在 19:21:54 重新拉起链路（本轮启动 32 秒，各话题 /scan 15.3Hz、/tracked_pose 30Hz、
> /imu 63.9Hz、/tofsense 55.8Hz、EV 15Hz 全正常）。时间点在改 `.bashrc` **之前**，
> 所以与本次改动无关。**但要留意**：这是今天第二次"不干净的下电"（第一次是 17:52 飞控飞行中重启），
> 如果这次不是用户自己拔的电，那就和飞控那次是同一类现象 —— 供电问题（见 §5）。

---

## 2026-10-10 深夜第二轮：**任务改成"0.5m 高度前进 1.2m（不掉头）" + 新增两道交叉校验 / 分段缓转**

> 用户指令：「先别掉头了，直接把任务变成在 0.5m 高度下前进 1.2m。然后再把你前面一些优化加上」

### 1. 任务形态改了（默认就是用户要的那条）

| 项 | 以前 | 现在（默认） |
| --- | --- | --- |
| 航线 | 前进 0.5m → 掉头 180° → 飞回起飞点 | **前进 1.2m → 就地悬停 → 交还 PX4 降落** |
| 掉头 | 必做 | **不做**（`turn_deg` 默认 0；要恢复就给 `turn_deg:=180`） |
| 返航 | 必做 | **不做**（新参数 `return_home`，默认 `false`） |
| 落点 | 起飞点 | 前进 1.2m 处（原地降落） |

```bash
bash ~/px4_ws/tools/mission.sh                 # 0.5m 高 / 前进 1.2m / 不掉头 / 悬停 5s → 原地降落
bash ~/px4_ws/tools/mission.sh 0.8 1.5 0 8     # 高度 前进 掉头角 悬停秒
RETURN=1 bash ~/px4_ws/tools/mission.sh 0.5 0.5 180 5   # 老任务：前进→掉头180°→飞回原点
```

代码里：`MISSION_FORWARD` 到位后，若 `turn_deg==0 且 !return_home` 直接进 `HOVER`（连"掉头"那段航向指令都不发）；
否则走 `MISSION_TURN`（可选择是否分段缓转）→ 按 `return_home` 决定进 `MISSION_RETURN` 还是 `HOVER`。

### 2. 新增的四条保护（"前面那些优化"）

#### A. 掉头分段缓转 `turn_step_deg`（默认 30°）

每次只把"该发的航向设定值"往前推 ≤30°，等机体大致到位（`yawSettled`）再推下一段，
避免一次性 180° 让四个电机瞬间吃满差速 —— **10-10 炸机就发生在"开始掉头"那一瞬**。
`turn_step_deg:=0` 退化回老行为。干跑已验证参数生效（日志会打"掉头规划：…分段缓转步长 30°"）。
（现在默认不掉头，这条是给以后重新开掉头留的安全阀。）

#### B. 航向交叉校验 `yaw_xcheck_deg`（默认 20°，持续 1s 就中止）

拿**飞控 EKF 的 `heading`** 和 **EV 桥发给飞控的那份位姿**（`/fmu/in/vehicle_visual_odometry`）的 yaw 对比。
依据：2D 激光 SLAM 没有 IMU，快速掉头/漂移时航向可能"缓慢但持续地跑偏"，队友在 EV 桥里的
**45° 单帧跳变门限挡不住这种**；而 EKF 吃到错航向会驱动电机猛追（本次炸机的候选成因之一）。

> ⚠️ **实测抓到的坑（差点又埋一个雷）**：一开始我比的是 `/tracked_pose`，
> 实测 `/tracked_pose` 的 yaw ≈ **0°**，而飞控 `heading` = **90.4°** —— 差 90°！
> 原因是 `/tracked_pose` 是 SLAM 的 **map 系**，原点航向任意；队友的 EV 桥是在**发往飞控时**
> 才把它对齐到 NED（所以它日志里打的是 91.1°，跟飞控一致）。
> **按原写法一解锁就会误报"航向差 90°"并中止。** 改成用 EV 桥的 NED 输出，并且像高度那样
> **在地面记一次基准差**、只比"变化量"，坐标系常量差自动消掉。
> 实测基准：`EV 89.9° − 飞控 90.3° = −0.4°` ✔

#### C. 高度交叉校验 `alt_xcheck_m`（默认 0.5m，超 0.9m 且持续 1s 就中止）

拿**飞控的 z**（不是 `dist_bottom`——那个在 `dist_bottom_sensor_bitfield=NONE` 时是地形估计）
和**定高器 TOF**（`/tofsense/height`）对比；同样用"起飞前在地面记下的两者差值"当基准，比变化量。
实测基准：`定高器 0.05m − 飞控 0.00m = 0.05m` ✔

**倾斜门限 `tilt_gate_deg`（默认 25°）**：机体倾斜超过它就不做这项。因为 TOF 向下发射，
倾斜会让光路变斜、读数变长 —— 10-10 那次"1.5m / 2.07m"就是**机体翻滚时的斜射伪值**
（0.6m/cos65°≈1.4m），不是真高度（用户现场确认实际就是 60cm 左右）。

#### D. 飞控"已解锁→未解锁"单独报一条 ERROR

空中出现这个基本就是飞控重启/掉电/触发保护；日志里会提醒去 `/tmp/microxrce_agent.log`
找 `session re-established`（10-10 那次就是靠这条线定性的）。

### 3. 怎么验证的

| 项 | 结果 |
| --- | --- |
| `colcon build --packages-select px4_takeoff_land` | ✔ 通过（55s / 56s，两次） |
| 干跑 A（新默认） | 航线日志："前进 1.20m → 掉头 0° → 就地悬停降落（不返航）"、"不掉头、不返航（就地降落）" ✔ |
| 干跑 B（`RETURN=1` + 180°） | "掉头后飞回起飞点正上方" + "掉头规划：目标航向 −89.5°（转 180°），分段缓转步长 30°" ✔ |
| 两个地面基准 | 高度 0.05m / 航向 −0.4° ✔（说明两个来源本来就基本一致，校验不会误报） |
| 五个文件板上/本机 md5 | **完全一致**：mission.sh `6eb28c3b…`、node `237e0c9b…`、launch `16cc51c2…`、CMakeLists `e3f3b008…`、package.xml `2c13f8f0…` |

> 本轮中途还遇到一次**板子失联**（20:00 左右 scp/ssh 全部超时、ARP 表里没有它），
> 20:04 重新上线（`up 0 min`）—— 是用户自己断电。失联期间**没有任何半成品写到板上**
> （scp 全部失败），恢复后一次同步成功。

### 4. 新参数一览（launch 里都可调）

`return_home`(bool,false) / `turn_step_deg`(30°) / `yaw_xcheck_deg`(20°) / `alt_xcheck_m`(0.5) /
`alt_xcheck_abort_m`(0.9) / `xcheck_hold`(1.0s) / `tilt_gate_deg`(25°)
—— 想关掉某道校验就把对应阈值设 0。

### 5. 改动文件 / 回滚

- 改：`src/px4_takeoff_land/src/px4_takeoff_land_node.cpp`、`launch/takeoff_land.launch.py`、`tools/mission.sh`
- 勘误：中途给 `CMakeLists.txt` / `package.xml` 加过 `geometry_msgs`（原计划订阅 `/tracked_pose`），
  改用 `/fmu/in/vehicle_visual_odometry` 后**已经去掉**，两个文件回到原样。
- 回滚：删掉所有标了 `2026-10-10` 第二轮注释的块；或把 `mission.sh` 默认参数改回 `0.5 0.5 180 5`
  并把 `return_home` 默认设回 `true`（等价于回到"前进→掉头→返航"的老任务）。

---

## 2026-10-10 夜间：**飞控日志复盘（用户从 SD 卡拿回早上那份 ulog）+「下午日志全没了」的原因查明**

> 用户：「我看了一下 sd 卡，只有早上飞的记录，下午那几次都没有。」并附上
> `log_70_2026-10-10-10-58-28.ulg`（板上路径 `/fs/microsd/log/2026-10-10/02_58_14.ulg`，UTC，= 北京 10:58）。

### 1. 新增工具 `tools/analyze_ulog.py`（可复用）

本机 Windows 只有商店版 Python 占位符，没法解析 ulog；**香橙派上能上 PyPI**，所以用
"拉 wheel 解到 /tmp、不装 pip、不动系统包"的办法装 pyulog（纯 python，依赖 numpy，板上已有）：

```bash
mkdir -p /tmp/pyulog_pkg && cd /tmp
url=$(curl -s https://pypi.org/pypi/pyulog/json | python3 -c "import sys,json;print([u['url'] for u in json.load(sys.stdin)['urls'] if u['filename'].endswith('.whl')][0])")
curl -sL -o /tmp/pyulog.whl "$url" && python3 -m zipfile -e /tmp/pyulog.whl /tmp/pyulog_pkg
PYTHONPATH=/tmp/pyulog_pkg python3 ~/px4_ws/tools/analyze_ulog.py <某个.ulg> --tail 3
```

`analyze_ulog.py` 一次打出：基本信息/时长、**日志里的文字消息**（飞控自己说的话）、
话题清单（看谁先停）、丢包段、以及最后 N 秒的**电压/电流、电机输出、姿态、飞控状态**。

### 2. ⚠️ 关键结论①：**「下午的日志都没有」不是配置问题，而是"文件没来得及关闭"**

| 证据 | 说明 |
| --- | --- |
| `SDLOG_MODE = 0` | PX4 默认：**解锁开始记录、上锁时结束并关闭文件** |
| `COM_ARM_SDCARD = 1` | "没插好卡不许解锁" —— 而下午两次任务**都成功解锁了**（`命令 400 回执 result=0`）→ 卡在、能写 |
| 早上那份日志 | 恰好是**唯一一次正常上锁**的飞行（日志里能看到 `Landing detected` → `Disarmed by RC`）→ 文件被正常关闭 → 目录项写进了 FAT → 看得见 |
| 下午两次 | 都在空中"硬掉电/硬复位"，**没走到上锁那一步** → 文件没关闭 → FAT 里没有目录项 → **在电脑上看就是"没有"** |

**所以"下午的日志不见"本身就是"硬掉电/硬复位"的证据**：如果是"受控的终止飞行→落地→
自动上锁"，日志反而会留下来（`COM_DISARM_LAND=2s` 会自动上锁）。

**抢救办法**：把卡插电脑做一次文件系统检查（Windows `chkdsk X: /f`；Linux `sudo fsck.vfat -a /dev/sdX1`），
会产出 `FILE0000.CHK` 之类文件 → 用 pyulog 打开，日志开头的文字消息里就有它**自己的文件名**
（`[logger] Opened full log file: /fs/microsd/log/2026-10-10/xx_xx_xx.ulg`）→ 能对上是哪一次。
**恢复概率不低**：飞控重启后再没飞过，也就没有新写入覆盖那些块。

### 3. 关键结论②：用户说的「PX4 自动断电保护」= **失效检测器（FD）的 Terminate**

参数：`FD_FAIL_R = 60°`、`FD_FAIL_P = 60°`（`FD_FAIL_*_TTRI = 0.3s`）。
源码：`failsafe.cpp:530` —— `fd_critical_failure` → `Action::Terminate`（`cannotBeDeferred`，**立即执行**）。
**即：姿态超过 60° 持续 0.3 秒，飞控立刻切动力**（用户感知为"自动断电"）。
→ 它是**结果**：先有姿态失控，才有它。**不是** `COM_OBL_RC_ACT`（那个值是 4 = **Land 降落**，
枚举 0=Position/1=Altitude/2=Manual/3=Return/4=Land/5=Hold/6=Terminate/7=Disarm）。

> 实测彩蛋：早上那次手飞落地时侧翻到 **−53°**（全段最大横滚 60.3°）——**地面上翻个身就离 60° 阈值不远**，
> 所以"摔了之后断动力"完全可能是 FD 干的，别把它当成原因。

### 4. 关键结论③：这条飞机的**位置/高度/速度/航向全靠 SLAM 一个来源**

| 参数 | 值 | 含义 |
| --- | --- | --- |
| `EKF2_EV_CTRL` | 15 | 位/高/速/**航向** 全部使用 EV |
| `EKF2_HGT_REF` | 3 | 高度基准 = **Vision（EV）** |
| `EKF2_MAG_TYPE` / `SYS_HAS_MAG` | 5 / 0 | 不用磁罗盘（也没装） |
| 实测 | — | 早上手飞全程 `cs_ev_pos / cs_ev_hgt / cs_ev_vel / cs_ev_yaw` **17/17 全 true** ✓ |

→ **一旦 EV 流断掉或被带偏，飞控的航向和高度就没有第二来源**（只剩陀螺积分+气压）。
这解释了为什么"航向估计空中崩掉"这类事故会反复出现，也说明本轮加的
**航向交叉校验**是对症的（EV yaw 与飞控 heading 相对地面基准跑偏 >20° 持续 1s 就中止）。

### 5. 早上那份日志到底是什么（别误读）

**一次正常手飞**，不是炸机：`Armed by RC` → 4.2s `Takeoff detected` → 悬停 7 秒
（POSCTL、电机 ~0.4、姿态 ±2°，**非常稳**）→ 10.5s 落地时侧翻到 −53° → 11.7s `Landing detected`
→ 13.2s `Disarmed by RC`，全程**无任何 failsafe**。
→ 结论：**硬件 + EV + 定点悬停在手动模式下是好的**。但它是手飞，**不覆盖任务/Offboard 那条路**，
所以它单独不能定案下午那次。

其余顺手记下的基线：悬停 3~5A；起飞爬升峰值 **25.9A**，此刻电池 15.7→15.0V（≈27mΩ 内阻，正常）
→ **电源在 26A 下扛得住**；`MPC_YAWRAUTO_MAX = 45°/s` → 180° 掉头本身要转 4 秒，不是"一瞬间猛转"；
`BAT1_CAPACITY = -1` + `COM_LOW_BAT_ACT = 0` → 飞控自带低电保护确实是关的；
`SER_TEL3_BAUD = 1500000` / `UXRCE_DDS_CFG = 103` 与我们的链一致 ✓。

### 6. 下一步（定案路径）

1. **先把卡做文件系统检查**，把下午那两个 `.CHK` 捞出来发我 → 用 `analyze_ulog.py` 看最后 1 秒
   电机输出（是飞控自己下的满油门、还是指令正常）、电压有没有塌、有没有 hardfault、EV 还在不在。
2. 捞不回来就用现在的"前进 1.2m、不掉头"任务复飞一次：**飞完先看卡里有没有新日志**——
   没有 ⇒ 这次又是硬掉电/硬复位（本身就是结论）；有 ⇒ 发我。

---

## 2026-10-10 深夜（第三次炸机，20:42）：**"前进 1.2m 不掉头"任务照样炸 —— 根因抓到现行：2D SLAM 航向崩掉 + EV 桥把它整段停发**

> 用户：「刚才又炸了一次，你分析一下，然后同步到日志」，附 `log_79_UnknownDate.ulg`。

### 0. 结论速览

**这次和"掉头"完全无关**（任务已经改成不掉头了），事故点仍然是**"前进段刚到位"那一刻**。
根因链条（全部有原始日志佐证）：

1. **前进段结束时飞机要减速 → 机体俯仰 → 2D 激光 SLAM（`fly_no_imu.lua`，无 IMU）的航向估计崩掉**：
   `20:42:05.24` 我新加的航向校验就报 `飞控 88.8° / EV 142.8°（偏差 53.9°）`；
   EV 桥随后在自己的阈值（45°/帧）上炸锅：`20:42:05.57` 起 **每秒 20+ 次** `SLAM 位姿跳变：航向 45~137°`，
   一直持续到 `20:42:39`（`reset_counter` 累到 250+ 后归零重来）。
2. **EV 桥的反应是"暂停发 EV 1.0s"**（本意是让 EKF2 干净重接）→ 但对这条飞机来说，
   EV 是位置/高度/航向**唯一**来源（`EKF2_EV_CTRL=15`、无磁罗盘，实测 `cs_ev_*` 全 true）
   → **暂停发 EV = 让飞控失明**，而且这次是每秒钟触发好几次 → 等于持续失明。
3. **飞机同时被推着往上蹿**：TOF 直读 `0.5m → 1.29m（被接受）→ 2.10m（被当跳变丢弃）`，
   任务节点侧也看到 `定高器 2.19m / 飞控 0.50m`（倾斜才 7°，所以不是倾斜伪值，是真在爬）
   → 电机在猛推（用户听到的"暴动+大声响"）。
4. **飞控在 20:42:05.44 断链**（lpos/status 停更），agent 在 **20:42:10 记到 `session re-established`**，
   **20:42:17** 飞控（新开机的状态）报 `nav_state=14 / 已解锁→未解锁` → **飞控整机重启**。
5. 坠落 → 姿态超 60° 持续 0.3s → **FD 触发 `Action::Terminate`（= 用户说的"自动断电保护"）**。

### 1. 时间线（20:41:53~20:42:17，三方日志对齐）

| 时刻 | 来源 | 内容 |
| --- | --- | --- |
| 20:41:53.24 | 任务节点 | 就绪（`dry_run=off`，前进 1.20m、掉头 0°、不返航） |
| 20:41:56.34 | 任务节点 | 高度基准 0.04m / 航向基准 −0.1° / 参考点 (0.03,−0.04,−0.03) 航向 90.0° |
| 20:41:58.44 | 任务节点 | `SET_ARM_MODE` → 飞控已允许解锁（nav_state=14）→ ARMING |
| 20:41:58.85 | 任务节点 | 命令 400 回执 `result=0` → 已解锁 → 进 Offboard → TAKEOFF |
| 20:42:02.33 | EV 桥 | `NED pos(-0.10, 0.45, -0.35) yaw 87.5°` ← 前进中，一切正常 |
| 20:42:02.54 | 任务节点 | 到达目标高度 0.35m → MISSION_FORWARD |
| **20:42:04.64** | 任务节点 | **前进段到位**（水平误差 0.109m）→ 不打掉头，进 HOVER |
| **20:42:05.24** | 任务节点 | ⚠️ **航向对不上：飞控 88.8° / EV 142.8°（偏差 53.9°，门限 20°）** |
| 20:42:05.31 | EV 桥 | ⚠️ 高度读数 **2.10m** 偏离有效值 1.29m —— 丢弃 |
| 20:42:05.34 | 任务节点 | ⚠️ 高度对不上：定高器 **2.19m** / 飞控 0.50m（差 1.65m，倾斜 7°） |
| 20:42:05.44 | 任务节点 | ❌ 任务中止：飞控链路中断（lpos 超时；vehicle_status 已 1.02s 未更新） |
| 20:42:05.57→39 | EV 桥 | ❌ **每秒 20+ 次** `SLAM 位姿跳变：航向 45°~137°`，反复"暂停发 EV 1.0s" |
| 20:42:10 | agent | `session re-established` ← 飞控重新连上（= 重启） |
| 20:42:17.87 | 任务节点 | ❌ 飞控由「已解锁」变成「未解锁」（nav_state=14）← 新开机状态 |

### 2. 这次 ulog 为什么没用上

`log_79_UnknownDate.ulg`（291KB）解析结果：`file_corruption = True`、`data_list = 0`、
顺序扫描**只干净解析到第 56 字节**（只有头 + 一个 `B` 标志消息），其余全是无效数据；
只读到 4 条文字消息（含 `Opened full log file: /fs/microsd/log/sess129/log100.ulg`、`Takeoff detected`）。

两个信息点：

- **数据没刷盘**：日志内容基本是空的 ⇒ 飞控在写入被刷进卡之前就掉电了 ⇒ **又一次"硬掉电"的旁证**
  （与 `session re-established` + 重启后报 disarmed 一致）。
- 路径是 `log/sess129/log100.ulg`（"sessNNN" 而不是日期目录）⇒ **这次飞行时飞控时钟没同步**（没有有效时间）。

### 3. 三名嫌疑人现在的排序（第三次炸机后）

| 机制 | 把握 | 依据 / 反证 |
| --- | --- | --- |
| **① 2D SLAM 航向崩 → EV 桥整段停发 → 飞控失去唯一位置/高度/航向来源 → 控制失控** | **最强** | EV 桥自己的日志（45~137° 连续跳变、反复停发）、我的航向校验抓到 54°、TOF 显示真爬升；且与"前进段刚到位"精确同步 |
| ② 飞控供电在电机猛推时被拉垮（掉电→重启） | 中 | 日志没刷盘 + 会话重建 + 重启后 disarmed 都是"硬掉电"证据；但**这更像是①的结果**（猛推→大电流） |
| ③ 物理撞击 | 低 | 这次没有撞击描述；且所有异常都发生在飞控失控之前/之中 |

**关于"为什么总在前进段结束时"**：那一刻飞机要**减速**，机体必然俯仰；2D 激光 SLAM 假设世界是平的，
俯仰会让扫描平面倾斜 → 扫描匹配把"倾斜"解释成"旋转+平移" → **航向估计崩掉**。
这也解释了 10-08 那次"航向估计空中崩掉"、以及 17:52 和 10:27 两次都死在同一点。

### 4. 下一步（按优先级；①③④是我这边能做的，②要用户确认）

1. **SLAM 必须带 IMU（治本）**：cartographer 的 `fly.lua`（`use_imu_data=true`）一直都在，
   但 `run.sh` 需要显式 `SLAM_USE_IMU=1`，且**当前 `/imu` 不合格**：
   实测 `orientation = 单位四元数`、`orientation_covariance[0] = -1`（= "不提供姿态"），
   而 cartographer 用它做重力对齐/去俯仰 ⇒ **必须先让 IMU 桥发有效姿态**
   （从 `/fmu/out/vehicle_attitude` 取，按 body→laser_link 正确换算，带正确 covariance），
   地面转动飞控验证后再切 `SLAM_USE_IMU=1`。
2. **EV 桥两处改**：
   a. TOF 高度**按倾斜补偿**（`h = tof·cos(tilt)`，tilt 取 `/fmu/out/vehicle_attitude`）——
      这样"跳变判据"不用猜真假，**真高度也不会被误丢**（这次就是真高度被丢，飞控高度回路瞎了）。
   b. **取消"跳变后暂停发 EV 1 秒"**：对这条只有 EV 一个来源的飞机，停发 = 让飞控失明；
      应改为"只拒绝/降权跳变那一帧"，绝不能整段静默。
3. **任务节点加硬阈值**（这次的 54°/2.19m 都属于"必须立刻停"的量级）：
   航向偏差 >40° 或 TOF 与飞控高度差 >1.5m → **立即**中止（不等 `xcheck_hold` 的 1s）；
   另加"EV 停发监控"（`/fmu/in/vehicle_visual_odometry` 频率掉到 0 就中止）。
4. **供电仍要查**（②）：这次飞控是掉电重启，说明猛推时的电流/供电裕度不足。

> 本轮新增/更新的工具：`tools/analyze_ulog.py`（本文件 §前一条已记录用法）。

---

## 2026-10-10 收尾：**文档"双端同步"补齐 + 备份快照**

> 用户提醒：「日志同步是双端的，你备份好了吗」

核查结果：**板上原先根本没有 `CHANGELOG.md` / `AGENTS.md`**（`~/px4_ws/docs/` 里只有队友的
`串口雷达改造.md`），本机也没有任何 `.bak` 快照 —— 也就是说"双端"这条一直是断的。

本轮的处置（规则写进 `AGENTS.md` §7.1）：

1. **定正本**：`D:\px4无人机\CHANGELOG.md` / `AGENTS.md` 为唯一正本；
   板上 `~/px4_ws/docs/` 是**只读镜像**，谁在板上改都会被下次推送覆盖（规则里写明，
   并要求队友若在板上改过，先拉回来合并）。
2. **`SSH_ACCESS.md` 不推到板上**（含明文口令）。
3. **留日期快照**：本机 `CHANGELOG.md.bak-20261010` / `AGENTS.md.bak-20261010` /
   `SSH_ACCESS.md.bak-20261010`；板上镜像同留 `.bak-20261010`。
   （说明：今天之前没有更早的快照可用，但本文件是**追加式**的，逐条日期分节自包含，
   回滚粒度到"删掉某一节"即可，风险低；**从今天起按上面的规则留快照**。）
4. **两端哈希对比**通过（数值见 `AGENTS.md` §7.1 的记录方式，或直接 `md5sum` 复核）。

---
## 2026-10-12 晚：**登机核对 10-11 的改动 + 板端整体备份 + 双端同步；p_to_p 移植待源**

> 用户原话（本轮指令）：「我现在需要你先备份现在香橙派上的原文件到香橙派上然后沿用我上面给你发的那个_ws的逻辑来用到我的无人机上，
> 以上所有操作均要记录到双端日志上，然后一后每次改动都得加上日志记录，保留这个约束。」

### 1. 核对结果：10-11 确实被改过（本会话只读核对，未动机载端）

方法：两端逐文件 md5（板上 372 / 本机 329，同名 320）+ 文档镜像哈希 + systemd/进程状态。

**（a）唯一的代码改动 —— `px4_ev_bridge` 被回滚（板上 mtime 2026-10-11 20:39）**

- `src/px4_ev_bridge/src/px4_ev_bridge_node.cpp` + `launch/ev_bridge.launch.py`：
  **删掉航向跳变检测**（`yaw_jump_deg` 参数、`dyaw > yaw_jump_deg_` 判据、对应日志），
  恢复成"只查位置台阶 > `reset_jump_m`（1.0 m）才 `reset_counter++` 并停发 EV 1 s"；
  launch 里 `yaw_jump_deg` 换成暴露 `reset_jump_m` / `jump_pause_sec`。
- 板上注释给的理由：对称环境里 2D SLAM 航向频繁 180° 跳，带航向判据时每跳都 reset + 停发 EV 1 s，
  而 EV 同时承载高度（`EKF2_HGT_REF=3`）⇒ EKF2 每次同时丢水平位置和高度参考，
  **实测 30 秒触发 223 次**，比航向跳变本身更危险。
- ⚠️ 这与本文件 2026-10-10 深夜的结论方向不同：10-10 的药方是"把停发 1 s 改成只拒一帧"，
  治本要"SLAM 带 IMU"。**"2D 激光 SLAM 没 IMU 时航向会崩"这个根因仍然没解决。**
  两版差异已留档：本机 `*.bak-20261012`（= 带航向判据那版）。

**（b）链路看门狗被停用（10-11 20:40）**

- `tools/chain_watchdog.sh` → `tools/chain_watchdog.sh.disabled-20261011_204002`；
  `px4-chain-watch.service` 现为 disabled / inactive
  ⇒ 本机 `run.sh` 里"组件失败交给 px4-chain-watch 分段恢复"这条**现在是关着的**。

**（c）10-11 的两处现场（留在板上）**

- `~/backups/20261011_203907_before_rollback/`（回滚前快照）
- `~/flight_records/20261011_crash_1757/`（**10-11 17:57 又坠机一次**；cartographer 日志 20 MB）

**（d）核对通过**

- 板上 `docs/CHANGELOG.md` = `70b47ead…`、`docs/AGENTS.md` = `292fd4e8…`，与本机正本逐字节一致
  ⇒ 没有人在板上改过文档镜像（§7.1 的检查项）。
- 除上述 2 个 EV 文件外，其余 320 个同名文件内容**完全一致**。

**（e）登录时的状态**：uptime 2 分钟（2026-10-12 20:18:43 上电）；`px4-ev-chain` active；
  MicroXRCEAgent(`ttyS6` @ **1500000**)、`px4_imu_bridge.py`、tofsense_m、bluesea2(uart)、
  `tf_to_pose.py`、px4_ev_bridge、cartographer（**`fly_no_imu.lua`**，纯激光）全在跑。
  板上**没有** `onboard_ws` / `p_to_p_flight_px4` / `h30_ws`。

### 2. 板端备份（本轮新增）—— **备份放在哪（一览 + 以后的约定）**

> - **板端备份**一律 `~/backups/<时间戳>_<用途>/`；本次 = `~/backups/20261012_203137_before_p2p_port/`
>   （内部布局：`px4_ws-full-src.tar.gz` + `etc/` + `home/`）。
> - **本机快照**一律"同目录同名 + `.bak-<YYYYMMDD>`"；本次 =
>   `D:\px4无人机\CHANGELOG.md.bak-20261012`、`D:\px4无人机\AGENTS.md.bak-20261012`、
>   `D:\px4无人机\px4_ws\src\px4_ev_bridge\src\px4_ev_bridge_node.cpp.bak-20261012`、
>   `D:\px4无人机\px4_ws\src\px4_ev_bridge\launch\ev_bridge.launch.py.bak-20261012`。
> - **板上文档镜像快照**：`~/px4_ws/docs/CHANGELOG.md.bak-20261012`、`…/AGENTS.md.bak-20261012`。
> - **用户给的参考资料**固化到 `D:\px4无人机\ref_<名字>\`（本次 `onboard_ws` 没来得及固化，
>   临时目录被清理，见 §4 —— 所以这条要从"收到就固化"做起）。
> - 临时下载件（`%TEMP%\…`、板端 `/tmp`）**不算留档**，会被清理；要留就拷到上面这些固定位置。

`~/backups/20261012_203137_before_p2p_port/`（10 个文件，500 KB）：

| 内容 | 说明 |
| --- | --- |
| `px4_ws-full-src.tar.gz` | `src/ tools/ systemd/ docs/` + 顶层脚本（不含 build/install/log）；sha256 `0f6772797eddffd1a4ba9e04cb605bdca4369196f201d458eaa95ec6fbb5d98e` |
| `etc/` | `px4-ev-chain.service`、`px4-chain-watch.service`、`microxrce-agent.service`、`microxrce-agent`(defaults)、`u-boot`、`fastdds-udp.sh` |
| `home/` | `.bashrc`、`px4_imu_bridge.py`、`tf_to_pose.py` |

回滚：`tar -xzf …/px4_ws-full-src.tar.gz -C ~/px4_ws`；只取单个文件用
`tar -xzf … --wildcards '*/<相对路径>'`。

### 3. 同步回本机（§7.1 的"拉回合并"）

- 覆盖 `src/px4_ev_bridge/{src/px4_ev_bridge_node.cpp, launch/ev_bridge.launch.py}` ← 板上版；
  本机旧版留 `.bak-20261012`（node 29028 B、launch 7080 B）。覆盖后 md5 与板上一致
  （`277016d1…`、`7827b2f8…`）。
- 补回本机缺的 3 个板上文件：`tools/lidar_watchdog.sh`、`systemd/px4-lidar-watch.service`、
  `docs/README_镜像说明.md`。（板上这两个 lidar 看门狗文件其实还留着，只是没装、没 enable。）

### 4. `onboard_ws`（p_to_p 点对点）移植 —— **卡在缺源**

- 你上一条消息附的 `onboard_ws` 是从 360 压缩包临时目录
  （`%TEMP%\360zip$Temp\360$1\onboard_ws`）读的；本轮想把它固化到 `D:\px4无人机\ref_onboard_ws`
  时发现**临时目录已被系统清理**，全盘（C:/D:）也搜不到第二份 ⇒ **源文件没了，需要重发。**
- 教训：以后收到参考资料，**第一步先固化到项目目录**，不要留在临时目录里干活。

### 5. 约束（用户 2026-10-12 再次明确，重申保留）

> 「以上所有操作均要记录到双端日志上，然后一后每次改动都得加上日志记录，保留这个约束。」

⇒ 即日起：**任何改动**（备份、同步、移植、脚本/服务/参数/文档改动）都要写进本文件，
并在当天把 `CHANGELOG.md` + `AGENTS.md` 推到板上 `~/px4_ws/docs/`，两端 `md5sum` 核对。
正文见 `AGENTS.md` §7.1 / §8.1 / §11。

---
## 2026-10-12 晚（补记）：**把"备份放在哪"写进日志（用户要求）**

> 用户原话：「还有你备份之后放在哪记得也在日志说在哪」

- 在本文件 2026-10-12 那节的 §2 补了「备份放在哪（一览 + 以后的约定）」，写明本轮所有产物的确切路径：
  板端 `~/backups/20261012_203137_before_p2p_port/`（tar + etc + home）、本机四个 `*.bak-20261012` 快照、
  板上镜像 `~/px4_ws/docs/{CHANGELOG,AGENTS}.md.bak-20261012`。
- 在 `AGENTS.md` §7.1 升格成**约定**：以后每次备份/固化，都要在 `CHANGELOG.md` 里写明本轮产物的确切路径。
- 本轮**只改文档**（未动代码/服务/参数）；本机快照 `*.bak-20261012b` 保留改前版本；
  两端 `md5sum` 已核对一致（数值见会话记录）。

---
## 2026-10-12 晚（补记 2）：**重发收到：`onboard_ws` 已固化 + 双端镜像核对通过**

- 用户原话：「刚才发错了」——上一轮附的 `shadow-swipe.html`（网页小游戏，与无人机无关）作废；
  重发的是 `onboard_ws.zip`。
- **本轮产物（固化位置）**：`D:\px4无人机\ref_onboard_ws\`
  - 原始压缩包 `ref_onboard_ws\onboard_ws.zip`：md5 `f0fb2448ab089478463e6e2e4b704807`，555917 B，
    312 个条目，解压后 1,299,449 B，包内时间戳 2026-10-11 17:30
  - 解压源码 `ref_onboard_ws\onboard_ws\src\`：8 个包 / 253 个文件（含 73 个 `__pycache__` 垃圾）
  - 来源与用法说明 `ref_onboard_ws\说明.md`
  - 与 2026-10-12 早些时候那个临时副本**关键文件字节数逐一相同**，这次已落到固定目录，不会再随临时目录消失。
- **文档镜像核对**：上一轮推送其实已完成 —— 板上 `~/px4_ws/docs/CHANGELOG.md` = `986f0215…`、
  `AGENTS.md` = `a7bcc86e…`，与本机正本一致；`.bak-20261012b` 也已在板上。
- 本轮只做"收件 + 固化 + 核对 + 记录"，**未动机载端**。

---
## 2026-10-12 晚（补记 3）：**p_to_p 点对点任务移植落地（第一步：新增两包 + 编译通过 + 30 秒 bench 预演）**

> 用户 2026-10-12 批准口径：「按你说的来，逻辑用给你发的这个」（= 用 px4_msgs 把 `onboard_ws` 的
> A→B 逻辑移植进 `px4_ws`，不搬它那层 STM32 `0x31/0x32` 串口控制）。

### 1. 改了什么（新增，未覆盖任何现有文件）

| 项 | 内容 |
| --- | --- |
| 新包 1 | `px4_ws/src/p_to_p_mission`（任务状态机，**代码基本原样**：`mission_node.py` 31418 B + `geometry/landing/preflight/task_guard` + `test/`） |
| 新包 2 | `px4_ws/src/p_to_p_px4`（**精简**到 `frames.py` / `position_ramp.py` / `offboard_bridge.py` + `config/px4.yaml`） |
| 新启动 | `p_to_p_mission/launch/p_to_p.launch.py`（只起"任务 + 桥"；雷达/SLAM/EV 仍由 `run.sh`/`px4-ev-chain` 负责） |
| 新配置 | `p_to_p_mission/config/p_to_p.yaml`、`p_to_p_px4/config/px4.yaml` |

**删掉了哪些参考件模块（及理由）**

- `usb_odometry_transport.py`、`tof_mavlink_bridge.py`：USB MAVLink 备用通路，本机走 uXRCE-DDS；
- `odometry_bridge.py`、`odometry_output.py`、`lidar_velocity.py`：**本机 EV 由 `px4_ev_bridge` 负责，
  两处同时发 `/fmu/in/vehicle_visual_odometry` 是禁止的**，所以整条 EV 支路不搬；
- `three_axis_preview.py`、`range_preview.py`、`sample_alignment.py`、`tofsense.py`：那套 TOF/三轴可视化是给 USB 通路的；
- `deployment/`（systemd 模板引用不存在的 `scripts/`）、`launch/`（引用不存在脚本）、`test/`（测的是已删模块）。

**配置改动（相对参考件）**

- `p_to_p.yaml`：删 `position_pid_controller` / `uart_to_stm32_node` 两节；`cruise_height_cm: 50`、
  `b_offset_x_cm: 100`（正前方 1 m）、`terminal_behavior: hold`（不自动落地）、
  包线收紧 `task_radius_cm 150 / task_max_height_cm 120 / 各段超时 20 s`、`require_rc_offboard: true`。
- `px4.yaml`：删 `cartographer_odometry_bridge` 一节；`bench_only: true`、`allow_mode_command: false`、
  `allow_land_command: false`、`land_on_mission_complete: false`（参考件是 true → 收紧）。
- `package.xml` / `setup.py`：去掉 `uart_to_stm32` / `pid_control_pkg` / `my_carto_pkg` / `bluesea2` 依赖；
  入口只留 `px4_offboard_bridge`。

### 2. 改前备份 / 怎么回滚

- 本次改动**发生在** `~/backups/20261012_203137_before_p2p_port/` **之后**，那个备份就是"改前状态"；
- 回滚 = `rm -rf ~/px4_ws/src/{p_to_p_mission,p_to_p_px4}`（或从该备份恢复 `src/`），再 `colcon build --packages-select ...`。

### 3. 怎么验证（2026-10-12 20:5x 板上实测）

1. **语法/配置**：`python3 -m py_compile`（11 个 py 全过，`PY_SYNTAX_OK`）+ `yaml.safe_load` 两份配置通过。
2. **编译**：`colcon build --symlink-install --packages-select p_to_p_mission p_to_p_px4`
   → `Summary: 2 packages finished [5.67s]`。
3. **30 秒 bench 预演**（`ros2 launch p_to_p_mission p_to_p.launch.py`，bench_only 默认 true）：
   - 两个节点都起来了；`/p_to_p/state` = `IDLE: 等待定位、测高与人工启动`；
   - `/p_to_p/readiness_check` → `success=True, '任务启动条件就绪（未启动任务）'`；
   - 桥日志：`PX4 bridge started in BENCH/只读 mode. This node never sends arm/disarm commands.`；
   - **代码层确认**：`_timer_callback` 在 `bench_only` 时**先 return**，不会执行
     `_publish_offboard_control_mode()` / `_publish_trajectory_setpoint()`
     ⇒ `/fmu/in/*` 上只有 DDS 端点、没有任何消息；
   - 跑完按进程名精确清掉，`pgrep` 无残留。
4. ⚠️ **预演时的一条告警（不是代码问题）**：桥报
   `TF map->base_link unavailable: "map" ... does not exist` ——
   手工起节点这次没和飞行链路用同一条 FastDDS 传输（本机已知的 SHM/UDP 坑）。
   下次要么在 `run.sh fly` 那套环境里起，要么确认 `FASTRTPS_DEFAULT_PROFILES_FILE` 一致。

### 4. 还没做（下一步，按优先级）

1. 飞控上电后确认桥发的 `/height`（来自 `local_position.dist_bottom`）有数据、量纲对（Int16 厘米）；
2. 带 `bench_only:=false` 的**地面**流程演练：遥控器解锁 → 切 Offboard → 观察
   `OffboardControlMode/TrajectorySetpoint` 是否 20 Hz、位置斜坡是否限速（**不装桨**）；
3. `terminal_behavior:=guided_land_handoff`（限速下降 + 交还 PX4 Land）单独验证；
4. 任务的"到点判定/超时/包线"用 `/p_to_p/state` 与实际飞行轨迹逐段对账。

---
## 2026-10-12 晚（补记 4）：**p_to_p 移植后的"下次照着做"操作清单（runbook）+ 板上就位确认**

> 用户 2026-10-12：先把新文件放进香橙派、把后续操作写进板上日志（用户随后离线）。

### 0. 板上就位确认（已实测，2026-10-12 21:0x）

- `~/px4_ws/src/p_to_p_mission`、`~/px4_ws/src/p_to_p_px4` 各就位（两棵树共 40 个文件）；
- `install/p_to_p_mission/lib/p_to_p_mission/p_to_p_mission_node`、
  `install/p_to_p_px4/lib/p_to_p_px4/px4_offboard_bridge` 都在；
  `ros2 pkg executables p_to_p_mission|p_to_p_px4` 都能列出；
  配置/启动已装到 `install/*/share/*/{config,launch}`；
- 目录权限已统一成 `drwxrwxr-x`（本轮 scp 带过去的 0705 → 改为 0775，只做加法）。

### 1. 第一步：先确认现有链路（只读，不动）

```bash
bash ~/px4_ws/tools/preflight.sh                       # 逐项体检（22 项）
python3 ~/px4_ws/tools/watch_fc_link.py --minutes 5    # 看飞控有没有偷偷重启（正常=0 次）
systemctl is-active px4-ev-chain.service               # 应为 active
ros2 topic hz /scan ; ros2 topic hz /tofsense/height ; ros2 topic hz /fmu/in/vehicle_visual_odometry
```

期望：`/scan` ~15 Hz、`/tofsense/height` ~55 Hz、`/fmu/in/vehicle_visual_odometry` ~15 Hz。

### 2. 第二步：只预演（不解锁、不发控制）

```bash
source /opt/ros/humble/setup.bash; source ~/px4_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml   # 必须和链路同一条传输！
ros2 launch p_to_p_mission p_to_p.launch.py                                 # bench_only 默认 true
```

另开一个终端看：

```bash
ros2 topic echo /p_to_p/state          # 应为 IDLE: 等待定位、测高与人工启动
ros2 topic echo /px4_bridge/state      # BENCH_LINK_OK / BENCH_WAITING + 细节
ros2 service call /p_to_p/readiness_check std_srvs/srv/Trigger {}
ros2 run tf2_ros tf2_echo map base_link   # 必须查得到；查不到=传输没对齐（FastDDS SHM/UDP 坑）
```

判定：桥日志出现 `BENCH/只读 mode`、`/fmu/in/offboard_control_mode` 上没有消息 ⇒ 预演合格。

### 3. 第三步（关键前置）：飞控上电后确认 `/height`

```bash
ros2 topic hz /height ; ros2 topic echo /height --once
```

要求：有数据，且**单位是厘米**（贴地读数 ≈ 地面到测距点的距离）。
**没数据就不要往下走** —— 任务的升降判据全靠它。

### 4. 第四步：拆桨地面演练（第一次真发 Offboard 流）

```bash
ros2 launch p_to_p_mission p_to_p.launch.py bench_only:=false cruise_height_cm:=50 b_offset_x_cm:=0 terminal_behavior:=hold
```

顺序：**先卸桨** → 遥控器解锁（POSCTL）→ 遥控器切 Offboard（任务会自动开始，`auto_start_on_offboard=true`）→ 观察：

```bash
ros2 topic hz /fmu/in/trajectory_setpoint      # 应 ~20 Hz
ros2 topic hz /fmu/in/offboard_control_mode    # 应 ~20 Hz
ros2 topic echo /p_to_p/state                  # IDLE → TAKEOFF → GOTO_B(=原地) → HOLD_B
```

`b_offset_x_cm:=0` 时飞机应当"原地起飞到 0.5 m 再悬停"——这是最保守的第一枪。
中止手段（任选）：遥控器切回 POSCTL / Land；或 `ros2 service call /p_to_p/stop_output std_srvs/srv/Trigger {}`。
桥检测到掉出 Offboard 会自己停输出（`_abort_offboard_output`）。

### 5. 第五步：一次只放开一个变量

1. `b_offset_x_cm` 0 → 50 → 100（前进距离）；
2. `cruise_height_cm` 50 → 80 → 120（上限，包线 `task_max_height_cm=120` 挡着）；
3. 最后才试 `terminal_behavior:=guided_land_handoff`（限速下降 + 交还 PX4 Land）。
**每改一次都要在本文件加一条**（改了什么/为什么/怎么验证/怎么回滚），并推到板上镜像。

### 6. 回滚

```bash
# 停任务栈：Ctrl-C，或
ros2 service call /p_to_p/stop_output std_srvs/srv/Trigger {}
# 只回滚这两个新包（最常用）
rm -rf ~/px4_ws/src/p_to_p_mission ~/px4_ws/src/p_to_p_px4
cd ~/px4_ws && colcon build --symlink-install --packages-select p_to_p_mission p_to_p_px4   # 或整个 src 重编
# 整机回到 2026-10-12 20:31 状态
tar -xzf ~/backups/20261012_203137_before_p2p_port/px4_ws-full-src.tar.gz -C ~/px4_ws
```

### 7. 红线（不要做）

- **不要**在 `px4_ev_bridge` 之外再引入第二个 `/fmu/in/vehicle_visual_odometry` 发布者
  —— 这就是参考件的 EV 支路没搬过来的原因；
- **不要**同时跑 `px4_bridge` / `px4_takeoff_land`（它们也发 `/fmu/in/offboard_control_mode`，会抢）；
- **不要**跳过 `tools/preflight.sh` 直接飞；**不要**在没有遥控器时试飞；
- 包线（1.5 m / 1.2 m）是**节点本地限制**，不是飞控 geofence，别当成硬保护。

### 8. 已知观察点（飞之前心里有数）

- `/height` 的取值与量纲**尚未在真机确认**（依赖飞控 `dist_bottom`）；
- 桥的 `map↔PX4` 锚点是**进入 Offboard 那一刻**抓的：中途 SLAM 重定位会让目标漂移，
  目前只有"位置台阶 > 1 m → reset + 停发 1 s"这一道保护（10-11 板上回滚后的行为）；
- 根因（2D 激光 SLAM 没 IMU 时航向会崩）**仍未解决**，所以第一枪必须是"原地 + 低高度 + 有桨保护"。

---
## 2026-10-12 晚（补记 5）：**p_to_p 移植「还没做」第 4 条 —— 逐段对账工具落地**

> 用户 2026-10-12：「他完成了一部分 现在还差一部分 需要你补做第四条」
> ⇒ 对应本节前方 §4「还没做（下一步，按优先级）」的第 4 项：
> 「任务的『到点判定/超时/包线』用 `/p_to_p/state` 与实际飞行轨迹逐段对账」。

### 1. 改了什么（**只新增一个工具，未动任何现有代码/配置/服务/参数**）

| 项 | 内容 |
| --- | --- |
| 新增 | `px4_ws/tools/p2p_audit.py`（约 480 行，**只读**：只订阅，不发布/不调服务/不解锁） |

**它做什么**：用**同一把尺子**（直接 `import p_to_p_mission.{geometry,task_guard,landing}`，
即任务节点自己那两个判据函数）把同一时刻的**实际轨迹**
（TF `map→laser_link` + `/height`(cm) + `/fmu/out/vehicle_local_position`）记下来，
按段（TAKEOFF/GOTO_B/HOLD_B/DESCEND_B/FC_LANDING/各 FAULT）算：
```text
目标 vs 实际 —— 三个误差 vs 三个容差 —— 持续时间 vs 该段超时 —— 包线(半径/高度)
```
并标出「节点判定 vs 审计判定」不一致处：假到位 / 漏切 / 超时未生效 / 包线未生效 /
段内传感器超时 / 下降段目标回抬 / 下降超速 / 水平误差超容差 3 倍。

**关键设计点（写清楚免得后人改错）**
- `z` 必须取 **`/height`（cm，AGL）**，与任务节点 `_height_callback` 同源；
  **不能用 TF 的 z** —— 2D SLAM 的 z 是常数偏移（URDF 的 -0.1 m），拿它算高度误差是错的。
  （首版就是这样写的，板上实测发现并已改；CSV 里另留 `slz_cm_slam` 列仅作诊断。）
- `A` 点锚定 = 首次进入 `TAKEOFF` 那一拍的实际位置（与节点 `_start_callback` 的 a_pose 同义）。
- 用法：`--duration N` 到点自动出表；不加则 Ctrl-C 出表。
- 输出默认落 **`~/flight_records/p2p_audit_<时间戳>/`**（`samples.csv` + `report.txt`），
  **不放 /tmp**（重启会丢，10-11 已踩过）。
- ⚠️ 必须与链路同传输：`export FASTRTPS_DEFAULT_PROFILES_FILE=~/px4_ws/tools/fastdds_udp_only.xml`，
  否则 TF 查不到（本机已知的 SHM/UDP 坑，见 2026-10-12 补记 3 §4）。

### 2. 怎么验证的（2026-10-12 21:1x 板上实测）

1. `python3 tools/p2p_audit.py --self-test` → **8 项全过**（正常到位 / 假到位 / 超时未生效 /
   包线超限 / 下降回抬 / 传感器超时 / 不穿地、不追向上假读数 / stage_reason 与 Limits 一致）。
2. **真机通路验证**（`--duration 12` + 合成状态序列）：
   - 起 `px4_offboard_bridge`（**bench_only=true**，不发任何 `/fmu/in` 消息）提供 `/height`；
   - `/height` **97.2 Hz，读数 `10`**（= 飞控 `dist_bottom` 约 0.10 m ⇒ **单位确认是 cm**）
     ⇒ **顺手把「还没做」第 1 项验了**：`/height` 有数据、量纲对；
   - 审计正确分段（IDLE→TAKEOFF→GOTO_B→HOLD_B），把"飞机没动但目标 1 m/0.5 m"
     正确标成 **假到位**，并给出「水平误差峰值 100 cm = 容差 15 cm 的 6.7 倍」；
   - `samples.csv` 列数一致（NF=22）；报告落 `~/flight_records/p2p_audit_verify/report.txt`。
3. 清理：审计与桥进程 0 残留；链路（cartographer/ev_bridge 等 4 个节点）未受影响。
4. 既有单测：`python3 -m pytest src/p_to_p_mission/test -q` → **35 passed**。

### 3. 静态对账结论（第 4 条的"设计层"产出，写下来给后面的人）

按现配置（`cruise 50 cm / b_offset_x 100 cm / 容差 xy 15 cm、z 12 cm、yaw 8°、稳定 1.0 s /
包线 半径 150 cm、高 120 cm / 超时 20-20-20-30 s`）逐项对：

| 判据 | 数值 | 结论 |
| --- | --- | --- |
| 到点 xy 容差 | 15 cm | 地面静置 SLAM 抖动 ±1~2 cm ⇒ 够；但**飞行中 SLAM 会滑**（10-11/10-12 都实测到米级跳），靠它当"到了"要小心 |
| 到点 yaw 容差 | **8°** | ⚠️ **最可能误判的一条**：SLAM 航向在对称环境会漂/翻（10-11 实测 30 s 内 223 次 >45° 跳），8° 远严于 SLAM 的可靠度 ⇒ 很可能一直"没到位"→ 20 s 段超时 → FAULT_HOLD（结果安全但任务失败）。**首次台架建议先放宽到 20~30° 或接受 FAULT_HOLD。** |
| 超时预算 | leg 1 m ÷ 0.30 m/s = 3.3 s ≪ 20 s；爬升 0.5 m ÷ 0.25 m/s = 2 s ≪ 20 s；下降 46 cm ÷ 10 cm/s = 4.6 s ≪ 30 s | ✅ 预算充裕 |
| 包线 | 半径 150 cm vs leg 100 cm（余量 50 cm）；高度 120 vs 巡航 50 | ⚠️ 余量只有 50 cm，而 SLAM 位置可能滑 >50 cm ⇒ 可能触发 FAULT_HOLD（安全但失败）。**首次台架可临时 `task_radius_cm:=200`。** |
| 高度来源 | `/height` ← 桥的 `local_position.dist_bottom` | ✅ 已验证有数据、cm 量纲 |
| 下降段 | `next_descent_setpoint_cm` 不穿地、不追向上假读数（自检 7） | ✅ |

> 以上两条 ⚠️ 都是"保守过头导致的假故障"，不是安全漏洞；要不要放宽由用户/队友定。

### 4. 还没做的（**留给真正飞行**）

- 第 4 条的"**实跑对账**"要在真机（拆桨→地面→低空）跑完后，用
  `python3 ~/px4_ws/tools/p2p_audit.py --duration <秒数>` 出的 `report.txt` 逐段核对；
  现在的结论是"工具就绪 + 设计层对账完成"，**不等于**已经拿到真实航迹的对账结果。
- 「还没做」第 2、3 项（`bench_only:=false` 地面流程 / `guided_land_handoff` 验证）
  仍需用户按 runbook 第四、五步做；第 4 条工具正好配套用（与任务节点同时开即可）。

### 5. 回滚

```bash
rm -f ~/px4_ws/tools/p2p_audit.py      # 只删这一个新文件，其余零影响
```
（本工具不装服务、不改配置、不写任何现有文件；删掉即完全回滚。本机镜像同步删除即可。）

### 6. ⚠️ 双端一致性待办（重要）

- 本轮改的是**板上的** `docs/CHANGELOG.md`（把本机镜像 `…\pi_mirror\px4_ws\docs\CHANGELOG.md`
  追加本节后推回板上，md5 见下）；
- 但按 §7.1 约定，**正本在另一台 Windows 机的 `D:\px4无人机\CHANGELOG.md`** ——
  **本会话无法访问那台机器**，所以那一份需要队友把本节原文合并过去，
  否则"双端一致"这条会破。已在会话里如实说明。
- 本轮产物/校验：
  - 板上新文件 `~/px4_ws/tools/p2p_audit.py`
  - 板上验证产物 `~/flight_records/p2p_audit_verify/{samples.csv,report.txt}`
  - 两端 `docs/CHANGELOG.md` md5 见会话记录（推送后核对）

---
## 2026-10-12 晚（补记 6）：**按用户要求"完全照搬"参考件的感知层（EV）—— 代码已就位并预览验证，但未切换**

> 用户 2026-10-12：「完全照搬他们的」。
> 在此之前本机只搬了"任务层"（补记 3），感知层（EV）仍是我们自己的 `px4_ev_bridge`。

### 1. 改了什么（**全部是新增/补齐，未动现有链路**）

| 项 | 内容 |
| --- | --- |
| 新增（照搬，字节一致） | `src/p_to_p_px4/p_to_p_px4/odometry_bridge.py`、`odometry_output.py`、`lidar_velocity.py`（与参考件 md5 相同：`d9e38e57…` / `581551f6…` / `9f6fdda8…`；`frames.py` 本来就是同一份 `7a39611a…`） |
| 新增入口 | `src/p_to_p_px4/launch/final_position.launch.py`（照搬参考件同名入口；只改三处硬件差异：串口 `ttyS6`、波特率 **1500000**、雷达参数 `uart_lidar.yaml`；建图用我们 `carto_pkg`，SLAM 配置用 **amphi.lua**——该文件两边逐字节相同且正是参考件飞行用的那份） |
| `setup.py` | 加回 `cartographer_odometry_bridge` 入口 + 安装 `launch/` 数据文件 |
| `package.xml` | 补 `sensor_msgs`、`tf2_ros`（`odometry_bridge` 需要） |

**参考件的 EV 契约（照搬后的行为，与我们的 `px4_ev_bridge` 完全不同）**
```text
位姿源      : TF odom→base_link（连续局部帧，**不是 map**）
pose_frame  : FRD；position = [x, y, NaN]；position_variance = [0.08², 0.08², 100]
姿态        : 只给 yaw（orientation_variance = [100, 100, (8°)²]）
速度        : [vx, vy, NaN]（窗口最小二乘拟合，velocity_variance=[0.15²,0.15²,100]）
reset       : 来自速度估计器的失效/跳变计数（jump 0.75 m / 45°）
发布门控    : fail-closed（/scan 新鲜、TF 新鲜、时间戳推进、四元数合法，否则整帧不发）
速率        : 30 Hz；连不上 10 s 会自行抛异常退出（交给 systemd 整栈重启）
```

### 2. 怎么验证的（2026-10-12 21:4x 板上，**全程未碰飞控**）

1. `py_compile` 三个新模块 → `PY_SYNTAX_OK`；四个 .py 与参考件 md5 逐一相同。
2. 编译：`Summary: 1 package finished`；`ros2 pkg executables p_to_p_px4` 出现 `cartographer_odometry_bridge`。
3. **预览模式实测**（`-p enabled:=true -p publish_to_px4:=false`，只发 `/px4_bridge/visual_odometry_preview`，不碰 `/fmu/in/*`）：
   - 节点状态 `PUBLISHING`；预览话题 **29.97 Hz**；
   - 报文与上表契约完全一致：`pose_frame=2(FRD)`、`position=[-0.13, 0.35, NaN]`、
     `position_variance=[0.0064, 0.0064, 100]`、`orientation_variance=[100, 100, 0.0195]`；
   - 对照：同一时刻我们的 `/tracked_pose`(map) = (0.006, 0.003) —— **两帧原点不同**，
     正好印证"map 会被位姿图修正、odom 不会"这一点。
4. `ros2 launch p_to_p_px4 final_position.launch.py --show-args` 解析通过
   （默认：`ttyS6` / `1500000` / `uart_lidar.yaml` / `amphi.lua` / `bench_only=false`）。
5. 现有链路未受影响：`px4_ev_bridge` + cartographer(`fly_no_imu.lua`) 仍在跑；
   预览节点已清理，0 残留。

### 3. ⚠️ 为什么**还没有切换**（两个硬前提，必须你们先定）

**（a）高度来源 —— 参考件把高度完全交给飞控，我们没有这个传感器**
- 参考件 EV 不带高度（Z=NaN、垂直方差 100），并要求 **`EKF2_EV_CTRL` 的垂直位置位关闭**；
  它的高度来自**飞控自己的测距**（`local_position.dist_bottom`），任务读的是那个 `/height`。
- 我们的 ToFSense-M 是**挂在香派上**的（`/dev/ttyS3`），现在的 EV 就是靠它给 EKF 供高度
  （`EKF2_HGT_REF=3` + `cs_ev_hgt=true`）。
- ⇒ 如果**只换 EV 不动飞控参数**：PX4 的视觉高度源会消失 → 高度退回气压计（漂移大），
  任务的"到点/降落"高度判据（12 cm / 3 cm）基本站不住；**必须两边同时改**。
- 三个选择（需用户拍板）：① 把 ToFSense-M 改接到飞控 UART + 配 PX4 测距（最接近参考件，要动硬件）；
  ② 高度改用气压计（零硬件改动，精度差）；③ 保留"派 TOF → EV 高度"这一小段（= 混合，不是完全照搬）。

**（b）飞控参数 —— 只能在 QGC 里改，本会话无法通过 DDS 写参数**
建议值（改完读回核对）：
```text
EKF2_EV_CTRL   = 9      # bit0 水平位置 + bit3 航向；**bit1 垂直位置必须关**（参考件要求）
EKF2_HGT_REF   = 2      # ★ 更正：**2 = Range sensor**（本文件第 70 行早就记过 1=GPS！）
                        #   1.15.4 的枚举是 0=Baro / 1=GPS / 2=Range / 3=Vision
EKF2_EV_NOISE_MD = 0    # 参考件要求：噪声取报文方差
EKF2_OF_CTRL   = 0      # 参考件要求
（速度通路要定制固件才真融合 —— 参考件 LIDAR_VELOCITY.md 明说；stock 固件下 VZ=NaN 会被忽略，无害）
```

### 4. 切换步骤（等上面两条定下来，一次做完；现在**不要**执行）

```bash
# 1) 停掉现有链路（避免两个 EV 发布者 + 资源互抢）
sudo systemctl stop px4-ev-chain
bash ~/px4_ws/run.sh stop
# 2) 按参考件起整链（默认 bench_only=false；先预演就加 bench_only:=true）
ros2 launch p_to_p_px4 final_position.launch.py
# 3) 配套：
#    ros2 topic hz /fmu/in/vehicle_visual_odometry      # 应 ~30 Hz（现在是我们 15 Hz 的桥）
#    ros2 topic echo /px4_bridge/odometry_state --once  # PUBLISHING
#    python3 ~/px4_ws/tools/p2p_audit.py --duration 600 # 逐段对账（补记 5 的工具）
# 4) 回滚：停掉本 launch → sudo systemctl start px4-ev-chain（回到 px4_ev_bridge 那套）
```

### 5. 回滚（只回滚本轮的"新增"）

```bash
rm -f ~/px4_ws/src/p_to_p_px4/p_to_p_px4/{odometry_bridge,odometry_output,lidar_velocity}.py
rm -rf ~/px4_ws/src/p_to_p_px4/launch
# setup.py / package.xml 从 ~/backups/20261012_203137_before_p2p_port/px4_ws-full-src.tar.gz 取回
cd ~/px4_ws && colcon build --symlink-install --packages-select p_to_p_px4
```

### 6. 本轮产物 / 一致性

- 板上新增：`src/p_to_p_px4/p_to_p_px4/{odometry_bridge,odometry_output,lidar_velocity}.py`、
  `src/p_to_p_px4/launch/final_position.launch.py`；改动 `setup.py`、`package.xml`；
  板上镜像同步删改前快照见会话记录。
- 双端 `docs/CHANGELOG.md` md5 见会话记录（推送后核对）。
- ⚠️ 同补记 5 §6：本机"正本"在另一台机器的 `D:\px4无人机\CHANGELOG.md`，本会话无法访问，
  需队友把补记 5、6 两节合并过去。

---
## 2026-10-12 晚（补记 7）：**⚠️ 更正补记 6 的一个错误结论 —— 定高是接在香橙派上的；顺带把"定高链路"也照搬过来**

> 用户 2026-10-12：「不对呀 他说是吧定高接给香橙派的呀 你再好好检查检查」
> **用户是对的，补记 6 §3(a) 我判错了。** 当时我把 `tof_mavlink_bridge.py` / `tofsense.py`
> 当成"USB 通路附属"跳过了，没细看 —— 那正是定高链路本体。

### 1. 更正：参考件的定高链路（原文证据）

`p_to_p_px4/tof_mavlink_bridge.py` 第 1 行就写着
`"""TOFSense-M UART input to one-way MAVLink range output."""`，代码里：
```python
source = os.open('/dev/ttyS3', ...)          # ← TOF 接在**香橙派**上（与我们接线完全相同）
output = os.open('/dev/ttyS1', ...)          # ← 从香橙派另一个串口发给飞控
configure(source, termios.B921600)           # TOF 侧 921600
configure(output, termios.B57600)            # 飞控侧 57600
packet = distance_sensor_v1(distance, ...)   # 编成 MAVLink v1 DISTANCE_SENSOR
os.write(output, packet)                     # 一路写出去给飞控当"自带测距"用
ipc.sendto(tof_ipc_packet(...), ('127.0.0.1', 14581))   # 同时给本地预览一份
```
⇒ **完整链路是**：
```text
TOFSense-M --(UART 921600)--> 香橙派 /dev/ttyS3
        -> tof_mavlink_bridge（派上）
        -> MAVLink DISTANCE_SENSOR --(/dev/ttyS1 @57600)--> 飞控 MAVLink 口
        -> PX4 distance_sensor -> EKF2 测距高度 -> local_position.dist_bottom
        -> offboard_bridge 转成 /height(Int16 cm) -> 任务
```
**不是**"飞控自带测距传感器"。补记 6 里"要把 ToFSense 改接到飞控"的结论作废。
旁证：`test/test_deployment_profile.py` 引用他们自己的
`deploy/systemd/p-to-p-tofsense-height.service` —— 说明这条链路有独立的开机服务。

### 2. 本轮新照搬的文件（全部与参考件 md5 一致）

| 文件 | md5 | 说明 |
| --- | --- | --- |
| `p_to_p_px4/tofsense.py` | `70d093b168b0` | TOFSense-M 解码 + Kalman + MAVLink v1 DISTANCE_SENSOR 编码 |
| `p_to_p_px4/tof_mavlink_bridge.py` | `a9be2352a109` | 定高链路本体（ttyS3 → ttyS1） |
| `p_to_p_px4/range_preview.py` | `f2f0144b7c73` | **不是纯预览**：`tofsense.py` 依赖它的 `crc_x25` |
| `p_to_p_px4/test/test_tofsense.py` | `e98069da8160` | 参考件自带单测 |
| `setup.py` | 改动 | 新增入口 `tof_mavlink_bridge = p_to_p_px4.tof_mavlink_bridge:main` |

### 3. 怎么验证的（2026-10-12 22:0x 板上）

1. `py_compile` 两个新模块 → OK；三个新文件 md5 与参考件逐一相同。
2. 编译通过；`ros2 pkg executables p_to_p_px4` 出现 **`tof_mavlink_bridge`**。
3. **跑参考件自带单测**：`pytest src/p_to_p_px4/test/test_tofsense.py -q` → **15 passed**
   （纯软件：解码器/Kalman/CRC/MAVLink 组包都过，未碰任何串口）。
4. 回归：`pytest src/p_to_p_mission/test -q` → 仍 **35 passed**。
5. 现场核对：`/dev/ttyS3` 现在由**我们的** `tofsense_m_node`(pid 1082) 占着；
   `/dev/ttyS1` **3 秒读到 0 字节** ⇒ 目前没有任何东西在往 ttyS1 写，也没有飞控 MAVLink 进来。
   飞控侧 `/fmu/out/distance_sensor` **无数据**、`dist_bottom_sensor_bitfield = 0`
   ⇒ 飞控自己没有测距传感器（现有的 `dist_bottom` 是 EKF 用我们 EV 的高度算出来的）。

### 4. ⚠️ 真正的硬件前提（更正后）

参考件的定高链路需要**香橙派的第二个串口接到飞控的一个 MAVLink 口**：
```text
香橙派 /dev/ttyS1  @57600  ←→  飞控一个空闲 TELEM 口（TELEM1/TELEM2，协议设 MAVLink @57600）
```
**ToFSense 完全不用动**（仍在派上 `/dev/ttyS3` @921600，与参考件一致）。
现场检查表明这条线**现在不存在**（ttyS1 收不到任何字节、飞控也没有 distance_sensor 数据）
⇒ 需要用户/队友确认接线（或补一根线 + 在飞控上把那个口的协议设成 MAVLink）。

另外两点：
- **`/dev/ttyS3` 只能被一个进程读**：换上他们的 `tof_mavlink_bridge` 就必须**停掉我们的
  `tofsense_m` 节点**（否则两边抢数据、谁都收不全）。整链切换时一起停（见下）。
- 他们的 `deploy/systemd/p-to-p-tofsense-height.service` **不在**给我的那份参考件里
  —— 需要队友补，或我按同样形状写一个等价 unit（开机自启这条链路）。

### 5. 更正后的飞控参数（配他们的 EV + 他们的定高）

```text
EKF2_EV_CTRL     = 9    # bit0 水平位置 + bit3 航向；**bit1 垂直位置必须关**（EV 不带高度）
EKF2_HGT_REF     = 2    # ★ 更正：2 = Range sensor（1 = GPS！见补记 8）
EKF2_EV_NOISE_MD = 0    # 参考件要求：噪声取报文方差
EKF2_OF_CTRL     = 0    # 参考件要求
飞控那个 TELEM 口：协议 = MAVLink，波特率 = 57600（与 tof_mavlink_bridge 的 57600 对齐）
```
（速度通路仍要定制固件才真融合；stock 固件下 VZ=NaN 会被忽略，无害。）

### 6. 切换与回滚（等接线 + 参数就绪后一次做完）

```bash
# 切换（注意：先停我们的 tofsense_m，避免和 tof_mavlink_bridge 抢 ttyS3）
sudo systemctl stop px4-ev-chain && bash ~/px4_ws/run.sh stop
ros2 run p_to_p_px4 tof_mavlink_bridge &          # 定高链路（ttyS3 -> ttyS1）
ros2 launch p_to_p_px4 final_position.launch.py   # 感知(EV odom) + 任务 + Offboard 桥
# 回滚：停掉以上两者 -> sudo systemctl start px4-ev-chain（回到 px4_ev_bridge + 我们的 tofsense_m）
```

### 7. 双端一致性

- 板上新增：`tofsense.py`、`tof_mavlink_bridge.py`、`range_preview.py`、
  `test/test_tofsense.py`；改动 `setup.py`（新增入口）。
- 双端 `docs/CHANGELOG.md` md5 见会话记录（推送后核对）。
- ⚠️ 同前：正本在 `D:\px4无人机\CHANGELOG.md`，需队友合并补记 5、6、**7** 三节。

---
## 2026-10-12 晚（补记 8）：**⚠️ 再更正一处（`EKF2_HGT_REF` 数值）+ 把"照搬方案"的切换做成可一键启用（**尚未启用**）**

> 用户 2026-10-12：「飞控参数EKF2_HGT_REF=1的话 可是GPS呀 你确认没问题吗？」
> **用户是对的。**

### 1. 更正：`EKF2_HGT_REF` 的枚举在 1.15.4 里是 `0=Baro / 1=GPS / 2=Range / 3=Vision`

证据（官方源码，PX4 v1.15.4 `src/modules/ekf2/module.yaml`）：
```yaml
EKF2_HGT_REF:
  type: enum
  values:
    0: Barometric pressure
    1: GPS            # ★ 1 是 GPS！
    2: Range sensor   # ★ 测距是 2
    3: Vision
  default: 1
  reboot_required: true
```
而且**本文件第 70 行早就记过**「`EKF2_HGT_REF` | **1 = GPS** | 没装 GPS 时必须改 **3(Vision)** 或 2(Range)」。
⇒ 补记 6 §3(b) 和补记 7 §5 里我写的 `EKF2_HGT_REF = 1 # 测距` **两处都错**（已在本文件里就地更正为 **2**）。
⇒ 照搬他们方案时的正确值：**`EKF2_HGT_REF = 2`（Range sensor，即 ttyS1 送进来的 DISTANCE_SENSOR）**；
我们现状 `= 3`（Vision，因为我们的 EV 自带高度）——**切换时必须从 3 改成 2**，不然高度参考会指向没接的 GPS/视觉。
（另：`EKF2_EV_CTRL=9` 的算法也对上了 —— module.yaml 明确 bit0=水平位置、bit1=垂直位置、bit2=速度、bit3=航向，
 所以 9 = 水平位置 + 航向、垂直位置关 = 参考件要求 ✓）

### 2. 现在"一致到哪一步"

| 层面 | 状态 |
| --- | --- |
| 参考件**飞行路径上跑的文件** | ✅ **逐字节一致**：`mission_node.py`、`offboard_bridge.py`、`frames.py`、`position_ramp.py`、`odometry_bridge.py`、`odometry_output.py`、`lidar_velocity.py`、`tofsense.py`、`tof_mavlink_bridge.py`、`range_preview.py`（md5 全同） |
| 入口 launch | 🟡 我的适配版 `p_to_p_px4/launch/final_position.launch.py`（只差三处硬件：串口/波特率/雷达参数；建图用我们 `carto_pkg`） |
| 未搬的模块 | ⬜ `usb_odometry_transport.py`、`three_axis_preview.py`、`sample_alignment.py`、`deployment/systemd/*.in`、`pid_control_pkg`、`uart_to_stm32`（position 路径用不到，但严格说"没搬全"） |
| **定高链路的那根线** | ❌ **不存在**（`/dev/ttyS1` 3 秒 0 字节；飞控 `/fmu/out/distance_sensor` 无数据） |
| **飞控参数** | ❌ 未改（现在还是 `EV_CTRL=15`、`HGT_REF=3`） |
| **是否已切换/自启** | ❌ 未切；旧链 `px4-ev-chain` 仍是开机默认 |

### 3. 本轮新增：一键切换的准备件（**已就位但未启用**）

| 文件 | 作用 |
| --- | --- |
| `px4_ws/run_p2p.sh` | 启/停"照搬方案"整链（停旧进程 → `ros2 launch p_to_p_px4 final_position.launch.py`） |
| `systemd/px4-p2p-chain.service` | 开机自启上面那个脚本（含 `ROS_DOMAIN_ID`/FastDDS profile/dialout 组） |
| `launch/final_position.launch.py` 增补 | 新增 `start_tof_bridge`（默认 true）→ 用 `ExecuteProcess` 起 `ros2 run p_to_p_px4 tof_mavlink_bridge`（定高链路：ttyS3 → ttyS1） |

**切换（等飞控参数与那根线就绪后再执行，两条命令）**
```bash
sudo systemctl disable --now px4-ev-chain     # 关掉旧方案（含我们的 px4_ev_bridge / tofsense_m）
sudo systemctl enable  --now px4-p2p-chain    # 开新方案（照搬件）
# 回滚：sudo systemctl disable --now px4-p2p-chain && sudo systemctl enable --now px4-ev-chain
```

### 4. ⚠️ 为什么现在**不**替用户直接切（飞控参数未改时切换的风险）

- 他们的 EV 报文是 `position=[x, y, NaN]` + `position_variance.z=100`，**要求 `EKF2_EV_CTRL` 的垂直位置位关闭**；
  我们飞控现在 bit1 是**开**的 → 切换后 EKF2 会收到一个"垂直位置无效"的观测（NaN），
  参考件自己的注释就写着"vertical position must remain disabled" ⇒ 不能带着旧参数切。
- 同时我们的 EV 高度（TOF→EV）会消失，而 ttyS1 那条线还没接 ⇒ **高度没有任何来源**
  （`HGT_REF` 还是 3=Vision 而视觉不再供高度；改成 2=Range 又没有测距报文）
  ⇒ 切换后**这架飞机不能起飞**（预检也不会过），且有把 EKF 高度搞坏的余地。
- 所以：**先（a）飞控端把 `EKF2_EV_CTRL=9`、`EKF2_HGT_REF=2`、`EKF2_EV_NOISE_MD=0`、`EKF2_OF_CTRL=0` 设好、
  （b）确认/补上 `香橙派 ttyS1 @57600 ←→ 飞控一个 MAVLink 口` 这根线**，再执行上面两条命令一次切过去。

### 5. 双端一致性

- 板上新增：`run_p2p.sh`、`systemd/px4-p2p-chain.service`；改动 `launch/final_position.launch.py`（加 TOF 桥）。
- 双端 md5 见会话记录（推送后核对）。
- ⚠️ 正本仍在 `D:\px4无人机\CHANGELOG.md`，需队友合并补记 5、6、7、**8**。

---
## 2026-10-13：**已切换到"参考件感知层 + 保留我们的 EV 高度"（开机默认），但飞控参数要先改回来**

> 用户 2026-10-13：「先保留我们的EV高度吧 接线不好接 然后你帮我切换到这套方案 默认自启动使用他们的方案」

### 1. 最终形态（混合，已作为开机默认）
```text
定高   : 我们的 tofsense_m（/dev/ttyS3）→ /tofsense/height（**不接 ttyS1，不走 MAVLink**）
感知   : 参考件 cartographer_odometry_bridge（**odom→base_link**、FRD、30 Hz、fail-closed、
         窗口最小二乘速度、reset_counter 0.75m/45°）+ 我们把定高并进 EV 的垂直分量
SLAM   : cartographer + **amphi.lua**（参考件飞行用的那份；与本机原本逐字节相同）
任务   : 参考件 p_to_p_mission + px4_offboard_bridge（position 模式）
飞控参数: 保持 **EKF2_EV_CTRL=15 + EKF2_HGT_REF=3(Vision)**（因为高度仍来自 EV）
自启   : px4-p2p-chain.service（新）已 enable；px4-ev-chain（旧，含 px4_ev_bridge）已 disable
```
**相对参考件的两处偏离（都写在这里备案）**
1. `odometry_bridge.py`：新增 `use_height/height_topic/height_timeout_sec/height_sigma_m`，
   把 `/tofsense/height`(m) 并进 `position.z = -h`（FRD 向下为正）、`position_variance[2]=σ²`；
   高度超时/非法 → `z=NaN` 且方差 100（明确"不提供"，不谎报 0）。备份原件：
   `odometry_bridge.py.bak-20261013`。
2. `odometry_bridge.py`：`quality` 由参考件的 **0** 改成 **100**。
   原因（PX4 v1.15.4 源码）：`EKF/ev_control.cpp:54`
   `quality_sufficient = (ev_quality_minimum<=0) || (quality >= ev_quality_minimum)`（默认门限 50），
   且 `EKF/ev_height_control.cpp:126` 明确：**EV 重置 + quality 不足 → stopEvHgtFusion() 且不再启动**。
   参考件填 0 无所谓（它高度不走 EV），我们高度走 EV ⇒ 必须给足质量。

### 2. 切换动作（2026-10-13 10:2x 板上执行）
```bash
sudo cp ~/px4_ws/systemd/px4-p2p-chain.service /etc/systemd/system/ && sudo systemctl daemon-reload
sudo systemctl disable --now px4-ev-chain      # 旧链（px4_ev_bridge + tofsense 由旧 run.sh 管）
sudo systemctl enable  --now px4-p2p-chain     # 新链（run_p2p.sh → final_position.launch.py）
```
新增 `run_p2p.sh`（start/bench/stop/status）、`systemd/px4-p2p-chain.service`
（`Conflicts=px4-ev-chain`、`Restart=on-failure`、dialout、FastDDS profile）、
launch 增补 `start_tofsense:=true` / `start_tof_bridge:=false`、`config/px4.yaml` 恢复
`cartographer_odometry_bridge` 段 + 高度四项。

### 3. 验证结果（板上实测）
- 进程：MicroXRCEAgent(ttyS6@1500000)、tofsense_m、bluesea2、cartographer(**amphi.lua**)、
  robot_state_publisher、occupancy、**cartographer_odometry_bridge**、p_to_p_mission、px4_offboard_bridge；
  **没有 px4_ev_bridge**（红线满足：EV 只有一个发布者）。
- 频率：`/fmu/in/vehicle_visual_odometry` **29.99 Hz**、`/scan` 15.18、`/tofsense/height` 55.7、
  `/height` 96.3。
- EV 报文：`pose_frame=2(FRD)`、`position=[x,y,**-0.0465**]`（= 我们的定高）、
  `position_variance=[0.0064,0.0064,**0.0225**]`、`quality=100`、`reset_counter=0`。
- 单测：`test_tofsense.py` 15 passed（照搬件）、`p_to_p_mission/test` 35 passed。

### 4. ⚠️ 当前阻塞：飞控侧"EV 高度融合"是关着的（**改参数前不能飞**）
实测：`cs_ev_pos=true`、`cs_ev_yaw=true`、**`cs_ev_hgt=false`**、`z_valid=false`、`v_z_valid=false`。
这个组合等价于 **`EKF2_EV_CTRL = 9`**（bit0 水平位置 + bit3 航向，**bit1 垂直位置=关**）——
正是**补记 6 §3(b) / 补记 7 §5 里我写错的那个值**（用户 2026-10-13 已指出 `HGT_REF` 那条错误）。
⇒ **需要在 QGC 里改回**：
```text
EKF2_EV_CTRL = 15   # 打开 bit1 垂直位置（我们的高度从 EV 来）
EKF2_HGT_REF = 3    # Vision（我们的高度参考；1=GPS、2=Range）
```
改完读回核对；`cs_ev_hgt` / `z_valid` 应变回 true。**在那之前高度无效，禁止起飞。**
（注：这两个参数只能在 QGC/MAVLink 侧改，DDS 写不了 —— 本会话无法代劳。）

### 5. 回滚
```bash
sudo systemctl disable --now px4-p2p-chain
sudo systemctl enable  --now px4-ev-chain     # 回到我们原来的链（含 px4_ev_bridge）
```

### 6. 双端一致性
- 本轮改动文件：`src/p_to_p_px4/p_to_p_px4/odometry_bridge.py`（+`.bak-20261013`）、
  `src/p_to_p_px4/config/px4.yaml`、`src/p_to_p_px4/launch/final_position.launch.py`、
  `run_p2p.sh`、`systemd/px4-p2p-chain.service`、`docs/CHANGELOG.md`。
- md5 见会话记录（推送后核对）。⚠️ 正本在 `D:\px4无人机\CHANGELOG.md`，需合并补记 5~9。

---
## 2026-10-13（补记 10）：**用户改好飞控参数 → EV 高度融合恢复；又发现并修掉 FRD 帧导致的 `yaw_align=false`**

### 1. 参数生效确认（用户改完 `EKF2_EV_CTRL=15` / `EKF2_HGT_REF=3` 之后实测）
```
cs_ev_pos: true   cs_ev_yaw: true   cs_ev_hgt: true      ← EV 三路都进 EKF 了
z_valid: true     v_z_valid: true                         ← 高度/垂速有效
/height 96.4 Hz  值 = 10 (cm)                            ← 任务的高度来源正常
/p_to_p/state = IDLE: 等待定位、测高与人工启动           ← 任务侧认为"就绪"
```

### 2. 新发现的问题：FRD 帧会让 EKF 认为"航向不可用于控制"
实测 `cs_yaw_align=false`、`heading_good_for_control=false`（而 `cs_ev_yaw=true`）。
根因（PX4 v1.15.4 `EKF/ev_yaw_control.cpp`）：
```text
L141-155  pos_frame == LOCAL_FRAME_NED → 把偏航重置到 EV 并 **yaw_align = true**
L157-167  pos_frame == LOCAL_FRAME_FRD → **yaw_align = false**（"用 EV 自己的任意航向基准"）
```
`vehicle_local_position.heading_good_for_control = _ekf.isYawFinalAlignComplete()`
（`EKF2.cpp:1617`）⇒ FRD 模式下这项永远是 false，PX4 不把该航向当作"可用于控制的航向"。
参考件用 FRD 是因为它的高度不走 EV；但它同时也放弃了 `yaw_align`。

### 3. 修法：EV 桥加帧选择 `pose_frame: ned|frd`，**默认 ned**
- NED 分支（与旧 `px4_ev_bridge` 同一约定）：位置 `(x_enu,y_enu) → (y,x)`、
  偏航 `yaw_ned = π/2 − yaw_ros`、`pose_frame=VELOCITY_FRAME=NED`；
- FRD 分支保留参考件原样（`-p pose_frame:=frd` 可切回）；
- 高度两分支共用 `z = -height_m`（NED/FRD 都是向下为正），`quality=100` 不变。

### 4. 验证（2026-10-13 11:0x 板上，重启整链后）
```
EV 报文: pose_frame=1(NED)  velocity_frame=1(NED)  position=[0.003, -0.004, -0.0475]
        q=[0.7073, 0, 0, 0.7069]  (yaw≈90°，与 position 同一坐标系)
cs_tilt_align: true   cs_yaw_align: true      ← 恢复
cs_ev_pos/yaw/hgt: true
xy_valid/z_valid/v_xy_valid/v_z_valid: true
heading_good_for_control: true                ← 恢复（旧方案也是 true）
进程：无 px4_ev_bridge（EV 单一发布者，红线满足）
```

### 5. 当前状态 & 下一步
- **链路已就绪**（高度有效、航向可用于控制、任务 IDLE 就绪）；飞控参数无需再改。
- **仍未验证**：这套任务栈从未在真机上发过 Offboard 流（`/fmu/in/offboard_control_mode` +
  `trajectory_setpoint` 的坐标/符号/限速、模式切换、状态机推进、超时/包线）⇒ 按 runbook
  第 2 步**拆桨地面演练**是第一次真发流的场合，建议不要跳过。
- 仍未解决的根因：SLAM 纯激光（`amphi.lua` `use_imu_data=false`）⇒ 快速旋转时局部匹配
  仍可能滑（改用 odom 后已免疫"map 修正"这一类，但没免疫"匹配滑移"）。下一步可选：
  启用 IMU（`fly.lua` + `px4_imu_bridge`）或按参考件 `h30.lua` 的形状接飞控 IMU。

### 6. 本轮改动文件 / 双端
`src/p_to_p_px4/p_to_p_px4/odometry_bridge.py`（+ `pose_frame` 分支）、
`src/p_to_p_px4/config/px4.yaml`（+ `pose_frame: ned`）、`docs/CHANGELOG.md`。md5 见会话记录。
⚠️ 正本在 `D:\px4无人机\CHANGELOG.md`，需合并补记 5~10。

---
## 2026-10-13（补记 11）：**`tools/preflight.sh` 适配新方案（用户问「还是用 preflight 做起飞前检查吗」）**

### 1. 为什么必须改
旧版 preflight 是按"我们原来那套"写的，在新链上会**误报 4 项**：
| 旧检查 | 新链上的实际情况 |
| --- | --- |
| 进程 `tf_to_pose` | 新方案**不跑**它（EV 桥直接读 TF odom→base_link）⇒ 误报 |
| 进程 `px4_ev_bridge_node` | 新方案换成 `cartographer_odometry_bridge` ⇒ 误报 |
| 话题 `/tracked_pose` 有数据 | 新方案没有这个话题 ⇒ 误报 |
| `cs_ev_vel` 必须 true | 新桥发 `[vx,vy,NaN]`，PX4 stock 要求三分量全有限 ⇒ 必然 false（**属预期**）⇒ 误报 |

### 2. 改成什么
- 进程名单换成：MicroXRCEAgent / bluesea2_node / cartographer_node / tofsense_m_node /
  **cartographer_odometry_bridge / p_to_p_mission_node / px4_offboard_bridge**；
  并**新增一条红线检查**：`px4_ev_bridge` 不能同时在跑（EV 只允许一个发布者）。
- `/tracked_pose` → 改成检查 **TF `odom→base_link` 存在**（新 EV 桥的位姿源）。
- 新增 **EV 报文契约三项**：`pose_frame` 必须 =1(NED)（=2 FRD 会让 PX4 `yaw_align=false`）、
  `quality >= 50`（PX4 质量门）、`position.z` 不是 NaN（本机定高已并进 EV）。
- `cs_ev_vel=false` 由 NG 改成 **`[!!]` 预期提示**（附原因）。
- 结尾提示里的重启命令 `px4-ev-chain` → **`px4-p2p-chain`**（新默认链路）。

### 3. 验证（2026-10-13 11:1x 板上实测）
`bash -n` 通过；实跑结果：**26 项通过 / 1 项 NG**，唯一 NG 是
`pre_flight_checks_pass != true` —— 因为**遥控器还没打开**（`manual_control_signal_lost: true`），
属预期；打开遥控器后应全过。新加的检查项全部 OK：
```
OK TF odom→base_link 存在      OK EV pose_frame=1(NED)
OK EV quality=100              OK EV position.z=-0.0482（定高已并进 EV）
OK cs_ev_pos/yaw/hgt = true    OK xy/z/v_xy/v_z_valid = true
OK 没有第二份 EV 发布者
```

### 4. 两个旧工具先别用（新链上会误报）
`tools/check_ev.sh`、`tools/watch_pose.py` 读的是 `/tracked_pose` 与 `/tmp/px4_ws_ev.log`，
新方案里都不存在 ⇒ **误报**。等价替代：`tools/preflight.sh`（已更新）+ `tools/p2p_audit.py`（逐段对账）。
（要的话可以把 `watch_pose.py` 加一个 VehicleOdometry 模式，直接读 `/fmu/in/vehicle_visual_odometry`。）

### 5. 回滚
`cp ~/px4_ws/tools/preflight.sh.bak-20261013 ~/px4_ws/tools/preflight.sh`（原件已备份）。

### 6. 双端
改动文件：`tools/preflight.sh`（+ `preflight.sh.bak-20261013`）、`docs/CHANGELOG.md`；
md5 见会话记录。⚠️ 正本在 `D:\px4无人机\CHANGELOG.md`，需合并补记 5~11。

## 2026-10-13（补记 12）：**用户决定先不用 Offboard，改「纯遥控器定点」飞；顺带修 EV 桥冷启动自杀 + preflight 识别 pos 模式**

> 用户原话：「我暂时不想 offboard，直接定点遥控器飞可以吗？」
>
> 答复：**可以，而且不用改飞控逻辑**。定点(Position)是飞控**自己的**位置环；Offboard 只是
> "谁来给位置设定点"的问题。遥控器定点 = 你打杆 → 遥控通道 → 飞控位置设定点，
> **完全不需要派发任何 `/fmu/in/*` 控制消息**。EKF 仍照吃我们喂的 EV（位置+航向+高度），
> 所以水平定位、定高、航向都还是雷达那套在支撑。

### 1. 新增模式：`run_p2p.sh pos`（纯定点，只起感知链）
- `run_p2p.sh` 新增 `pos` 分支：
  `ros2 launch p_to_p_px4 final_position.launch.py start_mission:=false start_offboard:=false`
  ⇒ 只起 agent + 雷达 + cartographer + occupancy + tofsense_m + `cartographer_odometry_bridge`；
  **不起** `p_to_p_mission_node`、**不起** `px4_offboard_bridge`。
- 也支持环境变量选模式：`P2P_MODE=pos`（`case "${1:-${P2P_MODE:-start}}"`）。
- systemd 不写死模式：`px4-p2p-chain.service` 的 `ExecStart` 不带参数（默认 = `start` 整链）；
  开机默认切 pos 靠 drop-in
  `/etc/systemd/system/px4-p2p-chain.service.d/10-mode.conf`（root 所有）：
  ```
  [Service]
  Environment=P2P_MODE=pos
  ```
- **恢复整链（含任务/Offboard）**：`sudo rm /etc/systemd/system/px4-p2p-chain.service.d/10-mode.conf
  && sudo systemctl daemon-reload && sudo systemctl restart px4-p2p-chain`。

### 2. 修的坑：EV 桥「冷启动自杀」（SCAN_STALE）
- 现象：`cartographer_odometry_bridge` 有「45 s 内没收到 `/scan` 就
  `RuntimeError: ... SCAN_STALE` 退出」的看门狗（`odometry_bridge.py` 里
  `startup_timeout_sec` 默认 45）；而串口款雷达上电握手要 25~40 s，冷启动会踩线
  ⇒ EV 桥自己死掉，飞控就断 EV 了。
- 修法（只改 `final_position.launch.py`，**不动**桥代码/参数默认值）：
  - EV 桥 `parameters` 里显式给 `{"enabled": True, "startup_timeout_sec": 120.0}`（45→120）
  - EV 桥节点加 `respawn=True, respawn_delay=3.0`
  - 雷达 `bluesea2_node` 也加 `respawn=True, respawn_delay=2.0`

### 3. `tools/preflight.sh` 适配 pos 模式
- 新增模式识别：launch 命令行含 `start_offboard:=false` ⇒ `POS_MODE=1`。
  pos 模式下 `p_to_p_mission_node` / `px4_offboard_bridge` 记作 **`[!!]` 预期**，不再记 NG。
- 新增：当 `pre_flight_checks_pass=false` 且 `manual_control_signal_lost=true` 时，
  给出「定点遥控飞必须先开遥控器」的明确提示。
- 顺手优化：`failsafe_flags` 只抓一次（第 6/7 节复用），少一次 6 s 超时。

### 4. 验证（2026-10-13 11:3x 板上实测，pos 模式）
- 进程：只有感知链 6 个（agent / bluesea2 / cartographer / occupancy / tofsense_m / odometry_bridge）；
  另有一个 launch 父进程。**没有** mission、**没有** offboard_bridge。
- `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`：**Publisher count = 0**
  （一条控制消息都没发；飞控侧 Subscription count=1，在等）。这是 pos 模式的核心安全保证。
- EV：`/fmu/in/vehicle_visual_odometry` 30 Hz；`pose_frame=1`(NED)、`quality=100`、
  `position.z≈-0.047`（定高已并进 EV）。`Publisher=1 / Subscription=1`。
- 估计器：`cs_ev_pos/yaw/hgt=true`、`cs_yaw_align=true`；`xy/z/v_xy/v_z_valid=true`；
  `heading≈1.573`（≈90°）。`cs_ev_vel=false` 属预期（位置差分方案）。
- `preflight.sh`：**24 通过 / 1 NG**，唯一 NG = 遥控器没开（`manual_control_signal_lost=true`）。
  ⇒ 打开遥控器后应全过。

### 5. 怎么飞（纯定点遥控，标准流程）
1. **先开遥控器**（再给飞控/派上电），QGC 里能看到 RC 通道在动。
2. `bash ~/px4_ws/tools/preflight.sh` → 期望 25 项全过（唯一可能 NG 就是遥控器）。
3. QGC 里飞行模式打 **Position(定点)**。
4. 遥控器解锁 → 起飞 → 打杆飞。全程不碰 Offboard。
5. 落地后要恢复整链：见 §1 的 `rm drop-in` 命令。

### 6. 回滚
- **launch**（本轮只有 3 处新增）：把 EV 桥 `parameters` 改回 `[px4_params, {"enabled": True}]`，
  并删掉雷达/EV 桥上的 `respawn=True` / `respawn_delay=...` 两行。
  备份快照：`~/backups/20261013_114000_pos_mode/final_position.launch.py`
  （本轮改动后的版本，方便再次比对）。
- **preflight**：`cp ~/px4_ws/tools/preflight.sh.bak-20261013b ~/px4_ws/tools/preflight.sh`
- **模式**：`sudo rm /etc/systemd/system/px4-p2p-chain.service.d/10-mode.conf`
  `&& sudo systemctl daemon-reload && sudo systemctl restart px4-p2p-chain`
- **整个新方案退回旧方案**：`sudo systemctl disable --now px4-p2p-chain`
  `&& sudo systemctl enable --now px4-ev-chain`

### 7. 双端（md5 已逐文件核对，本地镜像 == 板上）
```
src/p_to_p_px4/launch/final_position.launch.py   00047c235938c4c4c46faa3be62df87d
run_p2p.sh                                       d5e81b33bbc30b4774017f774ba32667
systemd/px4-p2p-chain.service                    a83718339ab412ef8d69cd45047cfdf8
tools/preflight.sh                               2476f4b5acf9840744aebddf0b95dd97
/etc/.../10-mode.conf (drop-in)                  120f509e4f39db59be25d19438e220c5
```
备份快照目录：`~/backups/20261013_114000_pos_mode/`（4 个文件）。
⚠️ CHANGELOG 正本在 `D:\px4无人机\CHANGELOG.md`，需合并补记 5~12。

## 2026-10-13（补记 13）：**发现并修掉「开机早期雷达静默失灵」——串口打不开时驱动进程不退，`respawn` 救不了**

### 0. 怎么发现的
为了验证 pos 模式的开机自启，整机重启了一次，结果 `/scan` **完全没数据**，
EV 桥被 `RuntimeError: Lidar odometry unhealthy for 120.0s: SCAN_STALE` 掐死
（之后被 respawn 拉回，但雷达一直是哑的）。根因在雷达驱动日志里：
```
[ERROR] [open_serial_port 269]: code:-106 value:Failure to modify serial port baud rate
[INFO]  [UartThreadProc 594]: Connection info:uartname /dev/ttyS4 baudrate 921600
```
即**开机 ~9 s 时 `/dev/ttyS4` 还没就绪**，`change_baud()`（走 `TCGETS2/TCSETS2` termios2 ioctl）失败。
同一个节点在开机 4 分钟后手动重启就完全正常（跟雷达握手 V4 / LOCONH / LSRPM…，稳定 15 Hz 出数据）。
⇒ **开机时序性瞬时故障**，不是接线或雷达本体坏了。

### 1. 为什么"只失败一次"会变成"永久失灵"
`bluesea2` SDK 的 `UartThreadProc`（`src/bluesea2/src/bluesea-ros2/sdk/src/uart_reader.cpp`）
在 `open_serial_port()` 失败时只 `return NULL` —— **线程退出、进程还活着**。
于是 launch 里的 `respawn=True` 永远不触发（进程没死）⇒ 雷达从此静默无数据，
直到有人手工重启这个节点才恢复。

### 2. 修法（给驱动打补丁，配合已有的 respawn）
`uart_reader.cpp::UartThreadProc` 里两处「静默 return」改成让**进程退出**：
- `access(port, R_OK)` 失败（设备节点还不存在）→ `fflush` + `_exit(3)`
- `open_serial_port()` 失败（打不开 / 设不了波特率）→ `fflush` + `_exit(3)`

配合 `final_position.launch.py` 里雷达节点已有的 `respawn=True, respawn_delay=2.0`：
开机早期会自己反复重试，串口一就绪就自动接上 ⇒ **自愈**，不需要任何人工介入。

### 3. 验证（2026-10-13 11:4x 板上）
- 重建：`colcon build --symlink-install --packages-select bluesea2` → `Finished bluesea2 [13.2s]`，rc=0
  （`install/bluesea2/lib/bluesea2/bluesea2_node` 是 symlink，指向 `build/` 里的真二进制）
- **失败路径单测**：把 port 指到不存在的设备
  `ros2 run bluesea2 bluesea2_node --ros-args -r __node:=bluesea_test -p type:=uart -p port:=/dev/ttyS99 ...`
  ```
  [ERROR] [UartThreadProc 581]: code:-114 value:Serial device list does not exist...
  [ros2run]: Process exited with failure 3
  ```
  ⇒ 进程**立刻退出**（旧版会一直挂着不退）。行号 581/599 与 594 的差别也证明跑的是打过补丁的二进制。
- **整机重启后实测**：`/scan` 15.1 Hz、EV 30.0 Hz、EV `pose_frame=1 / quality=100 / z≈-0.040`、
  `cs_ev_pos/yaw/hgt=true`、`cs_yaw_align=true`。这次开机雷达一次就成功；
  补丁的价值在"万一失败也能自愈"，并有上面的单测兜底。

### 4. ⚠️ 运维教训（重要，别再踩）
- **重启/恢复链路一律用 `sudo systemctl restart px4-p2p-chain`**。
  本次我图省事用了 `pkill -f 'ros2 launch p_to_p_px4'`，结果很糟：
  · `run_p2p.sh` 是 `exec ros2 launch`，父进程被 SIGTERM 杀掉后 systemd 判定为
    "正常退出（active (exited)）"，**不会**触发 `Restart=on-failure` ⇒ 服务停在半死状态；
  · 只有部分子进程（agent / occupancy / 雷达）变孤儿留在 cgroup，cartographer / odometry / 定高 死掉。
  必须用 `systemctl restart`（unit 默认 `KillMode=control-group`，才会把整组清干净）。
- 该机器 sudo 需要密码；可用 `echo <pw> | sudo -S <cmd>`（用法同既有脚本 `build_restart.sh`）。

### 5. 回滚
```
cp ~/px4_ws/src/bluesea2/src/bluesea-ros2/sdk/src/uart_reader.cpp.bak-20261013 \
   ~/px4_ws/src/bluesea2/src/bluesea-ros2/sdk/src/uart_reader.cpp
cd ~/px4_ws && colcon build --symlink-install --packages-select bluesea2
sudo systemctl restart px4-p2p-chain
```

### 6. 双端
```
src/bluesea2/src/bluesea-ros2/sdk/src/uart_reader.cpp   3f77f4a3965f7b87505cb2f5cb9f5605
（+ uart_reader.cpp.bak-20261013 原件备份）
```
⚠️ CHANGELOG 正本在 `D:\px4无人机\CHANGELOG.md`，需合并补记 5~13。

## 2026-10-13（补记 14）：**炸机机理研判（用户补充："空中突然失去动力掉下去，落地翻滚几圈"）**

用户补充的现象：**空中突然失去全部推力 → 直落 → 落地翻滚 → 紧急断电**。
结合飞控参数与两个日志，把"能造成这种'突然没动力'"的机理列出来（按可疑度排序）：

### 候选 1：当时根本没在 Position（最可疑，也和既往症状一致）
`COM_FLTMODE1 = 8` = **Stabilized**，`COM_FLTMODE6 = 2` = Position。实测：
- **11:58:24（log_88，炸机后 2.5 分钟）`manual_control_switches.mode_slot = 1`（Stabilized）**，
  且 `vehicle_control_mode` = `manual=1 + attitude=1 + rates=1`，**位置/高度/速度控制全为 0**；
- **11:58:33（log_89）`mode_slot = 6`（Position），`nav_state = 2`** ⇒ 用户随后把模式开关拨到了 Position。
Stabilized/Manual 下**油门直接控转速、没有定高**：把油门拨到底就是电机怠速 → **推力突然消失**，
无人机直接掉下来。这与用户历次描述"起飞到一定高度、一拨油门降落就炸"、"往一个固定方向飘"
完全吻合（Stabilized 没有位置保持，本来就会漂）。

### 候选 2：杀开关（RC 第 6 通道）
`RC_MAP_KILL_SW = 6`、阈值≈1750 µs、`COM_KILL_DISARM = 5`。杀开关触发会**立刻切电机**
（5 秒后彻底上锁）。控制台在炸机前后（11:57~11:58）**反复报 `Preflight Fail: Kill switch engaged`（×7）**
⇒ ch6 当时确实被拉高过。若飞行中 ch6 被碰到，就会是"突然失去动力"。

### 候选 3：主电源/电调侧瞬断
飞控是 `usb_connected=1`（USB 在位），主电源瞬断时飞控可以靠 USB 活着 —— 所以
"飞控没重启" **不能**排除主电源/电调瞬断。但炸机后实测 `power_input_valid=1`、电池 16.56 V
（4S≈4.14 V/节）⇒ 事后电池是好的，需要看飞行段日志才能判断是否瞬断。
附带发现：`BAT1_CAPACITY=-1` ⇒ 飞控算不出真实电量（`remaining` 不可信，别只看百分比）。

### 怎么才能定论（关键）
看**飞行段**那 30 秒的日志：`manual_control_switches.mode_slot` / `kill_switch` /
`vehicle_control_mode` / `actuator_motors.control` 四条就能把上面三种分开：
- 候选 1 → mode_slot=1 且油门通道被拉到最低
- 候选 2 → kill_switch=ON 且 `flag_control_termination_enabled=1`、电机指令瞬间归零
- 候选 3 → 电机指令仍然很大但电压/电流异常
飞控 logger 每 ~9 秒断一次，飞行段大概率在 **log_84~87** 之间的分片里（或由新的机载黑匣子录到）。

### 下次飞之前（QGC 侧，用户操作）
1. **确认模式开关在槽位 6 = Position**（起飞前在 QGC 里看模式名，别只看开关位置）；
2. **把 `RC_MAP_KILL_SW` 设为 0 关掉杀开关**（或先确认 ch6 到底绑的是什么开关）；
3. **修 RC 通道接收**：ch3(油门)/ch5(模式) 现在读 999（无效），在 QGC→Radio 里逐通道确认并重校准；
4. **拆桨地面演练**：进 Position 后把油门拨到底，正常应该"受控下降到落地"而不是"电机停转"；
   如果电机直接停，说明当时不在 Position。

## 2026-10-13（补记 13）：**炸机复盘 —— 用户给的两个飞控日志其实不是飞行段；飞控 ulog 已不可靠；加装机载"黑匣子"**

### 0. 结论速览
1. 用户给的 `log_88_2026-10-13-11-58-24.ulg` / `log_89_...-11-58-34.ulg` **不是飞行段**。
   两个文件各只有约 2.8 秒：FC 开机计时 3419.0→3422.0 s（≈11:58:24）与 3428.2→3431.0 s（≈11:58:33），
   即**炸机（≈11:56:02）之后约 2.5 分钟的地面测试**。证据：`actuator_motors.control` 全 0、
   `manual_control_setpoint.throttle=-1`、`vehicle_land_detected.landed=1/ground_contact=1`。
2. **飞控没有重启**：两份日志的 FC 开机计时是连续的（3419.0 → 3428.2 s），反推飞控开机 ≈11:01:25，
   到 11:58:34 一直没断 ⇒ 这次炸机**不是飞控掉电/复位**。
3. **派侧链路全程健康**：上一 boot 的 journal 里没有任何 `SCAN_STALE`/Traceback/进程重启；
   `/scan` 15 Hz、定高 5.00 s 一次无抖动，直到 11:58:35 派断电 ⇒ 也不是感知链断。
4. **飞控自己的 ulog 已经不可靠**：每个文件只有 ~3 秒，而且中间"订阅表(A)整段损坏",
   于是 pyulog/QGC 报 `File corruption detected` / `no subscription found for message id`
   ⇒ 这就是用户说的 "data unknown / 不全"。根因在**飞控 / SD 卡侧**（`SDLOG_MODE=0` 本应一直录）。
5. 因此新增**机载黑匣子**（§3）：下次炸机不依赖飞控 SD 卡也能复盘。

### 1. 怎么把坏日志读出来的（新增工具）
| 文件（本机 `work/`，也放到了板上 `~/px4_ws/tools/`） | 作用 |
| --- | --- |
| `ulog_recover.py` | 不依赖 A 记录：按话题名反扫订阅表，再按 `(uint16 size,'D',uint16 msg_id)` 精确模式扫数据并解码 |
| `ulog_console4.py` | 抽出飞控**控制台原文**（`boot_console_output` 分片，每片 219 B，共 19 片） |
| `ulog_params2.py` | 导出全部飞控参数（键带类型前缀，如 `int32_t RC_MAP_KILL_SW`） |
| `ulog_dump2.py` | 按话题打印字段（带时间窗过滤，滤掉按长度误命中的假行） |
实测 log_88 恢复出 **101 条订阅、80 个话题有数据**。

### 2. 从这两个日志里读到的**飞控侧事实**
**a) 控制台原文**（覆盖 ≈11:46:14 DDS 连上 → 11:58:24 解锁），全文只有这些：
```
WARN  [health_and_arming_checks] Preflight Fail: Gyro 0 inconsistent - check cal   ×3
WARN  [health_and_arming_checks] Preflight Fail: Yaw estimate error                ×3
WARN  [health_and_arming_checks] Preflight Fail: heading estimate not stable
WARN  [health_and_arming_checks] Preflight Fail: Kill switch engaged               ×7
WARN  [commander] Arming denied: switch to manual mode first
WARN  [commander] Arming denied: Resolve system health failures first               ×多次
INFO  [commander] Calibration: Disabling RC input  /  Restoring RC input
INFO  [commander] Armed by RC
```
⚠️ 这段窗口里**没有任何飞行中的报错**（无 failsafe、无 motor failure），但也没有那次飞行的
"Armed by RC" —— 说明它只覆盖到 11:58 的地面测试，**飞行段在缺失的日志分片里**。

**b) 模式开关映射（`COM_FLTMODE1..6`）**
```
COM_FLTMODE1 = 8 (Stabilized)   ← 11:58:24 实测 manual_control_switches.mode_slot = 1
COM_FLTMODE4 = 7 (Offboard)
COM_FLTMODE6 = 2 (Position)
COM_FLTMODE2/3/5 = -1（未使用）
```
⇒ **槽位 1 是 Stabilized，不是 Position**。Stabilized 没有位置保持、会随气流/姿态漂 ——
与用户一贯描述的"往一个方向飘"一致。要定点必须把开关打到**槽位 6**。

**c) 杀开关（Kill switch）** `RC_MAP_KILL_SW = 6`、`RC_KILLSWITCH_TH = 0.75`（阈值≈1750 µs）、
`COM_KILL_DISARM = 5`。log_88 里 ch6=1499 ⇒ `kill_switch=3`(OFF)。
⚠️ 但控制台在解锁前**反复报 `Kill switch engaged`** ⇒ 当时 ch6 曾超过阈值。
**杀开关在飞行中被触发会立刻切电机** —— 与用户反复说的"飞到一定高度突然电机不转了"高度吻合，
**是本次最可疑的一条**。

**d) `input_rc`：多个通道读 999（无效值）**
```
values=[1496 1504 999 1377 999 1499 999 999 ...]  channel_count=18  input_source=9(SBUS)
```
有效：ch1(roll)/ch2(pitch)/ch4(yaw,时有时无)/ch6(杀开关)；
**999（无效）：ch3(油门!) / ch5(模式开关!) / ch7 及以上**。
而 `RC_MAP_THROTTLE=3`、`RC_MAP_FLTMODE=5` ⇒ 飞控**收不到油门与模式开关通道**。
（`rc_lost=0 / rc_failsafe=0 / link_quality=255`，飞控自己认为 RC 正常 ⇒ 不是"丢信号"报警。）
⚠️ 需要用户在 QGC → Radio 逐通道确认哪几路真的在动、发射机通道数/协议是否匹配。

**e) 关键飞控参数（实测）**
```
EKF2_EV_CTRL=15  EKF2_HGT_REF=3  EKF2_EV_QMIN=0（质量门被关掉）  EKF2_EV_NOISE_MD=0
EKF2_GPS_CTRL=0  EKF2_BARO_CTRL=0（高度只来自 EV）
COM_ARM_WO_GPS=1  COM_RC_IN_MODE=3  MAN_ARM_GESTURE=1（摇杆解锁）  COM_KILL_DISARM=5
BAT1_CAPACITY=-1（**电池容量没配**）  BAT1_V_CHARGED=4.2  BAT1_V_EMPTY=3.2
```
固件 `micoair-1.15.4`，硬件 `MICOAIR_H743_V2`。
`estimator_status`：`filter_fault_flags=0`、`innovation_check_flags=0`、`pre_flt_fail_*` 全 0；
`cs_ev_pos/yaw/hgt=1` ⇒ 那一刻 **EKF 无故障、我们的 EV 一直在被融合**。
另：`unaided_heading=1.62` vs `heading=2.93`（差 ≈75°）⇒ EKF 航向被外部观测拉过较大角度，
值得单独做一次 **EV 航向一致性**检查。

### 3. 新增：机载黑匣子 `tools/record_flight.sh`
- 录**飞控视角**：`/fmu/out/{vehicle_status,vehicle_control_mode,vehicle_local_position,vehicle_attitude,
  vehicle_odometry,estimator_status_flags,failsafe_flags,battery_status,manual_control_setpoint,
  vehicle_command_ack}` + 我们发的 `/fmu/in/vehicle_visual_odometry` + `/tofsense/height` + `/tf` + `/tf_static`。
- 输出 `~/flights/<时间戳>/`，只保留最近 8 份（`FLIGHT_KEEP`）。
- 已挂进 `run_p2p.sh`（`start`/`bench`/`pos` 三分支都起），随 systemd 服务一起停；
  `P2P_RECORD=0` 关闭；日志 `~/px4_ws/logs/flight_recorder.out`。
- **实测对串口零负担**：开/关录制，ttyS6 都是 FC→Pi ≈70 kB/s、Pi→FC 4.2 kB/s
  （飞控本来就无条件推 `/fmu/out/*`）；本机 ≈100 kB/s ≈ 6 MB/min，磁盘 205 GB 空闲。
- 实测：13 s → 1.3 MB / 7157 条，关键话题齐全；录制期间 EV 30.0 Hz、`/scan` 15.2 Hz、load 1.4。

### 4. 下一步（需要用户配合）
1. **补飞行段日志**：飞控 SD 里找 11:55:2x–11:56:1x 的分片。飞控 logger 每 ~9 秒断一次、每次 ~1 MB，
   飞行段可能被切成 3~4 个文件，编号大概率在 84~87 之间。给新的分片我能直接解码（工具已就绪）。
2. **查 RC 通道**：QGC → Radio 逐通道看；重点 ch5(模式)、ch3(油门) 是否真的收不到；
   **并把 ch6 杀开关确认清楚**（不用就设 `RC_MAP_KILL_SW=0` 关掉）。
3. **飞行模式**：确认开关在**槽位 6 = Position** 才起飞（槽位 1 是 Stabilized）。
4. `BAT1_CAPACITY` 按实际电池填（现为 -1）。

### 5. 双端
```
tools/record_flight.sh         e8710184eb0e5b8cde5ee1d775184e29   （新增）
tools/ulog_recover.py / ulog_console4.py / ulog_params2.py / ulog_dump2.py  （新增）
run_p2p.sh                     14c7a0ba0634fa1a5a0483199d1cc8d4   （备份 run_p2p.sh.bak-20261013b）
```
⚠️ CHANGELOG 正本在 `D:\px4无人机\CHANGELOG.md`，需合并补记 5~13。

---
## 2026-10-13 傍晚（本机侧）：**把板上 10-12/10-13 的 820 行新日志合并回本机正本（§7.1）**

- 起因：用户 2026-10-13「连上香橙派看看有没有什么改动，日志看一眼」。
- 核对：板上 `~/px4_ws/docs/CHANGELOG.md`（3364 行）= 本机正本（2544 行）**逐字节一致的前缀**
  + 820 行新增（纯追加、零冲突）⇒ 以板上版本为正本；本机旧版留 `CHANGELOG.md.bak-20261013`。
- 板上新增 11 节（补记 5–14，含一节重号 13）要点：
  1. **补记 5**：`tools/p2p_audit.py`（任务逐段对账工具）落地；
  2. **补记 6**：按用户"完全照搬"把**参考件感知层（EV）**搬进来（含 `launch/final_position.launch.py`），当时未切换；
  3. **补记 7 / 8（两次自我更正）**：定高其实也接在香橙派上（`tof_mavlink_bridge.py` 是定高链路本体，
     不是 USB 附属）；`EKF2_HGT_REF` 枚举更正为 `0=Baro / 1=GPS / 2=Range / 3=Vision`；
  4. **2026-10-13 切换**：开机默认改成"参考件感知层（`cartographer_odometry_bridge`：odom→base_link、FRD、
     30 Hz、fail-closed、窗口最小二乘速度、reset 0.75 m/45°）+ **保留我们的 EV 高度**（定高仍走
     `tofsense_m` /dev/ttyS3 → `/tofsense/height`，不接 ttyS1）"；新增互斥单元
     `systemd/px4-p2p-chain.service`（`Conflicts=px4-ev-chain.service`）与 `run_p2p.sh`
     （`start|bench|pos|stop|status`）；
  5. **补记 10**：用户改回飞控参数（`EKF2_EV_CTRL=15 / EKF2_HGT_REF=3`）后 EV 高度融合恢复；修掉 FRD 帧导致的 `yaw_align=false`；
  6. **补记 11**：`tools/preflight.sh` 适配新链（旧版会误报 4 项：`tf_to_pose`、`px4_ev_bridge_node`、`/tracked_pose` 等）；
  7. **补记 12**：用户决定**先不用 Offboard，改「纯遥控器定点」**（`run_p2p.sh pos` = 只起感知链，
     绝不发 `/fmu/in/offboard_*`）；修 EV 桥冷启动自杀；preflight 识别 pos 模式；
  8. **补记 13**：修掉「开机早期雷达静默失灵」（串口打不开时驱动进程不退，`respawn` 救不了）
     —— 对应 `sdk/src/uart_reader.cpp` 的改动；
  9. **补记 14 + 13（重号）**：**2026-10-13 ≈11:56 又炸机**（用户描述"空中突然失去全部推力 → 直落 → 落地翻滚几圈"）。
     研判：飞控**没有重启**（两份 ulog 的 FC 开机计时连续）；用户给的 `log_88/log_89` **不是飞行段**
     （是炸机后约 2.5 分钟的地面测试，电机指令全 0、landed=1）；最可疑的是**当时根本不在 Position 模式**
     （`COM_FLTMODE1=8`=Stabilized、`mode_slot=1`，位置/高度/速度控制全 0）。因飞控 ulog 不可靠，
     新增机载"黑匣子" `tools/record_flight.sh`（rosbag2 录飞控视角到 `~/flights/`）。
- **本轮未推板**：板上现在另有会话在跑（板上有 `.vscode-server`/`.copilot`；`px4-p2p-chain.service`
  自 17:48 起以 pos 模式运行、黑匣子正在录 `~/flights/20261013_174839`），此刻推文档会互相覆盖。
  等板上安静后按 §7.1 推一次，推完两端哈希应一致。
- **本机代码仍落后（暂存未覆盖）**：板上已改而本机还是旧版的有
  `src/p_to_p_px4/{config/px4.yaml +33, package.xml +3, setup.py +6}`、
  新增 `launch/final_position.launch.py`、`p_to_p_px4/{lidar_velocity,odometry_bridge,odometry_output,range_preview,tof_mavlink_bridge,tofsense}.py`、
  `test/test_tofsense.py`，以及 `src/bluesea2/.../sdk/src/uart_reader.cpp +18/-1`、`tools/preflight.sh +67/-8`、
  新增 `run_p2p.sh`、`systemd/px4-p2p-chain.service`、`tools/{p2p_audit.py,record_flight.sh,ulog_*.py×5}`。
  这些已暂存到 `D:\px4无人机\ref_board_sync_20261013\`
  （tar sha256 `e613476a23d8149d971aa02f00ad4c052aad10abefc56fd901bf50f9bc319963`），**暂未覆盖本机 `px4_ws`**。
- 遗留小事：10-12 20:30 我那条失败命令在板上家目录留了两个空目录（名字分别是空格和 `cd`），
  下次上板清掉并在此补记。

---
## 2026-10-13 18:1x–18:3x（本机侧）：**又一次炸机 · 黑匣子已录到 · 但香橙派 SSH 失联**

- 用户 2026-10-13 傍晚：「刚才又飞了一次，然后炸机了，要怎么看你说的那个飞行日志」。
- **只读核对（18:18 那次还连得上）**：
  - `px4-p2p-chain.service` = active（18:11:11 起），跑 pos 模式 + 黑匣子；
  - `~/flights/` 中与本次相关的两份 bag：
    - `20261013_180742/20261013_180742_0.db3`（10.8 MB，18:07:42 起 —— **大概就是这次炸机的窗口**）
    - `20261013_181115/20261013_181115_0.db3`（48.7 MB，18:11:15 起，登录时仍在写）
  - ⚠️ **两份都没有 `metadata.yaml`** ⇒ `ros2 bag info / play` 报
    `Could not find metadata in bag directory`（录制进程被 kill 或服务重启，rosbag 来不及收尾）。
    **数据本体在 `.db3`（sqlite）里是完整的**，直接读即可。
- **新增工具（本机已写；板上是否已传到未确认）**：`px4_ws/tools/bag_crash_report.py`
  - 用 `sqlite3` 直读 `.db3` + `rclpy` 反序列化，**不需要 metadata.yaml**；
  - 输出：话题清单（条数/首末墙钟时间）、状态变化时间线（arming/nav/failsafe/pre_flight_checks）、
    电池电压电流极值、`local_position` 高度范围与最高点时刻、`manual_control_setpoint` 油门范围、
    姿态极值、EV 帧率、`/tofsense/height` 范围；
  - 用法：`python3 ~/px4_ws/tools/bag_crash_report.py ~/flights/20261013_180742`
  - 本机 md5 `a008d17ab601cc768fc2ab352a0a034f`；`--csv` 可导出明细。
- ⚠️ **香橙派 SSH 失联（18:2x 起）**：`ping` 通、`Test-NetConnection -Port 22` = **True**，
  但 `ssh` **卡在 banner exchange**（连续 4 次超时/无回显）⇒ sshd 在监听但处理不过来，
  疑似高负载 / fork 失败（撞机后遗留进程？）。
  **本轮对板子只发了只读命令**，没做任何写操作。
  恢复建议：**给香橙派断电重启**（已落盘的 bag 不受影响；正在写的 `20261013_181115` 会停在当前状态）。
- 待办：
  1. 恢复 SSH 后先跑 `bag_crash_report.py` 读 `20261013_180742`，出炸机时间线；
  2. ⚠️ 黑匣子**话题表缺关键几路**：`actuator_outputs`/`actuator_motors`（电机到底还有没有指令）、
     `input_rc`/`manual_control_switches`（杀开关/模式开关通道）、`/fmu/in/vehicle_command`。
     "空中突然失去动力"要区分"飞控自己切电机"还是"供电瞬断"，这几路必须补上
     ⇒ 要改 `tools/record_flight.sh`（改完按规矩记双端）。
- 本轮**未推板**（板上 SSH 失联）；本机正本记本节；旧版留 `CHANGELOG.md.bak-20261013b`。

---
## 2026-10-13 18:4x（本机侧）：**第三次炸机的黑匣子复盘（数据在，但只有香橙派侧）+ 更正一条错误结论**

- 用户 2026-10-13 傍晚又飞一次并炸机；飞控 SD 卡上的 ulog 仍是**损坏/unknown data**（老毛病）。
- **黑匣子位置与内容**：
  - `~/flights/20261013_180742/`（10.8 MB，18:07:43 → 18:09:25）**只有 `/tofsense/height`**：
    5670 帧，0.020 → **2.815 m**（即这次飞到了 2.8 m 高）；
  - `~/flights/20261013_181115/`（48.7 MB，18:11:15 → 18:30:52）同样**只有 `/tofsense/height`**（65712 帧）；
  - 两份都**没有 `metadata.yaml`** ⇒ `ros2 bag info/play` 报 "Could not find metadata"（进程被 kill 时来不及收尾）。
- **这次炸机能确认的事实**：18:09:25 数据断掉，而香橙派是 **18:10:36 才重新开机**
  ⇒ 大撞击/摔机时**整机（含香橙派）掉过电**（这跟"空中突然失去动力直落、落地翻滚"吻合）。
- **新增工具**：`tools/bag_crash_report.py`（直读 `.db3`，不需要 metadata；已传到板上，md5 `a008d17a…`）。
  用法：`python3 ~/px4_ws/tools/bag_crash_report.py ~/flights/<目录>`
- ⚠️ **更正我自己的一个错误结论**：我一度判断"bag 里没有 `/fmu/*` 是 QoS 不匹配（录制器 RELIABLE vs
  飞控 BEST_EFFORT）"，并据此做了 `tools/bag_qos_overrides.yaml`。**对照实验证明这是错的**：
  - 带覆盖：25 s 只录到 `/tofsense/height`；14 s 只录到 `/fmu/out/battery_status`；
  - **不带覆盖**：20 s 录到 `/fmu/out/vehicle_status` **33 帧**（1.8 Hz，`arming=1 nav=4 failsafe=False`）✅
  ⇒ **该覆盖表已标注"不要使用"**（文件头写了实验记录），**没有**接到 `record_flight.sh` 上。
- **真正的根因（待下一轮定）**：最可能是**录制器启动时飞控 DDS 话题还不存在/还没被发现**
  （`fastdds_udp_only.xml` 有"新进程看不到已有话题"的老毛病，见 §10）。
  验证方向：录制前先等 `/fmu/*` 出现（或先起 agent 再起录制器），再复测。
- 结论：**这次炸机目前没有飞控侧一手数据**；先把黑匣子修好（并处理飞控 ulog 损坏 = SD 卡/SDLOG），
  下次飞行才有复盘依据。

---
## 2026-10-13 晚（本机侧）：**按"悬停约 10 秒 → 空中猛侧翻"排查"保护"相关代码（飞控 + 香橙派）**

现象（用户原话）：「飞在空中悬停差不多 10 秒钟就会炸，空中猛侧翻然后炸机，看起来有点像触发了什么保护
使飞控停止工作了一样」。

### 1. 飞控侧（本机 PX4 1.15.4 源码 = 飞控固件版本）会"切动力/停电机"的保护

`src/modules/commander/failsafe/failsafe.cpp`：

- **`fd_critical_failure`（最要紧）**
  - 起飞窗口内（`armed < COM_LKDOWN_TKO(默认 3 s) + COM_SPOOLUP_TIME(默认 1 s)`）→ **`Action::Disarm`（立刻停电机）**；
  - 之后 → **`Action::Terminate`**（输出置 failsafe 值 = 停动力）；**除非 `CBRK_FLIGHTTERM` 设为魔法值(121212)**，
    才降级为"只告警"；
  - 两条都 `cannotBeDeferred()` ⇒ **不受 `COM_FAIL_ACT_T`(5 s) 影响，立即执行**。
  - 触发来源（`HealthAndArmingChecks/checks/failureDetectorCheck.cpp:97`）：
    `FAILURE_ROLL | FAILURE_PITCH | FAILURE_ALT | FAILURE_EXT`
    ⇒ **`FD_FAIL_R` / `FD_FAIL_P` 默认 60°**，超 60° 持续 **`FD_FAIL_R_TTRI`/`FD_FAIL_P_TTRI` = 0.3 s** 即判失效；
    另有**高度失效**与**外部 ATS**（`FD_EXT_ATS_EN` 默认 0 = 关）。
- `fd_motor_failure` → `COM_ACT_FAIL_ACT`（默认 **0 = 仅告警**）
- `fd_imbalanced_prop` → `COM_IMB_PROP_ACT`（默认 **0 = 仅告警**）；`FD_IMB_PROP_THR=30`、`FD_ACT_EN=1`
- 其它同类（本次不一定相关）：`COM_OBL_RC_ACT`（默认 0=Position；**6=Terminate / 7=Disarm**）、
  `NAV_RCL_ACT`（默认 2=Land；**4=Terminate**）、GCS 断链（Terminate=5/Disarm=6）、
  **杀开关**（`RC_MAP_KILL_SW` + `RC_KILLSWITCH_TH` 默认 0.75≈1750 µs + `COM_KILL_DISARM` 默认 5 s）、
  `COM_POSCTL_NAVL`（默认 0 = 位置失效时退 AltCtrl/Manual；1 = 退 Land/Descend）、
  `COM_LOW_BAT_ACT`（默认 0 = 仅告警）、`COM_DL_LOSS_T`(10 s)、`COM_RC_LOSS_T`(0.5 s)。

### 2. 判读（"猛侧翻"的两种解释）

1. **先真侧翻、后被保护切断**：滚/俯真超 60° 持续 0.3 s → `fd_critical_failure` → Terminate
   （或在起飞 4 s 窗口内 → Disarm）⇒ 看到的"猛侧翻 + 电机停"里，**保护是结果不是原因**；
2. **保护先误触发**：`FAILURE_ALT`（高度失效）或 `FAILURE_EXT` → 直接 Terminate。
   **这条对这台飞机特别可疑**：高度全靠 EV（TOF 塞进 EV 的 z、`EKF2_HGT_REF=3`），
   而日志里早有"TOF 贴地读 2.55 m / 高度跳变"的记录；另外杀开关(ch6)在既往复盘里被列为本轮头号嫌疑
   （`RC_MAP_KILL_SW=6`、控制台反复报 `Kill switch engaged`、`input_rc` 里 ch3/ch5 读 999 无效）。
3. `FD_ACT_*`（失衡桨/电机失效）**默认只告警**，先排除。

### 3. 香橙派侧：没有能"切电机"的东西

`p_to_p_px4/offboard_bridge.py` 的 `stop_output` / `_handle_fault` / `land_on_fault=false`
**只停 Offboard 输出**，不会停电机；而且用户已决定只飞**纯遥控定点**（`run_p2p.sh pos`，
不起任务/不起 Offboard 桥）⇒ **嫌疑人全在飞控侧**。

### 4. 下一步（建议，未执行）

1. **QGC → MAVLink 控制台逐条读回**：
   `param show CBRK_FLIGHTTERM`、`param show FD_FAIL_*`、`param show FD_ACT_*`、
   `param show COM_ACT_FAIL_ACT`、`param show COM_IMB_PROP_ACT`、`param show COM_OBL_RC_ACT`、
   `param show NAV_RCL_ACT`、`param show RC_MAP_KILL_SW`、`param show RC_KILLSWITCH_TH`、
   `param show COM_KILL_DISARM`、`param show COM_LOW_BAT_ACT`、`param show COM_POSCTL_NAVL`、
   `param show COM_RC_LOSS_T`、`param show COM_FAIL_ACT_T`；
2. **判据**：只要 `CBRK_FLIGHTTERM != 121212`，滚/俯 60°(0.3 s)、高度失效、外部 ATS
   都会**直接切动力**；**不要盲目改成 121212**（那是安全设计，先搞清楚触发的是哪一路）；
3. 杀开关：不用就 `RC_MAP_KILL_SW=0`；要用先确认 ch6 映射正确（现在 ch5/ch3 读 999=无效）；
4. 以上都还是**假设** —— 黑匣子（`/fmu/*`）没修好、飞控 ulog 损坏，**先修数据链再飞**才能定案。

---
## 2026-10-13 晚（本机侧）：**用户决定关闭"飞行终止"保护做 A/B 试验（CBRK_FLIGHTTERM）—— 我方给出范围/风险**

- 用户原话：「那你把这个断电保护给关掉，我想看看是不是因为这个引起的炸机」。
- **能关的就是这一条**：`CBRK_FLIGHTTERM = 121212`（魔法值 = 关闭 flight termination）。
  改完 `fd_critical_failure` 的来源（滚/俯 >60° 撑 0.3 s、**高度失效 FAILURE_ALT**、外部 ATS）
  **不再直接切动力，只告警**（走 `failsafe.cpp:533` 的 `Action::Warn` 分支）。
- ⚠️ **它管不到的地方（做试验前必须知道）**：
  1. **起飞窗口内照样 Disarm**：`armed < COM_LKDOWN_TKO(3 s) + COM_SPOOLUP_TIME(1 s)` 期间的
     `fd_critical_failure` 走 `Action::Disarm`，**与 CBRK_FLIGHTTERM 无关**
     （源码注释原话："On takeoff the flag triggers lockdown (irrespective of @CBRK_FLIGHTTERM)"）；
  2. **杀开关**（`RC_MAP_KILL_SW` / `RC_KILLSWITCH_TH` / `COM_KILL_DISARM`）不受影响；
  3. **GCS 链路丢失 10 s**（`COM_DL_LOSS_T`，动作枚举里同样有 Terminate/Disarm）不受影响；
  4. `NAV_RCL_ACT`(RC 丢失)、`COM_OBL_RC_ACT`(Offboard 丢失)、`COM_ACT_FAIL_ACT`(电机失效) 都不受影响。
  ⇒ 这个试验**只能排除"fd_critical_failure 切动力"这一种**；另外三种嫌疑人还在。
- **怎么改（只能用户在 QGC 做；DDS 侧写不了飞控参数）：**
  QGC → 设置 → 参数 → 搜 `CBRK_FLIGHTTERM` → 改成 `121212` → 写入 → 读回确认。
- **风险（已向用户说明）**：关掉 termination 后，飞机真滚过 60° 时**不会自动停电机**，
  翻滚/落地过程中会**继续输出电机**（更伤桨、对人更危险）。这是安全设计，
  只在可控场地、明确为定位故障时临时关，**测完必须改回**。
- **回滚**：把 `CBRK_FLIGHTTERM` 改回 `0`（默认 = 终止启用）。
- **更强的替代（推荐并行）**：先把黑匣子修好（让 `/fmu/*` 进 bag），下次飞行直接用
  `failsafe_flags.fd_critical_failure` / `vehicle_status.failsafe` / `nav_state` 判定是哪条保护、
  何时跳的 —— 不必靠"再炸一次"来排除。
- 本条只记录"用户决定 + 我方范围/风险说明"；**参数由用户执行**，执行后请在此补记实际值。

---
## 2026-10-13 晚（本机侧）：**修黑匣子（录制器时序）—— 新版已生成，推送被板上 sshd 挡住**

- 根因（对照实验确定）：
  - 服务里录制器由 `run_p2p.sh` 随整链启动拉起，**比 MicroXRCEAgent 把 `/fmu/*` 建出来早约 10 s**
    ⇒ 整份 bag 只有 `/tofsense/height`，`/fmu/*` 一条都收不到（18:07 / 18:11 / 18:36 三份皆然）；
  - 手工在 `/fmu/*` 已存在之后再起录制器 ⇒ **完全正常**（`/fmu/out/vehicle_status` 33 帧/18 s）。
  ⇒ **不是 QoS**（带 QoS 覆盖反而录不到，见本文件上一条更正），是**启动时序 / DDS 发现问题**。
- 修复（本机已生成 `px4_ws/tools/record_flight.sh`，2724 字节）：
  1. 在 `ros2 bag record` 之前插入 `wait_for_fcu_topics`：轮询 `ros2 topic list`，
     等 `/fmu/out/vehicle_status` 出现（最多 120 s），再 `sleep 3` 让 DDS 发现稳定；
  2. 话题表补两路（专门为查"空中突然失去动力"）：`/fmu/out/manual_control_switches`
     （**杀开关 + 模式槽位**）、`/fmu/out/actuator_outputs`（电机输出指令）。
- 原件留存：本机 `ref_board_sync_20261013\tools\record_flight.sh` 与板上原件**字节一致**
  ⇒ 要回滚就从这里 cp 回去；板上**尚未**留 `.bak`（推送未完成）。
- ⚠️ **推送/验证被挡住**：本轮 scp 报 `Connection timed out during banner exchange`
  —— 香橙派 sshd 又不响应（今晚第 3 次；前两次等一阵 / 断电重启后恢复）。
- 待办：
  1. 板上 sshd 恢复后：推 `record_flight.sh` → 板上前台实测 25 s
     （`FLIGHT_DIR=/tmp/boxtest timeout 30 bash tools/record_flight.sh` + `bag_crash_report.py`）
     确认 `/fmu/*` 与 `/tf` 真的进 bag；
  2. 生效需 `sudo systemctl restart px4-p2p-chain`（重启整链；**无人机必须在地面且未解锁**）；
  3. 补推到板上镜像并核对 md5。

---
## 2026-10-13 19:2x（本机侧）：**飞控参数实测回报 —— 关键反转：CBRK_FLIGHTTERM 早已是 121212**

用户从 NSH 控制台读回（原文见会话记录）：

| 参数 | 值 | 判读 |
| --- | --- | --- |
| **`CBRK_FLIGHTTERM`** | **121212** | **飞行终止早就关着** ⇒ `fd_critical_failure`（滚/俯 60°、高度失效、外部 ATS）**不会切动力，只告警**；只有解锁后 `COM_LKDOWN_TKO(3s)+COM_SPOOLUP_TIME(1s)`=**4 秒窗口内**才会 Disarm。⇒ "保护切动力"这条**基本可排除**（本次是悬停 ~10 s，已过窗口） |
| `FD_FAIL_R` / `FD_FAIL_P` | 60 / 60 | 默认；阈值本身仍会"判定失效"（只是动作被 CBRK 降级为告警） |
| `FD_FAIL_R_TTRI` / `FD_FAIL_P_TTRI` | 0.3 / 0.3 | 默认 |
| `FD_ACT_EN` | 1 | 电机/执行器失效检测**开着**（配 `COM_ACT_FAIL_ACT`，**该参数用户没读，必须补查**：若=4(Terminate) 就会切动力） |
| `COM_DL_LOSS_T` + `NAV_DLL_ACT` | 10 + **0** | 动作 = **Disabled** ⇒ GCS 断链**不切动力**，"10 秒"巧合排除 |
| `NAV_RCL_ACT` | **3 (Hold)**（用户改过，+ 号） | RC 丢失只 Hold，不切动力 |
| `RC_MAP_KILL_SW` | **6**（用户改过，+ 号） | **杀开关映射在 ch6，是活的** ⇒ 当前**头号嫌疑** |
| `RC_KILLSWITCH_TH` | 0.75 (≈1750 µs) | 触发阈值 |
| `COM_KILL_DISARM` | 5 s | 触发后 5 s 自动上锁 |
| `COM_LOW_BAT_ACT` | 0 | 仅告警 |
| `COM_POSCTL_NAVL` | 0 | 位置失效退 AltCtrl/Manual |
| `COM_FAIL_ACT_T` | 5 s | 推迟时间 |

**结论 / 修正后的嫌疑排序**
1. **杀开关（`RC_MAP_KILL_SW=6`）**：既往日志里 ch6 曾超阈值（"反复报 Kill switch engaged"）；一旦飞行中触发 → 电机立刻切、5 s 后自动上锁 ⇒ 与"空中突然失去动力 + 猛侧翻"最吻合。
   处置：**不用就 `RC_MAP_KILL_SW=0`**；要用则必须确认 ch6 是真正的开关通道（`input_rc` 里 ch3/ch5 读 999 无效是另一个隐患）。
2. **`COM_ACT_FAIL_ACT`（待补查）** + `FD_ACT_EN=1`：电机失效检测开着，若动作被设成 Terminate 就会切动力。
3. 供电/硬件瞬断（BEC/电池）—— 靠黑匣子的 `battery_status` + `actuator_outputs` 判。
4. 姿态/估计（SLAM 航向）—— 靠 bag 里 `vehicle_attitude` + EV 判。

**"关掉那个保护"这件事的现状**：`CBRK_FLIGHTTERM` **已经是 121212**（关闭状态），不需要改 —— 也就是说这次炸机**不是**飞行终止保护引起的。

**黑的匣子已能录**：`manual_control_switches`（**能直接看杀开关是否置位**）+ `actuator_outputs`（电机指令）+ `battery_status` ⇒ 下一次飞行就能在三者之间定性，不用再猜。

（另：NSH 开机那句 `sysinit: fopen failed` 是飞控开机脚本缺文件的老毛病，板上日志记过，与本次无关。）

---
## 2026-10-13 19:3x（本机侧）：**补齐三个参数 ⇒ 飞控侧"切动力"路径全部排除，只剩杀开关**

用户 NSH 读回：`COM_ACT_FAIL_ACT = 0`（仅告警）、`COM_IMB_PROP_ACT = 0`（仅告警）、
`COM_OBL_RC_ACT = 4`（+ 用户改过；枚举 4 = **Land**，只在 Offboard 模式生效 ⇒ 丢流是"降落"不是"切动力"）。

**排除清单（全部不会空中切电机）**：
`CBRK_FLIGHTTERM=121212`（fd_critical_failure 只告警；仅解锁后 4 s 窗口内 Disarm，本次 ~10 s 已过）、
`COM_ACT_FAIL_ACT=0`、`COM_IMB_PROP_ACT=0`、`NAV_RCL_ACT=3`(Hold)、`NAV_DLL_ACT=0`(Disabled)、
`COM_LOW_BAT_ACT=0`、`COM_POSCTL_NAVL=0`、`COM_OBL_RC_ACT=4`(Land，且非 Offboard 模式不适用)。

⇒ **代码侧唯一还能"空中切动力"的只剩 `RC_MAP_KILL_SW=6`（杀开关 / ch6，阈值 1750 µs，触发后 5 s 自动上锁）**。
处置：NSH `param set RC_MAP_KILL_SW 0` + `param save`。

若关掉后仍"空中失去动力"，则指向硬件/供电（电池/BEC/电调），判据用刚修好的黑匣子：
`actuator_outputs`（指令在不在）+ `battery_status`（电压是否塌）+ `vehicle_attitude`（姿态是否先崩）。

---
## 2026-10-13 21:1x（本机侧）：**结论：关掉杀开关后两次正常起降 ⇒ 炸机根因高度指向"杀开关"**

- 用户报：`RC_MAP_KILL_SW` 6→0 之后飞了**两次，均正常起降**（不再"悬停十几秒猛侧翻"）。
- **两次正常飞 与 之前炸机 之间，确认过的参数差异只有这一处**：`RC_MAP_KILL_SW: 6 → 0`。
  其余"能切动力"的路径在出事时**本来就是安全默认值**（读回无 `+` 修改标记）：
  `CBRK_FLIGHTTERM=121212`（默认即关闭终止）、`COM_ACT_FAIL_ACT=0`、`COM_IMB_PROP_ACT=0`、
  `COM_LOW_BAT_ACT=0`、`COM_POSCTL_NAVL=0`、`NAV_DLL_ACT=0`、`NAV_RCL_ACT=3`(Hold)。
  ⇒ **结论（高置信）：此前几次"空中突然失去动力 + 猛侧翻"是杀开关在飞行中被触发**
  （ch6 越过 1750 µs → 电机立即切、5 s 后自动上锁），"悬停约 10 秒"与 ch6 漂移/抖动到阈值吻合。
- ⚠️ **遗留风险**：现在这台飞机**完全没有杀开关**。要么把 RC 链路查干净后重新映射到**验证过的通道**
  并地面实测触发；要么明确接受"无急停"（要有意识），靠模式开关/上锁兜底。
- ⚠️ **RC 链路本身仍未解决**：起飞前 `listener input_rc` 曾 **never published**；历史上 ch3/ch5 读 999。
- ⚠️ **黑匣子仍未修好（更正我上一轮的过早结论）**：新脚本订阅日志正常（含全部 `/fmu/out/*`），
  但 `20261013_210222`（102.9 MB，21:02:33→21:16:48，覆盖这两次飞行）**topics 表里只有 `/tf`**
  （143,339 帧），`/fmu/*` 与 `/tofsense/height` 都没有 ⇒ **必须逐话题核对消息数，不能只看订阅日志**。
  待查：rosbag2 与 agent(`_CREATED_BY_BARE_DDS_APP_`) 端点的 QoS/发现协商；"每次落盘话题不同"的现象。
- ⚠️ **香橙派 sshd 今晚 4 次"能 ping 通但不回 banner"**（另一次直接 ping 不通）⇒ 需单独修
  （WiFi 省电 / 负载 / FastDDS 配置），否则每次出事都够不着板子。

---
## 2026-10-13 21:5x（本机侧）：**已完成部署（切 start 模式 + 原地 0.5 m）并通过启动自检**

- 板上实测（重启后 50 s 起完）：
  - `/etc/systemd/system/px4-p2p-chain.service.d/10-mode.conf` → **`P2P_MODE=start`** ✅
  - `src/p_to_p_mission/config/p_to_p.yaml` → **`b_offset_x_cm: 0.0`**（原地）、`cruise_height_cm: 50.0`、`terminal_behavior: hold` ✅
  - 备份：`p_to_p.yaml.bak-20261013`；`install/.../config/p_to_p.yaml` 是符号链接（→ build/ → src），改 src 即生效
- **启动自检结果（无异常）**：
  | 项 | 结果 |
  | --- | --- |
  | `px4-p2p-chain` | **active** |
  | `/p_to_p/state` | `IDLE: 等待定位、测高与人工启动` |
  | `/px4_bridge/state` | `NOT_READY DISARMED NON_OFFBOARD: PX4 preflight checks have not passed` ⇒ **桥是 control 模式（不是 BENCH）、且未解锁时不发流 = 安全** |
  | `/fmu/in/trajectory_setpoint` | `does not appear to be published yet` ⇒ 确认**此刻不发流** ✅ |
  | journal 错误（近 400 行） | **空**（无 error/fail/exception/traceback） |
  | `/p_to_p/readiness_check` | ⚠️ **本次没返回**（服务调用 8 s 超时/无输出）→ 下次复查 |
- **当前唯一阻塞 = 飞控 `pre_flight_checks_pass: false`**（就是 RC 那条：`listener input_rc` = never published）。
  ⇒ 桥会一直 `NOT_READY`，任务停在 `IDLE`：**这是设计上的默认安全态**，不是启动故障。
- 待办：① 修 RC（接收机/对频/接线）→ `pre_flight_checks_pass` 变 true；② 决定杀开关（现为 0=无急停）；
  ③ 复查 `/p_to_p/readiness_check`；④ **黑匣子仍未修好**（只落 `/tf`），修好前首飞无数据；
  ⑤ 香橙派 sshd 今晚多次失联 + 出现 polkit"阻止修改网络设置"弹窗 ⇒ 需单独查（可能有脚本周期改网络/WiFi 省电）。

---
## 2026-10-14（续 7）：**打开 uart DMA 修掉 DDS 断链隐患 + EV 桥改为"数据异常时沿用最后一次正常数据（有界）"**

### 1. 背景（16:23 炸机复盘结论）
- 16:23:57.2 飞控数据在板上断档，16:24:02.98 agent 记录 `session re-established`（客户端重建会话）；
  飞控全程保持 armed（不是整机重启）。EV 是本机**唯一**的位置+高度来源（无 GPS、cs_baro_hgt/cs_rng_hgt=false），
  链路/数据一断 = 瞬间失去参考 → 从 0.85 m 掉下来。
- 链路层隐患：ttyS3/ttyS4/ttyS6 都**没有 DMA**（dmesg: `failed to request DMA, use interrupt mode`）——
  厂商 DTS 里这些 uart 节点**有 dmas、没有 dma-names**，内核因此退回中断模式；39 分钟实测
  ttyS6 累计 oe:12 / brk:1、ttyS3 fe:40，FIFO 溢出会丢 uxrce 帧，是触发会话重建的头号嫌疑。

### 2. 改动 A：给 uart3/uart4/uart6 打开 DMA（用 overlay，不动基础 DTB）
- 新增 `device-tree/rockchip/overlay/rk3588-uart-dma.dtbo`，给 uart3(ttyS3=TOF)、uart4(ttyS4=雷达)、
  uart6(ttyS6=DDS) 补 `dma-names = "tx", "rx";`（`dmas` 本来就在，请求号与 mainline rk3588s.dtsi 完全一致：
  uart3=dmac0 12/13、uart4=dmac1 9/10、uart6=dmac1 13/14，顺序 tx,rx）。
- 加入 `/etc/default/u-boot` 的 `U_BOOT_FDT_OVERLAYS`（配置备份 `.bak-uartdma`），`u-boot-update` 后重启。
- 验证：活动设备树出现 dma-names；dmesg 不再对这三个口报 DMA 失败；ttyS6 已收 79.6 MB 且 **oe/fe/brk 全 0**；
  中断效率 95–229 B/中断（FIFO 模式约 16–32 B/中断）= 确实在用 DMA；EV 30 Hz、雷达 15 Hz、TOF 55 Hz 正常；
  本 boot 会话重建 0 次。
- 注意：`ttyS7(feba0000)` 仍无 dma-names（流量极小、用途不明，未动）。

### 3. 改动 B：EV 桥"数据异常时沿用最后一次正常数据"（src/p_to_p_px4/p_to_p_px4/odometry_bridge.py）
- 原来：扫描/位姿一 stale（>0.25/0.3 s）就**整帧停发** EV → 飞控瞬间失去唯一来源。
- 现在（有界，仍保留"真故障要能被飞控察觉"）：
  · 异常 ≤ hold_extrapolate_sec(0.4 s)：用最后一次正常速度外推；
  · 之后到 hold_max_sec(2.0 s)：冻结最后一次正常位姿，position/orientation/velocity_variance 随缺失时间增大
    （PX4 EKF2 确实把这些方差当量测协方差用：ekf2/EKF/ev_pos_control.cpp 里 pos_cov = diag(ev_sample.position_var)）；
  · 超过 2.0 s：仍然停发，交给飞控侧兜底（COM_POSCTL_NAVL / EKF 失效动作）。
  · 高度：TOF **单点跳变**（16:23 出现过 0.85→2.33 m 的读数）先丢弃、沿用上次正常高度，连续 5 个样本才认新电平；
    高度超时 ≤ height_hold_max_sec(1.0 s) 沿用上次正常高度（方差增大），再久才 NaN（不提供、绝不谎报 0）。
  · quality 恒为 100：低于 EKF2_EV_QUAL_MIN 会让飞控**永久**停掉 EV 高度融合（原代码注释里已写明），
    "可信度下降"只用方差表达。
- 实测：把 cartographer 冻住 2 s → 日志出现
  `Data problem (...) holding last good odometry for up to 2.0s`；期间 EV 仍以 ~30 Hz 输出（最大间隔 0.166 s，
  不是整段停发），恢复后回到 PUBLISHING。
- 可调参数：hold_last_good / hold_extrapolate_sec / hold_max_sec / hold_sigma_rate_mps /
  height_hold_max_sec / height_jump_max_m / height_jump_confirm_n。
- 备份：板上 odometry_bridge.py.bak-20261014；本机原文件 board_backup_20261014 之外另存
  C:\Users\Administrator\AppData\Local\Temp\px4_rollback_20261014\odometry_bridge_orig.py。

### 4. 仍然要做（不在本次改动内）
1. 飞控侧高度冗余（只能台架用 QGC/MAVLink 改，PX4 1.15 的 uxrce 话题表里没有参数话题）：
   确认/打开 EKF2_BARO_CTRL（1.15.4 默认 1，本机 cs_baro_hgt=false 说明被关了）；评估 EKF2_HGT_REF
   是否从 3(Vision) 改为 0(Baro)；COM_POSCTL_NAVL 按"现场有没有飞手"选 0（切高度/手动）或 1（直接降落）。
2. 急停开关装回来。
3. WiFi：热点 OnePlus 12 反复 assoc fail（已把该连接的 802-11-wireless.powersave 设为 2 = 关闭省电）；
   它只影响远程连接/rviz，不影响飞行链路。

---
## 2026-10-14（续 8）：**"上电到 EV 数据要等一分多钟"的根因 = 整条链被"等 NTP"卡在起跑线**（雷达本身不慢）

### 0. 结论（先看这个）
用户报"雷达传输数据很慢，每次上电切到定点模式都要等个一分多钟才收到上位机给飞控的 EV 数据"。
**不是雷达慢**：雷达驱动 **0.46 秒**完成握手（`/dev/ttyS4` @921600），`/scan` 稳定 15 Hz、EV 30 Hz。
真因是 `px4-p2p-chain.service` 里的 `After=/Wants=time-sync.target` —— 它让整条飞行链（雷达/SLAM/EV）
等到 `systemd-time-wait-sync` 报"内核时钟已同步"之后才允许启动，而 NTP 源是 systemd **编译默认**的
`ntp.ubuntu.com`（实测经常一直超时）。链路启动时刻与"NTP 同步完成"时刻**逐次相差 0.02~0.03 秒**。

### 1. 证据（单调时钟 `journalctl -o short-monotonic`，同一天四次上电）
| 开机 | "Finished Wait Until Kernel Time Synchronized" | px4-p2p-chain 启动 | 差 |
| --- | --- | --- | --- |
| 21:10 | t=35.25 s | t=35.28 s | 0.03 s |
| 21:06 | t=35.53 s | t=35.56 s | 0.03 s |
| 20:32 | t=172.89 s | t=172.92 s | 0.03 s |
| 20:16 | t=283.04 s | t=283.05 s | 0.02 s |

21:10 那次的完整时间线：4.5 s `systemd-timesyncd` 报 "System clock time unset or jumped backwards,
restoring from recorded timestamp"（开机时钟差约 52 分钟）→ 35.24 s "Initial synchronization to time
server ... (ntp.ubuntu.com)" → 35.3 s 链路起跑 → 38.98 s 雷达握手 → 45.6 s cartographer 发出
`odom→base_link` → EV 开始 30 Hz。即"上电 → EV"≈ **46 s，其中 35 s 是干等 NTP，链路自身只花约 10 s**。
同一个 boot 到 t=1063 s（17 分钟后）还在 `Timed out waiting for reply from 185.125.190.56:123
(ntp.ubuntu.com)`（换了 IP 继续试）。这也解释了等待时长为什么会在 35 s ~ 283 s 之间随机波动。

### 2. 改动 A：NTP 换源 + 缩短"最短轮询间隔"
- 新增本机文件 `D:\px4无人机\px4_ws\systemd\timesyncd.conf.d\10-ntp-cn.conf`：
  `NTP=ntp.aliyun.com`、`FallbackNTP=cn.pool.ntp.org ntp.ubuntu.com`、`PollIntervalMinSec=16`。
  为什么动 `PollIntervalMinSec`：默认 32 s 意味着"4.5 s 就已经校正完时钟，却要再等一整轮（→35.2 s）
  才把'已同步'位置起来"；16 s 是 RFC 5905 建议的客户端最小轮询间隔。
- 板端安装到 **`/etc/systemd/timesyncd.conf.d/10-ntp-cn.conf`**（用 drop-in，不动厂商文件 ⇒ 回滚=删一个文件）。
- 验证：`sudo systemctl restart systemd-timesyncd` 后 **0.07 s** 即出现
  `Initial synchronization to time server 203.107.6.88:123 (ntp.aliyun.com)`；`ping ntp.aliyun.com` 30~40 ms。

### 3. 改动 B：链路不再硬等 NTP（改成有界等待）
- `D:\px4无人机\px4_ws\systemd\px4-p2p-chain.service` 与 `px4-ev-chain.service`：
  `After=network-online.target time-sync.target` → `After=network-online.target`（`Wants=` 同步去掉），
  并在文件里写明"为什么去掉 / 代价是什么 / 怎么回滚"。
- `D:\px4无人机\px4_ws\run_p2p.sh`：新增 `wait_clock_bounded()`，在 `start|bench|pos` 三条分支的
  `stop_all` 之后调用，**最多等 `${NTP_WAIT_MAX:-15}` 秒**（环境变量设 0 可完全关掉）就照常起链；
  一旦等到"已同步"立刻返回（配合上面的 16 s 轮询，通常 0~几秒）。
- 保留的意图 + 残留风险（已写进源码注释）：原本等 NTP 是为了避免"cartographer 用错误时钟起步、
  之后被 NTP 大跳变打崩"（2026-10-08 那次根因）。现在改成有界等待，若 NTP 恰好在链跑起来之后才
  校正时钟，cartographer 可能崩一次 —— 由 launch 的 `respawn` 兜底；根除办法是换 RTC 纽扣电池。
- 验证：
  · `systemctl show px4-p2p-chain.service -p After` 已无 `time-sync.target`；
  · `systemctl list-dependencies --after px4-p2p-chain.service | grep -c time-sync` = **0**；
  · `systemctl restart px4-p2p-chain` 后日志依次出现
    `已停止（旧链 + 参考件链）相关进程。` → `等系统时间同步（最多 15s；超时照常启动…）` →
    `时间已同步（等了 0s）` → `黑匣子：已启动` → `起【纯定点】感知链…`；
  · EV 桥 21:40:22.97 启动、21:40:27 最后一条 `TF odom->base_link unavailable` 之后即开始发
    ⇒ **服务起跑 → EV ≈ 8 秒**；复核实测 `/scan` 15.17 Hz、`/fmu/in/vehicle_visual_odometry` 29.99 Hz。
  · 上电（冷启动）路径的证明用 `list-dependencies`（上面那条 = 0）：启动依赖链里已不存在 NTP 环节。

### 4. 产物路径与回滚
- **板端备份**：`~/backups/20261014_2140_fix_ntp_gate/`（含 `etc/`（单元 + 原 `timesyncd.conf` +
  drop-in 目录）、`home/`（`run_p2p.sh`/`run.sh`/`systemd/`）、`px4_ws-full-src.tar.gz`，共 528K）。
- **本机快照**（改动前的原文件，同日日期后缀）：
  `D:\px4无人机\CHANGELOG.md.bak-20261014`、`AGENTS.md.bak-20261014`、
  `px4_ws\systemd\px4-p2p-chain.service.bak-20261014`、`px4_ws\systemd\px4-ev-chain.service.bak-20261014`、
  `px4_ws\run_p2p.sh.bak-20261014`。
- **本机新增**：`px4_ws\systemd\timesyncd.conf.d\10-ntp-cn.conf`（板上对应
  `/etc/systemd/timesyncd.conf.d/10-ntp-cn.conf`）。
- 回滚 NTP：`sudo rm /etc/systemd/timesyncd.conf.d/10-ntp-cn.conf && sudo systemctl restart systemd-timesyncd`。
- 回滚链路单元：从上面的板端备份 `etc/systemd/system/` 覆盖回 `/etc/systemd/system/` 后 `systemctl daemon-reload`；
  或直接 `sudo systemctl restart px4-p2p-chain` 用旧脚本（`home/run_p2p.sh`）。

### 5. 两个连带发现（**本次未改动**，等用户决定）
1. **停链要卡满 60 秒**：`systemctl restart px4-p2p-chain` 实测 21:39:16 发起 → 21:40:19 新进程才起（63 s）。
   journal 证据：`px4-p2p-chain.service: State 'stop-sigterm' timed out. Killing.` 之后 SIGKILL 掉
   4 个 python3 进程（1104/1138/1140/1141）。根因：ROS 的 python 节点不处理 SIGTERM。
   建议单元加 `KillSignal=SIGINT` + `KillMode=mixed` + `TimeoutStopSec=15`（SIGINT 才是 ROS 节点的正常退出路径）。
2. **`run_p2p.sh stop_all` 的 `pkill -f` 会误杀旁观进程**：本会话执行 `systemctl restart px4-p2p-chain` 时，
   我自己那条 SSH 命令行里含 `cartographer_odometry_bridge` 字样，被 `pkill` 当成节点杀掉，会话中途断开
   （`AGENTS.md` 里记的"香橙派 sshd 反复失联"很可能同源）。建议改成按 cgroup 杀
   （`systemctl kill`）或用 `pgrep -x` 精确匹配可执行名，避免"命令行里提到名字就被杀"。

---
## 2026-10-14（续 9）：**取消 pos/start/bench 三种启动形态，合并成一套（= 纯感知链）**

### 0. 用户要求
"关于香橙派上面的两种模式 pos 和 start……不要搞那么多种模式，本来也不应该有那么多种形式，
飞行逻辑也不应该有什么变化才对，你告诉我哪个模式更好一点，就只用那个模式不要用另一个了。"

### 1. 两种模式到底差什么（结论：留 `pos` 那套）
三种形态的差别**只在"任务状态机 / Offboard 桥起不起"**，感知链（雷达 / cartographer / 定高 / EV 桥）
完全一样：

| 形态 | 额外拉起 | 飞行逻辑 |
| --- | --- | --- |
| `start` | `p_to_p_mission_node`（A→B 任务状态机）+ `px4_offboard_bridge`（control 模式） | 遥控器一旦切到 Offboard，**任务自动接管**（`auto_start_on_offboard: true`；launch 里还传了 `allow_land_command: True`，能自己落） |
| `bench` | 同上，但 Offboard 桥 `bench_only:=true` | 只预演、不发控制 |
| `pos` | 无 | **飞行只由遥控器 + PX4 位置环决定**（EV 只喂 EKF） |

⇒ 留 **`pos` 那套**：① 手动飞时"飞不飞 / 往哪飞"必须只由遥控器决定，`start` 多出来的那套本身就是
"另一种飞行逻辑"（还有"误切 Offboard 就被接管"的风险）；② 用户现在就是遥控器定点飞；
③ `bench` 是 Offboard/任务的预演模式，去掉 Offboard 桥后它没有意义。

### 2. 改动
- `px4_ws/run_p2p.sh`：**只有一套启动形态**。`bash run_p2p.sh`（默认，= 原 `pos`）起感知链
  （`start_mission:=false start_offboard:=false`）；`stop`/`status` 保留；`pos`/`bench` 留作
  **等价别名**（会打一行"已废弃"提示，免得残留调用把人卡住）。顺带把文件头过时的说明改对：
  原来写"参考件方案 EV 只给水平+航向、HGT_REF=2、定高走 ttyS1→MAVLink"——那根线**本机没接**，
  实际是 EV 自带高度（`/tofsense/height`）、飞控 `EKF2_EV_CTRL=15` / `EKF2_HGT_REF=3`。
- `px4_ws/tools/preflight.sh`：删掉"靠 `start_offboard:=false` 判 pos 模式"的 `POS_MODE` 分支，
  改成**固定**预期"任务状态机 / Offboard 桥不该在跑"；真在跑才报 NG（并提示它切 Offboard 后会接管飞机）。
- `px4_ws/systemd/px4-p2p-chain.service`：删掉 P2P_MODE 相关注释。板端把 drop-in
  `/etc/systemd/system/px4-p2p-chain.service.d/10-mode.conf`（内容 `Environment=P2P_MODE=pos`）
  **mv** 到备份目录（不是删除），`.d/` 空目录保留（无害）。

### 3. 验证（板端实测，22:00 那次重启）
- 服务日志按顺序：`已停止（旧链 + 参考件链）相关进程。` → `时间已同步（等了 0s）` →
  `黑匣子：已启动` → `起感知链（唯一形态）：agent(ttyS6) + 雷达(uart) + cartographer + 定高(ttyS3) + EV 桥(odom→base_link)`。
- launch 命令行 = `final_position.launch.py start_mission:=false start_offboard:=false`；
  `/scan` 15.18 Hz、`/fmu/in/vehicle_visual_odometry` 29.98 Hz；`systemctl is-active` = active。
- `bash tools/preflight.sh`：进程段 5 项全 [OK]；任务/Offboard 两项按"预期如此"报 [!!]（不再判 NG）；
  EV `pose_frame=1(NED)` / `quality=100` / `position.z=-0.049`（本机定高已并进 EV）；
  `cs_ev_pos / cs_ev_yaw / cs_ev_hgt = true`；`xy_valid / z_valid / v_xy_valid / v_z_valid = true`。
- 两个脚本 `bash -n` 均通过；板端 `/etc/systemd/system/px4-p2p-chain.service` 与
  `~/px4_ws/systemd/px4-p2p-chain.service` md5 一致（`2578dfc8ba4f2af56f911d4a7617edc9`）。

### 4. ⚠️ 唯一的行为变化（要记住）
不再起 Offboard 桥 ⇒ **飞控不能停在 OFFBOARD 模式**：本次 preflight 就读到 `nav_state=14`
（飞控当前正好停在 Offboard），这种状态下 `pre_flight_checks_pass` 会一直是 false、**解不了锁**。
起飞前用遥控器把模式拨到 **定点(Position)** 再解锁（沿用原来手动飞的流程）。
这是"去掉 Offboard 桥"的直接代价，也正是当初 `pos` 模式的设计意图。

### 5. 产物路径 / 回滚
- 板端备份：`~/backups/20261014_2158_mode_collapse/`（`etc/` 含旧单元 + 旧 drop-in
  `…service.d.removed-10-mode.conf` + `timesyncd.conf.d/`；`home/` 含 `run_p2p.sh`/`preflight.sh`/`systemd/`；
  另有 `px4_ws-full-src.tar.gz`，共 524K）。
- 本机快照（改动前版本）：`CHANGELOG.md.bak-20261014b`、`AGENTS.md.bak-20261014b`、
  `px4_ws\run_p2p.sh.bak-20261014b`、`px4_ws\tools\preflight.sh.bak-20261014`、
  `px4_ws\systemd\px4-p2p-chain.service.bak-20261014b`。
- 回滚：三个文件用上面的 `.bak-*` 覆盖回本机并 scp 上板；把
  `…service.d.removed-10-mode.conf` 放回 `/etc/systemd/system/px4-p2p-chain.service.d/10-mode.conf`；
   `sudo systemctl daemon-reload && sudo systemctl restart px4-p2p-chain`。

---
## 2026-10-15（补记 1）：**修好黑匣子（目录名撞车 ⇒ 整架次无录像）+ 16:0x 那次飞行的复盘证据**

### 1. 修复：`record_flight.sh` 目录名撞车
- 现象（今天没有录像的直接原因）：两次开机都算出同一个目录名 `20261014_221013` ⇒ 第二次
  `ros2 bag record -o` 报 `[ERROR] [ros2bag]: Output folder ... already exists` 并 **exit=1**
  ⇒ 16:0x 那次飞行（以及今天更早的班次）**一条数据都没录上**。
- 根因：目录名 = `date +%Y%m%d_%H%M%S`，而 **RTC 停走 + timesyncd 把这几次开机统统"恢复到"几乎同一时刻**
  （22:09:53 / 22:01:08）⇒ 两次开机算出同名。**它与"开机时钟差十几小时"是同一个根因**。
- 改法（`px4_ws/tools/record_flight.sh`）：① 名字追加本次开机唯一 id（`/proc/sys/kernel/random/boot_id`
  前 8 位）；② 真有残留就自动加 `-2/-3/…`；③ 开录 20 s 内就失败 → 换名重试，连续 3 次才放弃；
  ④ 跑起来之后再崩（如 10-14 那次 SQLite locked）→ 自动重起一份新 bag（宁可拆段，不能空白）；
  ⑤ `trap TERM/INT`：服务停时优雅收尾，不跟 systemd 抢。
- 验证：`bash -n` 通过；部署后录制器在跑，新目录 `~/flights/20261015_165758_e02deba6_a1`
  （`e02deba6` = 本次开机 id）；md5 `b6b0a8024262f56772d43a85754d9ff2`。
- 产物/回滚：板端备份 `~/backups/record_flight.sh.pre-fix-20261015`；本机
  `px4_ws\tools\record_flight.sh.bak-20261015`（改动前原文件，md5 `f7a259582977689f73c3984d313846a9`）。

### 2. 16:0x 那次的证据（boot 217612d5，真实 16:04:23–16:29:18，含 4 点多那次飞行）
- 开机时时钟**慢约 17h54m**（timesyncd 恢复到 "10-14 22:09:53"）；链路 t=23.7 s 起跑（有界等待 15 s
  超时未同步）；**NTP 直到 t=246.3 s 才同步（真实 16:08:29）**，时钟前跳约 18 小时 ⇒ **t=249.2 s
  cartographer 被 SIGABRT 打死**（`process has died [exit code -6]`），t=266 s 重新拉起 —— 即
  2026-10-08 记过的"时间跳变打崩 cartographer"，也是"起链不再死等 NTP"的代价。
- 该开机里无人机在 t=44–79 s（真实约 16:05:07–16:05:42）离地 0.2→0.67→0.11 m；t=149–249 s 长时间在
  0.10–0.42 m 低空活动 —— **正好横跨 t=246 s 的那次时钟跳变**（TOF 每 5 s 才打一行，分辨不出"5 秒"量级）。
- t=1431 s（真实约 16:28:13）**EV 桥自杀**：`RuntimeError: Lidar odometry unhealthy for 10.6s: SCAN_STALE`
  ⇒ 雷达 `/scan` 断了 10.6 s（> `unhealthy_timeout_sec=10`），桥按设计 fail-closed 退出；重启后一直刷
  `TF odom->base_link unavailable: "odom" ... does not exist`（cartographer 已不发布）。约 1 分钟后断电。
- 本次开机的黑匣子**没录上**（就是第 1 节的撞名 bug）⇒ 飞控侧数据缺失。

### 3. 仍未解决
- **13:00 前后那三次飞行的日志已不在板上**：journald 只保留最近 8 次开机（现在最老的是约 16:01 那次）。
  建议加长保留（`/etc/systemd/journald.conf` 的 `SystemMaxUse` / `MaxRetentionSec`），否则一天测试就被冲掉。
- 时钟根因未动：RTC 纽扣电池 + "时钟跳变看门狗（跳变且未解锁 → 重启链路）"待做。
- 门限取舍待讨论：`pose_timeout_sec=0.25` 是否太紧；雷达断 10 s 就让 EV 桥自杀是否符合"保飞机"优先。

---
## 2026-10-15（补记 2）：**飞行期间冻结校时 —— 链路不再被 NTP 跳变打死（方案①）**

### 0. 决策依据（用户提问："NTP 一定要现实时间吗"）
不要现实时间，只要**不跳变**。证据：2026-10-08 那次整条链就是在**慢 19 小时**的时钟上正常跑起来的，
出事恰恰是 NTP 把它校正过来的那一瞬间；今天 16:08:29 又抓到现行（跳变后 3 s cartographer SIGABRT）。
各组件需求：cartographer/SLAM、tofsense、EV 桥都只做"同一时钟下的相对比较"；PX4 自己的 hrt 与 UTC 无关；
**只有日志/录像的事后对时**才需要现实时间 ⇒ 所以"飞行期间冻结、停链后校正"是最划算的做法。

### 1. 改动：`px4_ws/systemd/px4-p2p-chain.service`
```
ExecStartPre=-+systemctl stop systemd-timesyncd
ExecStopPost=-+systemctl start systemd-timesyncd
```
- **只 `stop` 不 `timedatectl set-ntp false`**：后者会 *disable* 单元 —— 万一断电时正好在飞行中，
  下次开机就没有 NTP 了。`stop` 是非持久的（已确认 `systemd-timesyncd` = enabled，重启后自己回来）。
- `+` = 以 root 跑（systemctl 需要）；`-` = 失败不阻塞起链（失败仍记进 journal）。
- 效果：飞行期间时钟"错但稳定"（本次场景会差十几分钟到十几小时），**绝不会中途跳变**；
  停链后 timesyncd 启动即同步，日志/录像时间戳马上回到现实。

#### 1.1 已知副作用（用户 2026-10-15 要求"把副作用加到日志里"）
1. **飞行期间日志/录像的墙钟时间可能是错的**（今天这种场景差十几分钟到十几小时）。看 bag 时以
   bag **内部的相对时间线**为准，别信文件名或 `journalctl` 上的钟点；新目录名带 `boot_id`，
   仍能一眼认出是哪次开机、哪一段。
2. **手工 `bash run_p2p.sh`（不经 systemd）不会冻结校时** —— 这层保护只挂在单元上。
   想确认保护是否生效：链路运行中跑 `timedatectl`，应显示 **`NTP service: inactive`**。
3. `anacron` / `kerneloops` 这两个 `After=time-sync.target` 的单元，在"起链时时钟还没同步上"的
   情况下会跟着一起等（无害，不阻塞 `multi-user.target`）。
4. NTP 单元只是被 `stop`（**不是** `set-ntp false`），所以断电重启后它照常自己回来
   —— 已确认 `systemctl is-enabled systemd-timesyncd` = `enabled`。

### 2. 动手前的排查（板上只读检查）
- `systemd-timesyncd` / `systemd-time-wait-sync` 均为 enabled；`timedatectl` 改动前 = synchronized yes。
- 引用 `time-sync.target` 的单元只有：本链两个单元（`px4-ev-chain` 已禁用）、`systemd-time-wait-sync`、
  `anacron`、`kerneloops`。后两个即使一起等下去也无害，且不阻塞 multi-user.target（已核对
  `list-dependencies --after px4-p2p-chain` 里没有 time-sync）。

### 3. 验证（2026-10-15 17:08–17:10 实测）
- `systemd-analyze verify /etc/systemd/system/px4-p2p-chain.service`：本单元**无报错**
  （只有 netplan / snapd 两个无关单元的警告）⇒ `-+` 这个前缀组合 systemd 是认可的。
- `systemctl restart px4-p2p-chain`（17:08:58 → 17:10:01，63 s，仍是那个"停链要卡满
  TimeoutStopSec"的老毛病）之后：
  · `timedatectl` = **`NTP service: inactive`** ← **运行期间校时确实被冻结（本条改动的判据）**；
  · `journalctl -u systemd-timesyncd` 在 17:08:5x 有 `Stopping Network Time Synchronization... Stopped`；
  · `/fmu/in/vehicle_visual_odometry` 仍 **29.96 Hz**；黑匣子录制器在跑，新目录 `20261015_171004_e02deba6_a1`；
  · `systemctl is-enabled systemd-timesyncd` = **enabled**（重启后自己回来，没踩"永久禁用"的坑）。
- ⚠️ **journal 里看不到 ExecStartPre / ExecStopPost 的命令行本身**（这两条命令不打印任何东西），
  判据只能看"效果"：`NTP service: inactive` + timesyncd 的 Stopping/Stopped 记录。
- 双端 md5：单元 `53d7347b990da0c10fd350efc242f6fc`
  （本机 == 板上 `/etc/systemd/system/` == 板上 `~/px4_ws/systemd/`）。

### 4. 回滚 / 产物
- 回滚：删掉上述两行 + `sudo systemctl daemon-reload`。
- 本机快照：`px4_ws\systemd\px4-p2p-chain.service.bak-20261015`（改动前原文件）。

### 5. ⚠️ 还没做的（下一步）
- **台架验证"错但稳定"对飞控侧无害**（用户 2026-10-15 决定**暂缓**，先让把副作用记进日志）：
  停链 → 故意把时钟调错（例如 -18h）→ 起链 → 看 `/fmu/in/vehicle_visual_odometry` 仍 30 Hz、
  `cs_ev_pos/yaw/hgt` 全 true、`xy_valid/z_valid` 全 true、journal 无 reject。
  **这一步做之前，"错但稳定"只在本机（雷达/SLAM/EV 生成侧）有证据；飞控侧 epoch↔hrt 换算仍是
  未验证假设 —— 飞行前要么补做，要么上电后先确认时钟已经同步过再起飞（见补记 1 的临时规则）。**
- 方案②时钟跳变看门狗、journald 保留量、RTC 纽扣电池（见补记 1 第 3 节）。

---
## 2026-10-16：**回退机载代码到 2026-10-09 晚那一版（老方案）—— 已实测链路全绿**

### 0. 用户要求

> 「回退香橙派代码到刚给你发的那版 10.9 号的版本」

来源就是队友 2026-10-16 通过微信发来的 `restore_1009_evening.zip`（334 KB）。

### 1. 先核对「这份留档到底是什么」（重要，结论出乎意料）

- **形态**：解压后是一个 git worktree 的导出。`.git` 只是个 97 字节的指针文件，指向
  `C:/Users/lenovo/Documents/Codex/2026-10-09/wo/px4_ws/.git/worktrees/restore_1009_evening`
  —— 也就是**别人那台机器**（用户 `lenovo`）上的仓库；那台机器不在我们这儿，历史读不到。
  317 个文件、纯源码、全部 LF（被 `.gitattributes` 归一过）、无 `build/install/log`、无任何 `*.bak-*`（被 `.gitignore` 排除）。
- **时间点**：不早于 **10-09 22:0x**。依据：`px4_takeoff_land_node.cpp` 里有那条
  「⚠️ 2026-10-09 22:0x 修正：看门狗原来是盯 `/fmu/out/vehicle_status`（1.3Hz）→ 改成盯 `vehicle_local_position`（95Hz）」；
  同时带 10-09 白天/晚上的其余成果（串口雷达文档、IMU 桥、`report_velocity`、高度跳变保护、`mission_enable` 航线）。
- **不含 10-10 之后的任何东西**：没有 `p_to_p_mission` / `p_to_p_px4` 两个包、没有 `run_p2p.sh`、
  没有 `tools/{preflight.sh,set_dds_baud.sh,fastdds_udp_only.xml,mission.sh,record_flight.sh,watch_fc_link.py,…}`、
  没有 `systemd/px4-p2p-chain.service`，也没有 10-13 给雷达驱动打的 `uart_reader.cpp` 补丁。
- ⚠️ **它不是当晚板子状态的逐字节快照**：里面 `run.sh` 还带着「探 `/imu` 在 `pipefail` 下永远失败」的老 bug
  （= 实际总是选 `fly_no_imu.lua`，纯激光），`agent.sh` 也还是 15:58 那版（默认 921600、**不读 /etc/default**）。
  这两个文件比留档其余部分旧 —— 回退后要用下面的 §4「板上附加项」补齐，否则 DDS 直接不通。
- 与本机镜像 `D:\px4无人机\px4_ws` 逐文件比对：317 个文件全部存在于镜像中，247 个字节相同、
  57 个只差换行符（镜像 CRLF / 留档 LF）、13 个是 10-09 与后版的真实差异（run.sh、agent.sh、imu 桥、
  EV 桥 source+launch、takeoff_land source+launch+README、tofsense source+launch、ev-chain 单元、px4_goto.py、uart_reader.cpp）。

### 2. 动手前的备份（两层，都能整份回滚）

- 板上 `~/backups/20261016_before_rollback_0909/`（2.3 MB）：
  `px4_ws-src-pre-rollback.tar.gz`（回退前的源码树，不含 build/install）、
  `etc/systemd/system/{px4-p2p-chain.service,px4-ev-chain.service,px4-chain-watch.service}`、
  `etc/systemd/system/px4-p2p-chain.service.d/`、`etc/default/microxrce-agent`、
  `etc/profile.d/fastdds-udp.sh`、`home/bashrc`、`systemd-units.txt`、`fastdds_udp_only.xml`。
- 回退前的整套 p2p 代码**仍原样留在板上**：`~/px4_ws.p2p-20261016/`（含 build/install，未删）。
- 本机 `D:\px4无人机\px4_ws` 已 git 化：commit `4f4fa18`、tag `snapshot-20261016-board`
  （= 回退前那一版，逐文件 md5 与板上对齐过）。

### 3. 做了哪些动作

1. `sudo systemctl stop px4-p2p-chain`（原链停掉，确认无残留进程）。
2. 10-09 源码上传到 `~/px4_ws.new0909/`；然后把 `~/px4_ws` 改名 `~/px4_ws.p2p-20261016`、暂存树顶上叫 `~/px4_ws`。
3. `~/px4_imu_bridge.py` 换成 10-09 那版（10-09 的 `run.sh` 读的是 `$HOME/*.py`，不是工作区里的）；
   旧的留 `.bak-20261016`。`~/tf_to_pose.py` 核对 md5 与留档一致，未动。
4. systemd：`px4-p2p-chain` → **disable**；`px4-ev-chain` → 装回 10-09 的单元（727 B）+ **enable**；
   旧单元留 `.bak-20261016`。
5. `colcon build --symlink-install`：**10 个包全过**（`px4_msgs` 7min41s，其余 ~30s/包，合计 8min16s）。
6. `systemctl restart px4-ev-chain` → active，话题全绿（见 §5）。

### 4. ⚠️ 三个「板上附加项」（不在 10-09 代码里；删掉即完全回到留档原样）

写在 `/etc/systemd/system/px4-ev-chain.service.d/10-board-env.conf`：

- `XRCE_DEV=/dev/ttyS6`、`XRCE_BAUD=1500000` —— **必须**。
  10-09 的 `agent.sh` 只认环境变量（不读 `/etc/default/microxrce-agent`），而飞控侧 `SER_TEL3_BAUD`
  在 10-09 21:50 就已改成 1500000。不补这条，Agent 会按 921600 起来 → 与飞控不匹配 → DDS 一个字节都不通。
- `FASTRTPS_DEFAULT_PROFILES_FILE=/home/orangepi/px4_ws/tools/fastdds_udp_only.xml` —— **必须**。
  第一次起链实测：`/tracked_pose` 无数据、EV 桥报「没收到 /tofsense/height」（而 `/tofsense/height` 自己 55 Hz 正常）
  —— 就是 10-09 晚记录过的「FastDDS 同机 SHM 看得见、收不到」。给链路进程强制 UDP 后**两条同时恢复**。
  该 xml 是 10-10 才进 `tools/` 的，回退时特意保留（板上 `~/.bashrc` 和 `/etc/profile.d/fastdds-udp.sh` 都指向它，
  不保留会变成指向不存在的文件）。
- 另外：删掉留档根目录那个无意义的 `.git` 指针文件（它指向 `C:/Users/lenovo/...`，在板上没有意义）。

### 5. 验证（2026-10-16 17:00 板上实测；拆桨台架，全程未碰飞控参数）

| 项 | 结果 |
| --- | --- |
| `systemctl is-active px4-ev-chain` | **active** |
| Agent | `MicroXRCEAgent serial --dev /dev/ttyS6 -b 1500000` |
| `/scan` | 15.32 Hz |
| `/imu` | 63.99 Hz |
| `/tofsense/height` | 56.49 Hz |
| `/tracked_pose` | 29.95 Hz |
| `/fmu/in/vehicle_visual_odometry` | 15.00 Hz |
| EV 桥日志 | `EV: NED pos(0.01, 0.00, -0.05) yaw 90.0° \| 速度有(位置差分) \| 高度有效` |
| 板上时钟 | 2026-10-16 17:00，`NTPSynchronized=yes` |

飞控侧**只读**现象：EV 已在往飞控发（15 Hz），`/fmu/in/vehicle_visual_odometry` 有数据；
`listener vehicle_visual_odometry` / `cs_ev_*` / `xy_valid` 这些要上机或连 QGC 才能看（本次没做）。

### 6. 怎么回退这次回退（回到 p2p 那一版）

```bash
sudo systemctl stop px4-ev-chain
sudo systemctl disable px4-ev-chain
mv ~/px4_ws ~/px4_ws.0909 && mv ~/px4_ws.p2p-20261016 ~/px4_ws
sudo systemctl enable --now px4-p2p-chain        # 单元文件一直没删，只是 disable 了
cp ~/px4_imu_bridge.py.bak-20261016 ~/px4_imu_bridge.py
```

（更彻底的话：用 `~/backups/20261016_before_rollback_0909/` 里的 tar + etc 覆盖回去。）

### 7. 遗留 / 注意

- **飞控侧参数本次一律没动**：`SER_TEL3_BAUD=1500000`、`EKF2_EV_CTRL=15`、`EKF2_HGT_REF=3`、`EKF2_MAG_TYPE=5` 等
  都还是 10-13 那套（为「参考件 EV」调的）。10-09 老 EV 桥能用这套参数（10-09 晚就是它），
  但要不要把 `EKF2_EV_NOISE` 之类的值改回 10-09 的，**等用户定**。
- 10-09 版 `run.sh` 的 `/imu` 探针 bug **原样保留** → 实际跑 `fly_no_imu.lua`（纯激光），
  这正是 10-09 当晚「定点遥控垂直起降可用」的那套配置；那版还没有 `SLAM_USE_IMU` 开关。
- `px4_ws` 与 `px4_ws.p2p-20261016` 同时在板上：**`px4-p2p-chain.service` 里写死的路径是 `~/px4_ws/run_p2p.sh`**，
  现在那个文件不在 `~/px4_ws` 里了 —— 单元已 disable，但别手贱 `systemctl start px4-p2p-chain`。
- 本机镜像 `D:\px4无人机\px4_ws` **仍是回退前那一版**（= git tag `snapshot-20261016-board`），
  板上现在是 10-09 版，**两端已不一致** —— 下次要改板上代码前，先决定镜像跟不跟着切到 10-09。

---
## 2026-10-16：**本机 `px4_ws` 建 git 留档**

### 做了什么
- `git init`（仓库根 = `D:\px4无人机\px4_ws`，分支 `master`），提交 **`4f4fa18`**，打 tag **`snapshot-20261016-board`**，入库 **392 个文件**。
- 为保证「就是板上那版」，先做了逐文件 md5 对齐（板上 500 个文件）：**442 个相同**；2 份 `docs/{CHANGELOG,AGENTS}.md` 原本落后（10-14），按本机正本（10-15）刷新，旧版留 `.bak-20261016`；板上多出来的**全部是** `__pycache__` / `.pytest_cache` / `*.bak-*`（不入库，`.gitignore` 补了 `.pytest_cache/`）。
- 按 `.gitattributes`（`* text=auto eol=lf`）入库内容统一存为 LF；有 60 个文件工作区/板上是 CRLF（蓝海 SDK、`carto_pkg`、`serial_comm` 那批，队友打包带进来的），库内存 LF，**内容一致、只差换行符**。

### 回滚
`git -C "D:\px4无人机\px4_ws" reset --hard 4f4fa18`；不想要这层 git 就删掉 `D:\px4无人机\px4_ws\.git`（工作区文件不受影响）。

---
## 2026-10-16：**炸机复盘 —— 飞控在空中自动重启（附机载侧代码审计）**

### 0. 现象（用户描述）
低空飞了 20 多秒后「像断电一样掉下来」；悬停时约有 15 cm 漂移。

### 1. 时间线（全部来自板上日志，可复现）
| 时刻 | 事件 | 证据 |
| --- | --- | --- |
| 17:10:35 | 香橙派开机（本次飞行那次） | `journalctl --list-boots` |
| 17:11:05 | DDS 会话建立，链路起齐 | `/tmp/microxrce_agent.log` |
| 17:12:20 | 起飞（EV 的 z 从 −0.04 → −0.60） | `/tmp/px4_ws_ev.log` |
| 17:12:20–17:12:50 | **SLAM/EV 位置从 (0,0) 跑到 (−15.3, −1.3)**，约 0.15→0.75 m/s | EV 日志 |
| 17:12:27–17:12:40 | **TOF 高度逐帧乱跳 0.19~0.83 m**（跳变保护连续告警） | EV 桥 WARN |
| **17:13:14–17:13:22** | **飞控整段数据消失 7.8 s**（电机瞬间没输出） | IMU 桥：`17:13:22 7.8 秒没收到 IMU（飞控断了？）` |
| **17:13:19** | **uXRCE-DDS 会话被重新建立**（飞控侧重建全部 DDS 实体） | agent 日志 `session re-established`（Agent 进程未重启） |
| 17:14:05–17:14:25 | 位置平滑移动约 9 m、高度 0.2~0.8 m = 有人把飞机抱回来 | EV 日志 |

### 2. 结论与排除项
- **结论：飞控在空中自己重启了**。三条独立证据：① IMU 流中断 7.8 s；② DDS `session re-established`（Agent 进程自始至终没重启，是**飞控侧**客户端重连）；③ 飞控 RAM 被清零——`latest_arming_reason = 0` / `latest_disarming_reason = 0`，而这两个字段只在**解锁/上锁那一刻写入**（`Commander.cpp:610 / :655`），实战中解锁过却读到 0，只能是复位。
  （注：香橙派**没有**重启——`uptime` 连续、链服务一直 active；电池事后正常 16.1 V；遥控链路 `valid: true`。）
- **排除「机载侧安全措施/看门狗导致」**（用户直接问过）：
  · 机载侧唯一能碰飞控的是 EV 桥，只发 `/fmu/in/vehicle_visual_odometry`；`run.sh fly` 不启动任何发命令的节点；整棵 10-09 树里**没有** 185（飞行终止）、**没有** 246（重启）命令码。
  · 10-09 那版**没有看门狗**；10-10 以后加的 `chain_watchdog/lidar-watch` 单元当时一个 disabled、一个没安装；`px4-ev-chain.service` 是 oneshot，**无 `Restart=`**；定时器/cron 里没有任何碰链路的条目。
  · PX4 侧也堵死了：**armed 状态下 246（重启）与 `VEHICLE_CMD_PREFLIGHT_STORAGE`（重置校准会重启）都被 `TEMPORARILY_REJECTED`**（`Commander.cpp:1219 / :1402`）；失败检测器 `fd_critical_failure` 的动作在 `CBRK_FLIGHTTERM=121212` 下退化成 **Disarm**（切动力，不是重启）；FMU **没有**活的硬件看门狗（PX4 只在 px4io/uavcan 固件里调 `watchdog_init()`）；`load_mon` 只告警。
  ⇒ 剩下只有两类可能：**供电瞬断（brownout/BOR）** 或 **飞控固件硬故障**，或有人手发命令。
- **另外两个实测问题（与"掉下来"无直接因果，但必须修）**：① SLAM/EV 位置在 25 s 内跑飞 15 m（起步 ~0.15 m/s，正是用户看到的"15 cm 漂移"，与 10-08「飞走 19 米」同机理）；② TOF 高度贴地时逐帧乱跳，喂给 EKF 的 z 不可信。
- 查到/搜到的相关线索：上游 micoair 板子的 PR #27876 / #28201（复位瞬间把 PWM/DShot 输出脚拉低）、#28731（H7 上确保 ESC 完成 DShot 初始化）、#28746；H7 复位/hardfault 相关 #27642（bootloader 启动前清理 flash ECC）、#28976、#29252。**这些修复都不在 1.15.4 里**。
- ⚠️ 顺带发现：本机 `D:\px4无人机\PX4-Autopilot-1.15.4` 的 `boards/` 里**没有 micoair**（上游 1.15.4 不含该板）⇒ 飞控固件不是从这棵树编的（应是 MicoAir 自己那条线/预编译包），**固件侧细节（看门狗配置、复位原因）待确认**。

### 3. 回滚 / 产物
只读排查，无需回滚。相关日志仍在板上 `/tmp/px4_ws_*.log`（那次开机）。

---
## 2026-10-17：**机载端换成「队友那套 MAVLink 链」（`~/mavlink_ws`）—— 已飞通，比原方案更好**

### 0. 用户要求
> 「把这个工作空间放到我最开始那架无人机那里……放一个不冲突原本工程的又一个新工程_ws……我想试试这个新的_ws能不能用」
> 之后：「一开机用的是新链而不是旧链」。

### 1. 新链 vs 原 `px4_ws`（差异一览，做对比时看这张）
| | 原 `~/px4_ws`（10-09 那版） | 新 `~/mavlink_ws`（队友那套） |
| --- | --- | --- |
| 定位注入通道 | **uXRCE-DDS**（Agent + FastDDS，ttyS6 @1500000） | **MAVLink ODOMETRY(msg 331) + TIMESYNC**（ttyS6 @115200） |
| 高度 | EV 桥把 TOF 高度并进 EV 一起发 | 独立 **DISTANCE_SENSOR(msg 132)**，飞控 `EKF2_HGT_REF=2(Range)` |
| 里程计来源 | `px4_ev_bridge`（自己写，读 `/tracked_pose`） | `mavlink_bridge/tf_to_odom`（读 TF `odom→base_link`，带速度时窗差分/抗跳变） |
| SLAM 配置 | `carto_pkg` + **`fly_no_imu.lua`**（纯激光） | `my_carto_pkg` + **`amphi.lua`**（与 10-09 留档里那份**逐字节相同**） |
| 雷达 | 串口 LDS-50C-R（`uart_lidar.yaml`，ttyS4） | 原机是网口 LDS-50C-E（`lidar_net.yaml`）；**在 147 上已切回串口参数** |
| 控制 | PX4 Position 模式 + 遥控器 | 队友自研位置 PID（`tasks/hover_test.py`，走 `/target_velocity` → SET_POSITION_TARGET_LOCAL_NED = Offboard） |
| 起飞/高度基准 | `EKF2_HGT_REF=3(Vision)`、`SYS_HAS_GPS=0` | `EKF2_HGT_REF=2(Range)` |
| 启动 | `px4-ev-chain.service` → `run.sh fly`（7 步） | `mavlink-ws-chain.service` → `start_mavlink_ws.sh` → `ros2 launch mavlink_bridge drone_stack.launch.py` |
| 依赖 | Agent 二进制（/usr/local/bin + /usr/local/lib 5 个库） | `pymavlink`/`fastcrc`/`pyserial`（本机无 pip，用 **third_party 挂载**，不动系统） |

### 2. 落地内容（`10.23.87.147:/home/orangepi/mavlink_ws/`）
来源：队友 2026-10-16 的交接包（`D:\px4无人机\新建文件夹\`，原机 = 10.23.87.150）。
```
mavlink_ws/
├── src/{mavlink_bridge, my_carto_pkg, bluesea2}   # 见 §3 的适配
├── config/{lidar_uart.yaml, lidar_net.yaml, fastdds_udp_only.xml}
├── tasks/        # 队友的 hover_test.py / run_hover.sh 等（原样参考）
├── optional_pkgs/{activity_control_pkg, pid_control_pkg}  # 原工程残留，放 src 外不编译（有 COLCON_IGNORE）
├── ref/{README_交接包.md, px4_params/, sample_log/, notes/, tools/}  # 含队友的 px4_log.py / ulg_*.py
├── third_party/{pymavlink-2.4.50, fastcrc-0.5.0, pyserial-3.5}  # wheel 解包，PYTHONPATH 挂载
├── start_mavlink_ws.sh   # 147 版启动/停止脚本（带 PYTHONPATH + FastDDS UDP 导出）
└── ADAPT_FOR_147.md      # 改了哪几行、跑之前要满足什么、判据、回滚
```
编译：`colcon build --symlink-install` → **4 个包全过**（base_lidar / bluesea2 / mavlink_bridge / my_carto_pkg）。

### 3. 为 147 做的 5 处适配（都在上面那份 `ADAPT_FOR_147.md` 里）
1. `mavlink_bridge/launch/drone_stack.launch.py`：写死的 `/home/orangepi/install/my_carto_pkg/...` → `get_package_share_directory("my_carto_pkg")`。
2. 同文件：默认雷达参数 `/home/orangepi/lidar_net.yaml`（网口）→ 包内 `config/lidar_uart.yaml`（**147 是串口雷达**）；网口参数一并打包，`lidar_params_file:=` 可切。
3. `mavlink_bridge/setup.py`：`data_files` 补两行，把上面两个 yaml 装进 share。
4. `my_carto_pkg`：交接包只给了 `03_slam/`（配置/launch/urdf），补了标准 `package.xml` + `CMakeLists.txt`（只做 install）。
5. `start_mavlink_ws.sh`：**去掉 `set -u`**（否则 `source /opt/ros/humble/setup.bash` 会因 `AMENT_TRACE_SETUP_FILES` 未定义而失败 → 服务反复重启，实测踩过）、加 `PYTHONPATH=$WS/third_party`、加 `FASTRTPS_DEFAULT_PROFILES_FILE`（同机 SHM 偶发"看得见收不到"）。
   另加 systemd 单元 `/etc/systemd/system/mavlink-ws-chain.service`（`Type=simple`、`SupplementaryGroups=dialout`、`Restart=on-failure`、`KillMode=control-group`）。
   依赖用 **wheel 解包 + PYTHONPATH**（147 上没有 pip，也不装系统包）：`pymavlink 2.4.50` + `fastcrc 0.5.0` + `pyserial 3.5`。

### 4. 飞控侧参数改动（QGC 里做，只有人工能改）
| 参数 | 改成 | 为什么 |
| --- | --- | --- |
| `UXRCE_DDS_CFG` | `0`（原 TELEM3） | 关掉 DDS，TELEM3 让给 MAVLink |
| `SER_TEL3_BAUD` | `115200`（原 1500000） | 与桥接一致 |
| `MAV_1_CONFIG` | `103`（= TELEM3）+ `MAV_PROTO_VER=0` + `MAV_HASH_CHK_EN=1` | 给 TELEM3 建一个 MAVLink 实例（**改完必须重启飞控才生效**，实测） |
| `EKF2_HGT_REF` | `2`（Range） | **关键**：这套桥接在垂直方向明确"不提供信息"（`odom_position_noise_z=1e6`），高度只能靠测距 |
| `EKF2_RNG_CTRL` / `EKF2_RNG_POS_Z` | `2` / `0.044` | 启用测距融合 + 测距相对 IMU 的 z 偏置 |
| `EKF2_RNG_NOISE/GATE/DELAY/QLTY_T/SFE/MIN_RNG`、`EKF2_GND_EFF_DZ=4`、`EKF2_GND_MAX_HGT=0.5`、`EKF2_BARO_CTRL=1` | 同队友那台 | 照抄队友那台"能飞"的取值 |
> 注：147 的固件里**没有** `MAV_1_MODE` / `MAV_1_FORWARD`（队友那台有）⇒ 两架固件构建可能不同，待确认。

### 5. 验证
- 开机自启：`px4-ev-chain = disabled`（保留）、`mavlink-ws-chain = enabled + active`。
- 机上：`链路=OK`、`ekf_ok=True`、`坏帧=0`、`电池≈16.3V/90%`、雷达+SLAM+桥接+定高全在跑；`ODOMETRY`/`DISTANCE_SENSOR` 帧数持续上涨。
- 飞控 uORB 侧（用户在 QGC MAVLink 控制台直接核实）：
  · `listener vehicle_visual_odometry` → position ≈ (−0.009,−0.004,0)、`position_variance=[0.0025,0.0025,1e6]`（z=不提供，符合设计）、`pose_frame=1(NED)`、`quality=100`；
  · `listener distance_sensor` → `current_distance=0.05 m`、`variance=0.0004`、`orientation=25(朝下)`、`type=0(激光)`、`signal_quality=100`。
- 高度：从改参数前的 **±352 cm 乱跳** → **稳定 9~10 cm**，`Preflight Fail: height estimate not stable` 消失。
- **实际飞行：成功，且比原方案更好**（用户反馈）。

### 6. 切换 / 回滚
```bash
# 用新链（当前默认）
sudo systemctl disable --now px4-ev-chain
sudo systemctl enable  --now mavlink-ws-chain
# 手动起/停新链
bash ~/mavlink_ws/start_mavlink_ws.sh        # 或 --net 用网口雷达参数；stop 停
# 回旧链（10-09 DDS 那版）
sudo systemctl disable --now mavlink-ws-chain
sudo systemctl enable  --now px4-ev-chain
# 并把飞控参数改回：UXRCE_DDS_CFG=TELEM3、SER_TEL3_BAUD=1500000、MAV_1_CONFIG 复原
```
产物：板上打包 `/tmp/mavlink_ws_147_20261017.tar.gz`（源文件，不含 build/install/log）；本机留档 `D:\px4无人机\mavlink_ws_147_20261017.tar.gz`。

### 7. 与「硬件 vs 软件」那条主线的关系（重要观察，非定论）
**同一架机体**：10-16 用 DDS 链飞 → 飞控空中重启掉下来；10-17 换 MAVLink 链 → 飞得比以前都好。
差异集中在：① 定位注入通道（uXRCE-DDS 1.5 Mbaud + 飞控 100 Hz 遥测洪流 vs MAVLink 115200 轻载）；② 高度来源（Vision vs Range）；③ 机载软件栈（我们的 vs 队友的）。
⇒ **证据指向"原链那条路径（DDS 传输/负载 或 其下的供电/固件行为）"，而不是机体本身**；但严格定论还需要那次崩溃的飞控 ulog（复位原因、电机指令、电压电流曲线）。

### 8. 另一架（10.23.87.150）的状态
按用户要求把**同一份 10-09 工作区**拷成了 `~/px4_ws_0909`（与该机无冲突）：与 147 的源码树 **逐字节同源**（文件名清单指纹 `92c6d9ab…`、名+内容指纹 `550b86df…`，322 文件），`colcon build` 10 个包全过；它自己的 `drone-link.service`/`ros2_ws`/`cartographer_ws` **未动**。
只差一样：该机没有 `MicroXRCEAgent`（包已备好 `/tmp/xrce_agent_147.tar.gz`，md5 `6d18c179…`，本机也留了一份），传过去就能跑 DDS 链——**目前没装、也没启用**（它仍跑自己的 MAVLink 链）。

### 9. 仍未做 / 待确认
- 取回 **10-16 那次崩溃的飞控 ulog**（SD 卡 `log/*.ulg` + 有没有 `fault_*.txt`），跑 `~/mavlink_ws/ref/tools/ulg_report.py` / `ulg_hover_diag.py` → 给"飞控为什么重启"定论。
- 两架飞控**固件是否同一构建**（`ver all` + `param show MAV*` 对比）。
- journald 保留量太短（13:00 前后那几次飞行日志已被冲掉）、RTC 纽扣电池、时钟跳变看门狗（见 2026-10-15 补记）。

---
## 2026-10-17 晚：**跳变保护（fail-closed）——已注入测试通过**

### 1. 事故证据（22:07 那次）
```
22:06:59  alt=58cm  PX4位置=(+0.11,+0.10)
22:07:01  → 里程计跳变 272.35m（阈值 0.5m）→ reset_counter=1
22:07:03  PX4位置=(+112.75,+248.10)      ← EKF 锚到 273 m 外
22:07:11  位姿跳变 102cm/s（累计 32 次）
22:07:13  测距 5.1cm → 287.6cm           ← 已翻
22:07:17  链路失联 3s（复位，电机抽动）
```
⇒ 坠落起因是 **SLAM 位姿跳变**（不是飞控供电）。飞控复位是落地后的结果。
桥上累计记录 19 次跳变（多数在链路启动瞬间，1 次在飞行中 272 m）。

### 2. 改动（`~/mavlink_ws/src/mavlink_bridge/`，已备份 `.bak-20261017`）
- `bridge_node.py` 第 4 段（跳变检测）重写为 fail-closed；新增参数 `odom_jump_hold_s`（默认 2.0 s）。
- `config/mavlink_bridge.yaml` 增加 `odom_jump_hold_s: 2.0`。
- 启动预热：位置连续稳定 `odom_jump_hold_s` 秒才开始发 EV（防冷启动假位置成为基准）。

### 3. 验证（板上实测）
- 正常启动：`EV 预热完成（位置稳定 2.0s），开始发送里程计` ✓
- 注入 5.01 m 假位姿（停掉 `tf_to_odom`、另起假发布器 6 s）：
  `里程计跳变 5.01m（阈值 0.5m），停止发送 EV（不递增 reset_counter）` ✓
  `位姿稳定在新位置（偏离 5.01m）——判定为 SLAM 重定位，EV 保持停发；请停链重启` ✓
- 测试后重启链路恢复：服务 active、预热完成 ✓

### 4. 回滚
把 `bridge_node.py.bak-20261017` / `mavlink_bridge.yaml.bak-20261017` 覆盖回去 →
`colcon build --symlink-install --packages-select mavlink_bridge` → `sudo systemctl restart mavlink-ws-chain`。

### 5. 仍未解决（下一步）
- **SLAM 为什么会跳**（根因）：147 是串口 LDS-50C-R + 10-09 版 `bluesea2`，队友那台是网口 LDS-50C-E；`amphi.lua` 按网口款调。150 上线后需逐文件比驱动/雷达参数。
- **yaw 不稳**：1.16 上仍有 `Preflight Fail: heading estimate not stable`；确认 `EKF2_MAG_TYPE`（改回 5 或校准磁力计）。
- 飞控当前停在 **OFFBOARD**（未解锁），飞前要拨回 **POSCTL**。

## 2026-10-19 修复记录 #6：Cartographer 改为队友同款 2.0.9003（147 源码编译）

### 1. 背景
147 之前用的是 apt 混装版本：`cartographer 2.0.9004` + `cartographer_ros 2.0.9002`；
队友那台是源码编译的 `2.0.9003 / 2.0.9003`。apt 源里没有 9003，无法直接降级，
因此在 147 上按队友源码重新编译，并让启动脚本优先加载新版本。

### 2. 编译
- 源码：`cartographer` 提交 `ddfd4414ef2907b6c46a195f1d4e380beedce05d`，
  `cartographer_ros` 提交 `1bbf9af3d250ba7d17f9b2340e7fe01ac22cf7a7`，均为 `ros2` 分支。
- 目录：`/home/orangepi/cartographer_ws`；补装 `liblua5.3-dev`。
- 命令：
  ```bash
  cd /home/orangepi/cartographer_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --parallel-workers 2 \
    --packages-select cartographer cartographer_ros cartographer_ros_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```
- 结果：3 个包全部成功，10 min 10 s；生成 `cartographer_node` 和 `libcartographer.a`。

### 3. 切换
- 只改 `~/mavlink_ws/start_mavlink_ws.sh`：在 `/opt/ros/humble/setup.bash` 之后、
  `$WS/install/setup.bash` 之前增加
  `source /home/orangepi/cartographer_ws/install/setup.bash`。
- 备份：`start_mavlink_ws.sh.bak-20261019`。
- `my_carto_pkg` 的配置、launch、URDF、`tf_to_odom.yaml` 均未动。

### 4. 验证（2026-10-19 21:01，地面、未解锁）
- `cartographer_ros` / `cartographer_ros_msgs` 版本均为 `2.0.9003`。
- `cartographer_DIR` 与 `link.txt` 均指向 `/home/orangepi/cartographer_ws/install`。
- 运行进程路径为 `.../cartographer_ws/install/cartographer_ros/lib/cartographer_ros/cartographer_node`。
- 服务 `mavlink-ws-chain` active；`链路=OK`、`ekf_ok=True`、`坏帧=0`；
  `/scan` 约 15.1 Hz；ODOMETRY 持续递增，跳变丢弃 0；测距约 5.9 cm。

### 5. 回滚
恢复 `start_mavlink_ws.sh.bak-20261019` → `sudo systemctl restart mavlink-ws-chain`，
即回到 apt 的 9004/9002。

### 补充结论（2026-10-19 21:40）
- `2.0.9004` 相对 `2.0.9003` 只有 `package.xml` 和 CMake 查找 Boost 的改动；
  `2.0.9002 → 2.0.9003` 只有 Ceres 2.1 `Manifold` API 和 rosbag2 Rolling 编译兼容。
  147 的系统 Ceres 是 2.0.0，`Manifold` 分支不会编译，因此 2D SLAM 算法基本一致。
- 源码里的未提交补丁（landmark 清理、velocity/tracked_pose 发布）在本机配置下均未生效：
  `use_landmarks=false`、`publish_tracked_pose` 关闭、`/velocity_map` 无订阅者。
- 21:28:47 仍出现 4.95 m 跳变，桥停发 EV、`ekf_ok=False`；21:39:41 地面重启后恢复。
  说明“不炸机”主要归因于 `mavlink_bridge` 的 fail-closed 跳变保护，而不是 Cartographer 版本本身。

## 2026-10-20 修复记录 #7：链路启动前强制等待 NTP 同步

### 1. 事故与根因
- 20:16:40.523 `systemd-timesyncd` 首次同步；20:16:40.887 桥检测到
  **180.34 m** SLAM 跳变；20:16:47 Cartographer 报 `scan rate: 0.03 Hz`、
  `38s ± 569s`，20:17:02 恢复 15.12 Hz。
- `bluesea2` 串口驱动默认 `time_mode=0`，`DecTimestamp()` 使用
  `gettimeofday()`。链路启动早于 NTP 校时，校时后扫描时间戳跳变，
  Cartographer 把巨大时间差当成运动，输出数百米假位姿。

### 2. 改动
- 去掉 `mavlink-ws-chain.service` 对
  `systemd-time-wait-sync.service` 的硬依赖，避免离线飞行时链路无法启动。
- `start_mavlink_ws.sh` 增加条件门控：`wlan0` 已连接（调试模式）时等待
  `NTPSynchronized=yes`，最多 300 s；未连接 Wi-Fi 时按离线飞行模式立即启动。
- 离线启动后不要中途连接 Wi-Fi；如需连网调试，先停链、同步 NTP 后重启。
- 备份：`mavlink-ws-chain.service.bak-20261020`、
  `start_mavlink_ws.sh.bak-20261020`。

### 3. 验证
- 重启日志：`等待 NTP 时间同步 ...` → `NTP 已同步，继续启动链路。`
- 扫描 15.15 Hz；EV 预热完成；`ekf_ok=True flags=0x1af`，
  高度 9 cm，ODOMETRY 934 帧，跳变 0。
- 首次 NTP 同步约 155.9 s；首次 `ekf_ok=True` 约 15 s after gate；
  总开机到可用约 2 分 51 秒。主要耗时是 Wi-Fi 关联，不是 NTP 查询。
- `amphi.lua`、飞控参数、Cartographer 配置均未改动。

## 2026-10-24 D 题任务移植到 PX4（机载端代码就绪，未实飞）

### 1. 需求与来源
- 用户要求把 2024 电赛 D 题匿名飞控版本的货架盘点逻辑移植到当前 PX4 飞机；
  激光雷达继续使用 `/dev/ttyS4`，摄像头使用 `/dev/video0`。
- 源码基线：`D:\24电赛d题\drone_d_ws`（该目录文档记录其机载端在 2026-09-30
  已逐文件核对一致）。匿名飞机旧 IP `10.23.87.97` 本次不在当前网段，未能重新登机
  比对；实飞前如该板 IP 已变化，仍应再做一次双端源码哈希核对。
- 未修改匿名飞机代码，所有 PX4 改动落在
  `D:\px4无人机\mavlink_ws_git_repo_20261020\repo`。

### 2. 移植内容
- 新增 `shelf_scan_pkg`：保留场地几何、两排弓字形 12 点位航线、二维码识别、
  激光上升沿打点、地面站协议、避障规划和 23 项机载自测。
- 新增 `pid_control_pkg`：保留位置 PID，但新增 `velocity_output_frame: body`；
  将原匿名飞控版的地图系 `(vx,vy)` 通过当前 yaw 旋转成机体系 FLU，
  再接现有 `mavlink_bridge`，避免 PX4 前后/左右方向发反。
- 新增 `shelf_px4_bridge`：代替 `uart_to_stm32` 和匿名 `takeoff_if` 逻辑，
  负责预检、零速度 Offboard 预推、GO 后自动解锁、发布 `/fc_takeoff_if`、
  监控 SLAM/高度/任务状态，结束后发送 PX4 Land 并等待上锁。
- 启动方式改为与现有已验证流程一致的两段式：
  `~/mavlink_ws/shelf_preflight.sh` → `~/mavlink_ws/shelf_takeoff.sh`；
  中止为 `~/mavlink_ws/shelf_abort.sh`。
- `mavlink_bridge.yaml` 的 `max_yaw_rate_deg_s` 从 6 提到 20；
  6°/s 时 D 航线 A→B 面约 180° 转向会逼近 25 s 转场超时。
- 摄像头默认 `/dev/video0`；激光笔沿用排针 18 脚，Linux 映射为
  `gpiochip1` 第 4 脚（全局 GPIO 36），低电平点亮。新增
  `99-orange-pi-gpio.rules`，将 gpiochip 权限交给已有的 `dialout` 组。
- 新增 `laser_test.sh`，允许现场分别测试低电平/高电平极性，最长 10 秒，
  结束或中断时自动恢复关闭电平；任务或激光节点运行时拒绝执行。

### 3. 香橙派环境改动
- 安装编译依赖：`libopencv-dev`、`libzbar-dev`、`ros-humble-angles`、`gpiod`。
- 安装 `/etc/udev/rules.d/99-orange-pi-gpio.rules`；本次没有改飞控固件或参数。
- 改动前完整源码备份：
  `~/backups/20261024_before_d_mission_px4_port/mavlink_ws-src-before-d-port.tar.gz`
  SHA-256 `87579b98423ff4dff277bbeb02f885192abfc1e51b96c237469cbce56ff00ad4`。

### 4. 验证
- 香橙派上 `colcon build --symlink-install --packages-select
  shelf_scan_pkg pid_control_pkg shelf_px4_bridge`：3 包成功；
  最终增量编译无 compiler warning/error。
- `shelf_selftest`：23/23 通过，原 12 点位航线长度 1094.5 cm、
  预计用时 71.5 s 与匿名飞机地面站黄金数据一致。
- `shelf_px4_bridge` Python 单测：5/5 通过。
- 地面非接管启动：`/dev/video0` 摄像头、MJPEG 8080、激光节点、任务节点、
  位置 PID、PX4 适配器全部启动且无残留；PX4 当前为 `AUTO`、未解锁，
  因此预检按设计停在 `PREFLIGHT WAIT: PX4 mode is AUTO, select OFFBOARD`。
- 重启 `mavlink-ws-chain` 后 `map -> odom -> base_link` TF 恢复，
  说明此前“新会话看不到 TF”是旧链路网络切换后的 DDS 会话问题。
- **未解锁、未起飞、未做激光真实点亮和二维码现场识别验收。**

### 5. 回滚
- 机载端顶层文件和三个新包可直接由上述 tar 包恢复；
- 本机 Git 分支为 `mavlink_ws`，本次提交可用 `git revert <commit>` 回滚；
- `99-orange-pi-gpio.rules` 若不需要，可恢复到本次安装前的同名
  `.bak-20261024`（如存在），再执行
  `sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=gpio`。

### 6. 实飞前硬门禁
- 必须在 QGC/遥控器把 PX4 放在 **OFFBOARD**，确认未解锁、EKF 正常；
- 必须按当前 PX4 飞机的相机、激光和雷达安装位置重新核对 `field.json`
  悬停点与激光光斑，不能直接认为匿名飞机实测坐标完全复用；
- 先拆桨验证 `/target_velocity` 的机体系 X/Y 方向、激光 GPIO 和摄像头预览，
  再做单点短航线，最后才跑整趟 12 点扫描。

## 2026-10-24 补记：PX4 专用网页地面站

### 1. 新增内容
- 新增 `mavlink_ws/groundstation_px4/`，在原 D 题地面站基础上重做为简洁
  单页界面；保留场地/航线/盘点结果/查询/CSV，新增机载相机实时 MJPEG、
  PX4 预检状态、飞行模式、电量、解锁状态。
- 网页主按钮“起飞并开始盘点”在 PX4 状态为 `WAIT_GO` 时启用；地面站发送
  `px4_start`，由 `shelf_px4_adapter` 再复核预检、解锁、切换/确认 Offboard、
  最后才对任务节点放行。地面站不能绕过机载端安全门。
- 支持遍历盘点和定向盘点。地面站通过 `/shelf_scan/takeoff_mode` 指定本次
  任务模式，不再使用会提前把任务节点置为 Arming 的匿名 `start_route`。
- 机载端新增 `/shelf_scan/gs_out` 的 `px4_status` 心跳和
  `/shelf_scan/gs_in` 的 `px4_start/land/estop` 指令处理。
- 默认地面站电脑 IP 写入 `mavlink_ws/config/groundstation.env`：
  `ws://10.249.114.28:8890/ws`；IP 变化时只需改这一行。

### 2. 验证
- 网页模拟模式完整跑完 12 点遍历盘点，3 个地面站后端单测通过。
- 实机端 `mavlink-ws-chain` active，地面站 WebSocket 已连接香橙派
  `10.249.123.10`；PX4=OFFBOARD、未解锁、EKF 正常、电量 89%。
- 实际验证进入 `WAIT_GO`，网页按钮可用；相机画面为 640×480、浏览器无
  JS 报错。未点击起飞按钮，未解锁。
- 通过网页发送未解锁状态下的 `land` 请求，机载适配器收到并安全退出
  启动流程，证明地面站→WebSocket→ROS 下行链路已连通。
- `maps -> odom -> base_link` 与实时相机均在预检中正常。

### 3. 使用入口
- Windows：`mavlink_ws/groundstation_px4/run_groundstation.bat`
- 演示：`run_demo.bat`
- 页面：`http://127.0.0.1:8765/`
- 机载预检：`~/mavlink_ws/shelf_preflight.sh`

### 4. 回滚
- 删除 `groundstation_px4/` 不影响机载任务；
- 机载恢复上一提交即可回到只有命令行起飞流程的版本。
