# drone_d_ws 代码解析（2024 电赛 D 题 · 机载端）

> 解析对象：`drone_d_ws.zip`（已解压到 `_extract_drone_d_ws/drone_d_ws/`）。
> 这是一套 **ROS 2 Humble colcon 工作区**，跑在香橙派（Orange Pi）上，配套电脑端
> `groundstation/`（**不在本压缩包内**）使用。

---

## 0. 一句话结论

代码本身结构清晰、工程化程度高：**D 题新增的 `shelf_scan_pkg` 只负责"任务层"**
（场地几何 → 航线执行 → 定高/到位 → 激光 → 识别 → 上报），
**飞行控制完全复用原有工程**（cartographer 定位 + `pid_control_pkg` 位置 PID +
`uart_to_stm32` 串口桥 + STM32 飞控）。

但**配置层有一个必须先解决的问题**：比赛用的
[field.json](C:/Users/zxcv3/Documents/ChatGPT/无人机题目/_extract_drone_d_ws/drone_d_ws/src/shelf_scan_pkg/config/field.json)
只建模了 **12 个点位（1 个货架的 A/B 两面）**，而题目要求 **24 个二维码（A/B/C/D 四面）**。
详见第 6 节 P0-1。

---

## 1. 目录与包一览

```
drone_d_ws/
├── build.sh / run.sh / stop.sh / env.sh     一键编译 / 启动 / 停止 / 环境
├── src/
│   ├── shelf_scan_pkg/     ★ D 题新增（本次解析重点，约 5.4k 行）
│   ├── uart_to_stm32/      原有包 + 新增 /laser_control → 0x21 激光帧
│   ├── pid_control_pkg/    原有包：位置 PID（/target_position → /target_velocity）
│   ├── my_carto_pkg/       原有包：雷神雷达 + cartographer 建图定位 + URDF
│   ├── bluesea2/           雷神雷达驱动（含 base_lidar）
│   ├── serial_comm/        Boost 串口库（uart_to_stm32 依赖）
│   └── my_launch/          只保留 drone_config.yaml（PID 参数）+ 配置加载器
├── tools/                  gen_golden / verify_model / gs_stub / ws_bridge / flight_test
└── docs/                   5 份中文文档（使用说明、集成、STM32 改造、自测、验收清单）
```

`src/shelf_scan_pkg` 内部分层（这就是"ShelfScan 核心库 + 4 个可执行节点"）：

| 文件 | 行数 | 职责 |
|---|---|---|
| `src/mission_node.cpp` | 1482 | **任务主状态机**（最大最核心的文件） |
| `src/gs_link_node.cpp` | 732 | 链路桥：WebSocket / 串口 / UDP ↔ ROS 话题 |
| `src/code_reader_node.cpp` | 516 | 二维码 / AprilTag 识别 |
| `src/laser_node.cpp` | 205 | 激光笔驱动（topic/gpio/serial/none） |
| `src/field_geometry.cpp` | 393 | 场地几何 → 24(12) 个点位坐标、障碍、安全包络 |
| `src/route_planner.cpp` | 556 | 可见图避障 + 最近邻 + 2-opt 航线规划 |
| `src/gs_protocol.cpp` | 378 | 坐标变换 + JSON 报文构造/解析 |
| `src/simple_serial.cpp` | 191 | poll 实现的串口（Linux only） |
| `src/shelf_selftest.cpp` | 328 | 逻辑自测（与地面站模型逐项比对） |
| `include/shelf_scan_pkg/json_lite.hpp` | 476 | 零依赖 JSON 解析/生成（含 `\uXXXX` 代理对） |

---

## 2. 运行时数据流

```
  地面站(PC)                        香橙派 drone_d_ws
  ┌────────┐   WebSocket/数传/UDP   ┌──────────────────────────────────────┐
  │ 规划/显示│◀──────────────────────▶│ shelf_gs_link_node                   │
  └────────┘   (一行一条 JSON)        │   /shelf_scan/gs_in  ◀──┐            │
                                     │   /shelf_scan/gs_out ──▶┘            │
                                     └───────┬──────────────▲───────────────┘
                                             │ String(JSON) │ String(JSON)
                                     ┌───────▼──────────────┴───────────────┐
                                     │ shelf_mission_node（任务状态机）      │
                                     │  · 场地坐标 ⇄ map 坐标（标定）        │
                                     │  · 航段调度 / 定高 / 到位判定         │
                                     │  · 激光时序 / 识别确认 / 结果上报     │
                                     └──┬────────┬──────────┬───────────────┘
                /target_position ───────┘        │          │ /shelf_scan/laser
                [x,y,z,yaw] map 系               │          ▼
                        │                        │    shelf_laser_node → GPIO36 / 0x21
                        ▼                        │
              pid_control_pkg ──/target_velocity─┼──▶ uart_to_stm32 ──0x31──▶ STM32
                        ▲                        │
                   /height (STM32 0x05)          │ /shelf_scan/code
                                                 ▼
                                        shelf_code_reader_node（相机 + zbar/aruco）
```

三条"安全门"必须同时成立飞机才会动：

1. `shelf_mission_node` 启动即把 `/pillar_detection_valid` 置 `true`（latched）
   → `pid_control_pkg` 才会输出速度；`estop` 时置 `false` 即"就地悬停"。
2. `shelf_mission_node` 通过 `/mission_active`（latched）告诉 `uart_to_stm32`
   是否转发 0x31 速度帧（解决"同一上电周期跑第二趟任务不动"的历史 bug）。
3. `/target_position` 用 `transient_local` QoS，保证 PID 后启动也能收到第一条目标。

---

## 3. 核心节点解析

### 3.1 `shelf_mission_node`（任务主状态机）

**状态机**：`Idle → Arming → Running → Finished / Aborted`

| 阶段 | 触发 | 行为 |
|---|---|---|
| 标定（在 `Idle` 之前） | `auto_calibrate=true` 且连续 5 帧位姿漂移 < 2cm | 以起飞点做一次平面标定，日志 `坐标标定完成` |
| `Idle` | `auto_start_mode=sweep/target` 且延时到 | 本地兜底自动起飞（无地面站时） |
| `Arming` | 收到 `start_route` | 等 `calibrated && has_height` → `enterLeg()` |
| `Running` | — | 按航段类型分派执行 |
| `Finished` | 最后一段 land 完成 | 发 `mission` + `/mission_complete`，关 `/mission_active` |
| `Aborted` | `estop` | 关激光、置 PID 安全门 false、抢发一帧零速度、**只悬停不降落** |

**航段类型**（`RouteLeg.type`）与执行函数：

```cpp
if (leg.type == "scan")        executeScanLeg(leg);   // 逐点悬停盘点（老方案）
else if (leg.type == "vscan")  executeVScanLeg(leg);  // 纵向扫描一列（新方案）
else if (leg.type == "land")   executeLandLeg(leg);   // 到点上方 → 垂直下降
else                           executePathLeg(leg);   // takeoff / transit / return
```

- **takeoff**：只看最后一个点的高度，原地垂直爬升到 150cm（`climb_timeout` 30s 超时放行）。
  判定用 `/height` 实测值闭环，满足要求 1-1。
- **transit / return**：逐航点判到位（`position_tolerance_cm` 默认 8cm），
  超时（25s）也放行，保证不卡死。`desiredYawFor()` 会**提前一段转向**盘点朝向，省到位后的转向时间。
- **land**：两段式——先水平飞到降落点上方，再垂直下降到 8cm 判定落地。

**`executeScanLeg`（"即扫即走"设计，值得细看）**：

```
距悬停点 <15cm  → 开始识别（不等完全到位）
连续 2 帧认到同一编号 → 记编号
距悬停点 <8cm（光斑已压在码上）→ 记录坐标 + 点激光 + 立刻切下一段
激光由主循环按 0.5s 自动熄灭（不阻塞任务流程）
扫描窗口 2.5s 内没认到 → 沿板面法线自适应微调（没看到码 → 后退 10cm；
                         一帧看到多个码 → 贴近 5cm），最多 2 次、累计 25cm
```

这个"识别"和"到位"解耦 + 法线自适应微调的两点，是这套代码里对**用时（10 分）**和
**识别鲁棒性**最有效的设计。

**`executeVScanLeg`（纵向扫描，2026-09-13 起的新方案）**：

一列 = 同一悬停 x/y 上的上下两排，从 150cm 以 10cm/s 缓慢降到 70cm 边降边认：

| 阶段 | 逻辑 |
|---|---|
| `Descending` | 每周期 `z -= v*dt`；若认到"本列还没记录过"的编号 → 转 `Centering` |
| `Centering` | 先用画面上下偏差 `error_y` 把二维码推到画面中心（±40px 容差），超时 3s 按当前高度兜底记录 |
| `Holding` | 点激光 → 停 1.5s（其中点亮 0.5s 后自动灭）→ 记录实测高度并上报 → 回到 `Descending` |

归属哪一排用 **实测高度离模型行高谁近** 决定（`vscanSlotForHeight`），
降到 70cm 硬地板就结束本列，没找到的点位按 `ok:false` 补报，保证结果表填满。

**坐标标定（`FrameTransform`）**：

```
field = R(yaw_offset) · map + t            （两套坐标只差一个平面旋转 + 平移）
fromCalibration(map位姿, 场地起飞点位姿) → yaw_offset = field_yaw - map_yaw
```

起飞前把飞机**准确摆在起飞点 (75,75)、机头正对货架（场地 0°）**，
用那一刻的 map 位姿反解出 `yaw_offset` 与 `t`；之后所有航点都走
`fieldToMap()` 再发 `/target_position`。摆错 90° 会让整个场地坐标系旋转 90°。

**安全兜底**：`publishSetpoint()` 里每次发目标前都会 `clampToSafeStandoff`
（不允许进入货架板面 `standoff-15cm` 以内），一旦触发宁可原地不动也不撞货架。

**上报报文**（`gs_protocol.cpp`，与 `groundstation/PROTOCOL.md` 一一对应）：
`hello / telemetry(10Hz) / detect / target_id / target_hit / mission / ack`。
注意 `telemetry.pos.z` 在 `/height` 无数据时报 **0 而不是巡航高度**，
避免"飞控没上电却显示已悬停 150cm"。

### 3.2 `shelf_gs_link_node`（链路桥）

- 三链路可同时开：**WebSocket（推荐）/ 无线数传串口 / UDP**，都是从 `/shelf_scan/gs_out` 取行、
  把收到的整行 JSON 灌进 `/shelf_scan/gs_in`。
- 线程模型干净：ROS 回调只入队（`tx_queue_`，上限 500 丢最旧），
  独立 `writer_thread_` 做真实 IO；串口读、UDP 收、WebSocket 各一个线程。
- WebSocket 用 Boost.Beast 异步 + `asio::post` 单线程串行化 + **generation 计数丢弃旧连接回调**，
  断线按 `ws_reconnect_sec` 自动重连；另有 **30s 无成功发送的看门狗**主动断链重连
  （防"socket 还开着但谁也收不到"的假连接）。
- Boost.Beast 缺失时用 `SHELF_NO_WEBSOCKET=1` 整块关掉，退化成串口/UDP，不影响其它功能。
- 缓冲区按"整条 start_route 可能 30KB"设计（UDP 65536，串口 256KB 才判异常）。

### 3.3 `shelf_code_reader_node`（识别）

- **QR**：`zbar`（比 OpenCV 自带解码器在远距/小码上更稳）；一帧多个码时取
  **离画面中心最近**的那个（纵向扫描时才能确定正对着哪一排），
  并把"本帧一共看到几个码"作为第 5 个元素发出去（驱动自适应贴近/后退）。
- **AprilTag**：OpenCV `aruco`，`DICT_APRILTAG_36h11`，取面积最大的。
- **预处理**：灰度 → CLAHE(2.0) → 反锐化掩膜（`addWeighted(2,-1)`），抗白板强光/反光。
- 发布 `/shelf_scan/code = [编号, 置信度, x偏差px, y偏差px, 本帧码数]`；
  可选按原工程约定发 `/fine_data = [y偏差, x偏差]`（顺序与原 `drone_camera_pkg` 一致）。
- **要求 2-1 的单次抓取**四种触发：服务 `/shelf_scan/capture_target`、
  话题 `/shelf_scan/capture_trigger`、参数 `auto_capture_target:=true`、
  图片路径 `target_image_path:=`。连拍 12 帧 **投票**取多数，结果发 latched 的 `/shelf_scan/target_id`。
- 镜头用 MJPG + 640×480 + 15fps。

### 3.4 `shelf_laser_node`（激光）

四种驱动：`topic`（转发 `/laser_control` → `uart_to_stm32` 0x21 帧）/ `gpio`（本机实际用法：
**GPIO36 = 物理 18 脚，低电平点亮 `active_high=false`**）/ `serial`（写 `L1\n`）/ `none`。
安全：启动即强制关灯、`max_on_sec`（3s）看门狗强制熄灭、析构关灯。

> 注意本机激光接在**香橙派 GPIO36**，不是在飞控 PD13 上——所以要测激光用
> `/shelf_scan/laser`，`/laser_control`（走飞控）对本机无效。

### 3.5 `shelf_scan_core`（纯逻辑库，与地面站一一对应）

这是整套代码的"可信内核"：**同一套算法在地面站（Python）和机载端（C++）各实现一遍**，
用 `test/golden.json` 逐项比对，保证两端算出的坐标/顺序/航程/用时完全一致。

**场地几何 `field_geometry.cpp`**：

```
板面中心 + 法线 × 厚度/2 + 长边方向 u × 列偏移  →  二维码坐标 qr
qr + 法线 × standoff_cm                        →  悬停点 hover
yaw = atan2(-normal)                           →  机头朝向（正对板面）
```

行列 → 编号按"站在面外正对该面"的 viewer_rowmajor 约定，所以
**A 面 1 号在北端、B 面 1 号在南端**（两面观察方向相反）。
`field.json` 的 `faces[].row_delta_cm / y_offset_cm` 支持逐面微调行高和整体 y 偏移。

**航线规划 `route_planner.cpp`**：

- 避障：把每个货架矩形按 `safety_margin_cm` 膨胀，以膨胀矩形 4 个角点（再外扩 `kEdgeOffset=2cm`）
  + 起点 + 终点建**可见图**，边权=欧氏距离，Dijkstra 求最短路，最后做"共线/可直达"简化。
- 顺序优化：**最近邻**（代价含 z 权重 + 换通道惩罚 `aisle_switch_penalty_cm`）+ **2-opt**。
- 遍历顺序另有一条固定 **serpentine（弓字形）**：每面上排按 y 升序、下排反向折返
  （本次 12 点配置下是 `A3 A2 A1 A4 A5 A6 / B1 B2 B3 B6 B5 B4`）。
- `eta()` 按分段速度估算用时（航程只统计水平距离，与地面站一致）。

**协议 `gs_protocol.cpp`**：`FrameTransform` 正反变换 + 报文构造 + `parseRoute`/`parseCommand`。
下行命令支持 `start_route / land / estop(stop) / calibrate(set_origin) / goto(scan_slot) /
set_target(target_id)`。

### 3.6 自测与校验工具链（这套工程最"专业"的部分）

| 工具 | 跑在哪 | 作用 |
|---|---|---|
| `ros2 run shelf_scan_pkg shelf_selftest` | 香橙派 | 校验 C++ 实际实现的几何/规划/协议/变换/JSON（5 组、约 30 项） |
| `tools/verify_model.py` | 电脑 | 把 C++ 算法**逐行镜像成 Python**，与 golden.json 比对（不需要 ROS） |
| `tools/gen_golden.py` | 电脑 | 调地面站 `gs/field.py`、`gs/planner.py` 生成 `test/golden.json` |
| `tools/gs_stub.py` | 电脑 | 极简地面站桩（WebSocket/串口/UDP 三种传输） |
| `tools/ws_bridge.py` | 香橙派 | 装不了 Boost.Beast 时，用纯 Python 做 WebSocket↔UDP 兜底桥 |
| `tools/flight_test.py` | 香橙派 | 无地面站小航线：起飞 → 前飞 N cm → 返航降落 |

---

## 4. 与题目评分点的对应

| 题目要求 | 分值 | 代码落点 | 状态 |
|---|---|---|---|
| 1-1) 垂直起飞至 150±10cm | 5 | `executePathLeg` 的 takeoff 段 + `/height` 闭环 | ✅ |
| 1-2)① 激光在码内亮 0.5s | 24 | `executeScanLeg`/`executeVScanLeg` + `laser_hold_s=0.5` | ⚠️ 点数只有 12（见 P0-1） |
| 1-2)② 实时显示编号+坐标、LED | 12 | `detect` 报文（编号/slot/坐标/置信度/用时） | ✅（地面站侧） |
| 1-3) 稳定降落 | 5 | `return` + `land` 两段式 | ✅ |
| 1-4) 全结果 + 按编号查询 | 5 | `mapping_` + 失败点位补报 `ok:false` | ✅ |
| 1-5) 用时 | 10 | 最近邻+2-opt、vscan、即扫即走、提前转向 | ✅ |
| 2-1) 识别抽取二维码并报送 | 2 | `code_reader` 单次抓取 + `/shelf_scan/target_id` | ✅ |
| 2-2) LCD 显示规划航线图 | 8 | 地面站侧（机载端原样执行 `route`） | ✅ |
| 2-3) 飞抵目标→激光→识别→回传 | 9 | `planDirect` 直达 / `planTarget` 搜索；命中即 `jumpToReturn` | ✅ |
| 2-4) 稳定降落 | 5 | 同上 | ✅ |
| 2-5) 用时 | 10 | 已知目标直达 | ✅ |

时限保护：`time_limit_sweep_s=270 / time_limit_target_s=180` 只用于**地面站显示与 ETA**，
机载端本身不做超时中止（靠 `transit_point_timeout`、`arrive_timeout` 等分段超时保证不卡死）。

---

## 5. 关键参数速查（`config/field.json` + `config/shelf_scan.yaml`）

| 参数 | 值 | 含义 |
|---|---|---|
| `takeoff` | (75, 75), yaw **0** | 起飞点，机头**正对货架 +X**（2026-09-13 改，原来是 90） |
| `landing` | (425, 325), yaw 90 | 降落点 |
| `standoff_cm` | 70 | 悬停点离板面距离 |
| `slot_rows_cm` | [155, 115] | 两排行高（经 `row_delta_cm` 逐面修正） |
| `slot_cols_offset_cm` | [-50, 0, 50] | 三列位置 |
| `cruise_alt_cm` | 150 | 巡航/起飞高度 |
| `vscan_top/floor` | 150 / 70 | 纵向扫描高度范围，速度 10cm/s |
| `speeds` | 45 / 50 / 90 / 30 | 水平 / 垂直 / 偏航 / 降落（cm/s、°/s，仅用于 ETA） |
| `position_tolerance_cm` | 8 | 到位容差 |
| `laser_hold_sec` | 0.5 | 激光点亮时长（题目要求） |
| `scan_go_trigger_cm` / `scan_laser_arm_cm` | 15 / 8 | 开始识别 / 允许点灯的距离 |
| `scan_code_window_sec` | 2.5 | 单点识别窗口 |
| `vscan_hold_sec` / `vscan_center_tol_px` | 1.5 / 40 | 对准后停留 / 画面中心像素容差 |

---

## 6. 问题清单（按严重度）

### P0-1　场地配置只有 12 个点，题目要求 24 个

- 比赛用 [field.json](C:/Users/zxcv3/Documents/ChatGPT/无人机题目/_extract_drone_d_ws/drone_d_ws/src/shelf_scan_pkg/config/field.json)
  的 `racks` 数组里**只有一个货架 S1**（`board_x=150`，面 A/B）
  → `FieldGeometry` 只生成 **A1~A6、B1~B6 共 12 个点位**。
- 但 [field_labtest.json](C:/Users/zxcv3/Documents/ChatGPT/无人机题目/_extract_drone_d_ws/drone_d_ws/tools/field_labtest.json)
  （实验室用）里**有 S1(A/B) + S2(C/D) 两个货架 → 24 个点**。
  说明这是**配置被裁剪**，不是代码不支持。
- 证据链：`golden.json` 只有 12 个 slot（`A1..A6 B1..B6` 可验证）；
  `verify_model.py:466` 的注释直接写着"当前场地只有近端货架 A/B 两面 = 12 个点"；
  `docs/现场联调与验收清单.md` 写的是"把 12 张二维码放在桌面上"。
- **影响**：要求 1-2)① 是 24 分（每个码 1 分），只飞 12 个点最多拿一半。
- **做法**：把 S2（`board_x=350`，C 面 `normal=-X`、D 面 `normal=+X`）按实验室配置的写法补进比赛
  `field.json`，然后两端重新生成 `golden.json`、跑 `shelf_selftest`。
  代码侧不用改（`field_geometry.cpp` 本来就是遍历 `racks` 数组的）。

### P0-2　行高配置与题目反推值差 10cm，需现场确认

- 比赛 `field.json`：`slot_rows_cm = [155, 115]`，A 面 `row_delta=[-5,-5]` → **150 / 110**；
  B 面 `row_delta=[+5,-5]` → **160 / 110**。
- 题目图 2 反推（见 `D题_解析与实现思路.md` 3.3）：上排中心 **140**、下排中心 **100**，
  上排码幅面 130.5~149.5cm。
- 由于**激光笔水平安装 ⇒ 光斑高度 = 出光口高度**，若按 150cm 飞而码在 140cm，
  光斑正好落在上排码上沿**之外 0.5cm**。
- B 面上排 160 比 A 面上排 150 高 10cm，两个面不一致，看起来**更像填写笔误**。
- **做法**：用卷尺测实际二维码中心高度与激光光斑落点高度，统一写进 `field.json`；
  如果光斑恒定高于码心，用 `shelf_scan.yaml` 的 `altitude_trim_cm`（填负值）统一补偿。

### P1-1　本地兜底航线不含 vscan，与 README 描述的现场方案不一致

- README 第 6 节描述现场跑的是"长城状纵向扫描"（`vscan` 航段），
  但 [route_planner.cpp](C:/Users/zxcv3/Documents/ChatGPT/无人机题目/_extract_drone_d_ws/drone_d_ws/src/shelf_scan_pkg/src/route_planner.cpp)
  里**没有任何构造 `vscan` 航段的代码**，`planSweep()` 走的是 `planPointwise()`（逐点悬停）。
- 也就是说：**`vscan` 只能由地面站下发**；地面站不在线时，机载端本地兜底跑的是老方案。
- 连带问题：`golden.json` 里的 `sample_route` 是逐点版（27 段，`takeoff transit scan ... return land`），
  `shelf_selftest` 第 [3] 组会拿它和本地 `planSweep()` 比"航段类型序列一致"。
  如果地面站改成下发 `vscan` 后重新生成 golden，而本地 planner 没同步改，**自测会失败**。
- **做法**：二选一——① 在 `route_planner.cpp` 里补 `planVScan()` 并同步地面站；
  ② 明确"vscan 只由地面站生成"，同时把 golden 固定成逐点版并更新 README 描述。

### P1-2　`tools/flight_test.py` 的"前飞方向"与新版起飞朝向矛盾

- 脚本写死 `target = (x0, y0 + forward_cm)`，即"前飞 = 场地 +Y"，并注释说这等于标定时机头朝向。
- 但比赛 `field.json` 的 `takeoff.yaw` 已在 2026-09-13 改成 **0（机头朝 +X 正对货架）**。
  用比赛场地配置跑这个脚本，飞机会**侧着飞**（场地 +Y 是它的左手边）。
- `field_labtest.json` 仍是 `yaw=90`，所以实验室自测不受影响。
- **做法**：把脚本改成按 `takeoff.yaw` 方向前飞，或明确只在实验室配置下使用。

### P2-1　`golden.json` 是 12 点版本，且与 README 描述并存

- `golden.json` = 12 点 + 逐点航线；README = 12 点 + vscan 流程。
  两者只要有一处改动（补 S2、改 vscan、改行高），都必须
  `python tools/gen_golden.py` + `python tools/verify_model.py` + `ros2 run shelf_scan_pkg shelf_selftest`
  三条命令重跑一遍。

### P2-2　`vscan` 航段强依赖地面站下发两个点

- `executeVScanLeg` 用 `leg.points.front()` 当起点高度、`leg.points.back().z` 当硬地板。
  如果地面站只发了 `hover` 没发第二个点，`floor_z == top.z`，**整段会立刻结束**。
- 建议在 `gs_protocol.cpp::parseRoute` 里对 `vscan` 段做"至少 2 个点"的校验并报错，
  而不是静默跑完（当前只有 `scan` 段有非空校验）。

### P2-3　`routeToJson()` 丢弃 vscan 字段

- `parseRoute` 能读 `face / column / slots / rows`，但 `routeToJson` 只写
  `type/slot/order/yaw/laser_s/points` → **vscan 段序列化后会退化**。
  当前主流程没有调用 `routeToJson`（只在头文件里声明），属于潜在坑而非现网问题。

### P2-4　混排朝向的货架会算错"通道"

- `field_geometry.cpp` 里 `band_axis_x_` 是**单个成员变量**，在遍历 `racks` 时被反复覆盖，
  最终值等于最后一个货架的朝向。所有货架都是 `axis: "y"` 时没问题，
  一旦配置里同时出现 `axis: "y"` 与 `axis: "x"`，`aisleOf()` 和 `clampToSafeStandoff()` 会算错。

### P3　其它值得知道的行为/取舍

- **急停只是就地悬停**，不会自动降落；要落地必须再发 `land`（代码里 `Aborted → Arming` 的返航路径是故意留的）。
- **`transit_point_timeout_sec=25s` / `arrive_timeout_sec=8s`**：到位失败会"带着偏差继续走"，
  这是为了不超 270s 的保时间设计，但会牺牲精度——现场定位差时要盯这两条日志。
- **`standoff_cm=70`**，而 `docs/现场联调与验收清单.md` 仍写"板面前 55cm"，文档与配置不同步。
- **`rack_height_cm=100`** 在比赛 `field.json` 里（默认 190、题目 190cm），
  该字段目前只做记录不参与几何计算，但容易误导，建议改成 190。
- **地面站代码不在本压缩包内**（`tools/*.py` 都依赖 `../groundstation/gs/`），
  所以"两端一致性"目前只能靠已有的 `golden.json` 间接确认，无法核对地面站实现本身。
- `run.sh` 里 `ROS_DOMAIN_ID` 默认 2，与 `drone2=2` 的约定一致。
- 代码里大量"踩坑注释"（QoS、params_file 撞名、0x31 转发门、坐标系旋转）说明这套工程
  是**真上过飞机飞出来的**，不是纸面代码——这些注释本身就是很有价值的调试资产。

---

## 7. 建议的下一步（按顺序）

1. **确认场地到底是 12 个码还是 24 个码**（决定 P0-1 怎么改）；
2. 卷尺实测二维码行高 + 激光光斑落点高度，定 `slot_rows_cm` 与 `altitude_trim_cm`（P0-2）；
3. 决定 vscan 由谁生成，并让 `route_planner.cpp` / 地面站 / golden 三者一致（P1-1、P2-1）；
4. 改完重跑三条校验：`gen_golden.py` → `verify_model.py` → `shelf_selftest`；
5. 再按 `docs/现场联调与验收清单.md` 做链路、标定、激光、识别的逐项联调。
