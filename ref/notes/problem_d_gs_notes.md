# D 题地面站（`Desktop/D题地面站`）代码解析

> 解析对象：`C:\Users\zxcv3\Desktop\D题地面站`（Python 3.13 + PySide6 上位机）。
> 结论分两类标注：**[实测]** = 本次实际运行验证过；**[静态]** = 只读代码得出的判断，未运行验证。

---

## 0. 一句话结论

这是一套**"地面站 + 模拟机载端 + 识别链路"三件套**：地面站本身只做四件事——
**收帧 → 维护 24 格库存 → 按库存定位/规划航线 → 显示**；
真机能力（飞控、相机、二维码识别）全部被抽象成可替换的接口，
所以无真机时也能走完整链路。结构清晰、职责边界干净，
但**当前这台电脑缺少运行依赖，GUI 起不来**，另有 3 个会在真机/高速场景下暴露的缺陷（见第 6 节）。

---

## 1. 项目定位与运行入口

| 入口 | 作用 | 依赖 |
|---|---|---|
| `main.py` | 启动 PySide6 地面站 GUI，默认停在遍历盘点页，不自动跑模拟 | PySide6 |
| `test_smoke.py` | 无头冒烟测试：协议编解码 + 内置模拟跑完整个遍历流程 | PySide6 |
| `simulator.py --udp` | 独立进程，走**真实 UDP** 发帧给地面站 | 无 Qt 依赖（CLI 模式） |
| `drone_link_test.py` | "机载端雏形"，周期发 STATUS、收 START 就跑流程，可搬去 RDK X5 | 仅 stdlib + `protocol.py` + `layout.py` |
| `vm_qr_link.py` | 虚拟机侧：摄像头 → OpenCV 识别二维码 → UDP 发给地面站 | opencv + numpy |
| `host_cam_stream.py` | 宿主机把摄像头转成 MJPEG 流，绕过 VMware USB 直通问题 | opencv |
| `make_qr_sheet.py` | 生成 A4 二维码打印表 + `num_cell.json` 对照表 | qrcode + PIL |

**这台机器的实际依赖状况 [实测]**：

```
python 3.13 ✓ | numpy ✓ | opencv-python 4.13 ✓ | pillow ✓
PySide6 ✗ | qrcode ✗ | pyserial ✗
```

→ 所以 `main.py` / `test_smoke.py` **现在无法运行**（`ModuleNotFoundError: PySide6`）；
`drone_link_test.py --selfcheck` 可正常运行并通过。

---

## 2. 模块地图（分层）

```
        ┌──────────── UI 层 (PySide6) ────────────┐
        │ ui/main_window.py   主窗口/链路/帧路由/日志 │
        │ ui/page_control.py  遍历盘点页(24格+查询+LED)│
        │ ui/page_targeted.py 定点盘点页(目标/阶段/二维码)│
        │ ui/page_route.py    航线图控件(仓库俯视+折线) │
        └───────────────┬─────────────────────────┘
                        │ Qt Signal/Slot
        ┌───────────────▼─────────────────────────┐
        │ models.py       Inventory / DroneState   │  ← 唯一状态源
        │ led_controller.py  虚拟 LED 抽象          │
        └───────────────┬─────────────────────────┘
                        │
        ┌───────────────▼──────────┐   ┌──────────────────────┐
        │ links.py  链路抽象         │   │ simulator.py 内置模拟 │
        │  UdpLink / SerialLink     │◄──┤  SimulatorCore(时间线)│
        └───────────────┬──────────┘   └──────────────────────┘
                        │
        ┌───────────────▼──────────┐   ┌──────────────────────┐
        │ protocol.py  帧协议 v0     │   │ layout.py 几何真相源   │
        │  FrameParser / build_*    │   │  货架坐标/航线规划     │
        └──────────────────────────┘   └──────────────────────┘
```

设计上值得肯定的三点：

1. **`layout.py` 是唯一几何真相源** —— 模拟器、航线图、机载端全部 import 它，
   改图纸尺寸只需要改一个文件。
2. **`protocol.py` 无 Qt 依赖** —— 机载端可以直接拷走 `protocol.py` + `layout.py`。
3. **`inject_frame()` 统一收帧入口** —— 内置模拟和真实 UDP 走同一条解析路径，
   模拟能跑通基本等于链路能跑通。

---

## 3. 帧协议 v0（`protocol.py`）

```
[0xAA][0x55][type:u8][len:u8][payload…][sum:u8]     小端；sum = (type+len+Σpayload) & 0xFF
```

| type | 名称 | 方向 | payload | 构造/解析 |
|---|---|---|---|---|
| 0x01 | EVENT 盘点事件 | 上行 | 编号u8, 面u8(0=A..3=D), 位u8, 时间戳u32(ms) | `build_event` / `parse_event` |
| 0x02 | STATUS 状态 | 上行 | 状态u8(0就绪1起飞2飞行3降落4落地), 高度u16(cm), 电量u8 | `build_status` / `parse_status` |
| 0x03 | ROUTE 航线 | 上行 | 点数u8 + 每点{x i16, y i16} | `build_route` / `parse_route` |
| 0x04 | REPORT 识别报送 | 上行 | 编号u8 | `build_report` / `parse_report` |
| 0x10 | QUERY 查询 | 下行 | 编号u8 | **只有 parse，没有 build** |
| 0x11 | START 启动 | 下行 | 模式u8(0遍历/1定点) | 直接 `bytes([code])` |

`FrameParser` 是**增量解析器**：任意字节流喂进去，自动丢头前杂散字节、
缓冲半截帧、校验和计数；`test_smoke.py` 验证过"混入 `b"garbage"` 后仍能正确解出 4 帧"。

**载荷上限 [实测]**：`MAX_PAYLOAD = 64`，而 `build_route` 按 32 点裁剪。
整条遍历航线 30 点 = **121 字节，`pack_frame` 直接抛 `ValueError`**：

```
遍历航点数: 30   前3点: [(75,325), (125.0,266.7), (125.0,200.0)]   后2点: [(375.0,133.3), (425,75)]
build_route 字节数: 121   MAX_PAYLOAD: 64
打包结果: 失败 -> payload 超长: 121 > 64
定点航线(C2): [(75,325), (308,325), (308,200.0), (308,75), (425,75)]  字节: 21  可打包: True
```

即：**大航线只能在内置模拟里"发"（走内存信号，不打包），一旦接真 UDP 就发不出去**。
定点航线（5 点 / 21B）没问题——刚好符合题目"要求 2 需要显示规划航线图"的需求。

---

## 4. 数据流（三条路径）

### 4.1 内置模拟（默认，无真机）

```
SimulatorCore(时间线) --frame_out--> MainWindow._on_frame --+--> Inventory.add_event
                                                            +--> DroneState.update
                                                            +--> LED.blink(1000)
                                                            +--> 日志
```

`simulator.build_simulation()` 生成的是**绝对时刻表** `[(delay_ms, type, payload)]`：
就绪→起飞→巡航(顶排 140cm)→ROUTE→24 个 EVENT(按蛇形顺序)→降落(3/4)。
电量随进度 100%→65% 线性下降，换排时补发 STATUS。

### 4.2 真实 UDP

```
UdpLink._recv_loop(线程) -> FrameParser.feed -> frame_received 信号 -> 主线程 _on_frame
MainWindow -> link.send(START/…) -> pack_frame -> sendto(机载IP:端口)
```

注意：地面站 `bind(0.0.0.0:8888)` 收帧，同时把下行发到 `(机载IP, 8888)`，
和 `drone_link_test.py` 的 `rx_port` 约定对齐，属于**收发同端口**设计。

### 4.3 定点语义（这是最容易被误解、也是写得最讲究的一段）

题目要求 2 只抽 **1 张**二维码，所以"扫码"是**确认到达**，不是"决定盘哪个"。
代码严格按这个语义分流（`page_targeted.target_num()`）：

| 输入框 | 目标来源 | REPORT 帧的作用 |
|---|---|---|
| 填了编号 | **以输入框为准**，立刻按遍历库存定位 + 规划航线 | 只显示，**不改航线** |
| 留空 | 随机模式，跟随 REPORT 上报的编号 | **由它决定**目标并规划 |

另有兜底：定点模式下收到 EVENT 会**再校准一次航线**，
用于"遍历漏了这格、上报前无坐标画不出线"的情况（`main_window._on_frame`）。

---

## 5. 关键几何（`layout.py`）[实测]

坐标系：**厘米，原点 = 仓库左上角**，X 向右 0~500，Y 向下 0~400
（题图 1 以左下角为原点、Y 向上，代码里做了 `scene_y = 400 - 题图_y` 翻转）。

| 要素 | 数值 |
|---|---|
| 仓库 | 500 × 400 cm |
| 起飞区（黑方块中心） | (75, 325) |
| 降落区（黑圆圆心） | (425, 75) |
| 货架 1 / 货架 2 杆线 | x = 150 / x = 350，长 y ∈ [100, 300] |
| 面朝向 | A 朝西、B 朝东（货架1）；C 朝西、D 朝东（货架2） |
| 二维码平面距杆线 | 25 cm（`_FACE_OFFSET`） |
| 上/下排二维码高度 | 140 cm / 100 cm |

实测输出：

```
识别走廊 x: {'A': 108, 'B': 192, 'C': 308, 'D': 392}
面坐标 A1: (125.0, 133.3)    D6: (375.0, 266.7)
```

**定点航线（不穿货架）**：起飞 → 沿南侧通道(y=325)走到目标面走廊 → 沿走廊北上到目标列中心
→ 继续北上出货架区(y=75) → 降落。走廊 = 二维码平面再外移 `_APPROACH = 17cm`（相机识别距离）。

核对过：4 个面的走廊 x（108/192/308/392）**都在货架杆线之外或两杆之间**，
南北通道 y=325 / y=75 分别 >300 / <100，全部绕开了 `x∈{150,350} 且 y∈[100,300]` 的杆体区域，
几何上**无穿模**。

---

## 6. 问题清单

### P1 — 真机联调会直接卡住的

**① 已在定点页时，再点"开始定点盘点"不会下发 START**（`main_window.py:_on_targeted_start`）

```python
if self.page_ctrl.mode != MODE_TARGETED:
    self.page_ctrl._set_mode(MODE_TARGETED)   # 只有切页才走这里 → 才会发 START
```

`_set_mode` 在模式相同时直接 `return`，不 emit `mode_changed`，
于是 `_on_work_mode` 里的 `link.send(TYPE_START, ...)` 被跳过。
结果：**第二次定点盘点时真机会一直待机**（内置模拟因为不依赖 START 所以看不出来）。
建议把 START 下发从 `_set_mode` 回调里拆出来，在 `_on_targeted_start` 里显式发一次。

**② 串口链路空转吃满一个核**（`links.py:SerialLink._recv_loop`）

```python
while self._running and self._ser:
    n = self._ser.in_waiting
    if n:
        data = self._ser.read(n)
        self._consume(data)
    # ← 没有 else: time.sleep(...)
```

`in_waiting` 无数据时立即返回 0，循环变成忙等。建议 `read()` 阻塞读或加 `sleep(0.005)`。

### P2 — 影响判分的

**③ LED 可能亮不满 1 秒**（`led_controller.py:blink`）

```python
self._timers = [QTimer.singleShot(duration_ms, lambda: self.set_on(False))]
```

`QTimer.singleShot` 是静态方法，**返回 None** —— 所以 `self._timers` 里装的是 `[None]`，
`tm.stop()` 每次都抛 `AttributeError` 被 `except` 吞掉，**旧定时器根本没被取消**。
后果：若两次 EVENT 间隔 <1s，第一个定时器到点会把第二次的亮灯提前掐灭，
实际亮灯时间不足 1 秒，与题目"LED 亮 1 秒左右（12 分项）"不符。
当前模拟默认间隔 1800ms > 1000ms，所以**演示时看不出来**；
但"遍历用时越少越好"这条 10 分项正鼓励把间隔压到 1s 以内，风险会暴露。
修法：用成员 `QTimer`，每次 `blink()` 调 `timer.start(duration_ms)` 重置。

**④ 整页二维码图多码识别会漏码 [实测]**

```
整页 qr_sheet_p1.png (3507×2481) 解出 5 个: ['11','5','6','8','9']   ← 缺 '7'
裁出 '7' 那一格(700×700)         解出: ['7']
整页缩到 1/4 后再解              解出 6 个: ['11','5','6','7','8','9']
```

即**码本身没问题**，是 `decode_qr` 的 `_variants()` 只在 `max(h,w) ≤ 1280` 时做放大，
对超大整页图没有降采样档，OpenCV 的 `detectAndDecodeMulti` 在这种尺度下漏检。
实际使用是"一帧一个码"，影响有限；但若要用整页图做自检/校准，建议补一个降采样变体。

### P3 — 工程整洁度

**⑤ `_verify_targeted.py` 里写死了别人的路径**

```python
sys.path.insert(0, r"C:\Users\金洲毅\Desktop\D题地面站")
```

本机是 `zxcv3`，这行是开发机残留（侥幸因为 `sys.path[0]` 本来就是脚本目录才没报错）。

**⑥ 三个页面是死代码**：`ui/page_live.py`、`ui/page_overview.py`、`ui/page_query.py`
功能已被 `page_control.py` 完全覆盖，`main_window.py` 不再实例化它们。
`ui/__init__.py` 的注释还写着"四个页面"，容易误导。

**⑦ 其他小项**：协议有 `TYPE_QUERY` 但没有 `build_query()`；盘点结果**不落盘**（重启即丢，不利于复现与写报告）；
`simulator.py` 文档提到 `python main.py --sim`，但 `main.py` 不解析参数，该用法不存在。

---

## 7. 题目评分点覆盖对照

| 评分点 | 分值 | 地面站是否覆盖 | 说明 |
|---|---|---|---|
| 1-② 实时显示编号 + 坐标（LCD≥3.5″） | 12 | ✅ | `page_control` 24 格总览实时刷新，每格显示 `A1` + 编号 |
| 1-② 每盘到一个 LED 亮 1s | 12 | ⚠️ | `VirtualLed` + 托盘指示灯已实现，但见问题 ③ 的计时缺陷 |
| 1-④ 显示全部结果 + 编号查询坐标 | 5 | ✅ | 总览网格 + 查询框（含未盘点/越界提示、清空/复位） |
| 2-① 识别 1 张抽取码并报送编号 | 2 | ✅ | 状态行"识别报送"实时显示 REPORT 编号 |
| 2-② LCD 显示规划的定点航线图 | 8 | ✅ | `page_route` 画仓库/双货架/起降点/规划折线，目标高亮 |
| 2-③ 显示编号 + 坐标 + LED | 9 | ✅ | 定点页"识别传回: 编号·坐标·高度·时间戳" |
| 要求 3 其他（创新） | 5 | — | 可发挥：结果导出、链路质量统计、多机位识别 |

地面站侧**未覆盖、也不该由地面站覆盖**的：激光笔点亮 0.5s、起飞高度 150±10cm、
稳定降落、用时指标——这些在飞行器侧（`drone_d_ws` / RDK X5）。

---

## 8. 建议的修复顺序

1. **补依赖**：`pip install PySide6 qrcode pyserial`，然后跑 `test_smoke.py` 验证 GUI 链路。
2. **修 ②（START 下发）**：直接决定要求 2 能否在真机上跑起来，改动最小、收益最大。
3. **修 ③（LED 计时）**：直接关联 12 分项，且改动只有几行。
4. **修 ①（串口忙等）**：蓝牙链路一旦接上就吃满 CPU。
5. **决策 ④（MAX_PAYLOAD）**：若真机需要接收整条遍历航线，扩到 128 或改多帧拼接；
   若机载端自己规划（现在 `drone_link_test.py` 就是自己规划），维持现状并在文档里明确。
6. 清理 ⑤⑥⑦：删死代码、去硬编码路径、补 `build_query()`、加结果导出。
