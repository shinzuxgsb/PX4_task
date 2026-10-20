# 飞控代码解析（ANO_LX / STM32F407 转接板固件）

> 解析对象：`C:\Users\zxcv3\Desktop\飞控代码\`（1106 个文件，约 129 MB）
> 最后编译时间：**2026-09-25 15:03**（`ProjectSTM32F407/build/ANO_LX.axf/.hex`、`ANO-LX.bin` 59920 字节）

---

## 0. 一句话结论

这不是"飞控稳定器"本体，而是**凌霄（ANO）飞控系统的 F407 转接/中控板固件**：
它一边用 ANO DT V7 协议跟 **IMU 核心板**（真正的姿态/位置控制器）通信，
一边用自定义的 `0xAA 0xFF …` 帧协议跟 **机载计算机（香橙派 / RDK）**、
**TJC 串口屏**、**光流模块**通信，自己主要负责"转发 + 门控 + 显示 + 少量逻辑"。

**最重要的结论**：这份固件在 2026-09-13 之后被改成了"**由机载计算机用 `0x66` 命令控制起飞/降落**"，
而我手上的机载端代码（`drone_d_ws`，9-17 版）**只发 1 字节的 `0x66`，且根本不发起飞命令** ——
两边对不上，见第 8 节 P0-1。

---

## 1. 工程结构与构建

```
飞控代码/
├── FcSrc/                    应用层（本工程核心，13 个模块）
│   ├── main.c / Ano_Scheduler.c        主循环 + 软调度器
│   ├── ANO_LX.c/.h                    1ms 主任务、遥控解析、rt_tar、电机输出
│   ├── ANO_DT_LX.c/.h                 与 IMU 核心板的 ANO DT V7 协议
│   ├── AnoDTRasp.c/.h                 与机载计算机（香橙派/RDK）的协议  ★
│   ├── User_Task.c/.h                 一键起飞/降落状态机  ★
│   ├── LX_FC_Fun.c/.h                 给核心板下发功能命令（解锁/模式/起飞/降落/返航/校准）
│   ├── LX_FC_State.c/.h               摇杆解锁/上锁、摇杆校准触发
│   ├── LX_FC_EXT_Sensor.c/.h          外部速度源（SLAM / 光流）→ 核心板
│   ├── TJC_lcd.c/.h                   陶晶驰串口屏 UI
│   ├── Bluetooth.c/.h                 蓝牙遥控（已注释掉，死代码）
│   ├── AnoRos_DT.c/.h                 ★ 未加入 Keil 工程，纯死代码
│   └── Others.c/.h / SysConfig.h      版本号、宏、类型定义
├── DriversBsp/                板级驱动：BSP 初始化、光流、GPS、数学工具
├── DriversMcu/                三个平台（STM32F407 / MSP432 / TM4C123）
│   └── STM32F407/Drivers/     本机实际使用：UART/定时器/PWM/RC/ADC/LED
├── ProjectSTM32F407/          Keil MDK 工程（本机在用的那个）
├── ProjectMSP432/ ProjectTM4C123/   另外两个平台的工程（历史遗留）
├── FcSrc_backup_20260913/     09-13 的整份 FcSrc 快照  ★
└── boot0_uart_test.py         用 STM32 ROM bootloader(AN3155) 握手，
                               判断芯片是"RDP2 锁死了"还是"SWD 引脚坏了"
```

编译进固件的文件（来自 `ANO_LX_STM32F407.uvprojx`）：
`main.c, Ano_Scheduler.c, User_Task.c, ANO_DT_LX.c, ANO_LX.c, LX_FC_EXT_Sensor.c,
LX_FC_Fun.c, LX_FC_State.c, Bluetooth.c, TJC_lcd.c, AnoDTRasp.c, Drv_BSP.c,
Ano_Math.c, Drv_AnoOf.c, Drv_UbloxGPS.c` + `DriversMcu/STM32F407/Drivers` 下的
`stm32f4xx_it.c / Drv_Sys / Drv_adc / Drv_led / Drv_PwmOut / Drv_RcIn / Drv_Timer / Drv_Uart`。
**`AnoRos_DT.c` 不在工程里**（另一套 ROS 协议实现，从没被编译过）。

---

## 2. 硬件资源与串口拓扑

`Drv_BSP.c::All_Init()` 的初始化顺序与真实引脚（`Drv_Uart.c`）：

| 串口 | 引脚 | 波特率 | 接什么 | 收字节回调 |
|---|---|---|---|---|
| **USART3** | **PB10(TX)/PB11(RX)** | **921600** | **机载计算机（香橙派/RDK）** | `AnoDTRaspRecvOneByte` |
| **UART5** | **PC12(TX)/PD2(RX)** | **500000** | **IMU 核心板（真正的飞控）** | `ANO_DT_LX_Data_Receive_Prepare` |
| UART4 | PA0(TX)/PA1(RX) | 500000 | 光流模块 | `AnoOF_GetOneByte` |
| USART1 | PA9/PA10 | 115200 | TJC 串口屏 | `MY_TJC_GetOneByte` |
| USART2 | PD5/PD6 | 115200 | 蓝牙（功能已注释） | `Bluetooth_GetOneByte` |
| TIM3 / USART6 | — | — | PPM / SBUS 遥控输入 | `PPM_IRQH` / `Sbus_IRQH` |

> 这解释了机载端文档里那句"香橙派 `/dev/ttyS6` ↔ 飞控 UART3"——
> 香橙派侧的 ttyS6 就是这里的 PB10/PB11。

中断优先级（`McuConfig.h`，NVIC 分组 3，抢占优先级数字越小越高）：

| 中断 | 抢占优先级 | 说明 |
|---|---|---|
| UART5（IMU 核心板） | **1** | 最高，保证核心板数据不被拖 |
| USART3（机载计算机） | 2 | 次高 |
| TIM3（PPM 捕获） | 2 | |
| UART4（光流）/ USART1（屏） | 3 | |
| USART2（蓝牙） | 4 | |
| TIM7（1ms 主节拍） | **7** | 最低，可被上述串口打断 |

其它外设：TIM5/TIM1/TIM8 输出 8 路电机 PWM（`PWM_FRE_HZ=400`，1000~2000µs，
`ESC_CALI=0` 电调校准关闭）；TIM4_CH1(PD12) 输出舵机 PWM（600/1000/1400/1800µs 四档）；
ADC 采集电池电压；LED 驱动。

---

## 3. 运行时架构：一个 1ms 中断 + 一个软调度器

```
main()
 ├─ All_Init()                      各外设初始化（中间夹 ~1.1s 硬延时）
 └─ while(1) Scheduler_Run()        ← 软调度器（按毫秒时间戳轮询）
       ├─ 50Hz  UserTask_OneKeyCmd()      一键起飞/降落状态机
       └─ 20Hz  TJC_Send() / Duoji_top_set()   串口屏刷新、舵机输出

TIM7 中断（1ms；84MHz/84 预分频 ×1000 周期）
 └─ ANO_LX_Task()
      ├─ 10ms：DrvRcInputTask / RC_Data_Task / LX_FC_State_Task（摇杆解锁上锁、校准）
      │         AnoOF_Check_State
      │         └─ 100ms：Bat_Voltage_Data_Handle
      ├─  1ms：DrvUartDataCheck()            5 路串口缓冲喂给各自状态机
      │         GPS_Data_Prepare_Task
      │         LX_FC_EXT_Sensor_Task()      SLAM/光流速度 → 0x33 帧给核心板
      │         ANO_LX_Data_Exchange_Task()  与核心板的收发
      │         AnoDTRaspRunTask1Ms()        ★ 与机载端的收发 + 看门狗 + 0x66 处理
      │         ESC_Output()                 电机 PWM 输出
      │         LED_1ms_DRV()
```

**结构特点**：几乎所有实时工作都塞在 1ms 中断里（含串口解析、命令重发、电机 PWM 输出），
主循环只做两个低频任务。好处是时序确定；风险是中断偏长（见第 9 节）。

---

## 4. 三条通信链路

### 4.1 与 IMU 核心板（UART5，ANO DT V7 协议）

帧格式：`0xAA | 地址(0x61/0xFF) | 帧ID | 长度(<250) | 数据… | SC1 | SC2`
（SC1 = 逐字节累加，SC2 = SC1 再累加），双向校验（`ANO_DT_LX.c`）。

**F407 → 核心板**（只发 3 种）：

| 帧 | 内容 |
|---|---|
| `0xE0` CMD 帧 | CID + 10 字节 CMD（解锁/上锁/模式/起飞带高度/降落/返航/校准…） |
| `0x00` CK 回应 | 对核心板 E0 的应答校验（`CK_Back`） |
| `0xE2` 参数回应 | 参数读取回包 |

可靠性：`CK_Back_Check()` 每 50ms 重发一次、最多 5 次，超时放弃；
`dt.wait_ck` 门闩保证同一时刻只有一条命令在飞。

**核心板 → F407**（`ANO_DT_LX_Data_Receive_Anl` 解析）：
`0x01` IMU 原始 / `0x02` 磁+气压 / `0x03` 姿态角 / `0x04` 四元数 / `0x05` 高度 /
`0x06` **飞行状态（模式、解锁状态、最后执行的命令）** / `0x07` 速度 /
`0x0F` RGB 灯 / `0x20` 8 路 PWM 回传 / `0x30` GPS / `0xE1` 参数读 / `0xE2` 参数写。

> `0x06` 里的 `fc_mode_sta / unlock_sta / cmd_fun` 原来"解析后丢掉、没人读"，
> 现在被延长到给机载端的 `0xF1` 帧里（见 4.2），专为排查"电机不转"而加。

### 4.2 与机载计算机（UART3 @921600，`AnoDTRasp.c`）★

沿用同一套 `0xAA 0xFF …` 帧格式（双累加校验），但 ID 是另一套：

**下行：机载计算机 → F407**

| ID | 数据 | F407 的处理 |
|---|---|---|
| `0x31` | vx,vy,vz,vyaw（各 s16；cm/s、°/s） | `Set_m_speed()` 限幅 ±60cm/s / ±30°/s，写 `m_speed_*`；喂命令看门狗 |
| `0x32` | vx,vy,vz（s16）SLAM 速度 | `Set_m_speed_now()` → 打包成 `0x33` 帧发给核心板当外部速度源；喂反馈看门狗 |
| `0x33` | 定位（3×s16） | 存 `rosData.loc[]`，只用于**串口屏**显示 tf_x/tf_y |
| `0x63` | 航线记录（tf,x,y,z,w） | 写入屏的 page4/page5 表格 |
| `0x66` | **任务控制**：`sub` + 高度 | `0x01`=起飞到指定高度、`0x02`=降落、`0x03`=返航、`0x04`=上锁 |
| `0xA2` | 上位机设置 | `Con_flag` / `Ano_Duoji` / `qr_num`（扫码编号） |

**上行：F407 → 机载计算机**

| ID | 内容 | 周期 |
|---|---|---|
| `0x05` | 高度（s16 cm） | 5ms |
| `0xF1` | 用户数据 **16 字节** | 10ms |
| `0x40` | 遥控 10 通道（µs） | 20ms |
| `0x0D` | 电池电压/电流 | 100ms |
| `0x01` | IMU 原始 20 字节 | 默认关闭（代码里注释掉了） |
| `0x07` | 速度 | 默认关闭 |
| `0x02` | 磁/气压 | 默认关闭 |
| `0x62` | 航线回读（屏上按钮触发） | 事件 |
| `0x30` | GPS | 事件（`fre_ms=0`，靠触发） |

`0xF1` 的 16 字节布局（这是排查问题最有用的一帧）：

```
[0] mission_end_sub   [1] mission_step   [2] qr_num        [3..4] 最近一帧 dataLen
[5..6] tacc[0]        [7..8] tacc[1]
[9]  fc_mode_sta      [10] unlock_sta     [11] fc_mode_cmd  [12] unlock_cmd
[13] cmd_fun.CID      [14] cmd_fun.CMD_0  [15] cmd_fun.CMD_1
```

**新增的两个看门狗**（1ms 任务里计数）：

```c
#define CMD_TIMEOUT_MS 200   // 200ms 没收到 0x31 → Set_m_speed(0,0,0,0)
#define FBK_TIMEOUT_MS 200   // 200ms 没收到 0x32 → Set_m_speed_now_Invalid()
```

- 速度看门狗**故意不动 `takeoff_if`**：一次 200ms 通信抖动不应该结束任务；
- 反馈看门狗把速度源标成无效（核心板侧填 `0x8000`），防止核心板继续积分陈旧速度
  （注释指出 `speed_cnt_t++` 是触发 `0x33` 帧的唯一条件，漏了它核心板会一直用老速度）。

### 4.3 与串口屏 / 光流 / 蓝牙

- **TJC 串口屏**（UART1）：发 `txt="…"` + `0xFF 0xFF 0xFF`，显示电池、
  `tf_x/tf_y/tf_z`（来自机载端的 `0x33` 定位 + 核心板高度）、实时/目标速度、扫码编号、航线表格；
  接收屏上按钮（速度清零、切换速度源 lidar/光流、录航线、舵机四档、
  `takeoff_ready` 标志 = 屏上"起飞准备"手势 `0x23 0x04 0x07 0x31`）。
- **光流**（UART4，`Drv_AnoOf.c`）：`0xAA 0x61 …` 帧，解析出 `of1_dx/of1_dy`，
  与 SLAM 速度二选一（`source_speed`：0 = 雷达/SLAM，1 = 光流，屏上按钮切换）。
- **蓝牙**（UART2）：解析九宫格方向指令写 `b_speed_x/y`，但真正下发速度的
  `Bluetooth_speed_check()` **整段被注释掉了**，等于死代码。

---

## 5. 控制通路：谁真正决定飞机怎么动

```
遥控器 ──SBUS/PPM──▶ rc_in.rc_ch[] ──┐
                                     ├─▶ rt_tar（rol/pit/thr/yaw_dps/vel_x/y/z）
机载端 ──0x31──▶ Set_m_speed() ──────┘        │
                  (±60cm/s, ±30°/s)           │ 每 10ms 随 0x41 帧发给核心板
                                               ▼
                                    核心板执行 ──▶ 电机
```

`ANO_LX.c::RC_Data_Task()` 里的核心分支（**全部速度门控都在这里**）：

```c
if (rc_in.fail_safe == 0)          // 遥控信号正常
{
    ...摇杆 → rt_tar.rol/pit/thr/yaw_dps...

    if (takeoff_if)                // ★ 门 1
    {
        rt_tar.vel_x  = flown_flag ? m_speed_x : 0;   // ★ 门 2（仅水平）
        rt_tar.vel_y  = flown_flag ? m_speed_y : 0;
        rt_tar.vel_z  = m_speed_z;                    // 垂直不受门 2 限制
        rt_tar.yaw_dps= m_speed_yaw;
    }
    else
    {
        rt_tar.vel_x = vel_y = vel_z = yaw_dps = 0;   // 机载端发什么都没用
    }
    dt.fun[0x41].WTS = 1;          // 周期性把 rt_tar 发给核心板
}
else                               // 遥控失控
{
    if (已解锁) { 切模式 3 → 一键返航 }
    rt_tar = {rol=0, pit=0, thr=350, yaw=0, vel=0};   // ★ 速度被清零
}
```

三条硬性事实：

1. 没有 `takeoff_if=1`，机载端所有速度指令等于 0 → 电机不转
   （这是历次"发了速度飞机不动"的根因）。
2. `flown_flag` 是更细的一道门：起飞命令被接受前**水平速度锁死**，防止地面上一帧野的
   `0x31` 把飞机拖着走；垂直速度不受它限制（要留给核心板解锁定高度）。
3. **遥控失控分支会把速度清零并自动切返航**（`LX_Change_Mode(3)` + `OneKey_Return_Home()`），
   所以现场**必须保持遥控链路常开**，或者按机载端文档的建议改这一分支。

另外 `LX_FC_State_Task()` 是**摇杆解锁/上锁**：油门低位 + 特定摇杆手势保持 1s 解锁、
0.3s 上锁（`TOE_OUT / TOE_IN / LD_LD / RD_RD` 四个宏），且只在遥控正常时执行。

---

## 6. 一键起飞状态机：这次最大的改动

`User_Task.c::UserTask_OneKeyCmd()`（50Hz 调用）两版对比。

**09-13 版（`FcSrc_backup_20260913/User_Task.c`）**

```
CH6 高位(1700~2200) 且 takeoff_ready（屏上的"起飞准备"手势）
  step1: my_fly_flag=1；if (Con_flag) LX_Change_Mode(2)
  step2: FC_Unlock()
  step3: 等 2000ms
  step4: takeoff_if = 1                                  ← 直接放开速度门
  step5: if (rosData.acc[2] ∈ [-102,-98]) { FC_Lock(); takeoff_if = 0; }   ← 检测到 -100cm/s 下降就上锁
  step6/7: 复位、回到 step3
CH6 低位: takeoff_if = 0
```

**当前版（`FcSrc/User_Task.c`）**

```
CH6 高位
  if (mission_end_flag) 保持不动（机载端已宣布任务结束，不要重启状态机）
  else 启动 → step1: my_fly_flag=1；LX_Change_Mode(2)      ← 去掉了 Con_flag 条件
              step2: FC_Unlock()
              step3: 等 2000ms
              step4: （空）——takeoff_if / flown_flag 交给 0x66 起飞命令释放
CH6 低位: takeoff_if=0; mission_end_flag=0; flown_flag=0; mission_end_req=0; mission_tk_req=0
```

配合 `AnoDTRasp.c` 新增的 1ms 定时逻辑：

```c
if (mission_tk_req) {                          // 收到 0x66/0x01
    u16 h = mission_tk_h ? mission_tk_h : 150;
    if (OneKey_Takeoff(h)) {                   // 反复重试，直到核心板确认
        mission_tk_req = 0;
        takeoff_if = 1;                        // 放开速度门
        flown_flag = 1;                        // 放开水平速度
    }
}
if (mission_end_req) {                         // 收到 0x66/0x02~0x04
    0x02 → OneKey_Land() / 0x03 → OneKey_Return_Home() / 0x04 → FC_Lock()；
    成功清零，失败下一毫秒继续重试
}
```

语义变化总结：

| | 09-13 版 | 当前版 |
|---|---|---|
| 谁决定起飞 | 遥控 CH6 + 屏上 `takeoff_ready` | 遥控 CH6 只负责"模式2 + 解锁"，**真正起飞由机载端 0x66 决定** |
| 爬升高度 | 机载端 PID 闭环爬升 | 核心板自己的"一键起飞到 h cm" |
| 谁决定结束 | 检测到 -100cm/s 下降自动上锁 | 机载端发 `0x66` 降落/返航/上锁 |
| 重做任务 | 拨 CH6 低位再拨高位 | 必须拨 CH6 低位复位全部标志 |

方向是对的（更符合"全自主 + 一键启动"的赛题要求），
**但它要求机载端配套改成发 `0x66/0x01`** —— 正是第 8 节 P0-1 的问题。

---

## 7. 版本与变更记录（用哈希/差异实测）

| 文件 | FcSrc | backup_20260913 | 说明 |
|---|---|---|---|
| `AnoDTRasp.c` | 21265 B | 20310 B | **当前多了 0xF1 扩展**（9→16 字节，回显核心板状态）；`FcSrc/AnoDTRasp.c.bak` 与备份版 MD5 完全相同 |
| `User_Task.c` | 5269 B | 5545 B | **状态机重写**（见第 6 节） |
| `LX_FC_EXT_Sensor.c` | 3867 B | 5698 B | 删掉大段厂商注释；`Set_m_speed_now_Invalid()` 两版都有；当前版有 `tacc` vs `tloc` 的修正说明 |
| `ANO_LX.c` | 12964 B | 12964 B | 与备份相同；相对更老的 `ANO_LX.c.bak`（9649 B）多了 `flown_flag` 双门控 |
| 其余 23 个文件 | — | — | MD5 完全相同 |

目录里同时散落着 `ANO_LX.c.bak`、`AnoDTRasp.c.bak` 和整份 `FcSrc_backup_20260913/`，
**说明版本管理是靠"复制一份再改"**。这次解析就发现两版行为语义完全不同，
建议至少给关键文件加版本/日期注释，或者纳入 git。

---

## 8. 与机载端（`drone_d_ws`）的接口对照

| 帧 | 飞控侧（本份代码） | 机载端（`uart_to_stm32`） | 结论 |
|---|---|---|---|
| `0x31` 速度指令 | 8 字节 vx/vy/vz/vyaw，限幅 ±60/±30，200ms 看门狗 | 同 | ✅ 一致 |
| `0x32` 外部速度 | 3×s16 → `Set_m_speed_now` → `0x33` 给核心板 | TF 差分速度 | ✅ 一致（**必须是速度不是位置**，当前版注释专门修过 tloc→tacc 的错） |
| `0x05` 高度 | 2 字节 s16，5ms | → `/height` | ✅ 一致 |
| `0x40` 遥控 | 20 字节 µs | → `/rc_channels`（调试） | ✅ 一致 |
| `0x0D` 电池 | 4 字节，100ms | 未处理（ignore） | ⚠️ 地面站遥测里电量一直为空 |
| `0xF1` 用户数据 | 16 字节（见 4.2 布局） | 读 `data[0]`→`/mission_step`、`data[1]==1`→`/is_st_ready`、`data[3..4]`→`/fc_m_speed_z` | ⚠️ **语义对不上**（P1-4） |
| `0x62` 航线回读 | F407 → 机载端（屏上按钮触发） | 未处理 | ⚠️ 忽略即可 |
| `0x63` 航线写入 | 机载端 → F407（写屏表格） | 机载端不发 | — |
| `0x66` 任务控制 | **起飞(0x01+高度) / 降落(0x02) / 返航(0x03) / 上锁(0x04)，要求 dataLen≥3** | 只发 1 字节 `0x06`（任务结束） | ❌ **P0-1** |
| `0x33` | F407 当"定位（6 字节）"用 | 机载端当"电磁铁（1 字节）"用（上电发一次 `0x00`） | ❌ **ID 冲突，P1-2** |
| `0x21` 激光 | **没有该 case**，走 default 丢弃 | 激光 `driver=topic` 时发 `0x21` | ❌ P1-3（所以只能用香橙派 GPIO 那条路） |
| `0x11` 舵机 | 没有该 case | 会发 | ⚠️ D 题用不到 |

### P0-1（最严重）机载端不发起飞命令，按当前固件飞不起来

实测证据（`rg "takeoff_if\s*=\s*1"` 全仓库）：

```
FcSrc/AnoDTRasp.c:74                   takeoff_if = 1;   ← 只在 0x66/0x01 分支里
FcSrc_backup_20260913/User_Task.c:150  takeoff_if = 1;   ← 旧版在这里（CH6 状态机）
```

即：**当前 `FcSrc` 里 `takeoff_if=1` 的唯一来源是机载端发 `0x66` 且 `sub=0x01`**；
而机载端 `uart_to_stm32.hpp` 里 `MISSION_COMPLETE_VALUE = 0x06`，
`sendMissionCompleteToSerial()` 只发 **1 字节**，且只在任务结束时发一次。

后果：CH6 拨高位只完成"切模式 2 + 解锁 + 等 2s"，`takeoff_if` 恒为 0 →
`rt_tar.vel_* = 0` → **飞机不动**。

两个修法（选一个，别两边都改）：

1. **改机载端**（推荐，顺着现在固件的设计方向）：在 `uart_to_stm32` 加
   `sendMissionTakeoffToSerial(h_cm)`，帧 `0x66` + 3 字节 `[0x01, LO(h), HI(h)]`，
   在任务开始（收到 `start_route`）时发一次并重发几次；结束时发 `[0x02]`（降落）或 `[0x03]`（返航）。
2. **改飞控**：把 `User_Task.c` 的 `case 4` 恢复成 `takeoff_if=1; flown_flag=1;`
   （回到 09-13 版行为，由遥控器负责起飞），`0x66` 只用于降落/返航。

### P1-2　`0x33` ID 冲突

飞控把 `0x33` 当"定位 3×s16"，且**没做长度检查**：

```c
case 0x33:
    rosData.loc[0] = *(s16*)(dataBuf + 0);
    rosData.loc[1] = *(s16*)(dataBuf + 2);
    rosData.loc[2] = *(s16*)(dataBuf + 4);
```

机载端上电时会发一帧 1 字节的 `0x33 0x00`（电磁铁上电初始化）→ 飞控**越界读**帧缓冲，
把垃圾写进 `rosData.loc[]`，最终显示到串口屏的 tf_x/tf_y 上。
不会崩（缓冲是 256 字节的结构体），但属于明确隐患。
建议给 `case 0x33` 加 `dataLen>=6` 校验，或给机载端的电磁铁换一个 ID。

同类问题：`case 0x31`（应要求 ≥8 字节）、`case 0xA2`（应要求 ≥9 字节）也都**没有长度校验**。

### P1-3　激光走 STM32 这条路没实现

飞控端**没有** `0x21` 的 case，收到就走 `default` 丢弃。这与机载端 README 的说法一致
（"本机激光接在香橙派 GPIO36，`/laser_control` 走飞控 PD13 对它无效"）——
所以不是 bug，但两份配置必须保持这个结论，别再切回 `driver:=topic`。
以后若要走飞控，需要在 `AnoDTRasp.c` 的 switch 里补 `case 0x21`（引脚 PD13，注意激光 MOS 低电平导通）。

### P1-4　`0xF1` 各字段语义漂移

| 字节 | 飞控实际发 | 机载端当什么用 | 结果 |
|---|---|---|---|
| `data[0]` | `mission_end_sub`（最近一次 0x66 子命令） | `/mission_step` | 名称对不上，值会是 0~4 |
| `data[1]` | `mission_step`（一键状态机 1~4） | `==1` 则 `/is_st_ready=1` | 只在状态机第 1 步瞬间触发，属巧合 |
| `data[2]` | `qr_num`（扫码编号） | — | 未使用 |
| `data[3..4]` | **最近一帧的 dataLen** | `/fc_m_speed_z`（以为是最新 z 速度） | ❌ 会一直是个小数字（3/8…），不是速度 |
| `data[5..8]` | `rosData.tacc[0..1]` | — | 未使用 |
| `data[9..15]` | 核心板状态回显（模式/解锁/最后命令 CID+CMD） | — | **机载端完全没用，但这是排查"电机不转"最有价值的信息** |

建议：机载端把 `data[9..15]` 解析出来（发 `/fc_status` 或并入 `/shelf_scan/status`），
并把 `/fc_m_speed_z` 的来源换成飞控真正持有的 `m_speed_z`（需要在 0xF1 里换个字段）。

### P2　其它不对齐

- 机载端不处理 `0x0D`（电池）→ 地面站遥测 `batt` 一直为空；飞控其实已经在 100ms 周期发。
- 飞控侧 `AnoRos_DT.c`、蓝牙、`checkZeroDistance` 等属死代码/半死代码，
  读代码时容易被误导（例如 `AnoRos_DT.c` 从没进过编译）。

---

## 9. 代码质量问题清单

| 级别 | 位置 | 问题 | 建议 |
|---|---|---|---|
| 中 | `AnoDTRasp.c` case `0x31/0x33/0xA2` | 不校验 `dataLen` 就按固定偏移取数 | 加长度判断，短帧直接丢 |
| 中 | `AnoDTRaspRecvOneByte` | 状态机无长度上限校验（依赖 `dataLen<250` 的隐含约定），且校验失败后**没有重新同步机制** | 加范围判断 + 校验失败丢弃整帧并重新找帧头 |
| 中 | `Drv_Uart.c` | 发送缓冲 `Tx1Buffer[256]` 等用 `u8` 计数、无越界检查；高频发送可能覆盖未发完的数据 | 加剩余空间判断或改成 FIFO |
| 中 | `ANO_LX_Task()` | 1ms 中断里做 5 路串口解析 + 电机 PWM + 命令重发；任一帧处理超时就会丢节拍 | 把非紧急工作（命令重发、LCD、统计）移到主循环 |
| 中 | `RC_Data_Task` 失控分支 | 遥控失控 ⇒ 速度清零 + 强制模式 3 + 一键返航；不能靠拔遥控做"全自主" | 赛前确认遥控常开；或改这一分支由 `0x31` 超时决定 |
| 低 | `User_Task.c` | `mission_end_flag` 与 `one_key_mission_f` 两套状态标志并存，靠 CH6 低位复位 | 文件头补一张状态图 |
| 低 | `Bluetooth.c` | `case 0xB1:` / `case 0xB2:` **缺 break**（fall-through），且 `Bluetooth_speed_check()` 整段被注释 | 文件整体属死代码，建议删除或明确标注"未启用" |
| 低 | `TJC_lcd.c` | `Send_speed(x,y,z,mission_step_cun,0)` —— 第 4 个参数位置塞的是**状态机步数**（借用了"yaw"槽位）；`sprintf` 无长度检查 | 加注释，改用 `snprintf` |
| 低 | `Drv_BSP.c::All_Init()` | 初始化里夹着 `MyDelayMs(100/800)` 硬延时（合计约 1.1s） | 无伤大雅，但要知道上电约 1s 后才开始跑 |
| 低 | 全仓库 | 源码是 GBK 编码，与 UTF-8 注释混用（本次 diff 里大量"乱码"其实是编码差异，不是逻辑差异） | 统一编码，否则 diff 会被噪声淹没 |

---

## 10. 建议的下一步验证

1. **先确认机上跑的是哪份固件**：把 `ProjectSTM32F407/ANO-LX.bin`（9-25 编译）
   与飞机实际运行的版本/日期对比。如果机上还是 09-13 之前的版本，第 8 节 P0-1 要按机上版本重判。
2. **打通起飞链路**（二选一，见 P0-1），再按机载端 `docs/无地面站飞行自测.md` 的拆桨流程验证：
   `/target_velocity` 有输出 → CH6 拨高 → 电机转 → 日志里出现"模式 2 / 解锁"。
3. **把 0xF1 的核心板状态接进机载端**（`fc_mode_sta / unlock_sta / cmd_fun`），
   以后再遇到"飞机不动"，从 ROS 侧就能看到：核心板认没认模式、解没解锁、最后执行的是哪条命令。
4. 给 `case 0x31/0x33/0xA2` 补长度校验，并消掉 `0x33` 的 ID 冲突。
5. 把 `FcSrc` 纳入 git（或至少删掉散落的 `.bak`），避免再出现
   "两份代码语义不同、不知道哪份在飞机上"的情况。
