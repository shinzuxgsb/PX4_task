# shelf_scan_pkg (PX4 localized)

2024 电赛 D 题立体货架盘点任务包。任务算法沿用匿名飞控版本，飞控通信层已
替换为当前 PX4 飞机的 MAVLink/Offboard 链路。

## 输入

- `/scan`：`bluesea2`，串口雷达 `/dev/ttyS4`
- `map -> laser_link` TF：Cartographer
- `/height`：`mavlink_bridge` 发布的相对高度（cm）
- `/shelf_scan/code`：`/dev/video0` 二维码识别结果
- `/fc_takeoff_if`：`shelf_px4_adapter` 的 PX4 GO 信号

## 输出

- `/target_position`：场地坐标目标 `[x_cm,y_cm,z_cm,yaw_deg]`
- `/target_velocity`：由 `pid_control_pkg` 输出为机体系 FLU
- `/shelf_scan/laser`：激光短脉冲控制
- `/mission_complete`、`/mission_active`：任务生命周期

相机为 640×480 MJPG，采集请求 120 FPS；识别独立为 30 FPS，地面站
MJPEG 预览固定 30 FPS。采集、识别和预览分线程运行。

匿名飞控的 `uart_to_stm32`、`0x31/0x32/0x66` 和 STM32 串口协议在此版本中
均不使用。

## 运行

不要单独启动本包；使用工作区顶层的两段式流程：

```bash
~/mavlink_ws/shelf_preflight.sh
~/mavlink_ws/shelf_takeoff.sh
```

完整说明见 `~/mavlink_ws/SHELF_PX4_FLIGHT.md`。
