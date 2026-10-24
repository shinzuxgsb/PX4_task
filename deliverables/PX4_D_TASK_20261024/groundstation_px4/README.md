# PX4 Shelf Inventory Ground Station

基于原 2024 电赛 D 题地面站裁剪的 PX4 版本，保留场地绘图、航线规划、
盘点结果、查询和 CSV 导出，并增加：

- `/dev/video0` 机载相机实时 MJPEG 画面；
- PX4 预检状态与解锁状态；
- 预检通过后，在网页直接“起飞并开始盘点”；
- 返航降落和立即降落按钮；
- 遍历盘点与定向盘点。

## 启动

Windows 双击 `run_groundstation.bat`。默认界面：

`http://127.0.0.1:8765/`

演示模式双击 `run_demo.bat`，不需要飞机。

## 机载端连接

1. 地面站默认监听 `0.0.0.0:8890/ws`。
2. 修改 `~/mavlink_ws/config/groundstation.env` 中的电脑 IP：

```bash
GROUNDSTATION_ENABLED=true
GROUNDSTATION_WS_URL=ws://10.249.114.28:8890/ws
```

3. 在飞机端运行：

```bash
~/mavlink_ws/shelf_preflight.sh
```

等到网页顶部显示“机载已连接”，飞行状态显示“可以起飞”，再点击
“起飞并开始盘点”。地面站只发送请求；适配器仍会复核 PX4 模式、EKF、
定位稳定、ToF 地面高度与预检状态，任何一项不满足都不会解锁。

## 相机

相机 URL 在 `config/runtime.json`：

```json
"camera": {
  "enabled": true,
  "url": "http://10.249.123.10:8080/stream",
  "label": "机载摄像头 /dev/video0"
}
```

香橙派 IP 变化时修改此地址。相机画面由 `shelf_code_reader_node` 直接输出，
地面站不抢占摄像头设备。
