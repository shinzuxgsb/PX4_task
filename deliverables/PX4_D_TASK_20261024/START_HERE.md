# PX4 D 题任务执行包

版本：2026-10-24

本包用于在已经部署好 PX4 MAVLink、雷达、Cartographer、定高器的香橙派上，
执行 2024 电赛 D 题立体货架盘点任务。包含：

- `groundstation_px4/`：简洁网页地面站、实时相机、状态显示和起飞控制；
- `ros2_src/shelf_scan_pkg/`：场地、航线、二维码、激光和地面站协议；
- `ros2_src/pid_control_pkg/`：机体系速度 PID；
- `ros2_src/shelf_px4_bridge/`：PX4 预检、解锁、Offboard 和降落适配器；
- `scripts/`：预检、起飞、中止、激光测试和 GPIO 权限规则；
- `config/`：地面站 IP、场地参数参考和 PX4 桥接参数参考；
- `docs/SHELF_PX4_FLIGHT.md`：更详细的操作与安全说明。

## 1. 硬件和网络

- 香橙派：Orange Pi 5 Max，Ubuntu 22.04 + ROS 2 Humble
- 飞控：PX4 1.15.4，MicoAir743 v2
- 雷达：`/dev/ttyS4`，921600
- 相机：`/dev/video0`，640×480 MJPG
- 香橙派默认地址：`10.249.123.10`
- 地面站电脑当前地址：`10.249.114.28`

若地址变化：

1. 修改 `groundstation_px4/config/runtime.json` 中的 `camera.url`；
2. 登录香橙派修改 `~/mavlink_ws/config/groundstation.env` 的
   `GROUNDSTATION_WS_URL`，改为 `ws://<电脑IP>:8890/ws`。

## 2. 安装

在香橙派解压本包并执行：

```bash
cd PX4_D_TASK_20261024
bash scripts/install_to_orange_pi.sh --force
```

默认安装到 `~/mavlink_ws`。脚本会编译：

```text
shelf_scan_pkg
pid_control_pkg
shelf_px4_bridge
```

`config/mavlink_bridge.reference.yaml` 仅用于核对当前 `mavlink_bridge`
参数。请将它与飞控实际配置比较后手动合并，不能直接覆盖队友的桥接文件。

若新板尚未配置激光 GPIO 权限，一次性执行：

```bash
sudo install -m 0644 scripts/99-orange-pi-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=gpio
sudo chgrp dialout /dev/gpiochip1
sudo chmod 0660 /dev/gpiochip1
```

## 3. 启动地面站

在 Windows 电脑解压后双击：

```text
groundstation_px4\run_groundstation.bat
```

浏览器打开：

```text
http://127.0.0.1:8765/
```

`run_demo.bat` 不需要飞机，用于界面和流程演练。

## 4. 起飞前

1. 把飞机放在场地起飞点，机头按 `field.json` 方向摆正；
2. 检查电池、桨叶、遥控器、TOF、雷达和相机；
3. 根据现场重新核对悬停点与激光光斑；
4. 香橙派执行：

```bash
~/mavlink_ws/shelf_preflight.sh
```

5. 按终端提示把 PX4 切到 **OFFBOARD**，保持未解锁；
6. 地面站必须同时显示：

```text
机载已连接
EKF 正常
未解锁
可以起飞
```

## 5. 开始任务

确认现场安全后，点击“起飞并开始盘点”，再确认。

实际时序：

```text
地面站发送 px4_start
→ 适配器复核预检
→ PX4 解锁
→ 确认 Offboard + EKF
→ 任务节点起飞
→ 12 点位遍历盘点
→ 激光短脉冲与二维码结果回传
→ 返航到降落点
→ PX4 Land 并等待上锁
```

## 6. 中止

优先点击网页“立即降落”，或执行：

```bash
~/mavlink_ws/shelf_abort.sh
```

遥控器永远是最高优先级的人工接管方式。异常时把遥控器切出 OFFBOARD，
任务适配器会释放任务信号。

## 7. 完成

确认地面站显示任务完成且 PX4 已上锁、电机停止后，人员再接近飞机。
再次执行任务前重新运行 `shelf_preflight.sh`。

## 8. 已验证状态

- 机载三包编译通过；
- 原任务自测 23/23；
- 适配器单测 5/5；
- 实测相机采集约 99 FPS；
- 二维码识别约 28–30 FPS；
- 地面站 MJPEG 稳定 30 FPS；
- 实机链路已验证至 `WAIT_GO`，网页起飞按钮可用；
- **尚未在实际 PX4 飞机上完成整套飞行。**
