# 2024 D 题任务移植到 PX4

## 已本地化的接口

- 激光雷达：沿用 `/dev/ttyS4`，由现有 `mavlink-ws-chain.service` 的
  `bluesea2` 发布 `/scan`。
- 摄像头：`/dev/video0`，由 `shelf_code_reader_node` 独占。
- 激光笔：香橙派排针 18 脚（`gpiochip1` 第 4 脚，全局 GPIO 36），
  低电平点亮；需要 `/dev/gpiochip1` 对 `dialout` 组可写。
- 定位：`map -> laser_link` TF、`/odom_from_slam`、`/height`。
- 飞控：原有匿名 STM32 `0x31/0x32/0x66` 不再使用。PID 输出的机体系
  `/target_velocity` 由 `mavlink_bridge` 转成 PX4
  `SET_POSITION_TARGET_LOCAL_NED`。
- 起飞触发：PX4 适配器在 OFFBOARD、已解锁、EKF 有效后发布
  `/fc_takeoff_if`，任务节点继续使用原版遍历盘点逻辑。
- 结束：任务节点发布 `/mission_complete`，适配器撤回任务信号并发送 PX4
  `Land`，等待 PX4 自行上锁。

## 编译

```bash
cd ~/mavlink_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select shelf_scan_pkg pid_control_pkg shelf_px4_bridge
source install/setup.bash
```

## 起飞前

1. 确认定位服务已运行：

```bash
systemctl is-active mavlink-ws-chain
pgrep -af 'cartographer_node|mavlink_bridge'
```

2. 在浏览器打开 `http://<香橙派IP>:8080/` 检查摄像头画面和二维码识别。
3. 把飞机放在场地起飞点，机头按 `field.json` 的
   `field_home_yaw_deg` 摆放。
4. 用遥控器选中 **OFFBOARD**，保持未解锁。
5. 执行预检，飞机仍不会解锁：

```bash
~/mavlink_ws/shelf_preflight.sh
```

看到 `PREFLIGHT PASSED` 后再执行：

```bash
~/mavlink_ws/shelf_takeoff.sh
```

中止：

```bash
~/mavlink_ws/shelf_abort.sh
```

或在飞行日志窗口按一次 `Ctrl-C`。

若 `gpioinfo gpiochip1` 报权限错误，安装仓库中的 udev 规则：

```bash
sudo cp ~/mavlink_ws/99-orange-pi-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=gpio
sudo chgrp dialout /dev/gpiochip*
sudo chmod 0660 /dev/gpiochip*
```

手动检查激光笔：

```bash
# 默认接线为低电平点亮，最长只允许 10 秒
bash ~/mavlink_ws/laser_test.sh low 10

# 若默认极性不亮，再用高电平试一次
bash ~/mavlink_ws/laser_test.sh high 10

# 强制恢复默认关闭电平
bash ~/mavlink_ws/laser_test.sh off
```

如果确认只有高电平才会亮，启动预检时加：

```bash
LASER_ACTIVE_HIGH=true ~/mavlink_ws/shelf_preflight.sh
```

测试期间保持激光朝向安全方向，并确保没有执行 `shelf_preflight.sh` 或
`shelf_takeoff.sh`。脚本发现激光节点或任务进程在运行时会拒绝执行。

## 首次实飞顺序

1. 拆桨完成预检、摄像头预览、激光开关和 `shelf_selftest`；
2. 地面验证 `/target_velocity` 坐标变换：机头 90° 时世界 `+X` 必须变成
   机体系相应轴，PX4 不应向错误方向加速；
3. 在空旷场地先用较低 `cruise_alt_cm` 和 `field.json` 中实际安全点位；
4. 第一次只做起飞、短航点、返航，确认 PX4 EKF 和 Offboard 连续；
5. 最后再执行整趟 12 点位遍历盘点与激光打点。

`field.json` 中的坐标来自匿名飞控飞机和原场地，换了机架、相机、激光或
定位安装位置后，必须重新测量悬停点和激光光斑，不能直接假定旧坐标仍准确。
