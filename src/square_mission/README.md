# square_mission

香橙派机载 PX4 板外（Offboard）正方形定点导航任务。起飞点是正方形的左下角，
航线固定为：

```text
左下角（起飞点）
   ↑ 前进 120 cm
左上角 → 右 120 cm → 右上角
                         ↓ 后 120 cm
左下角 ← 左 120 cm ← 右下角
                         ↓
                       降落
```

这个顺序在世界坐标的 x 向右、y 向前的平面中是顺时针。

## 依赖

使用现有 `mavlink_bridge` 的话题，不额外占用飞控串口：

- 订阅 `/odom_from_slam`、`/height`、`/is_st_ready`
- 发布 `/target_velocity`、`/px4_command`

飞行前必须已经启动整套链路，并确认 `/is_st_ready` 有发布：

```bash
source /opt/ros/humble/setup.bash
source /home/orangepi/cartographer_ws/install/setup.bash
ros2 topic list
ros2 topic echo /height --once
ros2 topic echo /is_st_ready --once
```

## 编译

```bash
cd /home/orangepi/mavlink_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select square_mission --symlink-install
source install/setup.bash
```

## 先干跑

不带 `--arm` 时程序不会解锁、不会起飞，只等待定位和高度数据稳定：

```bash
ros2 run square_mission square_mission
```

明确写 `--dry-run` 效果相同：

```bash
ros2 run square_mission square_mission --dry-run
```

## 实飞

推荐使用 `~/mavlink_ws/RUN_TWO_COMMANDS.md` 中的两命令流程：

```bash
~/mavlink_ws/square_preflight.sh
~/mavlink_ws/square_takeoff.sh
```

第一条命令启动预检并保持零速度 setpoint 流，你用遥控器手动切入 OFFBOARD。
预检确认模式、EKF、定位、电池等均正常后，第二条命令触发自动解锁和起飞。

降落策略：

1. 在约 100 cm 高度返回起点，直接复用普通航点容差和稳定时间：
   位置误差不超过 8 cm，并连续保持 0.8 秒；最长等待 15 秒。
2. 把起飞点作为下降过程中的水平目标点，以约 25 cm/s 持续下降。
3. 下降过程中不暂停、不进入低空悬停追点。
4. TOF 相对高度回到起飞零点附近、垂直速度不超过 3 cm/s 并保持 0.5 秒，
   才认为进入最终触地段。
5. 最终触地段继续朝起点以 5 cm/s 下降 1 秒，避免近地面 TOF 多径反射
   导致“还在离地几厘米”就误判。
6. 然后发送一次 disarm；若 PX4 返回拒绝或 1 秒内没有确认上锁，
   第二条 PX4 命令直接切换为 Land，不再反复尝试 disarm。

把飞机放在正方形左下角，机头朝正方形第一条边（前进方向）。清空人员，确认
遥控器已连接且随时可切回 Position 人工接管，然后运行：

```bash
ros2 run square_mission square_mission --arm --ros-args \
  -p side_cm:=120.0 \
  -p height_cm:=100.0 \
  -p waypoint_speed_cm_s:=16.0
```

默认目标高度是离起飞地面 100 cm，可改成现场安全高度。边长默认就是 120 cm。

## 中止

任一终端执行下面命令，程序会立即下发 PX4 `land`：

```bash
touch /tmp/square_abort
```

也可以直接按 `Ctrl-C`。遥控器始终是最高优先级的人工接管方式。

## 安全门禁与保护

- 起飞前要求外部定位在 1.5 秒内水平变化不超过 5 cm、偏航变化不超过 5°。
- 解锁前先切到 Position，并持续发送零速度 setpoint。
- 只有 `/is_st_ready=1`（已解锁且 EKF 有效）才切 Offboard 和起飞。
- 航点容差 8 cm，每段超时、总时长超时、定位跳变或数据陈旧都会请求降落。
- 正常回起点后先悬停 3 秒；返回目标使用与普通航点相同的 8 cm / 0.8 s。
- TOF 接近地面后仍继续短距离下降用于真正触地，然后只尝试一次 `disarm`。
- 第一次 `disarm` 被拒绝时立即交给 PX4 `land`，避免低空悬停等待上锁。
- 返回和下降期间持续把起点作为目标点，不需要额外的降落位置门限参数。
- 未收到“已上锁”的降落确认时不会把任务判为成功。

该程序不会替代飞控自身的预检、遥控器接管、电池保护和应急预案。
