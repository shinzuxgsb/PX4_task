# 两命令起飞流程

假设整套定位链路已经启动，进入香橙派 SSH 终端后只需要输入两条命令：

```bash
~/mavlink_ws/square_preflight.sh
```

第一条命令会：

1. 启动正方形任务控制进程并持续发送零速度 setpoint。
2. 检查定位频率、定位稳定性、高度数据和 MAVLink。
3. 提示你使用遥控器手动切入 OFFBOARD；飞机始终保持未解锁。
4. 检测到 PX4 已进入 OFFBOARD 后，检查 EKF、电池和未解锁状态。
5. 全部通过后打印 `PREFLIGHT PASSED`，并保持飞机未解锁等待 GO。

看到 `PREFLIGHT PASSED` 后再输入：

```bash
~/mavlink_ws/square_takeoff.sh
```

第二条命令会触发程序自动解锁、Offboard 起飞、顺时针 120 cm 正方形、
返回起点、下降和自动降落，并持续显示飞行日志。

中止：

```bash
touch /tmp/square_abort
```

在飞行日志窗口按 `Ctrl-C` 也会请求降落。

两条命令之间必须保持飞机静止。预检结果有效期为 300 秒；超过有效期会退出并
保持未解锁，需要重新执行第一条命令。
