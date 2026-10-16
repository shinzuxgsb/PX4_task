# p_to_p_mission —— A→B 点对点任务（PX4 Offboard 版）

逻辑移植自用户 2026-10-12 提供的参考件 `onboard_ws/p_to_p_flight_px4`（只读副本在
`D:\px4无人机\ref_onboard_ws`）。**改动与理由一律记在 `D:\px4无人机\CHANGELOG.md`。**

## 它做什么

`IDLE → TAKEOFF → GOTO_B → HOLD_B`（或 `DESCEND_B`），四道本地包线：
`task_guard`（半径/高度/各段超时）、起飞前地面测高基线自检、到点稳定判定、跳变/超时故障保持。
到 B 的行为由 `terminal_behavior` 决定：`hold`（默认，悬停）/ `guided_land_handoff`
（限速下降到地面基线上方约 4 cm，再交还 PX4 `NAV_LAND`）。

## 与参考件的差异（移植改动）

1. 删掉 `position_pid_controller` / `uart_to_stm32_node` 两节配置 —— 那是 STM32 串口那套，本机不用；
2. `terminal_behavior` 默认 `hold`（参考件默认值也是 hold，但它的 launch 会覆盖成
   `guided_land_handoff`），本机要落地必须显式改；
3. 包线按本机事故史收紧：半径 1.5 m / 高度上限 1.2 m / 各段超时 20 s；
4. 高度来源 `/height`（Int16，cm）由 `p_to_p_px4` 的桥从飞控 `local_position.dist_bottom` 发布。

## 怎么跑（**默认只预演，不解锁**）

```bash
# 前提：雷达+cartographer+px4_ev_bridge+MicroXRCEAgent 已在跑（run.sh fly / px4-ev-chain）
source /opt/ros/humble/setup.bash; source ~/px4_ws/install/setup.bash
ros2 launch p_to_p_mission p_to_p.launch.py                  # bench_only 默认 true
ros2 topic echo /p_to_p/state            # 看状态
ros2 service call /p_to_p/readiness_check std_srvs/srv/Trigger {}
ros2 service call /p_to_p/start std_srvs/srv/Trigger {}
```

真跑（**先看 CHANGELOG 的"下一步"和 preflight**）：`bench_only:=false cruise_height_cm:=50
b_offset_x_cm:=100 terminal_behavior:=hold`，遥控器解锁后再切 Offboard。