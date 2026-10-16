# p_to_p_px4 —— PX4 uXRCE-DDS Offboard 桥（本机用，只走 DDS）

从参考件 `onboard_ws/p_to_p_flight_px4` 的 `p_to_p_px4` 精简而来：**只保留
`px4_offboard_bridge`**（+ `frames.py` / `position_ramp.py`）。

## 为什么精简

| 参考件里的模块 | 本机处置 | 原因 |
| --- | --- | --- |
| `usb_odometry_transport.py`、`tof_mavlink_bridge.py` | 删 | 走 USB MAVLink 的备用通路，本机用 uXRCE-DDS |
| `odometry_bridge.py`、`odometry_output.py`、`lidar_velocity.py` | 删 | 本机 EV 由 `px4_ev_bridge` 负责，**不能有两处发 `vehicle_visual_odometry`** |
| `three_axis_preview.py`、`range_preview.py`、`sample_alignment.py`、`tofsense.py` | 删 | 那套 TOF/三轴可视化在参考件里是给 USB 通路的 |
| `frames.py`、`position_ramp.py`、`offboard_bridge.py` | **留** | 桥本体：map 目标 → 锚定 → PX4 NED 设定值 + 位置斜坡限速 |

## 桥的行为（保留参考件原逻辑）

- 进 Offboard 时抓一次 `map(TF) ↔ PX4 NED` 锚点，之后目标按锚点换算；
- 以 `output_rate_hz`(20 Hz) 发 `OffboardControlMode` + `TrajectorySetpoint`；
- **`bench_only=true`（默认）时一个字节都不发**；不发模式/解锁命令（`allow_mode_command=false`），
  `allow_land_command=false`、`land_on_mission_complete=false`；
- 由遥控器解锁 + 切 Offboard 触发任务（`auto_start_on_offboard=true`）；
- 目标超时/姿态超时 → 故障保持（`land_on_fault=false`，只停输出不外飞）；
- 顺带把飞控 `dist_bottom` 以 `Int16`（cm）发到 `/height`，供任务节点用。