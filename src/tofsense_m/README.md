# tofsense_m —— Nooploop TOFSense-M 激光定高器驱动

读串口 → 解析 NLink 协议 → 发布高度。这是"定高器接香橙派"这条路上缺的那块。

## 协议要点（来自 TOFSense-M 用户手册 V3.0）

- 串口默认 **921600**、8 数据位、1 停止位、无校验、无流控，**小端**
- 输出帧 `NLink_TOFSense_M_Frame0`：

| 字段 | 类型 | 长度 | 说明 |
| --- | --- | --- | --- |
| Frame Header | uint8 | 1 | 固定 `0x57` |
| Function Mark | uint8 | 1 | 输出帧 `0x01`；查询帧 `0x10` |
| reserved | uint8 | 1 | |
| id | uint8 | 1 | 模块 ID |
| system_time | uint32 | 4 | ms |
| zone_map | uint8 | 1 | `0x40`=8x8(64点/400字节/15Hz)；`0x10`=4x4(16点/112字节/60Hz) |
| data0..N | uint24+uint8+uint16 | 6/点 | 距离(×1000)、距离状态、信号强度 |
| reserved | - | 6 | |
| SumCheck | uint8 | 1 | 前面所有字节求和取低 8 位 |

- 整帧长 = `16 + N*6`
- 查询帧（要它回一帧时用）：`57 10 FF FF <id> FF FF <校验和>`，例如 id=0 是 `57 10 FF FF 00 FF FF 63`

## 编译 / 运行

```bash
# 权限（只做一次，然后重新登录）
sudo usermod -aG dialout orangepi

cd ~/px4_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tofsense_m
source install/setup.bash

ros2 run tofsense_m tofsense_m_node --ros-args -p port:=/dev/ttyS3
```

话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/tofsense/range` | `sensor_msgs/Range` | 高度（米） |
| `/tofsense/height` | `std_msgs/Float32` | 同一个数值 |
| `/tofsense/zones` | `std_msgs/Float32MultiArray` | 各像素距离，调试用 |

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `port` | `/dev/ttyS3` | 定高器所在的串口 |
| `baud` | 921600 | 手册默认值 |
| `range_mode` | min | `min`=取最近的有效像素（正下方地面）；`center`=取中心像素；`mean`=有效像素平均 |
| `distance_scale` | 1000.0 | 米 = int24值 / 该系数。手册示例按 492 当 mm 的话改成 1000000 |
| `status_ok_max` | 0 | 距离状态大于该值视为无效 |
| `frame_id` | tof_link | |
| `min_range` / `max_range` | 0.03 / 8.0 | Range 消息的范围 |

## 接到 PX4 的下一步

PX4 的 uXRCE-DDS 输入里**没有 distance_sensor 话题**，所以不能作为"测距"喂给 EKF2。
要把高度送进去，只有一条路：**把高度塞进 `vehicle_visual_odometry` 的垂直分量**，
配合飞控参数 `EKF2_EV_CTRL` 打开 bit1（垂直位置）、`EKF2_HGT_REF=3`（Vision）。
这一步在 `px4_ev_bridge` 里加一路高度输入即可（雷达给水平位置、定高器给垂直位置）。
