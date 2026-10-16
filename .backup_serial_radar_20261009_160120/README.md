# px4_ws —— PX4 无人机上位机工作区

Orange Pi 5 Max（Ubuntu 22.04 + ROS 2 Humble）上的机载端工作区。
面向 **PX4 1.15.4** 飞控 + **蓝海光电 LDS-50C-E-R 网口雷达**。

> 项目背景、硬件参数、雷达握手机制、踩坑记录见上级目录的
> [`../AGENTS.md`](../AGENTS.md) 和 [`../SSH_ACCESS.md`](../SSH_ACCESS.md)。

## 包一览

| 包 | 用途 | 状态 |
| --- | --- | --- |
| `bluesea2` | 雷达驱动（`base_lidar` + `bluesea-ros2`），UDP 网口 | ✅ 已跑通，`/scan` 15Hz |
| `carto_pkg` | cartographer 2D 建图（`fly.lua` + urdf） | ✅ 已适配（雷达由 run.sh 单独起） |
| `slam_pkg` | slam_toolbox 2D 建图（另一套方案） | ✅ 已适配 |
| `px4_msgs` | PX4 uORB 消息定义，**从 PX4 1.15.4 源码生成** | ✅ 与飞控版本一致 |
| `px4_bridge` | 上位机 ↔ PX4 Offboard 桥接（uXRCE-DDS） | ✅ 骨架完成，待联调 |
| `serial_comm` | boost 串口库（给非网口设备留的） | 备用 |

## 快速开始

```bash
# 编译
bash build.sh              # 或 bash build.sh --clean 干净重建

# 确认雷达（应看到 15Hz 左右）
bash run.sh lidar

# 建图（二选一）
bash run.sh carto          # cartographer
bash run.sh slam           # slam_toolbox

# PX4 桥接（前提：Micro-XRCE-DDS-Agent 已在跑）
bash run.sh px4

bash run.sh status         # 看进程/话题
bash run.sh stop           # 停掉
```

调试时 `source env.sh`，之后可直接用 `ros2 topic` / `ros2 launch` 等命令。

## 依赖

ROS 2 Humble 之外还需要（apt）：

```bash
sudo apt install -y ros-humble-cartographer-ros ros-humble-slam-toolbox \
                    ros-humble-nav2-map-server
```

## 重要说明

1. **雷达必须先绑定 `local_port(6668)` 再从同一个 socket 发指令**，否则收不到数据
   —— 这是这颗雷达的回发机制决定的，详见 `../SSH_ACCESS.md`。
2. 雷达参数在 `src/bluesea2/src/bluesea-ros2/params/udp_lidar.yaml`：
   `lidar_ip=192.168.158.98`、`lidar_port=6543`、`local_port=6668`、
   `interface=enP3p49s0`。
3. 本工作区的脚本默认 `ROS_DOMAIN_ID=0`，可用环境变量覆盖。
4. `px4_msgs` 与飞控固件版本**必须一致**；升级 PX4 后要重新从
   `PX4-Autopilot-<版本>/msg` 与 `srv` 同步。
