#!/usr/bin/env bash
# 147 上跑"队友那套"(MAVLink ODOMETRY + Cartographer) 的工作空间启动脚本
#   bash ~/mavlink_ws/start_mavlink_ws.sh          # 起整套（默认用串口雷达 LDS-50C-R）
#   bash ~/mavlink_ws/start_mavlink_ws.sh --net    # 用网口雷达参数（LDS-50C-E）
#   bash ~/mavlink_ws/start_mavlink_ws.sh stop     # 停掉本工作空间拉起的进程
#
# 前提（见 ADAPT_FOR_147.md §3）：
#   1) sudo systemctl stop px4-ev-chain        # 释放 ttyS4/ttyS3/ttyS6
#   2) 飞控侧：UXRCE_DDS_CFG=0、SER_TEL3_BAUD=115200（QGC 里改）

WS="/home/orangepi/mavlink_ws"

stop_all() {
  pkill -f 'ros2 launch mavlink_bridge' 2>/dev/null || true
  sleep 1
  pkill -f 'bridge_node|tf_to_odom|tofsense_node|bluesea2_node' 2>/dev/null || true
  pkill -f 'cartographer_node|cartographer_occupancy_grid_node|robot_state_publisher' 2>/dev/null || true
  sleep 1
  echo "已停止新工作空间的进程。"
}

if [ "${1:-}" = "stop" ]; then stop_all; exit 0; fi

# NTP is intentionally disabled. Do not wait for time sync here: the chain
# starts immediately using the board's current/RTC clock. This also avoids
# an in-flight NTP step changing LaserScan timestamps.

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/orangepi/cartographer_ws/install/setup.bash
source "$WS/install/setup.bash"
# pymavlink / fastcrc 用 third_party 挂载（香橙派上没 pip，也不动系统包）
export PYTHONPATH="$WS/third_party${PYTHONPATH:+:$PYTHONPATH}"
# 强制 FastDDS 走 UDP（同机 SHM 偶发"看得见收不到"，10-09 踩过）
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/config/fastdds_udp_only.xml"

SHARE="$WS/install/mavlink_bridge/share/mavlink_bridge/config"
LIDAR="$SHARE/lidar_uart.yaml"
[ "${1:-}" = "--net" ] && LIDAR="$SHARE/lidar_net.yaml"

echo "雷达参数：$LIDAR"
echo "（若起不来先看：旧链是否已停 / 飞控是否已切 MAVLink）"
exec ros2 launch mavlink_bridge drone_stack.launch.py lidar_params_file:="$LIDAR"
