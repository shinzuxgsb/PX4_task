#!/bin/bash
# 启动悬停测试任务（必须以 orangepi 用户运行：ROS 2 DDS 共享内存跨用户会失败）
#   sudo -u orangepi bash /home/orangepi/drone_link/tasks/run_hover.sh --height 50 --hover 5 [--dry-run]
set -e

source /opt/ros/humble/setup.bash
# cartographer_ws 的 setup 会把 ros2_ws/install（mavlink_bridge 等）一并带进来
source /home/orangepi/cartographer_ws/install/setup.bash

cd /home/orangepi/drone_link
exec python3 -u /home/orangepi/drone_link/tasks/hover_test.py "$@"
