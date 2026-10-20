#!/bin/bash
# 启动整套无人机链路（供 systemd 调用，也可手动执行）
set -e

source /opt/ros/humble/setup.bash
# cartographer_ws 的 setup 会把 ros2_ws/install 一并带进来（mavlink_bridge 在里面）
source /home/orangepi/cartographer_ws/install/setup.bash

exec ros2 launch mavlink_bridge drone_stack.launch.py
