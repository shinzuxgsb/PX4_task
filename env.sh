#!/usr/bin/env bash
# source env.sh 之后可直接用 ros2 topic / ros2 launch 等命令

_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
if [ -f "$_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$_WS/install/setup.bash"
else
  echo "[警告] 还没编译：先执行 bash $_WS/build.sh"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "px4_ws 已就绪。ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  bash $_WS/run.sh lidar    # 只起雷达"
echo "  bash $_WS/run.sh carto    # 雷达 + cartographer"
echo "  bash $_WS/run.sh slam     # 雷达 + slam_toolbox"
echo "  bash $_WS/run.sh px4      # 雷达 + PX4 桥接"
