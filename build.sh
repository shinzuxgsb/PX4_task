#!/usr/bin/env bash
# px4_ws 一键编译
#
#   bash build.sh            编译全部
#   bash build.sh --clean    先删 build/install/log 再编（干净重建）

set -Eeo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[错误] 找不到 /opt/ros/humble/setup.bash，请先装 ROS 2 Humble。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WS"
if [ "${1:-}" = "--clean" ]; then
  echo "清理 build/ install/ log/ ..."
  rm -rf build install log
  shift
fi

echo "工作区：$WS"
colcon build --symlink-install \
  --event-handlers console_cohesion+ \
  "$@"

echo
echo "编译完成。使用前先: source $WS/install/setup.bash"
