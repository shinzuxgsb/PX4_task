#!/usr/bin/env bash
# 停掉本工作区拉起的进程
set -Eeo pipefail

# 2026-10-09：补上 px4_imu_bridge（新）以及以前漏掉的 px4_ev_bridge_node /
# tofsense_m_node / tf_to_pose / MicroXRCEAgent
PATTERN='bluesea2_node|cartographer_node|cartographer_occupancy_grid_node|slam_toolbox|px4_bridge_node|px4_ev_bridge_node|tofsense_m_node|tf_to_pose|px4_imu_bridge|MicroXRCEAgent|robot_state_publisher'

mapfile -t PIDS < <(pgrep -f "$PATTERN" 2>/dev/null | grep -v -e "^$$" -e "^$PPID" || true)

if [ "${#PIDS[@]}" -eq 0 ]; then
  echo "没有匹配到正在运行的进程。"
  exit 0
fi

echo "匹配到的进程："
ps -o pid=,etime=,cmd= -p "$(IFS=,; echo "${PIDS[*]}")" || true

if [ "${1:-}" != "-y" ]; then
  read -r -p "确认停止以上 ${#PIDS[@]} 个进程？(y/N) " A
  case "$A" in y|Y|yes|YES) ;; *) echo "已取消。"; exit 0 ;; esac
fi

kill -INT "${PIDS[@]}" 2>/dev/null || true
sleep 2
STILL=$(pgrep -f "$PATTERN" 2>/dev/null | grep -v -e "^$$" -e "^$PPID" || true)
[ -n "$STILL" ] && { echo "仍有进程，发 SIGTERM:"; echo "$STILL"; kill -TERM $STILL 2>/dev/null || true; }
echo "已发送停止信号。"
