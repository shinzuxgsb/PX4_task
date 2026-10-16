#!/usr/bin/env bash
# px4_ws 一键启动
#
#   bash run.sh init         只 source 环境并打印可用话题（不启动节点）
#   bash run.sh lidar        只起雷达驱动（验证 /scan）
#   bash run.sh carto        雷达 + cartographer 建图
#   bash run.sh slam         雷达 + slam_toolbox 建图
#   bash run.sh agent        只起 Micro-XRCE-DDS-Agent（PX4 ↔ 香橙派的 DDS 链路）
#   bash run.sh px4          雷达 + PX4 桥接（会先检查 Agent 在不在跑）
#   bash run.sh fly          起飞前全套（定点模式必用）：
#                            Agent(ttyS6) + 雷达 + SLAM + tf_to_pose + 定高器(ttyS3) + EV 桥
#                            默认用 cartographer（它不发 /tracked_pose，所以要 tf_to_pose 转发）；
#                            想用 slam_toolbox：bash run.sh fly slam（这条路不需要转发脚本）
#   bash run.sh status      打印当前相关进程与话题
#   bash run.sh stop        停掉本工作区拉起的进程

set -Eeo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LIDAR_PARAMS="$WS/src/bluesea2/src/bluesea-ros2/params/udp_lidar.yaml"
LIDAR_LOG="${LIDAR_LOG:-/tmp/px4_ws_lidar.log}"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[错误] 还没编译：先执行 bash $WS/build.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

start_lidar() {
  pkill -f bluesea2_node 2>/dev/null || true
  sleep 1
  nohup ros2 run bluesea2 bluesea2_node --ros-args \
    --params-file "$LIDAR_PARAMS" > "$LIDAR_LOG" 2>&1 < /dev/null &
  # 驱动启动要先跟雷达握手：日志里会有几轮 udp_talk_GS_PACK 的
  # "lidar connection abnormality" 重试，约 20 秒后才开始出数据，属正常。
  echo "雷达驱动已后台启动（日志：$LIDAR_LOG），等 25 秒完成握手..."
  sleep 25
}

stop_all() {
  pkill -f bluesea2_node     2>/dev/null || true
  pkill -f MicroXRCEAgent    2>/dev/null || true
  pkill -f cartographer      2>/dev/null || true
  pkill -f slam_toolbox      2>/dev/null || true
  pkill -f tofsense_m_node   2>/dev/null || true
  pkill -f tf_to_pose        2>/dev/null || true
  pkill -f px4_bridge_node   2>/dev/null || true
  pkill -f px4_ev_bridge_node 2>/dev/null || true
  echo "已停止本工作区的进程。"
}

# 定点模式起飞前的全套链路（全部后台起）
start_fly() {
  local slam_flavor="${1:-carto}"
  local ev_log="${EV_LOG:-/tmp/px4_ws_ev.log}"
  local slam_log="${SLAM_LOG:-/tmp/px4_ws_slam.log}"
  local tof_log="${TOF_LOG:-/tmp/px4_ws_tof.log}"
  local tf_log="${TF_LOG:-/tmp/px4_ws_tf2pose.log}"

  # [0/6] 等系统时间同步 —— 2026-10-08 事故修复
  # 香橙派开机时 RTC 时间可能是旧的（实测慢了 18h41m），几分钟后 NTP 校正会造成
  # 一次巨大的时间跳变，cartographer 遇到时间跳变会抛 std::length_error 崩掉。
  # 所以先把时钟等稳再拉起 SLAM（最多等 60 秒，超时也继续，只是打警告）。
  echo "[0/6] 等待系统时间同步（防止 NTP 校正引发时间跳变）..."
  local t_wait=0
  while [ "$t_wait" -lt 60 ]; do
    if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
      echo "      时间已同步（等待 ${t_wait}s）"
      break
    fi
    sleep 1
    t_wait=$((t_wait + 1))
  done
  if [ "$t_wait" -ge 60 ]; then
    echo "      [警告] 60 秒内未完成 NTP 同步，仍然继续启动（存在时间跳变风险）" >&2
  fi

  echo "[1/6] 起 Micro-XRCE-DDS-Agent（ttyS6 = 飞控 DDS）..."
  bash "$WS/agent.sh" start

  echo "[2/6] 起雷达（要等 ~25 秒握手）..."
  start_lidar

  echo "[3/6] 起 SLAM（$slam_flavor）..."
  pkill -f cartographer 2>/dev/null || true
  pkill -f slam_toolbox 2>/dev/null || true
  if [ "$slam_flavor" = "slam" ]; then
    nohup ros2 launch slam_pkg slam.launch.py \
      > "$slam_log" 2>&1 < /dev/null &
  else
    nohup ros2 launch carto_pkg fly_carto.launch.py use_rviz:=false \
      > "$slam_log" 2>&1 < /dev/null &
  fi
  echo "     SLAM 日志：$slam_log；等 15 秒让它出第一帧位姿..."
  sleep 15

  if [ "$slam_flavor" = "slam" ]; then
    echo "[4/6] slam_toolbox 直接发 /pose，跳过 tf_to_pose"
  else
    echo "[4/6] 起 tf_to_pose.py（这版 cartographer 不发 /tracked_pose，必须转发）..."
    pkill -f tf_to_pose 2>/dev/null || true
    if [ -f "$HOME/tf_to_pose.py" ]; then
      nohup python3 "$HOME/tf_to_pose.py" > "$tf_log" 2>&1 < /dev/null &
      echo "     日志：$tf_log"
    else
      echo "     [错误] 找不到 $HOME/tf_to_pose.py —— EV 桥拿不到位姿！" >&2
    fi
  fi

  echo "[5/6] 起定高器 tofsense_m（/dev/ttyS3）..."
  pkill -f tofsense_m_node 2>/dev/null || true
  sleep 1
  nohup ros2 launch tofsense_m tofsense_m.launch.py \
    > "$tof_log" 2>&1 < /dev/null &
  echo "     日志：$tof_log"

  echo "[6/6] 起 EV 桥（定点模式的位姿/高度就靠它）..."
  pkill -f px4_ev_bridge_node 2>/dev/null || true
  sleep 1
  nohup ros2 launch px4_ev_bridge ev_bridge.launch.py \
    > "$ev_log" 2>&1 < /dev/null &
  sleep 3

  echo "--- 进程 ---"
  pgrep -af 'MicroXRCEAgent|bluesea2_node|cartographer|slam_toolbox|tf_to_pose|tofsense_m_node|px4_ev_bridge_node' || echo "(无)"
  echo "--- 关键话题频率 ---"
  for t in /scan /tofsense/height /tracked_pose /fmu/in/vehicle_visual_odometry; do
    printf '%-38s ' "$t"
    out="$(timeout 4 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' || true)"
    if [ -n "$out" ]; then echo "$out"; else echo "(无数据)"; fi
  done
  echo "--- EV 桥日志尾部 ---"
  tail -n 5 "$ev_log"
  echo
  echo "下一步（在飞控端 MAVLink 控制台）确认："
  echo "  listener vehicle_visual_odometry     # 应有数据，不是 never published"
  echo "  listener vehicle_local_position      # xy_valid / z_valid 要是 true"
  echo "两个都对上了，再解锁起飞。EV 中断会在 400ms 内触发位置失效，别在没起来时解锁。"
}

case "${1:-}" in
  init)
    echo "环境已就绪（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）"
    ros2 topic list
    ;;
  lidar)
    start_lidar
    echo "--- /scan 频率 ---"
    timeout 10 ros2 topic hz /scan || true
    ;;
  carto)
    start_lidar
    echo "启动 cartographer（Ctrl-C 退出）..."
    exec ros2 launch carto_pkg fly_carto.launch.py use_rviz:=false
    ;;
  slam)
    start_lidar
    echo "启动 slam_toolbox（Ctrl-C 退出）..."
    exec ros2 launch slam_pkg slam.launch.py
    ;;
  px4)
    start_lidar
    if ! pgrep -f MicroXRCEAgent > /dev/null; then
      echo "[提示] Micro-XRCE-DDS-Agent 没在跑，先执行： bash $WS/agent.sh start"
      echo "       （或 bash run.sh agent）"
    fi
    echo "启动 PX4 桥接..."
    exec ros2 launch px4_bridge px4_bridge.launch.py
    ;;
  agent)
    exec bash "$WS/agent.sh" start
    ;;
  fly)
    start_fly "${2:-carto}"
    ;;
  status)
    echo "--- 进程 ---"
    pgrep -af 'bluesea2_node|cartographer|slam_toolbox|px4_bridge_node' || echo "(无)"
    echo "--- 话题 ---"
    ros2 topic list 2>/dev/null || true
    ;;
  stop)
    stop_all
    ;;
  *)
    echo "用法: bash run.sh {init|lidar|carto|slam|agent|px4|fly|status|stop}"
    ;;
esac
