#!/usr/bin/env bash
# ============================================================================
#  链路看门狗：分段盯住「雷达 / SLAM / EV 桥 / 定高器 / IMU 桥 / Agent」，
#  哪一段坏了就**只重启那一段**（走 `run.sh restart <组件>`，和人工恢复同一条路径）。
#
#  为什么需要（2026-10-10）：这套链路出现过多种"静默失败"——进程活着、服务 active，
#  但某一环已经不干活了（雷达不发布 /scan、cartographer 崩、EV 桥停发、DDS 会话断），
#  结果整条链报废，只能靠人偶然发现。没有监督 = "突然崩掉"。
#
#  ⚠️ 安全约定：**只在能确认飞机未解锁（arming_state=1）时才动手**。
#     已解锁 → 只报警；读不到 arming_state（DDS 发现层偶发读不到）→ 也保守不动手，
#     但会提示人工确认后用 run.sh restart 处理。
#
#  判据原则：**以进程日志为地面真值，ROS 话题查询只作辅助** —— FastDDS 同机发现层
#  偶发"看得见收不到"（2026-10-10 误判过一次"雷达坏了"），只看 ros2 topic 会疯。
#
#  用法：
#     bash ~/px4_ws/tools/chain_watchdog.sh              # 前台跑
#     sudo systemctl status px4-chain-watch              # 作为服务跑时
#     可调：WATCH_INTERVAL=15 WATCH_FAILS=2 bash tools/chain_watchdog.sh
#  安装：systemd/px4-chain-watch.service（已 enable，开机自启）
# ============================================================================
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -f "$WS/tools/fastdds_udp_only.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_udp_only.xml"
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

INTERVAL="${WATCH_INTERVAL:-15}"   # 每多少秒查一轮
FAIL_NEED="${WATCH_FAILS:-2}"      # 连续几轮失败才动手
EV_LOG="${EV_LOG:-/tmp/px4_ws_ev.log}"
TOF_LOG="${TOF_LOG:-/tmp/px4_ws_tof.log}"
IMU_LOG="${IMU_LOG:-/tmp/px4_ws_imu.log}"
LIDAR_LOG="${LIDAR_LOG:-/tmp/px4_ws_lidar.log}"
SLAM_LOG="${SLAM_LOG:-/tmp/px4_ws_slam.log}"

log() { echo "[$(date +%H:%M:%S)] $*"; }

rate_of() {
  timeout "${2:-6}" ros2 topic hz "$1" 2>/dev/null | grep -m1 'average rate' || true
}
log_has()   { [ -f "$1" ] && grep -q -- "$2" "$1"; }
# 日志"新鲜度"：文件在 secs 秒内被写过（这些节点都是每 5 秒一行，很可靠）
log_fresh() { [ -f "$1" ] && [ $(( $(date +%s) - $(stat -c %Y "$1" 2>/dev/null || echo 0) )) -lt "${2:-40}" ]; }
proc_alive(){ pgrep -f "$1" > /dev/null; }
logsize()   { stat -c %s "$1" 2>/dev/null || echo 0; }
# 日志行形如：[cartographer_node-2] [INFO] [1791624266.288877695] [cartographer logger]: ...
# log_age <文件> <匹配模式> → 输出"最后一条匹配行的 ROS 时间戳 距今多少秒"（无匹配给一个大数）
log_age() {
  local f="$1" pat="$2" ts ti
  ts="$(grep -E -- "$pat" "$f" 2>/dev/null | tail -1 | grep -oE '\[[0-9]{10}\.[0-9]+\]' | tail -1 | tr -d '[]')"
  [ -n "$ts" ] || { echo 999999; return; }
  ti="${ts%%.*}"
  echo $(( $(date +%s) - ti ))
}

# 每个组件一个"连续失败"计数器
declare -A fails=( [lidar]=0 [slam]=0 [ev]=0 [tof]=0 [imu]=0 [agent]=0 )

# 健康判据（返回 0 = 健康）
healthy_lidar() {
  # 雷达：① 驱动进程在 ② 日志有握手标记 ③（DDS 查得到 /scan）或（下游 SLAM 日志新鲜＝有人收到扫描）
  # 注意：雷达健康时不写日志，所以只能用"下游在吃数据"来间接证明，不能只看文件时间。
  proc_alive "bluesea2[_]node"        || { log "[不健康] 雷达驱动进程不在"; return 1; }
  log_has "$LIDAR_LOG" 'lidar start work' || { log "[不健康] 雷达日志里没有握手标记"; return 1; }
  if [ -n "$(rate_of /scan 4)" ]; then return 0; fi
  if [ "$(log_age "$SLAM_LOG" '\[INFO\]')" -lt 60 ]; then
    log "[提示] 本进程看不到 /scan（DDS 发现层假象），但 SLAM 日志新鲜＝扫描确实在流，按健康处理"
    return 0
  fi
  log "[不健康] 既看不到 /scan，SLAM 日志也不新鲜"
  return 1
}
healthy_slam() {
  # 先查"有没有重复实例"：旧的 ros2 launch 包装带 respawn=True，只杀节点的话它会把节点
  # 再拉起来 → 两份 cartographer 打架（2026-10-10 实测：TF 抖动 + Requested submap 报错）
  # ⚠️ 必须锚定真实节点路径：`pgrep -f cartographer_node` 会把"正在看日志的人自己那条命令"
  #    （比如 pgrep/tail 带 cartographer_node 字样的 shell）也数进去 → 误判成两份实例、乱重启。
  local n
  n="$(pgrep -c -f '^/opt/ros/humble/lib/cartographer_ros/cartograph[e]r_node' 2>/dev/null || echo 0)"
  if [ "${n:-0}" -gt 1 ]; then
    log "[不健康] 有 ${n} 个 cartographer_node 实例（应该只有 1 个，多半是 launch 包装在 respawn）"
    return 1
  fi
  proc_alive "cartograph[e]r_node|slam_toolbox" || { log "[不健康] SLAM 进程不在"; return 1; }
  # 用**日志时间戳**判活：cartographer 正常时每 15~30 秒会打一行（scan rate / 约束搜索），
  # 冻住就说明它卡死了（2026-10-10 实测踩到：进程活着、CPU 18%，但输出停住、TF 不再更新）。
  local age; age="$(log_age "$SLAM_LOG" '\[INFO\]')"
  [ "$age" -lt 60 ] || { log "[不健康] SLAM 日志已经 ${age}s 没有新行了（>60s，卡死或挂了）"; return 1; }
  return 0
}
healthy_ev() {
  # EV 桥：进程在 + **最后一次 "EV: NED pos" 是近期**（它每 5 秒发一次；
  # 只看文件时间不行——位姿过期时它会不停打 ERROR，文件照样新鲜）
  proc_alive "px4[_]ev_bridge_node" || { log "[不健康] EV 桥进程不在"; return 1; }
  local age; age="$(log_age "$EV_LOG" 'EV: NED pos')"
  [ "$age" -lt 40 ] || { log "[不健康] EV 桥最后一次发 EV 是 ${age}s 前（位姿断了）"; return 1; }
  return 0
}
healthy_tof()   {
  proc_alive "tofsense[_]m_node" || { log "[不健康] 定高器进程不在"; return 1; }
  local age; age="$(log_age "$TOF_LOG" '帧')"
  [ "$age" -lt 40 ] || { log "[不健康] 定高器日志 ${age}s 没更新"; return 1; }
  return 0
}
healthy_imu()   { proc_alive "px4[_]imu_bridge" && log_fresh "$IMU_LOG" 60; }
healthy_agent() { proc_alive "MicroXRCE[A]gent"; }

restart_component() {
  local name="$1"
  log "→ 重启 $name（bash run.sh restart $name）"
  bash "$WS/run.sh" restart "$name" >> "/tmp/px4_chain_watchdog.log" 2>&1 || true
  sleep 3
  if "healthy_$name"; then
    log "✓ $name 恢复"
  else
    log "[警告] $name 重启后仍不健康；细节见 /tmp/px4_chain_watchdog.log"
  fi
}

# 能不能动手：只有明确读到 arming_state=1（未解锁）才动手。
# 唯一例外（2026-10-10 加）：**彻底读不到飞控 + IMU 桥也收不到飞控数据**时，
# 说明飞控根本没在上电/通信 —— 那就不可能在飞，此时重启组件是安全的。
# （避免"DDS 发现层偶发读不到 arming_state"把自动恢复永久锁死。）
may_act() {
  local armed="" i
  for i in 1 2 3 4; do
    armed="$(timeout 6 ros2 topic echo /fmu/out/vehicle_status --once 2>/dev/null \
             | grep -m1 '^arming_state:' | awk '{print $2}')"
    [ -n "$armed" ] && break
    sleep 2
  done
  case "$armed" in
    1) return 0 ;;    # 未解锁 → 可以恢复
    2) log "[警告] 飞机已解锁（arming_state=2）→ 只看不动，请人工处理！"; return 1 ;;
    *)
      if ! log_fresh "$IMU_LOG" 60; then
        log "[提示] 读不到 arming_state，但 IMU 桥也收不到飞控数据（至少 60 秒）→ 判定飞控没在通信，按安全处理、继续恢复"
        return 0
      fi
      log "[警告] 读不到 arming_state（读到 '$armed'，DDS 发现层偶发）→ 保守不动手；若飞机没在飞，可手动：bash $WS/run.sh restart <组件>"
      return 1
      ;;
  esac
}

log "链路看门狗启动：每 ${INTERVAL}s 查一轮，某组件连续 ${FAIL_NEED} 轮不健康才重启（解锁时只报警）"
while true; do
  sleep "$INTERVAL"
  for comp in lidar slam ev tof imu agent; do
    if "healthy_$comp"; then
      [ "${fails[$comp]}" -gt 0 ] && log "✓ $comp 恢复正常"
      fails[$comp]=0
      continue
    fi
    fails[$comp]=$(( ${fails[$comp]} + 1 ))
    log "[$comp 不健康] 连续第 ${fails[$comp]}/${FAIL_NEED} 轮"
    if [ "${fails[$comp]}" -ge "$FAIL_NEED" ]; then
      if may_act; then
        restart_component "$comp"
      fi
      fails[$comp]=0
    fi
  done
done
