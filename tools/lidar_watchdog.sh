#!/usr/bin/env bash
# ============================================================================
#  雷达看门狗：盯 /scan，连续几次没数据就重启雷达驱动
#
#  为什么需要（2026-10-10）：bluesea2 驱动这两天出现过**三种静默失败**，
#  而且进程/服务都显示 active，外表完全看不出来：
#    ① 开机 open 时报 -106（Failure to modify serial port baud rate）后卡死；
#    ② 起来了、握过手、甚至还握着串口句柄，却不发布 /scan（cartographer 扫描率日志直接断掉）；
#  任何一种都会让 SLAM/EV 整条链路一起废掉。
#
#  ⚠️ 安全约定：**飞机解锁时绝不自动重启**（半空中丢雷达 → EV 断 → 飞控进 failsafe）。
#     解锁状态下只打警告，交给人处理。
#
#  用法：
#     bash ~/px4_ws/tools/lidar_watchdog.sh                 # 前台跑（Ctrl-C 退出）
#     sudo systemctl status px4-lidar-watch                  # 作为服务跑时
#     可调：WATCH_INTERVAL=20 WATCH_FAILS=3 bash tools/lidar_watchdog.sh
#     安装见 systemd/px4-lidar-watch.service
# ============================================================================
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$WS/tools/fastdds_udp_only.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_udp_only.xml"
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

INTERVAL="${WATCH_INTERVAL:-20}"   # 每多少秒查一次
FAIL_NEED="${WATCH_FAILS:-3}"      # 连续失败几次才动手
WLOG="${WATCH_LOG:-/tmp/px4_lidar_watchdog.log}"

log() { echo "[$(date +%H:%M:%S)] $*"; }
rate() { timeout 8 ros2 topic hz /scan 2>/dev/null | grep -m1 'average rate' || true; }

log "雷达看门狗启动：每 ${INTERVAL}s 查一次 /scan，连续 ${FAIL_NEED} 次没数据就重启驱动（解锁时不重启）"
fails=0
while true; do
  sleep "$INTERVAL"

  out="$(rate)"
  if [ -n "$out" ]; then
    [ "$fails" -gt 0 ] && log "✓ 雷达已恢复：/scan $out"
    fails=0
    continue
  fi

  fails=$((fails + 1))
  log "[/scan 没数据] 连续第 ${fails}/${FAIL_NEED} 次"
  [ "$fails" -lt "$FAIL_NEED" ] && continue

  # 读 arming_state 判"能不能动手"。
  # ⚠️ 2026-10-10 修正：这个读操作自己也会被 FastDDS 同机 SHM 的坑坑到（读空），
  #    原来的写法"读不到就当已解锁"会把自动恢复**永久锁死**（实测：飞机明明是未解锁，
  #    看门狗却一直打"[警告] 飞机是解锁状态（arming_state=）→ 只看不动"）。
  #    改成：重试 3 次；只有明确读到 1(STANDBY/未解锁) 才动手；
  #    明确读到 2(ARMED) 或始终读不到 → 保守不动手，但把原因打清楚。
  armed=""
  for _ in 1 2 3; do
    armed="$(timeout 6 ros2 topic echo /fmu/out/vehicle_status --once 2>/dev/null \
             | grep -m1 '^arming_state:' | awk '{print $2}')"
    [ -n "$armed" ] && break
    sleep 2
  done
  case "$armed" in
    1) : ;;   # 明确未解锁 → 继续往下、动手恢复
    2) log "[警告] 飞机已解锁（arming_state=2）→ 只看不动，人工处理！"; fails=0; continue ;;
    *) log "[警告] 读不到 arming_state（读到 '$armed'）→ 保守起见不动手；人工确认飞机状态后重启雷达"; fails=0; continue ;;
  esac

  log "未解锁（arming_state=1）且 /scan 连续 ${FAIL_NEED} 次无数据 → 重启雷达驱动（日志 $WLOG）"
  bash "$WS/run.sh" lidar > "$WLOG" 2>&1
  out2="$(rate)"
  if [ -n "$out2" ]; then
    log "✓ 重启成功：/scan $out2"
  else
    log "[错误] 重启后仍然没有数据 —— 大概率是硬件侧（看 $WLOG；裸读验证：sudo timeout 6 cat /dev/ttyS4 | wc -c）"
  fi
  fails=0
done
