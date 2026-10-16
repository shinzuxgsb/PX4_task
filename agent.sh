#!/usr/bin/env bash
# Micro-XRCE-DDS-Agent 一键启停（香橙派侧）
#
#   bash agent.sh start    后台启动
#   bash agent.sh stop     停止
#   bash agent.sh status   看进程 + 相关话题
#   bash agent.sh detect   看哪路串口在收数据 -> 判断飞控接在哪个 UART（需要 sudo）
#
# 端口/波特率可用环境变量覆盖：
#   XRCE_DEV=/dev/ttyS3 XRCE_BAUD=921600 bash agent.sh start
#
# 注意：飞控侧必须同时设 UXRCE_DDS_CFG = 接香橙派的那路串口，否则飞控根本不会发数据。
set -uo pipefail

# 飞控 DDS 走 ttyS6（UA5/TELEM3）——见 CHANGELOG_2026-10-07_垂直起降链路.md §6
# ⚠️ ttyS3 是定高器 TOFsense-M 的口，占用了高度就没了！
#
# 2026-10-09：端口/波特率改成"配置文件优先，环境变量最高"：
#   配置文件 /etc/default/microxrce-agent（用 tools/set_dds_baud.sh 改，带 sudo）
#   临时覆盖  XRCE_DEV=... XRCE_BAUD=... bash agent.sh start
# 为什么关心波特率：飞控→派方向实测已占 921600 链路 83%，余量太小，
# 会话一超时就重建 → PX4 掉出 Offboard（见 CHANGELOG 2026-10-09 炸机复盘）。
_ENV_DEV="${XRCE_DEV:-}"; _ENV_BAUD="${XRCE_BAUD:-}"
[ -f /etc/default/microxrce-agent ] && . /etc/default/microxrce-agent
XRCE_DEV="${_ENV_DEV:-${XRCE_DEV:-/dev/ttyS6}}"
XRCE_BAUD="${_ENV_BAUD:-${XRCE_BAUD:-921600}}"
LOG="${XRCE_LOG:-/tmp/microxrce_agent.log}"
BIN="${XRCE_BIN:-/usr/local/bin/MicroXRCEAgent}"

start_agent() {
  if [ ! -x "$BIN" ]; then
    echo "[错误] 找不到 $BIN" >&2
    exit 1
  fi
  if [ ! -e "$XRCE_DEV" ]; then
    echo "[错误] $XRCE_DEV 不存在。现有串口：" >&2
    ls -l /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null >&2
    exit 1
  fi
  pkill -f MicroXRCEAgent 2>/dev/null && sleep 1
  echo "启动 Agent：$XRCE_DEV @ $XRCE_BAUD（日志 $LOG）"
  nohup "$BIN" serial --dev "$XRCE_DEV" -b "$XRCE_BAUD" \
    > "$LOG" 2>&1 < /dev/null &
  sleep 3
  if pgrep -f MicroXRCEAgent > /dev/null; then
    echo "已启动。"
    tail -n 5 "$LOG"
  else
    echo "[错误] 没起来，日志尾部：" >&2
    tail -n 20 "$LOG" >&2
    exit 1
  fi
}

stop_agent() {
  pkill -f MicroXRCEAgent 2>/dev/null && echo "已停止。" || echo "本来就没在跑。"
}

status_agent() {
  echo "--- Agent 进程 ---"
  pgrep -af MicroXRCEAgent || echo "(没在跑)"
  echo "--- 日志尾部 ---"
  [ -f "$LOG" ] && tail -n 10 "$LOG" || echo "(无日志)"
  echo "--- 串口占用 ---"
  command -v lsof > /dev/null && lsof "$XRCE_DEV" 2>/dev/null || echo "(未检查)"
}

# 对比两次 /proc/tty/driver/serial 的 rx 计数，看哪一路在收数据
detect_port() {
  local f=/proc/tty/driver/serial
  if [ ! -r "$f" ]; then
    echo "[错误] 读不了 $f，请用 sudo 运行：sudo bash $0 detect" >&2
    exit 1
  fi
  echo "第一次采样..."; local a; a="$(cat $f)"; sleep 3
  echo "第二次采样..." ; local b; b="$(cat $f)"
  echo "--- rx 计数变化（有变化的 = 有设备在发数据）---"
  while read -r line; do
    local idx; idx="$(echo "$line" | cut -d: -f1)"
    local ra rb
    ra="$(echo "$a" | awk -F'[ :]' -v i="$idx" '$1==i {for(j=1;j<=NF;j++) if($j=="rx") print $(j+1)}')"
    rb="$(echo "$b" | awk -F'[ :]' -v i="$idx" '$1==i {for(j=1;j<=NF;j++) if($j=="rx") print $(j+1)}')"
    if [ -n "$ra" ] && [ -n "$rb" ] && [ "$ra" != "$rb" ]; then
      echo "  ttyS$idx : rx $ra -> $rb  <== 有数据！"
    fi
  done <<< "$b"
  echo "（若全都没变化：飞控没接线 / 没上电 / 飞控侧 UXRCE_DDS_CFG 没配）"
  echo "已知占用：ttyS3=定高器 TOFSense-M，ttyS6=飞控 DDS，ttyS7=板载蓝牙。"
}

case "${1:-status}" in
  start)  start_agent ;;
  stop)   stop_agent ;;
  status) status_agent ;;
  detect) detect_port ;;
  *) echo "用法: bash $0 {start|stop|status|detect}"; exit 1 ;;
esac
