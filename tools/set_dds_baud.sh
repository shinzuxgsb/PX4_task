#!/usr/bin/env bash
# ============================================================================
#  切换「飞控 ↔ 香橙派」uXRCE-DDS 串口波特率
#
#    sudo bash ~/px4_ws/tools/set_dds_baud.sh 1500000            # 只改香橙派侧配置
#    sudo bash ~/px4_ws/tools/set_dds_baud.sh 1500000 --restart  # 改完顺便重启链路
#    bash ~/px4_ws/tools/set_dds_baud.sh status                  # 看当前配置与实测占用
#
#  为什么需要：实测 ttyS6 上「飞控 → 香橙派」方向已经 ~74 KB/s，
#  921600 8N1 上限约 90 KB/s → **占用 83%**，余量太小；
#  链路一超时 uXRCE-DDS 会话就会重建（Agent 日志 session re-established），
#  PX4 因 COM_OF_LOSS_T(1s) 收不到 offboard 流 → **掉出 Offboard**。
#  2026-10-09 晚第一次 Offboard 任务试飞炸机就是这个原因。
#
#  ⚠️ 两边必须一致！只改一边 = 完全连不上（连 DDS 都不通）。
#     飞控侧在 QGC 里改（我碰不到飞控参数）：
#         SER_TEL3_BAUD = <同一波特率>      # TELEM3 = 接香橙派的那路
#     改完 QGC 里重启飞控（或重新上电）。
#
#  香橙派 ttyS6 的支持上限是 1500000（dmesg: base_baud = 1500000），
#  所以 921600 → 1500000 后占用大约从 83% 降到 ~40%。
# ============================================================================
set -uo pipefail

CONF=/etc/default/microxrce-agent
DEV_DEFAULT=/dev/ttyS6

usage() {
  echo "用法: sudo bash $0 <波特率|status> [--restart]"
  echo "  例: sudo bash $0 1500000 --restart"
  echo "  常用值: 921600（当前）/ 1500000 / 3000000"
  exit 1
}

show_status() {
  echo "--- 香橙派侧配置（$CONF）---"
  if [ -f "$CONF" ]; then
    grep -E '^XRCE_(DEV|BAUD)=' "$CONF"
  else
    echo "(文件不存在，用内置默认 /dev/ttyS6 @921600)"
  fi

  echo "--- Agent 实际命令行（真正生效的是这个）---"
  local agent_line="" BAUD_ACTIVE="" DEV_ACTIVE=""
  agent_line="$(pgrep -af 'MicroXRCE[A]gent' || true)"
  if [ -n "$agent_line" ]; then
    echo "  $agent_line"
    BAUD_ACTIVE="$(sed -n 's/.*-b \([0-9][0-9]*\).*/\1/p' <<< "$agent_line" | head -1)"
    DEV_ACTIVE="$(sed -n 's/.*--dev \([^ ]*\).*/\1/p' <<< "$agent_line" | head -1)"
  else
    echo "  (Agent 没在跑)"
  fi
  BAUD_ACTIVE="${BAUD_ACTIVE:-921600}"
  DEV_ACTIVE="${DEV_ACTIVE:-/dev/ttyS6}"

  # 配置里写的和实际跑的如果不一致，明确点出来
  local cfg_baud=""
  [ -f "$CONF" ] && cfg_baud="$(sed -n 's/^XRCE_BAUD=\([0-9]*\).*/\1/p' "$CONF" | head -1)"
  if [ -n "$cfg_baud" ] && [ "$cfg_baud" != "$BAUD_ACTIVE" ]; then
    echo "  >>> 注意：配置写的是 $cfg_baud，但 Agent 现在跑的是 $BAUD_ACTIVE（要重启链路才生效）"
  fi

  echo "--- 实测流量（飞控→派，$DEV_ACTIVE）---"
  local idx="${DEV_ACTIVE#/dev/ttyS}"
  case "$idx" in ''|*[!0-9]*) idx=6 ;; esac
  if [ -r /proc/tty/driver/serial ]; then
    local a b rx0 rx1 tx0 tx1 d kbs limit_kb pct
    a="$(grep "^${idx}:" /proc/tty/driver/serial)"; sleep 5
    b="$(grep "^${idx}:" /proc/tty/driver/serial)"
    rx0="$(awk '{for(i=1;i<=NF;i++) if($i ~ /^rx:/){sub("rx:","",$i); print $i}}' <<< "$a")"
    rx1="$(awk '{for(i=1;i<=NF;i++) if($i ~ /^rx:/){sub("rx:","",$i); print $i}}' <<< "$b")"
    tx0="$(awk '{for(i=1;i<=NF;i++) if($i ~ /^tx:/){sub("tx:","",$i); print $i}}' <<< "$a")"
    tx1="$(awk '{for(i=1;i<=NF;i++) if($i ~ /^tx:/){sub("tx:","",$i); print $i}}' <<< "$b")"
    if [ -n "${rx0:-}" ] && [ -n "${rx1:-}" ]; then
      d=$(( rx1 - rx0 ))
      limit_kb=$(( BAUD_ACTIVE / 10 / 1024 ))
      kbs=$(( d / 5 / 1024 ))
      pct=0; [ "$limit_kb" -gt 0 ] && pct=$(( kbs * 100 / limit_kb ))
      echo "  飞控→派 ${kbs} KB/s（上限约 ${limit_kb} KB/s @${BAUD_ACTIVE}，占用 ${pct}%）；派→飞控 $(( (tx1-tx0)/5 )) B/s"
      echo "  $b"
      if [ "$d" -eq 0 ]; then
        echo "  >>> 一个字节都没收到：**两边波特率不一致** / 飞控没上电 / 线没接 / 飞控侧 UXRCE_DDS_CFG 不对"
      elif [ "$pct" -ge 70 ]; then
        echo "  >>> 占用 ${pct}%：危险区（会话一超时就掉出 Offboard）→ 提高波特率"
      else
        echo "  >>> 占用 ${pct}%：正常"
      fi
    fi
  else
    echo "  (需要 sudo 才能读 /proc/tty/driver/serial)"
  fi
}

[ $# -ge 1 ] || usage
if [ "$1" = "status" ]; then show_status; exit 0; fi

BAUD="$1"
RESTART=0
[ "${2:-}" = "--restart" ] && RESTART=1

case "$BAUD" in
  9600|57600|115200|230400|460800|500000|921600|1000000|1500000|2000000|3000000) ;;
  *) echo "[错误] 不支持的波特率：$BAUD"; usage ;;
esac

if [ "$(id -u)" != "0" ]; then
  echo "[错误] 需要 root：sudo bash $0 $*" >&2; exit 1
fi

# --- 写配置 ---
if [ ! -f "$CONF" ]; then
  printf 'XRCE_DEV=%s\nXRCE_BAUD=%s\n' "$DEV_DEFAULT" "$BAUD" > "$CONF"
  echo "已创建 $CONF"
else
  cp -a "$CONF" "$CONF.bak-$(date +%Y%m%d-%H%M%S)"
  if grep -q '^XRCE_BAUD=' "$CONF"; then
    sed -i "s#^XRCE_BAUD=.*#XRCE_BAUD=${BAUD}#" "$CONF"
  else
    echo "XRCE_BAUD=${BAUD}" >> "$CONF"
  fi
  echo "已更新 $CONF"
fi
grep -E '^XRCE_(DEV|BAUD)=' "$CONF"

cat <<EOF

============================================================
 香橙派这侧改好了。**还差飞控侧一步（必须在 QGC 里做）：**

   1) QGC → 参数（Parameters）→ 搜 SER_TEL3_BAUD
   2) 设成同一个值：${BAUD}
      （TELEM3 就是接香橙派的那路串口；UXRCE_DDS_CFG 保持指向 TELEM3）
   3) 重启飞控（QGC 里"重新启动飞控"或重新上电）

 两边不一致 = 完全连不上（连 DDS 都不通，表现为 /fmu/out/* 一个都没有）。

 验证：
   sudo bash ~/px4_ws/tools/set_dds_baud.sh status
   bash ~/px4_ws/tools/preflight.sh

 要回退：把飞控 SER_TEL3_BAUD 改回 921600，再
   sudo bash ~/px4_ws/tools/set_dds_baud.sh 921600
============================================================
EOF

if [ "$RESTART" = "1" ]; then
  echo "--- 重启链路（会重新等雷达/SLAM 约 80 秒）---"
  systemctl restart px4-ev-chain
  systemctl is-active px4-ev-chain
else
  echo "（没有加 --restart，链路要等下次重启才生效）"
fi
