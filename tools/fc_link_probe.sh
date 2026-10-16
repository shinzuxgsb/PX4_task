#!/usr/bin/env bash
# 探测「飞控 <-> 香橙派」那一路串口到底有没有信号（不依赖 DDS，直接看线）。
#
#   bash ~/px4_ws/tools/fc_link_probe.sh            # 默认探 /dev/ttyS6
#   bash ~/px4_ws/tools/fc_link_probe.sh /dev/ttyS1
#
# 做法：临时停掉 MicroXRCEAgent -> 逐个波特率裸读 10 秒 -> 打印收到的字节数和前几个字节
#       -> 自动把 agent 恢复。
#
# 判读：
#   收到字节（不是全 0）        -> 线是通的、飞控在发；问题在波特率或 DDS 配置
#   一个字节都没有              -> 飞控侧完全没在驱动这根线：没上电 / UXRCE_DDS_CFG=0 /
#                                  线接错（尤其 TX-RX 没交叉）/ 接的不是这个口
set -uo pipefail

DEV="${1:-/dev/ttyS6}"
# agent 恢复时**必须**回到飞控 DDS 那一路（默认 ttyS6），不能跟着 DEV 走 ——
# 否则探完 ttyS1 会把 agent 起错串口。
DDS_DEV="${DDS_DEV:-/dev/ttyS6}"
DDS_BAUD="${DDS_BAUD:-921600}"
BIN=/usr/local/bin/MicroXRCEAgent
LOG=/tmp/microxrce_agent.log

echo "探测串口：$DEV"
[ -e "$DEV" ] || { echo "[错误] $DEV 不存在"; exit 1; }

rxc() { sudo -n cat /proc/tty/driver/serial 2>/dev/null | grep -E "^${DEV#/dev/ttyS}:" || true; }

echo "--- 探测前驱动计数 ---"; rxc

echo "--- 停 agent（释放串口）---"
pkill -f MicroXRCEAgent 2>/dev/null || true
sleep 2

for b in 921600 57600 115200 230400; do
  stty -F "$DEV" "$b" cs8 -cstopb -parenb -crtscts raw -echo 2>/dev/null
  timeout 10 cat "$DEV" > "/tmp/fcprobe_$b.bin" 2>/dev/null
  n=$(stat -c %s "/tmp/fcprobe_$b.bin" 2>/dev/null || echo 0)
  printf '  %-8s : %6s 字节' "$b" "$n"
  if [ "$n" -gt 0 ]; then
    printf '   头几个: %s' "$(xxd -p -l 12 "/tmp/fcprobe_$b.bin")"
  fi
  echo
done

echo "--- 探测后驱动计数 ---"; rxc

echo "--- 恢复 agent ---"
nohup "$BIN" serial --dev "$DDS_DEV" -b "$DDS_BAUD" -v 4 > "$LOG" 2>&1 < /dev/null &
sleep 3
pgrep -af MicroXRCEAgent && echo "(已恢复到 $DDS_DEV @ $DDS_BAUD)" || echo "(!! 恢复失败)"

echo
echo "提示：若全为 0，请确认——飞控已上电；UXRCE_DDS_CFG 指向 TELEM3 对应的口；"
echo "      接线为 飞控TX->派RX、飞控RX->派TX、GND 共地（TX/RX 接反是最常见的）。"
