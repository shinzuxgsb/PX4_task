#!/usr/bin/env bash
# 在香橙派 5 Max 上启用 UART4（/dev/ttyS4），供串口雷达 LDS-50C-R 使用。
#
# 背景
#   rk3588 基准设备树里 serial@feb70000(UART4) 是 status="disabled"，
#   需要在 u-boot 阶段叠加 overlay rk3588-uart4-m2.dtbo 才会出现 /dev/ttyS4。
#   （40pin 排针：19 = TXD = GPIO1_B2，23 = RXD = GPIO1_B3，pinmux=UART4_M2）
#   本机 u-boot 的 overlay 列表在 /etc/default/u-boot 的 U_BOOT_FDT_OVERLAYS。
#
# 用法（在香橙派上执行，需要 sudo / root）
#   sudo bash tools/enable_ttyS4.sh            # 写配置 + u-boot-update，提示重启
#   sudo bash tools/enable_ttyS4.sh --reboot   # 顺便重启
#
# 幂等：重复执行不会重复添加。
set -euo pipefail

OV_REL="device-tree/rockchip/overlay/rk3588-uart4-m2.dtbo"
CONF="/etc/default/u-boot"
REBOOT=0
[ "${1:-}" = "--reboot" ] && REBOOT=1

if [ "$(id -u)" != "0" ]; then
  echo "[错误] 需要 root：sudo bash $0" >&2
  exit 1
fi

# 找到实际存在的 overlay 文件（内核版本目录可能变）
OV_FILE="$(ls -1 /lib/firmware/*/"$OV_REL" 2>/dev/null | head -n1 || true)"
if [ -z "$OV_FILE" ]; then
  echo "[错误] 找不到 overlay $OV_REL" >&2
  echo "可用 uart4 overlay：" >&2
  ls /lib/firmware/*/device-tree/rockchip/overlay/ 2>/dev/null | grep -i uart4 >&2 || true
  exit 1
fi
echo "overlay：$OV_FILE"

if [ ! -f "$CONF" ]; then
  echo "[错误] 没有 $CONF，本机可能不是 u-boot-menu/extlinux 引导" >&2
  exit 1
fi

if [ ! -f "$CONF.bak-ttyS4" ]; then
  cp -a "$CONF" "$CONF.bak-ttyS4"
  echo "已备份：$CONF.bak-ttyS4"
fi

if grep -q "rk3588-uart4-m2.dtbo" "$CONF"; then
  echo "配置里已包含 uart4-m2，跳过写入。"
else
  python3 - "$CONF" "$OV_REL" <<'PY'
import re, sys
conf, ov = sys.argv[1], sys.argv[2]
s = open(conf, encoding='utf-8').read()
m = re.search(r'^U_BOOT_FDT_OVERLAYS="([^"]*)"', s, re.M)
if not m:
    sys.stderr.write("!! 没找到 U_BOOT_FDT_OVERLAYS 行，请手工编辑 %s\n" % conf)
    sys.exit(2)
cur = m.group(1).strip()
s = s[:m.start()] + 'U_BOOT_FDT_OVERLAYS="%s"' % ((cur + " " + ov).strip()) + s[m.end():]
open(conf, 'w', encoding='utf-8').write(s)
print("已写入 U_BOOT_FDT_OVERLAYS")
PY
fi

echo "--- 当前配置 ---"
grep -n "U_BOOT_FDT_OVERLAYS" "$CONF"

echo "--- 重新生成 /boot/extlinux/extlinux.conf ---"
u-boot-update

echo
echo "完成。重启后才会出现 /dev/ttyS4："
echo "  ls -l /dev/ttyS4          # 应看到 crw-rw---- root dialout"
echo "  bash run.sh lidar         # 起雷达，检查 /scan"

if [ "$REBOOT" = "1" ]; then
  echo "3 秒后重启…"
  sleep 3
  reboot
fi
