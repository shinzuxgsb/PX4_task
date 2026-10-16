#!/usr/bin/env bash
# =====================================================================
#  任务.sh —— 一键跑小任务航线
#      默认（2026-10-10 深夜起）：起飞 0.5m → 沿起飞时刻机头方向前进 1.2m
#                                  → 悬停 5s → 交还 PX4 自动降落
#      **不掉头、不返航**（炸机复盘后先把"掉头"这段拿掉；要恢复用 RETURN=1 + 第 3 个参数）
#
#  2026-10-10 建立。为什么不直接手敲 `ros2 launch`：
#    手敲命令的那个交互 shell 里**没有** FASTRTPS_DEFAULT_PROFILES_FILE，
#    会踩 FastDDS 同机共享内存的坑（话题看得见、数据收不到/停住）。
#    本脚本统一负责：环境 → 强制 UDP → 起飞前自检 → 跑任务 → 告诉你日志在哪。
#
#  用法（在香橙派上）
#    bash ~/px4_ws/tools/mission.sh                  # 默认 0.5m 高 / 前进 1.2m / 不掉头 / 悬停 5s
#    bash ~/px4_ws/tools/mission.sh 0.8              # 只改起飞高度
#    bash ~/px4_ws/tools/mission.sh 0.8 0.3 0 5      # 高度 前进 掉头角 悬停秒
#    RETURN=1 bash ~/px4_ws/tools/mission.sh 0.5 0.5 180 5   # 老任务：前进→掉头180°→飞回原点
#    DRY=1   bash ~/px4_ws/tools/mission.sh          # 干跑：只发 Offboard 流，不解锁
#    FORCE=1 bash ~/px4_ws/tools/mission.sh          # 自检没过也硬起飞（不推荐）
#    SKIP_PREFLIGHT=1 bash ~/px4_ws/tools/mission.sh # 跳过自检
#
#  位置参数顺序： target_alt  forward_dist  turn_deg  hover_time
#
#  ⚠️ 这里**不能**写 `set -u`：`source /opt/ros/humble/setup.bash` 里有未定义变量，
#     在 set -u 下非交互 shell 会直接退出（2026-10-10 踩过：脚本刚打印完 1/4 就没了）。
# =====================================================================
set +u

ALT="${1:-0.5}"
FWD="${2:-1.2}"
TURN="${3:-0}"
HOVER="${4:-5.0}"
DRY="${DRY:-0}"
RETURN="${RETURN:-0}"      # 1 = 掉头后飞回起飞点正上方（老任务形态）
FORCE="${FORCE:-0}"
SKIP_PREFLIGHT="${SKIP_PREFLIGHT:-0}"

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="$WS/tools/fastdds_udp_only.xml"

echo "===== 1/4 环境 ====="
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
if [ -f "$PROFILE" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PROFILE"
  echo "  FastDDS 传输：强制 UDP（$PROFILE）"
else
  echo "  [!!] 找不到 $PROFILE —— 继续跑，但可能踩共享内存的坑"
fi
echo "  工作区：$WS"

echo "===== 2/4 起飞前自检 ====="
if [ "$SKIP_PREFLIGHT" = "1" ]; then
  echo "  （SKIP_PREFLIGHT=1 已跳过）"
else
  pf_out="$(bash "$WS/tools/preflight.sh" 2>&1)"
  echo "$pf_out" | tail -n 32
  if echo "$pf_out" | grep -q "全部通过"; then
    echo "  → 自检通过"
  else
    echo "  → 自检没过（见上面 [NG] 项）"
    if [ "$FORCE" != "1" ]; then
      echo "  已中止。确认还要飞就加 FORCE=1 重跑。"
      exit 1
    fi
    echo "  FORCE=1：继续。"
  fi
fi

echo "===== 3/4 任务参数 ====="
echo "  目标高度 ${ALT} m（相对起飞点）"
echo "  前进 ${FWD} m（沿起飞时刻机头方向）"
echo "  掉头 ${TURN} °"
echo "  悬停 ${HOVER} s"
if [ "$RETURN" = "1" ]; then echo "  掉头后：飞回起飞点正上方"; else echo "  掉头后：就地悬停降落（不返航）"; fi
if [ "$DRY" = "1" ]; then
  echo "  模式：干跑（dry_run:=true，不解锁）"
  DRY_ARG="true"
else
  echo "  模式：真飞"
  DRY_ARG="false"
fi
if [ "$RETURN" = "1" ]; then RETURN_ARG="true"; else RETURN_ARG="false"; fi

if [ "$DRY" != "1" ] && [ -t 0 ]; then
  printf '  确认起飞？输入 yes 回车：'
  read -r ans
  if [ "$ans" != "yes" ]; then
    echo "  已取消。"
    exit 1
  fi
fi

echo "===== 4/4 启动任务节点 ====="
echo "  看状态：另一个窗口 ros2 topic echo /mission_state"
echo "  节点日志：~/.ros/log/px4_takeoff_land_node_*.log（取最新的那个）"
echo "  发生异常后：sudo journalctl -u px4-chain-watch --no-pager | tail -30"
echo "  中止/链路提示：grep -E 'session re-established' /tmp/microxrce_agent.log"
echo

exec ros2 launch px4_takeoff_land takeoff_land.launch.py \
  target_alt:="$ALT" hover_time:="$HOVER" \
  mission_enable:=true forward_dist:="$FWD" turn_deg:="$TURN" \
  return_home:="$RETURN_ARG" dry_run:="$DRY_ARG"
