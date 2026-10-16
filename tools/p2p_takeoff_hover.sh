#!/usr/bin/env bash
# ============================================================================
#  p2p_takeoff_hover.sh —— "0.5 m 起飞 + 悬停 + 遥控接管降落" 专用一键脚本
#  它做：链路自检 → 提示你在遥控器上做什么 → 实时盯 /p_to_p/state
#  它不做：解锁 / 切模式 / 发控制命令 —— 这几步**故意留给你在遥控器上做**
#          （本栈设计上 allow_mode_command=false，程序不抢飞控的控制权）
#  用法：bash ~/px4_ws/tools/p2p_takeoff_hover.sh
# ============================================================================
set -uo pipefail
WS="$HOME/px4_ws"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_udp_only.xml"

fail(){ echo "X  $*"; exit 1; }
ok(){ echo "OK $*"; }
rate(){ timeout 6 ros2 topic hz "$1" 2>/dev/null | sed -n '1p'; }

echo "== [1/5] 链路 =="
systemctl is-active --quiet px4-p2p-chain && ok "px4-p2p-chain active" || fail "服务没在跑：sudo systemctl restart px4-p2p-chain"
pgrep -af 'final_position.launch' >/dev/null && ok "整链已在跑" || echo "  注意：没看到 final_position.launch（pos 模式只起感知链？）"

echo "== [2/5] 感知频率 =="
echo "  /scan                            $(rate /scan)"
echo "  /tofsense/height                 $(rate /tofsense/height)"
echo "  /fmu/in/vehicle_visual_odometry  $(rate /fmu/in/vehicle_visual_odometry)"

echo "== [3/5] 任务就绪 =="
timeout 5 ros2 service call /p_to_p/readiness_check std_srvs/srv/Trigger {} 2>&1 | tail -1
timeout 5 ros2 topic echo /p_to_p/state --once 2>&1 | sed -n '1,2p'
echo "  （期望：IDLE: 等待定位、测高与人工启动）"

echo "== [4/5] 桥是否已发流（此刻应为空=正常）=="
timeout 5 ros2 topic hz /fmu/in/trajectory_setpoint 2>&1 | sed -n '1p'

echo "== [5/5] 现在轮到你（遥控器）=="
cat <<'TIP'
  1) 遥控器解锁：油门最低 + 摇杆解锁手势
  2) 模式开关 → 槽位 4 = Offboard      ← 任务会自动开始（预发流 1 s 后）
     若 3 秒没动静：ros2 service call /p_to_p/start std_srvs/srv/Trigger {}
  3) 盯下面输出：应看到 IDLE → TAKEOFF → GOTO_B → HOLD_B，飞机升到 0.5 m
     另开一个窗口可看：ros2 topic hz /fmu/in/trajectory_setpoint   （应 ~20 Hz）
  4) 悬停满 10 秒 → 模式开关拨回 槽位 6 = Position → 自己摇杆降落 → 上锁
  异常处置（记这一条就够）：
     · 立刻把模式开关拨回 Position(槽位6) 或 Stabilized(槽位1) —— 桥会自动停发流
     · 或 ros2 service call /p_to_p/stop_output std_srvs/srv/Trigger {}
TIP
echo "== 状态流（Ctrl-C 退出）=="
exec ros2 topic echo /p_to_p/state