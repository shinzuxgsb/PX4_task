#!/usr/bin/env bash
# 起飞前自检：确认「跑 px4_takeoff_land 小任务航线」的前置条件是否齐备。
#
#   bash ~/px4_ws/tools/preflight.sh
#
# 只读：不启动、不控制、不发任何指令。
#
# 检查项来自 px4_takeoff_land_node.cpp 的 WaitReady 阶段：
#   have_status_(/fmu/out/vehicle_status) && !armed &&
#   localPosValid()(xy_valid && z_valid && v_xy_valid && v_z_valid) && heading 有限
set +e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# 2026-10-10 追加：新开的会话常出现「ros2 topic list 看得见话题、hz/echo 却收不到数据」——
# FastDDS 同机共享内存传输在多次重启后建不起数据通路（发现是好的；大消息如 /scan 尤其明显）。
# 本进程强制只用 UDP 就能收到，否则第 2 节会把 /scan 误判成"无数据"。
if [ -f "$WS/tools/fastdds_udp_only.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_udp_only.xml"
fi

OK=0; NG=0
ok()  { printf '  \033[32m[OK]\033[0m %s\n' "$1"; OK=$((OK+1)); }
ng()  { printf '  \033[31m[NG]\033[0m %s\n' "$1"; NG=$((NG+1)); }
info(){ printf '  \033[33m[!!]\033[0m %s\n' "$1"; }
hr()  { printf '\n\033[1m== %s ==\033[0m\n' "$1"; }
has_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }
field() {  # field <topic> <key>
  timeout 6 ros2 topic echo "$1" --once 2>/dev/null | grep -m1 -E "^${2}:" | awk '{print $2}'
}

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID   工作区=$WS"

# 2026-10-13：链路已切到"参考件方案"（见 docs/CHANGELOG.md 补记 9/10）：
#   进程名换成 p_to_p 那套；并且**必须**确认旧的 px4_ev_bridge 没在跑（EV 只允许一个发布者）。
hr "1. 链路进程（缺了就先 sudo systemctl start px4-p2p-chain）"
# 2026-10-14：链路只有一套启动形态（纯感知链，见 run_p2p.sh 头部说明）⇒ 任务状态机与
# Offboard 桥**本来就不该在跑**（它们在 = 有人手敲了参考件整链，会接管飞机），
# 所以这里一律按"预期如此"报告，不再按模式分支（原来靠 start_offboard:=false 判 pos 模式）。
for p in MicroXRCEAgent bluesea2_node cartographer_node tofsense_m_node \
         cartographer_odometry_bridge; do
  if pgrep -f "$p" >/dev/null; then ok "$p 在跑"; else ng "$p 没在跑"; fi
done
info "p_to_p_mission_node 未运行 —— 预期如此（本机只有感知链，任务状态机不启动）"
info "px4_offboard_bridge 未运行 —— 预期如此（不发 /fmu/in/offboard_*）"
if pgrep -f p_to_p_mission_node >/dev/null || pgrep -f px4_offboard_bridge >/dev/null; then
  ng "任务状态机 / Offboard 桥在跑 —— 不该有！切到 Offboard 后它们会直接接管飞机（A→B 自动飞）。
       先 pkill 掉，或 bash ~/px4_ws/run_p2p.sh stop 再 sudo systemctl restart px4-p2p-chain"
fi
if pgrep -f px4_ev_bridge_node >/dev/null; then
  ng "px4_ev_bridge 也在跑 —— 出现两个 EV 发布者（红线！）先 sudo systemctl stop px4-ev-chain"
else
  ok "没有第二份 EV 发布者（旧 px4_ev_bridge 未运行）"
fi

hr "2. 雷达 / SLAM / 定高 数据"
if timeout 6 ros2 topic hz /scan 2>/dev/null | grep -qm1 'average rate'; then ok "/scan 有数据"; else ng "/scan 无数据"; fi
# 2026-10-13：新方案不用 /tracked_pose（tf_to_pose 已退役），EV 桥直接读 TF odom→base_link。
if timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | grep -qm1 'Translation'; then
  ok "TF odom→base_link 存在（新 EV 桥的位姿源）"
else
  ng "TF odom→base_link 查不到（cartographer 起了吗？）"
fi
if timeout 6 ros2 topic hz /tofsense/height 2>/dev/null | grep -qm1 'average rate'; then ok "/tofsense/height 有数据"; else ng "/tofsense/height 无数据"; fi

# EV 报文契约（新方案：位置+航向+**高度**都靠它；FRD/低 quality/NaN 高度都会让飞控吃不到）
ev=$(timeout 8 ros2 topic echo /fmu/in/vehicle_visual_odometry --once 2>/dev/null)
pf=$(echo "$ev" | grep -m1 '^pose_frame:' | awk '{print $2}')
ql=$(echo "$ev" | grep -m1 '^quality:' | awk '{print $2}')
evz=$(echo "$ev" | awk '/^ *position:/{getline; getline; getline; print $2; exit}')
case "$pf" in
  1) ok "EV pose_frame=1(NED)" ;;
  2) ng "EV pose_frame=2(FRD) —— FRD 会让 PX4 把 yaw_align 置 false（heading_good_for_control=false）；本机该用 pose_frame:=ned" ;;
  "") ng "读不到 EV 报文（EV 桥在跑吗？）" ;;
  *) ng "EV pose_frame=$pf 异常" ;;
esac
if [ -n "$ql" ] && [ "$ql" -ge 50 ] 2>/dev/null; then
  ok "EV quality=$ql（PX4 的质量门 EKF2_EV_QUAL_MIN 默认 50）"
else
  ng "EV quality=${ql:-空} < 50 ⇒ PX4 会拒收 / 停掉 EV 融合"
fi
case "$evz" in
  ""|".nan"|"nan") ng "EV position.z 是 NaN ⇒ 高度没进 EV（查 height_topic / tofsense_m）" ;;
  *) ok "EV position.z=$evz（本机定高已并进 EV）" ;;
esac

hr "3. 飞控 DDS 链路（最关键）"
if has_topic /fmu/out/vehicle_status; then ok "/fmu/out/* 已出现（飞控在线）"; else ng "没有 /fmu/out/*，DDS 没通" ; fi
sub=$(timeout 10 ros2 topic info /fmu/in/vehicle_visual_odometry 2>/dev/null | grep -m1 'Subscription count' | awk '{print $3}')
if [ "${sub:-0}" -ge 1 ] 2>/dev/null; then ok "EV 订阅者 = $sub（报文已送出香橙派）"; else ng "EV 订阅者 = ${sub:-0}（报文没送出，EKF 拿不到雷达）"; fi
# 2026-10-15：**EV 发布者必须恰好 1 个**（项目红线）。>1 = 有重复实例（残留的旧链没杀干净），
#   飞控会收到来自不同地图原点的位置 → 冻住/跳变/漂 → 炸机（17:19 实测就是这么炸的）。
pub=$(timeout 10 ros2 topic info /fmu/in/vehicle_visual_odometry 2>/dev/null | grep -m1 'Publisher count' | awk '{print $3}')
if [ "${pub:-0}" -eq 1 ] 2>/dev/null; then
  ok "EV 发布者 = 1（红线：只能有一个）"
else
  ng "EV 发布者 = ${pub:-0} —— **必须恰好 1 个**！>1 说明有重复实例在同时发位置，
       飞控会收到两个地图原点的坐标 ⇒ 位置冻住/漂移 ⇒ 炸机。
       先清干净：sudo systemctl restart px4-p2p-chain，再 pgrep -af 'cartograph[e]r_node' 确认只剩一个。"
fi

hr "4. EKF 是否真在用雷达（4 个必须全 true）"
if has_topic /fmu/out/estimator_status_flags; then
  f=$(timeout 6 ros2 topic echo /fmu/out/estimator_status_flags --once 2>/dev/null)
  # 2026-10-10 修正：cs_ev_vel 是否该为 true 取决于 EV 桥的 report_velocity ——
  # 现在 EV 桥默认 report_velocity=false（保守配置，日志里打「速度不上报(交给飞控IMU)」），
  # 那种配置下 cs_ev_vel 必然是 false，不该算 NG。
  ev_vel_reported=true
  grep -q '速度不上报' /tmp/px4_ws_ev.log 2>/dev/null && ev_vel_reported=false
  for k in cs_ev_pos cs_ev_yaw cs_ev_hgt; do
    if echo "$f" | grep -qE "^${k}: true"; then ok "$k = true"; else ng "$k != true"; fi
  done
  if echo "$f" | grep -qE '^cs_ev_vel: true'; then
    ok "cs_ev_vel = true"
  else
    # 2026-10-13：新桥（cartographer_odometry_bridge）发的是 [vx, vy, NaN]，而 PX4 1.15 的
    # EKF2 要求速度三分量全为有限值才融合 ⇒ 这里必然是 false。属预期，不是故障：
    # 速度交给飞控自己的 IMU。（想真融合速度需要参考件那份定制固件。）
    info "cs_ev_vel = false（新 EV 桥发 [vx,vy,NaN]；stock 固件要求三分量全有限 ⇒ 不融合速度，符合预期）"
  fi
else
  ng "拿不到 estimator_status_flags"
fi

hr "5. 本地位置估计有效性（localPosValid 的四项）"
if has_topic /fmu/out/vehicle_local_position; then
  lp=$(timeout 6 ros2 topic echo /fmu/out/vehicle_local_position --once 2>/dev/null)
  for k in xy_valid z_valid v_xy_valid v_z_valid; do
    if echo "$lp" | grep -qE "^${k}: true"; then ok "$k = true"; else ng "$k != true"; fi
  done
  hd=$(echo "$lp" | grep -m1 '^heading:' | awk '{print $2}')
  if [ -n "$hd" ] && [ "$hd" != ".nan" ]; then ok "heading = $hd"; else ng "heading 无效（任务段的『前进方向』要用它）"; fi
else
  ng "拿不到 vehicle_local_position"
fi

hr "6. 解锁条件"
vs=$(timeout 6 ros2 topic echo /fmu/out/vehicle_status --once 2>/dev/null)
# 2026-10-13：顺手把 failsafe_flags 抓下来（第 6/7 节都要用）
vsf=$(timeout 6 ros2 topic echo /fmu/out/failsafe_flags --once 2>/dev/null)
ns=$(echo "$vs" | grep -m1 '^nav_state:' | awk '{print $2}')
case "$ns" in
  14) info "nav_state=14 飞控停在 OFFBOARD 模式。
       这不一定是问题：只有『完全没有 offboard 流』时才会让解锁检查不过。
       takeoff 节点会先自己发 2s offboard 流再解锁，所以它能自解。已实测：
       只要发 /fmu/in/offboard_control_mode，pre_flight_checks_pass 就会变 true。" ;;
  13) ng "nav_state=13 TERMINATION（飞行终止）—— 必须重启飞控才能恢复" ;;
  "") ng "读不到 nav_state（DDS 没通？）" ;;
  *)  ok "nav_state=$ns（不在 Offboard）" ;;
esac
if echo "$vs" | grep -qE '^pre_flight_checks_pass: true'; then
  ok "pre_flight_checks_pass = true"
elif [ "$ns" = "14" ]; then
  # 停在 Offboard 且此刻没有 offboard 流 —— 这是预期的，takeoff 节点会自己补流
  info "pre_flight_checks_pass = false（因为现在是 Offboard 且没流，属预期；
       一起跑 takeoff 节点后它会变 true，可先用 tools/probe_ocm.sh 复现验证）"
elif echo "$vsf" | grep -qE '^manual_control_signal_lost: true'; then
  # 2026-10-13：纯定点遥控飞**必须**有遥控器（要手动解锁 + 手动打杆）。
  # RC 不在线时这条会一直 false，且这是最常见的原因。
  ng "pre_flight_checks_pass = false —— 遥控器信号丢了（manual_control_signal_lost=true）。
       ⚠️ 定点遥控飞行必须先把遥控器开机、和接收机对好频（QGC 里能看到 RC 通道在动）。
       开机后重跑本检查；仍不过就在 QGC MAVLink 控制台敲 commander check 看具体哪一项。"
else
  ng "pre_flight_checks_pass != true（看 QGC 的拒绝原因）"
fi
if echo "$vs" | grep -qE '^arming_state: 1'; then ok "当前未解锁（符合预期）"; else ng "当前不是未解锁状态（WaitReady 会直接 ABORT）"; fi

hr "7. 已知会对时影响起飞的条件（信息，不拦截）"
if has_topic /fmu/out/failsafe_flags; then
  echo "$vsf" | grep ': true' | sed 's/^/    /'
fi

printf '\n\033[1m===== 结论 =====\033[0m\n'
if [ "$NG" -eq 0 ]; then
  printf '  \033[32m全部通过（%d 项）——可以起飞\033[0m\n' "$OK"
  echo
  # 2026-10-10：改成推荐 tools/mission.sh（它会强制 FastDDS 走 UDP + 先跑本自检）
  echo "  下一步："
  echo "    DRY=1 bash ~/px4_ws/tools/mission.sh                 # 干跑，确认链路与航线"
  echo "    bash ~/px4_ws/tools/mission.sh 0.5 0 0 3             # 保守复飞：只起飞+悬停3s+降落"
  echo "    bash ~/px4_ws/tools/mission.sh 0.5 0.5 180 5         # 完整任务（前进0.5m/掉头180°）"
  echo "  （等价的手敲写法仍是：ros2 launch px4_takeoff_land takeoff_land.launch.py \\"
  echo "     target_alt:=0.5 hover_time:=5.0 mission_enable:=true forward_dist:=0.5 turn_deg:=180）"
else
  printf '  \033[31m有 %d 项没过（通过 %d 项）——先别起飞\033[0m\n' "$NG" "$OK"
  echo "  按上面 [NG] 的条目处理。最常见的两个："
  echo "    · nav_state=14（飞控停在 Offboard）→ 切到定点(Position)再重跑本检查"
  echo "    · EV 订阅者=0（DDS 链路没建起来）→ 等 1 分钟，或"
  echo "        sudo systemctl restart px4-p2p-chain        # 2026-10-13 起本机默认链路是这条"
  echo "        （回到旧方案才是 px4-ev-chain）"
  echo "    · pre_flight_checks_pass=false → 先在 QGC 把飞行模式切到 定点(Position) 再重跑"
  echo "      （停在 AUTO_MISSION 又没任务时也会不过）；仍不过就看 QGC 的『Ready to Fly』列表，"
  echo "      或在 MAVLink 控制台敲 commander check 看具体哪一项（常见：没接遥控器 /"
  echo "      COM_RC_IN_MODE、无 GPS / COM_ARM_WO_GPS、电池参数没配）"
fi
