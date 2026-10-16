#!/usr/bin/env bash
# 体检「雷达 -> SLAM -> EV -> 飞控」整条链路。只读，不改任何东西。
#
#   bash ~/px4_ws/tools/check_ev.sh
#
# 用法建议：飞机上电、bash run.sh fly 拉起后运行；
#          做手推测试时也可以一直开着，边推边看第 2/3/7 节的数字。
# 注意：不要用 set -u（ROS 的 setup.bash 会引用未定义变量而报错），
#       也不要用 pipefail（timeout 杀 ros2 会返回非 0，导致误报"没有数据"）。

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

hr() { printf '\n\033[1m===== %s =====\033[0m\n' "$1"; }
have_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

hr "0. 进程"
pgrep -af 'MicroXRCEAgent|bluesea2_node|cartographer_node|tf_to_pose|tofsense_m_node|px4_ev_bridge_node' \
  || echo "(一个都没跑：先 bash run.sh fly)"

hr "1. 雷达 /scan（期望 ~15Hz，1200 点）"
timeout 6 ros2 topic hz /scan 2>/dev/null | tail -1

hr "2. SLAM 位姿 /tracked_pose（map 系；x = 开机瞬间的机头朝向，不是真正的东）"
timeout 6 ros2 topic echo /tracked_pose --once 2>/dev/null | head -22 || echo "(没有)"

hr "3. 发给飞控的 EV /fmu/in/vehicle_visual_odometry（NED：x 北 y 东 z 下）"
timeout 6 ros2 topic echo /fmu/in/vehicle_visual_odometry --once 2>/dev/null | head -30 || echo "(没有)"

hr "4. EV 桥日志（每 5 秒一行：NED 位姿 / yaw / 速度 / 高度）"
tail -n 5 /tmp/px4_ws_ev.log 2>/dev/null || echo "(无日志)"

hr "5. 【最关键】EV 有没有接收方（Subscription count 必须 >=1，否则报文根本没送出香橙派）"
timeout 10 ros2 topic info /fmu/in/vehicle_visual_odometry 2>/dev/null \
  | grep -E 'Publisher count|Subscription count'
echo "  0  = uXRCE-DDS Agent 没在桥接 → 飞控收不到任何东西（查飞控上电/接线/UXRCE_DDS_CFG）"
echo "  1+ = 报文已经出派、正送往飞控"

hr "6. 飞控链路 /fmu/out/*（一个都没有 = DDS 没通）"
ros2 topic list 2>/dev/null | grep '/fmu/out/' | sort | head -20 \
  || echo "!! 没有任何 /fmu/out 话题：检查飞控上电 / 接线 / UXRCE_DDS_CFG"

if have_topic /fmu/out/estimator_status_flags; then
  hr "7. EKF 融合状态（cs_ev_pos / cs_ev_vel / cs_ev_yaw 必须 true 才算真在用雷达）"
  timeout 6 ros2 topic echo /fmu/out/estimator_status_flags --once 2>/dev/null \
    | grep -E 'cs_ev_|cs_mag|cs_gps|cs_baro_hgt|cs_rng_hgt|reject_hor_|reject_yaw'
fi

if have_topic /fmu/out/vehicle_local_position; then
  hr "8. 飞控本地位置（xy_valid / v_xy_valid / z_valid 必须 true）"
  timeout 6 ros2 topic echo /fmu/out/vehicle_local_position --once 2>/dev/null \
    | grep -E '^x:|^y:|^z:|^vx:|^vy:|^vz:|xy_valid|v_xy_valid|z_valid|v_z_valid|heading'
fi

hr "9. 高度 /tofsense/height"
timeout 6 ros2 topic echo /tofsense/height --once 2>/dev/null || echo "(没有)"
