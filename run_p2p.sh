#!/usr/bin/env bash
# ============================================================================
#  run_p2p.sh —— "照搬参考件(onboard_ws)" 那一套的启停入口（2026-10-12 补记 8）
#
#  与 run.sh（我们自己那套：px4_ev_bridge + tofsense_m + fly_no_imu.lua）**互斥**，二选一。
#  本文件这套实际用的方案（飞控侧 EKF2_EV_CTRL=15 / EKF2_HGT_REF=3，见 src/p_to_p_px4/config/px4.yaml）：
#      EV = cartographer 的水平位置 + 航向 + **高度（来自本机定高器 /tofsense/height）**。
#      参考件原版的"EV 只给水平 + 定高走 ttyS3→ttyS1 MAVLink"那根线**本机没接**，别照它的参数改飞控。
#
#  用法（2026-10-14 起**只有一套启动形态**）：
#      bash run_p2p.sh           # 起链（= 老的 start/pos，见下面的合并说明）
#      bash run_p2p.sh stop      # 停掉本条链
#      bash run_p2p.sh status    # 看进程
#
#  2026-10-14：**取消 pos/bench/start 三种形态，合并成一套**（用户要求："不要搞那么多种模式，
#      飞行逻辑也不应该有什么变化"）。现在起的永远是**感知链**：
#      雷达(uart) + cartographer + 定高(ttyS3) + EV 桥(odom→base_link)；
#      **不起** p_to_p_mission 任务状态机、**不起** px4_offboard_bridge
#      ⇒ 一条 /fmu/in/offboard_* 都不会发。
#      理由：手动飞（遥控器定点）时"飞不飞 / 往哪飞"必须只由遥控器 + PX4 位置环决定；
#      任务状态机 + Offboard 桥是另一套飞行逻辑（切到 Offboard 它就会接管飞机、按 A→B 自己飞），
#      不该默认躺在链里。
#      以后真要跑 A→B 任务：先 `bash run_p2p.sh stop`，再手敲
#        ros2 launch p_to_p_px4 final_position.launch.py
#      （红线：同一时刻只能有一份 /fmu/in/vehicle_visual_odometry 发布者。）
#
#  2026-10-14：起链前加"**有界**等时钟"（wait_clock_bounded，最多 NTP_WAIT_MAX=15 s，
#      设 0 关掉）。原因：systemd 单元原先 After=time-sync.target，等于"公网 NTP 应答不上
#      就不起链"——实测上电到链路启动 35 s / 173 s / 283 s。单元的硬依赖已去掉，这里只留兜底。
# ============================================================================
set -Eeo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -f "$WS/tools/fastdds_udp_only.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_udp_only.xml"
fi
cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

OURS='bluesea2_node|cartographer_node|cartographer_occupancy_grid_node|px4_ev_bridge_node|tofsense_m_node|px4_imu_bridge|tf_to_pose'
P2P='tof_mavlink_bridge|cartographer_odometry_bridge|px4_offboard_bridge|ros2 launch p_to_p_px4'

stop_all() {
  # 2026-10-13：黑匣子（rosbag）也要停，否则重启链路会留下两份录制
  pkill -f 'ros2 bag record' 2>/dev/null || true
  pkill -f "$P2P"            2>/dev/null || true
  pkill -f "$OURS"           2>/dev/null || true
  pkill -f 'MicroXRCEAgent'  2>/dev/null || true
  sleep 2
  echo "已停止（旧链 + 参考件链）相关进程。"
}

# 2026-10-13（补记 14）：机载"黑匣子" —— 把飞控视角录成 rosbag2。
#  动机：飞控自己的 ulog 现在每个文件只有 ~3 秒、且订阅表损坏（"data unknown"），
#        炸机后复盘无据可依。这份 bag 不依赖飞控 SD 卡。
#  实测：对 ttyS6 串口零负担（开/关都是 FC→Pi ≈70 kB/s，飞控本来就无条件推
#        /fmu/out/*），只有本机 CPU/磁盘开销（~100 kB/s ≈ 6 MB/min）。
#  关闭：P2P_RECORD=0（或在 drop-in 里写 Environment=P2P_RECORD=0）。
start_recorder() {
  case "${P2P_RECORD:-1}" in
    0|false|no|off) echo "黑匣子：已关闭（P2P_RECORD=0）"; return 0 ;;
  esac
  mkdir -p "$WS/logs"
  nohup nice -n 10 bash "$WS/tools/record_flight.sh" >>"$WS/logs/flight_recorder.out" 2>&1 &
  echo "黑匣子：已启动（录到 ~/flights/，随本服务一起停）"
}

# 2026-10-14：时钟兜底（**有界**）。
#   原来靠单元里的 After=time-sync.target（systemd-time-wait-sync 等 NTP 真正同步上），
#   实测把"上电 → 链路启动"拖成 35 s / 173 s / 283 s（完全跟公网 NTP 应答速度绑定）。
#   现在单元不再挂那个硬依赖，这里用有界等待保住原来的意图：
#     · 最多等 NTP_WAIT_MAX（默认 15 s；设 0 = 完全不等），到点照常起链；
#     · 等到"已同步"立刻返回（配合 timesyncd 的 PollIntervalMinSec=16，通常几秒内满足）。
#   残留风险：万一 NTP 在链跑起来之后才校正时钟（大跳变），cartographer 可能崩一次 ——
#   由 launch 里的 respawn 兜底（2026-10-08 那次根因就是时间跳变，见 CHANGELOG）。
wait_clock_bounded() {
  local tmax="${NTP_WAIT_MAX:-15}" t0="$SECONDS"
  echo "等系统时间同步（最多 ${tmax}s；超时照常启动，NTP_WAIT_MAX=0 可关掉这个等待）..."
  if [ "$tmax" -le 0 ]; then
    echo "  （已按 NTP_WAIT_MAX=0 关闭等待）"
    return 0
  fi
  while [ $((SECONDS - t0)) -lt "$tmax" ]; do
    if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
      echo "  时间已同步（等了 $((SECONDS - t0))s）"
      return 0
    fi
    sleep 1
  done
  echo "  [警告] ${tmax}s 内未同步（NTP 不可达？）—— 继续启动链路" >&2
  return 1
}

# 2026-10-14：只有一套启动形态。`start` 是默认；`pos`/`bench` 保留为**等价别名**（老脚本/老
# drop-in 若还传这两个字，照样能起，只是会打一行"已废弃"提示），避免任何残留调用把人卡住。
case "${1:-start}" in
  start|pos|bench)
    if [ "${1:-}" = "pos" ] || [ "${1:-}" = "bench" ]; then
      echo "（提示）'$1' 已废弃 —— 2026-10-14 起只有一套启动形态，等价于 start。" >&2
    fi
    # 2026-10-15c：**防重复启动守卫**（run.sh 早就有，run_p2p.sh 一直缺这一道）。
    #   实测事故（17:19）：上一实例停链没杀干净（stop-sigterm 卡满 60 s 才 SIGKILL），残留下来的
    #   旧 launch 子进程与新实例并存 ⇒ **两套 cartographer / EV 桥** ⇒ /fmu/in/vehicle_visual_odometry
    #   出现两个发布者（项目红线），飞控收到两个地图原点的位置 → 位置冻住/跳变、EKF 追不上 → 炸机。
    #   有残留就拒绝启动，先把"是谁在跑"打出来（要强开：ALLOW_DOUBLE_START=1）。
    if [ "${ALLOW_DOUBLE_START:-0}" != "1" ]; then
      # ⚠️ 必须 `|| true`：pgrep 找不到进程时返回 1，配 `set -Eeo pipefail` 会让赋值语句失败
      #   ⇒ 脚本**一声不响地退出**（2026-10-15 实际踩到：systemd 一直 auto-restart，journal 里
      #   只有 "Main process exited, code=exited"，没有任何输出）。这正是本文件开头警告过的 pipefail 坑。
      busy="$( { pgrep -af 'cartographer_nod[e]|MicroXRCE[A]gent' 2>/dev/null || true; } | head -n 5 )"
      if [ -n "$busy" ]; then
        {
          echo "[错误] 已经有一条链在跑（cartographer / Micro-XRCE-DDS-Agent 还在）——拒绝启动第二份："
          echo "$busy" | sed 's/^/        /'
          echo "       先停干净：sudo systemctl restart px4-p2p-chain   （或 bash $WS/run_p2p.sh stop）"
          echo "       确认没有残留：pgrep -af 'cartographer_nod[e]|MicroXRCE[A]gent'"
          echo "       确实要强开第二份：ALLOW_DOUBLE_START=1 bash $WS/run_p2p.sh"
        } >&2
        exit 1
      fi
    fi
    # 旧的 run_p2p.sh 起的是"参考件整链"（含任务状态机 + Offboard 桥）；现在的唯一形态是
    # **只起感知链**（原来 pos 分支做的事）：给 EKF 喂位置/航向/高度，任务与 Offboard 都不起。
    stop_all
    wait_clock_bounded || true
    start_recorder
    echo "起感知链（唯一形态）：agent(ttyS6) + 雷达(uart) + cartographer + 定高(ttyS3) + EV 桥(odom→base_link)"
    echo "  （不起任务状态机、不起 Offboard 桥 ⇒ 不发任何 /fmu/in/offboard_*；飞行只由遥控器 + PX4 位置环决定）"
    exec ros2 launch p_to_p_px4 final_position.launch.py start_mission:=false start_offboard:=false
    ;;
  stop)
    stop_all
    ;;
  status)
    pgrep -af "MicroXRCEAgent|bluesea2_node|cartographer|odometry_bridge" || echo "（无）"
    ;;
  *)
    echo "用法: bash run_p2p.sh [start|stop|status]    # start 是默认，也是唯一的启动形态" >&2
    exit 1
    ;;
esac
