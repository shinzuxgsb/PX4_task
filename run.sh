#!/usr/bin/env bash
# ============================================================================
#  px4_ws 一键启动 / 单组件重启
#
#    bash run.sh fly              起飞前全套（定点模式必用）
#    bash run.sh restart <组件>   只重启一个组件：agent|imu|lidar|slam|tf|tof|ev
#    bash run.sh lidar            只起雷达（= restart lidar）
#    bash run.sh carto | slam     雷达 + 建图（前台，Ctrl-C 退出）
#    bash run.sh agent            只起 Micro-XRCE-DDS-Agent
#    bash run.sh px4              雷达 + PX4 速度桥（和任务节点抢 offboard，二选一）
#    bash run.sh status | stop | init
#
#  2026-10-10 重构（起因：自启动 ~100 秒、还经常"服务 active 但链子是坏的"）：
#    · **事件驱动**：不再用固定 sleep 猜"起好没"，而是轮询就绪条件
#      （雷达=驱动日志出现 `lidar start work` 且 /scan 有频率；SLAM=/tracked_pose 有频率；
#        EV 桥=它日志里出现 `EV: NED pos`）。正常启动 ~100s → ~15-25s。
#    · **并行启动**：Agent / IMU 桥 / 定高器 / 雷达 互不依赖，一起起。
#    · **判据以"进程日志"为准、ROS 话题查询为辅**：FastDDS 同机发现层偶发"看得见收不到"，
#      只看 `ros2 topic` 会误判（2026-10-10 就误判过一次"雷达坏了"）。
#    · **任何组件失败都不中断整条链**：交给看门狗 px4-chain-watch 分段恢复。
#    · 新增 `restart <组件>`：看门狗和人工恢复走同一条路径。
#
#  ⚠️ 注意 pipefail：`cmd | grep -qm1 X` 会**永远判失败**（grep 提前退出 → cmd 收 SIGPIPE），
#     本脚本一律用 `out="$(cmd | grep -m1 X || true)"` 的写法。
# ============================================================================
set -Eeo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 雷达连接方式：串口款 LDS-50C-R（/dev/ttyS4）
#   想临时切回网口款 LDS-50C-E：LIDAR_PARAMS=.../udp_lidar.yaml bash run.sh lidar
LIDAR_PARAMS="${LIDAR_PARAMS:-$WS/src/bluesea2/src/bluesea-ros2/params/uart_lidar.yaml}"
LIDAR_LOG="${LIDAR_LOG:-/tmp/px4_ws_lidar.log}"
IMU_LOG="${IMU_LOG:-/tmp/px4_ws_imu.log}"
SLAM_LOG="${SLAM_LOG:-/tmp/px4_ws_slam.log}"
TF_LOG="${TF_LOG:-/tmp/px4_ws_tf2pose.log}"
TOF_LOG="${TOF_LOG:-/tmp/px4_ws_tof.log}"
EV_LOG="${EV_LOG:-/tmp/px4_ws_ev.log}"
# 强制 UDP 传输，绕开 FastDDS 同机共享内存的坑（只影响本脚本起的查询进程）
FASTRTPS_XML="$WS/tools/fastdds_udp_only.xml"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[错误] 还没编译：先执行 bash $WS/build.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

# ------------------------------------------------------------------ 小工具
rate_of() {   # rate_of <话题> [超时秒]；输出 "average rate: ..." 或空
  FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_XML" \
    timeout "${2:-6}" ros2 topic hz "$1" 2>/dev/null | grep -m1 'average rate' || true
}
log_has()   { [ -f "$1" ] && grep -q -- "$2" "$1"; }
proc_alive(){ pgrep -f "$1" > /dev/null; }

# 轮询等待：wait_until <描述> <超时秒> <判据命令...>
# ⚠️ 必须用**墙钟**计时：判据本身（ros2 topic hz）就要花 4~5 秒，
#    如果按"循环次数"计数，30 次 × 5 秒 = 150 秒（2026-10-10 实测踩到过：重启花了 182 秒）。
wait_until() {
  local desc="$1" tmax="$2"; shift 2
  local t0="$SECONDS"
  while [ $((SECONDS - t0)) -lt "$tmax" ]; do
    if "$@" > /dev/null 2>&1; then
      echo "     ✓ $desc（等待 $((SECONDS - t0))s）"
      return 0
    fi
    sleep 1
  done
  echo "     [警告] $desc 等了 ${tmax}s 仍未就绪 —— 继续启动，交给看门狗处理" >&2
  return 1
}

# 就绪判据
ready_lidar()        { log_has "$LIDAR_LOG" 'lidar start work' && [ -n "$(rate_of /scan 4)" ]; }
ready_lidar_logonly(){ log_has "$LIDAR_LOG" 'lidar start work'; }
# SLAM 就绪：cartographer 已经在处理扫描（两种地面真值二选一）
#   ① 它自己日志里的 `scan rate:` 行（每 ~15-30 秒一行）
#   ② /scan_matched_points2 有频率（每处理一帧就发一次）
#   注意：**不能等 /tracked_pose** —— 那是 tf_to_pose 转发的，而 tf_to_pose 在 SLAM 之后才起。
ready_slam()         { log_has "$SLAM_LOG" 'scan rate:' || [ -n "$(rate_of /scan_matched_points2 4)" ]; }
ready_pose()         { [ -n "$(rate_of /tracked_pose 4)" ]; }
ready_tof()          { [ -n "$(rate_of /tofsense/height 4)" ]; }
ready_ev()           { log_has "$EV_LOG" 'EV: NED pos'; }

# ------------------------------------------------------------ 各组件启停
start_agent() {
  echo "     Micro-XRCE-DDS-Agent（ttyS6）..."
  bash "$WS/agent.sh" start
}

start_imu() {
  pkill -f px4_imu_bridge 2>/dev/null || true
  if [ -f "$HOME/px4_imu_bridge.py" ]; then
    nohup python3 "$HOME/px4_imu_bridge.py" > "$IMU_LOG" 2>&1 < /dev/null &
    echo "     IMU 桥已起（日志 $IMU_LOG；飞控没上电时这里只有警告，属正常）"
  else
    echo "     [警告] 找不到 $HOME/px4_imu_bridge.py" >&2
  fi
}

start_tof() {
  pkill -f tofsense_m_node 2>/dev/null || true
  pkill -f 'ros2 launch tofsense_m' 2>/dev/null || true   # 连 launch 包装一起杀，防止 respawn 出第二份
  sleep 1
  nohup ros2 launch tofsense_m tofsense_m.launch.py > "$TOF_LOG" 2>&1 < /dev/null &
  echo "     定高器已起（日志 $TOF_LOG）"
}

start_lidar() {
  local port="${LIDAR_PORT:-/dev/ttyS4}"
  local attempt
  if [ ! -e "$LIDAR_PARAMS" ]; then
    echo "[错误] 雷达参数文件不存在：$LIDAR_PARAMS" >&2
    return 1
  fi
  # ==========================================================================
  # 雷达加固（2026-10-10，实测踩过 3 种"静默失败"）：
  #   ① open 后报 `code:-106 Failure to modify serial port baud rate` 后卡死；
  #   ② 起来了、握手也过了，却不发布 /scan（进程活着、握着串口句柄）；
  #   ③ 同类：日志只有一行、之后再无输出。
  #   对策：起之前用 stty 预置端口；起完**按事件轮询就绪**（日志握手 + /scan 有频率，
  #        实测握手只要 ~2 秒，原来写死的 25 秒是网口款时代留下的，纯属白等）；
  #        没就绪重试一次；两个判据不一致时倾向"信任日志"（DDS 查询可能是假象）。
  # ==========================================================================
  for attempt in 1 2; do
    pkill -f bluesea2_node 2>/dev/null || true
    sleep 1
    if [ -e "$port" ]; then
      stty -F "$port" 921600 cs8 -cstopb -parenb -crtscts raw -echo 2>/dev/null \
        && echo "     已预置 $port = 921600 8N1 raw"
    fi
    : > "$LIDAR_LOG"        # 清掉旧日志，避免上次的 "lidar start work" 骗过就绪判据
    nohup ros2 run bluesea2 bluesea2_node --ros-args --params-file "$LIDAR_PARAMS" \
      > "$LIDAR_LOG" 2>&1 < /dev/null &
    echo "     雷达驱动已启动（第 ${attempt} 次，日志 $LIDAR_LOG）"
    if wait_until "雷达就绪（日志握手 + /scan 有频率）" "${LIDAR_READY_TIMEOUT:-30}" ready_lidar; then
      return 0
    fi
    if ready_lidar_logonly; then
      echo "     [提示] 驱动日志显示握手完成，但本进程看不到 /scan —— 可能是 DDS 发现层假象；交给看门狗复核" >&2
      return 0
    fi
    echo "     [警告] 第 ${attempt} 次启动后仍未就绪，重试..." >&2
  done
  {
    echo "[错误] 雷达连续两次都没就绪！依次检查："
    echo "        1) $LIDAR_LOG 里有没有 'Failure to modify serial port baud rate'"
    echo "        2) 雷达供电 / 排针 19(TX) 23(RX) / 共地；裸读验证（先把链路停掉）："
    echo "           sudo timeout 6 cat /dev/ttyS4 | wc -c    # 正常应有几十万字节"
    echo "        3) ls -l /dev/ttyS4 在不在（overlay 生效？）"
  } >&2
  return 1
}

start_slam() {
  local flavor="${1:-carto}"
  pkill -f cartographer 2>/dev/null || true
  pkill -f slam_toolbox 2>/dev/null || true
  # ⚠️ 2026-10-10 实测踩到：只杀节点不行！`ros2 launch` 的包装进程还活着，
  #    而 launch 里 respawn=True → 它会把 cartographer 再拉起来 → **两份实例**打架
  #    （现象：TF 抖动、`map_builder_bridge.cpp:180 Requested submap … does not exist`、
  #     /tracked_pose 断流）。所以必须把 launch 包装也杀掉，并等它们真的退出。
  pkill -f 'ros2 launch carto_pkg' 2>/dev/null || true
  pkill -f 'ros2 launch slam_pkg'  2>/dev/null || true
  sleep 2
  : > "$SLAM_LOG"
  if [ "$flavor" = "slam" ]; then
    nohup ros2 launch slam_pkg slam.launch.py > "$SLAM_LOG" 2>&1 < /dev/null &
    echo "     slam_toolbox 已起（日志 $SLAM_LOG）"
    return 0
  fi
  local cfg="fly_no_imu.lua"
  if [ "${SLAM_USE_IMU:-0}" = "1" ]; then
    local probe
    probe="$(rate_of /imu 8)"
    if [ -n "$probe" ]; then
      cfg="fly.lua"
      echo "     SLAM_USE_IMU=1 且 /imu 有数据（$probe）→ 用 fly.lua（启用 IMU 旋转预测）"
    else
      echo "     [警告] 你要求用 IMU，但没探到 /imu 数据 → 仍退回 fly_no_imu.lua" >&2
    fi
  else
    echo "     用 fly_no_imu.lua（纯激光匹配；想试 IMU：SLAM_USE_IMU=1 bash run.sh fly）"
  fi
  nohup ros2 launch carto_pkg fly_carto.launch.py use_rviz:=false config_basename:="$cfg" \
    > "$SLAM_LOG" 2>&1 < /dev/null &
  echo "     cartographer 已起（日志 $SLAM_LOG，配置 $cfg）"
}

start_tf() {
  pkill -f tf_to_pose 2>/dev/null || true
  : > "$TF_LOG"
  if [ -f "$HOME/tf_to_pose.py" ]; then
    nohup python3 "$HOME/tf_to_pose.py" > "$TF_LOG" 2>&1 < /dev/null &
    echo "     tf_to_pose 已起（日志 $TF_LOG）"
  else
    echo "     [警告] 找不到 $HOME/tf_to_pose.py —— EV 桥可能拿不到位姿！" >&2
  fi
}

start_ev() {
  pkill -f px4_ev_bridge_node 2>/dev/null || true
  pkill -f 'ros2 launch px4_ev_bridge' 2>/dev/null || true
  sleep 1
  : > "$EV_LOG"
  nohup ros2 launch px4_ev_bridge ev_bridge.launch.py > "$EV_LOG" 2>&1 < /dev/null &
  echo "     EV 桥已起（日志 $EV_LOG）"
}

# ------------------------------------------------------------------ 状态摘要
status_summary() {
  echo "--- 进程 ---"
  pgrep -af 'MicroXRCEAgent|bluesea2_node|cartographer|slam_toolbox|tf_to_pose|tofsense_m_node|px4_imu_bridge|px4_ev_bridge_node' \
    || echo "(无)"
  echo "--- 关键话题（并行采样，最多 6 秒；带 UDP profile 以免被 DDS 发现层骗）---"
  local d t pids=()
  d="$(mktemp -d)"
  for t in /scan /imu /tofsense/height /tracked_pose /fmu/in/vehicle_visual_odometry; do
    ( r="$(rate_of "$t" 6)"; printf '%-38s %s\n' "$t" "${r:-（无数据）}" > "$d/$(echo "$t" | tr '/' '_')" ) &
    pids+=($!)
  done
  # ⚠️ 只能等这几个采样子进程：本脚本此前用 nohup 起了整条链（雷达/SLAM/EV…），
  # 裸 `wait` 会等**所有后台任务**（等于等整条链退出）→ 启动脚本被 systemd 超时杀掉。
  wait "${pids[@]}"
  cat "$d"/* 2>/dev/null || true
  rm -rf "$d"
  # 日志地面真值：并发查话题时偶尔会互相干扰（实测 5 个 hz 并发时 /tracked_pose 报过"无数据"），
  # 所以再给一行"日志证据"，两者不一致时以日志为准。
  echo "--- 日志地面真值 ---"
  if log_has "$LIDAR_LOG" 'lidar start work'; then echo "  雷达驱动：已握手（日志 $LIDAR_LOG）"; else echo "  雷达驱动：日志里没有握手标记 ✗"; fi
  echo "  SLAM：$(log_has "$SLAM_LOG" 'scan rate:' && (grep 'scan rate:' "$SLAM_LOG" | tail -1 | sed 's/^.*scan rate/scan rate/') || echo '还没有 scan rate 行')"
  echo "  定高器：$(tail -n 1 "$TOF_LOG" 2>/dev/null | sed 's/^\[[^]]*\] //' | cut -c1-60)"
  echo "  EV 桥：$(tail -n 1 "$EV_LOG" 2>/dev/null | sed 's/^\[[^]]*\] //' | cut -c1-70)"
}

stop_all() {
  pkill -f bluesea2_node      2>/dev/null || true
  pkill -f MicroXRCEAgent     2>/dev/null || true
  pkill -f cartographer       2>/dev/null || true
  pkill -f slam_toolbox       2>/dev/null || true
  pkill -f tofsense_m_node    2>/dev/null || true
  pkill -f tf_to_pose         2>/dev/null || true
  pkill -f px4_bridge_node    2>/dev/null || true
  pkill -f px4_ev_bridge_node 2>/dev/null || true
  pkill -f px4_imu_bridge     2>/dev/null || true
  # launch 包装进程（带 respawn=True 的）也必须停，否则它们会把节点再拉起来
  pkill -f 'ros2 launch carto_pkg'    2>/dev/null || true
  pkill -f 'ros2 launch slam_pkg'     2>/dev/null || true
  pkill -f 'ros2 launch tofsense_m'   2>/dev/null || true
  pkill -f 'ros2 launch px4_ev_bridge' 2>/dev/null || true
  echo "已停止本工作区的进程。"
}

# ------------------------------------------------------- 起飞前全套（事件驱动）
start_fly() {
  local flavor="${1:-carto}"
  local t_start="$SECONDS"
  # 防重复启动：两份 run.sh fly 会互相 pkill，留下"进程都在却没数据"的半死状态
  if [ "${ALLOW_DOUBLE_START:-0}" != "1" ] && \
     pgrep -f "cartographer[_]node|MicroXRCE[A]gent" > /dev/null; then
    {
      echo "[错误] 已经有一条链路在跑（cartographer / Micro-XRCE-DDS-Agent 存在）。"
      echo "       停掉再起：bash $WS/run.sh stop ｜ 或 sudo systemctl restart px4-ev-chain"
      echo "       只重启单个组件：bash $WS/run.sh restart lidar|slam|ev|tof|tf|imu|agent"
      echo "       （确实要强开两份：ALLOW_DOUBLE_START=1 bash run.sh fly）"
    } >&2
    return 1
  fi

  # [0/7] 时钟：systemd 侧已 After=time-sync.target，这里只兜底，超时就不等了
  echo "[0/7] 等系统时间同步（防止 NTP 校正把 cartographer 打崩）..."
  local ntp_max="${NTP_WAIT_MAX:-20}" t_wait=0
  while [ "$t_wait" -lt "$ntp_max" ]; do
    if [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; then
      echo "      时间已同步（${t_wait}s）"; break
    fi
    sleep 1; t_wait=$((t_wait + 1))
  done
  [ "$t_wait" -ge "$ntp_max" ] && \
    echo "      [警告] ${ntp_max}s 内未同步，继续启动（注意时间跳变风险；RTC 纽扣电池该查了）" >&2

  # [1/7] **全部组件并行起**（都自带轮询等待，谁先就绪都不影响别人）：
  #   上游：Agent / IMU 桥 / 定高器 / 雷达
  #   中游：SLAM（订阅 /scan，不等雷达就绪也能先起，省掉串行等待）
  #   下游：tf_to_pose（等 TF）/ EV 桥（等位姿+高度）
  #   为什么能这样：每个节点自己都是"起来后轮询输入"，先把它们都拉起来，
  #   启动时间就从"各步 sleep 相加"变成"整条流水线的关键路径"。
  echo "[1/7] 并行起全部组件（Agent / IMU 桥 / 定高器 / 雷达 / SLAM / tf_to_pose / EV 桥）..."
  start_agent
  start_imu
  start_tof
  start_lidar || echo "     [警告] 雷达没就绪 —— 继续启动其余部分，交给 px4-chain-watch 恢复" >&2
  start_slam "$flavor"
  if [ "$flavor" = "slam" ]; then
    echo "     （slam_toolbox 直接发 /pose，无需 tf_to_pose）"
  else
    start_tf
  fi
  start_ev

  # [2/7] 等雷达（实测握手 ~2 秒，加上 15Hz 出帧，一般 5~10 秒）
  echo "[2/7] 等雷达就绪（驱动日志握手 + /scan 有频率）..."
  if [ -n "$(rate_of /scan 4)" ] || ready_lidar; then
    echo "     ✓ 雷达已就绪"
  else
    wait_until "雷达就绪" "${LIDAR_READY_TIMEOUT:-30}" ready_lidar || \
      echo "     [警告] 雷达没就绪：看 $LIDAR_LOG 与 /dev/ttyS4；看门狗会继续重试" >&2
  fi

  # [3/7] 等 SLAM 真正在处理扫描（cartographer 日志 scan rate / matched points）
  echo "[3/7] 等 SLAM 开始处理扫描..."
  wait_until "SLAM 在处理扫描" 45 ready_slam || true

  # [4/7] 等位姿转发出来（tf_to_pose 已经把 map→laser_link 转成 /tracked_pose）
  echo "[4/7] 等 /tracked_pose 出来..."
  wait_until "/tracked_pose 有频率" 20 ready_pose || true

  # [5/7] 等 EV 桥开始发（它的日志出现 'EV: NED pos'）
  echo "[5/7] 等 EV 桥开始发 vehicle_visual_odometry..."
  wait_until "EV 桥在发 vehicle_visual_odometry" 20 ready_ev || true

  # [6/7] 状态摘要
  echo "[6/7] 状态摘要"
  status_summary

  # [7/7] 提示
  echo
  echo "[7/7] 下一步（飞控端 MAVLink 控制台）确认："
  echo "  listener vehicle_visual_odometry     # 应有数据，不是 never published"
  echo "  listener vehicle_local_position      # xy_valid / z_valid 要是 true"
  echo "  起飞前再跑一遍： sudo bash $WS/tools/preflight.sh"
  echo "本轮启动耗时：$((SECONDS - t_start)) 秒"
}

case "${1:-}" in
  init)
    echo "环境已就绪（ROS_DOMAIN_ID=$ROS_DOMAIN_ID）"
    ros2 topic list
    ;;
  lidar)
    start_lidar || true
    echo "--- /scan 频率 ---"
    rate_of /scan 10 || echo "(无数据)"
    ;;
  carto)
    start_lidar || true
    echo "启动 cartographer（Ctrl-C 退出）..."
    exec ros2 launch carto_pkg fly_carto.launch.py use_rviz:=false
    ;;
  slam)
    start_lidar || true
    echo "启动 slam_toolbox（Ctrl-C 退出）..."
    exec ros2 launch slam_pkg slam.launch.py
    ;;
  px4)
    start_lidar || true
    proc_alive MicroXRCEAgent || {
      echo "[提示] Micro-XRCE-DDS-Agent 没在跑，先执行： bash $WS/agent.sh start"
    }
    echo "启动 PX4 桥接..."
    exec ros2 launch px4_bridge px4_bridge.launch.py
    ;;
  agent)
    start_agent
    ;;
  fly)
    start_fly "${2:-carto}"
    ;;
  restart)
    # 单组件重启：看门狗与人工恢复走同一条路径
    case "${2:-}" in
      agent) pkill -f MicroXRCEAgent 2>/dev/null || true; start_agent ;;
      imu)   start_imu ;;
      lidar) start_lidar || echo "[警告] 雷达没就绪（看门狗会在下一轮再试）" >&2 ;;
      slam)  start_slam "${3:-carto}"
             wait_until "SLAM 在处理扫描" 45 ready_slam || true
             wait_until "/tracked_pose 有频率" 20 ready_pose || true ;;
      tf)    start_tf ;;
      tof)   start_tof; wait_until "定高器出数据" 10 ready_tof || true ;;
      ev)    start_ev;   wait_until "EV 桥开始发 vehicle_visual_odometry" 15 ready_ev || true ;;
      *)     echo "用法: bash run.sh restart {agent|imu|lidar|slam|tf|tof|ev}" >&2; exit 1 ;;
    esac
    ;;
  status)
    status_summary
    ;;
  stop)
    stop_all
    ;;
  *)
    echo "用法: bash run.sh {init|lidar|carto|slam|agent|px4|fly|restart <组件>|status|stop}"
    ;;
esac
