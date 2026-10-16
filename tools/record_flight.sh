#!/usr/bin/env bash
# ============================================================================
#  record_flight.sh —— 机载"黑匣子"：把飞控视角录成 rosbag2
#
#  为什么需要它：
#    飞控自己的 ulog 目前不可靠（实测每个文件只有 ~3 秒、且订阅表损坏 ⇒
#    "data unknown"）。这份 bag 走 uXRCE-DDS，把飞控经 /fmu/out/* 推出来的
#    状态原样记下来（模式/解锁/位置估计/姿态/电池/故障），炸机后可直接复盘，
#    不依赖飞控 SD 卡。
#
#  用法：
#     bash ~/px4_ws/tools/record_flight.sh          # 前台录，Ctrl-C 停（服务里由 run_p2p.sh 拉起）
#     FLIGHT_KEEP=6 bash ~/px4_ws/tools/record_flight.sh
#  输出：~/flights/<YYYYmmdd_HHMMSS>_<本次开机 id>_a<n>/
#
#  2026-10-15 修复：**目录名撞车 ⇒ 整架次没有黑匣子**（本次事故的直接原因之一）
#    现象：boot -1（c0c70544）与 boot 0（217612d5）两次开机都算出同一个 STAMP
#      `20261014_221013` —— 因为 RTC 停走、timesyncd 又把这几次开机统统"恢复到"
#      几乎同一时刻（22:09:53 / 22:01:08），所以两次 `date +%Y%m%d_%H%M%S` 结果相同。
#      第二次 `ros2 bag record -o` 直接报
#        [ERROR] [ros2bag]: Output folder '.../20261014_221013' already exists.
#      然后 exit=1 ⇒ 16:0x 那次飞行一条数据都没录上。
#    处理：① 目录名带上本次开机唯一的 boot_id（跨开机不可能再撞车，顺带能一眼对上是哪次开机）；
#          ② 真还有同名残留就自动加 `-2/-3/...`；
#          ③ 开录 20 秒内就退出（启动期故障）→ 换名字重试，连续 3 次才放弃；
#          ④ 收到 SIGTERM/SIGINT（服务停）→ 让录制器优雅收尾后本脚本退出，不跟 systemd 抢。
#    ⚠️ 这只是"名字不再撞车"，根因（RTC 停走 / 时钟被 NTP 大跳变）另案处理。
# ============================================================================
set -Eeo pipefail

# 服务停（systemd 发 SIGTERM）时：把录制器优雅停掉，本脚本随即退出，不要在这里自旋
STOPPING=0
trap 'STOPPING=1; pkill -INT -f "ros2 bag record" 2>/dev/null || true' TERM INT
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -f "$WS/tools/fastdds_udp_only.xml" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/tools/fastdds_udp_only.xml"
fi

OUT_ROOT="${FLIGHT_DIR:-$HOME/flights}"
KEEP="${FLIGHT_KEEP:-8}"
mkdir -p "$OUT_ROOT"

# 只保留最近 KEEP 份（严格限制在 $OUT_ROOT 下一层目录内）
prune() {
  local n=0 d
  while IFS= read -r d; do
    n=$((n + 1))
    [ "$n" -le "$KEEP" ] && continue
    case "$d" in
      "$OUT_ROOT"/*) echo "[record_flight] 清理旧记录 $d"; rm -rf -- "$d" ;;
      *) echo "[record_flight] 跳过（不在 $OUT_ROOT 下）：$d" ;;
    esac
  done < <(find "$OUT_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -rn | cut -d' ' -f2-)
}
prune

# 2026-10-15：名字 = 墙钟 + 本次开机 id（`/proc/sys/kernel/random/boot_id` 前 8 位）。
#   墙钟可能因为 RTC/NTP 而重复，boot_id 不会 —— 两次开机不可能算出同名目录。
BOOTID="$(cut -c1-8 /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown-x)"
STAMP="$(date +%Y%m%d_%H%M%S)_${BOOTID}"

# 再兜一层：万一真还有同名残留（同名目录、上次崩溃留下的），自动加 -2/-3/…
unique_out() {
  local base="$OUT_ROOT/$1" i
  if [ ! -e "$base" ]; then printf '%s\n' "$base"; return 0; fi
  for i in $(seq 2 50); do
    if [ ! -e "$base-$i" ]; then printf '%s\n' "$base-$i"; return 0; fi
  done
  printf '%s-%s\n' "$base" "${RANDOM}"
}

# 话题：以"能复盘飞控行为"为准。
# 2026-10-14：加 /scan（起降漂移/扫描质量复盘必须看）、/imu（fly.lua 的旋转先验来源）、
#   /px4_bridge/odometry_state（EV 桥状态机：沿用了哪一帧、为什么停发）。
TOPICS=(
  /fmu/out/vehicle_status
  /fmu/out/vehicle_control_mode
  /fmu/out/vehicle_local_position
  /fmu/out/vehicle_attitude
  /fmu/out/vehicle_odometry
  /fmu/out/estimator_status_flags
  /fmu/out/failsafe_flags
  /fmu/out/battery_status
  /fmu/out/manual_control_setpoint
  /fmu/out/vehicle_command_ack
  /fmu/in/vehicle_visual_odometry
  /tofsense/height
  /tf
  /tf_static
  /fmu/out/manual_control_switches
  /fmu/out/actuator_outputs
  /scan
  /imu
  /px4_bridge/odometry_state
)

# ==== 2026-10-13 修复：开录前先等飞控 DDS 话题出现 ====
# 现象（CHANGELOG 同日条）：录制器比 MicroXRCEAgent 先起时，整份 bag 只有 /tofsense/height，
#   /fmu/* 一条都收不到（18:07 / 18:11 / 18:36 三份都这样）；手工在 /fmu/* 出现之后再起
#   录制器则完全正常（/fmu/out/vehicle_status 33 帧/18 s）⇒ 先等话题，再开录。
wait_for_fcu_topics() {
  local waited=0
  while [ "$waited" -lt 120 ]; do
    if ros2 topic list 2>/dev/null | grep -q '^/fmu/out/vehicle_status$'; then
      echo "[record_flight] 飞控话题已出现（等了 ${waited}s），再等 3s 让 DDS 发现稳定"
      sleep 3; return 0
    fi
    sleep 1; waited=$((waited + 1))
  done
  echo "[record_flight] 警告：等 /fmu/* 超时 120s，仍按现状开录"
  return 1
}
wait_for_fcu_topics || true

# ==== 2026-10-14 修复：保证只有一个录制器 ====
# 现象：2026-10-14 19:08 / 20:04 录制器崩在
#   SqliteException: ... SQLite error (5): database is locked
#   ⇒ 整架次没有黑匣子（有些飞行因此完全没录上）。
# 处理：开录前先温和停掉已有录制器（SIGINT 让它把 metadata 写完），再开新的。
if pgrep -f 'ros2 bag record' >/dev/null 2>&1; then
  echo "[record_flight] 警告：已有录制器在跑，先优雅停止（SIGINT）"
  pkill -INT -f 'ros2 bag record' 2>/dev/null || true
  for _ in $(seq 1 10); do
    pgrep -f 'ros2 bag record' >/dev/null 2>&1 || break
    sleep 1
  done
  if pgrep -f 'ros2 bag record' >/dev/null 2>&1; then
    echo "[record_flight] 仍在运行，强制结束（SIGTERM）"
    pkill -TERM -f 'ros2 bag record' 2>/dev/null || true
    sleep 2
  fi
fi

# ==== 2026-10-15b 修复：**bag 里只有 /tofsense/height，/fmu/* 与 /scan 一条都没有** ====
#   现象（17:10 那几份 bag，用户 15:5x 那次飞行也是）：话题清单里只有 /tofsense/height，
#     飞控状态、/scan、EV 全缺 ⇒ 飞行复盘等于零。2026-10-13 记过同一个坑
#     （"录制器早起时整份 bag 只有 /tofsense/height"），当时"等 /fmu/out/vehicle_status + 再等 3 s"
#     的办法**证明不够**（同机 FastDDS 发现层：看得见、收不到）。
#   对策：① 挂上仓库里现成但一直没用的 QoS 覆盖（/fmu/* 是 BEST_EFFORT/TRANSIENT_LOCAL，
#           大消息 /scan 也容易建不起通路）；
#         ② 开录 15 s 后**直接读 .db3 数帧数**（不依赖 metadata，它有 ended 才写），
#           必需的四个话题只要有一个 0 帧 → 立刻停掉这份 bag、换名重录（最多 3 次）。
REQUIRED_TOPICS=(
  /fmu/out/vehicle_status
  /fmu/out/vehicle_local_position
  /fmu/in/vehicle_visual_odometry
  /scan
)
# 2026-10-15b：`--qos-profile-overrides-path tools/bag_qos_overrides.yaml` 试过一版，**反而更糟**
#   （挂上后连 /tofsense/height 都收不到了）⇒ 撤掉，不再使用；真要用得先单独验证。
QOS_ARGS=()

# ⚠️ 本脚本的全部等待/重试都发生在**本进程内部**，而 run_p2p.sh 是用 `nohup ... &` 把它后台拉起的
#   ⇒ 这些 sleep 不会拖慢链路自身的启动与 DDS/RTPS 建链（agent → 会话 → EV 那条路径完全不动）。
#   刻意不在这里另起"订阅型探针"进程：多一个参与者反而会搅乱同机发现（项目里"看得见、收不到"的老毛病）。

# 直接读 .db3 判断"必须的话题是否都真的有帧"（0=齐全，非 0=缺）
check_bag_topics() {   # $1=bag 目录，其余=必须的话题
  local d="$1"; shift
  local db
  db="$(ls "$d"/*.db3 2>/dev/null | head -n 1)"
  [ -n "$db" ] || return 1
  python3 - "$db" "$@" <<'PY'
import sqlite3, sys
db, want = sys.argv[1], sys.argv[2:]
try:
    con = sqlite3.connect('file:%s?mode=ro' % db, uri=True)
    have = {}
    for tid, name in con.execute('select id, name from topics'):
        have[name] = con.execute('select count(*) from messages where topic_id=?', (tid,)).fetchone()[0] or 0
except Exception as exc:
    print('读 .db3 失败：%s' % exc, file=sys.stderr)
    sys.exit(1)
miss = [t for t in want if have.get(t, 0) == 0]
if miss:
    print('缺话题：%s' % ' '.join(miss), file=sys.stderr)
    sys.exit(1)
sys.exit(0)
PY
}

# ==== 2026-10-15：录制循环（黑匣子绝不允许"静默什么都不录"）====
#   · 正常结束（Ctrl-C / ros2 bag 返回 0）→ 退出；
#   · 开录 20 秒内就退出 或 15 秒自检缺话题（多半是发现层/QoS）→ 换名字重试，连续 3 次才放弃；
#   · 跑起来之后再崩（例如 2026-10-14 那次 SQLite database is locked）→ 重新起一份新 bag，
#     宁可把一次飞行拆成几段 bag，也不要整架次空白。
rc=0
attempt=0
rapid_fail=0
while :; do
  attempt=$((attempt + 1))
  OUT="$(unique_out "${STAMP}_a${attempt}")"
  echo "[record_flight] 开始录制（第 ${attempt} 次）→ $OUT"
  t0="$SECONDS"
  ros2 bag record -o "$OUT" --max-bag-size 268435456 "${QOS_ARGS[@]}" "${TOPICS[@]}" &
  bagpid=$!
  # 开录 15 s 后自检：必需话题必须都真的有帧（缺了=发现层/QoS 没建起来，这份 bag 没价值）
  sleep 15
  bad=0
  check_bag_topics "$OUT" "${REQUIRED_TOPICS[@]}" || bad=1
  if [ "$bad" -ne 0 ] && kill -0 "$bagpid" 2>/dev/null; then
    # 2026-10-15c：**只告警，绝不中断正在录的 bag**。
    #   原来"自检不合格就杀掉换名重录"会在飞行期间反复把正在录的 bag 切碎 ——
    #   当天两次飞行的数据就是这么丢的（目录里留下一串 _a1/_a2/_a3 的碎片）。
    #   现在照常录完：宁可这份 bag 少几个话题，也不能中断正在进行的记录。
    echo "[record_flight] [警告] 开录 15s 自检缺话题 —— 继续录，不做任何中断（飞行数据优先保住）" >&2
  fi
  wait "$bagpid" || rc=$?
  ran=$((SECONDS - t0))
  echo "[record_flight] 录制器退出，exit=$rc，已录 ${ran}s，记录在 $OUT"

  if [ "$rc" -eq 0 ]; then break; fi
  if [ "$STOPPING" -eq 1 ]; then
    echo "[record_flight] 服务正在停止，不再重试"; break
  fi
  if [ "$ran" -lt 20 ]; then
    rapid_fail=$((rapid_fail + 1))
    if [ "$rapid_fail" -ge 3 ]; then
      echo "[record_flight] 连续 ${rapid_fail} 次开录即失败，放弃（检查 ~/flights 权限 / ros2 bag / 磁盘）" >&2
      break
    fi
    echo "[record_flight] 开录 ${ran}s 就退出 —— 5s 后换名重试" >&2
  else
    rapid_fail=0
    echo "[record_flight] 录制中断 —— 5s 后重新起一份" >&2
  fi
  sleep 5
done
exit "$rc"
