#!/usr/bin/env bash
# ROS 2 Humble's setup.bash references AMENT_TRACE_SETUP_FILES without a
# default, so bash nounset (-u) must not be enabled while sourcing it.
set -eo pipefail

READY_FILE="/tmp/square_preflight_ready"
GO_FILE="/tmp/square_takeoff_go"
ABORT_FILE="/tmp/square_abort"
LOG_FILE="/tmp/square_mission.log"
PID_FILE="/tmp/square_mission.pid"
ROUTE_KP="${ROUTE_KP:-0.8}"
ROUTE_KI="${ROUTE_KI:-0.0}"
ROUTE_KD="${ROUTE_KD:-0.2}"
WAYPOINT_SPEED="${WAYPOINT_SPEED:-24.0}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "ERROR: square mission is already running (PID ${old_pid})."
    echo "Abort it with: touch ${ABORT_FILE}"
    exit 1
  fi
fi

if ! pgrep -f '/home/orangepi/mavlink_ws/install/mavlink_bridge/lib/mavlink_bridge/mavlink_bridge' >/dev/null; then
  echo "ERROR: the existing MAVLink bridge is not running."
  exit 1
fi

if ! pgrep -f '/cartographer_ws/install/cartographer_ros/lib/cartographer_ros/cartographer_node' >/dev/null; then
  echo "ERROR: Cartographer is not running."
  exit 1
fi

rm -f "${READY_FILE}" "${GO_FILE}" "${ABORT_FILE}" "${LOG_FILE}" "${PID_FILE}"

export HOME=/home/orangepi
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/orangepi/mavlink_ws/config/fastdds_udp_only.xml

source /opt/ros/humble/setup.bash
source /home/orangepi/mavlink_ws/install/setup.bash

if ! ros2 pkg prefix square_mission >/dev/null 2>&1; then
  echo "ERROR: square_mission is not built. Build it in /home/orangepi/mavlink_ws first."
  exit 1
fi

echo "Starting preflight. The aircraft will remain disarmed."
echo "TUNE: route_kp=${ROUTE_KP} route_ki=${ROUTE_KI} route_kd=${ROUTE_KD} speed=${WAYPOINT_SPEED}cm/s"
echo "When prompted, select OFFBOARD with your RC transmitter."
nohup setsid ros2 run square_mission square_mission --arm --wait-go --ros-args \
  -p side_cm:=120.0 \
  -p height_cm:=100.0 \
  -p vertical_source:=tof \
  -p max_px4_ascent_cm:=160.0 \
  -p "waypoint_speed_cm_s:=${WAYPOINT_SPEED}" \
  -p "route_kp:=${ROUTE_KP}" \
  -p "route_ki:=${ROUTE_KI}" \
  -p "route_kd:=${ROUTE_KD}" \
  -p takeoff_horizontal_speed_cm_s:=5.0 \
  -p descent_horizontal_speed_cm_s:=6.0 \
  </dev/null >"${LOG_FILE}" 2>&1 &
mission_pid="$!"
echo "${mission_pid}" >"${PID_FILE}"
tail_pid=""

stop_tail() {
  if [[ -n "${tail_pid}" ]] && kill -0 "${tail_pid}" 2>/dev/null; then
    kill "${tail_pid}" 2>/dev/null || true
    wait "${tail_pid}" 2>/dev/null || true
  fi
  tail_pid=""
}

on_interrupt() {
  touch "${ABORT_FILE}"
  stop_tail
  echo
  echo "Interrupt detected. Preflight stopped; aircraft remains disarmed."
  exit 130
}
trap on_interrupt INT TERM

tail -n +1 -f "${LOG_FILE}" &
tail_pid="$!"

for _ in $(seq 1 260); do
  if [[ -f "${READY_FILE}" ]] && grep -q '^READY$' "${READY_FILE}"; then
    stop_tail
    cat "${READY_FILE}"
    echo
    echo "PREFLIGHT PASSED. RC-selected OFFBOARD mode was confirmed; aircraft is disarmed."
    echo "Next command: ~/mavlink_ws/square_takeoff.sh"
    exit 0
  fi

  if ! kill -0 "${mission_pid}" 2>/dev/null; then
    stop_tail
    echo "ERROR: preflight process exited."
    exit 1
  fi
  sleep 0.5
done

stop_tail
echo "ERROR: preflight timed out after 130 seconds."
touch "${ABORT_FILE}"
exit 1
