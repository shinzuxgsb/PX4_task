#!/usr/bin/env bash
set -eo pipefail

WS="/home/orangepi/mavlink_ws"
READY_FILE="/tmp/shelf_preflight_ready"
GO_FILE="/tmp/shelf_takeoff_go"
ABORT_FILE="/tmp/shelf_abort"
LOG_FILE="/tmp/shelf_px4_launch.log"
PID_FILE="/tmp/shelf_px4.launch.pid"
LASER_GPIO_CHIP="${LASER_GPIO_CHIP:-gpiochip1}"
LASER_GPIO_LINE="${LASER_GPIO_LINE:-4}"
LASER_ACTIVE_HIGH="${LASER_ACTIVE_HIGH:-false}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "ERROR: D mission launch is already running (PID ${old_pid})."
    echo "Abort it with: touch ${ABORT_FILE}"
    exit 1
  fi
fi

for required in /dev/ttyS4 /dev/video0; do
  if [[ ! -e "${required}" ]]; then
    echo "ERROR: required device is missing: ${required}"
    exit 1
  fi
done

if ! pgrep -f "${WS}/install/mavlink_bridge/lib/mavlink_bridge/mavlink_bridge" >/dev/null; then
  echo "ERROR: the MAVLink bridge is not running."
  exit 1
fi
if ! pgrep -f '/cartographer_ws/install/cartographer_ros/lib/cartographer_ros/cartographer_node' >/dev/null; then
  echo "ERROR: Cartographer is not running."
  exit 1
fi
if pgrep -f 'shelf_code_reader_[n]ode|shelf_mission_[n]ode|shelf_px4_[a]dapter' >/dev/null; then
  echo "ERROR: another D mission process is already running."
  exit 1
fi

export HOME=/home/orangepi
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="${WS}/config/fastdds_udp_only.xml"
export PYTHONPATH="${WS}/third_party${PYTHONPATH:+:${PYTHONPATH}}"

source /opt/ros/humble/setup.bash
source /home/orangepi/cartographer_ws/install/setup.bash
source "${WS}/install/setup.bash"

for package in shelf_scan_pkg pid_control_pkg shelf_px4_bridge; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "ERROR: ${package} is not built in ${WS}."
    exit 1
  fi
done

rm -f "${READY_FILE}" "${GO_FILE}" "${ABORT_FILE}" "${LOG_FILE}" "${PID_FILE}"

echo "Starting D-theme payload and PX4 adapter in preflight mode."
echo "Aircraft remains disarmed. Select OFFBOARD with the RC transmitter."
echo "Camera=/dev/video0 lidar=/dev/ttyS4"

nohup setsid ros2 launch shelf_px4_bridge shelf_px4.launch.py \
  flight_enabled:=true \
  wait_for_go:=true \
  camera_device:=/dev/video0 \
  code_mode:=qr \
  laser_driver:=gpio \
  gpio_mode:=gpiod \
  gpio_chip:="${LASER_GPIO_CHIP}" \
  gpio_line:="${LASER_GPIO_LINE}" \
  laser_active_high:="${LASER_ACTIVE_HIGH}" \
  </dev/null >"${LOG_FILE}" 2>&1 &
launch_pid="$!"
echo "${launch_pid}" >"${PID_FILE}"
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
  echo "Interrupt detected. Pre-arm abort requested; no disarm is sent."
  exit 130
}
trap on_interrupt INT TERM

tail -n +1 -f "${LOG_FILE}" &
tail_pid="$!"

for _ in $(seq 1 360); do
  if [[ -f "${READY_FILE}" ]] && grep -q '^READY$' "${READY_FILE}"; then
    for component in \
      shelf_code_reader_node \
      shelf_mission_node \
      shelf_laser_node \
      position_pid_controller \
      shelf_px4_adapter; do
      if ! pgrep -f "[${component:0:1}]${component:1}" >/dev/null; then
        stop_tail
        echo "ERROR: required component exited: ${component}"
        exit 1
      fi
    done
    stop_tail
    cat "${READY_FILE}"
    echo
    echo "PREFLIGHT PASSED. PX4 is in OFFBOARD and remains disarmed."
    echo "Next command: ${WS}/shelf_takeoff.sh"
    exit 0
  fi

  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    stop_tail
    echo "ERROR: D mission launch exited before preflight passed."
    tail -n 80 "${LOG_FILE}" || true
    exit 1
  fi
  sleep 0.5
done

stop_tail
touch "${ABORT_FILE}"
echo "ERROR: preflight timed out after 180 seconds."
exit 1
