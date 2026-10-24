#!/usr/bin/env bash
set -euo pipefail

WS="/home/orangepi/mavlink_ws"
READY_FILE="/tmp/shelf_preflight_ready"
GO_FILE="/tmp/shelf_takeoff_go"
ABORT_FILE="/tmp/shelf_abort"
LOG_FILE="/tmp/shelf_px4_launch.log"
PID_FILE="/tmp/shelf_px4.launch.pid"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "ERROR: no D mission preflight. Run ${WS}/shelf_preflight.sh first."
  exit 1
fi
launch_pid="$(cat "${PID_FILE}")"
if ! kill -0 "${launch_pid}" 2>/dev/null; then
  echo "ERROR: D mission preflight process is not running."
  exit 1
fi
if [[ ! -f "${READY_FILE}" ]] || ! grep -q '^READY$' "${READY_FILE}"; then
  echo "ERROR: preflight has not passed."
  exit 1
fi

on_interrupt() {
  touch "${ABORT_FILE}"
  echo
  echo "Interrupt detected. PX4 landing/abort requested."
  exit 130
}
trap on_interrupt INT TERM

touch "${GO_FILE}"
echo "GO sent: PX4 arm, Offboard release, shelf sweep, return and Land."
echo "Press Ctrl-C to request landing."
tail --pid="${launch_pid}" -n +1 -f "${LOG_FILE}"
