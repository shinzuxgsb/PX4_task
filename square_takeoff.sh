#!/usr/bin/env bash
set -euo pipefail

READY_FILE="/tmp/square_preflight_ready"
GO_FILE="/tmp/square_takeoff_go"
ABORT_FILE="/tmp/square_abort"
LOG_FILE="/tmp/square_mission.log"
PID_FILE="/tmp/square_mission.pid"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "ERROR: no preflight process. Run ~/mavlink_ws/square_preflight.sh first."
  exit 1
fi

mission_pid="$(cat "${PID_FILE}")"
if ! kill -0 "${mission_pid}" 2>/dev/null; then
  echo "ERROR: preflight process is not running."
  exit 1
fi

if [[ ! -f "${READY_FILE}" ]] || ! grep -q '^READY$' "${READY_FILE}"; then
  echo "ERROR: preflight has not passed. Run ~/mavlink_ws/square_preflight.sh first."
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
echo "GO sent. Automatic arm, takeoff, square mission, and landing started."
echo "Press Ctrl-C to request landing. Monitoring ${LOG_FILE} ..."

tail --pid="${mission_pid}" -n +1 -f "${LOG_FILE}"
