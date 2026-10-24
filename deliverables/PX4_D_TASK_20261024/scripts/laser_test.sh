#!/usr/bin/env bash
set -euo pipefail

# Safe manual test for the D-theme laser pointer.
# Usage:
#   ./laser_test.sh low 10     # active-low module (default wiring)
#   ./laser_test.sh high 10    # test opposite polarity
#   ./laser_test.sh off

CHIP="${LASER_GPIO_CHIP:-gpiochip1}"
LINE="${LASER_GPIO_LINE:-4}"
MAX_SECONDS=10

usage() {
  cat <<EOF
Usage:
  $0 low [seconds]     Drive active-low output (0=on, 1=off)
  $0 high [seconds]    Drive active-high output (1=on, 0=off)
  $0 off               Return the line to active-low idle state

Defaults: chip=$CHIP line=$LINE seconds=$MAX_SECONDS
Environment overrides: LASER_GPIO_CHIP, LASER_GPIO_LINE
EOF
}

if ! command -v gpioset >/dev/null 2>&1; then
  echo "ERROR: gpioset is missing. Install it with: sudo apt install gpiod"
  exit 1
fi
if ! [[ "${CHIP}" =~ ^gpiochip[0-9]+$ ]]; then
  echo "ERROR: invalid GPIO chip: ${CHIP}"
  exit 1
fi
if ! [[ "${LINE}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: invalid GPIO line: ${LINE}"
  exit 1
fi
if ! [[ -e "/dev/${CHIP}" ]]; then
  echo "ERROR: /dev/${CHIP} does not exist."
  exit 1
fi
if pgrep -f 'shelf_laser_[n]ode|shelf_preflight[.]sh|shelf_takeoff[.]sh' >/dev/null; then
  echo "ERROR: a shelf mission/laser process is active; stop it before manual GPIO tests."
  exit 1
fi

line_status() {
  gpioinfo "${CHIP}" 2>/dev/null |
    awk -v wanted="${LINE}" '$1 == "line" && $2 == wanted ":" {print; found=1} END {if (!found) print "line status unavailable"}'
}

set_idle_low() {
  gpioset --mode=time --sec=0 --usec=200000 "${CHIP}" "${LINE}=1" >/dev/null 2>&1
}

set_idle_high() {
  gpioset --mode=time --sec=0 --usec=200000 "${CHIP}" "${LINE}=0" >/dev/null 2>&1
}

mode="${1:-low}"
if [[ "${mode}" == "-h" || "${mode}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${mode}" == "off" ]]; then
  set_idle_low
  echo "Laser set to active-low idle state: ${CHIP} line ${LINE}=1."
  line_status
  exit 0
fi
if [[ "${mode}" != "low" && "${mode}" != "high" ]]; then
  usage
  exit 2
fi

seconds="${2:-${MAX_SECONDS}}"
if ! [[ "${seconds}" =~ ^[0-9]+$ ]] || (( seconds < 1 || seconds > MAX_SECONDS )); then
  echo "ERROR: seconds must be an integer from 1 to ${MAX_SECONDS}."
  exit 1
fi

if [[ "${mode}" == "low" ]]; then
  active_level=0
  idle_level=1
else
  active_level=1
  idle_level=0
fi

cleanup() {
  gpioset --mode=time --sec=0 --usec=200000 \
    "${CHIP}" "${LINE}=${idle_level}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "WARNING: point the laser away from people and cameras. Do not run with props installed."
echo "GPIO: ${CHIP} line ${LINE}; mode=${mode}; active_level=${active_level}; duration=${seconds}s"
echo "Before: $(line_status)"
date '+ON  %Y-%m-%d %H:%M:%S'
gpioset --mode=time --sec="${seconds}" "${CHIP}" "${LINE}=${active_level}" >/dev/null 2>&1
date '+OFF %Y-%m-%d %H:%M:%S'

# Make the intended off state explicit after the timed process releases the line.
cleanup
trap - EXIT INT TERM
echo "After: $(line_status)"
echo "Laser restored to idle level ${idle_level}."
