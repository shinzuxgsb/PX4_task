#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TARGET_WS="${1:-$HOME/mavlink_ws}"
FORCE=0
for arg in "$@"; do
  [[ "$arg" == "--force" ]] && FORCE=1
done
if [[ "${1:-}" == "--force" ]]; then
  TARGET_WS="$HOME/mavlink_ws"
fi

mkdir -p "$TARGET_WS/src"
for pkg in shelf_scan_pkg pid_control_pkg shelf_px4_bridge; do
  src="$ROOT/ros2_src/$pkg"
  dst="$TARGET_WS/src/$pkg"
  if [[ -e "$dst" && "$FORCE" -ne 1 ]]; then
    echo "SKIP existing package: $dst (use --force to replace)"
    continue
  fi
  if [[ -e "$dst" ]]; then
    backup="$dst.bak-$(date +%Y%m%d_%H%M%S)"
    mv "$dst" "$backup"
    echo "Backup: $backup"
  fi
  cp -a "$src" "$dst"
  echo "Installed package: $pkg"
done

for name in shelf_preflight.sh shelf_takeoff.sh shelf_abort.sh laser_test.sh; do
  dst="$TARGET_WS/$name"
  if [[ -e "$dst" && "$FORCE" -ne 1 ]]; then
    echo "SKIP existing script: $dst"
    continue
  fi
  if [[ -e "$dst" ]]; then
    cp -p "$dst" "$dst.bak-$(date +%Y%m%d_%H%M%S)"
  fi
  cp -p "$ROOT/scripts/$name" "$dst"
  chmod +x "$dst"
done

mkdir -p "$TARGET_WS/config"
if [[ ! -e "$TARGET_WS/config/groundstation.env" ]]; then
  cp -p "$ROOT/config/groundstation.env" "$TARGET_WS/config/"
  echo "Installed config/groundstation.env"
else
  echo "KEEP existing config/groundstation.env (edit its WS URL if needed)"
fi

source /opt/ros/humble/setup.bash
if [[ -f "$HOME/cartographer_ws/install/setup.bash" ]]; then
  source "$HOME/cartographer_ws/install/setup.bash"
fi
if [[ -f "$TARGET_WS/install/setup.bash" ]]; then
  source "$TARGET_WS/install/setup.bash"
fi

cd "$TARGET_WS"
colcon build --symlink-install --packages-select \
  shelf_scan_pkg pid_control_pkg shelf_px4_bridge

echo
echo "Install complete. Compare config/mavlink_bridge.reference.yaml manually."
echo "Then run: $TARGET_WS/shelf_preflight.sh"
