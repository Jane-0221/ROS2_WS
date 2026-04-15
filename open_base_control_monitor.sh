#!/usr/bin/env bash

set -euo pipefail

source_setup() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "$1"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

cd "$(dirname "$0")"
source_setup /opt/ros/humble/setup.bash

if [[ ! -f install/setup.bash ]]; then
  echo "install/setup.bash not found. Please build the workspace first."
  echo "Command: colcon build --symlink-install"
  read -rp "Press Enter to close..."
  exit 1
fi

source_setup install/setup.bash
exec python3 /home/jszn/robot_ros2_ws/scripts/base_control_monitor.py
