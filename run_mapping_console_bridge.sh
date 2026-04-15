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
source_setup install/setup.bash

ros2 launch robot_hardware mapping_app_bridge.launch.py "$@"
