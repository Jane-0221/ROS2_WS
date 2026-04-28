#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

export MEDIPICK_SOFTWARE_GL=0

run_in_workspace ros2 launch medipick_planning_server gazebo_nav2_pick_trial.launch.py \
  target_seed:=0 \
  gazebo_gui:=false \
  rtabmap_viz:=false \
  rviz:=false \
  "$@"
