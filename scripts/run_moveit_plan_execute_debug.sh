#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

# Manual MoveIt debugging entry:
# - enable ros2_control so RViz exposes Plan & Execute
# - keep the task manager off so nothing steals control from the MotionPlanning panel
run_in_workspace ros2 launch medipick_planning_server moveit_mock_demo.launch.py \
  rviz:=true \
  use_ros2_control:=true \
  run_task_manager:=false \
  clutter_count:=0 \
  publish_target_box_points:=false \
  publish_clutter_points:=false \
  publish_floor_points:=false \
  "$@"
