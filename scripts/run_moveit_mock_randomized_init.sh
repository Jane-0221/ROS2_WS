#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

run_in_workspace ros2 launch medipick_planning_server moveit_mock_demo.launch.py \
  randomize_initial_pose:=true \
  discrete_lift_mode:=true \
  task_auto_start_on_target:=false \
  clutter_count:=0 \
  publish_target_box_points:=false \
  publish_clutter_points:=false \
  publish_floor_points:=false \
  "$@"
