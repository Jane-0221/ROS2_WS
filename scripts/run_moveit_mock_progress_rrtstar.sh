#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

run_in_workspace ros2 launch medipick_planning_server moveit_mock_demo.launch.py \
  rviz:=true \
  use_ros2_control:=false \
  clutter_count:=0 \
  publish_target_box_points:=false \
  publish_clutter_points:=false \
  publish_floor_points:=false \
  use_simplified_pipeline:=true \
  task_auto_start_on_target:=true \
  task_auto_accept_base_arrival:=true \
  task_auto_accept_lift_arrival:=true \
  task_stage_timeout:=60.0 \
  discrete_lift_mode:=false \
  base_standoff:=0.68 \
  base_lateral_offset:=0.18 \
  prepare_group_name:=arm \
  prepare_offset:=0.06 \
  candidate_planner_id:=RRTstarkConfigDefault \
  direct_final_try_before_prepare:=true \
  final_use_cartesian:=false \
  final_group_name:=mobile_arm \
  final_planner_id:=RRTstarkConfigDefault \
  final_prefer_seeded_ik:=false \
  retreat_use_cartesian:=false \
  retreat_group_name:=mobile_arm \
  retreat_planner_id:=RRTstarkConfigDefault \
  "$@"
