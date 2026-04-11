#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

run_in_workspace ros2 launch medipick_planning_server moveit_mock_demo.launch.py \
  rviz:=true \
  use_ros2_control:=false \
  random_seed:=5004 \
  shelf_levels:=6 \
  target_gap_index:=0 \
  target_level:=3 \
  shelf_width:=0.885 \
  shelf_depth:=0.227 \
  shelf_center_x:=0.905 \
  shelf_center_y:=-0.003 \
  shelf_bottom_z:=0.411 \
  shelf_level_gap:=0.262 \
  shelf_board_thickness:=0.026 \
  shelf_side_thickness:=0.033 \
  shelf_back_thickness:=0.024 \
  target_depth_ratio:=0.071 \
  target_lateral_margin:=0.118 \
  target_lateral_span:=0.075 \
  target_box_size_x:=0.105 \
  target_box_size_y:=0.111 \
  target_box_size_z:=0.094 \
  clutter_count:=0 \
  publish_target_box_points:=false \
  publish_clutter_points:=false \
  publish_floor_points:=false \
  run_task_manager:=true \
  task_auto_start_on_target:=false \
  task_auto_accept_base_arrival:=true \
  task_auto_accept_lift_arrival:=true \
  use_simplified_pipeline:=true \
  discrete_lift_mode:=false \
  task_stage_timeout:=60.0 \
  base_standoff:=0.60 \
  base_lateral_offset:=0.18 \
  adaptive_workspace_enabled:=true \
  pre_insert_offset:=0.06 \
  pre_insert_candidate_count:=3 \
  pre_insert_first_candidate_only:=false \
  pre_insert_select_pose_only:=true \
  pre_insert_try_arm_first:=true \
  pre_insert_arm_group_name:=arm \
  pre_insert_limit_base_motion:=true \
  pre_insert_base_translation_limit_m:=0.30 \
  pre_insert_base_rotation_limit_deg:=25.0 \
  pre_insert_group_name:=mobile_arm \
  insert_group_name:=arm \
  insert_fallback_group_name:=mobile_arm \
  insert_force_local_when_close:=true \
  insert_limit_base_motion:=true \
  insert_base_translation_limit_m:=0.07 \
  insert_base_rotation_limit_deg:=8.0 \
  safe_retreat_group_name:=arm \
  retreat_retrace_insert_first:=true \
  lift_band_half_width:=0.03 \
  lift_use_end_effector_height_alignment:=true \
  lift_end_effector_target_offset:=0.0 \
  direct_final_try_before_prepare:=true \
  allow_partial_direct_prepare:=false \
  use_entry_plane_prepare:=true \
  cabinet_entry_margin:=0.05 \
  final_use_cartesian:=false \
  retreat_use_cartesian:=false \
  final_prefer_seeded_ik:=false \
  "$@"
