#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bash "${SCRIPT_DIR}/record_defense_demo_segment.sh" \
  session_name:=saved_map_pick_camera_demo \
  backend:=obs \
  command_timeout_sec:=480 \
  -- \
  bash "${SCRIPT_DIR}/run_gazebo_nav2_saved_map_pick_trial.sh" \
    gazebo_gui:=true rviz:=true camera_view:=true \
    target_seed:=0 \
    target_entity_name:=aisle_b_right_04_l04_s03 \
    "$@"
