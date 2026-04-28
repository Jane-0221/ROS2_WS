#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bash "${SCRIPT_DIR}/record_defense_demo_segment.sh" \
  session_name:=rtabmap_mapping_windows_demo \
  backend:=obs \
  command_timeout_sec:=300 \
  -- \
  bash "${SCRIPT_DIR}/run_gazebo_nav2_auto_explore.sh" \
    gazebo_gui:=true rtabmap_viz:=true rviz:=false \
    "$@"
