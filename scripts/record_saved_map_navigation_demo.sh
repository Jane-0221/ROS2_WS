#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bash "${SCRIPT_DIR}/record_defense_demo_segment.sh" \
  session_name:=saved_map_navigation_demo \
  backend:=obs \
  command_timeout_sec:=240 \
  -- \
  bash "${SCRIPT_DIR}/run_gazebo_nav2_saved_map_validation.sh" gazebo_gui:=true rviz:=true "$@"
