#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

if [[ $# -lt 1 ]]; then
  echo "Usage: bash scripts/run_experiment_case_rviz.sh <case_dir> [extra launch args...]"
  echo "Example:"
  echo "  bash scripts/run_experiment_case_rviz.sh docs/experiment_records/20260409_102237_random_cabinet_experiment/case_003"
  exit 1
fi

case_dir="$1"
shift || true

params_file="${case_dir}/params.json"
if [[ ! -f "${params_file}" ]]; then
  echo "params.json not found: ${params_file}" >&2
  exit 1
fi

cleanup_medipick_stack

mapfile -t scenario_launch_args < <(
  python3 - "${params_file}" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as handle:
    payload = json.load(handle)

params = payload["params"]
ordered_keys = [
    "random_seed",
    "shelf_levels",
    "shelf_center_x",
    "shelf_center_y",
    "shelf_depth",
    "shelf_width",
    "shelf_bottom_z",
    "shelf_level_gap",
    "shelf_board_thickness",
    "shelf_side_thickness",
    "shelf_back_thickness",
    "target_level",
    "target_gap_index",
    "target_depth_ratio",
    "target_lateral_margin",
    "target_lateral_span",
    "target_box_size_x",
    "target_box_size_y",
    "target_box_size_z",
    "clutter_count",
]

for key in ordered_keys:
    value = params[key]
    if isinstance(value, bool):
        print(f"{key}:={'true' if value else 'false'}")
    else:
        print(f"{key}:={value}")
PY
)

echo "Replaying experiment case from: ${case_dir}"

run_in_workspace ros2 launch medipick_planning_server moveit_mock_demo.launch.py \
  rviz:=true \
  use_ros2_control:=false \
  run_task_manager:=true \
  task_auto_start_on_target:=false \
  task_auto_accept_base_arrival:=true \
  task_auto_accept_lift_arrival:=true \
  publish_target_box_points:=false \
  publish_clutter_points:=false \
  publish_floor_points:=false \
  "${scenario_launch_args[@]}" \
  "$@"
