#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

seed="${MEDIPICK_RANDOM_SEED:-$((RANDOM * RANDOM + RANDOM))}"
eval "$(
python3 - "${seed}" <<'PY'
import random
import sys

seed = int(sys.argv[1])
rng = random.Random(seed)
levels = rng.randint(4, 7)
target_gap = rng.randint(0, levels - 2)

def sample(a, b):
    return f"{rng.uniform(a, b):.3f}"

print(f"levels={levels}")
print(f"target_gap={target_gap}")
print(f"width={sample(0.58, 0.92)}")
print(f"depth={sample(0.18, 0.32)}")
print(f"center_x={sample(0.80, 0.98)}")
print(f"center_y={sample(-0.12, 0.12)}")
print(f"bottom_z={sample(0.40, 0.58)}")
print(f"level_gap={sample(0.19, 0.30)}")
print(f"board_thickness={sample(0.022, 0.045)}")
print(f"side_thickness={sample(0.030, 0.060)}")
print(f"back_thickness={sample(0.020, 0.050)}")
print(f"depth_ratio={sample(0.04, 0.14)}")
print(f"lateral_margin={sample(0.07, 0.14)}")
print(f"lateral_span={sample(0.02, 0.10)}")
print(f"target_box_x={sample(0.05, 0.11)}")
print(f"target_box_y={sample(0.08, 0.16)}")
print(f"target_box_z={sample(0.04, 0.10)}")
PY
)"

levels="${MEDIPICK_SHELF_LEVELS:-${levels}}"
target_gap="${MEDIPICK_TARGET_GAP_INDEX:-${target_gap}}"
width="${MEDIPICK_SHELF_WIDTH:-${width}}"
depth="${MEDIPICK_SHELF_DEPTH:-${depth}}"
center_x="${MEDIPICK_SHELF_CENTER_X:-${center_x}}"
center_y="${MEDIPICK_SHELF_CENTER_Y:-${center_y}}"
bottom_z="${MEDIPICK_SHELF_BOTTOM_Z:-${bottom_z}}"
level_gap="${MEDIPICK_SHELF_LEVEL_GAP:-${level_gap}}"
board_thickness="${MEDIPICK_SHELF_BOARD_THICKNESS:-${board_thickness}}"
side_thickness="${MEDIPICK_SHELF_SIDE_THICKNESS:-${side_thickness}}"
back_thickness="${MEDIPICK_SHELF_BACK_THICKNESS:-${back_thickness}}"
depth_ratio="${MEDIPICK_TARGET_DEPTH_RATIO:-${depth_ratio}}"
lateral_margin="${MEDIPICK_TARGET_LATERAL_MARGIN:-${lateral_margin}}"
lateral_span="${MEDIPICK_TARGET_LATERAL_SPAN:-${lateral_span}}"
target_box_x="${MEDIPICK_TARGET_BOX_SIZE_X:-${target_box_x}}"
target_box_y="${MEDIPICK_TARGET_BOX_SIZE_Y:-${target_box_y}}"
target_box_z="${MEDIPICK_TARGET_BOX_SIZE_Z:-${target_box_z}}"

echo "Random mock scene parameters:"
echo "  random_seed=${seed}"
echo "  shelf_levels=${levels}"
echo "  target_gap_index=${target_gap}"
echo "  shelf_width=${width}"
echo "  shelf_depth=${depth}"
echo "  shelf_center_x=${center_x}"
echo "  shelf_center_y=${center_y}"
echo "  shelf_bottom_z=${bottom_z}"
echo "  shelf_level_gap=${level_gap}"
echo "  shelf_board_thickness=${board_thickness}"
echo "  shelf_side_thickness=${side_thickness}"
echo "  shelf_back_thickness=${back_thickness}"
echo "  target_depth_ratio=${depth_ratio}"
echo "  target_lateral_margin=${lateral_margin}"
echo "  target_lateral_span=${lateral_span}"
echo "  target_box_size=(${target_box_x}, ${target_box_y}, ${target_box_z})"

run_in_workspace ros2 launch medipick_planning_server moveit_mock_demo.launch.py \
  random_seed:="${seed}" \
  shelf_levels:="${levels}" \
  target_gap_index:="${target_gap}" \
  shelf_width:="${width}" \
  shelf_depth:="${depth}" \
  shelf_center_x:="${center_x}" \
  shelf_center_y:="${center_y}" \
  shelf_bottom_z:="${bottom_z}" \
  shelf_level_gap:="${level_gap}" \
  shelf_board_thickness:="${board_thickness}" \
  shelf_side_thickness:="${side_thickness}" \
  shelf_back_thickness:="${back_thickness}" \
  target_depth_ratio:="${depth_ratio}" \
  target_lateral_margin:="${lateral_margin}" \
  target_lateral_span:="${lateral_span}" \
  target_box_size_x:="${target_box_x}" \
  target_box_size_y:="${target_box_y}" \
  target_box_size_z:="${target_box_z}" \
  clutter_count:=0 \
  publish_target_box_points:=false \
  publish_clutter_points:=false \
  publish_floor_points:=false \
  task_auto_start_on_target:=true \
  task_auto_accept_base_arrival:=true \
  "$@"
