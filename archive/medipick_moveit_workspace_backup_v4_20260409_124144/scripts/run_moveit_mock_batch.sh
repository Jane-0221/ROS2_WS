#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

count="${1:-5}"
shift || true

pass_count=0

for ((i = 1; i <= count; ++i)); do
  seed="$((1000 + i))"
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

  echo "[${i}/${count}] seed=${seed} levels=${levels} gap=${target_gap} width=${width} depth=${depth} center=(${center_x},${center_y}) bottom_z=${bottom_z} level_gap=${level_gap} board=${board_thickness} side=${side_thickness} back=${back_thickness} target_box=(${target_box_x},${target_box_y},${target_box_z})"

  log_file="$(mktemp)"
  if timeout 60s bash "${SCRIPT_DIR}/run_moveit_mock_demo.sh" \
    rviz:=false \
    use_ros2_control:=false \
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
    "$@" >"${log_file}" 2>&1; then
    :
  fi

  if rg -q "Stage -> COMPLETED" "${log_file}"; then
    echo "  PASS"
    pass_count=$((pass_count + 1))
  else
    echo "  FAIL"
    tail -n 20 "${log_file}" || true
  fi
  rm -f "${log_file}"
done

echo "Batch result: ${pass_count}/${count} completed."
