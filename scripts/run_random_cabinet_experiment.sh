#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

count="${1:-5}"
shift || true

seed_start=1001
case_timeout=90
record_video=false
video_fps=2
video_max_width=640
video_max_height=360
video_wait_timeout=30
video_capture_timeout=120
video_window_pattern="RViz|rviz"
launch_extra_args=()

for arg in "$@"; do
  case "${arg}" in
    seed_start:=*)
      seed_start="${arg#seed_start:=}"
      ;;
    case_timeout:=*)
      case_timeout="${arg#case_timeout:=}"
      ;;
    record_video:=*)
      record_video="${arg#record_video:=}"
      ;;
    video_fps:=*)
      video_fps="${arg#video_fps:=}"
      ;;
    video_max_width:=*)
      video_max_width="${arg#video_max_width:=}"
      ;;
    video_max_height:=*)
      video_max_height="${arg#video_max_height:=}"
      ;;
    video_wait_timeout:=*)
      video_wait_timeout="${arg#video_wait_timeout:=}"
      ;;
    video_capture_timeout:=*)
      video_capture_timeout="${arg#video_capture_timeout:=}"
      ;;
    video_window_pattern:=*)
      video_window_pattern="${arg#video_window_pattern:=}"
      ;;
    *)
      launch_extra_args+=("${arg}")
      ;;
  esac
done

timestamp="$(date +%Y%m%d_%H%M%S)"
record_root="${WORKSPACE_DIR}/docs/experiment_records/${timestamp}_random_cabinet_experiment"
mkdir -p "${record_root}"

echo "Experiment record directory: ${record_root}"

for ((i = 1; i <= count; ++i)); do
  seed="$((seed_start + i - 1))"
  case_name="$(printf "case_%03d" "${i}")"
  case_dir="${record_root}/${case_name}"
  mkdir -p "${case_dir}"

  python3 "${SCRIPT_DIR}/generate_random_cabinet_case.py" \
    --seed "${seed}" \
    --clutter-count 0 \
    --output "${case_dir}/params.json"

  python3 "${SCRIPT_DIR}/render_cabinet_experiment_svg.py" \
    --params "${case_dir}/params.json" \
    --output "${case_dir}/scene.svg"

  mapfile -t scenario_launch_args < <(
    python3 - "${case_dir}/params.json" <<'PY'
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

  echo "[${i}/${count}] ${case_name} seed=${seed}"

  log_file="${case_dir}/run.log"
  video_stop_file="${case_dir}/video.stop"
  video_log_file="${case_dir}/video.log"
  video_file="${case_dir}/rviz_capture.avi"
  video_metadata_file="${case_dir}/video.json"
  recorder_pid=""

  rm -f "${video_stop_file}"
  if [[ "${record_video}" == "true" ]]; then
    python3 "${SCRIPT_DIR}/record_x11_window.py" \
      --output "${video_file}" \
      --metadata-output "${video_metadata_file}" \
      --window-pattern "${video_window_pattern}" \
      --fps "${video_fps}" \
      --max-width "${video_max_width}" \
      --max-height "${video_max_height}" \
      --wait-timeout "${video_wait_timeout}" \
      --capture-timeout "${video_capture_timeout}" \
      --stop-file "${video_stop_file}" >"${video_log_file}" 2>&1 &
    recorder_pid="$!"
  fi

  if timeout "${case_timeout}s" bash "${SCRIPT_DIR}/run_moveit_mock_demo.sh" \
    "rviz:=${record_video}" \
    use_ros2_control:=false \
    publish_target_box_points:=false \
    publish_clutter_points:=false \
    publish_floor_points:=false \
    task_auto_start_on_target:=true \
    task_auto_accept_base_arrival:=true \
    task_auto_accept_lift_arrival:=true \
    "${scenario_launch_args[@]}" \
    "${launch_extra_args[@]}" >"${log_file}" 2>&1; then
    :
  fi

  if [[ -n "${recorder_pid}" ]]; then
    touch "${video_stop_file}"
    wait "${recorder_pid}" || true
    rm -f "${video_stop_file}"
  fi

  python3 "${SCRIPT_DIR}/parse_pick_experiment_log.py" \
    --log "${log_file}" \
    --output "${case_dir}/result.json"
done

python3 "${SCRIPT_DIR}/analyze_random_cabinet_experiment.py" --root "${record_root}"
python3 "${SCRIPT_DIR}/build_experiment_record_report.py" --root "${record_root}"

echo "Experiment record generated:"
echo "  ${record_root}"
