#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

RUNS="${RUNS:-1}"
TIMEOUT_SEC="${TIMEOUT_SEC:-960}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_DIR}/log/slam_validation_bench_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "${OUTPUT_DIR}"

default_profiles=(
  "rtabmap_default"
  "rtabmap_force_3dof"
  "rtabmap_base_scan"
  "rtabmap_loop_loose"
  "slam_toolbox_default"
  "cartographer_default"
)

profiles=()
if [[ -n "${PROFILES:-}" ]]; then
  IFS=',' read -r -a profiles <<< "${PROFILES}"
else
  profiles=("${default_profiles[@]}")
fi

extract_last_value() {
  local pattern="$1"
  local file="$2"
  local value
  value="$(rg -o "${pattern}" "${file}" 2>/dev/null | tail -n 1 || true)"
  if [[ -z "${value}" ]]; then
    echo "0"
    return
  fi
  echo "${value##*=}"
}

count_matches() {
  local pattern="$1"
  local file="$2"
  rg -c "${pattern}" "${file}" 2>/dev/null || echo "0"
}

run_profile() {
  local profile="$1"
  local run_index="$2"
  local log_file="${OUTPUT_DIR}/${profile}_run${run_index}.log"
  local -a launch_args=()

  case "${profile}" in
    rtabmap_default)
      launch_args=("slam_backend:=rtabmap")
      ;;
    rtabmap_force_3dof)
      launch_args=("slam_backend:=rtabmap" "rtabmap_force_3dof:=true")
      ;;
    rtabmap_base_scan)
      launch_args=(
        "slam_backend:=rtabmap"
        "rtabmap_scan_generator:=pointcloud_to_laserscan"
        "rtabmap_scan_frame:=base_link"
      )
      ;;
    rtabmap_loop_loose)
      launch_args=(
        "slam_backend:=rtabmap"
        "rtabmap_extra_args:=--Vis/MinInliers 8 --RGBD/OptimizeMaxError 5.0 --Icp/PointToPlaneMinComplexity 0.005"
      )
      ;;
    slam_toolbox_default)
      launch_args=("slam_backend:=slam_toolbox" "slam_map_topic:=/map")
      ;;
    cartographer_default)
      launch_args=("slam_backend:=cartographer" "slam_map_topic:=/map")
      ;;
    *)
      echo "Unknown profile: ${profile}" >&2
      return 2
      ;;
  esac

  echo "[slam-validation] profile=${profile} run=${run_index} timeout=${TIMEOUT_SEC}s"
  cleanup_medipick_stack

  local status=0
  set +e
  timeout --signal=INT "${TIMEOUT_SEC}s" \
    bash "${SCRIPT_DIR}/run_gazebo_nav2_slam_validation.sh" "${launch_args[@]}" \
    >"${log_file}" 2>&1
  status=$?
  set -e

  cleanup_medipick_stack

  if [[ "${status}" -ne 0 && "${status}" -ne 124 && "${status}" -ne 130 ]]; then
    echo "[slam-validation] profile=${profile} run=${run_index} failed with status=${status}" >&2
  fi

  local sent_goals completed_goals failed_goals loop_closures known_cells controller_timeouts tf_errors out_of_bounds finished
  sent_goals="$(count_matches "发送验证路点" "${log_file}")"
  completed_goals="$(count_matches "验证路点完成" "${log_file}")"
  failed_goals="$(count_matches "验证路点失败" "${log_file}")"
  loop_closures="$(count_matches "检测到 RTAB-Map 回环闭合" "${log_file}")"
  known_cells="$(extract_last_value "known_cells=[0-9]+" "${log_file}")"
  controller_timeouts="$(count_matches "Controller patience exceeded" "${log_file}")"
  tf_errors="$(count_matches "TF of received image|Timed out waiting for transform|Invalid frame ID|Exception in transformPose" "${log_file}")"
  out_of_bounds="$(count_matches "Robot is out of bounds of the costmap" "${log_file}")"
  finished="$(count_matches "SLAM 回环验证完成" "${log_file}")"

  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
    "${profile}" \
    "${run_index}" \
    "${status}" \
    "${sent_goals}" \
    "${completed_goals}" \
    "${failed_goals}" \
    "${loop_closures}" \
    "${known_cells}" \
    "${controller_timeouts}" \
    "${tf_errors}" \
    "${out_of_bounds}" \
    >>"${OUTPUT_DIR}/summary.tsv"

  echo "[slam-validation] ${profile} run=${run_index} status=${status} sent=${sent_goals} completed=${completed_goals} failed=${failed_goals} loops=${loop_closures} known_cells=${known_cells} controller_timeout=${controller_timeouts} tf_err=${tf_errors} out_of_bounds=${out_of_bounds} finished=${finished}"
}

printf "profile\trun\tstatus\tsent_goals\tcompleted_goals\tfailed_goals\tloop_closures\tknown_cells\tcontroller_timeouts\ttf_errors\tout_of_bounds\n" \
  >"${OUTPUT_DIR}/summary.tsv"

for profile in "${profiles[@]}"; do
  for ((run_index=1; run_index<=RUNS; run_index++)); do
    run_profile "${profile}" "${run_index}"
  done
done

echo "[slam-validation] summary: ${OUTPUT_DIR}/summary.tsv"
