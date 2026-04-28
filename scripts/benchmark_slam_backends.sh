#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

RUNS="${RUNS:-1}"
TIMEOUT_SEC="${TIMEOUT_SEC:-75}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_DIR}/log/slam_bench_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "${OUTPUT_DIR}"

default_profiles=(
  "rtabmap_default"
  "rtabmap_loop_loose"
  "slam_toolbox_default"
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

last_rtabmap_frame() {
  local file="$1"
  local value
  value="$(rg -o "rtabmap \\([0-9]+\\)" "${file}" 2>/dev/null | tail -n 1 || true)"
  if [[ -z "${value}" ]]; then
    echo "0"
    return
  fi
  echo "${value}" | tr -cd '0-9'
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
    rtabmap_loop_loose)
      launch_args=(
        "slam_backend:=rtabmap"
        "rtabmap_extra_args:=--Vis/MinInliers 8 --RGBD/OptimizeMaxError 5.0 --Icp/PointToPlaneMinComplexity 0.005"
      )
      ;;
    slam_toolbox_default)
      launch_args=("slam_backend:=slam_toolbox")
      ;;
    *)
      echo "Unknown profile: ${profile}" >&2
      return 2
      ;;
  esac

  echo "[bench] profile=${profile} run=${run_index} timeout=${TIMEOUT_SEC}s"
  cleanup_medipick_stack

  local status=0
  set +e
  timeout --signal=INT "${TIMEOUT_SEC}s" \
    bash "${SCRIPT_DIR}/run_gazebo_nav2_pick_trial_lite.sh" "${launch_args[@]}" \
    >"${log_file}" 2>&1
  status=$?
  set -e

  cleanup_medipick_stack

  if [[ "${status}" -ne 0 && "${status}" -ne 124 && "${status}" -ne 130 ]]; then
    echo "[bench] profile=${profile} run=${run_index} failed with status=${status}" >&2
  fi

  local stage_goals coarse_hits nav_begins known_cells imu_errors tf_errors loop_rejects rtabmap_frame
  stage_goals="$(count_matches "Nav2 开始导航到阶段导航点" "${log_file}")"
  coarse_hits="$(count_matches "粗到位阈值" "${log_file}")"
  nav_begins="$(count_matches "Begin navigating from current location" "${log_file}")"
  known_cells="$(extract_last_value "known_cells=[0-9]+" "${log_file}")"
  imu_errors="$(count_matches "IMU received doesn't have orientation set|medipick/base_theta_link/base_imu.*does not exist" "${log_file}")"
  tf_errors="$(count_matches "TF of received image|Timed out waiting for transform|Invalid frame ID" "${log_file}")"
  loop_rejects="$(count_matches "Loop closure .* rejected|Rejected loop closure" "${log_file}")"
  rtabmap_frame="$(last_rtabmap_frame "${log_file}")"

  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
    "${profile}" \
    "${run_index}" \
    "${status}" \
    "${stage_goals}" \
    "${coarse_hits}" \
    "${nav_begins}" \
    "${known_cells}" \
    "${imu_errors}" \
    "${tf_errors}" \
    "${loop_rejects}" \
    >>"${OUTPUT_DIR}/summary.tsv"

  echo "[bench] ${profile} run=${run_index} status=${status} stages=${stage_goals} coarse=${coarse_hits} nav=${nav_begins} known_cells=${known_cells} imu_err=${imu_errors} tf_err=${tf_errors} loop_rejects=${loop_rejects} rtabmap_frame=${rtabmap_frame}"
}

printf "profile\trun\tstatus\tstage_goals\tcoarse_hits\tnav_begins\tknown_cells\timu_errors\ttf_errors\tloop_rejects\n" \
  >"${OUTPUT_DIR}/summary.tsv"

for profile in "${profiles[@]}"; do
  for ((run_index=1; run_index<=RUNS; run_index++)); do
    run_profile "${profile}" "${run_index}"
  done
done

echo "[bench] summary: ${OUTPUT_DIR}/summary.tsv"
