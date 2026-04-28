#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-600}"
SEEDS="${SEEDS:-0 1 2 3 4}"
DELIVERY_GOAL_X="${DELIVERY_GOAL_X:-2.15}"
DELIVERY_GOAL_Y="${DELIVERY_GOAL_Y:-0.0}"
DELIVERY_GOAL_YAW_DEG="${DELIVERY_GOAL_YAW_DEG:-180.0}"

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${LOG_DIR:-${PWD}/docs/paper/grasp/artifacts/logs/random_pick_batch_${TIMESTAMP}}"
CSV_PATH="${CSV_PATH:-${PWD}/docs/paper/grasp/artifacts/metrics/saved_map_pick_random_batch_${TIMESTAMP}.csv}"

mkdir -p "${LOG_DIR}"
mkdir -p "$(dirname "${CSV_PATH}")"

printf 'seed,timeout_sec,exit_code,target_name,nav_arrived,suction_success,delivery_arrived,release_done,task_completed,return_to_stow_violation,log_path\n' > "${CSV_PATH}"

run_one() {
  local seed="$1"
  local log_path="${LOG_DIR}/seed_${seed}.log"
  local exit_code=0
  local target_name=""
  local nav_arrived="false"
  local suction_success="false"
  local delivery_arrived="false"
  local release_done="false"
  local task_completed="false"
  local return_to_stow_violation="false"
  local run_pid=0
  local task_finished="false"
  local start_epoch=0

  cleanup_medipick_stack

  timeout --signal=INT "${RUN_TIMEOUT_SEC}s" bash "${SCRIPT_DIR}/run_gazebo_nav2_saved_map_pick_and_place_trial.sh" \
    gazebo_gui:=false \
    rviz:=false \
    target_seed:="${seed}" \
    delivery_goal_x:="${DELIVERY_GOAL_X}" \
    delivery_goal_y:="${DELIVERY_GOAL_Y}" \
    delivery_goal_yaw_deg:="${DELIVERY_GOAL_YAW_DEG}" \
    > "${log_path}" 2>&1 &
  run_pid=$!
  start_epoch="$(date +%s)"

  while kill -0 "${run_pid}" >/dev/null 2>&1; do
    if grep -q "放置点释放动作完成" "${log_path}" || grep -q "Stage -> FAILED" "${log_path}"; then
      task_finished="true"
      kill -INT "${run_pid}" >/dev/null 2>&1 || true
      break
    fi

    if (( "$(date +%s)" - start_epoch >= RUN_TIMEOUT_SEC )); then
      break
    fi

    sleep 2
  done

  if ! wait "${run_pid}"; then
    exit_code=$?
  fi

  if [[ "${task_finished}" == "true" && "${exit_code}" -eq 130 ]]; then
    exit_code=0
  fi

  if grep -q "到达抓取工作位" "${log_path}"; then
    nav_arrived="true"
  fi
  if grep -q "removed after suction success" "${log_path}"; then
    suction_success="true"
  fi
  if grep -q "已到达放置点" "${log_path}"; then
    delivery_arrived="true"
  fi
  if grep -q "放置点释放动作完成" "${log_path}"; then
    release_done="true"
  fi
  if grep -q "Stage -> COMPLETED" "${log_path}"; then
    task_completed="true"
  fi
  if grep -q "return_to_stow" "${log_path}" && grep -q "path tolerance violation" "${log_path}"; then
    return_to_stow_violation="true"
  fi

  target_name="$(python3 - <<'PY' "${log_path}"
from pathlib import Path
import re
import sys

text = Path(sys.argv[1]).read_text(errors="ignore")
patterns = [
    r"Gazebo target '([^']+)' removed after suction success\.",
    r"目标药盒.*?:\s*([A-Za-z0-9_]+)",
    r"target_entity_name:=([A-Za-z0-9_]+)",
]
for pattern in patterns:
    match = re.search(pattern, text)
    if match:
        print(match.group(1))
        raise SystemExit
print("")
PY
)"

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${seed}" \
    "${RUN_TIMEOUT_SEC}" \
    "${exit_code}" \
    "${target_name}" \
    "${nav_arrived}" \
    "${suction_success}" \
    "${delivery_arrived}" \
    "${release_done}" \
    "${task_completed}" \
    "${return_to_stow_violation}" \
    "${log_path}" \
    >> "${CSV_PATH}"
}

for seed in ${SEEDS}; do
  echo "[saved-map-pick-batch] running seed=${seed}"
  run_one "${seed}"
done

echo "[saved-map-pick-batch] csv=${CSV_PATH}"
echo "[saved-map-pick-batch] logs=${LOG_DIR}"
