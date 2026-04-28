#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

BACKEND="auto"
if [[ $# -gt 0 && "${1}" != *:=* ]]; then
  BACKEND="${1}"
  shift
fi

run_in_workspace ros2 launch medipick_planning_server planning_demo.launch.py \
  backend:="${BACKEND}" \
  rviz:=false \
  "$@"
