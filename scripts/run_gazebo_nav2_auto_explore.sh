#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

cleanup_medipick_stack

export MEDIPICK_SOFTWARE_GL="${MEDIPICK_SOFTWARE_GL:-0}"

if [[ "${MEDIPICK_SOFTWARE_GL}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export QT_XCB_FORCE_SOFTWARE_OPENGL=1
  export QT_OPENGL=software
fi

run_in_workspace ros2 launch medipick_planning_server gazebo_nav2_auto_explore.launch.py "$@"
