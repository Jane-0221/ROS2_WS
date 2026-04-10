#!/usr/bin/env bash

set -euo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash

if [[ ! -f install/setup.bash ]]; then
  echo "install/setup.bash not found. Run: colcon build --symlink-install"
  exit 1
fi

source install/setup.bash

ros2 launch medipick_planning_server hardware_demo.launch.py "$@"
