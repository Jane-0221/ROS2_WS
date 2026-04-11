#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash

if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

ros2 launch medipick_moveit_config demo.launch.py "$@"
