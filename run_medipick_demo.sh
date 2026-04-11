#!/usr/bin/env bash

set -euo pipefail

source_setup() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "$1"
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
}

cd "$(dirname "$0")"
source_setup /opt/ros/humble/setup.bash

if [[ ! -f install/setup.bash ]]; then
  echo "install/setup.bash not found. Run: colcon build --symlink-install"
  exit 1
fi

source_setup install/setup.bash

if ! ros2 pkg prefix moveit_ros_move_group >/dev/null 2>&1; then
  echo "moveit_ros_move_group is missing."
  echo "Run: sudo ./scripts/install_medipick_host_deps.sh"
  exit 1
fi

if [[ ! -f /etc/udev/rules.d/99-obsensor-libusb.rules ]]; then
  echo "Orbbec udev rules are missing."
  echo "Run: sudo ./scripts/install_medipick_host_deps.sh"
  exit 1
fi

launch_args=("$@")
has_rviz_arg=0
for arg in "${launch_args[@]}"; do
  if [[ "$arg" == rviz:=* ]]; then
    has_rviz_arg=1
    break
  fi
done

if [[ "$has_rviz_arg" -eq 0 ]]; then
  launch_args=("rviz:=true" "${launch_args[@]}")
fi

ros2 launch medipick_planning_server hardware_demo.launch.py "${launch_args[@]}"
