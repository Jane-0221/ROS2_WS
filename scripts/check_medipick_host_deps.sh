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

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

source_setup /opt/ros/humble/setup.bash

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  source_setup "${WORKSPACE_ROOT}/install/setup.bash"
fi

status=0

check_pkg() {
  local pkg="$1"
  if ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "[ok] package ${pkg}"
  else
    echo "[missing] package ${pkg}"
    status=1
  fi
}

echo "Checking ROS packages"
check_pkg control_msgs
check_pkg moveit_planners_ompl
check_pkg moveit_ros_move_group
check_pkg moveit_ros_perception
check_pkg moveit_simple_controller_manager
check_pkg orbbec_camera
check_pkg robot_hardware
check_pkg medipick_planning_server

echo
echo "Checking Orbbec USB enumeration"
if lsusb | grep -i "Orbbec" >/dev/null 2>&1; then
  orbbec_line="$(lsusb | grep -i "Orbbec" | head -n 1)"
  echo "${orbbec_line}"
  if [[ "${orbbec_line}" == *"Bootloader"* ]]; then
    echo "[warning] Orbbec camera is enumerated as a bootloader device, not a streaming sensor."
    status=1
  fi
else
  echo "[missing] no Orbbec USB device found"
  status=1
fi

echo
echo "Checking STM32 serial alias"
if [[ -e /dev/ttySTM32 ]]; then
  ls -l /dev/ttySTM32
else
  echo "[missing] /dev/ttySTM32 does not exist"
  status=1
fi

echo
echo "Checking installed Orbbec udev rules"
if [[ -f /etc/udev/rules.d/99-obsensor-libusb.rules ]]; then
  echo "[ok] /etc/udev/rules.d/99-obsensor-libusb.rules"
else
  echo "[missing] /etc/udev/rules.d/99-obsensor-libusb.rules"
  status=1
fi

echo
echo "Checking installed STM32 serial udev rules"
if [[ -f /etc/udev/rules.d/99-serial-permissions.rules ]]; then
  echo "[ok] /etc/udev/rules.d/99-serial-permissions.rules"
else
  echo "[missing] /etc/udev/rules.d/99-serial-permissions.rules"
  status=1
fi

exit "${status}"
