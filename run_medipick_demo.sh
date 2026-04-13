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

print_usage() {
  cat <<'EOF'
Usage:
  bash run_medipick_demo.sh [--base-control-mode nav|stm32] [launch_arg:=value ...]
  bash run_medipick_demo.sh --nav-base [launch_arg:=value ...]
  bash run_medipick_demo.sh --stm32-base [launch_arg:=value ...]

Base control modes:
  nav    Nav2 publishes /cmd_vel directly to the Wheeltec base node.
  stm32  STM32 publishes /medipick/hardware/stm32_base_cmd_vel to the Wheeltec base node.

Examples:
  bash run_medipick_demo.sh --nav-base
  bash run_medipick_demo.sh --stm32-base rviz:=false
  bash run_medipick_demo.sh --base-control-mode stm32 start_nav2:=false
EOF
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

required_pkgs=(
  nav2_bringup
  nav2_mppi_controller
  robot_localization
  rtabmap_ros
  apriltag_ros
)

for pkg in "${required_pkgs[@]}"; do
  if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "${pkg} is missing."
    echo "Run: sudo ./scripts/install_medipick_host_deps.sh"
    exit 1
  fi
done

if [[ ! -f /etc/udev/rules.d/99-obsensor-libusb.rules ]]; then
  echo "Orbbec udev rules are missing."
  echo "Run: sudo ./scripts/install_medipick_host_deps.sh"
  exit 1
fi

launch_args=()
has_rviz_arg=0
has_base_control_arg=0
selected_base_control_mode=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      print_usage
      exit 0
      ;;
    --nav-base)
      selected_base_control_mode="nav"
      shift
      ;;
    --stm32-base)
      selected_base_control_mode="stm32"
      shift
      ;;
    --base-control-mode)
      if [[ $# -lt 2 ]]; then
        echo "--base-control-mode requires a value: nav or stm32"
        exit 1
      fi
      selected_base_control_mode="$2"
      shift 2
      ;;
    --base-control-mode=*)
      selected_base_control_mode="${1#*=}"
      shift
      ;;
    *)
      if [[ "$1" == rviz:=* ]]; then
        has_rviz_arg=1
      fi
      if [[ "$1" == base_control_mode:=* ]]; then
        has_base_control_arg=1
      fi
      launch_args+=("$1")
      shift
      ;;
  esac
done

if [[ -n "$selected_base_control_mode" ]]; then
  case "$selected_base_control_mode" in
    nav|stm32)
      ;;
    *)
      echo "Invalid base control mode: $selected_base_control_mode"
      echo "Expected: nav or stm32"
      exit 1
      ;;
  esac
fi

if [[ -n "$selected_base_control_mode" && "$has_base_control_arg" -eq 1 ]]; then
  echo "Do not pass both --base-control-mode/--nav-base/--stm32-base and base_control_mode:=..."
  exit 1
fi

if [[ "$has_rviz_arg" -eq 0 ]]; then
  launch_args=("rviz:=true" "${launch_args[@]}")
fi

if [[ "$has_base_control_arg" -eq 0 && -n "$selected_base_control_mode" ]]; then
  launch_args=("base_control_mode:=$selected_base_control_mode" "${launch_args[@]}")
fi

if [[ -n "$selected_base_control_mode" ]]; then
  echo "Using base control mode: $selected_base_control_mode"
fi

ros2 launch medipick_planning_server visual_navigation_demo.launch.py "${launch_args[@]}"
