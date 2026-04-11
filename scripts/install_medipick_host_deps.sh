#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script with sudo."
  exit 1
fi

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
ORBBEC_UDEV_INSTALLER="${WORKSPACE_ROOT}/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh"
ROS_APT_SOURCE_FIXER="${WORKSPACE_ROOT}/scripts/configure_ros_apt_source_aliyun.sh"
STM32_UDEV_RULE="${WORKSPACE_ROOT}/etc/udev/rules.d/99-serial-permissions.rules"

export DEBIAN_FRONTEND=noninteractive

if ! grep -RqsE '^[^#].*jammy-updates' /etc/apt/sources.list /etc/apt/sources.list.d/*.list 2>/dev/null; then
  echo "Detected missing Ubuntu jammy-updates repository."
  echo "Appending jammy-updates entries to /etc/apt/sources.list."
  cat >> /etc/apt/sources.list <<'EOF'

# Added by install_medipick_host_deps.sh for ROS/SLAM runtime dependencies
deb http://cn.archive.ubuntu.com/ubuntu/ jammy-updates main restricted
deb http://cn.archive.ubuntu.com/ubuntu/ jammy-updates universe
deb http://cn.archive.ubuntu.com/ubuntu/ jammy-updates multiverse
EOF
fi

if grep -RqsE 'mirrors\.(tuna|ustc)\..*/ros2/ubuntu' /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros-fish.list 2>/dev/null; then
  echo "Detected stale or unreliable ROS 2 mirror configuration."
  echo "Switching ROS 2 apt source to Aliyun before installation."
  "${ROS_APT_SOURCE_FIXER}"
fi

echo "[1/4] Updating apt metadata"
apt-get update -o Acquire::Retries=3

echo "[2/4] Installing minimal MoveIt runtime package set"
apt-get install -y \
  --no-install-recommends \
  ros-humble-control-msgs \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-perception \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-robot-localization \
  ros-humble-rtabmap-ros \
  ros-humble-apriltag-ros

echo "[3/4] Installing Orbbec udev rules"
"${ORBBEC_UDEV_INSTALLER}"

echo "[4/4] Installing STM32 serial udev rules"
install -m 0644 "${STM32_UDEV_RULE}" /etc/udev/rules.d/99-serial-permissions.rules
udevadm control --reload-rules
udevadm trigger

echo
echo "Host dependencies installed."
echo "If the Orbbec camera or STM32 serial device was already plugged in, unplug and replug it now."
