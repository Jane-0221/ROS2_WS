#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script with sudo."
  exit 1
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
ros2_list="/etc/apt/sources.list.d/ros2.list"
ros_fish_list="/etc/apt/sources.list.d/ros-fish.list"

backup_file() {
  local file="$1"
  if [[ -f "${file}" ]]; then
    cp "${file}" "${file}.bak.${timestamp}"
  fi
}

echo "[1/4] Backing up existing ROS 2 apt source files"
backup_file "${ros2_list}"
backup_file "${ros_fish_list}"

echo "[2/4] Writing Aliyun ROS 2 source"
cat > "${ros2_list}" <<'EOF'
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu jammy main
EOF

if [[ -f "${ros_fish_list}" ]]; then
  mv "${ros_fish_list}" "${ros_fish_list}.disabled.${timestamp}"
fi

echo "[3/4] Removing stale ROS 2 apt index files"
rm -f /var/lib/apt/lists/*ros2*

echo "[4/4] Refreshing apt metadata"
apt-get update -o Acquire::Retries=3

echo
echo "ROS 2 apt source switched to Aliyun."
echo "Backups are stored with suffix .bak.${timestamp}"
