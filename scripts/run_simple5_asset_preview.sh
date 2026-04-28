#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ROS 的 setup.bash 在部分 Conda / strict-mode 环境下会读取未定义变量，
# 因此先不要开启 nounset，等环境加载完成后再切回严格模式。
source /opt/ros/humble/setup.bash
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

bash "${SCRIPT_DIR}/sync_simple5_vendor_from_zip.sh"

ros2 launch medipick_simple3_description simple5_asset_preview.launch.py use_gui:=true "$@"
