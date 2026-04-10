#!/usr/bin/env bash

set -euo pipefail

ENV_NAME="${1:-MediPickROS}"

source "${HOME}/miniconda3/etc/profile.d/conda.sh"

conda env remove -n "${ENV_NAME}" -y >/dev/null 2>&1 || true
conda create -y -n "${ENV_NAME}" python=3.10 pip
conda activate "${ENV_NAME}"
conda install -y -c conda-forge colcon-common-extensions catkin_pkg empy pyyaml

cat <<EOF
Environment '${ENV_NAME}' is ready.

Use:
  conda activate ${ENV_NAME}
  source /opt/ros/humble/setup.bash
  source install/setup.bash

Note:
  This environment is aligned to system ROS 2 Humble with Python 3.10.
  Real MoveIt execution still requires ros2_control runtime packages on the host
  (for example: controller_manager, joint_state_broadcaster, joint_trajectory_controller).
EOF
