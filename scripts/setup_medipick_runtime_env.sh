#!/usr/bin/env bash

set -euo pipefail

ENV_NAME="${1:-MediPickRuntime}"

source "${HOME}/miniconda3/etc/profile.d/conda.sh"

conda env remove -n "${ENV_NAME}" -y >/dev/null 2>&1 || true
conda create -y -n "${ENV_NAME}" \
  --override-channels \
  -c conda-forge \
  -c robostack-humble \
  python=3.11 \
  colcon-common-extensions \
  cmake \
  pkg-config \
  make \
  compilers \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

cat <<EOF
Environment '${ENV_NAME}' is ready.

Use:
  conda activate ${ENV_NAME}
  source "\${CONDA_PREFIX}/setup.bash"
  cd /home/lzoolloozl/projects/medipick_moveit_workspace
  bash scripts/build.sh
  bash scripts/run_moveit_demo.sh

This environment is self-contained and does not rely on /opt/ros/humble.
EOF
