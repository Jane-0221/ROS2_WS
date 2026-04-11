#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source_if_exists() {
  local target="$1"
  if [[ -f "${target}" ]]; then
    local had_nounset=0
    if [[ $- == *u* ]]; then
      had_nounset=1
      set +u
    fi
    # shellcheck disable=SC1090
    source "${target}"
    if [[ ${had_nounset} -eq 1 ]]; then
      set -u
    fi
  fi
}

remove_path_prefix_matching() {
  local pattern="$1"
  local filtered=""
  local old_ifs="${IFS}"
  IFS=':'
  for entry in ${PATH}; do
    if [[ -n "${entry}" && "${entry}" == ${pattern}* ]]; then
      continue
    fi
    if [[ -z "${filtered}" ]]; then
      filtered="${entry}"
    else
      filtered="${filtered}:${entry}"
    fi
  done
  IFS="${old_ifs}"
  PATH="${filtered}"
}

conda_env_exists() {
  local env_name="$1"
  conda env list | awk '{print $1}' | grep -qx "${env_name}"
}

source_base_env() {
  source_if_exists "${HOME}/miniconda3/etc/profile.d/conda.sh"

  local use_system_ros="${MEDIPICK_USE_SYSTEM_ROS:-1}"

  if [[ "${use_system_ros}" != "1" ]] && command -v conda >/dev/null 2>&1; then
    export CONDA_BUILD="${CONDA_BUILD:-0}"
    export target_platform="${target_platform:-}"
    export build_platform="${build_platform:-}"
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-}"
    local preferred_env="${MEDIPICK_CONDA_ENV:-MediPickRuntime}"
    local had_nounset=0
    if [[ $- == *u* ]]; then
      had_nounset=1
      set +u
    fi
    if conda_env_exists "${preferred_env}"; then
      conda activate "${preferred_env}"
    elif conda_env_exists "MediPickROS"; then
      conda activate MediPickROS
    elif conda_env_exists "MediPick"; then
      conda activate MediPick
    elif conda_env_exists "medipick"; then
      conda activate medipick
    fi
    if [[ ${had_nounset} -eq 1 ]]; then
      set -u
    fi
  fi

  if [[ "${use_system_ros}" != "1" && -n "${CONDA_PREFIX:-}" && -f "${CONDA_PREFIX}/setup.bash" ]]; then
    source_if_exists "${CONDA_PREFIX}/setup.bash"
  else
    source_if_exists "/opt/ros/humble/setup.bash"
    remove_path_prefix_matching "${HOME}/miniconda3/bin"
    if [[ -n "${CONDA_PREFIX:-}" ]]; then
      remove_path_prefix_matching "${CONDA_PREFIX}/bin"
    fi
    export PATH="/usr/bin:/bin:${PATH}"
    export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
    unset CONDA_PREFIX
  fi
}

source_workspace_env() {
  source_base_env
  source_if_exists "${WORKSPACE_DIR}/install/setup.bash"

  if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/bin" ]]; then
    export PATH="${CONDA_PREFIX}/bin:${PATH}"
  fi
}

run_in_workspace() {
  cd "${WORKSPACE_DIR}"
  source_workspace_env
  "$@"
}

cleanup_medipick_stack() {
  local patterns=(
    "demo.launch.py"
    "gazebo_demo.launch.py"
    "randomized_pick_demo.launch.py"
    "randomized_pick_rviz.launch.py"
    "pose_goal_obstacle_demo.launch.py"
    "static_shelf_scene.launch.py"
    "planning_demo.launch.py"
    "moveit_mock_demo.launch.py"
    "pick_task_manager.launch.py"
    "mock_perception.launch.py"
    "planning_server.py --ros-args -r __node:=medipick_planning_server"
    "mock_vision_publisher.py --ros-args -r __node:=medipick_mock_vision_publisher"
    "mock_pick_path_publisher.py --ros-args -r __node:=medipick_mock_pick_path_publisher"
    "randomized_pick_demo.py --ros-args -r __node:=medipick_randomized_pick_demo"
    "pose_goal_obstacle_demo.py --ros-args -r __node:=medipick_pose_goal_obstacle_demo"
    "static_shelf_scene.py --ros-args -r __node:=medipick_static_shelf_scene"
    "pick_task_manager.py --ros-args -r __node:=medipick_pick_task_manager"
    "default_joint_state_publisher.py"
    "ros2_control_node"
    "controller_manager/spawner"
    "controller_manager spawner"
    "ros_gz_bridge"
    "ros_gz_sim"
    "gz sim"
    "ign gazebo"
    "gzserver"
    "gzclient"
    "robot_state_publisher"
    "move_group"
    "medipick_randomized_pick_rviz"
    "medipick_pose_goal_rviz"
    "medipick_static_shelf_rviz"
    "rviz2 -d .*moveit.rviz"
    "rviz2 -d .*randomized_pick_demo.rviz"
    "rviz2 -d .*pose_goal_obstacle_demo.rviz"
  )

  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -f "${pattern}" >/dev/null 2>&1 || true
  done

  sleep 1
}

cleanup_randomized_pick_rviz() {
  local patterns=(
    "randomized_pick_rviz.launch.py"
    "medipick_randomized_pick_rviz"
    "rviz2 -d .*randomized_pick_demo.rviz"
  )

  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -f "${pattern}" >/dev/null 2>&1 || true
  done

  sleep 1
}
