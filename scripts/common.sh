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

is_wsl_environment() {
  if [[ -f /proc/sys/kernel/osrelease ]] && grep -qiE 'microsoft|wsl' /proc/sys/kernel/osrelease; then
    return 0
  fi
  if [[ -f /proc/version ]] && grep -qiE 'microsoft|wsl' /proc/version; then
    return 0
  fi
  return 1
}

configure_local_dds() {
  if [[ "${MEDIPICK_DISABLE_FASTDDS_SHM:-1}" != "1" ]]; then
    return
  fi

  # This host repeatedly hits Fast DDS shared-memory port lock failures,
  # which can destabilize node discovery during large launch files.
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
}

configure_ros_domain() {
  if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID
    return
  fi

  export ROS_DOMAIN_ID="${MEDIPICK_ROS_DOMAIN_ID:-66}"
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
  configure_local_dds
  configure_ros_domain
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
    "/opt/ros/.*/bin/ros2 launch medipick_planning_server gazebo_nav2_pick_trial.launch.py"
    "/opt/ros/.*/bin/ros2 launch medipick_planning_server gazebo_rtabmap_slam.launch.py"
    "/opt/ros/.*/bin/ros2 launch medipick_planning_server gazebo_rtabmap_manual_mapping.launch.py"
    "/opt/ros/.*/bin/ros2 launch medipick_planning_server gazebo_nav2_auto_explore.launch.py"
    "/opt/ros/.*/bin/ros2 launch medipick_planning_server gazebo_nav2_slam_validation.launch.py"
    "/opt/ros/.*/bin/ros2 launch medipick_planning_server gazebo_nav2_saved_map_validation.launch.py"
    "/opt/ros/.*/bin/ros2 launch medipick_planning_server gazebo_nav2_saved_map_pick_trial.launch.py"
    "post_pick_delivery_manager.py"
    "medicine_memory_registry.py --ros-args -r __node:=medipick_medicine_memory_registry"
    "medicine_memory_target_publisher.py --ros-args -r __node:=medipick_medicine_memory_target_publisher"
    "medicine_memory_gazebo_recorder.py --ros-args -r __node:=medipick_medicine_memory_gazebo_recorder"
    "randomized_pick_demo.launch.py"
    "randomized_pick_rviz.launch.py"
    "pose_goal_obstacle_demo.launch.py"
    "static_shelf_scene.launch.py"
    "planning_demo.launch.py"
    "gazebo_pick_trial.launch.py"
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
    "gazebo_box_target_publisher.py --ros-args -r __node:=medipick_gazebo_box_target_publisher"
    "camera_info_stamp_republisher.py --ros-args -r __node:=medipick_camera_info_stamp_republisher"
    "gazebo_trial_initial_pose_manager.py --ros-args -r __node:=medipick_gazebo_trial_initial_pose_manager"
    "gazebo_cmd_vel_bridge.py --ros-args -r __node:=medipick_gazebo_cmd_vel_bridge"
    "gazebo_omni_base_driver.py --ros-args -r __node:=medipick_gazebo_omni_base_driver"
    "gazebo_discrete_lift_controller.py --ros-args -r __node:=medipick_gazebo_discrete_lift_controller"
    "gazebo_ground_truth_localization.py --ros-args -r __node:=medipick_gazebo_ground_truth_localization"
    "gazebo_map_publisher.py --ros-args -r __node:=medipick_gazebo_map_publisher"
    "latched_occupancy_grid_republisher.py --ros-args -r __node:=medipick_latched_slam_map_republisher"
    "gazebo_mapping_route_runner.py --ros-args -r __node:=medipick_gazebo_mapping_route_runner"
    "gazebo_mapping_route_runner.py"
    "gazebo_pre_navigation_manager.py --ros-args -r __node:=medipick_gazebo_pre_navigation_manager"
    "nav2_target_navigator.py --ros-args -r __node:=medipick_nav2_target_navigator"
    "frontier_explorer.py --ros-args -r __node:=medipick_frontier_explorer"
    "nav2_activation_gate.py --ros-args -r __node:=medipick_nav2_activation_gate"
    "slam_loop_validator.py --ros-args -r __node:=medipick_slam_loop_validator"
    "slam_loop_validator.py --ros-args -r __node:=medipick_saved_map_nav_validator"
    "rtabmap_param_tuner.py --ros-args -r __node:=medipick_rtabmap_param_tuner"
    "bool_topic_gate.py --ros-args -r __node:=medipick_.*_gate"
    "mapping_health_monitor.py --ros-args -r __node:=medipick_mapping_health_monitor"
    "mapping_keyboard_teleop.py --ros-args -r __node:=medipick_mapping_keyboard_teleop"
    "nav2_map_server.*map_server"
    "nav2_controller.*controller_server"
    "nav2_smoother.*smoother_server"
    "nav2_planner.*planner_server"
    "nav2_behaviors.*behavior_server"
    "nav2_bt_navigator.*bt_navigator"
    "nav2_waypoint_follower.*waypoint_follower"
    "nav2_velocity_smoother.*velocity_smoother"
    "nav2_lifecycle_manager.*lifecycle_manager"
    "ros2 topic pub .*medipick/controllers_ready"
    "lib/rtabmap_sync/rgbd_sync"
    "lib/rtabmap_odom/rgbd_odometry"
    "lib/rtabmap_odom/stereo_odometry"
    "lib/rtabmap_odom/icp_odometry"
    "lib/rtabmap_slam/rtabmap"
    "lib/rtabmap_viz/rtabmap_viz"
    "lib/slam_toolbox/async_slam_toolbox_node"
    "default_joint_state_publisher.py"
    "joint_state_stamp_republisher.py --ros-args -r __node:=medipick_joint_state_stamp_republisher"
    "tf_stamp_republisher.py --ros-args -r __node:=medipick_tf_stamp_republisher"
    "ros2_control_node"
    "controller_manager/spawner"
    "controller_manager spawner"
    "ros_gz_bridge"
    "ros_gz_sim"
    "ruby /usr/bin/ign gazebo"
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

  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" >/dev/null 2>&1 || true
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
