#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${1:-${WORKSPACE_DIR}/bags/gazebo_il_${timestamp}}"

mkdir -p "${output_dir}"

topics=(
  /clock
  /tf
  /tf_static
  /joint_states
  /medipick/depth_camera/image
  /medipick/depth_camera/depth_image
  /medipick/depth_camera/camera_info
  /medipick/depth_camera/points
  /monitored_planning_scene
  /planning_scene
  /planning_scene_world
  /move_group/display_planned_path
  /medipick/task/target_pose
  /medipick/task/stage
  /medipick/task/event
  /medipick/task/base_goal
  /medipick/task/lift_target_height
  /medipick/task/pre_insert_pose
  /medipick/task/pick_pose
  /medipick/task/retreat_pose
  /medipick/task/achieved_pose
  /medipick/task/planned_trajectory
  /mobile_arm_controller/controller_state
  /tool_controller/controller_state
  /head_controller/controller_state
)

echo "Recording Gazebo imitation-learning bag to:"
echo "  ${output_dir}"
echo
echo "Topics:"
printf '  %s\n' "${topics[@]}"
echo

run_in_workspace ros2 bag record -o "${output_dir}" "${topics[@]}"
