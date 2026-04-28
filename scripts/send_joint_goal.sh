#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/send_joint_goal.sh <controller> joint1=value1 [joint2=value2 ...] [duration:=2.0]

Examples:
  bash scripts/send_joint_goal.sh head_controller h1_joint=0.0 h2_joint=-0.5236 duration:=1.0
  bash scripts/send_joint_goal.sh mobile_arm_controller base_x=0.10 base_theta=0.20 duration:=2.0
  bash scripts/send_joint_goal.sh mobile_arm_controller raise_joint=0.20 r2_joint=0.40 r4_joint=-0.30 duration:=2.5
  bash scripts/send_joint_goal.sh tool_controller sucker_joint=0.03 duration:=1.0

Notes:
  - Controller names are usually: mobile_arm_controller, head_controller, tool_controller
  - The script sends a FollowJointTrajectory goal directly to ros2_control
  - Partial joint goals are supported by the current Gazebo trial controllers
EOF
}

if (( $# < 2 )); then
  usage
  exit 1
fi

controller_name="$1"
shift

duration="2.0"
declare -a joint_names=()
declare -a joint_positions=()

for arg in "$@"; do
  case "$arg" in
    duration:=*)
      duration="${arg#duration:=}"
      ;;
    duration=*)
      duration="${arg#duration=}"
      ;;
    --duration=*)
      duration="${arg#--duration=}"
      ;;
    *=*)
      joint_names+=("${arg%%=*}")
      joint_positions+=("${arg#*=}")
      ;;
    *)
      echo "Unsupported argument: ${arg}" >&2
      usage
      exit 1
      ;;
  esac
done

if (( ${#joint_names[@]} == 0 )); then
  echo "At least one joint=value pair is required." >&2
  usage
  exit 1
fi

if [[ ! "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Invalid duration: ${duration}" >&2
  exit 1
fi

read -r duration_sec duration_nanosec < <(
  awk -v duration="${duration}" 'BEGIN {
    if (duration < 0.1) {
      duration = 0.1
    }
    sec = int(duration)
    nanosec = int((duration - sec) * 1000000000 + 0.5)
    if (nanosec >= 1000000000) {
      sec += 1
      nanosec -= 1000000000
    }
    printf "%d %d\n", sec, nanosec
  }'
)

action_name="${controller_name}"
if [[ "${action_name}" != /* ]]; then
  action_name="/${action_name}"
fi
if [[ "${action_name}" != */follow_joint_trajectory ]]; then
  action_name="${action_name}/follow_joint_trajectory"
fi

joint_list=""
position_list=""
for index in "${!joint_names[@]}"; do
  if [[ -n "${joint_list}" ]]; then
    joint_list+=", "
    position_list+=", "
  fi
  joint_list+="'${joint_names[$index]}'"
  position_list+="${joint_positions[$index]}"
done

goal_yaml="{trajectory: {joint_names: [${joint_list}], points: [{positions: [${position_list}], time_from_start: {sec: ${duration_sec}, nanosec: ${duration_nanosec}}}]}}"

echo "Sending direct joint goal:"
echo "  action: ${action_name}"
echo "  joints: ${joint_names[*]}"
echo "  positions: ${joint_positions[*]}"
echo "  duration: ${duration_sec}s ${duration_nanosec}ns"

run_in_workspace ros2 action send_goal \
  "${action_name}" \
  control_msgs/action/FollowJointTrajectory \
  "${goal_yaml}"
