#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
DEFAULT_DB_PATH="$HOME/.ros/medipick_scene_mapping.db"
DEFAULT_APP_DEVICE_NAME="MediPick-Desk"

source_setup() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "$1"
  if [[ "$had_nounset" -eq 1 ]]; then
    set -u
  fi
}

print_usage() {
  cat <<'EOF'
Usage:
  ./switch_chassis_control_mode.sh
  ./switch_chassis_control_mode.sh --mode stm32
  ./switch_chassis_control_mode.sh --mode nav
  ./switch_chassis_control_mode.sh --mode app
  ./switch_chassis_control_mode.sh --mode nav --database /path/to/map.db
  ./switch_chassis_control_mode.sh --mode stm32 --dry-run

Modes:
  stm32  Receive STM32 chassis commands and relay them to the Wheeltec base.
  nav    Start localization + Nav2 and let ROS /cmd_vel control the Wheeltec base.
  app    Start the Android mapping bridge; the phone app controls the base through the mux.

Extra ROS launch arguments can be appended at the end:
  ./switch_chassis_control_mode.sh --mode nav start_rtabmap_viz:=true
  ./switch_chassis_control_mode.sh --mode app device_name:=My-MediPick-Desk
EOF
}

select_mode_gui() {
  if [[ -z "${DISPLAY:-}" ]] || ! command -v zenity >/dev/null 2>&1; then
    return 1
  fi

  zenity --list \
    --radiolist \
    --title="MediPick 底盘控制模式" \
    --text="请选择底盘控制模式。" \
    --column="选择" \
    --column="模式" \
    --column="说明" \
    TRUE stm32 "接收 STM32 上报码控制底盘" \
    FALSE nav "导航/ROS 通过 /cmd_vel 控制底盘" \
    FALSE app "Android App 通过桥接服务控制底盘"
}

select_mode_terminal() {
  local selection
  while true; do
    echo "请选择底盘控制模式："
    echo "  1) stm32  接收 STM32 上报码控制底盘"
    echo "  2) nav    导航/ROS 通过 /cmd_vel 控制底盘"
    echo "  3) app    Android App 通过桥接服务控制底盘"
    read -rp "请输入 1、2 或 3: " selection
    case "$selection" in
      1)
        echo "stm32"
        return 0
        ;;
      2)
        echo "nav"
        return 0
        ;;
      3)
        echo "app"
        return 0
        ;;
      *)
        echo "输入无效。"
        ;;
    esac
  done
}

stop_conflicting_processes() {
  echo "停止可能冲突的后台服务和进程..."
  systemctl stop --no-block medipick-stm32.service medipick-wheeltec.service >/dev/null 2>&1 || true

  local patterns=(
    "scene_mapping_navigation.launch.py"
    "visual_navigation_demo.launch.py"
    "mapping_app_bridge.launch.py"
    "medipick_mapping_app_bridge"
    "wheeltec_chassis_node"
    "stm32_serial_node"
    "rtabmap_viz"
    "/rtabmap_slam/rtabmap"
    "/rtabmap_odom/rgbd_odometry"
    "robot_localization/ekf_node"
    "planner_server"
    "controller_server"
    "smoother_server"
    "behavior_server"
    "bt_navigator"
    "waypoint_follower"
    "lifecycle_manager_navigation"
    "component_container"
  )

  local pattern
  for pattern in "${patterns[@]}"; do
    pkill -f "$pattern" >/dev/null 2>&1 || true
  done

  sleep 1
}

ensure_environment() {
  cd "$WORKSPACE_DIR"
  source_setup /opt/ros/humble/setup.bash

  if [[ ! -f install/setup.bash ]]; then
    echo "未找到 install/setup.bash，请先编译工作区。"
    echo "命令：colcon build --symlink-install"
    exit 1
  fi

  source_setup install/setup.bash
}

build_launch_command() {
  local mode="$1"
  local database_path="$2"
  shift 2

  local cmd=(
    ros2 launch medipick_planning_server scene_mapping_navigation.launch.py
  )

  if [[ "$mode" == "stm32" ]]; then
    cmd+=(
      start_camera:=false
      start_slam:=false
      start_nav2:=false
      start_rtabmap_viz:=false
      start_stm32:=true
      start_base:=true
      base_control_mode:=stm32
    )
  elif [[ "$mode" == "nav" ]]; then
    cmd+=(
      localization:=true
      start_nav2:=true
      start_rtabmap_viz:=false
      delete_db_on_start:=false
      start_stm32:=false
      start_base:=true
      base_control_mode:=nav
      rtabmap_database_path:="$database_path"
    )
  else
    cmd=(
      bash
      "$WORKSPACE_DIR/run_mapping_console_bridge.sh"
      "device_name:=$DEFAULT_APP_DEVICE_NAME"
    )
  fi

  if [[ $# -gt 0 ]]; then
    cmd+=("$@")
  fi

  printf '%s\n' "${cmd[@]}"
}

run_mode() {
  local mode="$1"
  local database_path="$2"
  shift 2

  ensure_environment

  if [[ "$mode" == "nav" && ! -f "$database_path" ]]; then
    echo "导航模式需要已有 RTAB-Map 数据库，但未找到："
    echo "  $database_path"
    exit 1
  fi

  stop_conflicting_processes

  local -a launch_cmd
  mapfile -t launch_cmd < <(build_launch_command "$mode" "$database_path" "$@")

  echo
  if [[ "$mode" == "stm32" ]]; then
    echo "即将启动：STM32 接收模式"
    echo "底盘将订阅 /medipick/hardware/stm32_base_cmd_vel"
  elif [[ "$mode" == "nav" ]]; then
    echo "即将启动：导航控制模式"
    echo "底盘将订阅 /cmd_vel，数据库：$database_path"
  else
    echo "即将启动：App 控制模式"
    echo "将启动 Android 桥接服务，手机 App 通过 /medipick/app/cmd_vel 控制底盘"
  fi
  echo
  printf '命令：'
  printf ' %q' "${launch_cmd[@]}"
  echo
  echo

  exec "${launch_cmd[@]}"
}

selected_mode=""
database_path="$DEFAULT_DB_PATH"
dry_run=0
launch_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      print_usage
      exit 0
      ;;
    --mode)
      if [[ $# -lt 2 ]]; then
        echo "--mode 需要参数：stm32、nav 或 app"
        exit 1
      fi
      selected_mode="$2"
      shift 2
      ;;
    --mode=*)
      selected_mode="${1#*=}"
      shift
      ;;
    --database)
      if [[ $# -lt 2 ]]; then
        echo "--database 需要数据库路径"
        exit 1
      fi
      database_path="$2"
      shift 2
      ;;
    --database=*)
      database_path="${1#*=}"
      shift
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    *)
      launch_args+=("$1")
      shift
      ;;
  esac
done

if [[ -z "$selected_mode" ]]; then
  selected_mode="$(select_mode_gui || true)"
fi

if [[ -z "$selected_mode" ]]; then
  selected_mode="$(select_mode_terminal)"
fi

case "$selected_mode" in
  stm32|nav|app)
    ;;
  *)
    echo "不支持的模式：$selected_mode"
    echo "可选值：stm32、nav 或 app"
    exit 1
    ;;
esac

if [[ "$dry_run" -eq 1 ]]; then
  mapfile -t preview_cmd < <(build_launch_command "$selected_mode" "$database_path" "${launch_args[@]}")
  printf '模式：%s\n' "$selected_mode"
  printf '命令：'
  printf ' %q' "${preview_cmd[@]}"
  echo
  exit 0
fi

run_mode "$selected_mode" "$database_path" "${launch_args[@]}"
