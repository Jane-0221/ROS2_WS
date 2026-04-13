#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_SCRIPT="$SCRIPT_DIR/run_medipick_demo.sh"

print_usage() {
  cat <<'EOF'
Usage:
  bash choose_base_control_mode.sh
  bash choose_base_control_mode.sh --nav-base [launch_arg:=value ...]
  bash choose_base_control_mode.sh --stm32-base [launch_arg:=value ...]
  bash choose_base_control_mode.sh --base-control-mode nav|stm32 [launch_arg:=value ...]

If no base-control mode is provided, a GUI chooser is shown when available.
EOF
}

open_in_terminal() {
  local mode="$1"
  shift

  local cmd
  cmd="cd $(printf '%q' "$SCRIPT_DIR") && bash $(printf '%q' "$RUN_SCRIPT") --base-control-mode $(printf '%q' "$mode")"
  for arg in "$@"; do
    cmd+=" $(printf '%q' "$arg")"
  done
  cmd+='; status=$?; echo; echo "Process exited with status ${status}."; read -rp "Press Enter to close..."; exit ${status}'

  if command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash -lc "$cmd"
    return
  fi

  echo "No terminal emulator found. Run this script from a terminal instead."
  exit 1
}

select_mode_gui() {
  if ! command -v zenity >/dev/null 2>&1; then
    return 1
  fi

  zenity --list \
    --radiolist \
    --title="Choose Base Control Mode" \
    --text="Select how the base should be controlled for this launch." \
    --column="Pick" \
    --column="Mode" \
    --column="Description" \
    TRUE nav "Nav2 publishes /cmd_vel directly" \
    FALSE stm32 "STM32 relays chassis commands to the base"
}

select_mode_terminal() {
  local selection
  while true; do
    echo "Choose base control mode:"
    echo "  1) nav    - Nav2 publishes /cmd_vel directly"
    echo "  2) stm32  - STM32 relays chassis commands to the base"
    read -rp "Enter 1 or 2: " selection
    case "$selection" in
      1)
        echo "nav"
        return 0
        ;;
      2)
        echo "stm32"
        return 0
        ;;
      *)
        echo "Invalid selection."
        ;;
    esac
  done
}

selected_mode=""
launch_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h)
      print_usage
      exit 0
      ;;
    --nav-base)
      selected_mode="nav"
      shift
      ;;
    --stm32-base)
      selected_mode="stm32"
      shift
      ;;
    --base-control-mode)
      if [[ $# -lt 2 ]]; then
        echo "--base-control-mode requires nav or stm32"
        exit 1
      fi
      selected_mode="$2"
      shift 2
      ;;
    --base-control-mode=*)
      selected_mode="${1#*=}"
      shift
      ;;
    *)
      launch_args+=("$1")
      shift
      ;;
  esac
done

if [[ -n "$selected_mode" && "$selected_mode" != "nav" && "$selected_mode" != "stm32" ]]; then
  echo "Invalid base control mode: $selected_mode"
  echo "Expected: nav or stm32"
  exit 1
fi

if [[ -z "$selected_mode" ]]; then
  if [[ -n "${DISPLAY:-}" ]]; then
    selected_mode="$(select_mode_gui || true)"
  fi

  if [[ -z "$selected_mode" ]]; then
    if [[ -t 0 && -t 1 ]]; then
      selected_mode="$(select_mode_terminal)"
    else
      echo "No control mode selected."
      exit 1
    fi
  fi
fi

if [[ ! -x "$RUN_SCRIPT" ]]; then
  echo "Expected launcher script not found or not executable: $RUN_SCRIPT"
  exit 1
fi

if [[ -t 0 && -t 1 ]]; then
  exec bash "$RUN_SCRIPT" --base-control-mode "$selected_mode" "${launch_args[@]}"
fi

open_in_terminal "$selected_mode" "${launch_args[@]}"
