#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

session_name="demo_segment"
backend="auto"
record_fps=10
record_timeout_sec=3600
command_timeout_sec=0
visible_terminal=true
screen_mode="all"
screen_x=0
screen_y=0
screen_width=0
screen_height=0
post_stop_delay_sec=2

args=("$@")
cmd_start=-1
for i in "${!args[@]}"; do
  if [[ "${args[$i]}" == "--" ]]; then
    cmd_start="$i"
    break
  fi
done

if [[ "${cmd_start}" -lt 0 ]]; then
  echo "Usage: bash scripts/record_defense_demo_segment.sh session_name:=... [options] -- <command...>" >&2
  exit 2
fi

for ((i = 0; i < cmd_start; ++i)); do
  arg="${args[$i]}"
  case "${arg}" in
    session_name:=*)
      session_name="${arg#session_name:=}"
      ;;
    backend:=*)
      backend="${arg#backend:=}"
      ;;
    record_fps:=*)
      record_fps="${arg#record_fps:=}"
      ;;
    record_timeout_sec:=*)
      record_timeout_sec="${arg#record_timeout_sec:=}"
      ;;
    command_timeout_sec:=*)
      command_timeout_sec="${arg#command_timeout_sec:=}"
      ;;
    visible_terminal:=*)
      visible_terminal="${arg#visible_terminal:=}"
      ;;
    screen_mode:=*)
      screen_mode="${arg#screen_mode:=}"
      ;;
    screen_x:=*)
      screen_x="${arg#screen_x:=}"
      ;;
    screen_y:=*)
      screen_y="${arg#screen_y:=}"
      ;;
    screen_width:=*)
      screen_width="${arg#screen_width:=}"
      ;;
    screen_height:=*)
      screen_height="${arg#screen_height:=}"
      ;;
    post_stop_delay_sec:=*)
      post_stop_delay_sec="${arg#post_stop_delay_sec:=}"
      ;;
    *)
      echo "Unsupported argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

command_args=("${args[@]:$((cmd_start + 1))}")
if [[ "${#command_args[@]}" -eq 0 ]]; then
  echo "No demo command provided." >&2
  exit 2
fi

start_output="$(
  bash "${SCRIPT_DIR}/start_defense_demo_recording.sh" \
    "session_name:=${session_name}" \
    "backend:=${backend}" \
    "fps:=${record_fps}" \
    "capture_timeout_sec:=${record_timeout_sec}" \
    "screen_mode:=${screen_mode}" \
    "screen_x:=${screen_x}" \
    "screen_y:=${screen_y}" \
    "screen_width:=${screen_width}" \
    "screen_height:=${screen_height}"
)"

echo "${start_output}"

session_dir="$(printf '%s\n' "${start_output}" | sed -n 's/^Session dir: //p' | tail -n 1)"
if [[ -z "${session_dir}" || ! -d "${session_dir}" ]]; then
  echo "Failed to parse recording session directory." >&2
  exit 1
fi

command_file="${session_dir}/demo_command.sh"
run_log="${session_dir}/demo_run.log"

{
  printf '#!/usr/bin/env bash\n'
  printf 'set -euo pipefail\n'
  printf 'exec > >(tee -a %q) 2>&1\n' "${run_log}"
  printf 'cd %q\n' "${WORKSPACE_DIR}"
  printf 'echo %q\n' "\$ $(printf '%q ' "${command_args[@]}")"
  if [[ "${command_timeout_sec}" -gt 0 ]]; then
    printf 'timeout --signal=INT %q ' "${command_timeout_sec}s"
  fi
  printf '%q ' "${command_args[@]}"
  printf '\n'
} > "${command_file}"
chmod +x "${command_file}"

run_status=0
if [[ "${visible_terminal}" == "true" ]]; then
  gnome-terminal --wait -- bash "${command_file}" || run_status=$?
else
  bash "${command_file}" || run_status=$?
fi

sleep "${post_stop_delay_sec}"
bash "${SCRIPT_DIR}/stop_defense_demo_recording.sh" "${session_dir}" || true

python3 - <<'PY' "${session_dir}" "${run_status}"
import json
import sys
from pathlib import Path

session_dir = Path(sys.argv[1])
run_status = int(sys.argv[2])
metadata_path = session_dir / "defense_demo.json"
payload = {}
if metadata_path.exists():
    payload = json.loads(metadata_path.read_text(encoding="utf-8"))
payload["demo_run_status"] = run_status
payload["demo_run_log"] = str(session_dir / "demo_run.log")
payload["demo_command_file"] = str(session_dir / "demo_command.sh")
metadata_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
PY

echo "Demo recording session complete."
echo "Session dir: ${session_dir}"
echo "Run log: ${run_log}"
echo "Command file: ${command_file}"
exit "${run_status}"
