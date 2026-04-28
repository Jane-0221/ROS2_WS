#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
video_root="${WORKSPACE_DIR}/docs/paper/defense/artifacts/videos"

session_dir="${1:-}"
if [[ -z "${session_dir}" ]]; then
  session_dir="$(find "${video_root}" -mindepth 1 -maxdepth 1 -type d | sort | tail -n 1)"
fi

if [[ -z "${session_dir}" || ! -d "${session_dir}" ]]; then
  echo "Recording session not found." >&2
  exit 1
fi

stop_file="${session_dir}/defense_demo.stop"
metadata_file="${session_dir}/defense_demo.json"

touch "${stop_file}"

recording_backend=""
pid_file=""
if [[ -f "${metadata_file}" ]]; then
  readarray -t backend_info < <(
    python3 - <<'PY' "${metadata_file}"
import json
import sys
from pathlib import Path

payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
print(payload.get("recording_backend", ""))
print(payload.get("pid_file", ""))
PY
  )
  recording_backend="${backend_info[0]:-}"
  pid_file="${backend_info[1]:-}"
fi

if [[ ("${recording_backend}" == "ffmpeg" || "${recording_backend}" == "obs") && -n "${pid_file}" && -f "${pid_file}" ]]; then
  recorder_pid="$(cat "${pid_file}")"
  if [[ -n "${recorder_pid}" ]] && kill -0 "${recorder_pid}" 2>/dev/null; then
    if [[ "${recording_backend}" == "obs" ]]; then
      kill -TERM "${recorder_pid}" >/dev/null 2>&1 || true
    else
      kill -INT "${recorder_pid}" >/dev/null 2>&1 || true
    fi
    for _ in $(seq 1 50); do
      if ! kill -0 "${recorder_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    kill -TERM "${recorder_pid}" >/dev/null 2>&1 || true
  fi
else
  gdbus call --session \
    --dest org.gnome.Shell.Screencast \
    --object-path /org/gnome/Shell/Screencast \
    --method org.gnome.Shell.Screencast.StopScreencast >/dev/null 2>&1 || true
fi

echo "Recording stopped."
echo "Session dir: ${session_dir}"
echo "Metadata: ${metadata_file}"
if [[ -f "${metadata_file}" ]]; then
  python3 - <<'PY' "${metadata_file}"
import json
import sys
from pathlib import Path

metadata_path = Path(sys.argv[1])
payload = json.loads(metadata_path.read_text(encoding="utf-8"))
print(f"Video: {payload.get('video_file', '')}")
PY
fi
