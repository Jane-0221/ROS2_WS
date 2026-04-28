#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

session_name="saved_map_pick_and_place_demo"
backend="auto"
fps=4
max_width=1920
max_height=1080
capture_timeout_sec=3600
screen_mode="primary"
screen_x=0
screen_y=0
screen_width=0
screen_height=0

for arg in "$@"; do
  case "${arg}" in
    session_name:=*)
      session_name="${arg#session_name:=}"
      ;;
    backend:=*)
      backend="${arg#backend:=}"
      ;;
    fps:=*)
      fps="${arg#fps:=}"
      ;;
    max_width:=*)
      max_width="${arg#max_width:=}"
      ;;
    max_height:=*)
      max_height="${arg#max_height:=}"
      ;;
    capture_timeout_sec:=*)
      capture_timeout_sec="${arg#capture_timeout_sec:=}"
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
    *)
      echo "Unsupported argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

timestamp="$(date +%Y%m%d_%H%M%S)"
artifact_root="${WORKSPACE_DIR}/docs/paper/defense/artifacts/videos/${timestamp}_${session_name}"
mkdir -p "${artifact_root}"

if [[ "${screen_mode}" == "primary" ]]; then
  mapfile -t geometry < <(
    xrandr --current | python3 -c '
import re
import sys

for line in sys.stdin:
    if " connected primary " not in line:
        continue
    match = re.search(r"([0-9]+)x([0-9]+)\+([0-9]+)\+([0-9]+)", line)
    if match:
        print(match.group(3))
        print(match.group(4))
        print(match.group(1))
        print(match.group(2))
        sys.exit(0)
sys.exit(1)
'
  )
  if [[ "${#geometry[@]}" -ne 4 ]]; then
    echo "Failed to detect primary monitor geometry from xrandr." >&2
    exit 1
  fi
  screen_x="${geometry[0]}"
  screen_y="${geometry[1]}"
  screen_width="${geometry[2]}"
  screen_height="${geometry[3]}"
elif [[ "${screen_mode}" == "all" ]]; then
  mapfile -t geometry < <(
    xrandr --current | python3 -c '
import re
import sys

min_x = None
min_y = None
max_x = None
max_y = None

for line in sys.stdin:
    if " connected " not in line:
        continue
    match = re.search(r"([0-9]+)x([0-9]+)\+([0-9]+)\+([0-9]+)", line)
    if not match:
        continue
    width = int(match.group(1))
    height = int(match.group(2))
    x = int(match.group(3))
    y = int(match.group(4))
    min_x = x if min_x is None else min(min_x, x)
    min_y = y if min_y is None else min(min_y, y)
    max_x = x + width if max_x is None else max(max_x, x + width)
    max_y = y + height if max_y is None else max(max_y, y + height)

if min_x is None:
    sys.exit(1)

print(min_x)
print(min_y)
print(max_x - min_x)
print(max_y - min_y)
'
  )
  if [[ "${#geometry[@]}" -ne 4 ]]; then
    echo "Failed to detect full desktop geometry from xrandr." >&2
    exit 1
  fi
  screen_x="${geometry[0]}"
  screen_y="${geometry[1]}"
  screen_width="${geometry[2]}"
  screen_height="${geometry[3]}"
fi

if [[ "${screen_width}" -le 0 || "${screen_height}" -le 0 ]]; then
  echo "Invalid recording geometry: ${screen_width}x${screen_height}" >&2
  exit 1
fi

video_template="${artifact_root}/defense_demo_%d.webm"
metadata_file="${artifact_root}/defense_demo.json"
log_file="${artifact_root}/defense_demo.log"
stop_file="${artifact_root}/defense_demo.stop"
pid_file="${artifact_root}/defense_demo.pid"
cmd_file="${artifact_root}/start_command.sh"

rm -f "${stop_file}"
recording_backend=""
video_file=""
obs_config_root=""

if [[ "${backend}" == "auto" ]]; then
  if command -v obs >/dev/null 2>&1; then
    backend="obs"
  elif command -v ffmpeg >/dev/null 2>&1; then
    backend="ffmpeg"
  else
    backend="gnome_screencast"
  fi
fi

if [[ "${backend}" == "obs" ]]; then
  obs_config_root="${artifact_root}/obs_xdg"
  mkdir -p "${obs_config_root}/obs-studio/basic/profiles/MedipickDefense" "${obs_config_root}/obs-studio/basic/scenes"
  cat > "${obs_config_root}/obs-studio/global.ini" <<EOF
[General]
Pre19Defaults=false
Pre21Defaults=false
Pre23Defaults=false
Pre24.1Defaults=false
FirstRun=false

[Basic]
Profile=MedipickDefense
ProfileDir=MedipickDefense
SceneCollection=MedipickDefense
SceneCollectionFile=MedipickDefense
EOF
  cat > "${obs_config_root}/obs-studio/basic/profiles/MedipickDefense/basic.ini" <<EOF
[General]
Name=MedipickDefense

[Video]
BaseCX=${screen_width}
BaseCY=${screen_height}
OutputCX=${screen_width}
OutputCY=${screen_height}
FPSCommon=${fps}

[Output]
Mode=Simple
FilenameFormatting=defense_demo

[SimpleOutput]
FilePath=${artifact_root}
RecFormat=mkv
RecQuality=Small
VBitrate=2500
StreamEncoder=x264
RecEncoder=x264
EOF
  cat > "${obs_config_root}/obs-studio/basic/scenes/MedipickDefense.json" <<'EOF'
{"current_program_scene":"Desktop","current_scene":"Desktop","name":"MedipickDefense","groups":[],"modules":{"auto-scene-switcher":{"active":false,"interval":300,"non_matching_scene":"","switch_if_not_matching":false,"switches":[]},"output-timer":{"autoStartRecordTimer":false,"autoStartStreamTimer":false,"pauseRecordTimer":true,"recordTimerHours":0,"recordTimerMinutes":0,"recordTimerSeconds":30,"streamTimerHours":0,"streamTimerMinutes":0,"streamTimerSeconds":30},"scripts-tool":[]},"preview_locked":false,"quick_transitions":[{"duration":300,"fade_to_black":false,"hotkeys":[],"id":1,"name":"Cut"},{"duration":300,"fade_to_black":false,"hotkeys":[],"id":2,"name":"Fade"}],"saved_projectors":[],"scaling_enabled":false,"scaling_level":0,"scaling_off_x":0.0,"scaling_off_y":0.0,"scene_order":[{"name":"Desktop"}],"sources":[{"balance":0.5,"deinterlace_field_order":0,"deinterlace_mode":0,"enabled":true,"flags":0,"hotkeys":{"OBSBasic.SelectScene":[]},"id":"scene","mixers":0,"monitoring_type":0,"muted":false,"name":"Desktop","prev_ver":453115907,"private_settings":{},"push-to-mute":false,"push-to-mute-delay":0,"push-to-talk":false,"push-to-talk-delay":0,"settings":{"custom_size":false,"id_counter":1,"items":[{"align":5,"blend_method":"default","blend_type":"normal","bounds":{"x":0,"y":0},"bounds_align":0,"bounds_type":0,"crop_bottom":0,"crop_left":0,"crop_right":0,"crop_top":0,"group_item_backup":false,"hide_transition":{"duration":0},"id":1,"locked":false,"name":"Screen Capture (XSHM)","pos":{"x":0.0,"y":0.0},"private_settings":{},"rot":0.0,"scale":{"x":1.0,"y":1.0},"scale_filter":"disable","show_transition":{"duration":0},"visible":true}]},"sync":0,"versioned_id":"scene","volume":1.0},{"balance":0.5,"deinterlace_field_order":0,"deinterlace_mode":0,"enabled":true,"flags":0,"hotkeys":{},"id":"xshm_input","mixers":0,"monitoring_type":0,"muted":false,"name":"Screen Capture (XSHM)","prev_ver":469762051,"private_settings":{},"push-to-mute":false,"push-to-mute-delay":0,"push-to-talk":false,"push-to-talk-delay":0,"settings":{},"sync":0,"versioned_id":"xshm_input","volume":1.0}],"transition_duration":300,"transitions":[{"fixed":true,"id":"fade_transition","name":"Fade","settings":{},"versioned_id":"fade_transition"}]}
EOF
  recording_backend="obs"
  video_file="${artifact_root}/defense_demo.mkv"
  nohup env XDG_CONFIG_HOME="${obs_config_root}" obs --multi --startrecording --minimize-to-tray >"${log_file}" 2>&1 < /dev/null &
  echo "$!" > "${pid_file}"
  sleep 3
  if ! kill -0 "$(cat "${pid_file}")" 2>/dev/null; then
    echo "Failed to start OBS recorder." >&2
    [[ -f "${log_file}" ]] && cat "${log_file}" >&2 || true
    exit 1
  fi
elif [[ "${backend}" == "ffmpeg" ]] && command -v ffmpeg >/dev/null 2>&1; then
  video_file="${artifact_root}/defense_demo.mp4"
  recording_backend="ffmpeg"
  nohup ffmpeg -y \
    -loglevel error \
    -video_size "${screen_width}x${screen_height}" \
    -framerate "${fps}" \
    -f x11grab \
    -draw_mouse 1 \
    -i "${DISPLAY}+${screen_x},${screen_y}" \
    -vcodec libx264 \
    -preset ultrafast \
    -pix_fmt yuv420p \
    "${video_file}" >"${log_file}" 2>&1 < /dev/null &
  echo "$!" > "${pid_file}"
  sleep 1
  if ! kill -0 "$(cat "${pid_file}")" 2>/dev/null; then
    echo "Failed to start ffmpeg recorder." >&2
    [[ -f "${log_file}" ]] && cat "${log_file}" >&2 || true
    exit 1
  fi
else
  gdbus_output="$(
    gdbus call --session \
      --dest org.gnome.Shell.Screencast \
      --object-path /org/gnome/Shell/Screencast \
      --method org.gnome.Shell.Screencast.ScreencastArea \
      "${screen_x}" "${screen_y}" "${screen_width}" "${screen_height}" \
      "${video_template}" \
      "{'framerate': <uint32 ${fps}>, 'draw-cursor': <true>}" 2>"${log_file}"
  )"

  mapfile -t screencast_fields < <(
    python3 - <<'PY' "${gdbus_output}"
import re
import sys

payload = sys.argv[1].strip()
match = re.match(r"\((true|false), '([^']*)'\)", payload)
if not match:
    sys.exit(1)
print(match.group(1))
print(match.group(2))
PY
  )

  if [[ "${#screencast_fields[@]}" -lt 2 || "${screencast_fields[0]}" != "true" ]]; then
    echo "Failed to start GNOME screencast." >&2
    [[ -f "${log_file}" ]] && cat "${log_file}" >&2 || true
    exit 1
  fi

  video_file="${screencast_fields[1]}"
  recording_backend="gnome_screencast"
fi

cat > "${cmd_file}" <<EOF
cd ${WORKSPACE_DIR}
bash scripts/start_defense_demo_recording.sh session_name:=${session_name}
EOF
chmod +x "${cmd_file}"

python3 - <<PY > "${metadata_file}"
import json

payload = {
    "success": True,
    "session_name": ${session_name@Q},
    "started_at": "$(date --iso-8601=seconds)",
    "screen_mode": ${screen_mode@Q},
    "crop_x": ${screen_x},
    "crop_y": ${screen_y},
    "crop_width": ${screen_width},
    "crop_height": ${screen_height},
    "fps": ${fps},
    "capture_timeout_sec": ${capture_timeout_sec},
    "recording_backend": ${recording_backend@Q},
    "video_file": ${video_file@Q},
    "pid_file": ${pid_file@Q},
    "stop_file": ${stop_file@Q},
    "log_file": ${log_file@Q},
}
print(json.dumps(payload, indent=2, ensure_ascii=False))
PY

cat > "${artifact_root}/README.md" <<EOF
# 答辩演示录屏会话

- 会话目录：\`${artifact_root}\`
- 视频文件：\`${video_file}\`
- 元数据：\`defense_demo.json\`
- 日志：\`defense_demo.log\`
- 停止文件：\`defense_demo.stop\`
- 录制区域：x=\`${screen_x}\` y=\`${screen_y}\` width=\`${screen_width}\` height=\`${screen_height}\`
- 启动时间：\`$(date --iso-8601=seconds)\`

停止录制：

\`\`\`bash
cd ${WORKSPACE_DIR}
bash scripts/stop_defense_demo_recording.sh ${artifact_root}
\`\`\`
EOF

echo "Recording started."
echo "Session dir: ${artifact_root}"
echo "Video file: ${video_file}"
echo "Stop command:"
echo "  cd ${WORKSPACE_DIR}"
echo "  bash scripts/stop_defense_demo_recording.sh ${artifact_root}"
