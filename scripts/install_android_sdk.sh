#!/usr/bin/env bash

set -euo pipefail

SDK_ROOT="${ANDROID_SDK_ROOT:-${ANDROID_HOME:-$HOME/Android/Sdk}}"
CMDLINE_ROOT="${SDK_ROOT}/cmdline-tools"
LATEST_DIR="${CMDLINE_ROOT}/latest"
TOOLS_URL="https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip"

if [[ -d /usr/lib/jvm/java-17-openjdk-amd64 ]]; then
  export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
  export PATH="${JAVA_HOME}/bin:${PATH}"
fi

mkdir -p "${CMDLINE_ROOT}"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT

archive_path="${tmp_dir}/commandlinetools.zip"
echo "Downloading Android command-line tools to ${archive_path}"
curl -L "${TOOLS_URL}" -o "${archive_path}"

rm -rf "${LATEST_DIR}" "${CMDLINE_ROOT}/cmdline-tools"
unzip -q "${archive_path}" -d "${CMDLINE_ROOT}"
mv "${CMDLINE_ROOT}/cmdline-tools" "${LATEST_DIR}"

set +o pipefail
yes | "${LATEST_DIR}/bin/sdkmanager" --sdk_root="${SDK_ROOT}" --licenses >/dev/null
set -o pipefail
"${LATEST_DIR}/bin/sdkmanager" --sdk_root="${SDK_ROOT}" \
  "platform-tools" \
  "platforms;android-33" \
  "build-tools;33.0.2"

echo
echo "Android SDK installed at ${SDK_ROOT}"
