#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ZIP_PATH="${WORKSPACE_DIR}/assets/simple5_urdf.zip"
TARGET_DIR="${WORKSPACE_DIR}/src/medipick_simple3_description/vendor/simple5_current"
TEMP_DIR="${WORKSPACE_DIR}/.tmp_simple5_vendor_sync"

if [[ ! -f "${ZIP_PATH}" ]]; then
  echo "未找到 ${ZIP_PATH}"
  exit 1
fi

rm -rf "${TEMP_DIR}"
mkdir -p "${TEMP_DIR}"
unzip -q "${ZIP_PATH}" -d "${TEMP_DIR}"

SOURCE_DIR="${TEMP_DIR}"
if [[ -d "${TEMP_DIR}/simple5_urdf" ]]; then
  SOURCE_DIR="${TEMP_DIR}/simple5_urdf"
fi

mkdir -p "${TARGET_DIR}"
rsync -a --delete "${SOURCE_DIR}/" "${TARGET_DIR}/"
rm -rf "${TEMP_DIR}"

echo "已同步 simple5 原始资产到 ${TARGET_DIR}"
