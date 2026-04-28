#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_NAME="$(basename "${WORKSPACE_DIR}")"
PARENT_DIR="$(dirname "${WORKSPACE_DIR}")"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${WORKSPACE_DIR}/deliverables"
ARCHIVE_PATH="${OUTPUT_DIR}/medipick_workspace_ubuntu22_migration_${TIMESTAMP}.tar.gz"

mkdir -p "${OUTPUT_DIR}"

cd "${PARENT_DIR}"

tar \
  --exclude="${WORKSPACE_NAME}/.git" \
  --exclude="${WORKSPACE_NAME}/build" \
  --exclude="${WORKSPACE_NAME}/install" \
  --exclude="${WORKSPACE_NAME}/log" \
  --exclude="${WORKSPACE_NAME}/deliverables" \
  --exclude="${WORKSPACE_NAME}/*/__pycache__" \
  --exclude="${WORKSPACE_NAME}/**/__pycache__" \
  --exclude="${WORKSPACE_NAME}/*.pyc" \
  --exclude="${WORKSPACE_NAME}/**/*.pyc" \
  --exclude="${WORKSPACE_NAME}/*.pyo" \
  --exclude="${WORKSPACE_NAME}/**/*.pyo" \
  --exclude="${WORKSPACE_NAME}/*:Zone.Identifier" \
  --exclude="${WORKSPACE_NAME}/**/*:Zone.Identifier" \
  -czf "${ARCHIVE_PATH}" \
  "${WORKSPACE_NAME}/README.md" \
  "${WORKSPACE_NAME}/assets" \
  "${WORKSPACE_NAME}/docs" \
  "${WORKSPACE_NAME}/environment" \
  "${WORKSPACE_NAME}/plan_prepare_rl" \
  "${WORKSPACE_NAME}/run_moveit_demo.sh" \
  "${WORKSPACE_NAME}/scripts" \
  "${WORKSPACE_NAME}/src"

echo "Created migration bundle:"
echo "  ${ARCHIVE_PATH}"
du -sh "${ARCHIVE_PATH}"
