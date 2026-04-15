#!/usr/bin/env bash

set -euo pipefail

python3 -m ensurepip --upgrade >/dev/null 2>&1 || true
python3 -m pip install --user --upgrade pip
python3 -m pip install --user fastapi uvicorn pillow zeroconf

echo
echo "Mapping console host dependencies installed for user: ${USER}"
