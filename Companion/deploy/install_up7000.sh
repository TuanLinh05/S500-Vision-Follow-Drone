#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

python3 -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .

echo "Installed s500-companion in $PROJECT_DIR/.venv"
echo "If USB access is denied, run: sudo usermod -aG dialout \"$USER\""
echo "Then log out and log back in before retrying."

