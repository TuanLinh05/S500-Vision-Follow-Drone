#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"

if [[ "${S500_PROPS_REMOVED:-}" != "YES" ]]; then
  echo "Refusing P1: remove all propellers, then export S500_PROPS_REMOVED=YES" >&2
  exit 2
fi

if [[ ! -x .venv/bin/s500-companion ]]; then
  echo "Virtual environment missing. Run: bash deploy/install_up7000.sh" >&2
  exit 2
fi

. .venv/bin/activate
mkdir -p logs
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
CSV="logs/p1_telemetry_${STAMP}.csv"
REPORT="logs/p1_validation_${STAMP}.json"

s500-companion discover | tee "logs/p1_usb_devices_${STAMP}.txt"
dmesg -T | tail -n 80 > "logs/p1_dmesg_${STAMP}.txt" || true
s500-companion probe --duration 10
s500-companion record --duration 1800 --rate 5 --output "$CSV"
s500-companion validate-log --input "$CSV" --output "$REPORT"

echo "P1 artifacts:"
echo "  $CSV"
echo "  $REPORT"
echo "  logs/p1_usb_devices_${STAMP}.txt"
echo "  logs/p1_dmesg_${STAMP}.txt"

