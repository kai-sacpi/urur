#!/usr/bin/env bash
set -euo pipefail

# Continue the already-live offline GPU producer; never starts a replacement.
# All outputs are new immutable versions. A failure stops this chain in place.
[[ "$(uname -s)" == Linux ]] || { echo "Run this on Ubuntu24 only." >&2; exit 2; }
[[ $# == 1 && "$1" =~ ^[0-9]+$ && "$1" -gt 1 ]] || {
  echo "Usage: bash ops/finish_parking_delivery.sh LIVE_PARKING_GPU_PRODUCER_PID" >&2
  exit 2
}
WM_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WM_PARENT="$(dirname "$WM_WORKSPACE")"
WM_PYTHON="$WM_PARENT/.venv/bin/python"
[[ -x "$WM_PYTHON" ]] || { echo "Existing ur-waypoint Python environment is required." >&2; exit 2; }
export PYTHONPATH="$WM_WORKSPACE:$WM_PARENT/src/hx_waypoint_curobo:$WM_PARENT/src/hx_waypoint_tools:$WM_PARENT/src/hx_waypoint_nucleo${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
cd "$WM_WORKSPACE"
WM_VALIDATION="$WM_WORKSPACE/runs/parking_gpu_validation_20261017_v2/validation.json"
WM_BUNDLE="$WM_WORKSPACE/collection/prepared_parking/parking_20261017_v2"
WM_MOCK="$WM_WORKSPACE/runs/parking_mock_validation_20261017_v2"

WM_CHILD=""
stop_owned() {
  trap - INT TERM
  if [[ -n "$WM_CHILD" ]]; then
    kill -TERM "$WM_CHILD" 2>/dev/null || true
    wait "$WM_CHILD" || true
  fi
  exit 130
}
trap stop_owned INT TERM
run_owned() {
  local status
  "$@" &
  WM_CHILD=$!
  if wait "$WM_CHILD"; then status=0; else status=$?; fi
  WM_CHILD=""
  return "$status"
}

printf 'Waiting for existing GPU parking producer PID %s; no new runtime started.\n' "$1"
run_owned "$WM_PYTHON" -c '
from pathlib import Path
import sys
from collection.mock_parking import wait_for_validation
wait_for_validation(Path(sys.argv[1]), wait=True, producer_pid=int(sys.argv[2]))
' "$WM_VALIDATION" "$1"

printf 'GPU report accepted; creating the exact-byte parking prepare bundle.\n'
run_owned "$WM_PYTHON" -m collection.prepared_parking create \
  --workspace "$WM_WORKSPACE" --validation "$WM_VALIDATION" \
  --producer-code-root "$(dirname "$WM_VALIDATION")" --output "$WM_BUNDLE"

printf 'Running independent mock-only parking playback in isolated domain 223.\n'
run_owned bash collection/run_mock_parking.sh \
  --validation "$WM_VALIDATION" --bundle "$WM_BUNDLE" --output "$WM_MOCK"

printf 'Auditing all 336 recordings and parking evidence for final delivery.\n'
run_owned bash ops/overnight_delivery.sh --once
