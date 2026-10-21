#!/usr/bin/env bash
set -euo pipefail

# Evidence audit/export only: never starts a runtime, GPU job or serial session.
WM_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WM_PARENT="$(dirname "$WM_WORKSPACE")"
[[ -x "$WM_PARENT/.venv/bin/python" ]] || {
  echo "Existing ur-waypoint Python environment is required." >&2
  exit 2
}
export PYTHONPATH="$WM_WORKSPACE:$WM_PARENT/src/hx_waypoint_curobo:$WM_PARENT/src/hx_waypoint_tools:$WM_PARENT/src/hx_waypoint_nucleo${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
cd "$WM_WORKSPACE"
exec "$WM_PARENT/.venv/bin/python" "$WM_WORKSPACE/ops/export_overnight_delivery.py" --workspace "$WM_WORKSPACE" "$@"
