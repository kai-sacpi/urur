#!/usr/bin/env bash
set -euo pipefail

WM_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WM_PARENT="$(dirname "$WM_WORKSPACE")"
set +u
source /opt/ros/jazzy/setup.bash
source "$WM_PARENT/.venv/bin/activate"
source "$WM_PARENT/install/setup.bash"
set -u
WM_COMMIT="$(awk -F': *' '$1 == "commit" {print $2; exit}' "$WM_WORKSPACE/curobo.lock")"
WM_CUROBO="$WM_PARENT/.deps/curobo-$WM_COMMIT"
[[ -f "$WM_CUROBO/curobo/__init__.py" ]] || {
  echo "Pinned cuRobo is unavailable: $WM_CUROBO" >&2
  exit 2
}
export PYTHONPATH="$WM_PARENT/src/hx_waypoint_curobo:$WM_PARENT/src/hx_waypoint_nucleo:$WM_CUROBO${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
# Offline only: no ROS node, serial device, robot driver or action is started.
cd "$WM_WORKSPACE"
exec "$WM_PARENT/.venv/bin/python" -m collection.validate_parking --workspace "$WM_WORKSPACE" "$@"
