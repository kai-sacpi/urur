#!/usr/bin/env bash
set -euo pipefail

# Source existing, already-installed Ubuntu24 dependencies. No rebuild, robot
# driver, sensor process, or environment edits are performed by this entry point.
WM_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WM_PARENT="$(dirname "$WM_WORKSPACE")"
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$WM_PARENT/install/setup.bash" ]]; then
  echo 'Ubuntu24 ROS Jazzy and the existing ur-waypoint installation are required.' >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$WM_PARENT/.venv/bin/activate"
source "$WM_PARENT/install/setup.bash"
set -u
export ROS_DOMAIN_ID=219
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS2CLI_NO_DAEMON=1
export PYTHONNOUSERSITE=1
export PYTHONUNBUFFERED=1
WM_CUROBO_COMMIT="$(awk -F': *' '$1 == "commit" {print $2; exit}' "$WM_WORKSPACE/curobo.lock")"
WM_CUROBO_SOURCE="$WM_PARENT/.deps/curobo-${WM_CUROBO_COMMIT}"
[[ -f "$WM_CUROBO_SOURCE/curobo/__init__.py" ]] || {
  echo "Locked cuRobo runtime is unavailable: $WM_CUROBO_SOURCE" >&2; exit 2;
}
WM_VENV_SITE=""
for WM_CANDIDATE in "$WM_PARENT"/.venv/lib/python*/site-packages; do
  [[ -d "$WM_CANDIDATE" ]] || continue
  [[ -z "$WM_VENV_SITE" ]] || { echo 'Ambiguous venv site-packages.' >&2; exit 2; }
  WM_VENV_SITE="$WM_CANDIDATE"
done
[[ -n "$WM_VENV_SITE" ]] || { echo 'Venv site-packages is unavailable.' >&2; exit 2; }
export PYTHONPATH="$WM_CUROBO_SOURCE:$WM_VENV_SITE${PYTHONPATH:+:$PYTHONPATH}"
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS
cd "$WM_WORKSPACE"
exec python "$WM_WORKSPACE/runtime/batch_recordings.py" --workspace "$WM_WORKSPACE" "$@"
