#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f .venv/bin/activate ]]; then
  source .venv/bin/activate
fi
export PYTHONNOUSERSITE=1
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

exec python3 -m hx_waypoint_tools.tcp_gui --workspace "$ROOT" "$@"
