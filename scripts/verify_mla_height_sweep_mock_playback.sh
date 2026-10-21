#!/usr/bin/env bash

# Independently replay the sealed 20-route height sweep in a fresh mock
# runtime.  This never connects to a real controller or alters a recording.

set -uo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
INDEX="$WORKSPACE/environments/mla/trajectories/icassp_f64_mj_height_sweep_validated_recordings_index.csv"
ROS_DOMAIN_ID="${MLA_HEIGHT_SWEEP_PLAY_ROS_DOMAIN_ID:-219}"
RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$WORKSPACE/runs/mla_height_sweep_independent_mock_playback_${RUN_STAMP}"

if [[ ! -f "$INDEX" ]]; then
  printf '[playback] missing validated recording index: %s\n' "$INDEX" >&2
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/.venv/bin/activate"
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS2CLI_NO_DAEMON=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$WORKSPACE/.deps/curobo-a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9:$WORKSPACE/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

mapfile -t trajectories < <(tail -n +2 "$INDEX" | cut -d, -f5)
mapfile -t recordings < <(tail -n +2 "$INDEX" | cut -d, -f8)
if [[ "${#recordings[@]}" -ne 20 || "${#trajectories[@]}" -ne 20 ]]; then
  printf '[playback] recording index must contain exactly 20 routes\n' >&2
  exit 2
fi
for index in "${!recordings[@]}"; do
  [[ "${recordings[$index]}" =~ ^[A-Za-z0-9_.-]+$ ]] || exit 2
  [[ "${trajectories[$index]}" =~ ^[A-Za-z0-9_.-]+$ ]] || exit 2
  [[ -d "$WORKSPACE/environments/mla/recordings/${recordings[$index]}" ]] || exit 2
done

mkdir -p "$RUN_DIR/logs" "$RUN_DIR/telemetry"
printf 'order,trajectory_id,recording_id,play_status,telemetry_path,play_log\n' \
  > "$RUN_DIR/summary.csv"
{
  printf 'schema_version: 1\n'
  printf 'environment_id: mla\n'
  printf 'ros_domain_id: %s\n' "$ROS_DOMAIN_ID"
  printf 'recording_index: %s\n' "$INDEX"
  printf 'route_count: 20\n'
  printf 'policy: sealed exact playback without scaling, retiming, clipping, cropping, substitution, or point dropping\n'
} > "$RUN_DIR/run_manifest.yaml"

sim_pid=""
cleanup() {
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    kill -INT -- "-$sim_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$sim_pid" 2>/dev/null || break
      sleep 1
    done
    kill -TERM -- "-$sim_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 environment_id:=mla workspace:="$WORKSPACE" launch_rviz:=false \
  > "$RUN_DIR/simulation.launch.log" 2>&1 &
sim_pid=$!
ready=0
for _ in $(seq 1 180); do
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    printf '[playback] simulation exited early\n' >&2
    exit 2
  fi
  nodes="$(ros2 node list 2>/dev/null || true)"
  actions="$(ros2 action list 2>/dev/null || true)"
  if grep -qx '/hx_waypoint_runtime' <<<"$nodes" \
    && grep -qx '/joint_trajectory_controller' <<<"$nodes" \
    && grep -qx '/hx_waypoint/play_recording' <<<"$actions"; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" -ne 1 ]]; then
  printf '[playback] simulation readiness timeout\n' >&2
  exit 3
fi
printf '[playback] simulation ready run_dir=%s\n' "$RUN_DIR"

overall=0
for index in "${!recordings[@]}"; do
  recording="${recordings[$index]}"
  trajectory="${trajectories[$index]}"
  play_log="$RUN_DIR/logs/${trajectory}.play.log"
  printf '[playback] [%02d/%02d] %s\n' "$((index + 1))" "${#recordings[@]}" "$trajectory"
  timeout 300 ros2 run hx_waypoint_tools tipdraw \
    --workspace "$WORKSPACE" play --env mla \
    --recording "$recording" --auto-prepare > "$play_log" 2>&1
  play_status=$?
  telemetry_path=""
  if [[ "$play_status" -eq 0 ]]; then
    telemetry_path="$(tail -n 1 "$play_log")"
    if [[ -f "$telemetry_path" ]]; then
      cp -- "$telemetry_path" "$RUN_DIR/telemetry/${trajectory}.execution_telemetry.csv"
    else
      telemetry_path=""
    fi
  else
    overall=1
  fi
  printf '%s,%s,%s,%s,%s,%s\n' \
    "$index" "$trajectory" "$recording" "$play_status" "$telemetry_path" "$play_log" \
    >> "$RUN_DIR/summary.csv"
  printf '[playback] %s status=%s\n' "$trajectory" "$play_status"
done

printf '[playback] complete overall=%s summary=%s\n' "$overall" "$RUN_DIR/summary.csv"
exit "$overall"
