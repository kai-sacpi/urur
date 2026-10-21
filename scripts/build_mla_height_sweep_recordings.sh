#!/usr/bin/env bash

# Seal and inspect mock recordings from the compile-only MLA height sweep.
# The action consumes the exact compiled/source files; it does not recompile,
# scale, retime, clip, crop, substitute, or drop source waypoints.

set -uo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
INDEX="$WORKSPACE/environments/mla/trajectories/icassp_f64_mj_height_sweep_feasible_v2_authority_index.csv"
ROS_DOMAIN_ID="${MLA_HEIGHT_SWEEP_RECORD_ROS_DOMAIN_ID:-218}"
RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$WORKSPACE/runs/mla_height_sweep_recordings_${RUN_STAMP}"
RECORDINGS_ROOT="$WORKSPACE/environments/$ENVIRONMENT_ID/recordings"

if [[ ! -f "$INDEX" ]]; then
  printf '[record] missing authority index: %s\n' "$INDEX" >&2
  exit 2
fi
mkdir -p "$RUN_DIR/logs"

set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/.venv/bin/activate"
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS2CLI_NO_DAEMON=1 PYTHONNOUSERSITE=1
export PYTHONPATH="$WORKSPACE/.deps/curobo-a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9:$WORKSPACE/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

mapfile -t ids < <(tail -n +2 "$INDEX" | cut -d, -f5 | sed -e 's/\.csv$//')
if [[ "${#ids[@]}" -ne 20 ]]; then
  printf '[record] expected 20 authority IDs, got %d\n' "${#ids[@]}" >&2
  exit 2
fi

printf 'order,trajectory_id,compile_run,build_status,inspect_status,recording_id,recording_path,build_log,inspect_log\n' \
  > "$RUN_DIR/summary.csv"
{
  printf 'schema_version: 1\n'
  printf 'environment_id: mla\n'
  printf 'ros_domain_id: %s\n' "$ROS_DOMAIN_ID"
  printf 'authority_index: %s\n' "$INDEX"
  printf 'authority_policy: exact immutable waypoint and time_s hard constraints\n'
  printf 'trajectories:\n'
  for id in "${ids[@]}"; do printf '  - %s\n' "$id"; done
} > "$RUN_DIR/run_manifest.yaml"

sim_pid=""
cleanup() {
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    kill -INT "$sim_pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$sim_pid" 2>/dev/null || break
      sleep 1
    done
    kill -TERM "$sim_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

printf '[record] run_dir=%s ROS_DOMAIN_ID=%s\n' "$RUN_DIR" "$ROS_DOMAIN_ID"
ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 environment_id:="$ENVIRONMENT_ID" workspace:="$WORKSPACE" \
  launch_rviz:=false > "$RUN_DIR/simulation.launch.log" 2>&1 &
sim_pid=$!

ready=0
for _ in $(seq 1 180); do
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    printf '[record] simulation exited early; see %s\n' "$RUN_DIR/simulation.launch.log" >&2
    exit 2
  fi
  nodes="$(ros2 node list 2>/dev/null || true)"
  servers="$(ros2 action info /hx_waypoint/build_recording 2>/dev/null || true)"
  if grep -qx '/hx_waypoint_runtime' <<<"$nodes" \
    && grep -qx '/joint_trajectory_controller' <<<"$nodes" \
    && grep -q '^Action servers: 1$' <<<"$servers"; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" -ne 1 ]]; then
  printf '[record] simulation readiness timeout\n' >&2
  exit 3
fi
printf '[record] simulation ready\n'

find_existing_validated() {
  local id="$1" path
  while IFS= read -r path; do
    if grep -q '^status: validated$' "$path/manifest.yaml" 2>/dev/null; then
      basename -- "$path"
      return 0
    fi
  done < <(find "$RECORDINGS_ROOT" -maxdepth 1 -mindepth 1 -type d \
    -name "${id}__*" -printf '%p\n' | sort -r)
  return 1
}

overall=0
for index in "${!ids[@]}"; do
  id="${ids[$index]}"
  printf '[record] [%02d/%02d] %s\n' "$((index + 1))" "${#ids[@]}" "$id"
  compile_dir="$(find "$WORKSPACE/runs" -maxdepth 1 -mindepth 1 -type d \
    -name "${id}__compiled__*" -printf '%p\n' | sort -r | head -n 1)"
  if [[ -z "$compile_dir" || ! -f "$compile_dir/source_tip_trajectory.csv" \
    || ! -f "$compile_dir/joint_trajectory.csv" \
    || ! -f "$compile_dir/compile_report.yaml" ]]; then
    printf '%s,%s,,MISSING_COMPILE,125,,,,\n' "$index" "$id" >> "$RUN_DIR/summary.csv"
    printf '[record] %s missing compile run\n' "$id" >&2
    overall=1
    continue
  fi
  compile_name="$(basename -- "$compile_dir")"
  existing="$(find_existing_validated "$id" || true)"
  if [[ -n "$existing" ]]; then
    inspect_log="$RUN_DIR/logs/${id}.inspect-existing.log"
    ros2 run hx_waypoint_tools tipdraw --workspace "$WORKSPACE" inspect \
      --env "$ENVIRONMENT_ID" --recording "$existing" > "$inspect_log" 2>&1
    inspect_rc=$?
    printf '%s,%s,%s,EXISTING,%s,%s,%s,,%s\n' \
      "$index" "$id" "$compile_name" "$inspect_rc" "$existing" \
      "$RECORDINGS_ROOT/$existing" "$inspect_log" >> "$RUN_DIR/summary.csv"
    [[ "$inspect_rc" -eq 0 ]] || overall=1
    continue
  fi

  build_log="$RUN_DIR/logs/${id}.build.log"
  inspect_log="$RUN_DIR/logs/${id}.inspect.log"
  goal="{environment_id: $ENVIRONMENT_ID, environment_path: $WORKSPACE/environments/$ENVIRONMENT_ID, trajectory_id: $id, source_trajectory_path: $compile_dir/source_tip_trajectory.csv, compiled_trajectory_path: $compile_dir/joint_trajectory.csv, launch_rviz: false}"
  timeout 900 ros2 action send_goal /hx_waypoint/build_recording \
    hx_waypoint_interfaces/action/BuildRecording "$goal" --feedback \
    > "$build_log" 2>&1
  build_rc=$?

  recording_id=""
  recording_path=""
  if [[ "$build_rc" -eq 0 ]] && grep -q '^    success: true$' "$build_log"; then
    recording_id="$(awk -F': ' '/^recording_id:/{value=$2} END{print value}' "$build_log")"
    recording_path="$(awk -F': ' '/^recording_path:/{value=$2} END{print value}' "$build_log")"
  fi
  inspect_rc=125
  if [[ -n "$recording_id" ]]; then
    ros2 run hx_waypoint_tools tipdraw --workspace "$WORKSPACE" inspect \
      --env "$ENVIRONMENT_ID" --recording "$recording_id" \
      > "$inspect_log" 2>&1
    inspect_rc=$?
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$index" "$id" "$compile_name" "$build_rc" "$inspect_rc" \
    "$recording_id" "$recording_path" "$build_log" "$inspect_log" \
    >> "$RUN_DIR/summary.csv"
  if [[ "$build_rc" -eq 0 && "$inspect_rc" -eq 0 ]]; then
    printf '[record] %s PASS recording=%s\n' "$id" "$recording_id"
  else
    printf '[record] %s FAIL build=%s inspect=%s\n' "$id" "$build_rc" "$inspect_rc" >&2
    overall=1
  fi
done

printf '[record] complete overall=%s summary=%s\n' "$overall" "$RUN_DIR/summary.csv"
exit "$overall"
