#!/usr/bin/env bash

# Seal mock recordings from already validated v2 compile runs.  The
# BuildRecording action is called directly so the immutable compile evidence
# is reused and no second GPU solve is performed.

set -Eeuo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
PREFIX="icassp_f64_mj_feasible_v2"
ROS_DOMAIN_ID="${ICASSPSIM_ROS_DOMAIN_ID:-229}"
RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$WORKSPACE/runs/icassp_f64_feasible_v2_recordings_${RUN_STAMP}"
RECORDINGS_ROOT="$WORKSPACE/environments/$ENVIRONMENT_ID/recordings"

ids=(
  ${PREFIX}_line_x_vertical_z09
  ${PREFIX}_line_x_tilt90_z09
  ${PREFIX}_line_y_vertical_z09
  ${PREFIX}_line_y_tilt90_z09
  ${PREFIX}_line_x_vertical_z13
  ${PREFIX}_line_x_tilt90_z13
  ${PREFIX}_line_y_vertical_z13
  ${PREFIX}_line_y_tilt90_z13
  ${PREFIX}_line_x_vertical_z15
  ${PREFIX}_line_x_tilt90_z15
  ${PREFIX}_line_y_vertical_z15
  ${PREFIX}_line_y_tilt90_z15
  ${PREFIX}_eight_tilt90_z13
)

mkdir -p "$RUN_DIR/logs"
printf 'order,trajectory_id,compile_run,build_status,inspect_status,recording_id,recording_path,build_log,inspect_log\n' > "$RUN_DIR/summary.csv"

set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/.venv/bin/activate"
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS2CLI_NO_DAEMON=1
export PYTHONNOUSERSITE=1
export PYTHONPATH="$WORKSPACE/.deps/curobo-a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9:$WORKSPACE/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

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

printf '[batch] workspace=%s\n' "$WORKSPACE"
printf '[batch] environment=%s prefix=%s ROS_DOMAIN_ID=%s\n' "$ENVIRONMENT_ID" "$PREFIX" "$ROS_DOMAIN_ID"
printf '[batch] run_dir=%s\n' "$RUN_DIR"

ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 environment_id:="$ENVIRONMENT_ID" workspace:="$WORKSPACE" \
  launch_rviz:=false > "$RUN_DIR/simulation.launch.log" 2>&1 &
sim_pid=$!

ready=0
for _ in $(seq 1 180); do
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    printf '[batch] simulation exited before readiness; see %s\n' "$RUN_DIR/simulation.launch.log" >&2
    exit 2
  fi
  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -qx '/hx_waypoint_runtime' <<<"$nodes" \
    && grep -qx '/joint_trajectory_controller' <<<"$nodes"; then
    servers="$(ros2 action info /hx_waypoint/build_recording 2>/dev/null || true)"
    if grep -q '^Action servers: 1$' <<<"$servers"; then
      ready=1
      break
    fi
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  printf '[batch] simulation readiness timeout; see %s\n' "$RUN_DIR/simulation.launch.log" >&2
  exit 3
fi
printf '[batch] simulation ready\n'

find_existing_validated() {
  local id="$1" path
  while IFS= read -r path; do
    if grep -q '^status: validated$' "$path/manifest.yaml" 2>/dev/null; then
      basename -- "$path"
      return 0
    fi
  done < <(find "$RECORDINGS_ROOT" -maxdepth 1 -mindepth 1 -type d -name "${id}__*" -printf '%p\n' | sort -r)
  return 1
}

for index in "${!ids[@]}"; do
  id="${ids[$index]}"
  printf '[batch] [%d/%d] %s\n' "$((index + 1))" "${#ids[@]}" "$id"
  compile_dir="$(find "$WORKSPACE/runs" -maxdepth 1 -mindepth 1 -type d -name "${id}__compiled__*" -printf '%p\n' | sort -r | head -n 1)"
  if [[ -z "$compile_dir" || ! -f "$compile_dir/source_tip_trajectory.csv" || ! -f "$compile_dir/joint_trajectory.csv" || ! -f "$compile_dir/compile_report.yaml" ]]; then
    printf '%d,%s,,MISSING_COMPILE,125,,,,\n' "$index" "$id" >> "$RUN_DIR/summary.csv"
    printf '[batch] %s missing compile run\n' "$id" >&2
    continue
  fi
  compile_name="$(basename -- "$compile_dir")"

  existing="$(find_existing_validated "$id" || true)"
  if [[ -n "$existing" ]]; then
    inspect_log="$RUN_DIR/logs/${id}.inspect-existing.log"
    set +e
    ros2 run hx_waypoint_tools tipdraw --workspace "$WORKSPACE" inspect \
      --env "$ENVIRONMENT_ID" --recording "$existing" > "$inspect_log" 2>&1
    inspect_rc=$?
    set -e
    printf '%d,%s,%s,EXISTING,%s,%s,%s,,%s\n' \
      "$index" "$id" "$compile_name" "$inspect_rc" "$existing" \
      "$RECORDINGS_ROOT/$existing" "$inspect_log" >> "$RUN_DIR/summary.csv"
    printf '[batch] %s existing=%s inspect=%s\n' "$id" "$existing" "$inspect_rc"
    continue
  fi

  build_log="$RUN_DIR/logs/${id}.build.log"
  inspect_log="$RUN_DIR/logs/${id}.inspect.log"
  goal="{environment_id: $ENVIRONMENT_ID, environment_path: $WORKSPACE/environments/$ENVIRONMENT_ID, trajectory_id: $id, source_trajectory_path: $compile_dir/source_tip_trajectory.csv, compiled_trajectory_path: $compile_dir/joint_trajectory.csv, launch_rviz: false}"
  set +e
  timeout 600 ros2 action send_goal /hx_waypoint/build_recording \
    hx_waypoint_interfaces/action/BuildRecording "$goal" --feedback \
    > "$build_log" 2>&1
  build_rc=$?
  set -e

  recording_id=""
  recording_path=""
  if [[ "$build_rc" == 0 ]] && grep -q '^    success: true$' "$build_log"; then
    recording_id="$(awk -F': ' '/^recording_id:/{value=$2} END{print value}' "$build_log")"
    recording_path="$(awk -F': ' '/^recording_path:/{value=$2} END{print value}' "$build_log")"
  fi

  inspect_rc=125
  if [[ -n "$recording_id" ]]; then
    set +e
    ros2 run hx_waypoint_tools tipdraw --workspace "$WORKSPACE" inspect \
      --env "$ENVIRONMENT_ID" --recording "$recording_id" > "$inspect_log" 2>&1
    inspect_rc=$?
    set -e
  fi

  printf '%d,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$index" "$id" "$compile_name" "$build_rc" "$inspect_rc" \
    "$recording_id" "$recording_path" "$build_log" "$inspect_log" \
    >> "$RUN_DIR/summary.csv"
  if [[ "$build_rc" == 0 && "$inspect_rc" == 0 ]]; then
    printf '[batch] %s PASS recording=%s\n' "$id" "$recording_id"
  else
    printf '[batch] %s FAIL build=%s inspect=%s (see %s)\n' "$id" "$build_rc" "$inspect_rc" "$build_log" >&2
  fi
done

printf '[batch] complete summary=%s\n' "$RUN_DIR/summary.csv"
