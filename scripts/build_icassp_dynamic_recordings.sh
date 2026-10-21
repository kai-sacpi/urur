#!/usr/bin/env bash
# Build and inspect ICASSP dynamic authorities in one isolated mock
# ROS session.  The script never invokes a real controller and never edits a
# source CSV.  A non-zero status is retained in the batch summary instead of
# silently substituting, scaling, retiming, clipping, or dropping a route.

set -o pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
WORKSPACE="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_DOMAIN_ID="${ICASSPSIM_ROS_DOMAIN_ID:-221}"
PROFILE="${ICASSPSIM_PROFILE:-legacy}"
RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
case "$PROFILE" in
  legacy)
    ;;
  four_shapes)
    ;;
  four_shapes_complete)
    ;;
  f64_13)
    ;;
  f64_13_mj)
    ;;
  *)
    printf '[batch] unsupported profile: %s (use legacy, four_shapes, four_shapes_complete, f64_13, or f64_13_mj)\n' "$PROFILE" >&2
    exit 2
    ;;
esac
RUN_DIR="$WORKSPACE/runs/icassp_dynamic_${PROFILE}_sim_${RUN_STAMP}"
RECORDINGS_DIR="$WORKSPACE/environments/mla/recordings"

mkdir -p "$RUN_DIR/logs"

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/.venv/bin/activate"
export PYTHONNOUSERSITE=1
source "$WORKSPACE/install/setup.bash"
set -u

# ``ros2 run`` executes the generated console-script with the interpreter that
# was used when the ROS workspace was built (on Ubuntu24 this is
# ``/usr/bin/python3``), not necessarily the activated venv interpreter.  The
# locked cuRobo checkout and the venv site-packages therefore have to be made
# explicit in the inherited Python path.  Without this, the simulation nodes
# can come up while the build client fails closed with a misleading
# "locked cuRobo runtime is unavailable" error.
CUROBO_COMMIT="$(awk -F': *' '$1 == "commit" {print $2; exit}' "$WORKSPACE/curobo.lock")"
CUROBO_SOURCE="$WORKSPACE/.deps/curobo-${CUROBO_COMMIT}"
if [[ -z "$CUROBO_COMMIT" || ! -f "$CUROBO_SOURCE/curobo/__init__.py" ]]; then
  printf '[batch] locked cuRobo source is unavailable: %s\n' "$CUROBO_SOURCE" >&2
  exit 4
fi
VENV_SITE_PACKAGES=""
for candidate in "$WORKSPACE"/.venv/lib/python*/site-packages; do
  if [[ -d "$candidate" ]]; then
    if [[ -n "$VENV_SITE_PACKAGES" ]]; then
      printf '[batch] ambiguous venv site-packages directories\n' >&2
      exit 4
    fi
    VENV_SITE_PACKAGES="$candidate"
  fi
done
if [[ -z "$VENV_SITE_PACKAGES" ]]; then
  printf '[batch] venv site-packages directory is unavailable\n' >&2
  exit 4
fi
INHERITED_PYTHONPATH="${PYTHONPATH:-}"
PYTHONPATH="$CUROBO_SOURCE:$VENV_SITE_PACKAGES"
if [[ -n "$INHERITED_PYTHONPATH" ]]; then
  PYTHONPATH="$PYTHONPATH:$INHERITED_PYTHONPATH"
fi
export PYTHONPATH
export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS2CLI_NO_DAEMON=1

if [[ "$PROFILE" == "four_shapes" ]]; then
  ids=()
  for shape in circle rounded_square figure8 space; do
    for height in 07 09 11 13 15; do
      ids+=("icassp_dynamic_${shape}_z${height}")
    done
  done
elif [[ "$PROFILE" == "four_shapes_complete" ]]; then
  for shape in circle rounded_square figure8; do
    for height in 07 09 11 13 15; do
      ids+=("icassp_dynamic_${shape}_z${height}")
    done
  done
  for height in 07 09 11 13 15; do
    ids+=("icassp_dynamic_depth_loop_safe10mm_z${height}")
  done
elif [[ "$PROFILE" == "f64_13" ]]; then
  ids=(
    icassp_f64_line_x_vertical_z09
    icassp_f64_line_x_tilt90_z09
    icassp_f64_line_y_vertical_z09
    icassp_f64_line_y_tilt90_z09
    icassp_f64_line_x_vertical_z13
    icassp_f64_line_x_tilt90_z13
    icassp_f64_line_y_vertical_z13
    icassp_f64_line_y_tilt90_z13
    icassp_f64_line_x_vertical_z15
    icassp_f64_line_x_tilt90_z15
    icassp_f64_line_y_vertical_z15
    icassp_f64_line_y_tilt90_z15
    icassp_f64_eight_tilt90_z13
  )
elif [[ "$PROFILE" == "f64_13_mj" ]]; then
  ids=(
    icassp_f64_mj_line_x_vertical_z09
    icassp_f64_mj_line_x_tilt90_z09
    icassp_f64_mj_line_y_vertical_z09
    icassp_f64_mj_line_y_tilt90_z09
    icassp_f64_mj_line_x_vertical_z13
    icassp_f64_mj_line_x_tilt90_z13
    icassp_f64_mj_line_y_vertical_z13
    icassp_f64_mj_line_y_tilt90_z13
    icassp_f64_mj_line_x_vertical_z15
    icassp_f64_mj_line_x_tilt90_z15
    icassp_f64_mj_line_y_vertical_z15
    icassp_f64_mj_line_y_tilt90_z15
    icassp_f64_mj_eight_tilt90_z13
  )
else
  ids=(
    icassp_dynamic_concentric_z07
    icassp_dynamic_concentric_z09
    icassp_dynamic_concentric_z11
    icassp_dynamic_concentric_z13
    icassp_dynamic_concentric_z15
    icassp_dynamic_figure8_z11
    icassp_dynamic_figure8_z15
    icassp_dynamic_rounded_square_z15
    icassp_dynamic_space_z11
    icassp_dynamic_circle_r15_z11
  )
fi

printf 'run_id,trajectory_id,build_status,inspect_status,recording_id,recording_path\n' \
  > "$RUN_DIR/summary.csv"
{
  printf 'schema_version: 1\n'
  printf 'environment_id: mla\n'
  printf 'profile: %s\n' "$PROFILE"
  printf 'ros_domain_id: %s\n' "$ROS_DOMAIN_ID"
  printf 'workspace: %s\n' "$WORKSPACE"
  printf 'authority_policy: immutable waypoint and time_s hard constraints\n'
  printf 'simulation_command: ros2 launch hx_waypoint_bringup simulation.launch.py ur_type:=ur5 environment_id:=mla launch_rviz:=false\n'
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

printf '[batch] run directory: %s\n' "$RUN_DIR"
printf '[batch] ROS_DOMAIN_ID=%s (localhost-only; existing real launch is untouched)\n' "$ROS_DOMAIN_ID"

ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 \
  environment_id:=mla \
  workspace:="$WORKSPACE" \
  launch_rviz:=false \
  > "$RUN_DIR/simulation.launch.log" 2>&1 &
sim_pid=$!

ready=0
for _ in $(seq 1 180); do
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    printf '[batch] simulation exited before readiness; see %s\n' "$RUN_DIR/simulation.launch.log"
    exit 2
  fi
  nodes="$(ros2 node list 2>/dev/null || true)"
  if grep -qx '/hx_waypoint_runtime' <<<"$nodes" \
    && grep -qx '/joint_trajectory_controller' <<<"$nodes"; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  printf '[batch] simulation readiness timeout; see %s\n' "$RUN_DIR/simulation.launch.log"
  exit 3
fi
printf '[batch] simulation ready\n'

overall=0
for id in "${ids[@]}"; do
  build_log="$RUN_DIR/logs/${id}.build.log"
  inspect_log="$RUN_DIR/logs/${id}.inspect.log"
  printf '[batch] building %s\n' "$id"
  set +e
  ros2 run hx_waypoint_tools tipdraw \
    --workspace "$WORKSPACE" \
    build-recording \
    --env mla \
    --trajectory "$id" \
    --external-runtime \
    > "$build_log" 2>&1
  build_rc=$?

  recording_id=""
  recording_path=""
  if [[ "$build_rc" == 0 ]]; then
    recording_path="$(awk '/\/environments\/mla\/recordings\// {line=$0} END {print line}' "$build_log")"
    if [[ -n "$recording_path" ]]; then
      recording_id="$(basename -- "$recording_path")"
    else
      recording_id="$(find "$RECORDINGS_DIR" -maxdepth 1 -mindepth 1 -type d -name "${id}__*" -printf '%f\n' 2>/dev/null | sort | tail -1)"
      if [[ -n "$recording_id" ]]; then recording_path="$RECORDINGS_DIR/$recording_id"; fi
    fi
  fi

  inspect_rc=125
  if [[ "$build_rc" == 0 && -n "$recording_id" ]]; then
    ros2 run hx_waypoint_tools tipdraw \
      --workspace "$WORKSPACE" \
      inspect \
      --env mla \
      --recording "$recording_id" \
      > "$inspect_log" 2>&1
    inspect_rc=$?
  fi

  if [[ "$build_rc" != 0 || "$inspect_rc" != 0 ]]; then overall=1; fi
  printf '%s,%s,%s,%s,%s,%s\n' \
    "$RUN_STAMP" "$id" "$build_rc" "$inspect_rc" "$recording_id" "$recording_path" \
    >> "$RUN_DIR/summary.csv"
  printf '[batch] %s build=%s inspect=%s recording=%s\n' "$id" "$build_rc" "$inspect_rc" "${recording_id:-NONE}"
done

printf '[batch] complete; summary=%s\n' "$RUN_DIR/summary.csv"
exit "$overall"
