#!/usr/bin/env bash

WORKSPACE="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
ROS_DOMAIN_ID="${ICASSPSIM_ROS_DOMAIN_ID:-221}"
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS2CLI_NO_DAEMON=1
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/.venv/bin/activate"
export PYTHONNOUSERSITE=1
source "$WORKSPACE/install/setup.bash"
set -u
export PYTHONPATH="$WORKSPACE/.deps/curobo-a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9:$WORKSPACE/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

status="$WORKSPACE/runs/icassp_f64_feasible_parallel_status.log"
: > "$status"

run_one() {
  local id="$1" rc
  timeout 360 ros2 run hx_waypoint_tools tipdraw \
    --workspace "$WORKSPACE" build-recording \
    --env mla --trajectory "$id" --compile-only \
    >"$WORKSPACE/runs/${id}.compile.log" 2>&1
  rc=$?
  printf '%s %s\n' "$id" "$rc" >> "$status"
}

groups=(
  "icassp_f64_mj_feasible_line_x_vertical_z09 icassp_f64_mj_feasible_line_x_tilt90_z09 icassp_f64_mj_feasible_line_y_vertical_z09 icassp_f64_mj_feasible_line_y_tilt90_z09"
  "icassp_f64_mj_feasible_line_x_vertical_z13 icassp_f64_mj_feasible_line_x_tilt90_z13 icassp_f64_mj_feasible_line_y_vertical_z13 icassp_f64_mj_feasible_line_y_tilt90_z13"
  "icassp_f64_mj_feasible_line_x_vertical_z15 icassp_f64_mj_feasible_line_x_tilt90_z15 icassp_f64_mj_feasible_line_y_vertical_z15 icassp_f64_mj_feasible_line_y_tilt90_z15"
  "icassp_f64_mj_feasible_eight_tilt90_z13"
)

for group in "${groups[@]}"; do
  pids=()
  for id in $group; do
    run_one "$id" & pids+=("$!")
  done
  for pid in "${pids[@]}"; do wait "$pid"; done
done

cat "$status"
