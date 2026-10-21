#!/usr/bin/env bash

# Compile the immutable MLA height-sweep authorities in bounded GPU batches.
# This stage is compile-only: it never starts a controller or edits an input.

set -uo pipefail

WORKSPACE="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
INDEX="${MLA_HEIGHT_SWEEP_AUTHORITY_INDEX:-$WORKSPACE/environments/mla/trajectories/icassp_f64_mj_height_sweep_feasible_v2_authority_index.csv}"
PARALLELISM="${MLA_HEIGHT_SWEEP_PARALLELISM:-4}"
RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="$WORKSPACE/runs/mla_height_sweep_compile_${RUN_STAMP}"

if [[ ! "$PARALLELISM" =~ ^[1-9][0-9]*$ ]]; then
  printf '[compile] invalid MLA_HEIGHT_SWEEP_PARALLELISM=%s\n' "$PARALLELISM" >&2
  exit 2
fi
if [[ ! -f "$INDEX" ]]; then
  printf '[compile] missing authority index: %s\n' "$INDEX" >&2
  exit 2
fi

mkdir -p "$RUN_DIR/logs" "$RUN_DIR/status"
set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/.venv/bin/activate"
source "$WORKSPACE/install/setup.bash"
set -u
export PYTHONNOUSERSITE=1
export PYTHONPATH="$WORKSPACE/.deps/curobo-a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9:$WORKSPACE/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"

mapfile -t ids < <(
  tail -n +2 "$INDEX" | cut -d, -f5 | sed -e 's/\.csv$//'
)
if [[ "${#ids[@]}" -ne 20 ]]; then
  printf '[compile] expected 20 authority IDs, got %d\n' "${#ids[@]}" >&2
  exit 2
fi

printf 'order,trajectory_id,status,compile_path,log_path\n' > "$RUN_DIR/summary.csv"
printf '[compile] run_dir=%s routes=%d parallelism=%s\n' "$RUN_DIR" "${#ids[@]}" "$PARALLELISM"

run_one() {
  local order="$1" id="$2"
  local log="$RUN_DIR/logs/${id}.compile.log"
  local output rc compile_path
  compile_path="$(find "$WORKSPACE/runs" -maxdepth 1 -mindepth 1 -type d \
    -name "${id}__compiled__*" -printf '%p\n' 2>/dev/null | sort -r | while read -r candidate; do
      if [[ -f "$candidate/source_tip_trajectory.csv" \
        && -f "$candidate/joint_trajectory.csv" \
        && -f "$candidate/compile_report.yaml" ]]; then
        printf '%s\n' "$candidate"
        break
      fi
    done)"
  if [[ -n "$compile_path" ]]; then
    printf 'existing validated compile: %s\n' "$compile_path" > "$log"
    printf '%s,%s,0,%s,%s\n' \
      "$order" "$id" "$compile_path" "$log" \
      > "$RUN_DIR/status/$(printf '%03d' "$order").csv"
    printf '[compile] [%02d/%02d] %s status=EXISTING\n' \
      "$((order + 1))" "${#ids[@]}" "$id"
    return 0
  fi
  output="$(timeout 600 ros2 run hx_waypoint_tools tipdraw \
    --workspace "$WORKSPACE" build-recording \
    --env mla --trajectory "$id" --compile-only 2>&1)"
  rc=$?
  printf '%s\n' "$output" > "$log"
  compile_path=""
  if [[ "$rc" -eq 0 ]]; then
    compile_path="$(printf '%s\n' "$output" | awk '/\/runs\// {value=$0} END {print value}')"
  fi
  printf '%s,%s,%s,%s,%s\n' \
    "$order" "$id" "$rc" "$compile_path" "$log" \
    > "$RUN_DIR/status/$(printf '%03d' "$order").csv"
  printf '[compile] [%02d/%02d] %s status=%s\n' \
    "$((order + 1))" "${#ids[@]}" "$id" "$rc"
  return 0
}

for ((base=0; base<${#ids[@]}; base+=PARALLELISM)); do
  pids=()
  for ((offset=0; offset<PARALLELISM && base+offset<${#ids[@]}; offset++)); do
    index=$((base + offset))
    run_one "$index" "${ids[$index]}" &
    pids+=("$!")
  done
  for pid in "${pids[@]}"; do
    wait "$pid"
  done
done

shopt -s nullglob
status_files=("$RUN_DIR"/status/*.csv)
if [[ "${#status_files[@]}" -ne "${#ids[@]}" ]]; then
  printf '[compile] expected %d status files, got %d\n' \
    "${#ids[@]}" "${#status_files[@]}" >&2
  exit 2
fi
for status in "${status_files[@]}"; do
  cat "$status" >> "$RUN_DIR/summary.csv"
done
failures="$(awk -F, 'NR>1 && $3 != 0 {count++} END {print count+0}' "$RUN_DIR/summary.csv")"
printf '[compile] complete failures=%s summary=%s\n' "$failures" "$RUN_DIR/summary.csv"
[[ "$failures" -eq 0 ]]
