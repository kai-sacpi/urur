#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="new_board_4cm_rod33365_up50_icassp_phase2"
ENVIRONMENT_PATH="$REPOSITORY_ROOT/environments/$ENVIRONMENT_ID"
INDEX_PATH="/data/ur-waypoint/icassp2027_phase2/build_evidence_reachable8_clean/exploratory/phase2_recording_build_index.csv"
SOURCE_RECORDINGS_ROOT="/data/ur-waypoint/icassp2027_phase2/workspace_reachable8/environments/$ENVIRONMENT_ID/recordings"
DEST_RECORDINGS_ROOT="$ENVIRONMENT_PATH/recordings"
NUCLEO_PORT=""
REPEATS=5
START_SEQUENCE=1
REUSE_EXISTING_SYNC=0
RUN_ROOT=""

fail() { echo "run_phase2_repeat5_real_nucleo.sh: $*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage: run_phase2_repeat5_real_nucleo.sh --port DEVICE [options]
  --port DEVICE             NUCLEO serial device
  --index FILE              clean Phase-2 recording build index
  --repeats N               plays per condition (default: 5)
  --start-sequence N        resume at schedule sequence (default: 1)
  --reuse-existing-sync     do not send SYNC on first play
  --run-root DIR            output directory
EOF
}
while (($#)); do
  case "$1" in
    --port) (($# >= 2)) || fail "--port requires a device"; NUCLEO_PORT="$2"; shift 2 ;;
    --index) (($# >= 2)) || fail "--index requires a file"; INDEX_PATH="$2"; shift 2 ;;
    --repeats) (($# >= 2)) || fail "--repeats requires an integer"; REPEATS="$2"; shift 2 ;;
    --start-sequence) (($# >= 2)) || fail "--start-sequence requires an integer"; START_SEQUENCE="$2"; shift 2 ;;
    --reuse-existing-sync) REUSE_EXISTING_SYNC=1; shift ;;
    --run-root) (($# >= 2)) || fail "--run-root requires a directory"; RUN_ROOT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown argument: $1" ;;
  esac
done
[[ -n "$NUCLEO_PORT" ]] || fail "--port is required"
[[ "$REPEATS" =~ ^[1-9][0-9]*$ ]] || fail "--repeats must be positive"
[[ "$START_SEQUENCE" =~ ^[1-9][0-9]*$ ]] || fail "--start-sequence must be positive"
[[ -r "$INDEX_PATH" ]] || fail "missing index: $INDEX_PATH"
[[ -d "$SOURCE_RECORDINGS_ROOT" ]] || fail "missing source recordings: $SOURCE_RECORDINGS_ROOT"
[[ -d "$ENVIRONMENT_PATH" ]] || fail "missing environment: $ENVIRONMENT_PATH"
[[ -c "$NUCLEO_PORT" && -r "$NUCLEO_PORT" && -w "$NUCLEO_PORT" ]] || fail "NUCLEO port is not readable/writable"
[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS Jazzy is unavailable"
[[ -r "$REPOSITORY_ROOT/install/setup.bash" ]] || fail "workspace is not built"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPOSITORY_ROOT/install/setup.bash"
set -u

action_server_count() { ros2 action info "$1" | awk '/^Action servers:/ {print $3}'; }
[[ "$(action_server_count /move_action)" == "1" ]] || fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] || fail "/hx_waypoint/play_recording must have exactly one server"
controller_state="$(timeout 15 ros2 control list_controllers | awk '$1=="joint_trajectory_controller" {print $3}')"
[[ "$controller_state" == "active" ]] || fail "joint_trajectory_controller is not active: $controller_state"
speed_value="$(timeout 5 ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once | awk '/^data:/ {print $2; exit}')"
python3 - "$speed_value" <<'PY'
import math, sys
try: value=float(sys.argv[1])
except (IndexError, ValueError): raise SystemExit("speed scaling unavailable")
if not math.isfinite(value) or abs(value-100.0)>1e-9:
    raise SystemExit(f"speed scaling must be 100.0, got {value}")
PY

read -r HOME1 HOME2 HOME3 HOME4 HOME5 HOME6 < <(python3 - "$ENVIRONMENT_PATH/robot/robot_model.yaml" <<'PY'
import sys, yaml
data=yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
values=data.get("home_joint_positions")
if values is None: values=data.get("robot",{}).get("home_joint_positions")
if not isinstance(values,list) or len(values)!=6: raise SystemExit("six home joints required")
print(" ".join(f"{float(v):.12f}" for v in values))
PY
)
[[ -n "$HOME6" ]] || fail "could not load home joints"

if [[ -z "$RUN_ROOT" ]]; then
  RUN_ROOT="$REPOSITORY_ROOT/runs/phase2_repeat$REPEATS"_real_nucleo__$(date -u +%Y%m%dT%H%M%SZ)
fi
mkdir -p "$RUN_ROOT"
exec 9>"$REPOSITORY_ROOT/runs/.phase2_real_batch.lock"
flock -n 9 || fail "another Phase-2 real batch is already running"
LOG_PATH="$RUN_ROOT/batch.log"
exec > >(tee -a "$LOG_PATH") 2>&1

echo "batch_root: $RUN_ROOT"
echo "environment: $ENVIRONMENT_ID"
echo "repeats_per_condition: $REPEATS"
echo "nucleo_port: $NUCLEO_PORT"

SELECTED_TSV="$RUN_ROOT/selected_recordings.tsv"
python3 - "$INDEX_PATH" "$SELECTED_TSV" <<'PY'
import csv, sys
from pathlib import Path
import yaml
rows=[]; seen=set()
with open(sys.argv[1], newline="", encoding="utf-8") as stream:
    for row in csv.DictReader(stream):
        if row.get("status") not in {"success","recovered"}: continue
        condition=row.get("condition_id",""); rid=row.get("recording_id","")
        path=Path(row.get("recording_path",""))
        if not condition or not rid or condition in seen: continue
        manifest=path/"manifest.yaml"
        if not path.is_dir() or not manifest.is_file():
            raise SystemExit(f"missing artifact: {path}")
        data=yaml.safe_load(manifest.read_text(encoding="utf-8"))
        if not isinstance(data,dict) or data.get("status")!="validated":
            raise SystemExit(f"artifact is not validated: {path}")
        rows.append((condition,rid,str(path))); seen.add(condition)
if len(rows)!=70: raise SystemExit(f"expected 70 validated conditions, found {len(rows)}")
with open(sys.argv[2],"w",encoding="utf-8") as out:
    for condition,rid,path in rows: out.write(f"{condition}\t{rid}\t{path}\n")
PY
recording_count="$(wc -l < "$SELECTED_TSV")"
[[ "$recording_count" == "70" ]] || fail "expected 70 clean recording rows, found $recording_count"

while IFS=$'\t' read -r condition recording_id source_path; do
  destination_path="$DEST_RECORDINGS_ROOT/$recording_id"
  [[ ! -L "$destination_path" ]] || fail "destination is a symlink: $destination_path"
  mkdir -p "$destination_path"
  rsync -a --exclude='*.staging*' "$source_path/" "$destination_path/"
  python3 - "$destination_path/manifest.yaml" "$recording_id" <<'PY'
import sys, yaml
data=yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
if not isinstance(data,dict) or data.get("status")!="validated": raise SystemExit("copied artifact is not validated")
if data.get("recording_id")!=sys.argv[2]: raise SystemExit("copied recording id mismatch")
PY
done < "$SELECTED_TSV"

SCHEDULE_PATH="$RUN_ROOT/execution_schedule.csv"
printf 'sequence,condition_id,recording_id,repeat_index,start_utc,end_utc,exit_code,robot_telemetry,nucleo_manifest\n' > "$SCHEDULE_PATH"
total=$((recording_count * REPEATS))
sequence=0
while IFS=$'\t' read -r condition recording_id source_path; do
  recording_root="$DEST_RECORDINGS_ROOT/$recording_id"
  start_values="$(awk -F, 'NR==2 {for (i=2;i<=7;++i) printf "%s%s",$i,(i<7?" ":""); exit}' "$recording_root/joint_trajectory.csv")"
  read -r START1 START2 START3 START4 START5 START6 <<<"$start_values"
  [[ -n "$START6" ]] || fail "cannot read sealed start joints: $recording_id"
  repeat_index=0
  while ((repeat_index < REPEATS)); do
    repeat_index=$((repeat_index + 1))
    sequence=$((sequence + 1))
    ((sequence < START_SEQUENCE)) && continue
    echo "===== [$sequence/$total] $condition repeat $repeat_index/$REPEATS ====="
    start_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
    play_log="$RUN_ROOT/$(printf '%04d' "$sequence")__$condition"_r"$repeat_index".log

    python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" --target "$HOME1" "$HOME2" "$HOME3" "$HOME4" "$HOME5" "$HOME6" --execute
    python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" --target "$START1" "$START2" "$START3" "$START4" "$START5" "$START6" --execute

    reuse_arg=""
    if ((sequence > START_SEQUENCE || REUSE_EXISTING_SYNC == 1)); then reuse_arg="--nucleo-reuse-sync"; fi
    set +e
    set -o pipefail
    ros2 run hx_waypoint_tools tipdraw --workspace "$REPOSITORY_ROOT" play --env "$ENVIRONMENT_ID" --recording "$recording_id" --externally-prepared-start --record-nucleo --nucleo-port "$NUCLEO_PORT" --nucleo-baudrate 115200 --nucleo-max-incomplete-groups 10 --nucleo-max-channel-sequence-drops 10 $reuse_arg 2>&1 | tee "$play_log"
    rc=$?
    set +o pipefail
    set -e
    end_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
    robot_telemetry="$(grep -E '^/[^ ]+/execution_telemetry\.csv$' "$play_log" | tail -1 || true)"
    nucleo_manifest="$(sed -n 's/^NUCLEO manifest: //p' "$play_log" | tail -1 || true)"
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$condition" "$recording_id" "$repeat_index" "$start_utc" "$end_utc" "$rc" "$robot_telemetry" "$nucleo_manifest" >> "$SCHEDULE_PATH"
    if ((rc != 0)); then
      echo "Batch stopped after sequence $sequence; play log: $play_log" >&2
      exit "$rc"
    fi
  done
done < "$SELECTED_TSV"
echo "All $total real robot + NUCLEO revisits completed."
echo "schedule: $SCHEDULE_PATH"
echo "log: $LOG_PATH"
