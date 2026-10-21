#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="new_board_4cm_rod33365_up50_icassp_phase2"
ENVIRONMENT_PATH="$REPOSITORY_ROOT/environments/$ENVIRONMENT_ID"
RUN_ROOT=""
NUCLEO_PORT=""
START_SEQUENCE=18
REPEATS=5
MAX_INCOMPLETE=10
MAX_CHANNEL_DROPS=10
MAX_DUPLICATE_PACKETS=10
MAX_REORDERED_PACKETS=10

fail() { echo "resume_phase2_repeat5_real_nucleo.sh: $*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage: resume_phase2_repeat5_real_nucleo.sh --run-root DIR --port DEVICE [options]
  --run-root DIR             existing phase2_repeat5 batch directory
  --port DEVICE              NUCLEO serial device
  --start-sequence N         first 1-based sequence to run (default: 18)
  --repeats N                plays per condition (default: 5)
EOF
}
while (($#)); do
  case "$1" in
    --run-root) (($# >= 2)) || fail "--run-root requires a directory"; RUN_ROOT="$2"; shift 2 ;;
    --port) (($# >= 2)) || fail "--port requires a device"; NUCLEO_PORT="$2"; shift 2 ;;
    --start-sequence) (($# >= 2)) || fail "--start-sequence requires an integer"; START_SEQUENCE="$2"; shift 2 ;;
    --repeats) (($# >= 2)) || fail "--repeats requires an integer"; REPEATS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown argument: $1" ;;
  esac
done
[[ -n "$NUCLEO_PORT" && -d "$RUN_ROOT" ]] || fail "--run-root and --port are required"
[[ "$START_SEQUENCE" =~ ^[1-9][0-9]*$ ]] || fail "--start-sequence must be positive"
[[ "$REPEATS" =~ ^[1-9][0-9]*$ ]] || fail "--repeats must be positive"
[[ -c "$NUCLEO_PORT" && -r "$NUCLEO_PORT" && -w "$NUCLEO_PORT" ]] || fail "NUCLEO port is not readable/writable"
[[ -r "$RUN_ROOT/selected_recordings.tsv" ]] || fail "selected_recordings.tsv is missing"
[[ -d "$ENVIRONMENT_PATH" ]] || fail "environment is missing: $ENVIRONMENT_PATH"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPOSITORY_ROOT/install/setup.bash"
set -u

action_server_count() { ros2 action info "$1" | awk '/^Action servers:/ {print $3}'; }
[[ "$(action_server_count /move_action)" == "1" ]] || fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] || fail "/hx_waypoint/play_recording must have exactly one server"
controller_state="$(timeout 15 ros2 control list_controllers | awk '$1=="joint_trajectory_controller" {print $3}')"
[[ "$controller_state" == "active" ]] || fail "joint_trajectory_controller is not active: $controller_state"

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

exec 9>"$REPOSITORY_ROOT/runs/.phase2_real_batch.lock"
flock -n 9 || fail "another Phase-2 real batch is already running"
LOG_PATH="$RUN_ROOT/resume_from_$START_SEQUENCE.log"
exec > >(tee -a "$LOG_PATH") 2>&1

QUALITY_PATH="$RUN_ROOT/bounded_sensor_loss_acceptance.csv"
if [[ ! -e "$QUALITY_PATH" ]]; then
  printf 'sequence,condition_id,repeat_index,decision,manifest,integrity_deltas,message\n' >"$QUALITY_PATH"
fi
SCHEDULE_PATH="$RUN_ROOT/execution_schedule.csv"
[[ -e "$SCHEDULE_PATH" ]] || fail "execution_schedule.csv is missing"

bounded_sensor_loss_ok() {
  local log_path="$1"
  local manifest_path="$2"
  [[ -n "$manifest_path" && -f "$manifest_path" ]] || return 1
  grep -q "robot playback succeeded, but NUCLEO capture is incomplete" "$log_path" || return 1
  python3 - "$manifest_path" "$MAX_INCOMPLETE" "$MAX_CHANNEL_DROPS" "$MAX_DUPLICATE_PACKETS" "$MAX_REORDERED_PACKETS" <<'PY'
import sys, yaml
path=sys.argv[1]
max_incomplete=int(sys.argv[2]); max_drop=int(sys.argv[3]); max_dup=int(sys.argv[4])
max_reordered=int(sys.argv[5])
try:
    data=yaml.safe_load(open(path, encoding="utf-8"))
except Exception:
    raise SystemExit(1)
if not isinstance(data,dict) or data.get("status")!="incomplete":
    raise SystemExit(1)
capture=data.get("capture"); mapping=data.get("mapping")
if not isinstance(capture,dict) or capture.get("capture_error") is not None:
    raise SystemExit(1)
if not isinstance(mapping,dict):
    raise SystemExit(1)
d=mapping.get("parser_integrity_deltas")
if not isinstance(d,dict):
    raise SystemExit(1)
def count(name):
    try: v=int(d.get(name,0))
    except (TypeError,ValueError): raise SystemExit(1)
    if v<0: raise SystemExit(1)
    return v
allowed={"parser_incomplete_groups","parser_channel_sequence_drops","parser_duplicate_packets","parser_reordered_packets"}
if any(int(v)!=0 for k,v in d.items() if k not in allowed):
    raise SystemExit(1)
if count("parser_incomplete_groups")>max_incomplete or count("parser_channel_sequence_drops")>max_drop or count("parser_duplicate_packets")>max_dup or count("parser_reordered_packets")>max_reordered:
    raise SystemExit(1)
if not any(count(k)>0 for k in allowed):
    raise SystemExit(1)
offsets=mapping.get("offset_ns",{})
try: maximum_abs=int(offsets.get("maximum_absolute",0))
except (TypeError,ValueError): raise SystemExit(1)
if maximum_abs>50_000_000:
    raise SystemExit(1)
print(",".join(f"{k}={count(k)}" for k in sorted(allowed)))
PY
}

total=$((70 * REPEATS))
sequence=0
while IFS=$'\t' read -r condition recording_id source_path; do
  recording_root="$ENVIRONMENT_PATH/recordings/$recording_id"
  [[ -d "$recording_root" ]] || fail "recording is missing: $recording_id"
  start_values="$(awk -F, 'NR==2 {for (i=2;i<=7;++i) printf "%s%s",$i,(i<7?" ":""); exit}' "$recording_root/joint_trajectory.csv")"
  read -r START1 START2 START3 START4 START5 START6 <<<"$start_values"
  [[ -n "$START6" ]] || fail "cannot read sealed start joints: $recording_id"
  repeat_index=0
  while ((repeat_index < REPEATS)); do
    repeat_index=$((repeat_index+1))
    sequence=$((sequence+1))
    ((sequence < START_SEQUENCE)) && continue
    echo "===== [$sequence/$total] $condition repeat $repeat_index/$REPEATS ====="
    start_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
    play_log="$RUN_ROOT/$(printf '%04d' "$sequence")__${condition}__r${repeat_index}.log"

    python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" --target "$HOME1" "$HOME2" "$HOME3" "$HOME4" "$HOME5" "$HOME6" --execute
    python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" --target "$START1" "$START2" "$START3" "$START4" "$START5" "$START6" --execute

    set +e
    set -o pipefail
    ros2 run hx_waypoint_tools tipdraw --workspace "$REPOSITORY_ROOT" play --env "$ENVIRONMENT_ID" --recording "$recording_id" --externally-prepared-start --record-nucleo --nucleo-reuse-sync --nucleo-port "$NUCLEO_PORT" --nucleo-baudrate 115200 --nucleo-max-incomplete-groups "$MAX_INCOMPLETE" --nucleo-max-channel-sequence-drops "$MAX_CHANNEL_DROPS" 2>&1 | tee "$play_log"
    rc=$?
    set +o pipefail
    set -e

    end_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
    robot_telemetry="$(grep -E 'execution_telemetry\.csv$' "$play_log" | tail -1 || true)"
    nucleo_manifest="$(sed -n 's/^NUCLEO manifest: //p' "$play_log" | tail -1 || true)"
    if ((rc != 0)); then
      deltas="$(bounded_sensor_loss_ok "$play_log" "$nucleo_manifest" 2>/dev/null || true)"
      if [[ -z "$deltas" ]]; then
        printf '%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$condition" "$repeat_index" "REJECTED" "$nucleo_manifest" "" "rc=$rc" >>"$QUALITY_PATH"
        echo "Unaccepted failure at sequence $sequence; log: $play_log" >&2
        exit "$rc"
      fi
      decision="ACCEPTED_INCOMPLETE"
      echo "bounded sensor loss accepted for sequence $sequence: $deltas"
    else
      decision="COMPLETE"
      deltas=""
    fi
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$condition" "$recording_id" "$repeat_index" "$start_utc" "$end_utc" "$rc" "$robot_telemetry" "$nucleo_manifest" >>"$SCHEDULE_PATH"
    printf '%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$condition" "$repeat_index" "$decision" "$nucleo_manifest" "$deltas" "robot_success" >>"$QUALITY_PATH"
  done
done <"$RUN_ROOT/selected_recordings.tsv"

echo "Resumed batch completed through sequence $total."
echo "schedule: $SCHEDULE_PATH"
echo "quality: $QUALITY_PATH"
echo "log: $LOG_PATH"
