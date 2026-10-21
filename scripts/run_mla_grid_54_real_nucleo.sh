#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
REPOSITORY_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ENVIRONMENT_ID="mla"
ENVIRONMENT_PATH="$REPOSITORY_ROOT/environments/$ENVIRONMENT_ID"
INDEX_PATH="/data/ur-waypoint/mla_grid_54/20260904T150316Z/build/selected_recordings.csv"
NUCLEO_PORT=""
RUN_ROOT=""
START_SEQUENCE=1
REUSE_EXISTING_SYNC=0
MAX_INCOMPLETE=10
MAX_CHANNEL_DROPS=10
MAX_DUPLICATE_PACKETS=10
MAX_REORDERED_PACKETS=10

fail() { echo "run_mla_grid_54_real_nucleo.sh: $*" >&2; exit 1; }

usage() {
  cat <<'EOF'
Usage: run_mla_grid_54_real_nucleo.sh --port DEVICE [options]
  --port DEVICE             NUCLEO serial device
  --index FILE              selected_recordings.csv
  --run-root DIR            durable result directory under /data
  --start-sequence N        resume at this 1-based point
  --reuse-existing-sync     NUCLEO is already streaming; never send SYNC
EOF
}

while (($#)); do
  case "$1" in
    --port) (($# >= 2)) || fail "--port requires a device"; NUCLEO_PORT="$2"; shift 2 ;;
    --index) (($# >= 2)) || fail "--index requires a file"; INDEX_PATH="$2"; shift 2 ;;
    --run-root) (($# >= 2)) || fail "--run-root requires a directory"; RUN_ROOT="$2"; shift 2 ;;
    --start-sequence) (($# >= 2)) || fail "--start-sequence requires an integer"; START_SEQUENCE="$2"; shift 2 ;;
    --reuse-existing-sync) REUSE_EXISTING_SYNC=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown argument: $1" ;;
  esac
done

[[ -n "$NUCLEO_PORT" ]] || fail "--port is required"
[[ "$START_SEQUENCE" =~ ^[1-9][0-9]*$ ]] || fail "--start-sequence must be positive"
[[ -r "$INDEX_PATH" ]] || fail "missing selected recording index: $INDEX_PATH"
[[ -d "$ENVIRONMENT_PATH" ]] || fail "missing environment: $ENVIRONMENT_PATH"
[[ -c "$NUCLEO_PORT" && -r "$NUCLEO_PORT" && -w "$NUCLEO_PORT" ]] || \
  fail "NUCLEO port is not readable/writable: $NUCLEO_PORT"
[[ -r /opt/ros/jazzy/setup.bash && -r "$REPOSITORY_ROOT/install/setup.bash" ]] || \
  fail "ROS Jazzy workspace is not built"

set +u
source /opt/ros/jazzy/setup.bash
source "$REPOSITORY_ROOT/install/setup.bash"
set -u

action_server_count() { ros2 action info "$1" | awk '/^Action servers:/ {print $3}'; }
[[ "$(action_server_count /move_action)" == "1" ]] || fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == "1" ]] || \
  fail "/hx_waypoint/play_recording must have exactly one server"
controller_state="$(timeout 15 ros2 control list_controllers | awk '$1=="joint_trajectory_controller" {print $3}')"
[[ "$controller_state" == "active" ]] || \
  fail "joint_trajectory_controller is not active: $controller_state"
speed_value="$(timeout 5 ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling --once | awk '/^data:/ {print $2; exit}')"
python3 - "$speed_value" <<'PY'
import math, sys
try:
    value = float(sys.argv[1])
except (IndexError, ValueError):
    raise SystemExit("speed scaling unavailable")
if not math.isfinite(value) or abs(value - 100.0) > 1e-9:
    raise SystemExit(f"speed scaling must be 100.0, got {value}")
PY

read -r HOME1 HOME2 HOME3 HOME4 HOME5 HOME6 < <(python3 - "$ENVIRONMENT_PATH/robot/robot_model.yaml" <<'PY'
import sys, yaml
data = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
values = data.get("home_joint_positions")
if values is None:
    values = data.get("robot", {}).get("home_joint_positions")
if not isinstance(values, list) or len(values) != 6:
    raise SystemExit("six home joints required")
print(" ".join(f"{float(value):.12f}" for value in values))
PY
)
[[ -n "$HOME6" ]] || fail "could not load home joints"

if [[ -z "$RUN_ROOT" ]]; then
  RUN_ROOT="/data/ur-waypoint/mla_grid_54_real/$(date -u +%Y%m%dT%H%M%SZ)"
fi
mkdir -p "$RUN_ROOT/artifacts"
exec 9>"/data/ur-waypoint/mla_grid_54_real/.batch.lock"
flock -n 9 || fail "another MLA 54-point real batch is already running"
LOG_PATH="$RUN_ROOT/batch.log"
exec > >(tee -a "$LOG_PATH") 2>&1

SELECTED_TSV="$RUN_ROOT/selected_recordings.tsv"
python3 - "$INDEX_PATH" "$REPOSITORY_ROOT" "$SELECTED_TSV" <<'PY'
import csv, hashlib, sys
from pathlib import Path
import yaml

index = Path(sys.argv[1])
workspace = Path(sys.argv[2]).resolve()
output = Path(sys.argv[3])
rows = []
seen_conditions = set()
seen_recordings = set()
with index.open(newline="", encoding="utf-8-sig") as stream:
    for row in csv.DictReader(stream):
        status = row.get("build_status", "")
        if status not in {"SUCCESS", "RECOVERED"}:
            raise SystemExit(f"unselected build state in formal index: {status!r}")
        condition = row.get("condition_id", "")
        recording_id = row.get("recording_id", "")
        relative = row.get("recording_relative_path", "")
        path = (workspace / relative).resolve()
        if not condition or not recording_id or condition in seen_conditions or recording_id in seen_recordings:
            raise SystemExit("selected index contains a missing or duplicate identity")
        if path.parent.parent != (workspace / "environments" / "mla").resolve():
            raise SystemExit(f"recording escaped the MLA environment: {path}")
        manifest_path = path / "manifest.yaml"
        if not path.is_dir() or not manifest_path.is_file():
            raise SystemExit(f"missing recording: {path}")
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        digest = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
        if (
            not isinstance(manifest, dict)
            or manifest.get("status") != "validated"
            or manifest.get("environment_id") != "mla"
            or manifest.get("recording_id") != recording_id
            or digest != row.get("recording_manifest_sha256")
        ):
            raise SystemExit(f"recording authority mismatch: {recording_id}")
        rows.append((int(row["order"]), condition, recording_id, str(path)))
        seen_conditions.add(condition)
        seen_recordings.add(recording_id)
rows.sort()
if len(rows) != 54 or [item[0] for item in rows] != list(range(1, 55)):
    raise SystemExit(f"expected exactly ordered 54 selected recordings, got {len(rows)}")
with output.open("w", encoding="utf-8") as stream:
    for order, condition, recording_id, path in rows:
        stream.write(f"{order}\t{condition}\t{recording_id}\t{path}\n")
PY

SCHEDULE_PATH="$RUN_ROOT/execution_schedule.csv"
QUALITY_PATH="$RUN_ROOT/sensor_quality.csv"
printf 'sequence,condition_id,recording_id,start_utc,end_utc,play_exit_code,decision,robot_telemetry,nucleo_manifest,durable_artifact\n' > "$SCHEDULE_PATH"
printf 'sequence,condition_id,decision,integrity_deltas,nucleo_manifest\n' > "$QUALITY_PATH"

bounded_sensor_loss_ok() {
  local log_path="$1"
  local manifest_path="$2"
  [[ -n "$manifest_path" && -f "$manifest_path" ]] || return 1
  grep -q "robot playback succeeded, but NUCLEO capture is incomplete" "$log_path" || return 1
  python3 - "$manifest_path" "$MAX_INCOMPLETE" "$MAX_CHANNEL_DROPS" "$MAX_DUPLICATE_PACKETS" "$MAX_REORDERED_PACKETS" <<'PY'
import sys, yaml
data = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
limits = {
    "parser_incomplete_groups": int(sys.argv[2]),
    "parser_channel_sequence_drops": int(sys.argv[3]),
    "parser_duplicate_packets": int(sys.argv[4]),
    "parser_reordered_packets": int(sys.argv[5]),
}
capture = data.get("capture") if isinstance(data, dict) else None
mapping = data.get("mapping") if isinstance(data, dict) else None
if data.get("status") != "incomplete" or not isinstance(capture, dict) or capture.get("capture_error") is not None:
    raise SystemExit(1)
if not isinstance(mapping, dict) or not isinstance(mapping.get("parser_integrity_deltas"), dict):
    raise SystemExit(1)
deltas = mapping["parser_integrity_deltas"]
for key, limit in limits.items():
    value = int(deltas.get(key, 0))
    if value < 0 or value > limit:
        raise SystemExit(1)
allowed = set(limits)
if any(int(value) != 0 for key, value in deltas.items() if key not in allowed):
    raise SystemExit(1)
maximum_abs = int(mapping.get("offset_ns", {}).get("maximum_absolute", 0))
if maximum_abs > 50_000_000:
    raise SystemExit(1)
print(";".join(f"{key}={int(deltas.get(key, 0))}" for key in sorted(allowed)))
PY
}

echo "batch_root: $RUN_ROOT"
echo "environment: $ENVIRONMENT_ID"
echo "recording_count: 54"
echo "nucleo_port: $NUCLEO_PORT"
echo "first_sequence: $START_SEQUENCE"

while IFS=$'\t' read -r sequence condition recording_id recording_root; do
  ((sequence < START_SEQUENCE)) && continue
  echo "===== [$sequence/54] $condition ====="
  start_values="$(awk -F, 'NR==2 {for (i=2;i<=7;++i) printf "%s%s",$i,(i<7?" ":""); exit}' "$recording_root/joint_trajectory.csv")"
  read -r START1 START2 START3 START4 START5 START6 <<<"$start_values"
  [[ -n "$START6" ]] || fail "cannot read sealed start joints: $recording_id"
  start_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
  play_log="$RUN_ROOT/$(printf '%03d' "$sequence")__${condition}.log"

  python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" \
    --target "$HOME1" "$HOME2" "$HOME3" "$HOME4" "$HOME5" "$HOME6" --execute
  python3 "$SCRIPT_DIR/moveit_joint_prepare.py" --workspace "$REPOSITORY_ROOT" \
    --target "$START1" "$START2" "$START3" "$START4" "$START5" "$START6" --execute

  reuse_args=()
  if ((sequence > START_SEQUENCE || REUSE_EXISTING_SYNC == 1)); then
    reuse_args+=(--nucleo-reuse-sync)
  fi
  set +e
  set -o pipefail
  ros2 run hx_waypoint_tools tipdraw --workspace "$REPOSITORY_ROOT" play \
    --env "$ENVIRONMENT_ID" --recording "$recording_id" \
    --externally-prepared-start --record-nucleo \
    --nucleo-port "$NUCLEO_PORT" --nucleo-baudrate 115200 \
    --nucleo-max-incomplete-groups "$MAX_INCOMPLETE" \
    --nucleo-max-channel-sequence-drops "$MAX_CHANNEL_DROPS" \
    "${reuse_args[@]}" 2>&1 | tee "$play_log"
  play_rc=$?
  set +o pipefail
  set -e

  end_utc="$(date -u +%Y-%m-%dT%H:%M:%S.%3NZ)"
  robot_telemetry="$(grep -E '^/[^ ]+/execution_telemetry\.csv$' "$play_log" | tail -1 || true)"
  nucleo_manifest="$(sed -n 's/^NUCLEO manifest: //p' "$play_log" | tail -1 || true)"
  decision="COMPLETE"
  deltas=""
  if ((play_rc != 0)); then
    deltas="$(bounded_sensor_loss_ok "$play_log" "$nucleo_manifest" 2>/dev/null || true)"
    if [[ -z "$deltas" ]]; then
      printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$condition" "$recording_id" "$start_utc" "$end_utc" "$play_rc" "REJECTED" "$robot_telemetry" "$nucleo_manifest" "" >> "$SCHEDULE_PATH"
      fail "unaccepted playback/capture failure at sequence $sequence; see $play_log"
    fi
    decision="ACCEPTED_BOUNDED_SENSOR_LOSS"
    echo "bounded sensor loss accepted: $deltas"
  fi

  durable_artifact=""
  if [[ -n "$robot_telemetry" && -f "$robot_telemetry" ]]; then
    source_run="$(dirname -- "$(dirname -- "$robot_telemetry")")"
    durable_artifact="$RUN_ROOT/artifacts/$(basename -- "$source_run")"
    mkdir -p "$durable_artifact"
    rsync -a "$source_run/" "$durable_artifact/"
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$condition" "$recording_id" "$start_utc" "$end_utc" "$play_rc" "$decision" "$robot_telemetry" "$nucleo_manifest" "$durable_artifact" >> "$SCHEDULE_PATH"
  printf '%s,%s,%s,%s,%s\n' "$sequence" "$condition" "$decision" "$deltas" "$nucleo_manifest" >> "$QUALITY_PATH"
done < "$SELECTED_TSV"

echo "All 54 real robot + NUCLEO point captures completed."
echo "schedule: $SCHEDULE_PATH"
echo "quality: $QUALITY_PATH"
echo "log: $LOG_PATH"
