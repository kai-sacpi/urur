#!/usr/bin/env bash
set -Eeuo pipefail

# Replay the six validated continuous LM routes for the 190-mm-TCP mla_short
# environment and capture synchronized NUCLEO measurements.

WORKSPACE="${WORKSPACE:-/home/huaxin/Codes/ur-waypoint}"
ENVIRONMENT_ID=mla_short
INDEX_PATH="${INDEX_PATH:-${WORKSPACE}/environments/${ENVIRONMENT_ID}/trajectories/mla_short_lm_vortex_main_index.csv}"
NUCLEO_PORT="${HX_WAYPOINT_NUCLEO_PORT:-}"
RUN_ROOT=
START_SEQUENCE=1
REUSE_EXISTING_SYNC=0
MAX_INCOMPLETE=10
MAX_CHANNEL_DROPS=10

fail() { echo "replay_mla_short_lm_vortex_main.sh: $*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage: replay_mla_short_lm_vortex_main.sh --port DEVICE [options]
  --port DEVICE          NUCLEO serial device (required)
  --index FILE           override the sealed recording index
  --run-root DIR         durable output directory
  --start-sequence N     resume at route N (1--6)
  --reuse-existing-sync  NUCLEO is already streaming before route 1
  --max-incomplete N     allowed incomplete NUCLEO groups (default 10)
  --max-channel-drops N  allowed channel sequence drops (default 10)
EOF
}
while (($#)); do
  case "$1" in
    --port) (($# >= 2)) || fail "--port requires DEVICE"; NUCLEO_PORT="$2"; shift 2 ;;
    --index) (($# >= 2)) || fail "--index requires FILE"; INDEX_PATH="$2"; shift 2 ;;
    --run-root) (($# >= 2)) || fail "--run-root requires DIR"; RUN_ROOT="$2"; shift 2 ;;
    --start-sequence) (($# >= 2)) || fail "--start-sequence requires N"; START_SEQUENCE="$2"; shift 2 ;;
    --reuse-existing-sync) REUSE_EXISTING_SYNC=1; shift ;;
    --max-incomplete) (($# >= 2)) || fail "--max-incomplete requires N"; MAX_INCOMPLETE="$2"; shift 2 ;;
    --max-channel-drops) (($# >= 2)) || fail "--max-channel-drops requires N"; MAX_CHANNEL_DROPS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) fail "unknown argument: $1" ;;
  esac
done

[[ -n "$NUCLEO_PORT" ]] || fail "--port is required"
[[ "$START_SEQUENCE" =~ ^[1-9][0-9]*$ ]] || fail "--start-sequence must be positive"
((START_SEQUENCE >= 1 && START_SEQUENCE <= 6)) || fail "--start-sequence must be in [1,6]"
[[ "$MAX_INCOMPLETE" =~ ^[0-9]+$ && "$MAX_CHANNEL_DROPS" =~ ^[0-9]+$ ]] || fail "capture limits must be non-negative integers"
[[ -r /opt/ros/jazzy/setup.bash ]] || fail "ROS 2 Jazzy is unavailable"
[[ -r "$WORKSPACE/install/setup.bash" ]] || fail "workspace is not built: $WORKSPACE"
[[ -r "$INDEX_PATH" ]] || fail "recording index is missing: $INDEX_PATH"
[[ -c "$NUCLEO_PORT" && -r "$NUCLEO_PORT" && -w "$NUCLEO_PORT" ]] || fail "NUCLEO port is not readable/writable: $NUCLEO_PORT"

set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

SELECTED_TSV="$(mktemp)"
trap 'rm -f "$SELECTED_TSV"' EXIT
python3 - "$INDEX_PATH" "$WORKSPACE" "$SELECTED_TSV" <<'PY'
import csv, hashlib, re, sys
from pathlib import Path
index = Path(sys.argv[1]).resolve()
workspace = Path(sys.argv[2]).resolve()
out = Path(sys.argv[3])
environment = workspace / "environments" / "mla_short"
expected = [
    "mla_lm_vortex_center_z07_r10",
    "mla_lm_vortex_center_z09_r10",
    "mla_lm_vortex_center_z11_r10",
    "mla_lm_vortex_center_z13_r10",
    "mla_lm_vortex_center_z15_r10",
    "mla_lm_helix_center_z07_to_z15_r10",
]
rows = list(csv.DictReader(index.open(newline="", encoding="utf-8-sig")))
if len(rows) != len(expected):
    raise SystemExit(f"expected {len(expected)} rows, got {len(rows)}")
seen = set()
selected = []
for number, (row, expected_id) in enumerate(zip(rows, expected), 1):
    if row.get("order") != str(number) or row.get("trajectory_id") != expected_id:
        raise SystemExit(f"index order/content mismatch at row {number}")
    recording = row.get("recording_id", "")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", recording) or recording in seen:
        raise SystemExit(f"unsafe or duplicate recording id: {recording!r}")
    if row.get("status") != "validated":
        raise SystemExit(f"recording is not validated: {recording}")
    rec = (environment / row.get("recording_relative_path", "")).resolve()
    if rec.parent != (environment / "recordings").resolve() or not rec.is_dir():
        raise SystemExit(f"recording path escaped or is missing: {rec}")
    manifest = rec / "manifest.yaml"
    if not manifest.is_file():
        raise SystemExit(f"missing recording manifest: {manifest}")
    if hashlib.sha256(manifest.read_bytes()).hexdigest() != row.get("recording_manifest_sha256"):
        raise SystemExit(f"recording manifest hash mismatch: {recording}")
    manifest_text = manifest.read_text(encoding="utf-8")
    if "status: validated" not in manifest_text or "environment_id: mla_short" not in manifest_text:
        raise SystemExit(f"recording is not a validated mla_short artifact: {recording}")
    authority = (environment / "trajectories" / row.get("authority_file", "")).resolve()
    if authority.parent != (environment / "trajectories").resolve() or not authority.is_file():
        raise SystemExit(f"missing authority: {authority}")
    if hashlib.sha256(authority.read_bytes()).hexdigest() != row.get("authority_sha256"):
        raise SystemExit(f"authority hash mismatch: {authority.name}")
    selected.append((number, row["trajectory_id"], recording))
    seen.add(recording)
with out.open("w", encoding="utf-8") as stream:
    for number, trajectory, recording in selected:
        stream.write(f"{number}\t{trajectory}\t{recording}\n")
PY

action_server_count() { ros2 action info "$1" | awk '/^Action servers:/ {print $3}'; }
[[ "$(action_server_count /move_action)" == 1 ]] || fail "/move_action must have exactly one server"
[[ "$(action_server_count /hx_waypoint/play_recording)" == 1 ]] || fail "/hx_waypoint/play_recording must have exactly one server"
controller_state="$(timeout 15 ros2 control list_controllers 2>/dev/null | awk '$1 == "joint_trajectory_controller" {print $3}')"
[[ "$controller_state" == active ]] || fail "joint_trajectory_controller is not active: $controller_state"

if [[ -z "$RUN_ROOT" ]]; then
  RUN_ROOT="$WORKSPACE/runs/mla_short_lm_vortex_real_$(date -u +%Y%m%dT%H%M%SZ)"
fi
mkdir -p "$RUN_ROOT/logs"
SUMMARY="$RUN_ROOT/summary.csv"
printf 'sequence,trajectory_id,recording_id,play_exit_code,nucleo_telemetry,nucleo_manifest,play_log\n' > "$SUMMARY"
{
  printf 'schema_version: 1\n'
  printf 'environment_id: %s\n' "$ENVIRONMENT_ID"
  printf 'recording_index: %s\n' "$INDEX_PATH"
  printf 'nucleo_port: %s\n' "$NUCLEO_PORT"
  printf 'start_sequence: %s\n' "$START_SEQUENCE"
} > "$RUN_ROOT/run_manifest.yaml"

exec > >(tee -a "$RUN_ROOT/batch.log") 2>&1
echo "run_root: $RUN_ROOT"
echo "environment: $ENVIRONMENT_ID"
echo "recording_count: 6"
echo "nucleo_port: $NUCLEO_PORT"

while IFS=$'\t' read -r sequence trajectory recording; do
  ((sequence < START_SEQUENCE)) && continue
  echo "===== [$sequence/6] $trajectory ====="
  play_log="$RUN_ROOT/logs/$(printf '%02d' "$sequence")_$trajectory.log"
  reuse_args=()
  if ((sequence > 1 || REUSE_EXISTING_SYNC == 1)); then
    reuse_args+=(--nucleo-reuse-sync)
  fi
  set +e
  set -o pipefail
  ros2 run hx_waypoint_tools tipdraw --workspace "$WORKSPACE" play \
    --env "$ENVIRONMENT_ID" --recording "$recording" --auto-prepare \
    --record-nucleo --nucleo-port "$NUCLEO_PORT" --nucleo-baudrate 115200 \
    --nucleo-max-incomplete-groups "$MAX_INCOMPLETE" \
    --nucleo-max-channel-sequence-drops "$MAX_CHANNEL_DROPS" \
    "${reuse_args[@]}" 2>&1 | tee "$play_log"
  play_rc=${PIPESTATUS[0]}
  set +o pipefail
  set -e
  nucleo_telemetry="$(sed -n 's/^NUCLEO telemetry: //p' "$play_log" | tail -1)"
  nucleo_manifest="$(sed -n 's/^NUCLEO manifest: //p' "$play_log" | tail -1)"
  printf '%s,%s,%s,%s,%s,%s,%s\n' "$sequence" "$trajectory" "$recording" "$play_rc" \
    "$nucleo_telemetry" "$nucleo_manifest" "$play_log" >> "$SUMMARY"
  ((play_rc == 0)) || fail "route $sequence failed with status $play_rc; see $play_log"
done < "$SELECTED_TSV"

echo "All six mla_short LM routes completed. Summary: $SUMMARY"
