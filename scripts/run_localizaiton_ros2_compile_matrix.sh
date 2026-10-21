#!/usr/bin/env bash
set -o pipefail

ROOT="${UR_WAYPOINT_ROOT:-/home/huaxin/Codes/ur-waypoint}"
PYTHON_BIN="${ROOT}/.venv/bin/python"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="$(command -v python3 || true)"
fi
if [[ -z "${PYTHON_BIN}" || ! -x "${PYTHON_BIN}" ]]; then
  echo "compile matrix requires an executable Python 3 interpreter" >&2
  exit 2
fi

# The importer owns the one audited trajectory -> profile mapping.  Reading its
# constants is side-effect free: import_directory() only writes when explicitly called.
CASES=()
CASE_OUTPUT="$(
  "${PYTHON_BIN}" - "${ROOT}" <<'PY'
from __future__ import annotations

import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
from scripts.import_localizaiton_ros2_data import (  # noqa: E402
    PLANE_POSES,
    TEST_PROFILES,
    TRAJECTORY_ASSIGNMENTS,
)

for trajectory_id, environment_id in TRAJECTORY_ASSIGNMENTS.items():
    profile = TEST_PROFILES[environment_id]
    pose = PLANE_POSES[profile.plane_pose_id]
    print(
        "|".join(
            (
                profile.profile_id,
                environment_id,
                trajectory_id,
                f"{profile.rod_length_m:.3f}",
                profile.plane_pose_id,
                pose.normal_direction,
            )
        )
    )
PY
)"
case_status="$?"
if [[ "${case_status}" -ne 0 || -z "${CASE_OUTPUT}" ]]; then
  echo "failed to initialize the audited compile matrix" >&2
  exit 2
fi
while IFS= read -r item; do
  if [[ -z "${item}" ]]; then
    echo "compile matrix initialization emitted an empty case" >&2
    exit 2
  fi
  CASES+=("${item}")
done <<< "${CASE_OUTPUT}"

EVIDENCE_HEADER='profile_id\tenvironment_id\ttrajectory_id\trod_length_m\tplane_pose_id\traw_base_to_plane1_pose_xyz_rotvec\tbase_link_conversion\tplane_translation_m\tplane_quaternion_xyzw\tplane_z_axis_base_link\tplane_normal_direction\timport_manifest_sha256\tsource_sha256'

validate_case() {
  local item="$1"
  local expected_profile_id
  local environment_id
  local trajectory_id
  local expected_rod_length_m
  local expected_plane_pose_id
  local expected_plane_normal_direction
  IFS='|' read -r expected_profile_id environment_id trajectory_id \
    expected_rod_length_m expected_plane_pose_id expected_plane_normal_direction \
    <<< "${item}"

  "${PYTHON_BIN}" - "${ROOT}" "${expected_profile_id}" "${environment_id}" \
    "${trajectory_id}" "${expected_rod_length_m}" "${expected_plane_pose_id}" \
    "${expected_plane_normal_direction}" <<'PY'
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import yaml

root = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
from scripts.import_localizaiton_ros2_data import (  # noqa: E402
    EXPECTED_SOURCE_SHA256,
    PLANE_POSES,
    SOURCE_BRANCH,
    SOURCE_COMMIT,
    SOURCE_PATH,
    SOURCE_REPOSITORY,
    TEST_PROFILES,
    TRAJECTORY_ASSIGNMENTS,
)


def load_yaml(path: Path) -> tuple[dict[str, object], bytes]:
    if path.is_symlink() or not path.is_file():
        raise SystemExit(f"missing regular YAML file: {path}")
    try:
        payload = path.read_bytes()
        document = yaml.safe_load(payload)
    except (OSError, yaml.YAMLError) as error:
        raise SystemExit(f"cannot load {path}: {error}") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise SystemExit(f"invalid schema_version in {path}")
    return document, payload


def finite_vector(value: object, size: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise SystemExit(f"{label} must contain {size} values")
    parsed = [float(item) for item in value]
    if any(not math.isfinite(item) for item in parsed):
        raise SystemExit(f"{label} contains a non-finite value")
    return parsed


expected_profile_id = sys.argv[2]
environment_id = sys.argv[3]
trajectory_id = sys.argv[4]
expected_rod_length_m = float(sys.argv[5])
expected_plane_pose_id = sys.argv[6]
expected_plane_normal_direction = sys.argv[7]
environment_root = (root / "environments" / environment_id).resolve()
try:
    environment_root.relative_to(root / "environments")
except ValueError as error:
    raise SystemExit("environment path escapes the workspace") from error

expected_profile = TEST_PROFILES.get(environment_id)
if expected_profile is None:
    raise SystemExit(f"{environment_id}: profile is missing from the importer")
if expected_profile.profile_id != expected_profile_id:
    raise SystemExit(f"{environment_id}: matrix profile differs from the importer")
if expected_profile.plane_pose_id != expected_plane_pose_id:
    raise SystemExit(f"{environment_id}: matrix pose differs from the importer")
if not math.isclose(
    expected_profile.rod_length_m,
    expected_rod_length_m,
    rel_tol=0.0,
    abs_tol=1.0e-12,
):
    raise SystemExit(f"{environment_id}: matrix rod length differs from the importer")

manifest_path = environment_root / "trajectories" / "localizaiton_ros2_import.yaml"
manifest, manifest_payload = load_yaml(manifest_path)
expected_source = {
    "repository": SOURCE_REPOSITORY,
    "branch": SOURCE_BRANCH,
    "commit": SOURCE_COMMIT,
    "path": SOURCE_PATH,
}
if manifest.get("source") != expected_source:
    raise SystemExit(f"{environment_id}: import manifest source provenance mismatch")
if manifest.get("environment_id") != environment_id:
    raise SystemExit(f"{environment_id}: import manifest environment_id mismatch")
manifest_profile = manifest.get("test_profile")
if not isinstance(manifest_profile, dict):
    raise SystemExit(f"{environment_id}: import manifest profile is missing")
if manifest_profile.get("profile_id") != expected_profile_id:
    raise SystemExit(f"{environment_id}: import manifest profile_id mismatch")
if manifest_profile.get("plane_pose_id") != expected_plane_pose_id:
    raise SystemExit(f"{environment_id}: import manifest plane_pose_id mismatch")
manifest_rod_length = manifest_profile.get("rod_length_m")
if isinstance(manifest_rod_length, bool):
    raise SystemExit(f"{environment_id}: import manifest rod length is invalid")
try:
    parsed_manifest_rod_length = float(manifest_rod_length)
except (TypeError, ValueError) as error:
    raise SystemExit(
        f"{environment_id}: import manifest rod length is invalid"
    ) from error
if not math.isclose(
    parsed_manifest_rod_length,
    expected_rod_length_m,
    rel_tol=0.0,
    abs_tol=1.0e-12,
):
    raise SystemExit(f"{environment_id}: import manifest rod length mismatch")

manifest_files = manifest.get("files")
if not isinstance(manifest_files, list):
    raise SystemExit(f"{environment_id}: import manifest files must be a list")
expected_trajectory_ids = {
    candidate_id
    for candidate_id, candidate_environment in TRAJECTORY_ASSIGNMENTS.items()
    if candidate_environment == environment_id
}
expected_output_files = {
    f"{candidate_id}.csv" for candidate_id in expected_trajectory_ids
}
manifest_entries: dict[str, dict[str, object]] = {}
manifest_output_digests: dict[str, str] = {}
for entry in manifest_files:
    if not isinstance(entry, dict):
        raise SystemExit(f"{environment_id}: import manifest file entry is invalid")
    output_file = entry.get("output_file")
    if not isinstance(output_file, str) or output_file in manifest_entries:
        raise SystemExit(
            f"{environment_id}: import manifest output_file is invalid or duplicated"
        )
    trajectory_name = Path(output_file).stem
    expected_output_file = f"{trajectory_name}.csv"
    expected_source_file = f"{trajectory_name}.xlsx"
    if output_file != expected_output_file:
        raise SystemExit(f"{environment_id}: import manifest output_file is not canonical")
    if entry.get("source_file") != expected_source_file:
        raise SystemExit(
            f"{environment_id}: import manifest source/output filename mismatch"
        )
    expected_source_sha256 = EXPECTED_SOURCE_SHA256.get(expected_source_file)
    if (
        expected_source_sha256 is None
        or entry.get("source_sha256") != expected_source_sha256
    ):
        raise SystemExit(f"{environment_id}: import manifest source SHA-256 mismatch")
    output_path = environment_root / "trajectories" / output_file
    if output_path.is_symlink() or not output_path.is_file():
        raise SystemExit(f"missing regular authoritative CSV: {output_path}")
    output_digest = hashlib.sha256(output_path.read_bytes()).hexdigest()
    if entry.get("output_sha256") != output_digest:
        raise SystemExit(
            f"{environment_id}: authoritative CSV differs from import manifest: "
            f"{output_file}"
        )
    manifest_entries[output_file] = entry
    manifest_output_digests[output_file] = output_digest
if set(manifest_entries) != expected_output_files:
    raise SystemExit(
        f"{environment_id}: import manifest output_file set differs from the matrix"
    )

environment, _ = load_yaml(environment_root / "environment.yaml")
setup, _ = load_yaml(environment_root / "setup.yaml")
profile = environment.get("legacy_test_profile")
if not isinstance(profile, dict):
    raise SystemExit(f"{environment_id}: missing legacy_test_profile")
if environment.get("environment_id") != environment_id:
    raise SystemExit(f"{environment_id}: environment_id mismatch")
if profile.get("profile_id") != expected_profile_id:
    raise SystemExit(f"{environment_id}: profile_id mismatch")
if profile.get("plane_pose_id") != expected_plane_pose_id:
    raise SystemExit(f"{environment_id}: plane_pose_id mismatch")
plane_pose = PLANE_POSES.get(expected_plane_pose_id)
if plane_pose is None:
    raise SystemExit(f"{environment_id}: plane pose is missing from the registry")
if plane_pose.normal_direction != expected_plane_normal_direction:
    raise SystemExit(f"{environment_id}: matrix normal direction differs from registry")
configured_normal_direction = profile.get("plane_normal_direction")
if configured_normal_direction is not None and (
    configured_normal_direction != expected_plane_normal_direction
):
    raise SystemExit(f"{environment_id}: profile normal direction differs from registry")
if expected_plane_normal_direction == "base_link_negative_z":
    if configured_normal_direction != "base_link_negative_z":
        raise SystemExit(f"{environment_id}: flipped-normal profile is not explicit")
    if profile.get("plane_pose_registry") != (
        "environments/legacy_plane1_pose_registry.yaml"
    ):
        raise SystemExit(f"{environment_id}: flipped-normal profile registry mismatch")
if profile.get("historical_per_file_frame_binding") is not False:
    raise SystemExit(f"{environment_id}: historical frame binding must be explicitly false")
conversion = profile.get("base_link_conversion")
if conversion != "controlled_urdf_base_link_to_base_rz_pi":
    raise SystemExit(f"{environment_id}: unsupported base_link conversion evidence")
raw_pose = finite_vector(
    profile.get("raw_base_to_plane1_pose_xyz_rotvec"),
    6,
    f"{environment_id}.legacy_test_profile.raw pose",
)
if not all(
    math.isclose(left, right, rel_tol=0.0, abs_tol=1.0e-12)
    for left, right in zip(
        raw_pose,
        plane_pose.raw_base_to_plane1_pose_xyz_rotvec,
        strict=True,
    )
):
    raise SystemExit(f"{environment_id}: raw pose differs from the registry")
work_frame = setup.get("work_frame")
if not isinstance(work_frame, dict):
    raise SystemExit(f"{environment_id}: setup work_frame is invalid")
setup_raw_pose = finite_vector(
    work_frame.get("teach_pendant_pose_xyz_rotvec"),
    6,
    f"{environment_id}.setup pose",
)
if work_frame.get("parent_frame") != "base" or work_frame.get("child_frame") != "plane1":
    raise SystemExit(f"{environment_id}: setup work-frame contract is invalid")
if not all(
    math.isclose(left, right, rel_tol=0.0, abs_tol=1.0e-12)
    for left, right in zip(
        setup_raw_pose,
        raw_pose,
        strict=True,
    )
):
    raise SystemExit(f"{environment_id}: setup pose differs from legacy evidence")

tool = setup.get("tool")
rod = tool.get("rod") if isinstance(tool, dict) else None
if not isinstance(tool, dict) or not isinstance(rod, dict):
    raise SystemExit(f"{environment_id}: setup tool/rod mapping is invalid")
axis = finite_vector(
    rod.get("axis_in_parent_xyz"),
    3,
    f"{environment_id}.setup rod axis",
)
actual_rod_length_m = float(rod.get("length_m"))
translation = [component * actual_rod_length_m for component in axis]
tool_quaternion = finite_vector(
    tool.get("tip_orientation_xyzw"),
    4,
    f"{environment_id}.setup tip orientation",
)
profile_rod_length_m = float(profile.get("rod_length_m"))
if not math.isclose(actual_rod_length_m, expected_rod_length_m, abs_tol=1.0e-12):
    raise SystemExit(f"{environment_id}: tcp rod length differs from matrix")
if not math.isclose(profile_rod_length_m, actual_rod_length_m, abs_tol=1.0e-12):
    raise SystemExit(f"{environment_id}: profile rod length differs from tcp")
if any(abs(value) > 1.0e-12 for value in translation[:2]) or not math.isclose(
    translation[2], actual_rod_length_m, abs_tol=1.0e-12
):
    raise SystemExit(f"{environment_id}: tool0-to-tip translation differs from rod length")
if tool_quaternion != [0.0, 0.0, 0.0, 1.0]:
    raise SystemExit(f"{environment_id}: unsupported rotational TCP offset")

plane_translation = list(plane_pose.translation_m)
plane_quaternion = list(plane_pose.quaternion_xyzw)
if not math.isclose(sum(value * value for value in plane_quaternion), 1.0, abs_tol=1.0e-9):
    raise SystemExit(f"{environment_id}: plane1 quaternion is not normalized")
if not all(
    math.isclose(left, right, rel_tol=0.0, abs_tol=1.0e-12)
    for left, right in zip(
        plane_translation,
        plane_pose.translation_m,
        strict=True,
    )
):
    raise SystemExit(f"{environment_id}: plane translation differs from registry")
if not all(
    math.isclose(left, right, rel_tol=0.0, abs_tol=1.0e-12)
    for left, right in zip(
        plane_quaternion,
        plane_pose.quaternion_xyzw,
        strict=True,
    )
):
    raise SystemExit(f"{environment_id}: plane quaternion differs from registry")

output_file = f"{trajectory_id}.csv"
if output_file not in manifest_entries:
    raise SystemExit(f"{environment_id}: trajectory is missing from import manifest")
digest = manifest_output_digests[output_file]
manifest_digest = hashlib.sha256(manifest_payload).hexdigest()
values = (
    expected_profile_id,
    environment_id,
    trajectory_id,
    f"{actual_rod_length_m:.3f}",
    expected_plane_pose_id,
    json.dumps(raw_pose, separators=(",", ":")),
    str(conversion),
    json.dumps(plane_translation, separators=(",", ":")),
    json.dumps(plane_quaternion, separators=(",", ":")),
    json.dumps(plane_pose.plane_z_axis_base_link, separators=(",", ":")),
    plane_pose.normal_direction,
    manifest_digest,
    digest,
)
print("\t".join(values))
PY
}

if [[ "${#CASES[@]}" -ne 20 ]]; then
  echo "compile matrix must contain exactly 20 trajectories" >&2
  exit 2
fi

SEEN_TRAJECTORIES='|'
for item in "${CASES[@]}"; do
  IFS='|' read -r _ _ trajectory_id _ _ _ <<< "${item}"
  if [[ "${SEEN_TRAJECTORIES}" == *"|${trajectory_id}|"* ]]; then
    echo "duplicate trajectory in compile matrix: ${trajectory_id}" >&2
    exit 2
  fi
  SEEN_TRAJECTORIES="${SEEN_TRAJECTORIES}${trajectory_id}|"
done

if [[ "${1:-}" == "--validate-only" ]]; then
  if ! printf '%b\n' "${EVIDENCE_HEADER}"; then
    echo "failed to write compile matrix validation header" >&2
    exit 2
  fi
  for item in "${CASES[@]}"; do
    validate_case "${item}" || exit 2
  done
  exit 0
fi

OUTPUT_DIR="${1:-}"
if [[ -z "${OUTPUT_DIR}" || "${OUTPUT_DIR}" != "${ROOT}"/runs/* ]]; then
  echo "usage: $0 ${ROOT}/runs/<evidence-dir>" >&2
  echo "       UR_WAYPOINT_ROOT=<workspace> $0 --validate-only" >&2
  exit 2
fi

if ! RUNS_ROOT="$(cd -- "${ROOT}/runs" 2>/dev/null && pwd -P)"; then
  echo "workspace runs directory is unavailable: ${ROOT}/runs" >&2
  exit 2
fi
OUTPUT_BASENAME="$(basename -- "${OUTPUT_DIR}")"
if ! OUTPUT_PARENT="$(cd -- "$(dirname -- "${OUTPUT_DIR}")" 2>/dev/null && pwd -P)" || \
  [[ "${OUTPUT_PARENT}" != "${RUNS_ROOT}" || -z "${OUTPUT_BASENAME}" || \
    "${OUTPUT_BASENAME}" == "." || "${OUTPUT_BASENAME}" == ".." ]]
then
  echo "compile matrix output must be a new direct child of ${RUNS_ROOT}" >&2
  exit 2
fi
OUTPUT_DIR="${RUNS_ROOT}/${OUTPUT_BASENAME}"
if [[ -e "${OUTPUT_DIR}" || -L "${OUTPUT_DIR}" ]]; then
  echo "compile matrix evidence directory already exists: ${OUTPUT_DIR}" >&2
  exit 2
fi
if ! mkdir "${OUTPUT_DIR}" || ! mkdir "${OUTPUT_DIR}/compile_logs"; then
  echo "failed to create compile matrix evidence directory: ${OUTPUT_DIR}" >&2
  exit 2
fi
if ! cd "${ROOT}"; then
  echo "failed to enter workspace: ${ROOT}" >&2
  exit 2
fi
if [[ ! -r /opt/ros/jazzy/setup.bash ]] || ! source /opt/ros/jazzy/setup.bash; then
  echo "failed to initialize ROS 2 Jazzy" >&2
  exit 2
fi
if [[ ! -r .venv/bin/activate ]] || ! source .venv/bin/activate; then
  echo "failed to initialize the workspace virtual environment" >&2
  exit 2
fi
export PYTHONNOUSERSITE=1
if [[ ! -r install/setup.bash ]] || ! source install/setup.bash; then
  echo "failed to initialize the built ROS 2 workspace" >&2
  exit 2
fi
set -u

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "compile matrix requires command: ${command_name}" >&2
    exit 2
  fi
}

sha256_file() {
  local path="$1"
  local line
  local digest
  if [[ -L "${path}" || ! -f "${path}" ]]; then
    echo "missing regular evidence input: ${path}" >&2
    return 1
  fi
  if ! line="$(sha256sum -- "${path}")"; then
    echo "failed to hash evidence input: ${path}" >&2
    return 1
  fi
  digest="${line%% *}"
  if [[ ! "${digest}" =~ ^[0-9a-f]{64}$ ]]; then
    echo "invalid SHA-256 output for evidence input: ${path}" >&2
    return 1
  fi
  printf '%s\n' "${digest}"
}

for required_command in nvidia-smi timeout ros2 tee sha256sum date tail mv wc; do
  require_command "${required_command}"
done

EVIDENCE_ROWS=()
for item in "${CASES[@]}"; do
  if ! evidence="$(validate_case "${item}")"; then
    echo "compile matrix profile validation failed" >&2
    exit 2
  fi
  if [[ -z "${evidence}" ]]; then
    echo "compile matrix profile validation emitted no evidence" >&2
    exit 2
  fi
  EVIDENCE_ROWS+=("${evidence}")
done

NVIDIA_EVIDENCE_TMP="${OUTPUT_DIR}/nvidia-smi.txt.partial"
if ! nvidia-smi > "${NVIDIA_EVIDENCE_TMP}" || \
  ! mv -- "${NVIDIA_EVIDENCE_TMP}" "${OUTPUT_DIR}/nvidia-smi.txt";
then
  echo "nvidia-smi GPU evidence probe failed" >&2
  exit 2
fi
TORCH_EVIDENCE_TMP="${OUTPUT_DIR}/torch-gpu.txt.partial"
if ! "${PYTHON_BIN}" -c '
import torch

if not torch.cuda.is_available():
    raise SystemExit("CUDA is not available")
if torch.cuda.device_count() < 1:
    raise SystemExit("no CUDA device is visible")
probe = torch.ones(1, device="cuda")
torch.cuda.synchronize()
print(torch.__version__)
print(True)
print(torch.cuda.get_device_name(0))
print(float(probe.item()))
' > "${TORCH_EVIDENCE_TMP}" || \
  ! mv -- "${TORCH_EVIDENCE_TMP}" "${OUTPUT_DIR}/torch-gpu.txt";
then
  echo "PyTorch CUDA evidence probe failed" >&2
  exit 2
fi
MATRIX_TMP="${OUTPUT_DIR}/compile_matrix.tsv.partial"
if ! printf '%b\texit_code\telapsed_s\tcompile_run\n' "${EVIDENCE_HEADER}" \
  > "${MATRIX_TMP}"; then
  echo "failed to initialize compile_matrix.tsv" >&2
  exit 2
fi

matrix_exit_code=0
for index in "${!CASES[@]}"; do
  evidence="${EVIDENCE_ROWS[${index}]}"
  IFS=$'\t' read -r profile_id environment_id trajectory_id rod_length_m \
    plane_pose_id raw_pose base_link_conversion plane_translation plane_quaternion \
    plane_z_axis plane_normal_direction import_manifest_sha256 source_sha256 \
    <<< "${evidence}"
  source_path="${ROOT}/environments/${environment_id}/trajectories/${trajectory_id}.csv"
  manifest_path="${ROOT}/environments/${environment_id}/trajectories/localizaiton_ros2_import.yaml"
  log_path="${OUTPUT_DIR}/compile_logs/${profile_id}__${trajectory_id}.log"
  log_tmp="${log_path}.partial"
  if ! current_sha256="$(sha256_file "${source_path}")"; then
    exit 2
  fi
  if [[ "${current_sha256}" != "${source_sha256}" ]]; then
    echo "authoritative CSV changed after profile validation: ${source_path}" >&2
    exit 2
  fi
  if ! current_manifest_sha256="$(sha256_file "${manifest_path}")"; then
    exit 2
  fi
  if [[ "${current_manifest_sha256}" != "${import_manifest_sha256}" ]]; then
    echo "import manifest changed after profile validation: ${manifest_path}" >&2
    exit 2
  fi
  if ! started="$(date +%s)"; then
    echo "failed to read compile start time" >&2
    exit 2
  fi
  echo "[compile-matrix] START ${profile_id} ${environment_id}/${trajectory_id} rod=${rod_length_m} pose=${plane_pose_id}"
  timeout --signal=INT --kill-after=30s 900s \
    ros2 run hx_waypoint_tools tipdraw build-recording \
      --env "${environment_id}" \
      --trajectory "${trajectory_id}" \
      --compile-only \
      2>&1 | tee "${log_tmp}"
  pipeline_status=("${PIPESTATUS[@]}")
  if [[ "${#pipeline_status[@]}" -ne 2 ]]; then
    echo "compile pipeline did not report both ros2 and tee statuses" >&2
    exit 2
  fi
  exit_code="${pipeline_status[0]}"
  tee_exit_code="${pipeline_status[1]}"
  if [[ "${tee_exit_code}" -ne 0 ]]; then
    echo "failed to persist compile log: ${log_tmp}" >&2
    exit 2
  fi
  if [[ "${exit_code}" -eq 125 || "${exit_code}" -eq 126 || "${exit_code}" -eq 127 ]]; then
    echo "timeout/command infrastructure failed for ${trajectory_id}: ${exit_code}" >&2
    exit 2
  fi
  if ! mv -- "${log_tmp}" "${log_path}"; then
    echo "failed to seal compile log: ${log_path}" >&2
    exit 2
  fi
  if ! finished="$(date +%s)"; then
    echo "failed to read compile finish time" >&2
    exit 2
  fi
  elapsed="$(( finished - started ))"
  compile_run=""
  if [[ "${exit_code}" -eq 0 ]]; then
    if ! compile_run="$(tail -n 1 "${log_path}")"; then
      echo "failed to read successful compile evidence: ${log_path}" >&2
      exit 2
    fi
    if [[ -z "${compile_run}" || "${compile_run}" == *$'\t'* || \
      "${compile_run}" == *$'\r'* ]]
    then
      echo "successful compile emitted invalid run evidence: ${trajectory_id}" >&2
      exit 2
    fi
  else
    matrix_exit_code=1
  fi
  if ! printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${profile_id}" "${environment_id}" "${trajectory_id}" "${rod_length_m}" \
    "${plane_pose_id}" "${raw_pose}" "${base_link_conversion}" \
    "${plane_translation}" "${plane_quaternion}" "${plane_z_axis}" \
    "${plane_normal_direction}" "${import_manifest_sha256}" "${source_sha256}" \
    "${exit_code}" "${elapsed}" "${compile_run}" \
    >> "${MATRIX_TMP}"; then
    echo "failed to append compile evidence for ${trajectory_id}" >&2
    exit 2
  fi
  echo "[compile-matrix] END ${profile_id} ${environment_id}/${trajectory_id} rc=${exit_code} elapsed=${elapsed}s"
done

if ! matrix_line_count="$(wc -l < "${MATRIX_TMP}")" || \
  [[ "${matrix_line_count//[[:space:]]/}" != "21" ]];
then
  echo "compile matrix evidence is incomplete" >&2
  exit 2
fi
if ! mv -- "${MATRIX_TMP}" "${OUTPUT_DIR}/compile_matrix.tsv"; then
  echo "failed to seal compile_matrix.tsv" >&2
  exit 2
fi
exit "${matrix_exit_code}"
