#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENVIRONMENT_ID="lab_wall_a"
UR_TYPE="ur5"
ROBOT_NAME="ur"

usage() {
  cat <<'EOF'
Usage: scripts/generate_ur_assets.sh [options]

  --env ID            environment id (default: lab_wall_a)
  --ur-type TYPE      UR description type (default: ur5)
  --name NAME         generated robot name (default: ur)
The only permitted kinematics input is the environment-controlled file
environments/<env>/robot/driver_kinematics.yaml. Generates the URDF and an
asset_provenance.yaml binding that source hash to the generated URDF hash.
EOF
}

fail() {
  echo "generate_ur_assets.sh: $*" >&2
  exit 1
}

while (($#)); do
  case "$1" in
    --env)
      (($# >= 2)) || fail "--env requires an identifier"
      ENVIRONMENT_ID="$2"
      shift 2
      ;;
    --ur-type)
      (($# >= 2)) || fail "--ur-type requires a value"
      UR_TYPE="$2"
      shift 2
      ;;
    --name)
      (($# >= 2)) || fail "--name requires a value"
      ROBOT_NAME="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

[[ "${ENVIRONMENT_ID}" =~ ^[A-Za-z0-9_-]+$ ]] || fail "unsafe environment id"
[[ "${UR_TYPE}" =~ ^ur[0-9]+e?$ ]] || fail "unsupported UR type syntax: ${UR_TYPE}"
[[ "${ROBOT_NAME}" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]] || fail "unsafe robot name"

[[ -r /opt/ros/jazzy/setup.bash ]] || fail "/opt/ros/jazzy/setup.bash is missing"
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
  fail "ROS_DISTRO=${ROS_DISTRO}; start a clean shell before sourcing ROS 2 Jazzy"
fi
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
[[ "${ROS_DISTRO:-}" == "jazzy" ]] || fail "failed to activate ROS 2 Jazzy"

command -v ros2 >/dev/null 2>&1 || fail "ros2 is unavailable"
command -v xacro >/dev/null 2>&1 || fail "xacro is unavailable"
command -v sha256sum >/dev/null 2>&1 || fail "sha256sum is unavailable"
UR_DESCRIPTION_SHARE="$(ros2 pkg prefix --share ur_description 2>/dev/null)" || \
  fail "ROS package ur_description is not installed"
UR_DRIVER_SHARE="$(ros2 pkg prefix --share ur_robot_driver 2>/dev/null)" || \
  fail "ROS package ur_robot_driver is not installed"
XACRO_FILE="${UR_DRIVER_SHARE}/urdf/ur.urdf.xacro"
if [[ ! -f "${XACRO_FILE}" ]]; then
  XACRO_FILE="${UR_DESCRIPTION_SHARE}/urdf/ur.urdf.xacro"
fi
CONFIG_DIR="${UR_DESCRIPTION_SHARE}/config/${UR_TYPE}"
JOINT_LIMIT_FILE="${CONFIG_DIR}/joint_limits.yaml"
PHYSICAL_FILE="${CONFIG_DIR}/physical_parameters.yaml"
VISUAL_FILE="${CONFIG_DIR}/visual_parameters.yaml"
ROBOT_DIRECTORY="${REPOSITORY_ROOT}/environments/${ENVIRONMENT_ID}/robot"
KINEMATICS_FILE="${ROBOT_DIRECTORY}/driver_kinematics.yaml"

for required in \
  "${XACRO_FILE}" \
  "${JOINT_LIMIT_FILE}" \
  "${KINEMATICS_FILE}" \
  "${PHYSICAL_FILE}" \
  "${VISUAL_FILE}"; do
  [[ -f "${required}" ]] || fail "required UR description input is missing: ${required}"
done

OUTPUT_DIRECTORY="${ROBOT_DIRECTORY}/assets"
OUTPUT_FILE="${OUTPUT_DIRECTORY}/${UR_TYPE}.urdf"
PROVENANCE_FILE="${ROBOT_DIRECTORY}/asset_provenance.yaml"
CALIBRATION_POLICY_FILE="${ROBOT_DIRECTORY}/calibrated_kinematics.yaml"
[[ -d "${ROBOT_DIRECTORY}" ]] || \
  fail "environment robot directory does not exist"
[[ -f "${CALIBRATION_POLICY_FILE}" ]] || fail "calibrated_kinematics.yaml is missing"
mkdir -p "${OUTPUT_DIRECTORY}"
TEMP_FILE="$(mktemp "${OUTPUT_DIRECTORY}/.${UR_TYPE}.urdf.XXXXXX")"
PROVENANCE_TEMP="$(mktemp "${ROBOT_DIRECTORY}/.asset_provenance.yaml.XXXXXX")"
cleanup() {
  [[ ! -e "${TEMP_FILE}" ]] || rm -f -- "${TEMP_FILE}"
  [[ ! -e "${PROVENANCE_TEMP}" ]] || rm -f -- "${PROVENANCE_TEMP}"
}
trap cleanup EXIT

xacro "${XACRO_FILE}" \
  "name:=${ROBOT_NAME}" \
  "ur_type:=${UR_TYPE}" \
  "tf_prefix:=" \
  "joint_limit_params:=${JOINT_LIMIT_FILE}" \
  "kinematics_params:=${KINEMATICS_FILE}" \
  "physical_params:=${PHYSICAL_FILE}" \
  "visual_params:=${VISUAL_FILE}" \
  "safety_limits:=false" \
  > "${TEMP_FILE}"

grep -q '<robot' "${TEMP_FILE}" || fail "xacro output has no <robot> element"
grep -q 'name="shoulder_pan_joint"' "${TEMP_FILE}" || \
  fail "xacro output has no shoulder_pan_joint"
grep -q 'name="wrist_3_joint"' "${TEMP_FILE}" || fail "xacro output has no wrist_3_joint"
SOURCE_SHA256="$(sha256sum "${KINEMATICS_FILE}")"
SOURCE_SHA256="${SOURCE_SHA256%% *}"
URDF_SHA256="$(sha256sum "${TEMP_FILE}")"
URDF_SHA256="${URDF_SHA256%% *}"
printf '%s\n' \
  'schema_version: 1' \
  "robot_type: ${UR_TYPE}" \
  "robot_name: ${ROBOT_NAME}" \
  'source_kinematics:' \
  '  path: robot/driver_kinematics.yaml' \
  "  sha256: ${SOURCE_SHA256}" \
  'generated_urdf:' \
  "  path: robot/assets/${UR_TYPE}.urdf" \
  "  sha256: ${URDF_SHA256}" \
  'generator:' \
  '  script: scripts/generate_ur_assets.sh' \
  '  ros_distro: jazzy' > "${PROVENANCE_TEMP}"
mv -- "${TEMP_FILE}" "${OUTPUT_FILE}"
mv -- "${PROVENANCE_TEMP}" "${PROVENANCE_FILE}"
trap - EXIT

echo "Generated ${OUTPUT_FILE}"
echo "Kinematics input: ${KINEMATICS_FILE}"
echo "Asset provenance: ${PROVENANCE_FILE}"
if grep -Eq '^deployment_allowed:[[:space:]]*false[[:space:]]*$' \
  "${CALIBRATION_POLICY_FILE}"; then
  echo "WARNING: nominal kinematics were used; this does not permit real deployment." >&2
fi
