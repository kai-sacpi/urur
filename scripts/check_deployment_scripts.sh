#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPLOYMENT_SCRIPTS=(
  "${SCRIPT_DIR}/setup_ubuntu24.sh"
  "${SCRIPT_DIR}/build_workspace.sh"
  "${SCRIPT_DIR}/sync_to_ubuntu24.sh"
  "${SCRIPT_DIR}/generate_ur_assets.sh"
)

fail() {
  echo "check_deployment_scripts.sh: $*" >&2
  exit 1
}

for script in "${DEPLOYMENT_SCRIPTS[@]}"; do
  [[ -x "${script}" ]] || fail "script is not executable: ${script}"
  bash -n "${script}"

  previous_effective_line=""
  source_needs_restore=0
  source_line_number=0
  line_number=0
  while IFS= read -r line || [[ -n "${line}" ]]; do
    line_number=$((line_number + 1))
    if [[ -z "${line}" || "${line}" == \#* ]]; then
      continue
    fi
    if ((source_needs_restore == 1)); then
      [[ "${line}" == "set -u" ]] || \
        fail "source must be followed by 'set -u' at ${script}:${source_line_number}"
      source_needs_restore=0
    fi
    if [[ "${line}" == source\ * ]]; then
      [[ "${previous_effective_line}" == "set +u" ]] || \
        fail "source must be preceded by 'set +u' at ${script}:${line_number}"
      source_needs_restore=1
      source_line_number="${line_number}"
    fi
    previous_effective_line="${line}"
  done < "${script}"
  ((source_needs_restore == 0)) || \
    fail "source must be followed by 'set -u' at ${script}:${source_line_number}"
done

# Regression for the Jazzy setup behavior observed on ubuntu24: a sourced setup
# file may read AMENT_TRACE_SETUP_FILES while the variable is still unset.
TEMPORARY_SETUP="$(mktemp "${TMPDIR:-/tmp}/ur-waypoint-setup.XXXXXX")"
cleanup() {
  rm -f -- "${TEMPORARY_SETUP}"
}
trap cleanup EXIT
printf '%s\n' ': "${AMENT_TRACE_SETUP_FILES}"' 'AMENT_TRACE_SETUP_FILES=ready' > \
  "${TEMPORARY_SETUP}"
unset AMENT_TRACE_SETUP_FILES || true
set +u
source "${TEMPORARY_SETUP}"
set -u
[[ "${AMENT_TRACE_SETUP_FILES}" == "ready" ]] || fail "nounset source regression failed"

echo "Deployment script syntax, executable bits, and nounset source guards passed."
