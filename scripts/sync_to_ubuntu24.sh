#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REMOTE="ubuntu24"
DESTINATION="/home/huaxin/Codes/ur-waypoint"
DRY_RUN=0
ASSUME_YES=0

usage() {
  cat <<'EOF'
Usage: scripts/sync_to_ubuntu24.sh [options]

  --remote HOST  SSH host/alias (default: ubuntu24)
  --dest PATH    absolute remote directory
  --dry-run      show rsync changes without writing files
  --yes          skip the interactive network/write confirmation

The script never uses rsync --delete and excludes remote runs/recordings.
EOF
}

fail() {
  echo "sync_to_ubuntu24.sh: $*" >&2
  exit 1
}

while (($#)); do
  case "$1" in
    --remote)
      (($# >= 2)) || fail "--remote requires a host"
      REMOTE="$2"
      shift 2
      ;;
    --dest)
      (($# >= 2)) || fail "--dest requires a path"
      DESTINATION="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --yes)
      ASSUME_YES=1
      shift
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

[[ "${REMOTE}" =~ ^[A-Za-z0-9._@-]+$ ]] || fail "unsafe SSH host/alias: ${REMOTE}"
[[ "${DESTINATION}" =~ ^/[A-Za-z0-9._/-]+$ ]] || \
  fail "destination must be a simple absolute path: ${DESTINATION}"
command -v ssh >/dev/null 2>&1 || fail "ssh is required"
command -v rsync >/dev/null 2>&1 || fail "rsync is required"

cat <<EOF
[sync][NETWORK REVIEW]
Source:      ${REPOSITORY_ROOT}/
Destination: ${REMOTE}:${DESTINATION}/
Mode:        $([[ ${DRY_RUN} -eq 1 ]] && echo dry-run || echo write)

The sync excludes .git, venv/build caches, runs, and all recording payloads.
It does not use --delete, so existing remote artifacts are preserved.
EOF

if ((ASSUME_YES == 0)); then
  if [[ ! -t 0 ]]; then
    fail "network confirmation requires an interactive terminal or --yes"
  fi
  read -r -p "Type 'yes' to connect to ${REMOTE}: " answer
  [[ "${answer}" == "yes" ]] || fail "sync was not approved"
fi

RSYNC_OPTIONS=(
  -az
  --checksum
  --human-readable
  --itemize-changes
  --exclude=/.git/
  --exclude=/.venv/
  --exclude=/build/
  --exclude=/install/
  --exclude=/log/
  --exclude=/.cache/
  --exclude=/.deps/
  --exclude=/.pytest_cache/
  --exclude=/.ruff_cache/
  --exclude=/.mypy_cache/
  '--exclude=/runs/**'
  '--exclude=/environments/*/recordings/**'
  --exclude='.DS_Store'
  --exclude='__pycache__/'
  --exclude='*.pyc'
  --exclude='*.pyo'
)

if ((DRY_RUN == 1)); then
  RSYNC_OPTIONS+=(--dry-run)
else
  ssh -- "${REMOTE}" "mkdir -p -- '${DESTINATION}'"
fi

rsync "${RSYNC_OPTIONS[@]}" -- "${REPOSITORY_ROOT}/" "${REMOTE}:${DESTINATION}/"
