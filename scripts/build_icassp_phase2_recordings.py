#!/usr/bin/env python3
"""Build, mock-record, and inspect ICASSP Phase-2 recordings safely.

This is deliberately an offline/simulation-only orchestrator.  It consumes the
hash-bound authority bundle produced by ``generate_icassp_phase2_trajectories``
and invokes only ``tipdraw build-recording`` and ``tipdraw inspect``.  It never
invokes playback or a real controller.

The canonical compiler runs and sealed recordings remain below the selected
workspace.  Batch indexes and logs may live on another filesystem (for example
``/data``).  To place canonical runs/recordings on ``/data``, put the *complete*
built workspace there and pass it with ``--workspace``; redirecting only either
store, especially through a symlink, is intentionally rejected.
"""

from __future__ import annotations

import argparse
import csv
import fcntl
import hashlib
import math
import os
import re
import secrets
import signal
import subprocess
import sys
import tempfile
import time
from collections.abc import Iterator, Mapping, Sequence
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any, TextIO

import yaml


PHASE2_ENVIRONMENT_ID = "new_board_4cm_rod33365_up50_icassp_phase2"
AUTHORITY_INDEX_NAME = "phase2_authority_index.csv"
AUTHORITY_MANIFEST_NAME = "phase2_authority_manifest.yaml"
BUILD_INDEX_NAME = "phase2_recording_build_index.csv"
BUILD_MANIFEST_NAME = "phase2_recording_build_manifest.yaml"

SOURCE_COLUMNS = (
    "time_s",
    "frame_id",
    "tip_x_m",
    "tip_y_m",
    "tip_z_m",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
    "phase",
)

AUTHORITY_REQUIRED_COLUMNS = (
    "condition_id",
    "authority_file",
    "authority_sha256",
    "input_conditions_sha256",
    "registration_sha256",
    "environment_id",
    "environment_sha256",
    "mode",
    "use_in_main_statistics",
    "source_phase",
    "source_status",
    "frame_id",
    "tip_x_m",
    "tip_y_m",
    "tip_z_m",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
)

BUILD_INDEX_COLUMNS = (
    "schema_version",
    "event_sequence",
    "timestamp_utc",
    "condition_id",
    "trajectory_id",
    "authority_sha256",
    "authority_index_sha256",
    "input_conditions_sha256",
    "registration_sha256",
    "environment_sha256",
    "mode",
    "ros_domain_id",
    "build_key",
    "status",
    "recording_id",
    "recording_path",
    "recording_manifest_sha256",
    "log_path",
    "message",
)

ROOT_CONFIG_FILES = (
    "environment.yaml",
    "setup.yaml",
    "zones.yaml",
    "planner.yaml",
    "recording.yaml",
    "controllers.yaml",
)
ROBOT_CONFIG_FILES = (
    "robot/robot_model.yaml",
    "robot/calibrated_kinematics.yaml",
    "robot/asset_provenance.yaml",
    "robot/curobo_robot.yaml",
    "robot/collision_spheres.yaml",
)
ROBOT_DATA_FILES = ("robot/driver_kinematics.yaml",)
REPRODUCIBLE_TREE_DIRECTORIES = ("meshes", "robot/assets")

_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
_SAFE_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*\Z")
_PREVIEW_RE = re.compile(
    r"preview|do[_ -]?not|recompute|held[_ -]?out|nominal|exploratory",
    re.IGNORECASE,
)


class Phase2BuildError(RuntimeError):
    """A fail-closed authority, storage, or build contract violation."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys instead of silently taking the last value."""


def _construct_unique_mapping(
    loader: _UniqueKeyLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as error:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                "found an unhashable mapping key",
                key_node.start_mark,
            ) from error
        if duplicate:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"found duplicate key {key!r}",
                key_node.start_mark,
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class AuthorityEntry:
    condition_id: str
    authority_path: Path
    authority_sha256: str
    input_conditions_sha256: str
    registration_sha256: str
    environment_sha256: str
    mode: str
    source_phase: str
    source_status: str
    frame_id: str
    tip_pose: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class AuthorityBundle:
    bundle_root: Path
    index_path: Path
    manifest_path: Path
    index_sha256: str
    conditions_path: Path
    conditions_sha256: str
    registration_path: Path
    registration_sha256: str
    environment_sha256: str
    environment_hashes: Mapping[str, str]
    mode: str
    entries: tuple[AuthorityEntry, ...]


@dataclass(frozen=True)
class StoragePaths:
    workspace: Path
    curobo_source_root: Path
    environment_root: Path
    trajectories_root: Path
    recordings_root: Path
    compiler_runs_root: Path
    output_root: Path
    log_root: Path
    index_path: Path
    manifest_path: Path
    ros_domain_id: int


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str
    stderr: str


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _normalize_sha256(value: object, label: str) -> str:
    if not isinstance(value, str):
        raise Phase2BuildError(f"{label} must be a SHA-256 string")
    normalized = value.strip().lower()
    if normalized.startswith("sha256:"):
        normalized = normalized.removeprefix("sha256:")
    if _SHA256_RE.fullmatch(normalized) is None:
        raise Phase2BuildError(f"{label} is not a valid SHA-256 digest")
    return normalized


def _require_regular_file(path: Path, label: str) -> Path:
    try:
        status = path.lstat()
    except OSError as error:
        raise Phase2BuildError(f"cannot inspect {label}: {path}: {error}") from error
    if path.is_symlink() or not path.is_file() or status.st_size <= 0:
        raise Phase2BuildError(
            f"{label} must be a non-empty regular file and not a symlink: {path}"
        )
    return path


def _require_real_directory(path: Path, label: str) -> Path:
    try:
        path.lstat()
    except OSError as error:
        raise Phase2BuildError(f"cannot inspect {label}: {path}: {error}") from error
    if path.is_symlink() or not path.is_dir():
        raise Phase2BuildError(f"{label} must be a real directory, not a symlink: {path}")
    return path


def _path_contains_symlink(path: Path) -> bool:
    """Return true when an existing component in the supplied path is a symlink."""

    absolute = path.expanduser().absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.is_symlink():
            return True
    return False


def _load_yaml(path: Path, label: str) -> Mapping[str, Any]:
    _require_regular_file(path, label)
    try:
        value = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise Phase2BuildError(f"cannot parse {label}: {path}: {error}") from error
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise Phase2BuildError(f"{label} must be a YAML mapping with string keys")
    return value


def _require_mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise Phase2BuildError(f"{label} must be a mapping with string keys")
    return value


def _require_string(mapping: Mapping[str, Any], key: str, label: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise Phase2BuildError(f"{label}.{key} must be a non-empty string")
    return value.strip()


def _require_bool(mapping: Mapping[str, Any], key: str, label: str) -> bool:
    value = mapping.get(key)
    if not isinstance(value, bool):
        raise Phase2BuildError(f"{label}.{key} must be a boolean")
    return value


def _require_positive_int(mapping: Mapping[str, Any], key: str, label: str) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise Phase2BuildError(f"{label}.{key} must be a positive integer")
    return value


def _parse_csv_bool(value: str, label: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true"}:
        return True
    if normalized in {"0", "false"}:
        return False
    raise Phase2BuildError(f"{label} must be exactly 0/1 or false/true")


def _safe_relative_path(root: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value or "\\" in value or "\0" in value:
        raise Phase2BuildError(f"unsafe relative path in {label}: {value!r}")
    relative = PurePosixPath(value)
    if (
        relative.is_absolute()
        or relative.as_posix() in {"", "."}
        or any(part in {"", ".", ".."} for part in value.split("/"))
        or relative.as_posix() != value
    ):
        raise Phase2BuildError(f"unsafe relative path in {label}: {value!r}")
    candidate = root.joinpath(*relative.parts)
    if _path_contains_symlink(candidate):
        raise Phase2BuildError(f"{label} contains a symlink component: {value}")
    resolved_root = root.resolve()
    resolved_candidate = candidate.resolve()
    if resolved_root not in resolved_candidate.parents:
        raise Phase2BuildError(f"{label} escapes its authority bundle: {value}")
    return candidate


def _environment_hashes(environment_root: Path) -> dict[str, str]:
    paths = [
        environment_root / relative
        for relative in (*ROOT_CONFIG_FILES, *ROBOT_CONFIG_FILES, *ROBOT_DATA_FILES)
    ]
    for relative in REPRODUCIBLE_TREE_DIRECTORIES:
        directory = environment_root / relative
        if directory.is_dir():
            paths.extend(path for path in directory.rglob("*") if path.is_file())
    output: dict[str, str] = {}
    for path in sorted(paths, key=lambda item: item.relative_to(environment_root).as_posix()):
        relative = path.relative_to(environment_root).as_posix()
        _require_regular_file(path, f"environment input {relative}")
        output[relative] = _sha256_file(path)
    return output


def _environment_tree_sha256(hashes: Mapping[str, str]) -> str:
    digest = hashlib.sha256()
    for relative, file_digest in sorted(hashes.items()):
        digest.update(f"{relative}\0{file_digest}\n".encode())
    return digest.hexdigest()


def _is_within(path: Path, parent: Path) -> bool:
    resolved_path = path.resolve()
    resolved_parent = parent.resolve()
    return resolved_path == resolved_parent or resolved_parent in resolved_path.parents


def _validate_environment(
    workspace: Path,
    environment_id: str,
) -> tuple[Path, dict[str, str], str]:
    if environment_id != PHASE2_ENVIRONMENT_ID:
        raise Phase2BuildError(
            f"this orchestrator accepts only {PHASE2_ENVIRONMENT_ID}; got {environment_id}"
        )
    environment_root = workspace / "environments" / environment_id
    _require_real_directory(environment_root, "Phase-2 environment")
    environment_yaml = _load_yaml(environment_root / "environment.yaml", "environment.yaml")
    actual_id = _require_string(environment_yaml, "environment_id", "environment.yaml")
    if actual_id != environment_id:
        raise Phase2BuildError(
            f"environment.yaml environment_id mismatch: expected {environment_id}, got {actual_id}"
        )
    hashes = _environment_hashes(environment_root)
    return environment_root, hashes, _environment_tree_sha256(hashes)


def _validated_curobo_source(workspace: Path) -> Path:
    lock = _load_yaml(workspace / "curobo.lock", "workspace curobo.lock")
    commit = _require_string(lock, "commit", "curobo.lock")
    source_tree = _require_string(lock, "source_tree", "curobo.lock")
    if re.fullmatch(r"[0-9a-f]{40}", commit) is None:
        raise Phase2BuildError("curobo.lock commit must be a lowercase 40-hex identity")
    if re.fullmatch(r"[0-9a-f]{40}", source_tree) is None:
        raise Phase2BuildError("curobo.lock source_tree must be a lowercase 40-hex identity")
    source_root = workspace / ".deps" / f"curobo-{commit}"
    _require_real_directory(source_root, "locked cuRobo source")
    commit_marker = _require_regular_file(
        source_root / ".ur-waypoint-curobo-commit",
        "locked cuRobo commit marker",
    )
    tree_marker = _require_regular_file(
        source_root / ".ur-waypoint-curobo-tree",
        "locked cuRobo tree marker",
    )
    if commit_marker.read_text(encoding="utf-8").strip() != commit:
        raise Phase2BuildError("locked cuRobo commit marker does not match curobo.lock")
    if tree_marker.read_text(encoding="utf-8").strip() != source_tree:
        raise Phase2BuildError("locked cuRobo tree marker does not match curobo.lock")
    _require_regular_file(source_root / "curobo" / "__init__.py", "locked cuRobo package")
    return source_root.resolve()


def _load_csv_rows(path: Path, label: str) -> tuple[tuple[str, ...], list[dict[str, str]]]:
    _require_regular_file(path, label)
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise Phase2BuildError(f"{label} is missing a header")
            fieldnames = tuple(reader.fieldnames)
            if len(set(fieldnames)) != len(fieldnames):
                raise Phase2BuildError(f"{label} contains duplicate header names")
            rows: list[dict[str, str]] = []
            for line_number, row in enumerate(reader, start=2):
                if None in row or any(value is None for value in row.values()):
                    raise Phase2BuildError(
                        f"{label} row {line_number} has missing or extra columns"
                    )
                rows.append({str(key): str(value) for key, value in row.items()})
    except (OSError, UnicodeError, csv.Error) as error:
        raise Phase2BuildError(f"cannot parse {label}: {path}: {error}") from error
    return fieldnames, rows


def _parse_finite(value: str, label: str) -> float:
    try:
        parsed = float(value)
    except ValueError as error:
        raise Phase2BuildError(f"{label} must be numeric") from error
    if not math.isfinite(parsed):
        raise Phase2BuildError(f"{label} must be finite")
    return parsed


def _validate_authority_csv(entry: AuthorityEntry) -> None:
    fieldnames, rows = _load_csv_rows(entry.authority_path, "authority trajectory")
    if fieldnames != SOURCE_COLUMNS:
        raise Phase2BuildError(
            f"authority trajectory header must be exactly {','.join(SOURCE_COLUMNS)}: "
            f"{entry.authority_path}"
        )
    if len(rows) != 3:
        raise Phase2BuildError(
            f"settled authority trajectory must contain exactly 3 rows: {entry.authority_path}"
        )
    if [row["time_s"] for row in rows] != ["0.000000000", "1.000000000", "3.000000000"]:
        raise Phase2BuildError(
            "settled authority times must be exactly 0.000000000, 1.000000000, "
            f"3.000000000: {entry.authority_path}"
        )
    if [row["phase"] for row in rows] != ["draw", "hold", "hold"]:
        raise Phase2BuildError(
            f"settled authority phases must be draw,hold,hold: {entry.authority_path}"
        )
    pose_columns = SOURCE_COLUMNS[1:9]
    first_pose = tuple(rows[0][column] for column in pose_columns)
    if any(tuple(row[column] for column in pose_columns) != first_pose for row in rows[1:]):
        raise Phase2BuildError(
            f"settled authority hold rows must remain strictly static: {entry.authority_path}"
        )
    if rows[0]["frame_id"] != entry.frame_id or entry.frame_id != "plane1":
        raise Phase2BuildError(
            f"authority trajectory must use the frozen plane1 frame: {entry.authority_path}"
        )
    numeric_columns = SOURCE_COLUMNS[2:9]
    pose = tuple(
        _parse_finite(rows[0][column], f"{entry.authority_path.name}.{column}")
        for column in numeric_columns
    )
    if any(abs(actual - expected) > 5.0e-12 for actual, expected in zip(pose, entry.tip_pose)):
        raise Phase2BuildError(
            f"authority trajectory pose does not match its index row: {entry.condition_id}"
        )
    quaternion_norm = math.sqrt(sum(value * value for value in pose[3:]))
    if abs(quaternion_norm - 1.0) > 1.0e-9:
        raise Phase2BuildError(
            f"authority trajectory quaternion is not normalized: {entry.condition_id}"
        )


def _validate_conditions_input(
    path: Path,
    entries: Sequence[AuthorityEntry],
    *,
    mode: str,
    declared_contains_preview: bool,
) -> None:
    fieldnames, rows = _load_csv_rows(path, "bundled conditions input")
    required = {"condition_id", "phase", "status"}
    if mode == "formal":
        required.add("use_in_main_statistics")
    if not required.issubset(fieldnames):
        missing = ", ".join(sorted(required.difference(fieldnames)))
        raise Phase2BuildError(f"conditions input is missing required columns: {missing}")
    entry_map = {entry.condition_id: entry for entry in entries}
    condition_ids: set[str] = set()
    actual_contains_preview = False
    for line_number, row in enumerate(rows, start=2):
        condition_id = row["condition_id"].strip()
        if condition_id in condition_ids:
            raise Phase2BuildError(f"duplicate bundled condition_id: {condition_id}")
        condition_ids.add(condition_id)
        entry = entry_map.get(condition_id)
        if entry is None:
            raise Phase2BuildError(
                f"bundled condition {condition_id!r} has no authority index entry"
            )
        if row["phase"] != entry.source_phase or row["status"] != entry.source_status:
            raise Phase2BuildError(
                f"bundled condition metadata disagrees with authority index: {condition_id}"
            )
        for name in ("phase", "status"):
            if _PREVIEW_RE.search(row[name]):
                actual_contains_preview = True
        if mode == "formal":
            if actual_contains_preview:
                raise Phase2BuildError(
                    f"formal build rejects preview/unfrozen conditions at row {line_number}"
                )
            if (
                row["phase"] != "hardware_frozen_main"
                or row["status"] != "HARDWARE_FROZEN"
                or not _parse_csv_bool(
                    row["use_in_main_statistics"],
                    f"formal conditions row {line_number}.use_in_main_statistics",
                )
            ):
                raise Phase2BuildError(
                    f"formal conditions row {line_number} is not explicitly hardware-frozen"
                )
        elif "use_in_main_statistics" in fieldnames and _parse_csv_bool(
            row["use_in_main_statistics"],
            f"exploratory conditions row {line_number}.use_in_main_statistics",
        ):
            raise Phase2BuildError(
                f"exploratory conditions row {line_number} cannot enter main statistics"
            )
    if condition_ids != set(entry_map):
        raise Phase2BuildError("bundled conditions/index condition sets do not match exactly")
    if actual_contains_preview != declared_contains_preview:
        raise Phase2BuildError(
            "authority manifest source_contains_preview contradicts the bundled conditions"
        )


def _manifest_entry_map(value: object) -> dict[str, Mapping[str, Any]]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise Phase2BuildError("authority manifest entries must be a sequence")
    output: dict[str, Mapping[str, Any]] = {}
    for index, raw in enumerate(value):
        entry = _require_mapping(raw, f"authority manifest entries[{index}]")
        condition_id = _require_string(entry, "condition_id", f"entries[{index}]")
        if condition_id in output:
            raise Phase2BuildError(f"duplicate manifest condition_id: {condition_id}")
        output[condition_id] = entry
    return output


def load_authority_bundle(
    index_path: Path,
    manifest_path: Path,
    *,
    expected_mode: str,
    environment_id: str,
    environment_hashes: Mapping[str, str],
    environment_sha256: str,
) -> AuthorityBundle:
    """Load and cross-check every authority/hash edge before any subprocess."""

    if _path_contains_symlink(index_path) or _path_contains_symlink(manifest_path):
        raise Phase2BuildError("authority index/manifest paths must not contain symlinks")
    index_path = _require_regular_file(index_path, "authority index").resolve()
    manifest_path = _require_regular_file(manifest_path, "authority manifest").resolve()
    if index_path.parent != manifest_path.parent:
        raise Phase2BuildError("authority index and manifest must share one bundle directory")
    bundle_root = index_path.parent
    manifest = _load_yaml(manifest_path, "authority manifest")
    if (
        isinstance(manifest.get("schema_version"), bool)
        or manifest.get("schema_version") != 1
    ):
        raise Phase2BuildError("authority manifest schema_version must be 1")
    if manifest.get("manifest_type") != "icassp_phase2_authority_bundle":
        raise Phase2BuildError("authority manifest_type is not recognized")
    if _require_string(manifest, "environment_id", "authority manifest") != environment_id:
        raise Phase2BuildError("authority manifest environment_id does not match the request")
    mode = _require_string(manifest, "mode", "authority manifest")
    if mode != expected_mode:
        raise Phase2BuildError(
            f"authority manifest mode must be {expected_mode}; got {mode}"
        )
    expected_use = expected_mode == "formal"
    if _require_bool(manifest, "use_in_main_statistics", "authority manifest") != expected_use:
        raise Phase2BuildError("authority manifest main-statistics flag contradicts its mode")
    source_contains_preview = _require_bool(
        manifest, "source_contains_preview", "authority manifest"
    )
    if expected_mode == "formal" and source_contains_preview:
        raise Phase2BuildError("formal authority manifest declares preview source conditions")

    index_evidence = _require_mapping(
        manifest.get("authority_index"), "authority manifest.authority_index"
    )
    declared_index = _safe_relative_path(
        bundle_root,
        _require_string(index_evidence, "path", "authority_index"),
        "authority_index.path",
    )
    if declared_index.resolve() != index_path:
        raise Phase2BuildError("authority manifest points to a different index file")
    index_sha256 = _sha256_file(index_path)
    if index_sha256 != _normalize_sha256(
        index_evidence.get("sha256"), "authority_index.sha256"
    ):
        raise Phase2BuildError("authority index SHA-256 does not match its manifest")

    conditions_evidence = _require_mapping(
        manifest.get("input_conditions"), "authority manifest.input_conditions"
    )
    conditions_path = _safe_relative_path(
        bundle_root,
        _require_string(conditions_evidence, "path", "input_conditions"),
        "input_conditions.path",
    )
    _require_regular_file(conditions_path, "bundled conditions input")
    conditions_sha256 = _normalize_sha256(
        conditions_evidence.get("sha256"), "input_conditions.sha256"
    )
    if _sha256_file(conditions_path) != conditions_sha256:
        raise Phase2BuildError("bundled conditions SHA-256 does not match the manifest")

    registration_evidence = _require_mapping(
        manifest.get("registration"), "authority manifest.registration"
    )
    registration_path = _safe_relative_path(
        bundle_root,
        _require_string(registration_evidence, "path", "registration"),
        "registration.path",
    )
    _require_regular_file(registration_path, "bundled registration input")
    registration_sha256 = _normalize_sha256(
        registration_evidence.get("sha256"), "registration.sha256"
    )
    if _sha256_file(registration_path) != registration_sha256:
        raise Phase2BuildError("bundled registration SHA-256 does not match the manifest")
    registration_document = _load_yaml(registration_path, "bundled registration input")
    if (
        isinstance(registration_document.get("schema_version"), bool)
        or registration_document.get("schema_version") != 1
        or registration_document.get("environment_id") != environment_id
    ):
        raise Phase2BuildError("bundled registration schema/environment identity is invalid")
    registration_status = _require_string(
        registration_evidence, "status", "authority manifest.registration"
    )
    document_status = _require_string(
        registration_document, "status", "bundled registration input"
    )
    if registration_status != document_status:
        raise Phase2BuildError(
            "authority manifest registration status contradicts registration.yaml"
        )
    document_use = _require_bool(
        registration_document,
        "use_in_main_statistics",
        "bundled registration input",
    )
    allowed_registration_statuses = (
        {"HARDWARE_FROZEN"}
        if expected_mode == "formal"
        else {"EXPLORATORY", "HARDWARE_FROZEN"}
    )
    if registration_status not in allowed_registration_statuses:
        raise Phase2BuildError(
            f"registration status {registration_status!r} is invalid for {expected_mode} mode"
        )
    if registration_status == "HARDWARE_FROZEN" and not document_use:
        raise Phase2BuildError("HARDWARE_FROZEN registration must permit main statistics")
    if registration_status == "EXPLORATORY" and document_use:
        raise Phase2BuildError("EXPLORATORY registration cannot permit main statistics")
    for metadata_name in ("magnet_id", "polarity", "roll_policy"):
        if _require_string(
            registration_evidence,
            metadata_name,
            "authority manifest.registration",
        ) != _require_string(
            registration_document,
            metadata_name,
            "bundled registration input",
        ):
            raise Phase2BuildError(
                f"authority manifest registration {metadata_name} contradicts registration.yaml"
            )

    environment_evidence = _require_mapping(
        manifest.get("environment"), "authority manifest.environment"
    )
    if _require_string(environment_evidence, "id", "environment") != environment_id:
        raise Phase2BuildError("authority environment.id does not match the request")
    declared_environment_sha256 = _normalize_sha256(
        environment_evidence.get("tree_sha256"), "environment.tree_sha256"
    )
    if declared_environment_sha256 != environment_sha256:
        raise Phase2BuildError(
            "authority environment tree hash is stale or for another environment"
        )
    raw_hashes = _require_mapping(
        environment_evidence.get("hashes"), "authority manifest.environment.hashes"
    )
    declared_hashes = {
        relative: _normalize_sha256(digest, f"environment.hashes.{relative}")
        for relative, digest in raw_hashes.items()
    }
    if declared_hashes != dict(environment_hashes):
        raise Phase2BuildError(
            "authority environment hashes do not exactly match the current Phase-2 environment"
        )

    fieldnames, rows = _load_csv_rows(index_path, "authority index")
    missing = set(AUTHORITY_REQUIRED_COLUMNS).difference(fieldnames)
    if missing:
        raise Phase2BuildError(
            "authority index is missing required columns: " + ", ".join(sorted(missing))
        )
    condition_count = _require_positive_int(
        manifest, "condition_count", "authority manifest"
    )
    expected_count = _require_positive_int(
        manifest, "expected_condition_count", "authority manifest"
    )
    if condition_count != len(rows):
        raise Phase2BuildError("authority manifest condition_count does not match its index")
    if expected_count <= 0 or expected_count != condition_count:
        raise Phase2BuildError(
            "authority bundle is incomplete relative to expected_condition_count"
        )

    manifest_entries = _manifest_entry_map(manifest.get("entries"))
    entries: list[AuthorityEntry] = []
    seen_paths: set[Path] = set()
    seen_conditions: set[str] = set()
    for line_number, row in enumerate(rows, start=2):
        condition_id = row["condition_id"].strip()
        if _SAFE_ID_RE.fullmatch(condition_id) is None:
            raise Phase2BuildError(
                f"authority index row {line_number} has an unsafe condition_id"
            )
        if condition_id in seen_conditions:
            raise Phase2BuildError(f"duplicate authority condition_id: {condition_id}")
        seen_conditions.add(condition_id)
        authority_path = _safe_relative_path(
            bundle_root, row["authority_file"], f"row {line_number}.authority_file"
        )
        _require_regular_file(authority_path, "authority trajectory")
        if authority_path.suffix != ".csv" or authority_path in seen_paths:
            raise Phase2BuildError("authority paths must be unique CSV files")
        seen_paths.add(authority_path)
        authority_sha256 = _normalize_sha256(
            row["authority_sha256"], f"row {line_number}.authority_sha256"
        )
        if _sha256_file(authority_path) != authority_sha256:
            raise Phase2BuildError(f"authority SHA-256 mismatch: {authority_path}")
        if row["environment_id"].strip() != environment_id:
            raise Phase2BuildError(f"authority row {line_number} environment_id mismatch")
        row_mode = row["mode"].strip()
        if row_mode != expected_mode:
            raise Phase2BuildError(f"authority row {line_number} mode mismatch")
        if _parse_csv_bool(
            row["use_in_main_statistics"],
            f"row {line_number}.use_in_main_statistics",
        ) != expected_use:
            raise Phase2BuildError(
                f"authority row {line_number} main-statistics flag mismatch"
            )
        if _normalize_sha256(
            row["input_conditions_sha256"],
            f"row {line_number}.input_conditions_sha256",
        ) != conditions_sha256:
            raise Phase2BuildError("authority rows do not share the frozen conditions hash")
        if _normalize_sha256(
            row["registration_sha256"],
            f"row {line_number}.registration_sha256",
        ) != registration_sha256:
            raise Phase2BuildError("authority rows do not share the frozen registration hash")
        if _normalize_sha256(
            row["environment_sha256"],
            f"row {line_number}.environment_sha256",
        ) != environment_sha256:
            raise Phase2BuildError("authority rows do not share the current environment hash")
        if expected_mode == "formal" and any(
            _PREVIEW_RE.search(row[name] or "")
            for name in ("authority_file", "source_phase", "source_status")
        ):
            raise Phase2BuildError(
                f"formal build rejects preview/unfrozen authority row {line_number}"
            )
        if expected_mode == "formal" and (
            row["source_phase"] != "hardware_frozen_main"
            or row["source_status"] != "HARDWARE_FROZEN"
        ):
            raise Phase2BuildError(
                f"formal authority row {line_number} is not explicitly hardware-frozen"
            )
        if expected_mode == "exploratory" and not authority_path.name.endswith(
            "__exploratory.csv"
        ):
            raise Phase2BuildError(
                "exploratory authority files must end in __exploratory.csv"
            )
        tip_pose = tuple(
            _parse_finite(row[name], f"row {line_number}.{name}")
            for name in (
                "tip_x_m",
                "tip_y_m",
                "tip_z_m",
                "tip_qx",
                "tip_qy",
                "tip_qz",
                "tip_qw",
            )
        )
        entry = AuthorityEntry(
            condition_id=condition_id,
            authority_path=authority_path,
            authority_sha256=authority_sha256,
            input_conditions_sha256=conditions_sha256,
            registration_sha256=registration_sha256,
            environment_sha256=environment_sha256,
            mode=expected_mode,
            source_phase=row["source_phase"].strip(),
            source_status=row["source_status"].strip(),
            frame_id=row["frame_id"].strip(),
            tip_pose=tip_pose,  # type: ignore[arg-type]
        )
        _validate_authority_csv(entry)
        manifest_entry = manifest_entries.get(condition_id)
        if manifest_entry is None:
            raise Phase2BuildError(f"authority manifest has no entry for {condition_id}")
        manifest_authority_file = manifest_entry.get(
            "authority_file", manifest_entry.get("path")
        )
        manifest_authority_hash = manifest_entry.get(
            "authority_sha256", manifest_entry.get("sha256")
        )
        if manifest_authority_file != row["authority_file"] or _normalize_sha256(
            manifest_authority_hash,
            f"manifest entry {condition_id}.authority_sha256",
        ) != authority_sha256:
            raise Phase2BuildError(
                f"authority manifest/index entry mismatch for {condition_id}"
            )
        if (
            manifest_entry.get("mode") != expected_mode
            or manifest_entry.get("use_in_main_statistics") is not expected_use
        ):
            raise Phase2BuildError(
                f"authority manifest entry mode/statistics mismatch for {condition_id}"
            )
        entries.append(entry)
    if set(manifest_entries) != seen_conditions:
        raise Phase2BuildError("authority manifest/index condition sets do not match exactly")
    _validate_conditions_input(
        conditions_path,
        entries,
        mode=expected_mode,
        declared_contains_preview=source_contains_preview,
    )

    return AuthorityBundle(
        bundle_root=bundle_root,
        index_path=index_path,
        manifest_path=manifest_path,
        index_sha256=index_sha256,
        conditions_path=conditions_path,
        conditions_sha256=conditions_sha256,
        registration_path=registration_path,
        registration_sha256=registration_sha256,
        environment_sha256=environment_sha256,
        environment_hashes=dict(environment_hashes),
        mode=expected_mode,
        entries=tuple(entries),
    )


def _resolve_storage(args: argparse.Namespace, mode: str) -> tuple[StoragePaths, dict[str, str]]:
    raw_workspace = Path(args.workspace).expanduser().absolute()
    if _path_contains_symlink(raw_workspace):
        raise Phase2BuildError("workspace path must not contain symlink components")
    workspace = raw_workspace.resolve()
    _require_real_directory(workspace, "workspace")
    _require_regular_file(workspace / "curobo.lock", "workspace curobo.lock")
    curobo_source_root = _validated_curobo_source(workspace)
    environment_root, environment_hashes, environment_sha256 = _validate_environment(
        workspace, args.environment_id
    )
    trajectories_root = _require_real_directory(
        environment_root / "trajectories", "environment trajectories store"
    )
    recordings_root = _require_real_directory(
        environment_root / "recordings", "canonical recording store"
    )
    compiler_runs_root = workspace / "runs"
    compiler_runs_root.mkdir(parents=True, exist_ok=True)
    _require_real_directory(compiler_runs_root, "canonical compiler runs store")

    if args.runs_root is not None:
        raw_requested = Path(args.runs_root).expanduser()
        requested = raw_requested.resolve()
        if _path_contains_symlink(raw_requested) or requested != compiler_runs_root:
            raise Phase2BuildError(
                "tipdraw fixes compile staging at <workspace>/runs; --runs-root cannot "
                "redirect it. Put the complete built workspace on /data and pass that "
                "path with --workspace. Symlink redirection is not supported."
            )
    if args.recordings_root is not None:
        raw_requested = Path(args.recordings_root).expanduser()
        requested = raw_requested.resolve()
        if _path_contains_symlink(raw_requested) or requested != recordings_root:
            raise Phase2BuildError(
                "sealed recordings must remain in the environment's canonical recordings "
                "store. Put the complete built workspace on /data; do not redirect or "
                "symlink only recordings/."
            )

    output_base = Path(args.output_root).expanduser().absolute()
    raw_log_base = Path(args.log_root).expanduser().absolute() if args.log_root else None
    if _path_contains_symlink(output_base) or (
        raw_log_base is not None and _path_contains_symlink(raw_log_base)
    ):
        raise Phase2BuildError("output/log roots must not contain symlink components")
    log_base = raw_log_base if raw_log_base is not None else output_base / "logs"
    output_root = output_base / mode
    log_root = log_base / mode
    if _is_within(output_root, environment_root) or _is_within(log_root, environment_root):
        raise Phase2BuildError(
            "batch output/log roots must not overlap the environment tree; doing so could "
            "mutate the hash-bound authority while recordings are being built"
        )
    output_root.mkdir(parents=True, exist_ok=True)
    log_root.mkdir(parents=True, exist_ok=True)
    _require_real_directory(output_root, "mode-specific output root")
    _require_real_directory(log_root, "mode-specific log root")
    return (
        StoragePaths(
            workspace=workspace,
            curobo_source_root=curobo_source_root,
            environment_root=environment_root,
            trajectories_root=trajectories_root,
            recordings_root=recordings_root,
            compiler_runs_root=compiler_runs_root,
            output_root=output_root,
            log_root=log_root,
            index_path=output_root / BUILD_INDEX_NAME,
            manifest_path=output_root / BUILD_MANIFEST_NAME,
            ros_domain_id=args.ros_domain_id,
        ),
        {**environment_hashes, "__tree_sha256__": environment_sha256},
    )


def _assert_environment_unchanged(paths: StoragePaths, bundle: AuthorityBundle) -> None:
    current_hashes = _environment_hashes(paths.environment_root)
    current_tree_sha256 = _environment_tree_sha256(current_hashes)
    if (
        current_hashes != dict(bundle.environment_hashes)
        or current_tree_sha256 != bundle.environment_sha256
    ):
        raise Phase2BuildError(
            "Phase-2 environment changed after authority validation; refusing build/recovery"
        )


def _atomic_write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            directory_descriptor = os.open(path.parent, os.O_RDONLY)
        except OSError:
            return
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_write_csv(path: Path, rows: Sequence[Mapping[str, str]]) -> None:
    import io

    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=BUILD_INDEX_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({name: row.get(name, "") for name in BUILD_INDEX_COLUMNS})
    _atomic_write_bytes(path, buffer.getvalue().encode())


@contextmanager
def _exclusive_lock(path: Path) -> Iterator[None]:
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags, 0o600)
    except OSError as error:
        raise Phase2BuildError(f"cannot open exclusive build lock {path}: {error}") from error
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise Phase2BuildError(f"another Phase-2 builder holds {path}") from error
        yield
    finally:
        os.close(descriptor)


@contextmanager
def _batch_locks(args: argparse.Namespace, mode: str) -> Iterator[None]:
    raw_workspace = Path(args.workspace).expanduser().absolute()
    raw_output_base = Path(args.output_root).expanduser().absolute()
    if _path_contains_symlink(raw_workspace) or _path_contains_symlink(raw_output_base):
        raise Phase2BuildError("workspace/output lock roots must not contain symlinks")
    _require_real_directory(raw_workspace, "workspace lock root")
    environment_root = (
        raw_workspace / "environments" / PHASE2_ENVIRONMENT_ID
    )
    if _is_within(raw_output_base, environment_root):
        raise Phase2BuildError(
            "output lock root must not overlap the hash-bound Phase-2 environment"
        )
    raw_output_base.mkdir(parents=True, exist_ok=True)
    _require_real_directory(raw_output_base, "output lock root")
    lock_paths = sorted(
        {
            raw_workspace / ".icassp_phase2_builder.lock",
            raw_output_base / f".icassp_phase2_builder_{mode}.lock",
        },
        key=str,
    )
    with ExitStack() as stack:
        for lock_path in lock_paths:
            stack.enter_context(_exclusive_lock(lock_path))
        yield


def _load_build_events(
    path: Path,
    bundle: AuthorityBundle,
    paths: StoragePaths,
) -> list[dict[str, str]]:
    if not path.exists():
        return []
    fieldnames, rows = _load_csv_rows(path, "recording build index")
    if fieldnames != BUILD_INDEX_COLUMNS:
        raise Phase2BuildError("existing recording build index has an incompatible schema")
    entries = {entry.condition_id: entry for entry in bundle.entries}
    allowed_statuses = {"success", "recovered", "failed"}
    for event_index, row in enumerate(rows, start=1):
        line_number = event_index + 1
        if row["schema_version"] != "1" or row["mode"] != bundle.mode:
            raise Phase2BuildError(f"build index contract mismatch at row {line_number}")
        expected = {
            "authority_index_sha256": bundle.index_sha256,
            "input_conditions_sha256": bundle.conditions_sha256,
            "registration_sha256": bundle.registration_sha256,
            "environment_sha256": bundle.environment_sha256,
        }
        if any(row[name] != value for name, value in expected.items()):
            raise Phase2BuildError(
                "existing build index belongs to a different authority/registration/"
                f"conditions/environment freeze (row {line_number})"
            )
        if row["event_sequence"] != str(event_index):
            raise Phase2BuildError(f"non-contiguous event_sequence at row {line_number}")
        entry = entries.get(row["condition_id"])
        if entry is None:
            raise Phase2BuildError(f"unknown condition_id in build index row {line_number}")
        trajectory_id = _trajectory_id(entry)
        if (
            row["trajectory_id"] != trajectory_id
            or row["authority_sha256"] != entry.authority_sha256
            or row["build_key"] != _build_key(bundle, entry, trajectory_id)
        ):
            raise Phase2BuildError(f"build identity mismatch at row {line_number}")
        if row["status"] not in allowed_statuses:
            raise Phase2BuildError(f"unknown build status at row {line_number}")
        try:
            ros_domain_id = int(row["ros_domain_id"])
        except ValueError as error:
            raise Phase2BuildError(
                f"invalid ros_domain_id at row {line_number}"
            ) from error
        if not 0 <= ros_domain_id <= 232:
            raise Phase2BuildError(f"invalid ros_domain_id at row {line_number}")
        if row["status"] in {"success", "recovered"}:
            recording_id = row["recording_id"]
            if _SAFE_ID_RE.fullmatch(recording_id) is None:
                raise Phase2BuildError(f"unsafe recording_id at row {line_number}")
            expected_path = paths.recordings_root / recording_id
            if Path(row["recording_path"]) != expected_path:
                raise Phase2BuildError(
                    f"noncanonical recording_path at row {line_number}"
                )
            _normalize_sha256(
                row["recording_manifest_sha256"],
                f"build index row {line_number}.recording_manifest_sha256",
            )
        elif any(
            row[name]
            for name in (
                "recording_id",
                "recording_path",
                "recording_manifest_sha256",
            )
        ):
            raise Phase2BuildError(
                f"failed build row {line_number} must not claim a recording"
            )
    return rows


def _trajectory_id(entry: AuthorityEntry) -> str:
    prefix = "icassp_phase2_exploratory" if entry.mode == "exploratory" else "icassp_phase2"
    trajectory_id = f"{prefix}__{entry.condition_id}"
    if _SAFE_ID_RE.fullmatch(trajectory_id) is None:
        raise Phase2BuildError(f"unsafe generated trajectory_id: {trajectory_id}")
    return trajectory_id


def _build_key(bundle: AuthorityBundle, entry: AuthorityEntry, trajectory_id: str) -> str:
    digest = hashlib.sha256()
    for value in (
        "phase2_recording_build_v1",
        bundle.mode,
        entry.condition_id,
        trajectory_id,
        entry.authority_sha256,
        bundle.index_sha256,
        bundle.conditions_sha256,
        bundle.registration_sha256,
        bundle.environment_sha256,
    ):
        digest.update(value.encode())
        digest.update(b"\0")
    return digest.hexdigest()


def _install_authority(entry: AuthorityEntry, destination: Path) -> None:
    if destination.exists() or destination.is_symlink():
        _require_regular_file(destination, "installed authority trajectory")
        if _sha256_file(destination) != entry.authority_sha256:
            raise Phase2BuildError(
                f"refusing to overwrite a different installed authority: {destination}"
            )
        return
    _atomic_write_bytes(destination, entry.authority_path.read_bytes())
    if _sha256_file(destination) != entry.authority_sha256:
        raise Phase2BuildError(f"installed authority hash mismatch: {destination}")


def _append_log(log_path: Path, command: Sequence[str], result: CommandResult) -> None:
    with log_path.open("a", encoding="utf-8") as stream:
        stream.write("$ " + " ".join(command) + "\n")
        stream.write(result.stdout)
        if result.stdout and not result.stdout.endswith("\n"):
            stream.write("\n")
        if result.stderr:
            stream.write("[stderr]\n")
            stream.write(result.stderr)
            if not result.stderr.endswith("\n"):
                stream.write("\n")
        stream.write(f"[exit={result.returncode}]\n")
        stream.flush()
        os.fsync(stream.fileno())


def _validate_offline_tipdraw_command(command: Sequence[str]) -> None:
    prefix = ("ros2", "run", "hx_waypoint_tools", "tipdraw", "--workspace")
    if tuple(command[:5]) != prefix or len(command) < 7:
        raise Phase2BuildError("refusing command outside the offline tipdraw allowlist")
    subcommand = command[6]
    tail = tuple(command[6:])
    if subcommand == "build-recording":
        valid = (
            len(tail) == 6
            and tail[1:3] == ("--env", PHASE2_ENVIRONMENT_ID)
            and tail[3] == "--trajectory"
            and _SAFE_ID_RE.fullmatch(tail[4]) is not None
            and tail[5:] == ("--external-runtime",)
        )
    elif subcommand == "inspect":
        valid = (
            len(tail) == 5
            and tail[1:3] == ("--env", PHASE2_ENVIRONMENT_ID)
            and tail[3] == "--recording"
            and _SAFE_ID_RE.fullmatch(tail[4]) is not None
        )
    else:
        valid = False
    if not valid:
        raise Phase2BuildError("refusing command outside the offline tipdraw allowlist")


def _run_logged(
    command: Sequence[str],
    log_path: Path,
    timeout_s: float,
    *,
    env: Mapping[str, str],
) -> CommandResult:
    _validate_offline_tipdraw_command(command)
    try:
        completed = subprocess.run(
            list(command),
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
            env=dict(env),
        )
        result = CommandResult(completed.returncode, completed.stdout, completed.stderr)
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout if isinstance(error.stdout, str) else ""
        stderr = error.stderr if isinstance(error.stderr, str) else ""
        result = CommandResult(124, stdout, stderr + f"\ncommand timed out after {timeout_s}s")
    except OSError as error:
        result = CommandResult(127, "", str(error))
    _append_log(log_path, command, result)
    return result


def _ros_cli(
    command: Sequence[str],
    *,
    env: Mapping[str, str],
    timeout_s: float = 5.0,
) -> CommandResult:
    try:
        result = subprocess.run(
            list(command),
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
            env=dict(env),
        )
    except subprocess.TimeoutExpired as error:
        raise Phase2BuildError(f"ROS discovery query timed out: {' '.join(command)}") from error
    except OSError as error:
        raise Phase2BuildError(f"cannot execute ROS discovery query: {error}") from error
    if result.returncode != 0:
        raise Phase2BuildError(
            f"ROS discovery query failed closed ({result.returncode}): "
            f"{' '.join(command)}: {result.stderr.strip()}"
        )
    return CommandResult(result.returncode, result.stdout, result.stderr)


def _action_server_count(action_name: str, *, env: Mapping[str, str]) -> int:
    # Jazzy's ros2action verbs unconditionally use the global XMLRPC daemon and
    # do not expose NodeStrategy's --no-daemon flag. That can return graph data
    # from another ROS_DOMAIN_ID. The two permitted action servers are owned by
    # fixed nodes in our isolated simulation, so discover those nodes directly
    # with the daemon-free node verb instead.
    owner_by_action = {
        "/hx_waypoint/build_recording": "/hx_waypoint_runtime",
        "/joint_trajectory_controller/follow_joint_trajectory": (
            "/joint_trajectory_controller"
        ),
    }
    owner = owner_by_action.get(action_name)
    if owner is None:
        raise Phase2BuildError(f"unsupported mock action identity: {action_name}")
    result = _ros_cli(
        ["ros2", "node", "list", "--no-daemon", "--spin-time", "1.0"],
        env=env,
    )
    node_names = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return node_names.count(owner)


def _dedicated_ros_environment(
    domain_id: int,
    curobo_source_root: Path,
) -> dict[str, str]:
    environment = dict(os.environ)
    for name in ("ROS_DISCOVERY_SERVER", "ROS_STATIC_PEERS"):
        environment.pop(name, None)
    environment.update(
        {
            "ROS_DOMAIN_ID": str(domain_id),
            "ROS2CLI_NO_DAEMON": "1",
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        }
    )
    inherited_pythonpath = environment.get("PYTHONPATH", "")
    pythonpath_parts = [str(curobo_source_root)]
    if inherited_pythonpath:
        pythonpath_parts.append(inherited_pythonpath)
    environment["PYTHONPATH"] = os.pathsep.join(pythonpath_parts)
    return environment


def _select_ros_domain(requested: int | None) -> int:
    caller_raw = os.environ.get("ROS_DOMAIN_ID", "0")
    try:
        caller_domain = int(caller_raw)
    except ValueError:
        caller_domain = 0
    if requested is not None:
        if not 0 <= requested <= 232:
            raise Phase2BuildError("--ros-domain-id must be between 0 and 232")
        if requested == caller_domain:
            raise Phase2BuildError(
                "dedicated mock ROS domain must differ from the caller's ROS_DOMAIN_ID"
            )
        return requested
    candidates = [value for value in range(101, 233) if value != caller_domain]
    return candidates[secrets.randbelow(len(candidates))]


def _assert_empty_ros_domain(*, env: Mapping[str, str]) -> None:
    # Two daemon-free snapshots reduce the chance of accepting a graph that is
    # still being discovered.  A dedicated random domain makes collision with
    # the user's real UR graph independently unlikely.
    for _ in range(2):
        nodes = _ros_cli(
            ["ros2", "node", "list", "--no-daemon", "--spin-time", "0.5"],
            env=env,
        )
        node_names = [line.strip() for line in nodes.stdout.splitlines() if line.strip()]
        if node_names:
            raise Phase2BuildError(
                "dedicated ROS domain is not empty; refusing any chance of real-controller "
                f"cross-talk (nodes={node_names})"
            )
        time.sleep(0.25)


def _stop_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    for signal_number, timeout_s in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0)):
        try:
            os.killpg(process.pid, signal_number)
            process.wait(timeout=timeout_s)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            continue
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


@contextmanager
def _simulation_session(
    paths: StoragePaths,
    launch_log_path: Path,
    launch_timeout_s: float,
    *,
    ros_environment: Mapping[str, str],
) -> Iterator[None]:
    _assert_empty_ros_domain(env=ros_environment)
    command = [
        "ros2",
        "launch",
        "hx_waypoint_bringup",
        "simulation.launch.py",
        "ur_type:=ur5",
        f"environment_id:={PHASE2_ENVIRONMENT_ID}",
        f"workspace:={paths.workspace}",
        "launch_rviz:=false",
    ]
    launch_stream: TextIO = launch_log_path.open("w", encoding="utf-8")
    try:
        launch_stream.write("$ " + " ".join(command) + "\n")
        launch_stream.flush()
        try:
            process = subprocess.Popen(
                command,
                stdout=launch_stream,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
                env=dict(ros_environment),
            )
        except OSError as error:
            raise Phase2BuildError(f"cannot launch isolated mock simulation: {error}") from error
        try:
            deadline = time.monotonic() + launch_timeout_s
            while time.monotonic() < deadline:
                return_code = process.poll()
                if return_code is not None:
                    raise Phase2BuildError(
                        "isolated mock simulation exited before BuildRecording became ready "
                        f"(exit={return_code}); see {launch_log_path}"
                    )
                server_count = _action_server_count(
                    "/hx_waypoint/build_recording", env=ros_environment
                )
                if server_count > 1:
                    raise Phase2BuildError(
                        "multiple BuildRecording servers appeared in the isolated simulation "
                        "domain; refusing ambiguous mock execution"
                    )
                if server_count == 1:
                    controller_count = _action_server_count(
                        "/joint_trajectory_controller/follow_joint_trajectory",
                        env=ros_environment,
                    )
                    if controller_count > 1:
                        raise Phase2BuildError(
                            "multiple trajectory-controller servers appeared in the isolated "
                            "simulation domain"
                        )
                    if controller_count == 1:
                        break
                time.sleep(1.0)
            else:
                raise Phase2BuildError(
                    f"isolated mock simulation was not ready within {launch_timeout_s}s; "
                    f"see {launch_log_path}"
                )
            yield
        finally:
            _stop_process_group(process)
    finally:
        launch_stream.close()


def _tipdraw_prefix(workspace: Path) -> list[str]:
    return ["ros2", "run", "hx_waypoint_tools", "tipdraw", "--workspace", str(workspace)]


def _inspect_recording(
    paths: StoragePaths,
    entry: AuthorityEntry,
    trajectory_id: str,
    recording_id: str,
    log_path: Path,
    timeout_s: float,
    *,
    ros_environment: Mapping[str, str],
    environment_hashes: Mapping[str, str],
) -> tuple[Path, str]:
    if _SAFE_ID_RE.fullmatch(recording_id) is None:
        raise Phase2BuildError(f"unsafe recording_id: {recording_id!r}")
    recording_path = paths.recordings_root / recording_id
    _require_real_directory(recording_path, "sealed recording")
    if recording_path.parent.resolve() != paths.recordings_root.resolve():
        raise Phase2BuildError("recording path escapes the canonical environment store")
    inspect_command = [
        *_tipdraw_prefix(paths.workspace),
        "inspect",
        "--env",
        PHASE2_ENVIRONMENT_ID,
        "--recording",
        recording_id,
    ]
    result = _run_logged(
        inspect_command,
        log_path,
        timeout_s,
        env=ros_environment,
    )
    if result.returncode != 0:
        raise Phase2BuildError(
            f"tipdraw inspect failed for {recording_id}; see {log_path}"
        )
    source_path = _require_regular_file(
        recording_path / "source_tip_trajectory.csv", "sealed source trajectory"
    )
    if _sha256_file(source_path) != entry.authority_sha256:
        raise Phase2BuildError(
            f"sealed source hash does not match authority for {entry.condition_id}"
        )
    manifest_path = recording_path / "manifest.yaml"
    manifest = _load_yaml(manifest_path, "sealed recording manifest")
    expected = {
        "status": "validated",
        "environment_id": PHASE2_ENVIRONMENT_ID,
        "trajectory_id": trajectory_id,
        "recording_id": recording_id,
    }
    if any(manifest.get(name) != value for name, value in expected.items()):
        raise Phase2BuildError(f"sealed recording manifest identity mismatch: {recording_id}")
    raw_manifest_hashes = _require_mapping(
        manifest.get("hashes"), "sealed recording manifest.hashes"
    )
    recorded_environment_hashes: dict[str, str] = {}
    for relative in environment_hashes:
        if relative not in raw_manifest_hashes:
            raise Phase2BuildError(
                f"sealed recording omits environment hash {relative}: {recording_id}"
            )
        recorded_environment_hashes[relative] = _normalize_sha256(
            raw_manifest_hashes[relative],
            f"sealed recording manifest.hashes.{relative}",
        )
    if recorded_environment_hashes != dict(environment_hashes):
        raise Phase2BuildError(
            f"sealed recording belongs to another environment revision: {recording_id}"
        )
    return recording_path, _sha256_file(manifest_path)


def _find_recoverable_recording(
    paths: StoragePaths,
    entry: AuthorityEntry,
    trajectory_id: str,
    log_path: Path,
    inspect_timeout_s: float,
    *,
    ros_environment: Mapping[str, str],
    environment_hashes: Mapping[str, str],
    excluded_recording_ids: frozenset[str] = frozenset(),
) -> tuple[Path, str] | None:
    candidates = sorted(
        (
            path
            for path in paths.recordings_root.glob(f"{trajectory_id}__*")
            if path.is_dir()
            and not path.is_symlink()
            and path.name not in excluded_recording_ids
        ),
        key=lambda path: path.name,
        reverse=True,
    )
    for candidate in candidates:
        try:
            return _inspect_recording(
                paths,
                entry,
                trajectory_id,
                candidate.name,
                log_path,
                inspect_timeout_s,
                ros_environment=ros_environment,
                environment_hashes=environment_hashes,
            )
        except Phase2BuildError:
            continue
    return None


def _event(
    bundle: AuthorityBundle,
    entry: AuthorityEntry,
    *,
    sequence: int,
    trajectory_id: str,
    build_key: str,
    status: str,
    ros_domain_id: int,
    log_path: Path,
    message: str,
    recording_path: Path | None = None,
    recording_manifest_sha256: str = "",
) -> dict[str, str]:
    return {
        "schema_version": "1",
        "event_sequence": str(sequence),
        "timestamp_utc": _utc_now(),
        "condition_id": entry.condition_id,
        "trajectory_id": trajectory_id,
        "authority_sha256": entry.authority_sha256,
        "authority_index_sha256": bundle.index_sha256,
        "input_conditions_sha256": bundle.conditions_sha256,
        "registration_sha256": bundle.registration_sha256,
        "environment_sha256": bundle.environment_sha256,
        "mode": bundle.mode,
        "ros_domain_id": str(ros_domain_id),
        "build_key": build_key,
        "status": status,
        "recording_id": recording_path.name if recording_path is not None else "",
        "recording_path": str(recording_path) if recording_path is not None else "",
        "recording_manifest_sha256": recording_manifest_sha256,
        "log_path": str(log_path),
        "message": message.replace("\n", " ").strip(),
    }


def _write_batch_manifest(
    paths: StoragePaths,
    bundle: AuthorityBundle,
    events: Sequence[Mapping[str, str]],
) -> None:
    latest_by_condition: dict[str, Mapping[str, str]] = {}
    for row in events:
        latest_by_condition[row["condition_id"]] = row
    expected_condition_ids = {entry.condition_id for entry in bundle.entries}
    latest_success = {
        condition_id: row
        for condition_id, row in latest_by_condition.items()
        if condition_id in expected_condition_ids
        and row.get("status") in {"success", "recovered"}
    }
    failure_count = sum(row.get("status") == "failed" for row in events)
    payload = {
        "schema_version": 1,
        "updated_utc": _utc_now(),
        "environment_id": PHASE2_ENVIRONMENT_ID,
        "mode": bundle.mode,
        "use_in_main_statistics": bundle.mode == "formal",
        "authority_index": {
            "path": str(bundle.index_path),
            "sha256": bundle.index_sha256,
        },
        "input_conditions_sha256": bundle.conditions_sha256,
        "registration_sha256": bundle.registration_sha256,
        "environment_sha256": bundle.environment_sha256,
        "canonical_compiler_runs_root": str(paths.compiler_runs_root),
        "canonical_recordings_root": str(paths.recordings_root),
        "batch_index": {
            "path": str(paths.index_path),
            "sha256": _sha256_file(paths.index_path),
        },
        "condition_count": len(bundle.entries),
        "successful_condition_count": len(latest_success),
        "failure_event_count": failure_count,
        "complete": len(latest_success) == len(bundle.entries),
        "hardware_motion_permitted": False,
        "commands_permitted": ["tipdraw build-recording", "tipdraw inspect"],
    }
    _atomic_write_bytes(
        paths.manifest_path,
        yaml.safe_dump(payload, sort_keys=False).encode(),
    )


def run_batch(args: argparse.Namespace) -> int:
    mode = "exploratory" if args.exploratory else "formal"
    args.ros_domain_id = _select_ros_domain(args.ros_domain_id)
    with _batch_locks(args, mode):
        return _run_batch_locked(args, mode)


def _run_batch_locked(args: argparse.Namespace, mode: str) -> int:
    paths, hashes_with_tree = _resolve_storage(args, mode)
    environment_sha256 = hashes_with_tree.pop("__tree_sha256__")
    index_path = Path(args.authority_index).expanduser().absolute()
    manifest_path = Path(
        args.authority_manifest or index_path.parent / AUTHORITY_MANIFEST_NAME
    ).expanduser().absolute()
    bundle = load_authority_bundle(
        index_path,
        manifest_path,
        expected_mode=mode,
        environment_id=args.environment_id,
        environment_hashes=hashes_with_tree,
        environment_sha256=environment_sha256,
    )
    events = _load_build_events(paths.index_path, bundle, paths)
    ros_environment = _dedicated_ros_environment(
        paths.ros_domain_id,
        paths.curobo_source_root,
    )
    if not paths.index_path.exists():
        _atomic_write_csv(paths.index_path, events)
        _write_batch_manifest(paths, bundle, events)

    pending: list[tuple[AuthorityEntry, str, str]] = []
    excluded_recordings: dict[str, set[str]] = {}
    for entry in bundle.entries:
        trajectory_id = _trajectory_id(entry)
        build_key = _build_key(bundle, entry, trajectory_id)
        successful = [
            row
            for row in reversed(events)
            if row["build_key"] == build_key and row["status"] in {"success", "recovered"}
        ]
        if successful:
            resume_log = paths.log_root / f"{trajectory_id}__resume.log"
            try:
                _assert_environment_unchanged(paths, bundle)
                _, current_manifest_sha256 = _inspect_recording(
                    paths,
                    entry,
                    trajectory_id,
                    successful[0]["recording_id"],
                    resume_log,
                    args.inspect_timeout,
                    ros_environment=ros_environment,
                    environment_hashes=bundle.environment_hashes,
                )
                if current_manifest_sha256 != successful[0]["recording_manifest_sha256"]:
                    raise Phase2BuildError(
                        "sealed recording manifest hash changed after it was indexed"
                    )
                print(f"SKIP {entry.condition_id}: validated same-hash recording exists")
                continue
            except Phase2BuildError as error:
                invalidated = _event(
                    bundle,
                    entry,
                    sequence=len(events) + 1,
                    trajectory_id=trajectory_id,
                    build_key=build_key,
                    status="failed",
                    ros_domain_id=paths.ros_domain_id,
                    log_path=resume_log,
                    message=f"previous success no longer validates: {error}",
                )
                events.append(invalidated)
                _atomic_write_csv(paths.index_path, events)
                _write_batch_manifest(paths, bundle, events)
                if successful[0]["recording_id"]:
                    excluded_recordings.setdefault(build_key, set()).add(
                        successful[0]["recording_id"]
                    )
        pending.append((entry, trajectory_id, build_key))

    if not pending:
        _write_batch_manifest(paths, bundle, events)
        print(f"All {len(bundle.entries)} same-hash recordings are already validated.")
        return 0

    print(f"Mode: {mode}")
    print(f"Compiler runs: {paths.compiler_runs_root}")
    print(f"Canonical recordings: {paths.recordings_root}")
    print(f"Batch index: {paths.index_path}")
    print(f"Logs: {paths.log_root}")
    print(f"Dedicated mock ROS_DOMAIN_ID: {paths.ros_domain_id}")

    failures = 0
    launch_log = paths.log_root / f"simulation__{datetime.now():%Y%m%d_%H%M%S}.log"
    with _simulation_session(
        paths,
        launch_log,
        args.launch_timeout,
        ros_environment=ros_environment,
    ):
        for ordinal, (entry, trajectory_id, build_key) in enumerate(pending, start=1):
            attempt = 1 + sum(row["build_key"] == build_key for row in events)
            log_path = paths.log_root / f"{trajectory_id}__attempt{attempt:02d}.log"
            print(f"[{ordinal}/{len(pending)}] {entry.condition_id}")
            try:
                _assert_environment_unchanged(paths, bundle)
                installed_path = paths.trajectories_root / f"{trajectory_id}.csv"
                _install_authority(entry, installed_path)
                recovered = _find_recoverable_recording(
                    paths,
                    entry,
                    trajectory_id,
                    log_path,
                    args.inspect_timeout,
                    ros_environment=ros_environment,
                    environment_hashes=bundle.environment_hashes,
                    excluded_recording_ids=frozenset(
                        excluded_recordings.get(build_key, set())
                    ),
                )
                status = "recovered"
                message = "recovered an already sealed same-hash recording"
                if recovered is None:
                    build_command = [
                        *_tipdraw_prefix(paths.workspace),
                        "build-recording",
                        "--env",
                        PHASE2_ENVIRONMENT_ID,
                        "--trajectory",
                        trajectory_id,
                        "--external-runtime",
                    ]
                    result = _run_logged(
                        build_command,
                        log_path,
                        args.build_timeout,
                        env=ros_environment,
                    )
                    if result.returncode != 0:
                        raise Phase2BuildError(
                            f"tipdraw build-recording failed with exit {result.returncode}; "
                            f"see {log_path}"
                        )
                    output_lines = [
                        line.strip()
                        for line in result.stdout.splitlines()
                        if line.strip()
                    ]
                    if not output_lines:
                        raise Phase2BuildError("build-recording returned no recording path")
                    returned_path = Path(output_lines[-1]).expanduser()
                    if not returned_path.is_absolute():
                        raise Phase2BuildError("build-recording returned a non-absolute path")
                    if returned_path.parent.resolve() != paths.recordings_root.resolve():
                        raise Phase2BuildError(
                            "build-recording returned a path outside the canonical store"
                        )
                    _assert_environment_unchanged(paths, bundle)
                    recovered = _inspect_recording(
                        paths,
                        entry,
                        trajectory_id,
                        returned_path.name,
                        log_path,
                        args.inspect_timeout,
                        ros_environment=ros_environment,
                        environment_hashes=bundle.environment_hashes,
                    )
                    status = "success"
                    message = "GPU compile, mock recording, and inspect succeeded"
                recording_path, recording_manifest_sha256 = recovered
                row = _event(
                    bundle,
                    entry,
                    sequence=len(events) + 1,
                    trajectory_id=trajectory_id,
                    build_key=build_key,
                    status=status,
                    ros_domain_id=paths.ros_domain_id,
                    log_path=log_path,
                    message=message,
                    recording_path=recording_path,
                    recording_manifest_sha256=recording_manifest_sha256,
                )
                events.append(row)
                print(f"  {status}: {recording_path.name}")
            except (OSError, Phase2BuildError) as error:
                failures += 1
                row = _event(
                    bundle,
                    entry,
                    sequence=len(events) + 1,
                    trajectory_id=trajectory_id,
                    build_key=build_key,
                    status="failed",
                    ros_domain_id=paths.ros_domain_id,
                    log_path=log_path,
                    message=str(error),
                )
                events.append(row)
                print(f"  FAILED: {error}", file=sys.stderr)
                _atomic_write_csv(paths.index_path, events)
                _write_batch_manifest(paths, bundle, events)
                if args.fail_fast:
                    break
                continue
            _atomic_write_csv(paths.index_path, events)
            _write_batch_manifest(paths, bundle, events)

    if failures:
        print(
            f"Completed with {failures} failed condition(s); rerun the same command to resume. "
            f"Index: {paths.index_path}",
            file=sys.stderr,
        )
        return 2
    print(f"All pending recordings validated. Index: {paths.index_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build/hash-resume ICASSP Phase-2 mock recordings (never moves hardware)."
    )
    parser.add_argument("--workspace", required=True, help="complete built ur-waypoint workspace")
    parser.add_argument("--authority-index", required=True, help="generator authority index CSV")
    parser.add_argument(
        "--authority-manifest",
        help=f"authority manifest (default: sibling {AUTHORITY_MANIFEST_NAME})",
    )
    parser.add_argument(
        "--environment-id",
        default=PHASE2_ENVIRONMENT_ID,
        help="must be the fixed Phase-2 environment ID",
    )
    parser.add_argument(
        "--exploratory",
        action="store_true",
        help="explicitly accept only isolated exploratory/preview authorities",
    )
    parser.add_argument(
        "--output-root",
        required=True,
        help="batch evidence base; a formal/ or exploratory/ child is always used",
    )
    parser.add_argument(
        "--log-root",
        help="log base (may be /data); mode-specific child is always used",
    )
    parser.add_argument(
        "--runs-root",
        help="optional assertion; must equal <workspace>/runs (no redirection)",
    )
    parser.add_argument(
        "--recordings-root",
        help="optional assertion; must equal the canonical environment recordings store",
    )
    parser.add_argument(
        "--ros-domain-id",
        type=int,
        help="dedicated empty mock-only DDS domain (default: random, never caller domain)",
    )
    parser.add_argument("--launch-timeout", type=float, default=180.0)
    parser.add_argument("--build-timeout", type=float, default=900.0)
    parser.add_argument("--inspect-timeout", type=float, default=120.0)
    parser.add_argument("--fail-fast", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    for name in ("launch_timeout", "build_timeout", "inspect_timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and greater than zero")
    try:
        return run_batch(args)
    except Phase2BuildError as error:
        print(f"build_icassp_phase2_recordings.py: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
