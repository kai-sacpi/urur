"""Fail-closed generation of ICASSP Phase-2 static rod-tip authorities.

The input condition position is the magnet centre in the array frame, not a
robot TCP target.  This module only emits a target after both required proper
SE(3) registrations have been supplied and validated.
"""

from __future__ import annotations

import csv
import hashlib
import io
import math
import os
import re
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.trajectory_io import SOURCE_COLUMNS


PHASE2_ENVIRONMENT_ID = "new_board_4cm_rod33365_up50_icassp_phase2"
PHASE2_FRAME_ID = "plane1"
PHASE2_TCP_FLOOR_Z_M = -0.072
PHASE2_SENSOR_PLANE_Z_M = -0.007
PHASE2_MINIMUM_TCP_DISTANCE_M = 0.065

_CONDITION_REQUIRED_COLUMNS = (
    "condition_id",
    "phase",
    "location_id",
    "direction_id",
    "p_A_x_m",
    "p_A_y_m",
    "p_A_z_m",
    "moment_q_A_x",
    "moment_q_A_y",
    "moment_q_A_z",
    "quat_A_M_x",
    "quat_A_M_y",
    "quat_A_M_z",
    "quat_A_M_w",
    "settle_s",
    "record_window_s",
    "status",
)

AUTHORITY_INDEX_COLUMNS = (
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
    "location_id",
    "direction_id",
    "moment_q_A_x",
    "moment_q_A_y",
    "moment_q_A_z",
    "quat_A_M_x",
    "quat_A_M_y",
    "quat_A_M_z",
    "quat_A_M_w",
    "frame_id",
    "tip_x_m",
    "tip_y_m",
    "tip_z_m",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
)

_CONDITION_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*\Z")
_PREVIEW_MARKERS = (
    "PREVIEW",
    "DO_NOT",
    "RECOMPUTE",
    "HELD_OUT",
    "NOMINAL",
)
_UNIT_TOLERANCE = 1.0e-8
_ROTATION_TOLERANCE = 1.0e-9


class Phase2GenerationError(ValueError):
    """The requested authority bundle is not safe or reproducible to emit."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Reject duplicate YAML keys in physical registration authorities."""


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
class Phase2GenerationResult:
    """Paths comprising one immutable formal or exploratory authority bundle."""

    mode: str
    mode_directory: Path
    authority_paths: tuple[Path, ...]
    index_path: Path
    manifest_path: Path


@dataclass(frozen=True)
class _Condition:
    condition_id: str
    phase: str
    status: str
    location_id: str
    direction_id: str
    position_A_m: tuple[float, float, float]
    moment_q_A: tuple[float, float, float]
    quaternion_A_M_xyzw: tuple[float, float, float, float]
    source_is_preview: bool


@dataclass(frozen=True)
class _Authority:
    condition: _Condition
    filename: str
    position_P_R_m: tuple[float, float, float]
    quaternion_P_R_xyzw: tuple[float, float, float, float]
    content: bytes


def sha256_bytes(content: bytes) -> str:
    """Return the lowercase SHA-256 digest of authoritative bytes."""

    return hashlib.sha256(content).hexdigest()


def environment_tree_sha256(hashes: Mapping[str, str]) -> str:
    """Digest an EnvironmentBuilder hash mapping with an unambiguous encoding."""

    digest = hashlib.sha256()
    for relative, file_digest in sorted(hashes.items()):
        if not isinstance(relative, str) or not relative or "\x00" in relative:
            raise Phase2GenerationError("environment hash path is not a safe non-empty string")
        if not isinstance(file_digest, str) or not re.fullmatch(
            r"[0-9a-f]{64}", file_digest
        ):
            raise Phase2GenerationError(
                f"environment hash for {relative} is not lowercase SHA-256"
            )
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(file_digest.encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _finite_float(raw: object, field: str) -> float:
    if isinstance(raw, bool):
        raise Phase2GenerationError(f"{field} must be a finite number, not boolean")
    try:
        value = float(raw)
    except (TypeError, ValueError) as error:
        raise Phase2GenerationError(f"{field} must be a finite number") from error
    if not math.isfinite(value):
        raise Phase2GenerationError(f"{field} must be a finite number")
    return value


def _required_text(raw: object, field: str) -> str:
    if not isinstance(raw, str) or not raw or raw.strip() != raw:
        raise Phase2GenerationError(f"{field} must be a non-empty trimmed string")
    return raw


def _parse_boolean(raw: object, field: str) -> bool:
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, int) and raw in (0, 1):
        return bool(raw)
    if isinstance(raw, str):
        normalized = raw.strip().lower()
        if normalized in {"1", "true"}:
            return True
        if normalized in {"0", "false"}:
            return False
    raise Phase2GenerationError(f"{field} must be an explicit true/false or 1/0")


def _normalized_vector(
    values: Sequence[float],
    field: str,
    width: int,
) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    if vector.shape != (width,) or not np.all(np.isfinite(vector)):
        raise Phase2GenerationError(f"{field} must contain {width} finite numbers")
    norm = float(np.linalg.norm(vector))
    if abs(norm - 1.0) > _UNIT_TOLERANCE:
        descriptor = "unit quaternion" if width == 4 else "unit vector"
        raise Phase2GenerationError(f"{field} must be a normalized {descriptor}")
    return vector / norm


def _quaternion_xyzw_to_rotation(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = _normalized_vector(quaternion, "quat_A_M", 4)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _rotation_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.asarray(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.asarray(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.asarray(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    quaternion = _normalized_vector(quaternion, "computed rod_tip quaternion", 4)
    # Quaternion sign is representational only.  Canonicalizing it makes the
    # authority byte-stable; the physical q sign and roll remain exactly those
    # encoded by quat_A_M and the frozen registration.
    for value in (quaternion[3], quaternion[0], quaternion[1], quaternion[2]):
        if abs(float(value)) > 1.0e-15:
            if value < 0.0:
                quaternion = -quaternion
            break
    return tuple(float(value) for value in quaternion)


def _parse_se3(raw: object, field: str) -> np.ndarray:
    if raw is None:
        raise Phase2GenerationError(f"{field} must be registered; null is forbidden")
    if not isinstance(raw, list) or len(raw) != 4:
        raise Phase2GenerationError(f"{field} must be a 4x4 proper SE(3) matrix")
    rows: list[list[float]] = []
    for row_index, row in enumerate(raw):
        if not isinstance(row, list) or len(row) != 4:
            raise Phase2GenerationError(f"{field} must be a 4x4 proper SE(3) matrix")
        rows.append(
            [
                _finite_float(value, f"{field}[{row_index}][{column_index}]")
                for column_index, value in enumerate(row)
            ]
        )
    matrix = np.asarray(rows, dtype=float)
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1.0e-12, rtol=0.0):
        raise Phase2GenerationError(f"{field} must have exact SE(3) homogeneous bottom row")
    rotation = matrix[:3, :3]
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=_ROTATION_TOLERANCE,
        rtol=0.0,
    ):
        raise Phase2GenerationError(f"{field} rotation must be orthonormal proper SE(3)")
    determinant = float(np.linalg.det(rotation))
    if abs(determinant - 1.0) > _ROTATION_TOLERANCE:
        raise Phase2GenerationError(
            f"{field} must be proper SE(3) with rotation determinant +1; got {determinant:.12g}"
        )
    return matrix


def _inverse_se3(transform: np.ndarray) -> np.ndarray:
    inverse = np.eye(4)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -transform[:3, :3].T @ transform[:3, 3]
    return inverse


def _load_registration(
    content: bytes,
    *,
    exploratory: bool,
) -> tuple[np.ndarray, np.ndarray, dict[str, str]]:
    try:
        data = yaml.load(content.decode("utf-8"), Loader=_UniqueKeyLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as error:
        raise Phase2GenerationError(f"cannot parse registration YAML: {error}") from error
    if not isinstance(data, dict) or any(not isinstance(key, str) for key in data):
        raise Phase2GenerationError(
            "registration YAML must contain a mapping with string keys"
        )
    if data.get("schema_version") != 1 or isinstance(data.get("schema_version"), bool):
        raise Phase2GenerationError("registration schema_version must be 1")
    if data.get("environment_id") != PHASE2_ENVIRONMENT_ID:
        raise Phase2GenerationError(
            f"registration environment_id must be {PHASE2_ENVIRONMENT_ID}"
        )
    if data.get("transform_convention") != "T_X_from_Y_column_vectors_metres":
        raise Phase2GenerationError(
            "registration transform_convention must be "
            "T_X_from_Y_column_vectors_metres"
        )
    # Parse both transforms before accepting any registration status.  This
    # keeps the checked-in null template visibly fail-closed at the missing
    # physical transform rather than letting metadata obscure that blocker.
    transform_P_A = _parse_se3(
        data.get("T_plane1_from_array"),
        "T_plane1_from_array",
    )
    transform_R_M = _parse_se3(
        data.get("T_rod_tip_from_magnet"),
        "T_rod_tip_from_magnet",
    )
    status = _required_text(data.get("status"), "registration.status")
    use_in_main = _parse_boolean(
        data.get("use_in_main_statistics"),
        "registration.use_in_main_statistics",
    )
    if status == "UNREGISTERED":
        raise Phase2GenerationError("registration status is UNREGISTERED")
    if exploratory and status not in {"EXPLORATORY", "HARDWARE_FROZEN"}:
        raise Phase2GenerationError(
            "exploratory generation requires EXPLORATORY or HARDWARE_FROZEN registration"
        )
    if status == "EXPLORATORY" and use_in_main:
        raise Phase2GenerationError(
            "EXPLORATORY registration must set use_in_main_statistics=false"
        )
    if status == "HARDWARE_FROZEN" and not use_in_main:
        raise Phase2GenerationError(
            "HARDWARE_FROZEN registration must set use_in_main_statistics=true"
        )
    if not exploratory and (status != "HARDWARE_FROZEN" or not use_in_main):
        raise Phase2GenerationError(
            "formal generation requires HARDWARE_FROZEN registration with "
            "use_in_main_statistics=true"
        )
    metadata = {
        field: _required_text(data.get(field), f"registration.{field}")
        for field in ("magnet_id", "polarity", "roll_policy")
    }
    return (
        transform_P_A,
        transform_R_M,
        {"status": status, **metadata},
    )


def _read_conditions(content: bytes, *, exploratory: bool) -> tuple[_Condition, ...]:
    try:
        text = content.decode("utf-8-sig")
    except UnicodeDecodeError as error:
        raise Phase2GenerationError("condition CSV must be UTF-8") from error
    reader = csv.DictReader(io.StringIO(text, newline=""))
    if reader.fieldnames is None:
        raise Phase2GenerationError("condition CSV is missing a header")
    if len(reader.fieldnames) != len(set(reader.fieldnames)):
        raise Phase2GenerationError("condition CSV header contains duplicate columns")
    missing = [field for field in _CONDITION_REQUIRED_COLUMNS if field not in reader.fieldnames]
    if missing:
        raise Phase2GenerationError(
            "condition CSV is missing required columns: " + ",".join(missing)
        )

    conditions: list[_Condition] = []
    identifiers: set[str] = set()
    portable_identifiers: set[str] = set()
    for line_number, row in enumerate(reader, start=2):
        if None in row:
            raise Phase2GenerationError(
                f"condition row {line_number} contains unrecognized extra values"
            )
        condition_id = _required_text(row["condition_id"], f"row {line_number} condition_id")
        if _CONDITION_ID_PATTERN.fullmatch(condition_id) is None:
            raise Phase2GenerationError(
                f"row {line_number} condition_id is not a safe authority filename"
            )
        if condition_id in identifiers:
            raise Phase2GenerationError(f"duplicate condition_id: {condition_id}")
        identifiers.add(condition_id)
        portable_identifier = condition_id.casefold()
        if portable_identifier in portable_identifiers:
            raise Phase2GenerationError(
                f"condition_id filename collision on case-insensitive filesystems: "
                f"{condition_id}"
            )
        portable_identifiers.add(portable_identifier)
        phase = _required_text(row["phase"], f"row {line_number} phase")
        status = _required_text(row["status"], f"row {line_number} status")
        source_is_preview = any(
            marker in f"{phase} {status}".upper() for marker in _PREVIEW_MARKERS
        )
        if source_is_preview and not exploratory:
            raise Phase2GenerationError(
                f"condition {condition_id} is preview/non-frozen; only explicit "
                "--exploratory may generate it"
            )
        if not exploratory:
            if phase != "hardware_frozen_main" or status != "HARDWARE_FROZEN":
                raise Phase2GenerationError(
                    f"condition {condition_id} is not explicitly hardware-frozen for formal use"
                )
            if "use_in_main_statistics" not in row or not _parse_boolean(
                row["use_in_main_statistics"],
                f"row {line_number} use_in_main_statistics",
            ):
                raise Phase2GenerationError(
                    f"condition {condition_id} must explicitly set use_in_main_statistics=true"
                )

        settle_s = _finite_float(row["settle_s"], f"row {line_number} settle_s")
        record_window_s = _finite_float(
            row["record_window_s"],
            f"row {line_number} record_window_s",
        )
        if settle_s != 1.0 or record_window_s != 2.0:
            raise Phase2GenerationError(
                f"condition {condition_id} must specify settle_s=1.0 and "
                "record_window_s=2.0 for the fixed [0,1)/[1,3) contract"
            )

        position = tuple(
            _finite_float(row[f"p_A_{axis}_m"], f"row {line_number} p_A_{axis}_m")
            for axis in "xyz"
        )
        moment = tuple(
            _finite_float(
                row[f"moment_q_A_{axis}"],
                f"row {line_number} moment_q_A_{axis}",
            )
            for axis in "xyz"
        )
        moment_vector = _normalized_vector(moment, f"row {line_number} moment_q_A", 3)
        quaternion = tuple(
            _finite_float(
                row[f"quat_A_M_{axis}"],
                f"row {line_number} quat_A_M_{axis}",
            )
            for axis in "xyzw"
        )
        rotation_A_M = _quaternion_xyzw_to_rotation(quaternion)
        if not np.allclose(
            rotation_A_M[:, 2],
            moment_vector,
            atol=5.0e-8,
            rtol=0.0,
        ):
            raise Phase2GenerationError(
                f"row {line_number} quat_A_M does not map +Z_M to moment_q_A"
            )
        conditions.append(
            _Condition(
                condition_id=condition_id,
                phase=phase,
                status=status,
                location_id=_required_text(
                    row["location_id"], f"row {line_number} location_id"
                ),
                direction_id=_required_text(
                    row["direction_id"], f"row {line_number} direction_id"
                ),
                position_A_m=position,
                moment_q_A=tuple(float(value) for value in moment_vector),
                quaternion_A_M_xyzw=tuple(
                    float(value)
                    for value in _normalized_vector(
                        quaternion,
                        f"row {line_number} quat_A_M",
                        4,
                    )
                ),
                source_is_preview=source_is_preview,
            )
        )
    if not conditions:
        raise Phase2GenerationError("condition CSV must contain at least one condition")
    return tuple(conditions)


def _render_csv(fieldnames: Sequence[str], rows: Sequence[Mapping[str, str]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def _render_authority(
    condition: _Condition,
    transform_P_A: np.ndarray,
    transform_R_M: np.ndarray,
    *,
    exploratory: bool,
) -> _Authority:
    transform_A_M = np.eye(4)
    transform_A_M[:3, :3] = _quaternion_xyzw_to_rotation(
        condition.quaternion_A_M_xyzw
    )
    transform_A_M[:3, 3] = condition.position_A_m
    transform_P_R = transform_P_A @ transform_A_M @ _inverse_se3(transform_R_M)
    rotation_P_R = transform_P_R[:3, :3]
    if not np.allclose(
        rotation_P_R.T @ rotation_P_R,
        np.eye(3),
        atol=_ROTATION_TOLERANCE,
        rtol=0.0,
    ) or abs(float(np.linalg.det(rotation_P_R)) - 1.0) > _ROTATION_TOLERANCE:
        raise Phase2GenerationError(
            f"computed rod_tip rotation for {condition.condition_id} is not proper SE(3)"
        )
    position = tuple(float(value) for value in transform_P_R[:3, 3])
    if position[2] > PHASE2_TCP_FLOOR_Z_M + 1.0e-12:
        raise Phase2GenerationError(
            f"condition {condition.condition_id} violates TCP floor: "
            f"z_rodtip={position[2]:.12f} m must be <= -0.072 m"
        )
    quaternion = _rotation_to_quaternion_xyzw(rotation_P_R)
    pose_values = {
        "frame_id": PHASE2_FRAME_ID,
        "tip_x_m": f"{position[0]:.9f}",
        "tip_y_m": f"{position[1]:.9f}",
        "tip_z_m": f"{position[2]:.9f}",
        "tip_qx": f"{quaternion[0]:.12f}",
        "tip_qy": f"{quaternion[1]:.12f}",
        "tip_qz": f"{quaternion[2]:.12f}",
        "tip_qw": f"{quaternion[3]:.12f}",
    }
    if float(pose_values["tip_z_m"]) > PHASE2_TCP_FLOOR_Z_M:
        raise Phase2GenerationError(
            f"condition {condition.condition_id} violates TCP floor after CSV encoding"
        )
    rows = []
    timestamps_and_phases = (
        ("0.000000000", "draw"),
        ("1.000000000", "hold"),
        ("3.000000000", "hold"),
    )
    for time_s, phase in timestamps_and_phases:
        rows.append({"time_s": time_s, **pose_values, "phase": phase})
    suffix = "__exploratory" if exploratory else ""
    return _Authority(
        condition=condition,
        filename=f"{condition.condition_id}{suffix}.csv",
        position_P_R_m=position,
        quaternion_P_R_xyzw=quaternion,
        content=_render_csv(SOURCE_COLUMNS, rows),
    )


def _index_row(
    authority: _Authority,
    *,
    conditions_sha256: str,
    registration_sha256: str,
    environment_sha256: str,
    mode: str,
) -> dict[str, str]:
    condition = authority.condition
    return {
        "condition_id": condition.condition_id,
        "authority_file": authority.filename,
        "authority_sha256": sha256_bytes(authority.content),
        "input_conditions_sha256": conditions_sha256,
        "registration_sha256": registration_sha256,
        "environment_id": PHASE2_ENVIRONMENT_ID,
        "environment_sha256": environment_sha256,
        "mode": mode,
        "use_in_main_statistics": "true" if mode == "formal" else "false",
        "source_phase": condition.phase,
        "source_status": condition.status,
        "location_id": condition.location_id,
        "direction_id": condition.direction_id,
        "moment_q_A_x": f"{condition.moment_q_A[0]:.12f}",
        "moment_q_A_y": f"{condition.moment_q_A[1]:.12f}",
        "moment_q_A_z": f"{condition.moment_q_A[2]:.12f}",
        "quat_A_M_x": f"{condition.quaternion_A_M_xyzw[0]:.12f}",
        "quat_A_M_y": f"{condition.quaternion_A_M_xyzw[1]:.12f}",
        "quat_A_M_z": f"{condition.quaternion_A_M_xyzw[2]:.12f}",
        "quat_A_M_w": f"{condition.quaternion_A_M_xyzw[3]:.12f}",
        "frame_id": PHASE2_FRAME_ID,
        "tip_x_m": f"{authority.position_P_R_m[0]:.9f}",
        "tip_y_m": f"{authority.position_P_R_m[1]:.9f}",
        "tip_z_m": f"{authority.position_P_R_m[2]:.9f}",
        "tip_qx": f"{authority.quaternion_P_R_xyzw[0]:.12f}",
        "tip_qy": f"{authority.quaternion_P_R_xyzw[1]:.12f}",
        "tip_qz": f"{authority.quaternion_P_R_xyzw[2]:.12f}",
        "tip_qw": f"{authority.quaternion_P_R_xyzw[3]:.12f}",
    }


def _yaml_bytes(data: Mapping[str, object]) -> bytes:
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=False).encode("utf-8")


def _require_existing_bundle_matches(
    mode_directory: Path,
    relative_contents: Mapping[Path, bytes],
) -> None:
    if mode_directory.is_symlink() or not mode_directory.is_dir():
        raise FileExistsError(
            f"refusing to overwrite non-directory authority bundle: {mode_directory}"
        )
    for relative, content in relative_contents.items():
        path = mode_directory / relative
        if (
            not path.exists()
            or path.is_symlink()
            or not path.is_file()
            or path.read_bytes() != content
        ):
            raise FileExistsError(
                f"refusing to overwrite different or partial authoritative content: {path}"
            )


def _write_staged_file(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())


def _publish_bundle_atomically(
    mode_directory: Path,
    relative_contents: Mapping[Path, bytes],
) -> None:
    if mode_directory.exists() or mode_directory.is_symlink():
        _require_existing_bundle_matches(mode_directory, relative_contents)
        return

    mode_directory.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(
            prefix=f".{mode_directory.name}.staging-",
            dir=mode_directory.parent,
        )
    )
    try:
        # Inputs and authorities are private in the staging directory.  The
        # index is written after them and the manifest last; the complete
        # directory becomes visible in one atomic rename.
        manifest_relative = Path("phase2_authority_manifest.yaml")
        index_relative = Path("phase2_authority_index.csv")
        regular_relatives = sorted(
            (
                relative
                for relative in relative_contents
                if relative not in {index_relative, manifest_relative}
            ),
            key=str,
        )
        for relative in regular_relatives:
            _write_staged_file(staging / relative, relative_contents[relative])
        _write_staged_file(staging / index_relative, relative_contents[index_relative])
        _write_staged_file(staging / manifest_relative, relative_contents[manifest_relative])
        try:
            os.rename(staging, mode_directory)
        except OSError as error:
            raise FileExistsError(
                f"could not atomically publish authority bundle without overwrite: "
                f"{mode_directory}"
            ) from error
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def generate_phase2_authorities(
    *,
    conditions_path: str | Path,
    registration_path: str | Path,
    environment_directory: str | Path,
    output_root: str | Path,
    expected_condition_count: int,
    exploratory: bool = False,
) -> Phase2GenerationResult:
    """Generate one immutable three-row static rod-tip authority per condition.

    Formal and exploratory outputs are structurally isolated below ``output_root``.
    Every output is precomputed and conflict-checked before any bytes are written.
    """

    if (
        isinstance(expected_condition_count, bool)
        or not isinstance(expected_condition_count, int)
        or expected_condition_count <= 0
    ):
        raise Phase2GenerationError("expected_condition_count must be a positive integer")
    condition_source = Path(conditions_path).expanduser().resolve()
    registration_source = Path(registration_path).expanduser().resolve()
    if not condition_source.is_file():
        raise Phase2GenerationError(f"condition CSV does not exist: {condition_source}")
    if not registration_source.is_file():
        raise Phase2GenerationError(
            f"experiment registration does not exist: {registration_source}"
        )
    conditions_content = condition_source.read_bytes()
    registration_content = registration_source.read_bytes()
    conditions = _read_conditions(conditions_content, exploratory=exploratory)
    if len(conditions) != expected_condition_count:
        raise Phase2GenerationError(
            f"condition count {len(conditions)} does not match expected "
            f"{expected_condition_count}"
        )
    transform_P_A, transform_R_M, registration_metadata = _load_registration(
        registration_content,
        exploratory=exploratory,
    )

    environment = EnvironmentBuilder().load(environment_directory)
    if environment.environment_id != PHASE2_ENVIRONMENT_ID:
        raise Phase2GenerationError(
            f"environment_id must be {PHASE2_ENVIRONMENT_ID}; got "
            f"{environment.environment_id}"
        )
    environment_hashes = dict(sorted(environment.hashes.items()))
    environment_sha256 = environment_tree_sha256(environment_hashes)
    conditions_sha256 = sha256_bytes(conditions_content)
    registration_sha256 = sha256_bytes(registration_content)
    mode = "exploratory" if exploratory else "formal"
    mode_directory = Path(output_root).expanduser().resolve() / mode

    authorities = tuple(
        _render_authority(
            condition,
            transform_P_A,
            transform_R_M,
            exploratory=exploratory,
        )
        for condition in conditions
    )
    index_rows = [
        _index_row(
            authority,
            conditions_sha256=conditions_sha256,
            registration_sha256=registration_sha256,
            environment_sha256=environment_sha256,
            mode=mode,
        )
        for authority in authorities
    ]
    index_content = _render_csv(AUTHORITY_INDEX_COLUMNS, index_rows)
    manifest_data: dict[str, object] = {
        "schema_version": 1,
        "manifest_type": "icassp_phase2_authority_bundle",
        "environment_id": PHASE2_ENVIRONMENT_ID,
        "environment": {
            "id": PHASE2_ENVIRONMENT_ID,
            "tree_sha256": environment_sha256,
            "hashes": environment_hashes,
        },
        "mode": mode,
        "use_in_main_statistics": not exploratory,
        "source_contains_preview": any(
            condition.source_is_preview for condition in conditions
        ),
        "condition_count": len(conditions),
        "expected_condition_count": expected_condition_count,
        "frame_id": PHASE2_FRAME_ID,
        "transform_formula": "T_P_R = T_P_A @ T_A_M @ inv(T_R_M)",
        "pose_realization_policy": (
            "quat_A_M_q_sign_and_roll_are_authoritative_no_runtime_changes"
        ),
        "tcp_floor": {
            "sensor_plane_z_m": PHASE2_SENSOR_PLANE_Z_M,
            "minimum_distance_m": PHASE2_MINIMUM_TCP_DISTANCE_M,
            "maximum_rodtip_z_m": PHASE2_TCP_FLOOR_Z_M,
        },
        "input_conditions": {
            "path": "inputs/conditions.csv",
            "sha256": conditions_sha256,
        },
        "registration": {
            "path": "inputs/registration.yaml",
            "sha256": registration_sha256,
            **registration_metadata,
        },
        "authority_index": {
            "path": "phase2_authority_index.csv",
            "sha256": sha256_bytes(index_content),
        },
        "entries": [
            {
                "condition_id": authority.condition.condition_id,
                "authority_file": authority.filename,
                "authority_sha256": sha256_bytes(authority.content),
                "mode": mode,
                "use_in_main_statistics": not exploratory,
            }
            for authority in authorities
        ],
    }
    manifest_content = _yaml_bytes(manifest_data)

    relative_contents: dict[Path, bytes] = {
        Path("inputs/conditions.csv"): conditions_content,
        Path("inputs/registration.yaml"): registration_content,
        Path("phase2_authority_index.csv"): index_content,
        Path("phase2_authority_manifest.yaml"): manifest_content,
    }
    portable_bundle_paths = {
        relative.as_posix().casefold() for relative in relative_contents
    }
    for authority in authorities:
        relative = Path(authority.filename)
        portable_relative = relative.as_posix().casefold()
        if portable_relative in portable_bundle_paths:
            raise Phase2GenerationError(
                f"condition {authority.condition.condition_id} collides with a reserved "
                "authority bundle path"
            )
        portable_bundle_paths.add(portable_relative)
        relative_contents[relative] = authority.content
    _publish_bundle_atomically(mode_directory, relative_contents)

    return Phase2GenerationResult(
        mode=mode,
        mode_directory=mode_directory,
        authority_paths=tuple(mode_directory / item.filename for item in authorities),
        index_path=mode_directory / "phase2_authority_index.csv",
        manifest_path=mode_directory / "phase2_authority_manifest.yaml",
    )
