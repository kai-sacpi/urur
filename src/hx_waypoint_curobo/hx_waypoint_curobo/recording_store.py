from __future__ import annotations

import csv
import hashlib
import io
import math
import os
import re
import shutil
import stat
import tempfile
from bisect import bisect_left
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime
from decimal import Decimal, InvalidOperation
from pathlib import Path
from pathlib import PurePosixPath
from typing import Any

import yaml

from .environment_builder import (
    REPRODUCIBLE_TREE_DIRECTORIES,
    EnvironmentBundle,
    ROBOT_DATA_FILES,
    ROBOT_CONFIG_FILES,
    ROOT_CONFIG_FILES,
)
from .errors import TrajectoryFormatError, ValidationError
from .models import CompiledTrajectory, TimedPose
from .robot_kinematics import (
    Pose,
    QuinticHermiteSegment,
    RobotKinematics,
    compose_poses,
    pose_error,
)
from .setup_config import SetupConfigurationError, parse_setup
from .trajectory_io import (
    joint_trajectory_columns,
    read_joint_trajectory_csv,
    read_timed_pose_csv,
    sha256_file,
)


REQUIRED_RECORDING_FILES = (
    "manifest.yaml",
    "source_tip_trajectory.csv",
    "joint_trajectory.csv",
    "joint_frames.csv",
    "sim_controller_state.csv",
    "sim_actual_joint.csv",
    "sim_actual_tip.csv",
    "compile_report.yaml",
    "validation_report.yaml",
    "sha256sums.txt",
)

_MANIFEST_SCHEMA_VERSION = 1
_COMPILE_REPORT_SCHEMA_VERSION = 2
_COMPILER_CONTRACT_REVISION = "fixed_time_waypoint_v2"
_JOINT_WINDING_POLICY = "constant_whole_path_nearest_home_without_margin_loss"
_SIMULATION_EXECUTION_CONTRACT_REVISION = 1
_SIMULATION_SPEED_SCALING_EVIDENCE = (
    "locked_standard_unscaled_joint_trajectory_controller"
)
_SIMULATION_CONTROLLER_NAME = "joint_trajectory_controller"
_SIMULATION_CONTROLLER_TYPE = (
    "joint_trajectory_controller/JointTrajectoryController"
)
_SIMULATION_HARDWARE_TYPE = "system"
_SIMULATION_HARDWARE_PLUGIN = "mock_components/GenericSystem"
_ACTIVE_LIFECYCLE_STATE_ID = 3
_SIMULATION_SPEED_SCALING_TOLERANCE = 1.0e-9
_TIME_TOLERANCE = 1.0e-6
_CANONICAL_DERIVATIVE_TOLERANCE = 1.0e-9
_CONTINUOUS_LIMIT_TOLERANCE = 1.0e-9
_AUDIT_METRIC_RELATIVE_TOLERANCE = 1.0e-6
# acos(dot(q1, q2)) is ill-conditioned at dot≈1.  Twelve-decimal telemetry
# serialization can therefore turn the same sub-arcsecond orientation into
# either exactly 0 or a few 1e-5 degrees across the C++ writer and Python
# inspector.  This tolerance is only for comparing two audit reports; the
# configured 0.1/0.5 degree safety limits remain unchanged and are rechecked.
_ORIENTATION_AUDIT_ABSOLUTE_TOLERANCE_DEG = 5.0e-5
_CUROBO_FEASIBILITY_COST_TOLERANCE = 1.0e-7
_SHA256_PATTERN = re.compile(r"[0-9a-fA-F]{64}\Z")
_GIT_COMMIT_PATTERN = re.compile(r"[0-9a-fA-F]{40}\Z")
_PROVENANCE_HASH_KEYS = {
    "environment_tree",
    "source_trajectory",
    "joint_trajectory",
    "curobo_revision",
}
_REQUIRED_ENVIRONMENT_FILES = (
    *ROOT_CONFIG_FILES,
    *ROBOT_CONFIG_FILES,
    *ROBOT_DATA_FILES,
)
_NANOSECONDS_PER_SECOND = Decimal(1_000_000_000)
_MAXIMUM_DURATION_NANOSECONDS = Decimal(2_147_483_647_999_999_999)
_CAPTURED_RECORDING_FILES = frozenset(
    {
        "manifest.yaml",
        "source_tip_trajectory.csv",
        "joint_trajectory.csv",
        "compile_report.yaml",
        "validation_report.yaml",
        "joint_frames.csv",
        "sim_controller_state.csv",
        "sim_actual_joint.csv",
        "sim_actual_tip.csv",
        "environment_snapshot/environment.yaml",
        "environment_snapshot/planner.yaml",
        "environment_snapshot/recording.yaml",
        "environment_snapshot/setup.yaml",
        "environment_snapshot/robot/asset_provenance.yaml",
        "environment_snapshot/robot/curobo_robot.yaml",
        "environment_snapshot/robot/robot_model.yaml",
    }
)
_EXPECTED_JOINT_FRAME_IDS = frozenset(
    {
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "tool0",
        "rod_tip",
    }
)
_STRICT_CSV_NUMBER_PATTERN = re.compile(
    r"[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?\Z"
)


class _UniqueKeySafeLoader(yaml.SafeLoader):
    """Safe YAML loader that fails closed on duplicate mapping keys."""


def _construct_unique_mapping(
    loader: _UniqueKeySafeLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
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
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _normalized_sha256(value: object, label: str) -> str:
    if not isinstance(value, str):
        raise ValidationError(f"{label} must be a SHA-256 string")
    normalized = value.strip().lower()
    if normalized.startswith("sha256:"):
        normalized = normalized.removeprefix("sha256:")
    if _SHA256_PATTERN.fullmatch(normalized) is None:
        raise ValidationError(f"{label} is not a valid SHA-256 digest")
    return normalized


def _normalized_git_commit(value: object, label: str) -> str:
    if not isinstance(value, str):
        raise ValidationError(f"{label} must be a Git commit string")
    normalized = value.strip().lower()
    if _GIT_COMMIT_PATTERN.fullmatch(normalized) is None:
        raise ValidationError(f"{label} must be a full 40-character Git commit")
    return normalized


def _safe_relative_path(value: object, label: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or "\\" in value
        or "\0" in value
    ):
        raise ValidationError(f"unsafe path in {label}: {value!r}")
    relative = PurePosixPath(value)
    normalized = relative.as_posix()
    if (
        relative.is_absolute()
        or normalized in {"", "."}
        or any(component in {"", ".", ".."} for component in value.split("/"))
        or normalized != value
    ):
        raise ValidationError(f"unsafe path in {label}: {value}")
    return normalized


def _require_mapping(value: object, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValidationError(f"{label} must be a mapping with string keys")
    return value


def _require_string(mapping: Mapping[str, Any], key: str, label: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f"{label}.{key} must be a non-empty string")
    return value.strip()


def _require_bool(mapping: Mapping[str, Any], key: str, label: str) -> bool:
    value = mapping.get(key)
    if not isinstance(value, bool):
        raise ValidationError(f"{label}.{key} must be a boolean")
    return value


def _require_int(mapping: Mapping[str, Any], key: str, label: str) -> int:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValidationError(f"{label}.{key} must be an integer")
    return value


def _require_float(mapping: Mapping[str, Any], key: str, label: str) -> float:
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"{label}.{key} must be numeric")
    try:
        parsed = float(value)
    except (OverflowError, ValueError) as error:
        raise ValidationError(f"{label}.{key} must be finite") from error
    if not math.isfinite(parsed):
        raise ValidationError(f"{label}.{key} must be finite")
    return parsed


def _require_string_sequence(
    mapping: Mapping[str, Any],
    key: str,
    label: str,
) -> tuple[str, ...]:
    value = mapping.get(key)
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or not value
        or any(not isinstance(item, str) or not item.strip() for item in value)
    ):
        raise ValidationError(f"{label}.{key} must be a non-empty string sequence")
    return tuple(item.strip() for item in value)


def _require_finite_sequence(
    mapping: Mapping[str, Any],
    key: str,
    expected_size: int,
    label: str,
) -> tuple[float, ...]:
    value = mapping.get(key)
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != expected_size
    ):
        raise ValidationError(
            f"{label}.{key} must contain exactly {expected_size} values"
        )
    parsed: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValidationError(f"{label}.{key} contains a non-numeric value")
        try:
            number = float(item)
        except (OverflowError, ValueError) as error:
            raise ValidationError(
                f"{label}.{key} contains a non-finite value"
            ) from error
        if not math.isfinite(number):
            raise ValidationError(f"{label}.{key} contains a non-finite value")
        parsed.append(number)
    return tuple(parsed)


def _float_matches(actual: float, expected: float) -> bool:
    tolerance = 1.0e-12 * max(1.0, abs(actual), abs(expected))
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def _audit_metric_matches(
    actual: float,
    expected: float,
    *,
    absolute_tolerance: float = 1.0e-6,
    relative_tolerance: float = _AUDIT_METRIC_RELATIVE_TOLERANCE,
) -> bool:
    tolerance = absolute_tolerance + relative_tolerance * max(
        abs(actual),
        abs(expected),
    )
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def _orientation_audit_metric_matches(actual: float, expected: float) -> bool:
    return _audit_metric_matches(
        actual,
        expected,
        absolute_tolerance=_ORIENTATION_AUDIT_ABSOLUTE_TOLERANCE_DEG,
    )


def _time_to_nanoseconds(value: object, label: str) -> int:
    try:
        decimal_time = Decimal(str(value))
    except (InvalidOperation, ValueError) as error:
        raise ValidationError(f"{label} is not a finite decimal timestamp") from error
    if not decimal_time.is_finite() or decimal_time < 0:
        raise ValidationError(f"{label} must be finite and non-negative")
    nanoseconds = decimal_time * _NANOSECONDS_PER_SECOND
    integral = nanoseconds.to_integral_value()
    if nanoseconds != integral or integral > _MAXIMUM_DURATION_NANOSECONDS:
        raise ValidationError(
            f"{label} must be exactly representable as a ROS nanosecond duration"
        )
    return int(integral)


def _seconds_to_nanoseconds(value: float, label: str) -> int:
    if not math.isfinite(value) or value < 0.0:
        raise ValidationError(f"{label} must be finite and non-negative")
    nanoseconds = value * 1.0e9
    rounded = round(nanoseconds)
    if (
        abs(nanoseconds - rounded) > 1.0e-3
        or rounded > int(_MAXIMUM_DURATION_NANOSECONDS)
    ):
        raise ValidationError(
            f"{label} must be exactly representable as a ROS nanosecond duration"
        )
    return int(rounded)


def _load_yaml_mapping(data: bytes, label: str) -> Mapping[str, Any]:
    try:
        document = yaml.load(data.decode("utf-8"), Loader=_UniqueKeySafeLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as error:
        raise ValidationError(f"cannot parse sealed {label}: {error}") from error
    return _require_mapping(document, label)


def _atomic_yaml(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(dict(data), stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def _tree_hash(root: Path, relative_paths: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in sorted(relative_paths, key=lambda item: str(item)):
        digest.update(str(path).encode("utf-8"))
        digest.update(b"\0")
        digest.update((root / path).read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


@dataclass(frozen=True)
class RecordingInspection:
    path: Path
    manifest: Mapping[str, Any]
    trajectory: CompiledTrajectory


@dataclass(frozen=True)
class _FileSeal:
    sha256: str
    device: int
    inode: int
    mode: int
    size: int
    mtime_ns: int
    ctime_ns: int


@dataclass(frozen=True)
class _ArtifactSnapshot:
    declared_hashes: Mapping[str, str]
    checksum_bytes: bytes
    directories: frozenset[str]
    files: Mapping[str, _FileSeal]
    captured_bytes: Mapping[str, bytes]

    def same_artifact(self, other: _ArtifactSnapshot) -> bool:
        return (
            self.declared_hashes == other.declared_hashes
            and self.checksum_bytes == other.checksum_bytes
            and self.directories == other.directories
            and self.files == other.files
        )


@dataclass(frozen=True)
class _SimulationValidationConfig:
    sample_dt_s: float
    max_missing_samples: int
    max_time_jitter_s: float
    max_quaternion_norm_error: float
    require_action_success: bool
    max_completion_overrun_s: float
    max_controller_reference_position_error_rad: float
    max_controller_reference_velocity_error_rad_s: float
    max_joint_position_tracking_error_rad: float
    max_joint_velocity_tracking_error_rad_s: float
    max_actual_controller_position_error_rad: float
    max_actual_controller_velocity_error_rad_s: float
    max_frame_fk_position_error_m: float
    max_frame_fk_orientation_error_deg: float


@dataclass(frozen=True)
class _RobotAuditContext:
    kinematics: RobotKinematics
    base_link: str
    tool_frame: str
    tip_frame: str
    tool_to_tip: Pose
    frame_to_base: Mapping[str, Pose]
    velocity_limits_rad_s: tuple[float, ...]
    acceleration_limits_rad_s2: tuple[float, ...]
    jerk_limits_rad_s3: tuple[float, ...]


@dataclass(frozen=True)
class _CanonicalAuditMetrics:
    max_joint_velocity_ratio: float
    max_joint_acceleration_ratio: float
    max_joint_jerk_ratio: float
    max_tip_position_error_m: float
    max_tip_orientation_error_deg: float


@dataclass(frozen=True)
class _SamplingMetrics:
    sample_count: int
    first_trajectory_time_s: float
    last_trajectory_time_s: float
    coverage_duration_s: float
    max_time_interval_s: float
    max_time_jitter_s: float
    missing_sample_estimate: int


@dataclass(frozen=True)
class _PoseMetrics:
    sampling: _SamplingMetrics
    max_quaternion_norm_error: float


@dataclass(frozen=True)
class _SimulationMetrics:
    actual_joint: _SamplingMetrics
    controller: _SamplingMetrics
    joint_frame_sample_count: int
    joint_frame_streams: Mapping[str, _PoseMetrics]
    actual_tip: _PoseMetrics
    max_joint_frame_quaternion_norm_error: float
    max_speed_scaling_error: float
    max_controller_reference_position_error_rad: float
    max_controller_reference_velocity_error_rad_s: float
    max_joint_position_tracking_error_rad: float
    max_joint_velocity_tracking_error_rad_s: float
    max_actual_controller_position_error_rad: float
    max_actual_controller_velocity_error_rad_s: float
    max_frame_fk_position_error_m: float
    max_frame_fk_orientation_error_deg: float
    max_tip_fk_position_error_m: float
    max_tip_fk_orientation_error_deg: float
    max_tip_source_position_error_m: float
    max_tip_source_orientation_error_deg: float


class RecordingBuilder:
    """Build in a staging directory and atomically publish only validated artifacts."""

    def __init__(
        self,
        environment: EnvironmentBundle,
        trajectory_id: str,
        timestamp: datetime | None = None,
    ) -> None:
        self.environment = environment
        safe_id = trajectory_id.strip()
        if not safe_id or any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for character in safe_id):
            raise ValidationError("trajectory_id may contain only letters, digits, underscore, and dash")
        stamp = (timestamp or datetime.now()).strftime("%Y%m%d_%H%M%S")
        self.recording_id = f"{safe_id}__{stamp}"
        self.recordings_root = environment.root / "recordings"
        self.staging_path = self.recordings_root / f".{self.recording_id}.staging"
        self.final_path = self.recordings_root / self.recording_id

    def begin(self) -> Path:
        self.recordings_root.mkdir(parents=True, exist_ok=True)
        if self.staging_path.exists() or self.final_path.exists():
            raise ValidationError(f"recording already exists: {self.recording_id}")
        self.staging_path.mkdir(parents=False)
        _atomic_yaml(
            self.staging_path / "manifest.yaml",
            {
                "schema_version": 1,
                "recording_id": self.recording_id,
                "status": "building",
                "environment_id": self.environment.environment_id,
                "trajectory_id": self.recording_id.rsplit("__", 1)[0],
            },
        )
        return self.staging_path

    def finalize(
        self,
        trajectory: CompiledTrajectory,
        validation: Mapping[str, Any],
        curobo_revision: str,
    ) -> Path:
        del trajectory, validation, curobo_revision
        raise ValidationError(
            "direct Python recording finalization is disabled; "
            "use the C++ BuildRecording action so simulation and safety gates cannot be bypassed"
        )

    def fail(self, message: str) -> None:
        if self.staging_path.is_dir():
            _atomic_yaml(
                self.staging_path / "manifest.yaml",
                {
                    "schema_version": 1,
                    "recording_id": self.recording_id,
                    "status": "failed",
                    "environment_id": self.environment.environment_id,
                    "message": message,
                },
            )

    def _snapshot_environment(self) -> Path:
        destination = self.staging_path / "environment_snapshot"
        destination.mkdir()
        for relative in (*ROOT_CONFIG_FILES, *ROBOT_CONFIG_FILES, *ROBOT_DATA_FILES):
            source = self.environment.root / relative
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
        for relative in REPRODUCIBLE_TREE_DIRECTORIES:
            source = self.environment.root / relative
            if source.is_dir():
                shutil.copytree(source, destination / relative, dirs_exist_ok=True)
        return destination

    def _write_checksums(self) -> None:
        checksum_path = self.staging_path / "sha256sums.txt"
        lines: list[str] = []
        for path in sorted(self.staging_path.rglob("*")):
            if not path.is_file() or path == checksum_path:
                continue
            lines.append(f"{sha256_file(path)}  {path.relative_to(self.staging_path)}")
        checksum_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _fsync_tree(self) -> None:
        for path in self.staging_path.rglob("*"):
            if path.is_file():
                with path.open("rb") as stream:
                    os.fsync(stream.fileno())
        descriptor = os.open(self.staging_path, os.O_RDONLY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


class RecordingInspector:
    @classmethod
    def audit_compiled_trajectory(
        cls,
        environment: EnvironmentBundle,
        trajectory: CompiledTrajectory,
        source: Sequence[TimedPose],
    ) -> _CanonicalAuditMetrics:
        """Recompute canonical metrics from the exact serialized trajectory payload."""

        if trajectory.joint_names != environment.joint_names:
            raise ValidationError(
                "canonical audit joint order does not match the environment"
            )
        generated = environment.asset_provenance.get("generated_urdf")
        if not isinstance(generated, Mapping):
            raise ValidationError("environment generated URDF provenance is invalid")
        relative_urdf = generated.get("path")
        if not isinstance(relative_urdf, str):
            raise ValidationError("environment generated URDF path is invalid")
        urdf_path = environment.root / relative_urdf
        try:
            kinematics = RobotKinematics.from_urdf_bytes(
                urdf_path.read_bytes(),
                base_link=environment.base_frame,
                joint_names=environment.joint_names,
            )
            configured_tool = environment.setup_config.tool
            tool_transform = configured_tool.parent_to_tip
            tool_to_tip = Pose(
                position_m=tool_transform.translation_m,
                quaternion_xyzw=tool_transform.quaternion_xyzw,
            )
            frame_to_base: dict[str, Pose] = {}
            for frame_id in {environment.base_frame, *(pose.frame_id for pose in source)}:
                transformed = environment.transform_pose_to_base(
                    TimedPose(
                        time_s=0.0,
                        frame_id=frame_id,
                        position_m=(0.0, 0.0, 0.0),
                        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                        phase="draw",
                    )
                )
                frame_to_base[frame_id] = Pose(
                    position_m=transformed.position_m,
                    quaternion_xyzw=transformed.quaternion_xyzw,
                )
            limits = environment.robot_model["robot"]["limits"]
            context = _RobotAuditContext(
                kinematics=kinematics,
                base_link=environment.base_frame,
                tool_frame=configured_tool.parent_link,
                tip_frame=configured_tool.tip_frame,
                tool_to_tip=tool_to_tip,
                frame_to_base=frame_to_base,
                velocity_limits_rad_s=tuple(
                    float(value) for value in limits["velocity_rad_s"]
                ),
                acceleration_limits_rad_s2=tuple(
                    float(value) for value in limits["acceleration_rad_s2"]
                ),
                jerk_limits_rad_s3=tuple(
                    float(value) for value in limits["jerk_rad_s3"]
                ),
            )
        except (KeyError, OSError, TypeError, ValueError) as error:
            raise ValidationError(
                f"environment cannot provide canonical audit context: {error}"
            ) from error
        return cls._audit_canonical_trajectory(trajectory, source, context)

    def inspect(self, path: str | Path) -> RecordingInspection:
        requested = Path(path).expanduser()
        directory_fd = self._open_recording_directory(requested)
        try:
            recording = self._resolve_open_recording_directory(requested, directory_fd)
            return self._inspect_open_recording(recording, directory_fd)
        finally:
            os.close(directory_fd)

    def _inspect_open_recording(
        self,
        recording: Path,
        directory_fd: int,
    ) -> RecordingInspection:
        snapshot = self._snapshot_artifact(
            directory_fd,
            capture_paths=_CAPTURED_RECORDING_FILES,
        )

        def captured(relative: str) -> bytes:
            try:
                return snapshot.captured_bytes[relative]
            except KeyError as error:
                raise ValidationError(
                    f"sealed recording snapshot is missing {relative}"
                ) from error

        manifest_bytes = captured("manifest.yaml")
        manifest = _load_yaml_mapping(manifest_bytes, "manifest.yaml")
        manifest_evidence = self._validate_manifest(manifest, recording)
        joint_names = manifest_evidence["joint_names"]
        manifest_hashes = manifest_evidence["hashes"]
        environment_hashes = manifest_evidence["environment_hashes"]

        source_bytes = captured("source_tip_trajectory.csv")
        joint_bytes = captured("joint_trajectory.csv")
        source_sha256 = _sha256_bytes(source_bytes)
        joint_sha256 = _sha256_bytes(joint_bytes)
        if manifest_hashes["source_trajectory"] != source_sha256:
            raise ValidationError(
                "manifest source_trajectory SHA-256 does not match the sealed CSV"
            )
        if manifest_hashes["joint_trajectory"] != joint_sha256:
            raise ValidationError(
                "manifest joint_trajectory SHA-256 does not match the sealed CSV"
            )

        compile_report = _load_yaml_mapping(
            captured("compile_report.yaml"),
            "compile_report.yaml",
        )
        validation_report = _load_yaml_mapping(
            captured("validation_report.yaml"),
            "validation_report.yaml",
        )

        for relative, expected_hash in environment_hashes.items():
            artifact_relative = f"environment_snapshot/{relative}"
            seal = snapshot.files.get(artifact_relative)
            if seal is None:
                raise ValidationError(
                    f"recording environment_snapshot/{relative} is missing"
                )
            if seal.size == 0:
                raise ValidationError(
                    f"recording environment_snapshot/{relative} must not be empty"
                )
            if seal.sha256 != expected_hash:
                raise ValidationError(
                    f"recording environment_snapshot hash mismatch for {relative}"
                )

        robot_model = _load_yaml_mapping(
            captured("environment_snapshot/robot/robot_model.yaml"),
            "environment_snapshot/robot/robot_model.yaml",
        )
        asset_provenance = _load_yaml_mapping(
            captured("environment_snapshot/robot/asset_provenance.yaml"),
            "environment_snapshot/robot/asset_provenance.yaml",
        )
        curobo_robot = _load_yaml_mapping(
            captured("environment_snapshot/robot/curobo_robot.yaml"),
            "environment_snapshot/robot/curobo_robot.yaml",
        )
        setup = _load_yaml_mapping(
            captured("environment_snapshot/setup.yaml"),
            "environment_snapshot/setup.yaml",
        )
        planner = _load_yaml_mapping(
            captured("environment_snapshot/planner.yaml"),
            "environment_snapshot/planner.yaml",
        )
        recording_config = _load_yaml_mapping(
            captured("environment_snapshot/recording.yaml"),
            "environment_snapshot/recording.yaml",
        )
        environment_document = _load_yaml_mapping(
            captured("environment_snapshot/environment.yaml"),
            "environment_snapshot/environment.yaml",
        )
        self._validate_snapshot_identity(
            environment_document,
            robot_model,
            manifest_evidence,
        )
        robot_audit = self._load_robot_audit_context(
            directory_fd,
            snapshot,
            manifest_evidence,
            robot_model,
            asset_provenance,
            curobo_robot,
            setup,
        )
        compile_evidence = self._validate_compile_report(
            compile_report,
            robot_model,
            manifest_evidence,
            source_sha256,
            joint_sha256,
        )

        joint_time_ns, joint_phases = self._parse_joint_timing(
            joint_bytes,
            joint_names,
        )
        try:
            with tempfile.TemporaryDirectory(prefix="hx_waypoint_inspect_") as temporary:
                temporary_root = Path(temporary)
                temporary_joint = temporary_root / "joint_trajectory.csv"
                temporary_source = temporary_root / "source_tip_trajectory.csv"
                temporary_joint.write_bytes(joint_bytes)
                temporary_source.write_bytes(source_bytes)
                trajectory = read_joint_trajectory_csv(temporary_joint, joint_names)
                source = read_timed_pose_csv(temporary_source)
        except (OSError, UnicodeError, csv.Error, TrajectoryFormatError) as error:
            raise ValidationError(f"sealed trajectory CSV is invalid: {error}") from error

        self._validate_canonical_trajectory(trajectory)
        source_time_ns = tuple(
            _seconds_to_nanoseconds(pose.time_s, "source waypoint time_s")
            for pose in source
        )
        source_phases = tuple(pose.phase for pose in source)
        matched_source_count = self._validate_authoritative_timing(
            source_time_ns,
            source_phases,
            joint_time_ns,
            joint_phases,
        )
        canonical_audit = self._audit_canonical_trajectory(
            trajectory,
            source,
            robot_audit,
        )
        self._reconcile_canonical_audit(
            canonical_audit,
            compile_evidence,
        )
        self._validate_actual_compile_evidence(
            compile_evidence,
            source_time_ns,
            joint_time_ns,
            len(trajectory.samples),
            matched_source_count,
        )
        self._validate_manifest_compile_evidence(
            manifest_evidence["trajectory"],
            compile_evidence,
        )
        self._validate_validation_evidence(
            validation_report,
            manifest_evidence["validation"],
            compile_evidence,
            manifest_evidence,
            recording_config,
            planner,
            trajectory,
            source,
            robot_audit,
            {
                "joint_frames.csv": captured("joint_frames.csv"),
                "sim_controller_state.csv": captured("sim_controller_state.csv"),
                "sim_actual_joint.csv": captured("sim_actual_joint.csv"),
                "sim_actual_tip.csv": captured("sim_actual_tip.csv"),
            },
        )
        self._validate_planner_policy(planner, compile_evidence)

        if len(trajectory.samples) != len(joint_time_ns):
            raise ValidationError(
                "joint trajectory parsed sample count differs from its sealed CSV rows"
            )
        manifest_duration = _require_float(
            manifest_evidence["trajectory"],
            "duration_s",
            "manifest.trajectory",
        )
        if abs(manifest_duration - trajectory.duration_s) > _TIME_TOLERANCE:
            raise ValidationError(
                "recording duration does not match joint_trajectory.csv"
            )
        manifest_nominal_dt = _require_float(
            manifest_evidence["trajectory"],
            "nominal_dt_s",
            "manifest.trajectory",
        )
        actual_nominal_dt = trajectory.samples[1].time_s - trajectory.samples[0].time_s
        if not _float_matches(manifest_nominal_dt, actual_nominal_dt):
            raise ValidationError(
                "recording nominal_dt_s does not match joint_trajectory.csv"
            )

        final_snapshot = self._snapshot_artifact(
            directory_fd,
            capture_paths=frozenset(),
        )
        if not snapshot.same_artifact(final_snapshot):
            raise ValidationError("recording artifact changed during inspection")
        self._require_open_directory_path_unchanged(recording, directory_fd)
        return RecordingInspection(
            path=recording,
            manifest=manifest,
            trajectory=trajectory,
        )

    @staticmethod
    def _directory_open_flags() -> int:
        no_follow = getattr(os, "O_NOFOLLOW", None)
        directory = getattr(os, "O_DIRECTORY", None)
        if no_follow is None or directory is None:
            raise ValidationError(
                "this platform cannot inspect recordings without following symlinks"
            )
        return (
            os.O_RDONLY
            | no_follow
            | directory
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NONBLOCK", 0)
        )

    @staticmethod
    def _file_open_flags() -> int:
        no_follow = getattr(os, "O_NOFOLLOW", None)
        if no_follow is None:
            raise ValidationError(
                "this platform cannot inspect recordings without following symlinks"
            )
        return (
            os.O_RDONLY
            | no_follow
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NONBLOCK", 0)
        )

    def _open_recording_directory(self, requested: Path) -> int:
        try:
            descriptor = os.open(requested, self._directory_open_flags())
        except OSError as error:
            raise ValidationError(
                "recording directory does not exist, is not a directory, or is a symlink: "
                f"{requested}"
            ) from error
        try:
            status = os.fstat(descriptor)
            if not stat.S_ISDIR(status.st_mode):
                raise ValidationError(
                    f"recording directory is not a directory: {requested}"
                )
        except BaseException:
            os.close(descriptor)
            raise
        return descriptor

    @staticmethod
    def _directory_identity(status: os.stat_result) -> tuple[int, int, int, int, int]:
        return (
            status.st_dev,
            status.st_ino,
            status.st_mode,
            status.st_mtime_ns,
            status.st_ctime_ns,
        )

    def _resolve_open_recording_directory(self, requested: Path, descriptor: int) -> Path:
        try:
            recording = requested.resolve(strict=True)
            path_status = recording.stat(follow_symlinks=False)
        except (OSError, RuntimeError) as error:
            raise ValidationError(
                f"recording directory does not exist: {requested}"
            ) from error
        descriptor_status = os.fstat(descriptor)
        if (
            not stat.S_ISDIR(path_status.st_mode)
            or self._directory_identity(path_status)
            != self._directory_identity(descriptor_status)
        ):
            raise ValidationError("recording directory changed while it was being opened")
        return recording

    def _require_open_directory_path_unchanged(
        self,
        recording: Path,
        descriptor: int,
    ) -> None:
        try:
            path_status = recording.stat(follow_symlinks=False)
        except OSError as error:
            raise ValidationError("recording directory changed during inspection") from error
        if self._directory_identity(path_status) != self._directory_identity(
            os.fstat(descriptor)
        ):
            raise ValidationError("recording directory changed during inspection")

    def _enumerate_artifact(
        self,
        recording_fd: int,
    ) -> tuple[set[str], set[str]]:
        files: set[str] = set()
        directories: set[str] = set()

        def visit(directory_fd: int, prefix: str) -> None:
            before = os.fstat(directory_fd)
            try:
                names = os.listdir(directory_fd)
            except OSError as error:
                raise ValidationError(
                    f"cannot enumerate recording artifacts under {prefix or '.'}"
                ) from error
            for name in names:
                if not isinstance(name, str) or name in {"", ".", ".."} or "/" in name:
                    raise ValidationError(
                        f"recording contains an unsafe directory entry: {name!r}"
                    )
                relative = f"{prefix}/{name}" if prefix else name
                try:
                    status = os.stat(
                        name,
                        dir_fd=directory_fd,
                        follow_symlinks=False,
                    )
                except OSError as error:
                    raise ValidationError(
                        f"cannot stat recording artifact {relative}"
                    ) from error
                if stat.S_ISLNK(status.st_mode):
                    raise ValidationError(
                        f"recording artifacts must not contain symlinks: {relative}"
                    )
                if stat.S_ISDIR(status.st_mode):
                    try:
                        child_fd = os.open(
                            name,
                            self._directory_open_flags(),
                            dir_fd=directory_fd,
                        )
                    except OSError as error:
                        raise ValidationError(
                            f"cannot open recording directory {relative}"
                        ) from error
                    try:
                        opened = os.fstat(child_fd)
                        if self._directory_identity(opened) != self._directory_identity(
                            status
                        ):
                            raise ValidationError(
                                f"recording directory changed during enumeration: {relative}"
                            )
                        directories.add(relative)
                        visit(child_fd, relative)
                    finally:
                        os.close(child_fd)
                    continue
                if not stat.S_ISREG(status.st_mode):
                    raise ValidationError(
                        "recording artifacts may contain only directories and regular files: "
                        f"{relative}"
                    )
                files.add(relative)
            after = os.fstat(directory_fd)
            if self._directory_identity(before) != self._directory_identity(after):
                raise ValidationError(
                    f"recording directory changed during enumeration: {prefix or '.'}"
                )

        visit(recording_fd, "")
        return files, directories

    def _open_relative_file(self, recording_fd: int, relative: str) -> int:
        normalized = _safe_relative_path(relative, "recording artifact")
        parts = PurePosixPath(normalized).parts
        parent_fd = os.dup(recording_fd)
        try:
            for component in parts[:-1]:
                next_fd = os.open(
                    component,
                    self._directory_open_flags(),
                    dir_fd=parent_fd,
                )
                os.close(parent_fd)
                parent_fd = next_fd
            return os.open(
                parts[-1],
                self._file_open_flags(),
                dir_fd=parent_fd,
            )
        except OSError as error:
            raise ValidationError(
                f"cannot open recording artifact {relative} without following symlinks"
            ) from error
        finally:
            os.close(parent_fd)

    @staticmethod
    def _file_seal(status: os.stat_result, digest: str) -> _FileSeal:
        return _FileSeal(
            sha256=digest,
            device=status.st_dev,
            inode=status.st_ino,
            mode=status.st_mode,
            size=status.st_size,
            mtime_ns=status.st_mtime_ns,
            ctime_ns=status.st_ctime_ns,
        )

    def _read_relative_file(
        self,
        recording_fd: int,
        relative: str,
        *,
        capture: bool,
    ) -> tuple[_FileSeal, bytes]:
        descriptor = self._open_relative_file(recording_fd, relative)
        try:
            before = os.fstat(descriptor)
            if not stat.S_ISREG(before.st_mode):
                raise ValidationError(
                    f"recording artifact must be a regular file: {relative}"
                )
            digest = hashlib.sha256()
            chunks: list[bytes] | None = [] if capture else None
            while True:
                chunk = os.read(descriptor, 1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
                if chunks is not None:
                    chunks.append(chunk)
            after = os.fstat(descriptor)
            before_seal = self._file_seal(before, digest.hexdigest())
            after_seal = self._file_seal(after, digest.hexdigest())
            if before_seal != after_seal:
                raise ValidationError(
                    f"recording artifact changed while being read: {relative}"
                )
            return before_seal, b"" if chunks is None else b"".join(chunks)
        except OSError as error:
            raise ValidationError(f"cannot read recording artifact {relative}") from error
        finally:
            os.close(descriptor)

    def _require_relative_file_identity(
        self,
        recording_fd: int,
        relative: str,
        expected: _FileSeal,
    ) -> None:
        descriptor = self._open_relative_file(recording_fd, relative)
        try:
            status = os.fstat(descriptor)
            actual = self._file_seal(status, expected.sha256)
            if actual != expected:
                raise ValidationError(
                    f"recording artifact changed during inspection: {relative}"
                )
        finally:
            os.close(descriptor)

    @staticmethod
    def _parse_checksum_bytes(
        checksum_bytes: bytes,
        artifact_files: set[str],
    ) -> dict[str, str]:
        try:
            lines = checksum_bytes.decode("utf-8").splitlines()
        except UnicodeError as error:
            raise ValidationError("sha256sums.txt is not valid UTF-8") from error
        declared: dict[str, str] = {}
        for line_number, raw_line in enumerate(lines, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if len(line) < 66:
                raise ValidationError(f"invalid checksum row {line_number}")
            expected_token = line[:64]
            if _SHA256_PATTERN.fullmatch(expected_token) is None:
                raise ValidationError(
                    f"invalid SHA-256 digest in checksum row {line_number}"
                )
            expected = expected_token.lower()
            relative_text = line[64:].strip()
            if relative_text.startswith("*"):
                relative_text = relative_text[1:]
            relative = _safe_relative_path(
                relative_text,
                f"sha256sums.txt row {line_number}",
            )
            if relative == "sha256sums.txt":
                raise ValidationError("sha256sums.txt must not checksum itself")
            if relative in declared:
                raise ValidationError(
                    f"sha256sums.txt contains a duplicate path: {relative}"
                )
            if relative not in artifact_files:
                raise ValidationError(
                    "sha256sums.txt references a missing or non-regular artifact: "
                    f"{relative}"
                )
            declared[relative] = expected
        unprotected = sorted(artifact_files.difference(declared))
        if unprotected:
            raise ValidationError(
                "sha256sums.txt does not protect every recording file; missing: "
                f"{unprotected[0]}"
            )
        return declared

    def _snapshot_artifact(
        self,
        recording_fd: int,
        *,
        capture_paths: frozenset[str],
    ) -> _ArtifactSnapshot:
        all_files, directories = self._enumerate_artifact(recording_fd)
        if "sha256sums.txt" not in all_files:
            raise ValidationError("recording is missing files: sha256sums.txt")
        if "environment_snapshot" not in directories:
            raise ValidationError("recording environment_snapshot is missing")
        artifact_files = set(all_files)
        artifact_files.remove("sha256sums.txt")

        checksum_seal, checksum_bytes = self._read_relative_file(
            recording_fd,
            "sha256sums.txt",
            capture=True,
        )
        if checksum_seal.size == 0:
            raise ValidationError("sha256sums.txt must not be empty")
        declared = self._parse_checksum_bytes(checksum_bytes, artifact_files)

        seals: dict[str, _FileSeal] = {"sha256sums.txt": checksum_seal}
        captured: dict[str, bytes] = {}
        for relative in sorted(artifact_files):
            seal, data = self._read_relative_file(
                recording_fd,
                relative,
                capture=relative in capture_paths,
            )
            if seal.sha256 != declared[relative]:
                raise ValidationError(f"checksum mismatch: {relative}")
            seals[relative] = seal
            if relative in capture_paths:
                captured[relative] = data

        missing = sorted(set(REQUIRED_RECORDING_FILES).difference(seals))
        if missing:
            raise ValidationError(f"recording is missing files: {', '.join(missing)}")
        empty = [relative for relative in REQUIRED_RECORDING_FILES if seals[relative].size == 0]
        if empty:
            raise ValidationError(f"recording files must not be empty: {', '.join(empty)}")

        for relative, expected in seals.items():
            self._require_relative_file_identity(recording_fd, relative, expected)
        final_checksum_seal, final_checksum_bytes = self._read_relative_file(
            recording_fd,
            "sha256sums.txt",
            capture=True,
        )
        if final_checksum_seal != checksum_seal or final_checksum_bytes != checksum_bytes:
            raise ValidationError("sha256sums.txt changed during artifact inspection")
        final_files, final_directories = self._enumerate_artifact(recording_fd)
        if final_files != all_files or final_directories != directories:
            raise ValidationError("recording directory entries changed during inspection")

        return _ArtifactSnapshot(
            declared_hashes=declared,
            checksum_bytes=checksum_bytes,
            directories=frozenset(directories),
            files=seals,
            captured_bytes=captured,
        )

    @staticmethod
    def _pose_from_mapping(
        mapping: Mapping[str, Any],
        *,
        translation_key: str,
        quaternion_key: str,
        label: str,
    ) -> Pose:
        translation = _require_finite_sequence(
            mapping,
            translation_key,
            3,
            label,
        )
        quaternion = _require_finite_sequence(
            mapping,
            quaternion_key,
            4,
            label,
        )
        try:
            return Pose(
                position_m=(translation[0], translation[1], translation[2]),
                quaternion_xyzw=(
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                ),
            )
        except ValueError as error:
            raise ValidationError(f"{label} contains an invalid rigid transform") from error

    def _load_robot_audit_context(
        self,
        directory_fd: int,
        snapshot: _ArtifactSnapshot,
        manifest: Mapping[str, Any],
        robot_model: Mapping[str, Any],
        asset_provenance: Mapping[str, Any],
        curobo_robot: Mapping[str, Any],
        setup: Mapping[str, Any],
    ) -> _RobotAuditContext:
        if (
            _require_int(asset_provenance, "schema_version", "snapshot.asset_provenance")
            != 1
        ):
            raise ValidationError("snapshot asset_provenance.yaml schema_version must be 1")
        if (
            _require_string(asset_provenance, "robot_type", "snapshot.asset_provenance")
            != manifest["robot_type"]
        ):
            raise ValidationError("snapshot asset provenance robot_type does not match")
        generated = _require_mapping(
            asset_provenance.get("generated_urdf"),
            "snapshot.asset_provenance.generated_urdf",
        )
        generated_path = _safe_relative_path(
            _require_string(
                generated,
                "path",
                "snapshot.asset_provenance.generated_urdf",
            ),
            "snapshot.asset_provenance.generated_urdf.path",
        )
        generated_sha256 = _normalized_sha256(
            _require_string(
                generated,
                "sha256",
                "snapshot.asset_provenance.generated_urdf",
            ),
            "snapshot.asset_provenance.generated_urdf.sha256",
        )

        if _require_int(curobo_robot, "schema_version", "snapshot.curobo_robot") != 1:
            raise ValidationError("snapshot curobo_robot.yaml schema_version must be 1")
        robot_cfg = _require_mapping(
            curobo_robot.get("robot_cfg"),
            "snapshot.curobo_robot.robot_cfg",
        )
        kinematics = _require_mapping(
            robot_cfg.get("kinematics"),
            "snapshot.curobo_robot.robot_cfg.kinematics",
        )
        asset_root = _safe_relative_path(
            _require_string(
                kinematics,
                "asset_root_path",
                "snapshot.curobo_robot.robot_cfg.kinematics",
            ),
            "snapshot.curobo_robot.robot_cfg.kinematics.asset_root_path",
        )
        urdf_path = _safe_relative_path(
            _require_string(
                kinematics,
                "urdf_path",
                "snapshot.curobo_robot.robot_cfg.kinematics",
            ),
            "snapshot.curobo_robot.robot_cfg.kinematics.urdf_path",
        )
        configured_urdf_path = (PurePosixPath(asset_root) / urdf_path).as_posix()
        if configured_urdf_path != generated_path:
            raise ValidationError(
                "asset provenance and curobo_robot.yaml resolve different URDF paths"
            )

        artifact_relative = f"environment_snapshot/{generated_path}"
        expected_seal = snapshot.files.get(artifact_relative)
        if expected_seal is None or expected_seal.size == 0:
            raise ValidationError("hash-bound generated URDF is missing or empty")
        environment_hash = manifest["environment_hashes"].get(generated_path)
        if environment_hash is None or environment_hash != expected_seal.sha256:
            raise ValidationError("manifest does not bind the generated URDF seal")
        if generated_sha256 != expected_seal.sha256:
            raise ValidationError("asset provenance generated URDF SHA-256 does not match")
        captured_seal, urdf_bytes = self._read_relative_file(
            directory_fd,
            artifact_relative,
            capture=True,
        )
        if captured_seal != expected_seal:
            raise ValidationError("generated URDF changed after the artifact snapshot")

        robot = _require_mapping(robot_model.get("robot"), "snapshot.robot_model.robot")
        base_link = _require_string(robot, "base_link", "snapshot.robot_model.robot")
        if (
            _require_string(
                kinematics,
                "base_link",
                "snapshot.curobo_robot.robot_cfg.kinematics",
            )
            != base_link
        ):
            raise ValidationError("curobo_robot base_link does not match robot_model.yaml")
        tool_frame = _require_string(robot, "tool_link", "snapshot.robot_model.robot")
        tool_frames = _require_string_sequence(
            kinematics,
            "tool_frames",
            "snapshot.curobo_robot.robot_cfg.kinematics",
        )
        if len(set(tool_frames)) != len(tool_frames) or tool_frame not in tool_frames:
            raise ValidationError("curobo_robot tool_frames do not contain the robot tool_link")
        cspace = _require_mapping(
            kinematics.get("cspace"),
            "snapshot.curobo_robot.robot_cfg.kinematics.cspace",
        )
        if (
            _require_string_sequence(
                cspace,
                "joint_names",
                "snapshot.curobo_robot.robot_cfg.kinematics.cspace",
            )
            != manifest["joint_names"]
        ):
            raise ValidationError("curobo_robot joint order does not match the manifest")

        limits = _require_mapping(robot.get("limits"), "snapshot.robot_model.robot.limits")
        joint_count = len(manifest["joint_names"])
        velocity_limits = _require_finite_sequence(
            limits,
            "velocity_rad_s",
            joint_count,
            "snapshot.robot_model.robot.limits",
        )
        acceleration_limits = _require_finite_sequence(
            limits,
            "acceleration_rad_s2",
            joint_count,
            "snapshot.robot_model.robot.limits",
        )
        jerk_limits = _require_finite_sequence(
            limits,
            "jerk_rad_s3",
            joint_count,
            "snapshot.robot_model.robot.limits",
        )
        if min(*velocity_limits, *acceleration_limits, *jerk_limits) <= 0.0:
            raise ValidationError("snapshot robot dynamic limits must be positive")

        try:
            parsed_kinematics = RobotKinematics.from_urdf_bytes(
                urdf_bytes,
                base_link=base_link,
                joint_names=manifest["joint_names"],
            )
        except ValueError as error:
            raise ValidationError(f"sealed generated URDF is invalid: {error}") from error

        home = _require_finite_sequence(
            robot,
            "home_joint_positions",
            joint_count,
            "snapshot.robot_model.robot",
        )
        for value, position_limit in zip(
            home,
            parsed_kinematics.position_limits,
            strict=True,
        ):
            if not position_limit.contains(value, tolerance_rad=_CONTINUOUS_LIMIT_TOLERANCE):
                raise ValidationError("robot home position is outside the sealed URDF bounds")

        try:
            setup_config = parse_setup(setup, label="snapshot.setup")
        except SetupConfigurationError as error:
            raise ValidationError(f"sealed setup.yaml is invalid: {error}") from error
        configured_tool = setup_config.tool
        if configured_tool.parent_link != tool_frame:
            raise ValidationError("snapshot setup tool parent_link does not match robot tool_link")
        tip_frame = configured_tool.tip_frame
        if tool_frame != "tool0" or tip_frame != "rod_tip":
            raise ValidationError("snapshot robot tool/TCP frame contract is unsupported")
        tool_transform = configured_tool.parent_to_tip
        tool_to_tip = Pose(
            position_m=tool_transform.translation_m,
            quaternion_xyzw=tool_transform.quaternion_xyzw,
        )
        work = setup_config.work_frame
        try:
            base_to_parent = parsed_kinematics.forward_kinematics(
                home,
                (work.parent_frame,),
            )[work.parent_frame]
            alternate = tuple(
                0.5 * (limit.lower_rad + limit.upper_rad)
                for limit in parsed_kinematics.position_limits
            )
            alternate_parent = parsed_kinematics.forward_kinematics(
                alternate,
                (work.parent_frame,),
            )[work.parent_frame]
        except ValueError as error:
            raise ValidationError(
                "sealed setup work-frame parent cannot be resolved from the URDF"
            ) from error
        fixed_parent_error = pose_error(base_to_parent, alternate_parent)
        if fixed_parent_error.position_m > 1.0e-12 or fixed_parent_error.orientation_rad > 1.0e-12:
            raise ValidationError(
                "sealed setup work-frame parent is not fixed relative to robot base_link"
            )
        parent_to_work = work.parent_to_child
        base_to_work = compose_poses(
            base_to_parent,
            Pose(
                position_m=parent_to_work.translation_m,
                quaternion_xyzw=parent_to_work.quaternion_xyzw,
            ),
        )
        frame_to_base = {
            base_link: Pose((0.0, 0.0, 0.0)),
            work.child_frame: base_to_work,
            **{alias: base_to_work for alias in work.aliases},
        }

        try:
            parsed_kinematics.forward_kinematics(
                home,
                tuple(sorted(_EXPECTED_JOINT_FRAME_IDS)),
                tool_to_tip=tool_to_tip,
                tool_frame=tool_frame,
                tip_frame=tip_frame,
            )
        except ValueError as error:
            raise ValidationError(
                f"sealed URDF cannot produce every required frame: {error}"
            ) from error

        return _RobotAuditContext(
            kinematics=parsed_kinematics,
            base_link=base_link,
            tool_frame=tool_frame,
            tip_frame=tip_frame,
            tool_to_tip=tool_to_tip,
            frame_to_base=frame_to_base,
            velocity_limits_rad_s=velocity_limits,
            acceleration_limits_rad_s2=acceleration_limits,
            jerk_limits_rad_s3=jerk_limits,
        )

    @staticmethod
    def _source_pose_in_base(
        source: TimedPose,
        robot: _RobotAuditContext,
    ) -> Pose:
        frame_to_base = robot.frame_to_base.get(source.frame_id)
        if frame_to_base is None:
            raise ValidationError(
                f"source trajectory frame {source.frame_id} is not sealed in setup.yaml"
            )
        try:
            frame_to_tip = Pose(
                position_m=source.position_m,
                quaternion_xyzw=source.quaternion_xyzw,
            )
        except ValueError as error:
            raise ValidationError("source trajectory contains an invalid pose") from error
        return compose_poses(frame_to_base, frame_to_tip)

    @staticmethod
    def _interpolate_pose(start: Pose, end: Pose, ratio: float) -> Pose:
        if not 0.0 <= ratio <= 1.0 or not math.isfinite(ratio):
            raise ValidationError("source pose interpolation ratio is invalid")
        position = (
            (1.0 - ratio) * start.position_m[0] + ratio * end.position_m[0],
            (1.0 - ratio) * start.position_m[1] + ratio * end.position_m[1],
            (1.0 - ratio) * start.position_m[2] + ratio * end.position_m[2],
        )
        start_quaternion = start.quaternion_xyzw
        end_quaternion = end.quaternion_xyzw
        dot = sum(
            before * after
            for before, after in zip(
                start_quaternion,
                end_quaternion,
                strict=True,
            )
        )
        if dot < 0.0:
            end_quaternion = (
                -end_quaternion[0],
                -end_quaternion[1],
                -end_quaternion[2],
                -end_quaternion[3],
            )
            dot = -dot
        dot = min(1.0, max(-1.0, dot))
        if dot > 0.9995:
            quaternion = (
                (1.0 - ratio) * start_quaternion[0] + ratio * end_quaternion[0],
                (1.0 - ratio) * start_quaternion[1] + ratio * end_quaternion[1],
                (1.0 - ratio) * start_quaternion[2] + ratio * end_quaternion[2],
                (1.0 - ratio) * start_quaternion[3] + ratio * end_quaternion[3],
            )
        else:
            angle = math.acos(dot)
            sine = math.sin(angle)
            if abs(sine) <= 1.0e-15:
                raise ValidationError("source quaternion interpolation is singular")
            start_weight = math.sin((1.0 - ratio) * angle) / sine
            end_weight = math.sin(ratio * angle) / sine
            quaternion = (
                start_weight * start_quaternion[0]
                + end_weight * end_quaternion[0],
                start_weight * start_quaternion[1]
                + end_weight * end_quaternion[1],
                start_weight * start_quaternion[2]
                + end_weight * end_quaternion[2],
                start_weight * start_quaternion[3]
                + end_weight * end_quaternion[3],
            )
        try:
            return Pose(position_m=position, quaternion_xyzw=quaternion)
        except ValueError as error:
            raise ValidationError("interpolated source pose is invalid") from error

    @classmethod
    def _interpolate_pose_stream_at_time(
        cls,
        times: Sequence[float],
        poses: Sequence[Pose],
        target_time_s: float,
        coverage_tolerance_s: float,
    ) -> Pose:
        if len(times) != len(poses) or len(times) < 2:
            raise ValidationError("recorded pose stream is internally inconsistent")
        if (
            not math.isfinite(target_time_s)
            or not math.isfinite(coverage_tolerance_s)
            or coverage_tolerance_s < 0.0
        ):
            raise ValidationError("recorded pose interpolation time is invalid")
        if (
            target_time_s < times[0] - coverage_tolerance_s - 1.0e-12
            or target_time_s > times[-1] + coverage_tolerance_s + 1.0e-12
        ):
            raise ValidationError(
                "sim_actual_tip.csv does not cover an authoritative source time"
            )
        if target_time_s <= times[0]:
            return poses[0]
        if target_time_s >= times[-1]:
            return poses[-1]

        finish_index = bisect_left(times, target_time_s)
        if finish_index >= len(times):
            raise ValidationError("recorded pose interpolation bracket is missing")
        if abs(times[finish_index] - target_time_s) <= 1.0e-12:
            return poses[finish_index]
        if finish_index == 0:
            raise ValidationError("recorded pose interpolation start bracket is missing")
        start_index = finish_index - 1
        interval_s = times[finish_index] - times[start_index]
        if (
            not math.isfinite(interval_s)
            or interval_s <= 0.0
            or target_time_s - times[start_index]
            > coverage_tolerance_s + 1.0e-12
            or times[finish_index] - target_time_s
            > coverage_tolerance_s + 1.0e-12
        ):
            raise ValidationError(
                "sim_actual_tip.csv does not cover an authoritative source time"
            )
        return cls._interpolate_pose(
            poses[start_index],
            poses[finish_index],
            (target_time_s - times[start_index]) / interval_s,
        )

    @classmethod
    def _audit_canonical_trajectory(
        cls,
        trajectory: CompiledTrajectory,
        source: Sequence[TimedPose],
        robot: _RobotAuditContext,
    ) -> _CanonicalAuditMetrics:
        maximum_velocity = [0.0] * len(trajectory.joint_names)
        maximum_acceleration = [0.0] * len(trajectory.joint_names)
        maximum_jerk = [0.0] * len(trajectory.joint_names)
        for segment_index, (start, end) in enumerate(
            zip(trajectory.samples[:-1], trajectory.samples[1:], strict=True)
        ):
            duration_s = end.time_s - start.time_s
            for joint_index, joint_name in enumerate(trajectory.joint_names):
                try:
                    segment = QuinticHermiteSegment.from_boundary_conditions(
                        duration_s=duration_s,
                        start_position=start.positions[joint_index],
                        start_velocity=start.velocities[joint_index],
                        start_acceleration=start.accelerations[joint_index],
                        end_position=end.positions[joint_index],
                        end_velocity=end.velocities[joint_index],
                        end_acceleration=end.accelerations[joint_index],
                    )
                    for candidate in segment.candidate_extrema(0):
                        if not robot.kinematics.position_limits[joint_index].contains(
                            candidate.value,
                            tolerance_rad=_CONTINUOUS_LIMIT_TOLERANCE,
                        ):
                            raise ValidationError(
                                "canonical trajectory exceeds sealed URDF position bounds: "
                                f"joint={joint_name}, segment={segment_index}, "
                                f"time_s={start.time_s + candidate.time_s:.9f}, "
                                f"position_rad={candidate.value:.9g}"
                            )
                    maximum_velocity[joint_index] = max(
                        maximum_velocity[joint_index],
                        segment.maximum_absolute(1).absolute_value,
                    )
                    maximum_acceleration[joint_index] = max(
                        maximum_acceleration[joint_index],
                        segment.maximum_absolute(2).absolute_value,
                    )
                    maximum_jerk[joint_index] = max(
                        maximum_jerk[joint_index],
                        segment.maximum_absolute(3).absolute_value,
                    )
                except ValueError as error:
                    raise ValidationError(
                        "canonical trajectory quintic segment is invalid: "
                        f"joint={joint_name}, segment={segment_index}: {error}"
                    ) from error

        ratios = (
            max(
                value / limit
                for value, limit in zip(
                    maximum_velocity,
                    robot.velocity_limits_rad_s,
                    strict=True,
                )
            ),
            max(
                value / limit
                for value, limit in zip(
                    maximum_acceleration,
                    robot.acceleration_limits_rad_s2,
                    strict=True,
                )
            ),
            max(
                value / limit
                for value, limit in zip(
                    maximum_jerk,
                    robot.jerk_limits_rad_s3,
                    strict=True,
                )
            ),
        )
        derivative_names = ("velocity", "acceleration", "jerk")
        for name, ratio in zip(derivative_names, ratios, strict=True):
            if ratio > 1.0 + _CONTINUOUS_LIMIT_TOLERANCE:
                raise ValidationError(
                    f"canonical trajectory continuous {name} limit ratio exceeds 1.0"
                )

        source_times = tuple(
            _seconds_to_nanoseconds(pose.time_s, "source waypoint time_s")
            for pose in source
        )
        source_poses_in_base = tuple(
            cls._source_pose_in_base(pose, robot) for pose in source
        )
        maximum_tip_position_error = 0.0
        maximum_tip_orientation_error = 0.0
        source_index = 0
        for sample in trajectory.samples:
            sample_time = _seconds_to_nanoseconds(
                sample.time_s,
                "canonical sample time_s",
            )
            while (
                source_index + 1 < len(source_times)
                and source_times[source_index + 1] <= sample_time
            ):
                source_index += 1
            if sample_time == source_times[source_index]:
                target_tip = source_poses_in_base[source_index]
            elif (
                source_index + 1 < len(source_times)
                and source_times[source_index] < sample_time < source_times[source_index + 1]
            ):
                ratio = (sample_time - source_times[source_index]) / (
                    source_times[source_index + 1] - source_times[source_index]
                )
                target_tip = cls._interpolate_pose(
                    source_poses_in_base[source_index],
                    source_poses_in_base[source_index + 1],
                    ratio,
                )
            else:
                raise ValidationError(
                    "canonical trajectory time is outside the authoritative source path"
                )
            try:
                actual_tip = robot.kinematics.forward_kinematics(
                    sample.positions,
                    (robot.tip_frame,),
                    tool_to_tip=robot.tool_to_tip,
                    tool_frame=robot.tool_frame,
                    tip_frame=robot.tip_frame,
                )[robot.tip_frame]
            except ValueError as error:
                raise ValidationError(f"canonical FK failed: {error}") from error
            tip_error = pose_error(actual_tip, target_tip)
            maximum_tip_position_error = max(
                maximum_tip_position_error,
                tip_error.position_m,
            )
            maximum_tip_orientation_error = max(
                maximum_tip_orientation_error,
                tip_error.orientation_deg,
            )

        return _CanonicalAuditMetrics(
            max_joint_velocity_ratio=ratios[0],
            max_joint_acceleration_ratio=ratios[1],
            max_joint_jerk_ratio=ratios[2],
            max_tip_position_error_m=maximum_tip_position_error,
            max_tip_orientation_error_deg=maximum_tip_orientation_error,
        )

    @staticmethod
    def _reconcile_canonical_audit(
        audit: _CanonicalAuditMetrics,
        compile_evidence: Mapping[str, Any],
    ) -> None:
        metrics = compile_evidence["metrics"]
        expected = {
            "max_joint_velocity_ratio": (
                audit.max_joint_velocity_ratio,
                1.0e-6,
                1.0e-3,
            ),
            "max_joint_acceleration_ratio": (
                audit.max_joint_acceleration_ratio,
                1.0e-6,
                1.0e-3,
            ),
            "max_joint_jerk_ratio": (
                audit.max_joint_jerk_ratio,
                1.0e-6,
                1.0e-3,
            ),
            "max_tip_position_error_m": (
                audit.max_tip_position_error_m,
                1.0e-6,
                _AUDIT_METRIC_RELATIVE_TOLERANCE,
            ),
            "max_tip_orientation_error_deg": (
                audit.max_tip_orientation_error_deg,
                1.0e-4,
                _AUDIT_METRIC_RELATIVE_TOLERANCE,
            ),
        }
        for key, (
            recomputed,
            absolute_tolerance,
            relative_tolerance,
        ) in expected.items():
            if not _audit_metric_matches(
                metrics[key],
                recomputed,
                absolute_tolerance=absolute_tolerance,
                relative_tolerance=relative_tolerance,
            ):
                raise ValidationError(
                    f"compile_report {key} does not match the sealed canonical trajectory"
                )

    def _validate_manifest(
        self,
        manifest: Mapping[str, Any],
        recording: Path,
    ) -> dict[str, Any]:
        if _require_int(manifest, "schema_version", "manifest") != _MANIFEST_SCHEMA_VERSION:
            raise ValidationError("recording manifest schema is unsupported")
        recording_id = _require_string(manifest, "recording_id", "manifest")
        if recording_id != recording.name:
            raise ValidationError("manifest recording_id does not match its directory name")
        if _require_string(manifest, "status", "manifest") != "validated":
            raise ValidationError(
                "only status=validated recordings can be inspected or played"
            )
        environment_id = _require_string(manifest, "environment_id", "manifest")
        trajectory_id = _require_string(manifest, "trajectory_id", "manifest")

        robot = _require_mapping(manifest.get("robot"), "manifest.robot")
        robot_type = _require_string(robot, "type", "manifest.robot")
        joint_names = _require_string_sequence(
            robot,
            "joint_names",
            "manifest.robot",
        )
        if len(set(joint_names)) != len(joint_names):
            raise ValidationError("manifest robot.joint_names contains duplicates")

        trajectory = _require_mapping(
            manifest.get("trajectory"),
            "manifest.trajectory",
        )
        if _require_float(trajectory, "duration_s", "manifest.trajectory") <= 0.0:
            raise ValidationError("manifest trajectory duration_s must be positive")
        if _require_float(trajectory, "nominal_dt_s", "manifest.trajectory") <= 0.0:
            raise ValidationError("manifest trajectory nominal_dt_s must be positive")
        if _require_int(trajectory, "point_count", "manifest.trajectory") < 2:
            raise ValidationError("manifest trajectory point_count must be at least two")
        if _require_string(trajectory, "interpolation", "manifest.trajectory") != "spline":
            raise ValidationError("recording interpolation must be spline")
        fields = _require_string_sequence(
            trajectory,
            "fields",
            "manifest.trajectory",
        )
        for required in ("position", "velocity", "acceleration"):
            if required not in fields:
                raise ValidationError(
                    f"recording trajectory fields are missing {required}"
                )

        validation = _require_mapping(
            manifest.get("validation"),
            "manifest.validation",
        )
        raw_hashes = _require_mapping(manifest.get("hashes"), "manifest.hashes")
        hashes: dict[str, str] = {}
        for key, value in raw_hashes.items():
            if key == "curobo_revision":
                hashes[key] = _normalized_git_commit(value, "manifest.hashes.curobo_revision")
            else:
                hashes[key] = _normalized_sha256(value, f"manifest.hashes.{key}")
        for required in ("source_trajectory", "joint_trajectory", "curobo_revision"):
            if required not in hashes:
                raise ValidationError(f"manifest hashes is missing {required}")

        environment_hashes: dict[str, str] = {}
        for key, digest in hashes.items():
            if key in _PROVENANCE_HASH_KEYS:
                continue
            safe_key = _safe_relative_path(key, "manifest.hashes")
            if safe_key in environment_hashes:
                raise ValidationError(f"manifest hashes contains duplicate path {safe_key}")
            environment_hashes[safe_key] = digest
        missing_environment = sorted(
            set(_REQUIRED_ENVIRONMENT_FILES).difference(environment_hashes)
        )
        if missing_environment:
            raise ValidationError(
                "manifest hashes is missing required environment file "
                f"{missing_environment[0]}"
            )
        return {
            "root": manifest,
            "recording_id": recording_id,
            "environment_id": environment_id,
            "trajectory_id": trajectory_id,
            "robot_type": robot_type,
            "joint_names": joint_names,
            "trajectory": trajectory,
            "validation": validation,
            "hashes": hashes,
            "environment_hashes": environment_hashes,
            "curobo_revision": hashes["curobo_revision"],
        }

    def _validate_snapshot_identity(
        self,
        environment: Mapping[str, Any],
        robot_model: Mapping[str, Any],
        manifest: Mapping[str, Any],
    ) -> None:
        if _require_int(environment, "schema_version", "snapshot.environment") != 1:
            raise ValidationError("snapshot environment.yaml schema_version must be 1")
        if (
            _require_string(environment, "environment_id", "snapshot.environment")
            != manifest["environment_id"]
        ):
            raise ValidationError(
                "snapshot environment_id does not match the recording manifest"
            )
        if _require_int(robot_model, "schema_version", "snapshot.robot_model") != 1:
            raise ValidationError("snapshot robot_model.yaml schema_version must be 1")
        robot = _require_mapping(robot_model.get("robot"), "snapshot.robot_model.robot")
        if (
            _require_string(robot, "type", "snapshot.robot_model.robot")
            != manifest["robot_type"]
        ):
            raise ValidationError("snapshot robot type does not match the manifest")
        snapshot_joint_names = _require_string_sequence(
            robot,
            "joint_names",
            "snapshot.robot_model.robot",
        )
        if snapshot_joint_names != manifest["joint_names"]:
            raise ValidationError("snapshot robot joint_names do not match the manifest")

    def _validate_compile_report(
        self,
        report: Mapping[str, Any],
        robot_model: Mapping[str, Any],
        manifest: Mapping[str, Any],
        source_sha256: str,
        joint_sha256: str,
    ) -> dict[str, Any]:
        if (
            _require_int(report, "schema_version", "compile_report")
            != _COMPILE_REPORT_SCHEMA_VERSION
        ):
            raise ValidationError(
                "legacy compile_report schema is unsupported; schema_version must be 2"
            )
        if (
            _require_string(report, "compiler_contract_revision", "compile_report")
            != _COMPILER_CONTRACT_REVISION
        ):
            raise ValidationError("compile_report compiler_contract_revision is unsupported")
        if (
            _require_string(report, "environment_id", "compile_report")
            != manifest["environment_id"]
        ):
            raise ValidationError("compile_report environment_id does not match the manifest")
        if (
            _require_string(report, "trajectory_id", "compile_report")
            != manifest["trajectory_id"]
        ):
            raise ValidationError("compile_report trajectory_id does not match the manifest")
        curobo_revision = _normalized_git_commit(
            _require_string(report, "curobo_revision", "compile_report"),
            "compile_report.curobo_revision",
        )
        if curobo_revision != manifest["curobo_revision"]:
            raise ValidationError("compile_report curobo_revision does not match the manifest")
        reported_source_hash = _normalized_sha256(
            _require_string(report, "source_trajectory_sha256", "compile_report"),
            "compile_report.source_trajectory_sha256",
        )
        reported_joint_hash = _normalized_sha256(
            _require_string(report, "joint_trajectory_sha256", "compile_report"),
            "compile_report.joint_trajectory_sha256",
        )
        if reported_source_hash != source_sha256:
            raise ValidationError(
                "compile_report source_trajectory_sha256 does not match the sealed CSV"
            )
        if reported_joint_hash != joint_sha256:
            raise ValidationError(
                "compile_report joint_trajectory_sha256 does not match the sealed CSV"
            )

        raw_environment_hashes = _require_mapping(
            report.get("environment_hashes"),
            "compile_report.environment_hashes",
        )
        if not raw_environment_hashes:
            raise ValidationError("compile_report environment_hashes must not be empty")
        environment_hashes: dict[str, str] = {}
        for key, value in raw_environment_hashes.items():
            safe_key = _safe_relative_path(key, "compile_report.environment_hashes")
            if safe_key in environment_hashes:
                raise ValidationError(
                    f"compile_report environment_hashes contains duplicate path {safe_key}"
                )
            environment_hashes[safe_key] = _normalized_sha256(
                value,
                f"compile_report.environment_hashes.{safe_key}",
            )
        if environment_hashes != manifest["environment_hashes"]:
            raise ValidationError(
                "compile_report environment_hashes do not exactly match the manifest"
            )

        self._validate_joint_winding(report, robot_model, manifest["joint_names"])
        count_fields = (
            "point_count",
            "source_waypoint_count",
            "matched_source_waypoint_count",
            "checked_sample_count",
        )
        float_fields = (
            "duration_s",
            "source_duration_s",
            "max_source_time_error_s",
            "uniform_time_scale",
            "max_joint_velocity_ratio",
            "max_joint_acceleration_ratio",
            "max_joint_jerk_ratio",
            "max_tip_position_error_m",
            "max_tip_orientation_error_deg",
            "minimum_collision_distance_m",
            "max_scene_collision_cost",
            "max_self_collision_cost",
            "max_joint_bound_cost",
        )
        counts = {
            key: _require_int(report, key, "compile_report") for key in count_fields
        }
        metrics = {
            key: _require_float(report, key, "compile_report") for key in float_fields
        }
        evidence = {
            "root": report,
            "counts": counts,
            "metrics": metrics,
            "environment_hashes": environment_hashes,
        }
        self._validate_compile_metrics(evidence)
        return evidence

    def _validate_joint_winding(
        self,
        report: Mapping[str, Any],
        robot_model: Mapping[str, Any],
        joint_names: tuple[str, ...],
    ) -> None:
        robot = _require_mapping(robot_model.get("robot"), "snapshot.robot_model.robot")
        snapshot_joint_names = _require_string_sequence(
            robot,
            "joint_names",
            "snapshot.robot_model.robot",
        )
        if snapshot_joint_names != joint_names:
            raise ValidationError("joint winding robot joint order does not match the manifest")
        home = _require_finite_sequence(
            robot,
            "home_joint_positions",
            len(joint_names),
            "snapshot.robot_model.robot",
        )
        winding = _require_mapping(report.get("joint_winding"), "compile_report.joint_winding")
        if (
            _require_string(winding, "policy", "compile_report.joint_winding")
            != _JOINT_WINDING_POLICY
        ):
            raise ValidationError("compile_report joint_winding policy is unsupported")
        reference = _require_finite_sequence(
            winding,
            "reference_positions_rad",
            len(joint_names),
            "compile_report.joint_winding",
        )
        selected = _require_finite_sequence(
            winding,
            "selected_seed_offsets_rad",
            len(joint_names),
            "compile_report.joint_winding",
        )
        optimized = _require_finite_sequence(
            winding,
            "post_optimization_offsets_rad",
            len(joint_names),
            "compile_report.joint_winding",
        )
        for index, (actual, expected) in enumerate(zip(reference, home, strict=True)):
            if abs(actual - expected) > 1.0e-12:
                raise ValidationError(
                    "compile_report joint_winding reference does not match robot home "
                    f"at joint {index}"
                )
        for offset in (*selected, *optimized):
            turns = offset / math.tau
            if abs(turns - round(turns)) > 1.0e-9:
                raise ValidationError(
                    "compile_report joint_winding offset is not an integer 2pi turn"
                )

    @staticmethod
    def _validate_compile_metrics(evidence: Mapping[str, Any]) -> None:
        counts = evidence["counts"]
        metrics = evidence["metrics"]
        if counts["point_count"] < 2 or counts["source_waypoint_count"] < 2:
            raise ValidationError("compile_report point counts are incomplete")
        if (
            counts["matched_source_waypoint_count"]
            != counts["source_waypoint_count"]
        ):
            raise ValidationError("compile_report did not match every source waypoint")
        if counts["checked_sample_count"] != counts["point_count"]:
            raise ValidationError(
                "compile_report checked_sample_count must equal point_count"
            )
        if metrics["duration_s"] <= 0.0 or metrics["source_duration_s"] <= 0.0:
            raise ValidationError("compile_report durations must be positive")
        if abs(metrics["duration_s"] - metrics["source_duration_s"]) > _TIME_TOLERANCE:
            raise ValidationError(
                "compile_report duration differs from authoritative source duration"
            )
        if abs(metrics["max_source_time_error_s"]) > 1.0e-12:
            raise ValidationError(
                "compile_report max_source_time_error_s must be exactly zero"
            )
        if abs(metrics["uniform_time_scale"] - 1.0) > 1.0e-12:
            raise ValidationError("compile_report uniform_time_scale must be exactly 1.0")
        for key in (
            "max_joint_velocity_ratio",
            "max_joint_acceleration_ratio",
            "max_joint_jerk_ratio",
        ):
            if metrics[key] < 0.0 or metrics[key] > 1.0 + _TIME_TOLERANCE:
                raise ValidationError(f"compile_report {key} is outside [0, 1]")
        for key in (
            "max_tip_position_error_m",
            "max_tip_orientation_error_deg",
            "minimum_collision_distance_m",
            "max_scene_collision_cost",
            "max_self_collision_cost",
            "max_joint_bound_cost",
        ):
            if metrics[key] < 0.0:
                raise ValidationError(f"compile_report {key} must be non-negative")
        for key in (
            "max_scene_collision_cost",
            "max_self_collision_cost",
            "max_joint_bound_cost",
        ):
            if metrics[key] > _CUROBO_FEASIBILITY_COST_TOLERANCE:
                raise ValidationError(
                    f"compile_report {key} exceeds the cuRobo feasibility tolerance"
                )

    def _parse_joint_timing(
        self,
        data: bytes,
        joint_names: tuple[str, ...],
    ) -> tuple[tuple[int, ...], tuple[str, ...]]:
        try:
            text = data.decode("utf-8-sig")
        except UnicodeDecodeError as error:
            raise ValidationError("joint_trajectory.csv is not valid UTF-8") from error
        reader = csv.reader(io.StringIO(text, newline=""))
        try:
            header = next(reader)
        except StopIteration as error:
            raise ValidationError("joint_trajectory.csv is empty") from error
        expected_header = list(joint_trajectory_columns(joint_names))
        if header != expected_header:
            raise ValidationError(
                "joint_trajectory.csv header does not exactly match manifest joint_names"
            )
        times: list[int] = []
        phases: list[str] = []
        for line_number, row in enumerate(reader, start=2):
            if len(row) != len(expected_header):
                raise ValidationError(
                    f"joint_trajectory.csv row {line_number} has the wrong column count"
                )
            time_ns = _time_to_nanoseconds(
                row[0].strip(),
                f"joint_trajectory.csv row {line_number} time_s",
            )
            if times and time_ns <= times[-1]:
                raise ValidationError(
                    "joint_trajectory.csv time_s must be strictly increasing"
                )
            phase = row[-1].strip()
            if phase not in {"draw", "hold"}:
                raise ValidationError(
                    f"joint_trajectory.csv row {line_number} has invalid phase"
                )
            times.append(time_ns)
            phases.append(phase)
        if len(times) < 2 or times[0] != 0:
            raise ValidationError(
                "joint_trajectory.csv must contain at least two points and start at zero"
            )
        return tuple(times), tuple(phases)

    @staticmethod
    def _validate_canonical_trajectory(trajectory: CompiledTrajectory) -> None:
        def require_zero(sample: Any, label: str) -> None:
            if any(
                abs(value) > _CANONICAL_DERIVATIVE_TOLERANCE
                for value in (*sample.velocities, *sample.accelerations)
            ):
                raise ValidationError(
                    f"joint_trajectory.csv {label} must have zero qd and qdd"
                )

        require_zero(trajectory.samples[0], "first point")
        require_zero(trajectory.samples[-1], "last point")
        for index, sample in enumerate(trajectory.samples):
            if sample.phase not in {"draw", "hold"}:
                raise ValidationError("canonical trajectory phases must be draw or hold")
            if sample.phase != "hold":
                continue
            if index == 0:
                raise ValidationError("canonical trajectory cannot start with hold")
            previous = trajectory.samples[index - 1]
            if any(
                abs(current - before) > _CANONICAL_DERIVATIVE_TOLERANCE
                for current, before in zip(
                    sample.positions,
                    previous.positions,
                    strict=True,
                )
            ):
                raise ValidationError(
                    "joint_trajectory.csv hold phase must keep joint positions fixed"
                )
            require_zero(previous, "hold boundary")
            require_zero(sample, "hold sample")

    @staticmethod
    def _validate_authoritative_timing(
        source_times: tuple[int, ...],
        source_phases: tuple[str, ...],
        joint_times: tuple[int, ...],
        joint_phases: tuple[str, ...],
    ) -> int:
        if source_times[-1] != joint_times[-1]:
            raise ValidationError(
                "recording duration differs from authoritative source duration"
            )
        joint_index = 0
        for source_time, source_phase in zip(
            source_times,
            source_phases,
            strict=True,
        ):
            while joint_index < len(joint_times) and joint_times[joint_index] < source_time:
                joint_index += 1
            if (
                joint_index >= len(joint_times)
                or joint_times[joint_index] != source_time
                or joint_phases[joint_index] != source_phase
            ):
                raise ValidationError(
                    "recording does not preserve every authoritative source time and phase"
                )
            joint_index += 1
        return len(source_times)

    @staticmethod
    def _validate_actual_compile_evidence(
        evidence: Mapping[str, Any],
        source_times: tuple[int, ...],
        joint_times: tuple[int, ...],
        parsed_point_count: int,
        matched_source_count: int,
    ) -> None:
        counts = evidence["counts"]
        metrics = evidence["metrics"]
        expected_counts = {
            "point_count": parsed_point_count,
            "source_waypoint_count": len(source_times),
            "matched_source_waypoint_count": matched_source_count,
            "checked_sample_count": parsed_point_count,
        }
        for key, expected in expected_counts.items():
            if counts[key] != expected:
                raise ValidationError(
                    f"compile_report {key} does not match the sealed trajectory"
                )
        if len(joint_times) != parsed_point_count:
            raise ValidationError(
                "joint_trajectory.csv row count differs from parsed point_count"
            )
        if _seconds_to_nanoseconds(metrics["duration_s"], "compile_report.duration_s") != joint_times[-1]:
            raise ValidationError(
                "compile_report duration_s does not match joint_trajectory.csv"
            )
        if (
            _seconds_to_nanoseconds(
                metrics["source_duration_s"],
                "compile_report.source_duration_s",
            )
            != source_times[-1]
        ):
            raise ValidationError(
                "compile_report source_duration_s does not match source_tip_trajectory.csv"
            )

    @staticmethod
    def _validate_manifest_compile_evidence(
        trajectory: Mapping[str, Any],
        evidence: Mapping[str, Any],
    ) -> None:
        counts = evidence["counts"]
        metrics = evidence["metrics"]
        for key in (
            "point_count",
            "source_waypoint_count",
            "matched_source_waypoint_count",
        ):
            if _require_int(trajectory, key, "manifest.trajectory") != counts[key]:
                raise ValidationError(
                    f"manifest trajectory {key} differs from compile_report"
                )
        for key in (
            "duration_s",
            "source_duration_s",
            "max_source_time_error_s",
            "uniform_time_scale",
            "max_joint_velocity_ratio",
            "max_joint_acceleration_ratio",
            "max_joint_jerk_ratio",
        ):
            actual = _require_float(trajectory, key, "manifest.trajectory")
            if not _float_matches(actual, metrics[key]):
                raise ValidationError(
                    f"manifest trajectory {key} differs from compile_report"
                )

    def _validate_validation_evidence(
        self,
        validation_report: Mapping[str, Any],
        manifest_validation: Mapping[str, Any],
        evidence: Mapping[str, Any],
        manifest: Mapping[str, Any],
        recording_config: Mapping[str, Any],
        planner: Mapping[str, Any],
        trajectory: CompiledTrajectory,
        source: Sequence[TimedPose],
        robot_audit: _RobotAuditContext,
        telemetry_bytes: Mapping[str, bytes],
    ) -> None:
        if validation_report != manifest_validation:
            raise ValidationError(
                "validation_report.yaml must exactly match manifest.validation"
            )
        if _require_int(validation_report, "schema_version", "validation_report") != 1:
            raise ValidationError("validation_report schema_version must be 1")
        if not _require_bool(
            validation_report,
            "simulation_completed",
            "validation_report",
        ):
            raise ValidationError("validation_report does not prove simulation completion")
        if (
            _require_string(validation_report, "controller_result", "validation_report")
            != "succeeded"
        ):
            raise ValidationError("validation_report controller_result must be succeeded")
        for key in (
            "actual_joint_recorded",
            "controller_state_recorded",
            "joint_frames_recorded",
            "actual_tip_recorded",
        ):
            if not _require_bool(validation_report, key, "validation_report"):
                raise ValidationError(f"validation_report {key} must be true")

        counts = evidence["counts"]
        metrics = evidence["metrics"]
        for key in (
            "point_count",
            "source_waypoint_count",
            "matched_source_waypoint_count",
            "checked_sample_count",
        ):
            if _require_int(validation_report, key, "validation_report") != counts[key]:
                raise ValidationError(
                    f"validation_report {key} differs from compile_report"
                )
        for key in metrics:
            actual = _require_float(validation_report, key, "validation_report")
            if not _float_matches(actual, metrics[key]):
                raise ValidationError(
                    f"validation_report {key} differs from compile_report"
                )

        if (
            _require_int(
                validation_report,
                "simulation_execution_contract_revision",
                "validation_report",
            )
            != _SIMULATION_EXECUTION_CONTRACT_REVISION
        ):
            raise ValidationError("simulation execution contract revision must be 1")
        expected_strings = {
            "speed_scaling_evidence": _SIMULATION_SPEED_SCALING_EVIDENCE,
            "simulation_controller_name": _SIMULATION_CONTROLLER_NAME,
            "simulation_controller_type": _SIMULATION_CONTROLLER_TYPE,
            "simulation_controller_state": "active",
            "simulation_hardware_name": manifest["robot_type"],
            "simulation_hardware_type": _SIMULATION_HARDWARE_TYPE,
            "simulation_hardware_plugin": _SIMULATION_HARDWARE_PLUGIN,
            "simulation_hardware_state_label": "active",
        }
        for key, expected in expected_strings.items():
            if _require_string(validation_report, key, "validation_report") != expected:
                raise ValidationError(
                    f"simulation execution contract field {key} is invalid"
                )
        if (
            _require_int(
                validation_report,
                "simulation_hardware_state_id",
                "validation_report",
            )
            != _ACTIVE_LIFECYCLE_STATE_ID
        ):
            raise ValidationError("simulation hardware lifecycle state must be active")

        command_interfaces = sorted(
            f"{joint_name}/position" for joint_name in manifest["joint_names"]
        )
        state_interfaces = sorted(
            interface
            for joint_name in manifest["joint_names"]
            for interface in (
                f"{joint_name}/position",
                f"{joint_name}/velocity",
            )
        )
        expected_interfaces = {
            "simulation_controller_claimed_interfaces": command_interfaces,
            "simulation_controller_required_command_interfaces": command_interfaces,
            "simulation_controller_required_state_interfaces": state_interfaces,
        }
        for key, expected in expected_interfaces.items():
            actual_interfaces = list(
                _require_string_sequence(validation_report, key, "validation_report")
            )
            if (
                actual_interfaces != expected
                or len(set(actual_interfaces)) != len(actual_interfaces)
            ):
                raise ValidationError(
                    f"simulation execution interface evidence {key} is invalid"
                )

        self._validate_simulation_evidence(
            validation_report,
            recording_config,
            trajectory,
            source,
            robot_audit,
            self._planner_tip_tolerances(planner),
            telemetry_bytes,
        )

    @staticmethod
    def _load_simulation_validation_config(
        document: Mapping[str, Any],
    ) -> _SimulationValidationConfig:
        if _require_int(document, "schema_version", "snapshot.recording") != 1:
            raise ValidationError("snapshot recording.yaml schema_version must be 1")
        recording = _require_mapping(
            document.get("recording"),
            "snapshot.recording.recording",
        )
        validation = _require_mapping(
            recording.get("validation"),
            "snapshot.recording.recording.validation",
        )
        output_fields = _require_mapping(
            recording.get("output_fields"),
            "snapshot.recording.recording.output_fields",
        )
        first_point = _require_mapping(
            recording.get("first_point"),
            "snapshot.recording.recording.first_point",
        )
        last_point = _require_mapping(
            recording.get("last_point"),
            "snapshot.recording.recording.last_point",
        )
        if (
            _require_string(recording, "time_source", "snapshot.recording.recording")
            != "trajectory_time"
            or _require_string(
                recording,
                "canonical_source",
                "snapshot.recording.recording",
            )
            != "controller_action_goal"
            or any(
                not _require_bool(recording, key, "snapshot.recording.recording")
                for key in (
                    "record_controller_actual",
                    "record_joint_states",
                    "record_joint_frames",
                    "record_tip_state",
                )
            )
            or _require_float(
                first_point,
                "time_s",
                "snapshot.recording.recording.first_point",
            )
            != 0.0
            or any(
                not _require_bool(
                    output_fields,
                    key,
                    "snapshot.recording.recording.output_fields",
                )
                for key in ("positions", "velocities", "accelerations")
            )
            or any(
                not _require_bool(first_point, key, "snapshot.recording.recording.first_point")
                for key in ("require_zero_velocity", "require_zero_acceleration")
            )
            or any(
                not _require_bool(last_point, key, "snapshot.recording.recording.last_point")
                for key in ("require_zero_velocity", "require_zero_acceleration")
            )
        ):
            raise ValidationError(
                "snapshot recording.yaml canonical q/qd/qdd contract is not strict"
            )

        config = _SimulationValidationConfig(
            sample_dt_s=_require_float(
                recording,
                "sample_dt_s",
                "snapshot.recording.recording",
            ),
            max_missing_samples=_require_int(
                validation,
                "max_missing_samples",
                "snapshot.recording.recording.validation",
            ),
            max_time_jitter_s=_require_float(
                validation,
                "max_time_jitter_s",
                "snapshot.recording.recording.validation",
            ),
            max_quaternion_norm_error=_require_float(
                validation,
                "max_quaternion_norm_error",
                "snapshot.recording.recording.validation",
            ),
            require_action_success=_require_bool(
                validation,
                "require_action_success",
                "snapshot.recording.recording.validation",
            ),
            max_completion_overrun_s=_require_float(
                validation,
                "max_completion_overrun_s",
                "snapshot.recording.recording.validation",
            ),
            max_controller_reference_position_error_rad=_require_float(
                validation,
                "max_controller_reference_position_error_rad",
                "snapshot.recording.recording.validation",
            ),
            max_controller_reference_velocity_error_rad_s=_require_float(
                validation,
                "max_controller_reference_velocity_error_rad_s",
                "snapshot.recording.recording.validation",
            ),
            max_joint_position_tracking_error_rad=_require_float(
                validation,
                "max_joint_position_tracking_error_rad",
                "snapshot.recording.recording.validation",
            ),
            max_joint_velocity_tracking_error_rad_s=_require_float(
                validation,
                "max_joint_velocity_tracking_error_rad_s",
                "snapshot.recording.recording.validation",
            ),
            max_actual_controller_position_error_rad=_require_float(
                validation,
                "max_actual_controller_position_error_rad",
                "snapshot.recording.recording.validation",
            ),
            max_actual_controller_velocity_error_rad_s=_require_float(
                validation,
                "max_actual_controller_velocity_error_rad_s",
                "snapshot.recording.recording.validation",
            ),
            max_frame_fk_position_error_m=_require_float(
                validation,
                "max_frame_fk_position_error_m",
                "snapshot.recording.recording.validation",
            ),
            max_frame_fk_orientation_error_deg=_require_float(
                validation,
                "max_frame_fk_orientation_error_deg",
                "snapshot.recording.recording.validation",
            ),
        )
        if (
            config.sample_dt_s <= 0.0
            or config.max_missing_samples < 0
            or config.max_time_jitter_s < 0.0
            or config.max_time_jitter_s >= config.sample_dt_s
            or not 0.0 < config.max_quaternion_norm_error < 1.0
            or not config.require_action_success
            or config.max_completion_overrun_s < 0.0
            or config.max_controller_reference_position_error_rad <= 0.0
            or config.max_controller_reference_velocity_error_rad_s <= 0.0
            or config.max_joint_position_tracking_error_rad <= 0.0
            or config.max_joint_velocity_tracking_error_rad_s <= 0.0
            or config.max_actual_controller_position_error_rad <= 0.0
            or config.max_actual_controller_velocity_error_rad_s <= 0.0
            or config.max_frame_fk_position_error_m <= 0.0
            or config.max_frame_fk_orientation_error_deg <= 0.0
        ):
            raise ValidationError(
                "snapshot recording.yaml simulation validation limits are invalid"
            )
        return config

    @staticmethod
    def _strict_csv_rows(
        data: bytes,
        expected_header: Sequence[str],
        label: str,
    ) -> list[list[str]]:
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError as error:
            raise ValidationError(f"{label} is not valid UTF-8") from error
        lines = text.split("\n")
        if lines and lines[-1] == "":
            lines.pop()
        if not lines:
            raise ValidationError(f"{label} is empty")

        def split_line(raw: str) -> list[str]:
            line = raw[:-1] if raw.endswith("\r") else raw
            if '"' in line:
                raise ValidationError(f"{label} must not contain quoted fields")
            return [field.strip() for field in line.split(",")]

        if split_line(lines[0]) != list(expected_header):
            raise ValidationError(f"{label} header does not match the runtime schema")
        rows: list[list[str]] = []
        for line_number, raw in enumerate(lines[1:], start=2):
            line = raw[:-1] if raw.endswith("\r") else raw
            if not line.strip():
                raise ValidationError(f"{label} contains an empty row at line {line_number}")
            fields = split_line(raw)
            if len(fields) != len(expected_header):
                raise ValidationError(
                    f"{label} row {line_number} has the wrong column count"
                )
            rows.append(fields)
        if len(rows) < 2:
            raise ValidationError(f"{label} requires at least two data samples")
        return rows

    @staticmethod
    def _parse_csv_float(value: str, label: str) -> float:
        if _STRICT_CSV_NUMBER_PATTERN.fullmatch(value) is None:
            raise ValidationError(f"{label} is not a finite decimal number")
        try:
            parsed = float(value)
        except (OverflowError, ValueError) as error:
            raise ValidationError(f"{label} is not a finite decimal number") from error
        if not math.isfinite(parsed):
            raise ValidationError(f"{label} is not finite")
        return parsed

    @staticmethod
    def _analyze_sampling(
        times: Sequence[float],
        duration_s: float,
        config: _SimulationValidationConfig,
        label: str,
    ) -> _SamplingMetrics:
        if len(times) < 2:
            raise ValidationError(f"{label} requires at least two data samples")
        first = times[0]
        last = times[-1]
        if first < 0.0:
            raise ValidationError(f"{label} contains a negative trajectory timestamp")
        maximum_interval = 0.0
        maximum_jitter = 0.0
        missing_samples = 0
        for before, current in zip(times[:-1], times[1:], strict=True):
            interval = current - before
            if not math.isfinite(interval) or interval <= 0.0:
                raise ValidationError(
                    f"{label} trajectory timestamps must be strictly increasing"
                )
            maximum_interval = max(maximum_interval, interval)
            if interval <= config.sample_dt_s:
                nominal_steps = 1
                jitter = 0.0
            else:
                nominal_steps = max(
                    1,
                    math.floor(interval / config.sample_dt_s + 0.5),
                )
                jitter = abs(interval - nominal_steps * config.sample_dt_s)
            maximum_jitter = max(maximum_jitter, jitter)
            missing_samples += nominal_steps - 1

        coverage = last - first
        endpoint_tolerance = config.sample_dt_s + config.max_time_jitter_s
        if first > endpoint_tolerance:
            raise ValidationError(f"{label} starts too late to cover the trajectory")
        if last + endpoint_tolerance < duration_s:
            raise ValidationError(f"{label} ends before the trajectory is covered")
        if last > duration_s + config.max_completion_overrun_s + 1.0e-12:
            raise ValidationError(f"{label} continues beyond the commanded duration")
        minimum_coverage = max(0.0, duration_s - 2.0 * endpoint_tolerance)
        if coverage < minimum_coverage:
            raise ValidationError(f"{label} coverage duration is too short")
        nominal_sample_count = math.floor(duration_s / config.sample_dt_s) + 1
        if len(times) + config.max_missing_samples + 2 < nominal_sample_count:
            raise ValidationError(f"{label} sample count is too small")
        if maximum_jitter > config.max_time_jitter_s + 1.0e-12:
            raise ValidationError(f"{label} exceeds the maximum time jitter")
        if missing_samples > config.max_missing_samples:
            raise ValidationError(f"{label} exceeds the maximum missing sample count")
        maximum_allowed_interval = (
            (config.max_missing_samples + 1) * config.sample_dt_s
            + config.max_time_jitter_s
        )
        if maximum_interval > maximum_allowed_interval + 1.0e-12:
            raise ValidationError(f"{label} contains an excessive sample interval")
        return _SamplingMetrics(
            sample_count=len(times),
            first_trajectory_time_s=first,
            last_trajectory_time_s=last,
            coverage_duration_s=coverage,
            max_time_interval_s=maximum_interval,
            max_time_jitter_s=maximum_jitter,
            missing_sample_estimate=missing_samples,
        )

    @classmethod
    def _analyze_pose_rows(
        cls,
        rows: Sequence[Sequence[str]],
        duration_s: float,
        config: _SimulationValidationConfig,
        label: str,
        *,
        translation_offset: int,
        quaternion_offset: int,
    ) -> _PoseMetrics:
        ros_times: list[float] = []
        trajectory_times: list[float] = []
        maximum_quaternion_error = 0.0
        for row_number, row in enumerate(rows, start=2):
            ros_times.append(cls._parse_csv_float(row[0], f"{label} row {row_number} ros_time_s"))
            trajectory_times.append(
                cls._parse_csv_float(
                    row[1],
                    f"{label} row {row_number} trajectory_time_s",
                )
            )
            for axis in range(3):
                cls._parse_csv_float(
                    row[translation_offset + axis],
                    f"{label} row {row_number} translation",
                )
            squared_norm = 0.0
            for component in range(4):
                value = cls._parse_csv_float(
                    row[quaternion_offset + component],
                    f"{label} row {row_number} quaternion",
                )
                squared_norm += value * value
            norm = math.sqrt(squared_norm)
            if not math.isfinite(norm):
                raise ValidationError(f"{label} contains a non-finite quaternion norm")
            maximum_quaternion_error = max(
                maximum_quaternion_error,
                abs(norm - 1.0),
            )
        if ros_times[0] < 0.0:
            raise ValidationError(f"{label} contains a negative ROS timestamp")
        for before, current in zip(ros_times[:-1], ros_times[1:], strict=True):
            interval = current - before
            if not math.isfinite(interval) or interval <= 0.0:
                raise ValidationError(f"{label} ROS timestamps must be strictly increasing")
        sampling = cls._analyze_sampling(
            trajectory_times,
            duration_s,
            config,
            label,
        )
        if maximum_quaternion_error > config.max_quaternion_norm_error + 1.0e-12:
            raise ValidationError(f"{label} exceeds the quaternion norm tolerance")
        return _PoseMetrics(
            sampling=sampling,
            max_quaternion_norm_error=maximum_quaternion_error,
        )

    @classmethod
    def _pose_from_csv_row(
        cls,
        row: Sequence[str],
        *,
        translation_offset: int,
        quaternion_offset: int,
        label: str,
    ) -> Pose:
        translation = tuple(
            cls._parse_csv_float(
                row[translation_offset + axis],
                f"{label} translation",
            )
            for axis in range(3)
        )
        quaternion = tuple(
            cls._parse_csv_float(
                row[quaternion_offset + component],
                f"{label} quaternion",
            )
            for component in range(4)
        )
        try:
            return Pose(
                position_m=(translation[0], translation[1], translation[2]),
                quaternion_xyzw=(
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                ),
            )
        except ValueError as error:
            raise ValidationError(f"{label} contains an invalid pose") from error

    @classmethod
    def _recompute_simulation_metrics(
        cls,
        telemetry_bytes: Mapping[str, bytes],
        trajectory: CompiledTrajectory,
        config: _SimulationValidationConfig,
        source: Sequence[TimedPose],
        robot: _RobotAuditContext,
        planner_tip_tolerances: tuple[float, float],
    ) -> _SimulationMetrics:
        joint_names = trajectory.joint_names
        actual_header = ["ros_time_s", "trajectory_time_s"]
        actual_header.extend(f"{name}_pos" for name in joint_names)
        actual_header.extend(f"{name}_vel" for name in joint_names)
        controller_header = ["ros_time_s", "trajectory_time_s", "speed_scaling"]
        for prefix in ("reference", "feedback", "error"):
            controller_header.extend(f"{prefix}_{name}_pos" for name in joint_names)
            controller_header.extend(f"{prefix}_{name}_vel" for name in joint_names)
        frame_header = [
            "ros_time_s",
            "trajectory_time_s",
            "frame_id",
            "x_m",
            "y_m",
            "z_m",
            "qx",
            "qy",
            "qz",
            "qw",
        ]
        tip_header = [
            "ros_time_s",
            "trajectory_time_s",
            "x_m",
            "y_m",
            "z_m",
            "qx",
            "qy",
            "qz",
            "qw",
        ]
        actual_rows = cls._strict_csv_rows(
            telemetry_bytes["sim_actual_joint.csv"],
            actual_header,
            "sim_actual_joint.csv",
        )
        controller_rows = cls._strict_csv_rows(
            telemetry_bytes["sim_controller_state.csv"],
            controller_header,
            "sim_controller_state.csv",
        )
        frame_rows = cls._strict_csv_rows(
            telemetry_bytes["joint_frames.csv"],
            frame_header,
            "joint_frames.csv",
        )
        tip_rows = cls._strict_csv_rows(
            telemetry_bytes["sim_actual_tip.csv"],
            tip_header,
            "sim_actual_tip.csv",
        )

        joint_count = len(joint_names)
        actual_times: list[float] = []
        actual_positions: list[tuple[float, ...]] = []
        actual_velocities: list[tuple[float, ...]] = []
        for row_number, row in enumerate(actual_rows, start=2):
            cls._parse_csv_float(row[0], f"sim_actual_joint.csv row {row_number} ros_time_s")
            actual_times.append(
                cls._parse_csv_float(
                    row[1],
                    f"sim_actual_joint.csv row {row_number} trajectory_time_s",
                )
            )
            actual_positions.append(
                tuple(
                    cls._parse_csv_float(
                        row[2 + joint],
                        f"sim_actual_joint.csv row {row_number} joint position",
                    )
                    for joint in range(joint_count)
                )
            )
            actual_velocities.append(
                tuple(
                    cls._parse_csv_float(
                        row[2 + joint_count + joint],
                        f"sim_actual_joint.csv row {row_number} joint velocity",
                    )
                    for joint in range(joint_count)
                )
            )

        reference_position_offset = 3
        reference_velocity_offset = reference_position_offset + joint_count
        feedback_position_offset = reference_velocity_offset + joint_count
        feedback_velocity_offset = feedback_position_offset + joint_count
        error_position_offset = feedback_velocity_offset + joint_count
        error_velocity_offset = error_position_offset + joint_count
        controller_times: list[float] = []
        recorded_reference_positions: list[tuple[float, ...]] = []
        recorded_reference_velocities: list[tuple[float, ...]] = []
        recorded_feedback_positions: list[tuple[float, ...]] = []
        recorded_feedback_velocities: list[tuple[float, ...]] = []
        maximum_speed_scaling_error = 0.0
        maximum_position_tracking_error = 0.0
        maximum_velocity_tracking_error = 0.0
        for row_number, row in enumerate(controller_rows, start=2):
            cls._parse_csv_float(
                row[0],
                f"sim_controller_state.csv row {row_number} ros_time_s",
            )
            controller_times.append(
                cls._parse_csv_float(
                    row[1],
                    f"sim_controller_state.csv row {row_number} trajectory_time_s",
                )
            )
            speed_scaling = cls._parse_csv_float(
                row[2],
                f"sim_controller_state.csv row {row_number} speed_scaling",
            )
            maximum_speed_scaling_error = max(
                maximum_speed_scaling_error,
                abs(speed_scaling - 1.0),
            )
            reference_positions: list[float] = []
            reference_velocities: list[float] = []
            feedback_positions: list[float] = []
            feedback_velocities: list[float] = []
            for joint in range(joint_count):
                reference_position = cls._parse_csv_float(
                    row[reference_position_offset + joint],
                    f"sim_controller_state.csv row {row_number} reference position",
                )
                reference_velocity = cls._parse_csv_float(
                    row[reference_velocity_offset + joint],
                    f"sim_controller_state.csv row {row_number} reference velocity",
                )
                feedback_position = cls._parse_csv_float(
                    row[feedback_position_offset + joint],
                    f"sim_controller_state.csv row {row_number} feedback position",
                )
                feedback_velocity = cls._parse_csv_float(
                    row[feedback_velocity_offset + joint],
                    f"sim_controller_state.csv row {row_number} feedback velocity",
                )
                reported_position_error = cls._parse_csv_float(
                    row[error_position_offset + joint],
                    f"sim_controller_state.csv row {row_number} position error",
                )
                reported_velocity_error = cls._parse_csv_float(
                    row[error_velocity_offset + joint],
                    f"sim_controller_state.csv row {row_number} velocity error",
                )
                reference_positions.append(reference_position)
                reference_velocities.append(reference_velocity)
                feedback_positions.append(feedback_position)
                feedback_velocities.append(feedback_velocity)
                maximum_position_tracking_error = max(
                    maximum_position_tracking_error,
                    abs(reference_position - feedback_position),
                    abs(reported_position_error),
                )
                maximum_velocity_tracking_error = max(
                    maximum_velocity_tracking_error,
                    abs(reference_velocity - feedback_velocity),
                    abs(reported_velocity_error),
                )
            recorded_reference_positions.append(tuple(reference_positions))
            recorded_reference_velocities.append(tuple(reference_velocities))
            recorded_feedback_positions.append(tuple(feedback_positions))
            recorded_feedback_velocities.append(tuple(feedback_velocities))

        frame_stream_rows: dict[str, list[Sequence[str]]] = {}
        for row_number, row in enumerate(frame_rows, start=2):
            frame_id = row[2]
            if not frame_id:
                raise ValidationError(
                    f"joint_frames.csv row {row_number} contains an empty frame_id"
                )
            frame_stream_rows.setdefault(frame_id, []).append(row)
        if set(frame_stream_rows) != _EXPECTED_JOINT_FRAME_IDS:
            raise ValidationError(
                "joint_frames.csv must contain exactly the eight required frame streams"
            )

        duration_s = trajectory.duration_s
        actual_metrics = cls._analyze_sampling(
            actual_times,
            duration_s,
            config,
            "sim_actual_joint.csv",
        )
        controller_metrics = cls._analyze_sampling(
            controller_times,
            duration_s,
            config,
            "sim_controller_state.csv",
        )
        joint_frame_metrics: dict[str, _PoseMetrics] = {}
        maximum_joint_frame_quaternion_error = 0.0
        for frame_id in sorted(frame_stream_rows):
            pose_metrics = cls._analyze_pose_rows(
                frame_stream_rows[frame_id],
                duration_s,
                config,
                f"joint_frames.csv frame {frame_id}",
                translation_offset=3,
                quaternion_offset=6,
            )
            joint_frame_metrics[frame_id] = pose_metrics
            maximum_joint_frame_quaternion_error = max(
                maximum_joint_frame_quaternion_error,
                pose_metrics.max_quaternion_norm_error,
            )
        actual_tip_metrics = cls._analyze_pose_rows(
            tip_rows,
            duration_s,
            config,
            "sim_actual_tip.csv rod_tip",
            translation_offset=2,
            quaternion_offset=5,
        )

        if maximum_speed_scaling_error > _SIMULATION_SPEED_SCALING_TOLERANCE:
            raise ValidationError(
                "sim_controller_state.csv speed_scaling must remain exactly 1.0"
            )
        if (
            maximum_position_tracking_error
            > config.max_joint_position_tracking_error_rad + 1.0e-12
        ):
            raise ValidationError("simulation joint position tracking error exceeds its limit")
        if (
            maximum_velocity_tracking_error
            > config.max_joint_velocity_tracking_error_rad_s + 1.0e-12
        ):
            raise ValidationError("simulation joint velocity tracking error exceeds its limit")

        maximum_reference_position_error = 0.0
        maximum_reference_velocity_error = 0.0
        nearest_index = 0
        match_tolerance = config.sample_dt_s + config.max_time_jitter_s
        for sample in trajectory.samples:
            while (
                nearest_index + 1 < len(controller_times)
                and abs(controller_times[nearest_index + 1] - sample.time_s)
                <= abs(controller_times[nearest_index] - sample.time_s)
            ):
                nearest_index += 1
            if abs(controller_times[nearest_index] - sample.time_s) > match_tolerance + 1.0e-12:
                raise ValidationError(
                    "simulation controller reference does not cover every canonical time"
                )
            maximum_reference_position_error = max(
                maximum_reference_position_error,
                *(
                    abs(actual - expected)
                    for actual, expected in zip(
                        recorded_reference_positions[nearest_index],
                        sample.positions,
                        strict=True,
                    )
                ),
            )
            maximum_reference_velocity_error = max(
                maximum_reference_velocity_error,
                *(
                    abs(actual - expected)
                    for actual, expected in zip(
                        recorded_reference_velocities[nearest_index],
                        sample.velocities,
                        strict=True,
                    )
                ),
            )
        if (
            maximum_reference_position_error
            > config.max_controller_reference_position_error_rad + 1.0e-12
        ):
            raise ValidationError(
                "simulation controller reference differs from canonical positions"
            )
        if (
            maximum_reference_velocity_error
            > config.max_controller_reference_velocity_error_rad_s + 1.0e-12
        ):
            raise ValidationError(
                "simulation controller reference differs from canonical velocities"
            )

        maximum_actual_controller_position_error = 0.0
        maximum_actual_controller_velocity_error = 0.0
        nearest_feedback_index = 0
        for time_s, positions, velocities in zip(
            actual_times,
            actual_positions,
            actual_velocities,
            strict=True,
        ):
            while (
                nearest_feedback_index + 1 < len(controller_times)
                and abs(controller_times[nearest_feedback_index + 1] - time_s)
                <= abs(controller_times[nearest_feedback_index] - time_s)
            ):
                nearest_feedback_index += 1
            if (
                abs(controller_times[nearest_feedback_index] - time_s)
                > match_tolerance + 1.0e-12
            ):
                raise ValidationError(
                    "sim_actual_joint.csv has no nearby controller feedback sample"
                )
            matched_feedback_positions = recorded_feedback_positions[
                nearest_feedback_index
            ]
            matched_feedback_velocities = recorded_feedback_velocities[
                nearest_feedback_index
            ]
            maximum_actual_controller_position_error = max(
                maximum_actual_controller_position_error,
                *(
                    abs(actual - expected)
                    for actual, expected in zip(
                        positions,
                        matched_feedback_positions,
                        strict=True,
                    )
                ),
            )
            maximum_actual_controller_velocity_error = max(
                maximum_actual_controller_velocity_error,
                *(
                    abs(actual - expected)
                    for actual, expected in zip(
                        velocities,
                        matched_feedback_velocities,
                        strict=True,
                    )
                ),
            )
        if (
            maximum_actual_controller_position_error
            > config.max_actual_controller_position_error_rad + 1.0e-12
        ):
            raise ValidationError(
                "simulation actual joint positions differ from controller feedback"
            )
        if (
            maximum_actual_controller_velocity_error
            > config.max_actual_controller_velocity_error_rad_s + 1.0e-12
        ):
            raise ValidationError(
                "simulation actual joint velocities differ from controller feedback"
            )

        required_frames = tuple(sorted(_EXPECTED_JOINT_FRAME_IDS))
        forward_kinematics: list[Mapping[str, Pose]] = []
        for row_number, positions in enumerate(actual_positions, start=2):
            try:
                forward_kinematics.append(
                    robot.kinematics.forward_kinematics(
                        positions,
                        required_frames,
                        tool_to_tip=robot.tool_to_tip,
                        tool_frame=robot.tool_frame,
                        tip_frame=robot.tip_frame,
                    )
                )
            except ValueError as error:
                raise ValidationError(
                    f"sim_actual_joint.csv row {row_number} cannot produce sealed FK: {error}"
                ) from error

        maximum_frame_fk_position_error = 0.0
        maximum_frame_fk_orientation_error = 0.0
        for frame_id in required_frames:
            frame_rows_for_id = frame_stream_rows[frame_id]
            if len(frame_rows_for_id) != len(actual_times):
                raise ValidationError(
                    f"joint_frames.csv frame {frame_id} sample count does not match actual joints"
                )
            for sample_index, (frame_row, actual_time) in enumerate(
                zip(frame_rows_for_id, actual_times, strict=True),
                start=2,
            ):
                frame_time = cls._parse_csv_float(
                    frame_row[1],
                    f"joint_frames.csv frame {frame_id} row {sample_index} trajectory_time_s",
                )
                if abs(frame_time - actual_time) > 1.0e-12:
                    raise ValidationError(
                        f"joint_frames.csv frame {frame_id} is not time-aligned with actual joints"
                    )
                recorded_pose = cls._pose_from_csv_row(
                    frame_row,
                    translation_offset=3,
                    quaternion_offset=6,
                    label=f"joint_frames.csv frame {frame_id} row {sample_index}",
                )
                frame_error = pose_error(
                    recorded_pose,
                    forward_kinematics[sample_index - 2][frame_id],
                )
                maximum_frame_fk_position_error = max(
                    maximum_frame_fk_position_error,
                    frame_error.position_m,
                )
                maximum_frame_fk_orientation_error = max(
                    maximum_frame_fk_orientation_error,
                    frame_error.orientation_deg,
                )
        if (
            maximum_frame_fk_position_error
            > config.max_frame_fk_position_error_m + 1.0e-12
        ):
            raise ValidationError("joint_frames.csv position differs from sealed URDF FK")
        if (
            maximum_frame_fk_orientation_error
            > config.max_frame_fk_orientation_error_deg + 1.0e-12
        ):
            raise ValidationError("joint_frames.csv orientation differs from sealed URDF FK")

        if len(tip_rows) != len(actual_times):
            raise ValidationError(
                "sim_actual_tip.csv sample count does not match actual joints"
            )
        maximum_tip_fk_position_error = 0.0
        maximum_tip_fk_orientation_error = 0.0
        recorded_tip_times: list[float] = []
        recorded_tip_poses: list[Pose] = []
        for sample_index, (row, actual_time) in enumerate(
            zip(tip_rows, actual_times, strict=True),
            start=2,
        ):
            tip_time = cls._parse_csv_float(
                row[1],
                f"sim_actual_tip.csv row {sample_index} trajectory_time_s",
            )
            if abs(tip_time - actual_time) > 1.0e-12:
                raise ValidationError(
                    "sim_actual_tip.csv is not time-aligned with actual joints"
                )
            recorded_tip = cls._pose_from_csv_row(
                row,
                translation_offset=2,
                quaternion_offset=5,
                label=f"sim_actual_tip.csv row {sample_index}",
            )
            recorded_tip_times.append(tip_time)
            recorded_tip_poses.append(recorded_tip)
            tip_fk_error = pose_error(
                recorded_tip,
                forward_kinematics[sample_index - 2][robot.tip_frame],
            )
            maximum_tip_fk_position_error = max(
                maximum_tip_fk_position_error,
                tip_fk_error.position_m,
            )
            maximum_tip_fk_orientation_error = max(
                maximum_tip_fk_orientation_error,
                tip_fk_error.orientation_deg,
            )
        if maximum_tip_fk_position_error > config.max_frame_fk_position_error_m + 1.0e-12:
            raise ValidationError("sim_actual_tip.csv position differs from sealed URDF FK")
        if (
            maximum_tip_fk_orientation_error
            > config.max_frame_fk_orientation_error_deg + 1.0e-12
        ):
            raise ValidationError("sim_actual_tip.csv orientation differs from sealed URDF FK")

        planner_position_tolerance, planner_orientation_tolerance = planner_tip_tolerances
        maximum_tip_source_position_error = 0.0
        maximum_tip_source_orientation_error = 0.0
        for source_pose in source:
            # The recorder cadence is independent from the authoritative source grid. Compare
            # at the exact source timestamp so a nearest-sample phase offset cannot consume the
            # fixed Cartesian tolerance. This changes neither the 1 mm limit nor any input time.
            actual_tip_at_source_time = cls._interpolate_pose_stream_at_time(
                recorded_tip_times,
                recorded_tip_poses,
                source_pose.time_s,
                match_tolerance,
            )
            tip_source_error = pose_error(
                actual_tip_at_source_time,
                cls._source_pose_in_base(source_pose, robot),
            )
            maximum_tip_source_position_error = max(
                maximum_tip_source_position_error,
                tip_source_error.position_m,
            )
            maximum_tip_source_orientation_error = max(
                maximum_tip_source_orientation_error,
                tip_source_error.orientation_deg,
            )
        if maximum_tip_source_position_error > planner_position_tolerance + 1.0e-12:
            raise ValidationError(
                "time-aligned actual rod_tip position differs from the source trajectory"
            )
        if maximum_tip_source_orientation_error > planner_orientation_tolerance + 1.0e-12:
            raise ValidationError(
                "time-aligned actual rod_tip orientation differs from the source trajectory"
            )

        return _SimulationMetrics(
            actual_joint=actual_metrics,
            controller=controller_metrics,
            joint_frame_sample_count=len(frame_rows),
            joint_frame_streams=joint_frame_metrics,
            actual_tip=actual_tip_metrics,
            max_joint_frame_quaternion_norm_error=maximum_joint_frame_quaternion_error,
            max_speed_scaling_error=maximum_speed_scaling_error,
            max_controller_reference_position_error_rad=maximum_reference_position_error,
            max_controller_reference_velocity_error_rad_s=maximum_reference_velocity_error,
            max_joint_position_tracking_error_rad=maximum_position_tracking_error,
            max_joint_velocity_tracking_error_rad_s=maximum_velocity_tracking_error,
            max_actual_controller_position_error_rad=(
                maximum_actual_controller_position_error
            ),
            max_actual_controller_velocity_error_rad_s=(
                maximum_actual_controller_velocity_error
            ),
            max_frame_fk_position_error_m=maximum_frame_fk_position_error,
            max_frame_fk_orientation_error_deg=maximum_frame_fk_orientation_error,
            max_tip_fk_position_error_m=maximum_tip_fk_position_error,
            max_tip_fk_orientation_error_deg=maximum_tip_fk_orientation_error,
            max_tip_source_position_error_m=maximum_tip_source_position_error,
            max_tip_source_orientation_error_deg=maximum_tip_source_orientation_error,
        )

    @classmethod
    def _validate_simulation_evidence(
        cls,
        report: Mapping[str, Any],
        recording_document: Mapping[str, Any],
        trajectory: CompiledTrajectory,
        source: Sequence[TimedPose],
        robot: _RobotAuditContext,
        planner_tip_tolerances: tuple[float, float],
        telemetry_bytes: Mapping[str, bytes],
    ) -> None:
        config = cls._load_simulation_validation_config(recording_document)
        metrics = cls._recompute_simulation_metrics(
            telemetry_bytes,
            trajectory,
            config,
            source,
            robot,
            planner_tip_tolerances,
        )

        def require_int(key: str, expected: int, label: str = "validation_report") -> None:
            if _require_int(report, key, label) != expected:
                raise ValidationError(f"{label}.{key} does not match sealed telemetry")

        def require_float(
            key: str,
            expected: float,
            label: str = "validation_report",
        ) -> None:
            if not _float_matches(_require_float(report, key, label), expected):
                raise ValidationError(f"{label}.{key} does not match sealed telemetry")

        def require_audit_float(
            key: str,
            expected: float,
            label: str = "validation_report",
        ) -> None:
            if not _audit_metric_matches(_require_float(report, key, label), expected):
                raise ValidationError(f"{label}.{key} does not match sealed telemetry")

        def require_orientation_audit_float(
            key: str,
            expected: float,
            label: str = "validation_report",
        ) -> None:
            if not _orientation_audit_metric_matches(
                _require_float(report, key, label),
                expected,
            ):
                raise ValidationError(f"{label}.{key} does not match sealed telemetry")

        require_float("sample_dt_s", config.sample_dt_s)
        require_int("max_missing_samples_allowed", config.max_missing_samples)
        require_float("max_time_jitter_s_allowed", config.max_time_jitter_s)
        if (
            not _require_bool(report, "require_action_success", "validation_report")
            or not config.require_action_success
        ):
            raise ValidationError(
                "validation_report and recording.yaml must require action success"
            )

        def require_sampling(prefix: str, actual: _SamplingMetrics) -> None:
            require_int(f"{prefix}_sample_count", actual.sample_count)
            require_float(
                f"{prefix}_first_trajectory_time_s",
                actual.first_trajectory_time_s,
            )
            require_float(
                f"{prefix}_last_trajectory_time_s",
                actual.last_trajectory_time_s,
            )
            require_float(f"{prefix}_coverage_duration_s", actual.coverage_duration_s)
            require_float(f"{prefix}_max_time_interval_s", actual.max_time_interval_s)
            require_float(f"{prefix}_max_time_jitter_s", actual.max_time_jitter_s)
            require_int(
                f"{prefix}_missing_sample_estimate",
                actual.missing_sample_estimate,
            )

        require_sampling("actual_joint", metrics.actual_joint)
        require_sampling("controller", metrics.controller)
        require_int("joint_frame_sample_count", metrics.joint_frame_sample_count)
        require_int("joint_frame_stream_count", len(metrics.joint_frame_streams))
        require_float(
            "max_joint_frame_quaternion_norm_error",
            metrics.max_joint_frame_quaternion_norm_error,
        )

        raw_frame_reports = report.get("joint_frame_streams")
        if (
            not isinstance(raw_frame_reports, Sequence)
            or isinstance(raw_frame_reports, (str, bytes))
        ):
            raise ValidationError("validation_report.joint_frame_streams must be a sequence")
        frame_reports: dict[str, Mapping[str, Any]] = {}
        for index, raw_frame in enumerate(raw_frame_reports):
            frame = _require_mapping(
                raw_frame,
                f"validation_report.joint_frame_streams[{index}]",
            )
            frame_id = _require_string(
                frame,
                "frame_id",
                f"validation_report.joint_frame_streams[{index}]",
            )
            if frame_id in frame_reports:
                raise ValidationError(
                    "validation_report.joint_frame_streams contains a duplicate frame_id"
                )
            frame_reports[frame_id] = frame
        if set(frame_reports) != _EXPECTED_JOINT_FRAME_IDS:
            raise ValidationError(
                "validation_report.joint_frame_streams must contain exactly eight required frames"
            )
        for frame_id, actual in metrics.joint_frame_streams.items():
            frame = frame_reports[frame_id]
            label = f"validation_report.joint_frame_streams[{frame_id}]"
            for key, expected_int in (
                ("sample_count", actual.sampling.sample_count),
                ("missing_sample_estimate", actual.sampling.missing_sample_estimate),
            ):
                if _require_int(frame, key, label) != expected_int:
                    raise ValidationError(f"{label}.{key} does not match sealed telemetry")
            for key, expected_float in (
                ("first_trajectory_time_s", actual.sampling.first_trajectory_time_s),
                ("last_trajectory_time_s", actual.sampling.last_trajectory_time_s),
                ("coverage_duration_s", actual.sampling.coverage_duration_s),
                ("max_time_interval_s", actual.sampling.max_time_interval_s),
                ("max_time_jitter_s", actual.sampling.max_time_jitter_s),
                ("max_quaternion_norm_error", actual.max_quaternion_norm_error),
            ):
                if not _float_matches(_require_float(frame, key, label), expected_float):
                    raise ValidationError(f"{label}.{key} does not match sealed telemetry")

        require_sampling("actual_tip", metrics.actual_tip.sampling)
        require_float(
            "actual_tip_max_quaternion_norm_error",
            metrics.actual_tip.max_quaternion_norm_error,
        )
        require_float(
            "max_quaternion_norm_error_allowed",
            config.max_quaternion_norm_error,
        )
        require_float(
            "max_completion_overrun_s_allowed",
            config.max_completion_overrun_s,
        )
        require_float("max_speed_scaling_error", metrics.max_speed_scaling_error)
        require_float(
            "max_controller_reference_position_error_rad",
            metrics.max_controller_reference_position_error_rad,
        )
        require_float(
            "max_controller_reference_position_error_rad_allowed",
            config.max_controller_reference_position_error_rad,
        )
        require_float(
            "max_controller_reference_velocity_error_rad_s",
            metrics.max_controller_reference_velocity_error_rad_s,
        )
        require_float(
            "max_controller_reference_velocity_error_rad_s_allowed",
            config.max_controller_reference_velocity_error_rad_s,
        )
        require_float(
            "max_joint_position_tracking_error_rad",
            metrics.max_joint_position_tracking_error_rad,
        )
        require_float(
            "max_joint_position_tracking_error_rad_allowed",
            config.max_joint_position_tracking_error_rad,
        )
        require_float(
            "max_joint_velocity_tracking_error_rad_s",
            metrics.max_joint_velocity_tracking_error_rad_s,
        )
        require_float(
            "max_joint_velocity_tracking_error_rad_s_allowed",
            config.max_joint_velocity_tracking_error_rad_s,
        )
        require_audit_float(
            "max_actual_controller_position_error_rad",
            metrics.max_actual_controller_position_error_rad,
        )
        require_float(
            "max_actual_controller_position_error_rad_allowed",
            config.max_actual_controller_position_error_rad,
        )
        require_audit_float(
            "max_actual_controller_velocity_error_rad_s",
            metrics.max_actual_controller_velocity_error_rad_s,
        )
        require_float(
            "max_actual_controller_velocity_error_rad_s_allowed",
            config.max_actual_controller_velocity_error_rad_s,
        )
        require_audit_float(
            "max_frame_fk_position_error_m",
            metrics.max_frame_fk_position_error_m,
        )
        require_float(
            "max_frame_fk_position_error_m_allowed",
            config.max_frame_fk_position_error_m,
        )
        require_orientation_audit_float(
            "max_frame_fk_orientation_error_deg",
            metrics.max_frame_fk_orientation_error_deg,
        )
        require_float(
            "max_frame_fk_orientation_error_deg_allowed",
            config.max_frame_fk_orientation_error_deg,
        )
        require_audit_float(
            "max_tip_fk_position_error_m",
            metrics.max_tip_fk_position_error_m,
        )
        require_float(
            "max_tip_fk_position_error_m_allowed",
            config.max_frame_fk_position_error_m,
        )
        require_orientation_audit_float(
            "max_tip_fk_orientation_error_deg",
            metrics.max_tip_fk_orientation_error_deg,
        )
        require_float(
            "max_tip_fk_orientation_error_deg_allowed",
            config.max_frame_fk_orientation_error_deg,
        )
        require_audit_float(
            "max_tip_source_position_error_m",
            metrics.max_tip_source_position_error_m,
        )
        require_float(
            "max_tip_source_position_error_m_allowed",
            planner_tip_tolerances[0],
        )
        require_orientation_audit_float(
            "max_tip_source_orientation_error_deg",
            metrics.max_tip_source_orientation_error_deg,
        )
        require_float(
            "max_tip_source_orientation_error_deg_allowed",
            planner_tip_tolerances[1],
        )

    @staticmethod
    def _planner_tip_tolerances(
        planner: Mapping[str, Any],
    ) -> tuple[float, float]:
        if _require_int(planner, "schema_version", "snapshot.planner") != 1:
            raise ValidationError("snapshot planner.yaml schema_version must be 1")
        validation = _require_mapping(
            planner.get("validation"),
            "snapshot.planner.validation",
        )
        position_tolerance = _require_float(
            validation,
            "tip_position_tolerance_m",
            "snapshot.planner.validation",
        )
        orientation_tolerance = _require_float(
            validation,
            "tip_orientation_tolerance_deg",
            "snapshot.planner.validation",
        )
        if position_tolerance < 0.0 or orientation_tolerance < 0.0:
            raise ValidationError("snapshot planner tip tolerances are invalid")
        return position_tolerance, orientation_tolerance

    @classmethod
    def _validate_planner_policy(
        cls,
        planner: Mapping[str, Any],
        evidence: Mapping[str, Any],
    ) -> None:
        position_tolerance, orientation_tolerance = cls._planner_tip_tolerances(
            planner
        )
        limits = _require_mapping(planner.get("limits"), "snapshot.planner.limits")
        if _require_bool(
            limits,
            "allow_uniform_time_scale",
            "snapshot.planner.limits",
        ):
            raise ValidationError("snapshot planner must forbid uniform time scaling")
        if (
            _require_string(limits, "on_violation", "snapshot.planner.limits")
            != "reject"
        ):
            raise ValidationError("snapshot planner must reject limit violations")
        maximum_scale = _require_float(
            limits,
            "max_uniform_time_scale",
            "snapshot.planner.limits",
        )
        if abs(maximum_scale - 1.0) > _TIME_TOLERANCE:
            raise ValidationError("snapshot planner max_uniform_time_scale must be 1.0")

        validation = _require_mapping(
            planner.get("validation"),
            "snapshot.planner.validation",
        )
        minimum_clearance = _require_float(
            validation,
            "minimum_collision_distance_m",
            "snapshot.planner.validation",
        )
        if min(position_tolerance, orientation_tolerance, minimum_clearance) < 0.0:
            raise ValidationError("snapshot planner validation thresholds are invalid")
        metrics = evidence["metrics"]
        if metrics["max_tip_position_error_m"] > position_tolerance + _TIME_TOLERANCE:
            raise ValidationError("compile_report tip position error exceeds planner tolerance")
        if (
            metrics["max_tip_orientation_error_deg"]
            > orientation_tolerance + _TIME_TOLERANCE
        ):
            raise ValidationError(
                "compile_report tip orientation error exceeds planner tolerance"
            )
        if (
            metrics["minimum_collision_distance_m"] + _TIME_TOLERANCE
            < minimum_clearance
        ):
            raise ValidationError(
                "compile_report minimum collision distance is below planner threshold"
            )
