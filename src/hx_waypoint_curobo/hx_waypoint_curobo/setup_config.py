from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_SAFE_FRAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_VECTOR_TOLERANCE = 1.0e-9


class SetupConfigurationError(ValueError):
    """Raised when a run setup cannot be interpreted without guessing."""


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SetupConfigurationError(f"{label} must be a mapping")
    return value


def _strict_fields(mapping: Mapping[str, Any], allowed: set[str], label: str) -> None:
    unknown = sorted(str(key) for key in mapping if key not in allowed)
    if unknown:
        raise SetupConfigurationError(
            f"{label} contains unsupported fields: {', '.join(unknown)}"
        )


def _string(value: Any, label: str, *, pattern: re.Pattern[str]) -> str:
    if not isinstance(value, str) or value.strip() != value or not pattern.fullmatch(value):
        raise SetupConfigurationError(f"{label} is not a valid identifier")
    return value


def _finite_vector(value: Any, size: int, label: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != size:
        raise SetupConfigurationError(f"{label} must contain exactly {size} finite numbers")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise SetupConfigurationError(f"{label} must contain exactly {size} finite numbers")
        number = float(item)
        if not math.isfinite(number):
            raise SetupConfigurationError(f"{label} must contain exactly {size} finite numbers")
        result.append(number)
    return tuple(result)


def _positive(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SetupConfigurationError(f"{label} must be a finite positive number")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise SetupConfigurationError(f"{label} must be a finite positive number")
    return number


def _normalized_quaternion(value: Any, label: str) -> tuple[float, float, float, float]:
    quaternion = _finite_vector(value, 4, label)
    norm = math.sqrt(sum(component * component for component in quaternion))
    if abs(norm - 1.0) > _VECTOR_TOLERANCE:
        raise SetupConfigurationError(f"{label} must be normalized")
    return (
        quaternion[0] / norm,
        quaternion[1] / norm,
        quaternion[2] / norm,
        quaternion[3] / norm,
    )


def _unit_vector(value: Any, label: str) -> tuple[float, float, float]:
    vector = _finite_vector(value, 3, label)
    norm = math.sqrt(sum(component * component for component in vector))
    if abs(norm - 1.0) > _VECTOR_TOLERANCE:
        raise SetupConfigurationError(f"{label} must be a unit vector")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def rotvec_to_quaternion_xyzw(
    rotation_vector_rad: Sequence[float],
) -> tuple[float, float, float, float]:
    """Convert a UR rotation vector to a normalized ROS xyzw quaternion."""

    vector = _finite_vector(rotation_vector_rad, 3, "rotation_vector_rad")
    angle = math.sqrt(sum(component * component for component in vector))
    if angle <= 1.0e-15:
        return (0.0, 0.0, 0.0, 1.0)
    scale = math.sin(0.5 * angle) / angle
    quaternion = (
        vector[0] * scale,
        vector[1] * scale,
        vector[2] * scale,
        math.cos(0.5 * angle),
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def _quaternion_product_xyzw(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    result = (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )
    norm = math.sqrt(sum(component * component for component in result))
    if norm <= 1.0e-15:
        raise SetupConfigurationError("composed quaternion is degenerate")
    return tuple(component / norm for component in result)  # type: ignore[return-value]


def _rotate_vector(
    quaternion_xyzw: Sequence[float],
    vector: Sequence[float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quaternion_xyzw
    vx, vy, vz = vector
    # Unit-quaternion vector rotation without constructing temporary matrices.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


@dataclass(frozen=True)
class RigidTransform:
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def compose(self, child: RigidTransform) -> RigidTransform:
        rotated = _rotate_vector(self.quaternion_xyzw, child.translation_m)
        return RigidTransform(
            translation_m=tuple(
                self.translation_m[index] + rotated[index] for index in range(3)
            ),  # type: ignore[arg-type]
            quaternion_xyzw=_quaternion_product_xyzw(
                self.quaternion_xyzw,
                child.quaternion_xyzw,
            ),
        )


@dataclass(frozen=True)
class WorkFrameConfig:
    parent_frame: str
    child_frame: str
    teach_pendant_pose_xyz_rotvec: tuple[float, float, float, float, float, float]
    aliases: tuple[str, ...]
    feature_to_work_frame_translation_m: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def parent_to_child(self) -> RigidTransform:
        parent_to_feature = RigidTransform(
            translation_m=self.teach_pendant_pose_xyz_rotvec[:3],
            quaternion_xyzw=rotvec_to_quaternion_xyzw(
                self.teach_pendant_pose_xyz_rotvec[3:]
            ),
        )
        return parent_to_feature.compose(
            RigidTransform(
                translation_m=self.feature_to_work_frame_translation_m,
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        )


@dataclass(frozen=True)
class ToolConfig:
    tool_id: str
    parent_link: str
    tip_frame: str
    tip_orientation_xyzw: tuple[float, float, float, float]
    axis_in_parent_xyz: tuple[float, float, float]
    length_m: float
    radius_m: float
    sphere_spacing_m: float

    @property
    def parent_to_tip(self) -> RigidTransform:
        return RigidTransform(
            translation_m=tuple(
                component * self.length_m for component in self.axis_in_parent_xyz
            ),  # type: ignore[arg-type]
            quaternion_xyzw=self.tip_orientation_xyzw,
        )

    def legacy_tcp_mapping(self) -> dict[str, Any]:
        """Expose the existing internal TCP view without retaining a second file."""

        transform = self.parent_to_tip
        return {
            "schema_version": 1,
            "tool_id": self.tool_id,
            "tool": {
                "parent_link": self.parent_link,
                "tip_frame": self.tip_frame,
                "translation_m": list(transform.translation_m),
                "quaternion_xyzw": list(transform.quaternion_xyzw),
            },
            "rod": {
                "length_m": self.length_m,
                "radius_m": self.radius_m,
                "sphere_spacing_m": self.sphere_spacing_m,
            },
        }


@dataclass(frozen=True)
class SetupConfig:
    setup_id: str
    work_frame: WorkFrameConfig
    tool: ToolConfig

    def resolved_frames_mapping(
        self,
        *,
        base_frame: str,
        base_to_parent: RigidTransform,
    ) -> dict[str, Any]:
        base_to_work = base_to_parent.compose(self.work_frame.parent_to_child)
        definitions: dict[str, Any] = {
            self.work_frame.child_frame: {
                "parent": base_frame,
                "translation_m": list(base_to_work.translation_m),
                "quaternion_xyzw": list(base_to_work.quaternion_xyzw),
            }
        }
        for alias in self.work_frame.aliases:
            definitions[alias] = {
                "parent": self.work_frame.child_frame,
                "translation_m": [0.0, 0.0, 0.0],
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        return {
            "schema_version": 1,
            "base_frame": base_frame,
            "frames": definitions,
        }


def parse_setup(value: Any, *, label: str = "setup.yaml") -> SetupConfig:
    root = _mapping(value, label)
    _strict_fields(root, {"schema_version", "setup_id", "work_frame", "tool"}, label)
    if type(root.get("schema_version")) is not int or root["schema_version"] != 1:
        raise SetupConfigurationError(f"{label}: schema_version must be integer 1")
    setup_id = _string(root.get("setup_id"), f"{label}.setup_id", pattern=_SAFE_ID)

    work = _mapping(root.get("work_frame"), f"{label}.work_frame")
    _strict_fields(
        work,
        {
            "parent_frame",
            "child_frame",
            "teach_pendant_pose_xyz_rotvec",
            "feature_to_work_frame_translation_m",
            "aliases",
        },
        f"{label}.work_frame",
    )
    parent_frame = _string(
        work.get("parent_frame"),
        f"{label}.work_frame.parent_frame",
        pattern=_SAFE_FRAME,
    )
    child_frame = _string(
        work.get("child_frame"),
        f"{label}.work_frame.child_frame",
        pattern=_SAFE_FRAME,
    )
    if parent_frame != "base" or child_frame != "plane1":
        raise SetupConfigurationError(
            f"{label}.work_frame must describe the authoritative base -> plane1 transform"
        )
    pose = _finite_vector(
        work.get("teach_pendant_pose_xyz_rotvec"),
        6,
        f"{label}.work_frame.teach_pendant_pose_xyz_rotvec",
    )
    feature_to_work_frame_translation = _finite_vector(
        work.get("feature_to_work_frame_translation_m", [0.0, 0.0, 0.0]),
        3,
        f"{label}.work_frame.feature_to_work_frame_translation_m",
    )
    raw_aliases = work.get("aliases", [])
    if isinstance(raw_aliases, (str, bytes)) or not isinstance(raw_aliases, Sequence):
        raise SetupConfigurationError(f"{label}.work_frame.aliases must be a sequence")
    aliases = tuple(
        _string(item, f"{label}.work_frame.aliases", pattern=_SAFE_FRAME)
        for item in raw_aliases
    )
    if len(set(aliases)) != len(aliases) or child_frame in aliases or parent_frame in aliases:
        raise SetupConfigurationError(
            f"{label}.work_frame.aliases must be unique and distinct from base/plane1"
        )

    tool = _mapping(root.get("tool"), f"{label}.tool")
    _strict_fields(
        tool,
        {"tool_id", "parent_link", "tip_frame", "tip_orientation_xyzw", "rod"},
        f"{label}.tool",
    )
    tool_id = _string(tool.get("tool_id"), f"{label}.tool.tool_id", pattern=_SAFE_ID)
    parent_link = _string(
        tool.get("parent_link"),
        f"{label}.tool.parent_link",
        pattern=_SAFE_FRAME,
    )
    tip_frame = _string(
        tool.get("tip_frame"),
        f"{label}.tool.tip_frame",
        pattern=_SAFE_FRAME,
    )
    if parent_link != "tool0" or tip_frame != "rod_tip":
        raise SetupConfigurationError(
            f"{label}.tool must use the controlled tool0 -> rod_tip frame contract"
        )
    tip_orientation = _normalized_quaternion(
        tool.get("tip_orientation_xyzw"),
        f"{label}.tool.tip_orientation_xyzw",
    )
    rod = _mapping(tool.get("rod"), f"{label}.tool.rod")
    _strict_fields(
        rod,
        {"axis_in_parent_xyz", "length_m", "radius_m", "sphere_spacing_m"},
        f"{label}.tool.rod",
    )
    axis = _unit_vector(
        rod.get("axis_in_parent_xyz"),
        f"{label}.tool.rod.axis_in_parent_xyz",
    )
    length = _positive(rod.get("length_m"), f"{label}.tool.rod.length_m")
    radius = _positive(rod.get("radius_m"), f"{label}.tool.rod.radius_m")
    spacing = _positive(
        rod.get("sphere_spacing_m"),
        f"{label}.tool.rod.sphere_spacing_m",
    )
    if length <= 2.0 * radius:
        raise SetupConfigurationError(f"{label}.tool.rod length must exceed its diameter")
    if spacing > 2.0 * radius:
        raise SetupConfigurationError(
            f"{label}.tool.rod collision spheres must overlap or touch"
        )

    return SetupConfig(
        setup_id=setup_id,
        work_frame=WorkFrameConfig(
            parent_frame=parent_frame,
            child_frame=child_frame,
            teach_pendant_pose_xyz_rotvec=pose,  # type: ignore[arg-type]
            aliases=aliases,
            feature_to_work_frame_translation_m=feature_to_work_frame_translation,  # type: ignore[arg-type]
        ),
        tool=ToolConfig(
            tool_id=tool_id,
            parent_link=parent_link,
            tip_frame=tip_frame,
            tip_orientation_xyzw=tip_orientation,
            axis_in_parent_xyz=axis,
            length_m=length,
            radius_m=radius,
            sphere_spacing_m=spacing,
        ),
    )


def load_setup_file(path: str | Path) -> SetupConfig:
    setup_path = Path(path)
    try:
        value = yaml.safe_load(setup_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise SetupConfigurationError(f"cannot load {setup_path}: {error}") from error
    return parse_setup(value, label=setup_path.name)
