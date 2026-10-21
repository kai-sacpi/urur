from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from .environment_builder import EnvironmentBundle
from .errors import ConfigurationError
from .math3d import matrix_pose, pose_matrix


_SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


class SceneConfigurationError(ConfigurationError):
    """Raised when MoveIt and cuRobo cannot share one exact scene."""


@dataclass(frozen=True)
class ResolvedCuboid:
    object_id: str
    source_frame_id: str
    base_frame_id: str
    dimensions_m: tuple[float, float, float]
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    @property
    def curobo_pose_wxyz(self) -> tuple[float, float, float, float, float, float, float]:
        qx, qy, qz, qw = self.quaternion_xyzw
        return (*self.position_m, qw, qx, qy, qz)


def _finite_vector(value: Any, size: int, field: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != size:
        raise SceneConfigurationError(f"{field} must contain exactly {size} numbers")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise SceneConfigurationError(f"{field} must contain only numbers")
        number = float(item)
        if not math.isfinite(number):
            raise SceneConfigurationError(f"{field} contains a non-finite number")
        result.append(number)
    return tuple(result)


def resolve_curobo_cuboids(environment: EnvironmentBundle) -> tuple[ResolvedCuboid, ...]:
    scene = environment.zones.get("curobo_scene")
    if not isinstance(scene, Mapping):
        raise SceneConfigurationError("zones.yaml curobo_scene must be a mapping")
    unsupported = sorted(str(key) for key in scene if key != "cuboid")
    if unsupported:
        raise SceneConfigurationError(
            "shared MoveIt/cuRobo scene does not support geometry: " + ", ".join(unsupported)
        )
    cuboid_mapping = scene.get("cuboid")
    if not isinstance(cuboid_mapping, Mapping) or not cuboid_mapping:
        raise SceneConfigurationError(
            "zones.yaml curobo_scene.cuboid must be a non-empty mapping"
        )

    cuboids: list[ResolvedCuboid] = []
    for raw_name in sorted(cuboid_mapping, key=str):
        if not isinstance(raw_name, str) or not _SAFE_IDENTIFIER.fullmatch(raw_name):
            raise SceneConfigurationError(f"invalid curobo_scene.cuboid object id: {raw_name!r}")
        raw_cuboid = cuboid_mapping[raw_name]
        if not isinstance(raw_cuboid, Mapping):
            raise SceneConfigurationError(
                f"curobo_scene.cuboid.{raw_name} must be a mapping"
            )
        unknown_fields = sorted(
            str(key) for key in raw_cuboid if key not in {"frame_id", "dims", "pose"}
        )
        if unknown_fields:
            raise SceneConfigurationError(
                f"curobo_scene.cuboid.{raw_name} contains unsupported fields: "
                + ", ".join(unknown_fields)
            )
        frame_id = raw_cuboid.get("frame_id")
        if (
            not isinstance(frame_id, str)
            or not frame_id
            or frame_id.strip() != frame_id
            or any(character.isspace() for character in frame_id)
        ):
            raise SceneConfigurationError(
                f"curobo_scene.cuboid.{raw_name}.frame_id must be a valid configured frame"
            )
        dimensions = _finite_vector(
            raw_cuboid.get("dims"),
            3,
            f"curobo_scene.cuboid.{raw_name}.dims",
        )
        if any(value <= 0.0 for value in dimensions):
            raise SceneConfigurationError(
                f"curobo_scene.cuboid.{raw_name}.dims must be strictly positive"
            )
        pose = _finite_vector(
            raw_cuboid.get("pose"),
            7,
            f"curobo_scene.cuboid.{raw_name}.pose",
        )
        qw, qx, qy, qz = pose[3:]
        norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if abs(norm - 1.0) > 1.0e-3:
            raise SceneConfigurationError(
                f"curobo_scene.cuboid.{raw_name}.pose quaternion must be normalized "
                f"(norm={norm:.8f})"
            )
        try:
            base_from_source = environment._frame_to_base(frame_id)
        except (KeyError, TypeError, ValueError, ConfigurationError) as error:
            raise SceneConfigurationError(
                f"curobo_scene.cuboid.{raw_name}.frame_id {frame_id!r} is not resolved by setup.yaml"
            ) from error
        source_from_object = pose_matrix(
            pose[:3],
            (qx / norm, qy / norm, qz / norm, qw / norm),
        )
        base_from_object = np.asarray(base_from_source, dtype=float) @ source_from_object
        position, quaternion = matrix_pose(base_from_object)
        cuboids.append(
            ResolvedCuboid(
                object_id=raw_name,
                source_frame_id=frame_id,
                base_frame_id=environment.base_frame,
                dimensions_m=(dimensions[0], dimensions[1], dimensions[2]),
                position_m=position,
                quaternion_xyzw=quaternion,
            )
        )
    return tuple(cuboids)


def resolved_curobo_scene(environment: EnvironmentBundle) -> dict[str, Any]:
    return {
        "cuboid": {
            cuboid.object_id: {
                "dims": list(cuboid.dimensions_m),
                "pose": list(cuboid.curobo_pose_wxyz),
            }
            for cuboid in resolve_curobo_cuboids(environment)
        }
    }
