from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .environment_builder import EnvironmentBundle
from .errors import ConfigurationError
from .math3d import inverse_pose, matrix_pose, pose_matrix
from .models import TimedPose


@dataclass(frozen=True)
class CollisionSphere:
    name: str
    link_name: str
    center_m: tuple[float, float, float]
    radius_m: float


class RobotToolBuilder:
    """Build the tool0-to-tip transform and rod collision sphere chain."""

    def __init__(self, environment: EnvironmentBundle) -> None:
        self.environment = environment

    @property
    def parent_link(self) -> str:
        return str(self.environment.tcp["tool"]["parent_link"])

    @property
    def tip_frame(self) -> str:
        return str(self.environment.tcp["tool"]["tip_frame"])

    @property
    def tool0_to_tip(self) -> np.ndarray:
        tool = self.environment.tcp["tool"]
        return pose_matrix(tool["translation_m"], tool["quaternion_xyzw"])

    def tip_pose_to_tool_pose(self, tip_pose: TimedPose) -> TimedPose:
        base_to_tip = pose_matrix(tip_pose.position_m, tip_pose.quaternion_xyzw)
        base_to_tool = base_to_tip @ inverse_pose(self.tool0_to_tip)
        position, quaternion = matrix_pose(base_to_tool)
        return TimedPose(
            time_s=tip_pose.time_s,
            frame_id=tip_pose.frame_id,
            position_m=position,
            quaternion_xyzw=quaternion,
            phase=tip_pose.phase,
        )

    def collision_spheres(self) -> tuple[CollisionSphere, ...]:
        rod: Mapping[str, Any] = self.environment.tcp["rod"]
        length = float(rod["length_m"])
        radius = float(rod["radius_m"])
        spacing = float(rod.get("sphere_spacing_m", radius))
        if length <= 0.0 or radius <= 0.0 or spacing <= 0.0:
            raise ConfigurationError("rod length, radius, and sphere spacing must be positive")
        if spacing > 2.0 * radius:
            raise ConfigurationError("rod collision spheres must overlap or touch")
        count = max(2, int(np.ceil(length / spacing)) + 1)
        tip_translation = self.tool0_to_tip[:3, 3]
        direction = tip_translation
        norm = float(np.linalg.norm(direction))
        if norm < 1.0e-9:
            raise ConfigurationError("tool translation must locate the rod tip away from tool0")
        unit = direction / norm
        usable_length = min(length, norm)
        centers = np.linspace(radius, max(radius, usable_length - radius), count)
        return tuple(
            CollisionSphere(
                name=f"rod_{index:03d}",
                link_name=self.parent_link,
                center_m=tuple(float(value) for value in unit * distance),
                radius_m=radius,
            )
            for index, distance in enumerate(centers)
        )
