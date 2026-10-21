from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite, sqrt
from typing import Any, Mapping, Sequence

from .errors import TrajectoryFormatError


PHASES = ("approach", "engage", "draw", "hold", "disengage", "exit")


def _finite_tuple(values: Sequence[float], size: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size:
        raise TrajectoryFormatError(f"{name} must have {size} elements, got {len(result)}")
    if not all(isfinite(value) for value in result):
        raise TrajectoryFormatError(f"{name} contains a non-finite value")
    return result


@dataclass(frozen=True)
class TimedPose:
    time_s: float
    frame_id: str
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    phase: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", float(self.time_s))
        if not isinstance(self.frame_id, str):
            raise TrajectoryFormatError("frame_id must be a non-empty ROS frame name")
        if (
            not self.frame_id
            or self.frame_id != self.frame_id.strip()
            or any(character.isspace() for character in self.frame_id)
        ):
            raise TrajectoryFormatError("frame_id must be a non-empty ROS frame name")
        object.__setattr__(self, "position_m", _finite_tuple(self.position_m, 3, "position_m"))
        quaternion = _finite_tuple(self.quaternion_xyzw, 4, "quaternion_xyzw")
        norm = sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1.0e-3:
            raise TrajectoryFormatError(
                f"quaternion_xyzw must be normalized (norm={norm:.8f})"
            )
        # Validate only.  Source Pose values are authoritative and must never be
        # silently normalized or otherwise rewritten by the data model.
        object.__setattr__(self, "quaternion_xyzw", quaternion)
        if not isinstance(self.phase, str) or self.phase not in PHASES:
            raise TrajectoryFormatError(
                f"phase must be one of {', '.join(PHASES)}, got {self.phase!r}"
            )
        if not isfinite(self.time_s) or self.time_s < 0.0:
            raise TrajectoryFormatError("time_s must be finite and non-negative")


@dataclass(frozen=True)
class IKCandidate:
    positions: tuple[float, ...]
    collision_distance_m: float
    joint_limit_margin_rad: float
    singularity_score: float

    def __post_init__(self) -> None:
        positions = tuple(float(value) for value in self.positions)
        if not positions or not all(isfinite(value) for value in positions):
            raise TrajectoryFormatError("IK candidate positions must be finite")
        object.__setattr__(self, "positions", positions)
        for name in (
            "collision_distance_m",
            "joint_limit_margin_rad",
            "singularity_score",
        ):
            value = float(getattr(self, name))
            if not isfinite(value):
                raise TrajectoryFormatError(f"{name} must be finite")
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class JointSample:
    time_s: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]
    phase: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "time_s", float(self.time_s))
        positions = tuple(float(value) for value in self.positions)
        velocities = tuple(float(value) for value in self.velocities)
        accelerations = tuple(float(value) for value in self.accelerations)
        width = len(positions)
        if width == 0 or len(velocities) != width or len(accelerations) != width:
            raise TrajectoryFormatError("q, qd, and qdd must have the same non-zero width")
        if not all(isfinite(value) for values in (positions, velocities, accelerations) for value in values):
            raise TrajectoryFormatError("joint sample contains a non-finite value")
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "velocities", velocities)
        object.__setattr__(self, "accelerations", accelerations)
        phase = self.phase.strip().lower()
        if phase not in PHASES:
            raise TrajectoryFormatError(f"invalid joint sample phase {self.phase!r}")
        object.__setattr__(self, "phase", phase)


@dataclass(frozen=True)
class CompiledTrajectory:
    joint_names: tuple[str, ...]
    samples: tuple[JointSample, ...]
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        names = tuple(str(name) for name in self.joint_names)
        samples = tuple(self.samples)
        if not names or len(set(names)) != len(names):
            raise TrajectoryFormatError("joint_names must be non-empty and unique")
        if len(samples) < 2:
            raise TrajectoryFormatError("compiled trajectory requires at least two samples")
        if samples[0].time_s != 0.0:
            raise TrajectoryFormatError("compiled trajectory must start at time_s=0")
        for index, sample in enumerate(samples):
            if len(sample.positions) != len(names):
                raise TrajectoryFormatError(
                    f"sample {index} width does not match joint_names"
                )
            if index and sample.time_s <= samples[index - 1].time_s:
                raise TrajectoryFormatError("compiled trajectory time must be strictly increasing")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "samples", samples)
        object.__setattr__(self, "metadata", dict(self.metadata))

    @property
    def duration_s(self) -> float:
        return self.samples[-1].time_s
