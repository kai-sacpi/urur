from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
from typing import Sequence

import numpy as np
import pytest

from hx_waypoint_curobo.curobo_adapter_v2 import JointPath, PathValidationMetrics
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder, EnvironmentBundle
from hx_waypoint_curobo.models import IKCandidate, TimedPose


def copy_environment_without_recordings(source: Path, destination: Path) -> Path:
    """Copy an environment fixture without duplicating mutable recording payloads."""

    shutil.copytree(
        source,
        destination,
        ignore=shutil.ignore_patterns("recordings"),
    )
    (destination / "recordings").mkdir()
    return destination


@dataclass
class FakeCuroboAdapter:
    environment: EnvironmentBundle
    delay_s: float = 0.0
    warmup_calls: int = 0
    ik_calls: int = 0
    optimize_calls: int = 0
    cspace_calls: int = 0
    joint_validation_calls: int = 0
    compiled_validation_constraint_count: int = 0

    @property
    def revision(self) -> str:
        return "a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9"

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.environment.joint_names

    def warmup(self) -> None:
        import time

        self.warmup_calls += 1
        if self.delay_s:
            time.sleep(self.delay_s)

    def solve_ik_candidates(
        self,
        tool_poses: Sequence[TimedPose],
        candidates_per_pose: int,
    ) -> tuple[tuple[IKCandidate, ...], ...]:
        self.ik_calls += 1
        home = np.asarray(self.environment.robot_model["robot"]["home_joint_positions"])
        output = []
        for pose in tool_poses:
            p = np.asarray(pose.position_m)
            base = home + np.array(
                [p[0], p[1], p[2], p[0] - p[1], p[1] - p[2], p[2] - p[0]]
            ) * 0.005
            output.append(
                tuple(
                    IKCandidate(
                        positions=tuple(base + seed * 1.0e-4),
                        collision_distance_m=0.02,
                        joint_limit_margin_rad=1.0,
                        singularity_score=float(seed),
                    )
                    for seed in range(min(candidates_per_pose, 3))
                )
            )
        return tuple(output)

    def optimize_pose_path(
        self,
        tool_poses: Sequence[TimedPose],
        authoritative_times_s: Sequence[float],
        seed_positions: Sequence[Sequence[float]],
    ) -> JointPath:
        self.optimize_calls += 1
        assert len(tool_poses) == len(seed_positions)
        return JointPath(
            times_s=tuple(float(value) for value in authoritative_times_s),
            positions=tuple(tuple(float(value) for value in row) for row in seed_positions),
        )

    def plan_cspace(
        self,
        start_positions: Sequence[float],
        goal_positions: Sequence[float],
    ) -> JointPath:
        self.cspace_calls += 1
        start = np.asarray(start_positions, dtype=float)
        goal = np.asarray(goal_positions, dtype=float)
        count = 21
        return JointPath(
            times_s=tuple(np.linspace(0.0, 2.0, count)),
            positions=tuple(
                tuple(float(value) for value in ((1.0 - ratio) * start + ratio * goal))
                for ratio in np.linspace(0.0, 1.0, count)
            ),
        )

    def validate_compiled_trajectory(
        self,
        trajectory,
        constrained_tip_poses,
        constrained_positions,
    ) -> PathValidationMetrics:
        assert len(constrained_tip_poses) == len(constrained_positions)
        self.compiled_validation_constraint_count = len(constrained_tip_poses)
        return PathValidationMetrics(
            max_tip_position_error_m=0.0,
            max_tip_orientation_error_deg=0.0,
            minimum_collision_distance_m=0.02,
            max_scene_collision_cost=0.0,
            max_self_collision_cost=0.0,
            max_joint_bound_cost=0.0,
            checked_sample_count=len(trajectory.samples),
        )

    def validate_joint_trajectory(self, trajectory) -> PathValidationMetrics:
        self.joint_validation_calls += 1
        return PathValidationMetrics(
            max_tip_position_error_m=0.0,
            max_tip_orientation_error_deg=0.0,
            minimum_collision_distance_m=0.02,
            max_scene_collision_cost=0.0,
            max_self_collision_cost=0.0,
            max_joint_bound_cost=0.0,
            checked_sample_count=len(trajectory.samples),
        )


@pytest.fixture
def repository_root() -> Path:
    return Path(__file__).resolve().parents[1]


@pytest.fixture
def environment(repository_root: Path) -> EnvironmentBundle:
    return EnvironmentBuilder().load(repository_root / "environments" / "lab_wall_a")
