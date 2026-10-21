from __future__ import annotations

from math import nan

import numpy as np
import pytest

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder, EnvironmentBundle
from hx_waypoint_curobo.curobo_adapter_v2 import JointPath
from hx_waypoint_curobo.errors import PlanningError, ValidationError
from hx_waypoint_curobo.models import CompiledTrajectory, JointSample
from hx_waypoint_curobo.prepare_planner import PreparePlanner
from hx_waypoint_curobo.trajectory_optimizer import TrajectoryOptimizer

from conftest import FakeCuroboAdapter


def test_prepare_planner_returns_complete_bounded_trajectory(
    environment: EnvironmentBundle,
) -> None:
    adapter = FakeCuroboAdapter(environment)
    planner = PreparePlanner(environment, adapter)
    current = tuple(float(value) for value in environment.robot_model["robot"]["home_joint_positions"])
    target = tuple(value + 0.02 for value in current)

    trajectory = planner.plan(environment.joint_names, current, target)

    assert adapter.cspace_calls == 1
    assert adapter.joint_validation_calls == 1
    assert trajectory.joint_names == environment.joint_names
    assert trajectory.samples[0].positions == pytest.approx(current, abs=1.0e-9)
    assert trajectory.samples[-1].positions == pytest.approx(target, abs=1.0e-9)
    assert trajectory.samples[0].time_s == 0.0
    assert trajectory.duration_s > 0.0
    assert trajectory.samples[0].velocities == pytest.approx((0.0,) * 6, abs=1.0e-12)
    assert trajectory.samples[-1].velocities == pytest.approx((0.0,) * 6, abs=1.0e-12)
    assert trajectory.samples[0].accelerations == pytest.approx((0.0,) * 6, abs=1.0e-12)
    assert trajectory.samples[-1].accelerations == pytest.approx((0.0,) * 6, abs=1.0e-12)
    assert all(len(sample.positions) == len(sample.velocities) == len(sample.accelerations) == 6 for sample in trajectory.samples)


def test_prepare_solver_times_are_canonicalized_to_ros_nanoseconds(
    environment: EnvironmentBundle,
) -> None:
    class SubNanosecondAdapter(FakeCuroboAdapter):
        def plan_cspace(self, start_positions, goal_positions):
            path = super().plan_cspace(start_positions, goal_positions)
            return JointPath(
                times_s=tuple(
                    time_s if index == 0 else time_s + 4.0e-10
                    for index, time_s in enumerate(path.times_s)
                ),
                positions=path.positions,
            )

    adapter = SubNanosecondAdapter(environment)
    current = tuple(environment.robot_model["robot"]["home_joint_positions"])
    target = tuple(value + 0.02 for value in current)

    trajectory = PreparePlanner(environment, adapter).plan(
        environment.joint_names,
        current,
        target,
    )

    assert all(
        abs(sample.time_s * 1.0e9 - round(sample.time_s * 1.0e9)) < 1.0e-6
        for sample in trajectory.samples
    )


def test_prepare_float32_solver_grid_avoids_only_optional_near_knot_samples(
    environment: EnvironmentBundle,
) -> None:
    solver_times = tuple(float(value) for value in np.arange(201, dtype=np.float32) * np.float32(0.01))
    solver_ns = tuple(round(value * 1.0e9) for value in solver_times)
    current = tuple(environment.home_joint_positions)
    target = tuple(value + 0.02 for value in current)

    class Float32GridAdapter(FakeCuroboAdapter):
        def plan_cspace(self, start_positions, goal_positions):
            self.cspace_calls += 1
            duration = solver_times[-1]
            positions = []
            for time_s in solver_times:
                u = time_s / duration
                blend = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
                positions.append(tuple(start + (goal - start) * blend for start, goal in zip(start_positions, goal_positions, strict=True)))
            return JointPath(times_s=solver_times, positions=tuple(positions))

    adapter = Float32GridAdapter(environment)
    trajectory = PreparePlanner(environment, adapter).plan(environment.joint_names, current, target)
    sample_ns = tuple(round(sample.time_s * 1.0e9) for sample in trajectory.samples)

    assert set(solver_ns) <= set(sample_ns)
    expected_path = adapter.plan_cspace(current, target)
    by_ns = {round(sample.time_s * 1.0e9): sample for sample in trajectory.samples}
    for knot_ns, positions in zip(solver_ns, expected_path.positions, strict=True):
        assert by_ns[knot_ns].positions == pytest.approx(positions, abs=1.0e-14)
    assert min(np.diff(sample_ns)) >= 2_500_000
    assert trajectory.samples[-1].time_s == solver_ns[-1] / 1.0e9
    assert adapter.joint_validation_calls == 1
    assert trajectory.metadata["prepare"]["optional_sampling"] == {
        "scope": "generated_prepare_only",
        "minimum_optional_sample_spacing_s": 0.0025,
        "solver_knots_and_ros_nanoseconds_unchanged": True,
        "source_csv_modified": False,
    }


def test_prepare_rejects_unsafe_actual_sent_payload_before_gpu_validation(
    environment: EnvironmentBundle, monkeypatch,
) -> None:
    original_build = TrajectoryOptimizer.build

    def unsafe_build(self, *args, **kwargs):
        trajectory, metrics = original_build(self, *args, **kwargs)
        samples = list(trajectory.samples)
        sample = samples[len(samples) // 2]
        samples[len(samples) // 2] = JointSample(
            sample.time_s, sample.positions, (10.0,) * len(sample.positions),
            sample.accelerations, sample.phase,
        )
        return CompiledTrajectory(trajectory.joint_names, tuple(samples), trajectory.metadata), metrics

    monkeypatch.setattr(TrajectoryOptimizer, "build", unsafe_build)
    adapter = FakeCuroboAdapter(environment)
    current = tuple(environment.home_joint_positions)
    target = tuple(value + 0.02 for value in current)
    with pytest.raises(ValidationError, match="prepare sent payload quintic dynamics"):
        PreparePlanner(environment, adapter).plan(environment.joint_names, current, target)
    assert adapter.joint_validation_calls == 0


def test_prepare_does_not_remove_adversarial_original_tiny_intervals(
    environment: EnvironmentBundle,
) -> None:
    class TinyOriginalIntervalAdapter(FakeCuroboAdapter):
        def plan_cspace(self, start_positions, goal_positions):
            middle = tuple((start + goal) / 2.0 for start, goal in zip(start_positions, goal_positions, strict=True))
            moved = tuple(value + 0.0001 for value in middle)
            return JointPath(
                times_s=(0.0, 1.0, 1.000000001, 2.0),
                positions=(tuple(start_positions), middle, moved, tuple(goal_positions)),
            )

    adapter = TinyOriginalIntervalAdapter(environment)
    current = tuple(environment.home_joint_positions)
    target = tuple(value + 0.02 for value in current)
    with pytest.raises(ValidationError, match="authoritative input timing violates joint dynamics"):
        PreparePlanner(environment, adapter).plan(environment.joint_names, current, target)
    assert adapter.joint_validation_calls == 0


def test_prepare_planner_collision_checks_an_already_reached_target(
    environment: EnvironmentBundle,
) -> None:
    adapter = FakeCuroboAdapter(environment)
    current = tuple(environment.home_joint_positions)

    trajectory = PreparePlanner(environment, adapter).plan(
        environment.joint_names,
        current,
        current,
    )

    assert adapter.cspace_calls == 0
    assert adapter.joint_validation_calls == 1
    assert len(trajectory.samples) == 2
    assert trajectory.samples[0].positions == current
    assert trajectory.samples[1].positions == current
    assert trajectory.samples[0].velocities == (0.0,) * 6
    assert trajectory.samples[1].accelerations == (0.0,) * 6
    assert trajectory.metadata["prepare"]["stationary_collision_check"] is True


def test_prepare_planner_rejects_crossing_the_installed_joint_branch(
    repository_root,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root / "environments" / "new_board_4cm_rod33365"
    )

    class CrossingAdapter(FakeCuroboAdapter):
        def plan_cspace(self, start_positions, goal_positions):
            self.cspace_calls += 1
            middle = list(start_positions)
            middle[0] = 1.1960208416
            return JointPath(
                times_s=(0.0, 1.0, 2.0),
                positions=(
                    tuple(start_positions),
                    tuple(middle),
                    tuple(goal_positions),
                ),
            )

    adapter = CrossingAdapter(environment)
    current = environment.home_joint_positions
    target = tuple(value + 0.01 for value in current)

    with pytest.raises(PlanningError, match="leaves the configured canonical joint branch"):
        PreparePlanner(environment, adapter).plan(
            environment.joint_names,
            current,
            target,
        )

    assert adapter.cspace_calls == 1
    assert adapter.joint_validation_calls == 0


@pytest.mark.parametrize(
    ("joint_names", "current", "target", "message"),
    [
        (("wrong",) * 6, (0.0,) * 6, (0.1,) * 6, "joint_names"),
        (None, (0.0,) * 5, (0.1,) * 6, "current_positions"),
        (None, (0.0,) * 6, (0.1, 0.1, 0.1, 0.1, 0.1, nan), "target_positions"),
    ],
)
def test_prepare_planner_rejects_invalid_requests(
    environment: EnvironmentBundle,
    joint_names: tuple[str, ...] | None,
    current: tuple[float, ...],
    target: tuple[float, ...],
    message: str,
) -> None:
    adapter = FakeCuroboAdapter(environment)
    planner = PreparePlanner(environment, adapter)
    names = environment.joint_names if joint_names is None else joint_names

    with pytest.raises(PlanningError, match=message):
        planner.plan(names, current, target)

    assert adapter.cspace_calls == 0
    assert adapter.joint_validation_calls == 0
