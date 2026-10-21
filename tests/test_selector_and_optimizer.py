from __future__ import annotations

import numpy as np
import pytest

from hx_waypoint_curobo.continuous_ik_selector import ContinuousIKSelector, SelectionLimits
from hx_waypoint_curobo.errors import PlanningError, ValidationError
from hx_waypoint_curobo.models import IKCandidate
from hx_waypoint_curobo.trajectory_optimizer import DynamicLimits, ResamplePolicy, TrajectoryOptimizer


def _candidate(q, collision=0.02, singularity=0.0):
    return IKCandidate(
        positions=tuple(q),
        collision_distance_m=collision,
        joint_limit_margin_rad=1.0,
        singularity_score=singularity,
    )


def test_continuous_selector_avoids_branch_jump() -> None:
    selector = ContinuousIKSelector(limits=SelectionLimits(reject_branch_jump_rad=0.3))
    candidates = (
        (_candidate([0.0, 0.0]), _candidate([2.0, 2.0])),
        (_candidate([0.1, 0.1]), _candidate([2.1, 2.1])),
        (_candidate([0.2, 0.2]), _candidate([2.2, 2.2])),
    )
    selected = selector.select([0.0, 1.0, 2.0], candidates)
    assert np.allclose(selected[-1].positions, (0.2, 0.2))


def test_selector_prefers_first_pose_branch_near_reference_without_hard_jump() -> None:
    selector = ContinuousIKSelector(
        limits=SelectionLimits(reject_branch_jump_rad=0.1)
    )
    candidates = (
        (_candidate([3.0]), _candidate([1.0])),
        (_candidate([3.01]), _candidate([1.01])),
    )

    selected = selector.select(
        [0.0, 1.0],
        candidates,
        reference_positions=[0.0],
    )

    assert [candidate.positions for candidate in selected] == [(1.0,), (1.01,)]


def test_selector_rejects_missing_continuous_branch() -> None:
    selector = ContinuousIKSelector(limits=SelectionLimits(reject_branch_jump_rad=0.2))
    with pytest.raises(PlanningError, match="pose index 1"):
        selector.select(
            [0.0, 1.0],
            ((_candidate([0.0]),), (_candidate([1.0]),)),
        )


def test_selector_reports_first_pose_clearance_failure() -> None:
    selector = ContinuousIKSelector(
        limits=SelectionLimits(minimum_collision_distance_m=0.005)
    )

    with pytest.raises(PlanningError, match="minimum collision clearance") as error:
        selector.select(
            [0.0, 1.0],
            (
                (_candidate([0.0], collision=0.004),),
                (_candidate([0.1]),),
            ),
        )

    assert "maximum=0.004 m" in str(error.value)
    assert "required=0.005 m" in str(error.value)


def test_selector_does_not_hide_full_turn_joint_discontinuity() -> None:
    selector = ContinuousIKSelector(limits=SelectionLimits(reject_branch_jump_rad=0.3))
    with pytest.raises(PlanningError, match="pose index 1"):
        selector.select(
            [0.0, 1.0],
            ((_candidate([0.0]),), (_candidate([2.0 * np.pi - 0.01]),)),
        )


def test_selector_preserves_endpoint_diversity_when_pruning_beam() -> None:
    selector = ContinuousIKSelector(
        limits=SelectionLimits(reject_branch_jump_rad=0.3, beam_width=2)
    )
    candidates = (
        (
            _candidate([0.0]),
            _candidate([0.05]),
            _candidate([0.1]),
            _candidate([1.0]),
        ),
        (
            _candidate([0.15]),
            _candidate([1.1], singularity=10.0),
        ),
        (_candidate([1.2]),),
    )

    selected = selector.select([0.0, 1.0, 2.0], candidates)

    assert [candidate.positions for candidate in selected] == [(1.0,), (1.1,), (1.2,)]


def test_q_qd_qdd_are_same_curve_and_zero_at_boundaries() -> None:
    optimizer = TrajectoryOptimizer(
        DynamicLimits((10.0,), (100.0,), (1000.0,)),
        ResamplePolicy(nominal_dt_s=0.01),
    )
    trajectory, metrics = optimizer.build(
        ("joint",),
        (0.0, 1.0, 2.0),
        ((0.0,), (0.1,), (0.0,)),
        ("approach", "draw", "exit"),
    )
    assert trajectory.samples[0].velocities == (0.0,)
    assert trajectory.samples[-1].velocities == (0.0,)
    assert trajectory.samples[-1].accelerations == (0.0,)
    assert metrics.uniform_time_scale == 1.0
    assert metrics.original_duration_s == metrics.final_duration_s == 2.0
    time = np.asarray([sample.time_s for sample in trajectory.samples])
    assert all(knot in time for knot in (0.0, 1.0, 2.0))
    position = np.asarray([sample.positions[0] for sample in trajectory.samples])
    velocity = np.asarray([sample.velocities[0] for sample in trajectory.samples])
    numeric = np.gradient(position, time)
    assert np.max(np.abs(numeric[2:-2] - velocity[2:-2])) < 1.0e-3


def test_ordinary_internal_knot_is_not_forced_to_rest() -> None:
    optimizer = TrajectoryOptimizer(
        DynamicLimits((10.0,), (100.0,), (1000.0,)),
        ResamplePolicy(nominal_dt_s=0.01),
    )
    trajectory, _ = optimizer.build(
        ("joint",),
        (0.0, 1.0, 2.0),
        ((0.0,), (0.1,), (0.2,)),
        ("approach", "draw", "exit"),
    )

    internal = next(sample for sample in trajectory.samples if sample.time_s == 1.0)
    assert internal.positions == pytest.approx((0.1,))
    assert abs(internal.velocities[0]) > 1.0e-6


def test_dynamics_violation_rejects_by_default() -> None:
    optimizer = TrajectoryOptimizer(
        DynamicLimits((0.01,), (0.01,), (0.01,)),
        ResamplePolicy(nominal_dt_s=0.01, allow_uniform_time_scale=False),
    )
    with pytest.raises(ValidationError, match="violates joint dynamics") as error:
        optimizer.build(
            ("joint",),
            (0.0, 0.1, 0.2),
            ((0.0,), (1.0,), (0.0,)),
            ("approach", "draw", "exit"),
        )
    message = str(error.value)
    assert "velocity_ratio=" in message
    assert "acceleration_ratio=" in message
    assert "jerk_ratio=" in message
    assert "joint=joint" in message
    assert "segment=" in message
    assert "time_s=" in message


def test_dynamics_validation_checks_between_output_samples() -> None:
    optimizer = TrajectoryOptimizer(
        DynamicLimits((1.0,), (1.0e6,), (1.0e6,)),
        ResamplePolicy(nominal_dt_s=10.0),
    )

    with pytest.raises(ValidationError, match="velocity="):
        optimizer.build(
            ("joint",),
            (0.0, 1.0, 2.0),
            ((0.0,), (1.0,), (0.0,)),
            ("draw", "draw", "draw"),
        )


@pytest.mark.parametrize(
    ("policy", "message"),
    [
        (ResamplePolicy(on_violation="uniform_scale"), "on_violation must be reject"),
        (
            ResamplePolicy(allow_uniform_time_scale=True),
            "allow_uniform_time_scale must be false",
        ),
    ],
)
def test_any_uniform_time_scaling_policy_is_rejected(
    policy: ResamplePolicy,
    message: str,
) -> None:
    with pytest.raises(PlanningError, match=message):
        TrajectoryOptimizer(
            DynamicLimits((1.0,), (1.0,), (1.0,)),
            policy,
        )


@pytest.mark.parametrize(
    "maximum",
    [0.99, 1.01, 3.0, float("inf"), float("nan"), True, None],
)
def test_invalid_uniform_time_scale_maximum_is_rejected(maximum) -> None:
    with pytest.raises(PlanningError, match="max_uniform_time_scale"):
        TrajectoryOptimizer(
            DynamicLimits((1.0,), (1.0,), (1.0,)),
            ResamplePolicy(max_uniform_time_scale=maximum),
        )
def test_hold_interval_is_exactly_stationary() -> None:
    optimizer = TrajectoryOptimizer(
        DynamicLimits((100.0,), (1000.0,), (10000.0,)),
        ResamplePolicy(nominal_dt_s=0.01),
    )
    trajectory, _ = optimizer.build(
        ("joint",),
        (0.0, 1.0, 2.0, 3.0),
        ((0.0,), (1.0,), (1.0,), (0.0,)),
        ("approach", "draw", "hold", "exit"),
    )
    hold_interval = [
        sample for sample in trajectory.samples if 1.0 <= sample.time_s <= 2.0
    ]
    assert hold_interval
    assert all(sample.positions == (1.0,) for sample in hold_interval)
    assert all(sample.velocities == (0.0,) for sample in hold_interval)
    assert all(sample.accelerations == (0.0,) for sample in hold_interval)


def test_hold_knot_with_joint_motion_is_rejected() -> None:
    optimizer = TrajectoryOptimizer(
        DynamicLimits((100.0,), (1000.0,), (10000.0,)),
        ResamplePolicy(nominal_dt_s=0.01),
    )
    with pytest.raises(PlanningError, match="hold knot 1 must repeat"):
        optimizer.build(
            ("joint",),
            (0.0, 1.0),
            ((0.0,), (0.1,)),
            ("draw", "hold"),
        )


def test_optional_spacing_preserves_every_original_knot_and_default_grid() -> None:
    limits = DynamicLimits((100.0,), (1000.0,), (10000.0,))
    times = (0.0, 0.100000001, 0.100000002, 1.0)
    positions = ((0.0,),) * len(times)
    phases = ("draw",) * len(times)
    default, _ = TrajectoryOptimizer(limits, ResamplePolicy()).build(
        ("joint",), times, positions, phases
    )
    spaced, _ = TrajectoryOptimizer(
        limits,
        ResamplePolicy(minimum_optional_sample_spacing_s=0.0025),
    ).build(("joint",), times, positions, phases)

    default_ns = {round(sample.time_s * 1.0e9) for sample in default.samples}
    spaced_ns = {round(sample.time_s * 1.0e9) for sample in spaced.samples}
    authority_ns = {round(time_s * 1.0e9) for time_s in times}
    assert authority_ns <= default_ns
    assert authority_ns <= spaced_ns
    assert 100_000_000 in default_ns  # Legacy default keeps a 1 ns synthetic split.
    assert 100_000_000 not in spaced_ns
    assert 100_000_001 in spaced_ns and 100_000_002 in spaced_ns
    assert default_ns - spaced_ns == {100_000_000}


@pytest.mark.parametrize("spacing", [-0.01, float("inf"), float("nan"), 1.0e308, True, None])
def test_invalid_optional_sample_spacing_is_rejected(spacing) -> None:
    with pytest.raises(PlanningError, match="minimum_optional_sample_spacing_s"):
        TrajectoryOptimizer(
            DynamicLimits((1.0,), (1.0,), (1.0,)),
            ResamplePolicy(minimum_optional_sample_spacing_s=spacing),
        )


@pytest.mark.parametrize(("knot_time", "kept"), [(0.1025, True), (0.102499999, False)])
def test_optional_spacing_threshold_inclusive(knot_time: float, kept: bool) -> None:
    trajectory, _ = TrajectoryOptimizer(
        DynamicLimits((100.0,), (1000.0,), (10000.0,)),
        ResamplePolicy(minimum_optional_sample_spacing_s=0.0025),
    ).build(
        ("joint",), (0.0, knot_time, 1.0), ((0.0,),) * 3, ("draw",) * 3
    )
    output_ns = {round(sample.time_s * 1.0e9) for sample in trajectory.samples}
    assert (100_000_000 in output_ns) == kept
    assert round(knot_time * 1.0e9) in output_ns
