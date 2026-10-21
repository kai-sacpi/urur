from __future__ import annotations

from math import tau

import numpy as np
import pytest

from conftest import FakeCuroboAdapter

from hx_waypoint_curobo.curobo_adapter_v2 import JointPath
from hx_waypoint_curobo.errors import PlanningError, ValidationError
from hx_waypoint_curobo.models import TimedPose
from hx_waypoint_curobo.timed_pose_compiler import CuroboTimedPathCompiler
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv


def _scale_candidate_sets(candidate_sets, home, scale):
    output = []
    for candidates in candidate_sets:
        scaled = []
        for candidate in candidates:
            positions = tuple(
                float(home_value + scale * (value - home_value))
                for value, home_value in zip(
                    candidate.positions,
                    home,
                    strict=True,
                )
            )
            scaled.append(
                type(candidate)(
                    positions=positions,
                    collision_distance_m=candidate.collision_distance_m,
                    joint_limit_margin_rad=candidate.joint_limit_margin_rad,
                    singularity_score=candidate.singularity_score,
                )
            )
        output.append(tuple(scaled))
    return tuple(output)


def test_full_compiler_preserves_source_constraints(repository_root, environment) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )
    adapter = FakeCuroboAdapter(environment)
    result = CuroboTimedPathCompiler(environment, adapter).compile(source)
    transformed = tuple(environment.transform_pose_to_base(pose) for pose in source)
    source_index = 0
    for pose in result.resampled_source_poses:
        if source_index < len(transformed) and pose == transformed[source_index]:
            source_index += 1
    assert source_index == len(transformed)
    assert adapter.warmup_calls == 1
    assert adapter.ik_calls == 1
    assert adapter.optimize_calls == 0
    assert adapter.compiled_validation_constraint_count > len(result.resampled_source_poses)
    assert adapter.cspace_calls == 0
    assert {sample.phase for sample in result.trajectory.samples} == {"draw", "hold"}
    assert result.trajectory.samples[0].time_s == 0.0
    assert result.trajectory.duration_s == source[-1].time_s
    assert result.metrics.original_duration_s == source[-1].time_s
    assert result.metrics.final_duration_s == source[-1].time_s
    assert result.metrics.uniform_time_scale == 1.0
    compiled_times = {sample.time_s for sample in result.trajectory.samples}
    assert all(pose.time_s in compiled_times for pose in source)
    assert result.trajectory.samples[0].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].velocities == (0.0,) * 6
    assert result.trajectory.samples[-1].accelerations == (0.0,) * 6
    hold_samples = [
        sample for sample in result.trajectory.samples if sample.phase == "hold"
    ]
    assert hold_samples
    assert len({sample.positions for sample in hold_samples}) == 1
    assert all(sample.velocities == (0.0,) * 6 for sample in hold_samples)
    assert all(sample.accelerations == (0.0,) * 6 for sample in hold_samples)


def test_compiler_rebases_only_the_complete_revolute_path(environment) -> None:
    path = JointPath(
        times_s=(0.0, 1.0, 2.0),
        positions=(
            (-2.6159053, 4.9225936, -2.0656557, 3.4262466, 5.2380781, 4.7123876),
            (-2.8000000, 4.9000000, -2.1000000, 3.5000000, 5.0000000, 4.7123000),
            (-2.9419010, 4.9614964, -1.9642697, 3.2861123, 4.9125109, 4.7124958),
        ),
    )
    reference = tuple(environment.robot_model["robot"]["home_joint_positions"])

    canonical, offsets = CuroboTimedPathCompiler._canonicalize_revolute_winding(
        path,
        reference,
        environment.joint_position_domains,
    )

    assert canonical.times_s == path.times_s
    assert offsets == pytest.approx((0.0, -tau, 0.0, -tau, -tau, -tau))
    assert np.diff(canonical.positions, axis=0) == pytest.approx(
        np.diff(path.positions, axis=0),
        abs=1.0e-12,
    )
    assert canonical.positions[0] == pytest.approx(
        (
            -2.6159053,
            4.9225936 - tau,
            -2.0656557,
            3.4262466 - tau,
            5.2380781 - tau,
            4.7123876 - tau,
        )
    )
    for index, domain in enumerate(environment.joint_position_domains):
        original = np.asarray(path.positions)[:, index]
        shifted = np.asarray(canonical.positions)[:, index]
        original_margin = np.min(
            np.minimum(original - domain.lower, domain.upper - original)
        )
        shifted_margin = np.min(
            np.minimum(shifted - domain.lower, domain.upper - shifted)
        )
        assert shifted_margin >= original_margin - 1.0e-12


def test_joint_winding_canonicalization_cannot_hide_an_internal_turn(environment) -> None:
    first = list(environment.robot_model["robot"]["home_joint_positions"])
    second = list(first)
    first[0] = tau - 0.05
    second[0] = -0.04
    path = JointPath(
        times_s=(0.0, 1.0),
        positions=(tuple(first), tuple(second)),
    )

    canonical, offsets = CuroboTimedPathCompiler._canonicalize_revolute_winding(
        path,
        tuple(environment.robot_model["robot"]["home_joint_positions"]),
        environment.joint_position_domains,
    )

    assert offsets[0] == 0.0
    assert canonical.positions[1][0] - canonical.positions[0][0] == pytest.approx(
        second[0] - first[0]
    )


def test_joint_winding_canonicalization_never_reduces_limit_margin(environment) -> None:
    first = list(environment.robot_model["robot"]["home_joint_positions"])
    second = list(first)
    first[0] = 4.0
    second[0] = 0.5
    path = JointPath(
        times_s=(0.0, 1.0),
        positions=(tuple(first), tuple(second)),
    )

    canonical, offsets = CuroboTimedPathCompiler._canonicalize_revolute_winding(
        path,
        tuple(environment.robot_model["robot"]["home_joint_positions"]),
        environment.joint_position_domains,
    )

    # Subtracting one turn would put the start closer to home, but it would
    # reduce this joint's complete-path bound margin from 2.283 rad to 0.5 rad.
    assert offsets[0] == 0.0
    assert canonical.positions == path.positions


def test_full_compiler_canonicalizes_equivalent_ik_winding(
    repository_root,
    environment,
) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )

    class WoundFakeAdapter(FakeCuroboAdapter):
        def solve_ik_candidates(self, tool_poses, candidates_per_pose):
            candidate_sets = super().solve_ik_candidates(tool_poses, candidates_per_pose)
            output = []
            for candidates in candidate_sets:
                wound = []
                for candidate in candidates:
                    positions = list(candidate.positions)
                    for index in (1, 3, 4):
                        positions[index] += tau
                    wound.append(
                        type(candidate)(
                            positions=tuple(positions),
                            collision_distance_m=candidate.collision_distance_m,
                            joint_limit_margin_rad=candidate.joint_limit_margin_rad,
                            singularity_score=candidate.singularity_score,
                        )
                    )
                output.append(tuple(wound))
            return tuple(output)

    baseline = CuroboTimedPathCompiler(environment, FakeCuroboAdapter(environment)).compile(source)
    wound = CuroboTimedPathCompiler(environment, WoundFakeAdapter(environment)).compile(source)

    assert [sample.time_s for sample in wound.trajectory.samples] == [
        sample.time_s for sample in baseline.trajectory.samples
    ]
    assert [sample.phase for sample in wound.trajectory.samples] == [
        sample.phase for sample in baseline.trajectory.samples
    ]
    assert np.asarray([sample.positions for sample in wound.trajectory.samples]) == pytest.approx(
        np.asarray([sample.positions for sample in baseline.trajectory.samples]),
        abs=1.0e-12,
    )
    assert np.asarray([sample.velocities for sample in wound.trajectory.samples]) == pytest.approx(
        np.asarray([sample.velocities for sample in baseline.trajectory.samples]),
        abs=1.0e-12,
    )
    assert np.asarray(
        [sample.accelerations for sample in wound.trajectory.samples]
    ) == pytest.approx(
        np.asarray([sample.accelerations for sample in baseline.trajectory.samples]),
        abs=1.0e-10,
    )
    winding = wound.trajectory.metadata["joint_winding"]
    assert winding["selected_seed_offsets_rad"] == pytest.approx(
        (0.0, -tau, 0.0, -tau, -tau, 0.0)
    )
    assert winding["post_optimization_offsets_rad"] == pytest.approx((0.0,) * 6)


def test_trajopt_fallback_output_is_canonicalized_again(
    repository_root,
    environment,
) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )

    class WoundFallbackAdapter(FakeCuroboAdapter):
        def solve_ik_candidates(self, tool_poses, candidates_per_pose):
            candidate_sets = super().solve_ik_candidates(tool_poses, candidates_per_pose)
            home = self.environment.robot_model["robot"]["home_joint_positions"]
            return _scale_candidate_sets(candidate_sets, home, 1000.0)

        def optimize_pose_path(self, tool_poses, authoritative_times_s, seed_positions):
            del seed_positions
            self.optimize_calls += 1
            home = np.asarray(
                self.environment.robot_model["robot"]["home_joint_positions"],
                dtype=float,
            )
            positions = []
            for pose in tool_poses:
                point = np.asarray(pose.position_m)
                row = home + np.array(
                    [
                        point[0],
                        point[1],
                        point[2],
                        point[0] - point[1],
                        point[1] - point[2],
                        point[2] - point[0],
                    ]
                ) * 0.005
                row[[1, 3, 4]] += tau
                positions.append(tuple(float(value) for value in row))
            return JointPath(
                times_s=tuple(float(value) for value in authoritative_times_s),
                positions=tuple(positions),
            )

    adapter = WoundFallbackAdapter(environment)
    result = CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert adapter.optimize_calls == 1
    winding = result.trajectory.metadata["joint_winding"]
    assert winding["post_optimization_offsets_rad"] == pytest.approx(
        (0.0, -tau, 0.0, -tau, -tau, 0.0)
    )
    assert result.trajectory.duration_s == source[-1].time_s
    assert result.metrics.uniform_time_scale == 1.0


def test_solver_delay_does_not_enter_trajectory_time(repository_root, environment) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )
    fast = CuroboTimedPathCompiler(environment, FakeCuroboAdapter(environment)).compile(source)
    slow = CuroboTimedPathCompiler(
        environment,
        FakeCuroboAdapter(environment, delay_s=0.01),
    ).compile(source)
    assert [sample.time_s for sample in fast.trajectory.samples] == [
        sample.time_s for sample in slow.trajectory.samples
    ]
    assert fast.trajectory.duration_s == slow.trajectory.duration_s


def test_compiler_rejects_infeasible_authoritative_timing(repository_root, environment) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )

    class AmplifiedFakeAdapter(FakeCuroboAdapter):
        def solve_ik_candidates(self, tool_poses, candidates_per_pose):
            candidate_sets = super().solve_ik_candidates(tool_poses, candidates_per_pose)
            home = self.environment.robot_model["robot"]["home_joint_positions"]
            return _scale_candidate_sets(candidate_sets, home, 1000.0)

        def optimize_pose_path(self, tool_poses, authoritative_times_s, seed_positions):
            path = super().optimize_pose_path(
                tool_poses,
                authoritative_times_s,
                seed_positions,
            )
            home = self.environment.robot_model["robot"]["home_joint_positions"]
            return JointPath(
                    times_s=path.times_s,
                    positions=tuple(
                        tuple(
                            float(home_value + 1000.0 * (value - home_value))
                            for value, home_value in zip(row, home, strict=True)
                        )
                    for row in path.positions
                ),
            )

    with pytest.raises(ValidationError, match="retiming is forbidden"):
        CuroboTimedPathCompiler(environment, AmplifiedFakeAdapter(environment)).compile(source)


def test_compiler_preserves_local_dynamics_rejection_when_fallback_fails(
    repository_root,
    environment,
) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )

    class FailingFallbackAdapter(FakeCuroboAdapter):
        def solve_ik_candidates(self, tool_poses, candidates_per_pose):
            candidate_sets = super().solve_ik_candidates(tool_poses, candidates_per_pose)
            home = self.environment.robot_model["robot"]["home_joint_positions"]
            return _scale_candidate_sets(candidate_sets, home, 1000.0)

        def optimize_pose_path(self, tool_poses, authoritative_times_s, seed_positions):
            del tool_poses, authoritative_times_s, seed_positions
            self.optimize_calls += 1
            raise PlanningError("synthetic cuRobo fixed-time failure")

    adapter = FailingFallbackAdapter(environment)
    with pytest.raises(ValidationError) as caught:
        CuroboTimedPathCompiler(environment, adapter).compile(source)

    message = str(caught.value)
    assert "authoritative input timing violates joint dynamics" in message
    assert "velocity_ratio=" in message
    assert "cuRobo fixed-time fallback also failed" in message
    assert "PlanningError: synthetic cuRobo fixed-time failure" in message
    assert "not retimed, scaled, or dropped" in message
    assert isinstance(caught.value.__cause__, PlanningError)
    assert adapter.optimize_calls == 1


def test_compiler_rejects_endpoint_motion_that_cannot_stop_without_retiming(
    repository_root,
    environment,
) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    with pytest.raises(
        ValidationError,
        match="endpoint-rest dynamics are infeasible; trajectory retiming is forbidden",
    ):
        CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert adapter.warmup_calls == 0
    assert adapter.ik_calls == 0
    assert adapter.optimize_calls == 0


def test_compiler_rejects_endpoint_translation_before_gpu(environment) -> None:
    source = (
        TimedPose(
            time_s=0.0,
            frame_id=environment.base_frame,
            position_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            phase="draw",
        ),
        TimedPose(
            time_s=0.001,
            frame_id=environment.base_frame,
            position_m=(0.02, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            phase="draw",
        ),
    )
    adapter = FakeCuroboAdapter(environment)

    with pytest.raises(ValidationError) as caught:
        CuroboTimedPathCompiler(environment, adapter).compile(source)

    message = str(caught.value)
    assert "violations=translation" in message
    assert "required_tool_translation_change_m>=" in message
    assert "maximum_tool_translation_bound_m=" in message
    assert "waypoint Pose and time_s are hard constraints" in message
    assert "cannot be retimed, scaled, or dropped" in message
    assert adapter.warmup_calls == 0
    assert adapter.ik_calls == 0
    assert adapter.optimize_calls == 0


def test_compiler_rejects_terminal_endpoint_motion_before_gpu(environment) -> None:
    stationary_pose = {
        "frame_id": "wall_draw_frame",
        "position_m": (0.0, 0.0, 0.0),
        "quaternion_xyzw": (0.0, 1.0, 0.0, 0.0),
        "phase": "draw",
    }
    source = (
        TimedPose(time_s=0.0, **stationary_pose),
        TimedPose(time_s=1.0, **stationary_pose),
        TimedPose(time_s=1.999, **stationary_pose),
        TimedPose(
            time_s=2.0,
            frame_id="wall_draw_frame",
            position_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            phase="draw",
        ),
    )
    adapter = FakeCuroboAdapter(environment)

    with pytest.raises(ValidationError) as caught:
        CuroboTimedPathCompiler(environment, adapter).compile(source)

    message = str(caught.value)
    assert "terminal_time_s=2.000000000" in message
    assert "comparison_time_s=" in message
    assert "waypoint Pose and time_s are hard constraints" in message
    assert adapter.warmup_calls == 0
    assert adapter.ik_calls == 0
    assert adapter.optimize_calls == 0


@pytest.mark.parametrize(
    "trajectory_name",
    [
        "circle",
        "figure8",
        "lissajous_xy_z_sweep",
        "rounded_square",
        "spiral_in_out",
        "straight_line",
        "vertical_bob",
        "zigzag_s_curve",
    ],
)
def test_dense_test_assets_are_strict_endpoint_rest_negative_cases(
    repository_root,
    environment,
    trajectory_name,
) -> None:
    source = read_timed_pose_csv(
        repository_root
        / "environments"
        / "lab_wall_a"
        / "trajectories"
        / f"{trajectory_name}.csv"
    )
    adapter = FakeCuroboAdapter(environment)

    with pytest.raises(ValidationError, match="endpoint-rest dynamics are infeasible"):
        CuroboTimedPathCompiler(environment, adapter).compile(source)

    assert adapter.warmup_calls == 0
    assert adapter.ik_calls == 0
    assert adapter.optimize_calls == 0


def test_densification_inserts_constraints_without_replacing_source(
    repository_root,
    environment,
) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )
    compiler = CuroboTimedPathCompiler(environment, FakeCuroboAdapter(environment))
    transformed = tuple(environment.transform_pose_to_base(pose) for pose in source)
    resampled = compiler._resample_authoritative_path(transformed)

    maximum_step = float(environment.planner["timed_path"]["max_translation_step_m"])
    distances = [
        np.linalg.norm(
            np.asarray(current.position_m) - np.asarray(previous.position_m)
        )
        for previous, current in zip(resampled[:-1], resampled[1:], strict=True)
    ]
    assert max(distances) <= maximum_step + 1.0e-12
    assert len(resampled) > len(transformed)
    source_index = 0
    for pose in resampled:
        if source_index < len(transformed) and pose == transformed[source_index]:
            source_index += 1
    assert source_index == len(transformed)
    assert all(
        current.time_s > previous.time_s
        for previous, current in zip(resampled[:-1], resampled[1:], strict=True)
    )


def test_optimizer_may_not_change_authoritative_times(repository_root, environment) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )

    class RetimingAdapter(FakeCuroboAdapter):
        def solve_ik_candidates(self, tool_poses, candidates_per_pose):
            candidate_sets = super().solve_ik_candidates(tool_poses, candidates_per_pose)
            home = self.environment.robot_model["robot"]["home_joint_positions"]
            return _scale_candidate_sets(candidate_sets, home, 1000.0)

        def optimize_pose_path(self, tool_poses, authoritative_times_s, seed_positions):
            path = super().optimize_pose_path(
                tool_poses,
                authoritative_times_s,
                seed_positions,
            )
            return JointPath(
                times_s=tuple(time_s * 1.01 for time_s in path.times_s),
                positions=path.positions,
            )

    with pytest.raises(ValidationError) as caught:
        CuroboTimedPathCompiler(environment, RetimingAdapter(environment)).compile(source)

    message = str(caught.value)
    assert "authoritative input timing violates joint dynamics" in message
    assert "changed authoritative constraint times" in message
    assert "not retimed, scaled, or dropped" in message


def test_no_source_waypoint_may_be_skipped(repository_root, environment) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )

    class MissingCandidateAdapter(FakeCuroboAdapter):
        def solve_ik_candidates(self, tool_poses, candidates_per_pose):
            result = list(super().solve_ik_candidates(tool_poses, candidates_per_pose))
            result[len(result) // 2] = ()
            return tuple(result)

    with pytest.raises(PlanningError, match="pose index"):
        CuroboTimedPathCompiler(environment, MissingCandidateAdapter(environment)).compile(source)


def test_compiler_freezes_independent_hold_ik_drift(repository_root, environment) -> None:
    source = read_timed_pose_csv(
        repository_root / "environments" / "lab_wall_a" / "trajectories" / "circle_01.csv"
    )

    class DriftingHoldAdapter(FakeCuroboAdapter):
        def optimize_pose_path(self, tool_poses, authoritative_times_s, seed_positions):
            path = super().optimize_pose_path(
                tool_poses,
                authoritative_times_s,
                seed_positions,
            )
            positions = list(path.positions)
            for index in range(1, len(tool_poses)):
                if (
                    tool_poses[index].position_m == tool_poses[index - 1].position_m
                    and tool_poses[index].quaternion_xyzw
                    == tool_poses[index - 1].quaternion_xyzw
                ):
                    drifted = list(positions[index])
                    drifted[0] += 0.1
                    positions[index] = tuple(drifted)
            return JointPath(times_s=path.times_s, positions=tuple(positions))

    result = CuroboTimedPathCompiler(environment, DriftingHoldAdapter(environment)).compile(source)
    hold_samples = [
        sample for sample in result.trajectory.samples if sample.phase == "hold"
    ]
    assert hold_samples
    assert len({sample.positions for sample in hold_samples}) == 1
    assert all(sample.velocities == (0.0,) * 6 for sample in hold_samples)
    assert all(sample.accelerations == (0.0,) * 6 for sample in hold_samples)
