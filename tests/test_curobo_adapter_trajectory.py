from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from hx_waypoint_curobo.curobo_adapter_v2 import (
    CuroboAdapterV2,
    _TrajectoryTrackingCSpaceCost,
    _TrajectoryTrackingToolPoseCost,
)
from hx_waypoint_curobo.errors import PlanningError
from hx_waypoint_curobo.models import CompiledTrajectory, JointSample, TimedPose


class _CpuTorch:
    float32 = torch.float32
    int32 = torch.int32

    def __getattr__(self, name: str):
        return getattr(torch, name)

    @staticmethod
    def _cpu_kwargs(kwargs: dict) -> dict:
        result = dict(kwargs)
        if result.get("device") == "cuda":
            result["device"] = "cpu"
        return result

    def as_tensor(self, data, *args, **kwargs):
        return torch.as_tensor(data, *args, **self._cpu_kwargs(kwargs))

    def tensor(self, data, *args, **kwargs):
        return torch.tensor(data, *args, **self._cpu_kwargs(kwargs))

    def full(self, size, fill_value, *args, **kwargs):
        return torch.full(size, fill_value, *args, **self._cpu_kwargs(kwargs))


@dataclass
class _GoalToolPose:
    tool_frames: list[str]
    position: torch.Tensor
    quaternion: torch.Tensor


class _JointState:
    def __init__(self, position: torch.Tensor, joint_names=None) -> None:
        self.position = position
        self.joint_names = joint_names
        self.dt = None

    @classmethod
    def from_position(cls, position: torch.Tensor, joint_names=None):
        return cls(position, joint_names)


class _FakeCost:
    def __init__(self, weight=(1.0, 1.0)) -> None:
        self._weight = torch.tensor(weight, dtype=torch.float32)
        self.enabled = True

    def setup_batch_tensors(self, *args, **kwargs) -> None:
        del args, kwargs

    def forward(self, state_batch, *args, **kwargs):
        del args, kwargs
        return torch.zeros_like(state_batch.position)


class _FakeManager:
    def __init__(self, *, include_cspace: bool) -> None:
        self.costs = {"tool_pose": _FakeCost()}
        if include_cspace:
            self.costs["cspace"] = _FakeCost((1.0,))

    def has_cost(self, name: str) -> bool:
        return name in self.costs

    def get_cost(self, name: str):
        return self.costs.get(name)


class _FakeRollout:
    def __init__(self, *, include_cspace: bool) -> None:
        self.cost_manager = _FakeManager(include_cspace=include_cspace)
        self.constraint_manager = None
        self.hybrid_cost_constraint_manager = None
        self.metrics_cost_manager = None
        self.metrics_constraint_manager = None
        self.metrics_hybrid_cost_constraint_manager = None
        self.metrics_convergence_manager = None
        self.transition_model = _FakeTeleportTransition()
        self.metrics_transition_model = _FakeTeleportTransition()


class _FakeTeleportStep:
    def forward(self, start_state, actions, out_state, *args, **kwargs):
        del start_state, actions, args, kwargs
        return out_state


class _FakeTeleportTransition:
    teleport_mode = True

    def __init__(self) -> None:
        self._rollout_step_fn = _FakeTeleportStep()
        self._cmd_step_fn = _FakeTeleportStep()


class _NeverIK:
    def __init__(self, *args, **kwargs) -> None:
        del args, kwargs
        raise AssertionError("trajectory optimization must not instantiate per-frame IK")


def _locked_curobo_interpolated_step_count(
    raw_dt: float,
    interpolation_dt: float,
    horizon: int,
) -> int:
    raw = np.float32(raw_dt)
    interpolation = np.float32(interpolation_dt)
    steps_per_interval = int((raw + raw % interpolation) / interpolation)
    return (horizon - 1) * steps_per_interval + 1


def _tool_poses(count: int, *, dt_s: float = 0.1) -> tuple[TimedPose, ...]:
    return tuple(
        TimedPose(
            time_s=dt_s * index,
            frame_id="base_link",
            position_m=(0.4 + 0.001 * index, 0.1, 0.3),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            phase="draw",
        )
        for index in range(count)
    )


def _adapter_runtime(
    adapter: CuroboAdapterV2,
    *,
    solver_success: bool = True,
    result_count_delta: int = 0,
    fk_position_error_index: int | None = None,
    midpoint_collision: bool = False,
):
    cpu_torch = _CpuTorch()

    class FakeTrajectoryOptimizerCfg:
        last_kwargs = None

        @classmethod
        def create(cls, **kwargs):
            cls.last_kwargs = kwargs
            return SimpleNamespace(
                interpolation_buffer_size=5000,
                interpolation_dt=0.025,
                kwargs=kwargs,
            )

    class FakeTrajectoryOptimizer:
        instance = None

        def __init__(self, config) -> None:
            self.config = config
            self.optimizer_rollouts = [_FakeRollout(include_cspace=True)]
            self.metrics_rollout = _FakeRollout(include_cspace=False)
            self.auxiliary_rollout = _FakeRollout(include_cspace=False)
            self.solve_calls = 0
            self.solve_kwargs = None
            self.interpolated_step_count = None
            self.last_result = None
            self.last_active_js = None
            self.dt_at_seed_rollout = None
            self.dt_at_interpolation = None
            self.interpolation_steps_per_interval = None
            self.midpoint_collision_detected = False
            self.collision_sample_fractions = ()
            self.collision_samples = ()
            self.additional_metrics_rollouts = {}
            FakeTrajectoryOptimizer.instance = self

        def solve_pose(self, **kwargs):
            self.solve_calls += 1
            self.solve_kwargs = kwargs
            seed = kwargs["seed_traj"]
            tracking_cost = self.optimizer_rollouts[0].cost_manager.get_cost("tool_pose")
            tracking_goal = tracking_cost._tracking_goal
            current_tool_poses = SimpleNamespace(
                tool_frames=tracking_goal.tool_frames,
                position=tracking_goal.position[:, :, :, 0, :].clone(),
                quaternion=tracking_goal.quaternion[:, :, :, 0, :].clone(),
            )
            cost, position_error, orientation_error, _ = tracking_cost.forward(
                current_tool_poses,
                kwargs["goal_tool_poses"],
            )
            assert cost.shape == (1, seed.shape[2], 2)
            assert position_error.shape == (1, seed.shape[2], 1)
            assert orientation_error.shape == (1, seed.shape[2], 1)
            cspace_cost = self.optimizer_rollouts[0].cost_manager.get_cost("cspace")
            cspace_cost.forward(SimpleNamespace(position=seed[:, 0]))

            count = seed.shape[2] + result_count_delta
            positions = seed[:, :, :count, :].clone()
            if count > seed.shape[2]:
                positions = torch.cat((positions, positions[:, :, -1:, :]), dim=2)
            seed_rollout_state = _JointState(positions)
            # Locked cuRobo's StateFromPositionTeleport leaves this unit dt in
            # place unless the adapter fixes the transition before metrics.
            seed_rollout_state.dt = torch.ones(
                positions.shape[:2],
                dtype=torch.float32,
            )
            seed_rollout_state = (
                self.metrics_rollout.transition_model._rollout_step_fn.forward(
                    None,
                    positions,
                    seed_rollout_state,
                )
            )
            self.dt_at_seed_rollout = seed_rollout_state.dt.clone()
            solution = _JointState(positions)
            solution.dt = torch.ones(positions.shape[:2], dtype=torch.float32)
            solution = (
                self.metrics_rollout.metrics_transition_model._rollout_step_fn.forward(
                    None,
                    positions,
                    solution,
                )
            )
            self.dt_at_interpolation = solution.dt.clone()
            raw_dt = float(solution.dt.reshape(-1)[0])
            interpolation_dt = float(self.config.interpolation_dt)
            self.interpolated_step_count = _locked_curobo_interpolated_step_count(
                raw_dt,
                interpolation_dt,
                positions.shape[2],
            )
            self.interpolation_steps_per_interval = int(
                (self.interpolated_step_count - 1) / max(positions.shape[2] - 1, 1)
            )
            if self.interpolated_step_count > self.config.interpolation_buffer_size:
                raise RuntimeError("fake cuRobo interpolation buffer is too small")
            if midpoint_collision:
                self.collision_sample_fractions = tuple(
                    index / self.interpolation_steps_per_interval
                    for index in range(self.interpolation_steps_per_interval + 1)
                )
                self.collision_samples = tuple(
                    bool(np.isclose(fraction, 0.5, rtol=0.0, atol=1.0e-12))
                    for fraction in self.collision_sample_fractions
                )
                self.midpoint_collision_detected = any(self.collision_samples)
            self.last_result = SimpleNamespace(
                success=torch.tensor(
                    [solver_success and not self.midpoint_collision_detected]
                ),
                js_solution=solution,
            )
            return self.last_result

        def get_active_js(self, joint_state):
            self.last_active_js = joint_state
            return joint_state

        def compute_kinematics(self, state):
            del state
            tracking_cost = self.optimizer_rollouts[0].cost_manager.get_cost("tool_pose")
            tracking_goal = tracking_cost._tracking_goal
            positions = tracking_goal.position[:, :, 0, 0, :].reshape(-1, 3).clone()
            if fk_position_error_index is not None:
                positions[fk_position_error_index, 0] += 0.01
            quaternions = tracking_goal.quaternion[:, :, 0, 0, :].reshape(-1, 4).clone()
            pose = SimpleNamespace(position=positions, quaternion=quaternions)
            tool_poses = SimpleNamespace(get_link_pose=lambda _: pose)
            return SimpleNamespace(tool_poses=tool_poses)

    adapter._runtime = {
        "torch": cpu_torch,
        "InverseKinematics": _NeverIK,
        "InverseKinematicsCfg": _NeverIK,
        "TrajectoryOptimizer": FakeTrajectoryOptimizer,
        "TrajectoryOptimizerCfg": FakeTrajectoryOptimizerCfg,
        "GoalToolPose": _GoalToolPose,
        "JointState": _JointState,
        "Pose": object,
    }
    adapter._robot_config = lambda: {}
    adapter._scene_config = lambda: {}
    adapter._target_link = lambda: "tool0"
    return FakeTrajectoryOptimizer, FakeTrajectoryOptimizerCfg


class _ValidationCollisionChecker:
    def __init__(self, *, nonfinite_query: str | None = None) -> None:
        self.nonfinite_query = nonfinite_query
        self.batch_sizes: list[int] = []

    def get_kinematics(self, joint_positions):
        self.batch_sizes.append(int(joint_positions.shape[1]))
        spheres = torch.zeros(
            (1, joint_positions.shape[1], 2, 4),
            dtype=torch.float32,
        )
        # The first two joints encode independent scene and self clearances.
        spheres[..., 0, 0] = joint_positions[..., 0]
        spheres[..., 1, 0] = joint_positions[..., 1]
        return SimpleNamespace(robot_spheres=spheres)

    def get_collision_distance(self, state):
        result = torch.zeros(state.robot_spheres.shape[:2], dtype=torch.float32)
        if self.nonfinite_query == "scene":
            result[0, 0] = float("nan")
        return result

    @staticmethod
    def get_collision_constraint(spheres):
        return torch.clamp(
            spheres[..., 0, 3] - spheres[..., 0, 0],
            min=0.0,
        )

    def get_self_collision_distance(self, spheres):
        result = torch.clamp(
            spheres[..., 0, 3] + spheres[..., 1, 3] - spheres[..., 1, 0],
            min=0.0,
        )
        if self.nonfinite_query == "self":
            result[0, 0] = float("nan")
        return result

    def get_bound(self, joint_positions):
        result = torch.zeros_like(joint_positions)
        if self.nonfinite_query == "bound":
            result[0, 0, 0] = float("nan")
        return result


def _validation_adapter(
    repository_root,
    environment,
    *,
    nonfinite_query: str | None = None,
) -> tuple[CuroboAdapterV2, _ValidationCollisionChecker]:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    checker = _ValidationCollisionChecker(nonfinite_query=nonfinite_query)
    adapter._runtime = {"torch": _CpuTorch()}
    adapter._collision_checker = checker
    return adapter, checker


def _compiled_trajectory(
    environment,
    positions: list[tuple[float, ...]],
) -> CompiledTrajectory:
    return CompiledTrajectory(
        joint_names=environment.joint_names,
        samples=tuple(
            JointSample(
                time_s=0.01 * index,
                positions=row,
                velocities=(0.0,) * len(environment.joint_names),
                accelerations=(0.0,) * len(environment.joint_names),
                phase="draw",
            )
            for index, row in enumerate(positions)
        ),
    )


def test_final_validation_reports_measured_scene_and_self_clearance(
    repository_root,
    environment,
) -> None:
    adapter, _ = _validation_adapter(repository_root, environment)
    trajectory = _compiled_trajectory(
        environment,
        [
            (0.08, 0.03, 0.0, 0.0, 0.0, 0.0),
            (0.04, 0.07, 0.0, 0.0, 0.0, 0.0),
        ],
    )

    metrics = adapter.validate_joint_trajectory(trajectory)

    assert metrics.minimum_collision_distance_m == pytest.approx(0.03, abs=5.0e-6)
    assert metrics.minimum_collision_distance_m != pytest.approx(
        environment.planner["validation"]["minimum_collision_distance_m"]
    )


def test_final_validation_rejects_insufficient_self_clearance(
    repository_root,
    environment,
) -> None:
    adapter, _ = _validation_adapter(repository_root, environment)
    trajectory = _compiled_trajectory(
        environment,
        [
            (0.08, 0.004, 0.0, 0.0, 0.0, 0.0),
            (0.07, 0.006, 0.0, 0.0, 0.0, 0.0),
        ],
    )

    with pytest.raises(PlanningError, match="scene/self clearance margin"):
        adapter.validate_joint_trajectory(trajectory)


def test_final_validation_accepts_clearance_just_above_configured_threshold(
    repository_root,
    environment,
) -> None:
    adapter, _ = _validation_adapter(repository_root, environment)
    required = float(
        environment.planner["validation"]["minimum_collision_distance_m"]
    )
    trajectory = _compiled_trajectory(
        environment,
        [
            (0.08, required + 1.0e-6, 0.0, 0.0, 0.0, 0.0),
            (0.07, required + 3.0e-6, 0.0, 0.0, 0.0, 0.0),
        ],
    )

    metrics = adapter.validate_joint_trajectory(trajectory)

    assert metrics.minimum_collision_distance_m >= required - 1.0e-12


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("positions", (float("nan"), 0.03, 0.0, 0.0, 0.0, 0.0)),
        ("velocities", (float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("accelerations", (float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0)),
        ("time_s", float("nan")),
    ],
)
def test_final_validation_rejects_nonfinite_input(
    repository_root,
    environment,
    field,
    value,
) -> None:
    adapter, _ = _validation_adapter(repository_root, environment)
    trajectory = _compiled_trajectory(
        environment,
        [
            (0.08, 0.03, 0.0, 0.0, 0.0, 0.0),
            (0.07, 0.03, 0.0, 0.0, 0.0, 0.0),
        ],
    )
    object.__setattr__(trajectory.samples[0], field, value)

    with pytest.raises(PlanningError, match="non-finite"):
        adapter.validate_joint_trajectory(trajectory)


@pytest.mark.parametrize("query", ["scene", "self", "bound"])
def test_final_validation_rejects_nonfinite_collision_or_bound_query(
    repository_root,
    environment,
    query,
) -> None:
    adapter, _ = _validation_adapter(
        repository_root,
        environment,
        nonfinite_query=query,
    )
    trajectory = _compiled_trajectory(
        environment,
        [
            (0.08, 0.03, 0.0, 0.0, 0.0, 0.0),
            (0.07, 0.03, 0.0, 0.0, 0.0, 0.0),
        ],
    )

    with pytest.raises(PlanningError, match=f"{query}.*non-finite"):
        adapter.validate_joint_trajectory(trajectory)


def test_final_clearance_validation_is_chunked(
    repository_root,
    environment,
) -> None:
    adapter, checker = _validation_adapter(repository_root, environment)
    trajectory = _compiled_trajectory(
        environment,
        [(0.05, 0.06, 0.0, 0.0, 0.0, 0.0)] * 513,
    )

    metrics = adapter.validate_joint_trajectory(trajectory)

    assert metrics.checked_sample_count == 513
    assert max(checker.batch_sizes) <= 256
    # Each full chunk is evaluated once for costs, once for the exact 5 mm
    # threshold predicate, and once for the reporting clearance search.
    assert checker.batch_sizes.count(256) == 6


def test_joint_trajopt_uses_complete_horizon_goal_and_seed(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    solver_type, config_type = _adapter_runtime(adapter)
    poses = _tool_poses(4)
    times = tuple(pose.time_s for pose in poses)
    seed = tuple(tuple(0.01 * index for _ in environment.joint_names) for index in range(4))

    result = adapter.optimize_pose_path(poses, times, seed)

    solver = solver_type.instance
    assert solver.solve_calls == 1
    assert solver.config.interpolation_buffer_size == 65_536
    assert solver.config.interpolation_dt == pytest.approx(0.025)
    assert solver.interpolated_step_count == 4 * (len(poses) - 1) + 1
    assert solver.solve_kwargs["current_state"].dt is None
    assert solver.solve_kwargs["seed_traj"].shape == (1, 1, 4, 6)
    assert solver.solve_kwargs["goal_tool_poses"].position.shape == (1, 1, 1, 1, 3)
    tracking_goal = solver.optimizer_rollouts[0].cost_manager.get_cost(
        "tool_pose"
    )._tracking_goal
    assert tracking_goal.position.shape == (1, 4, 1, 1, 3)
    assert tracking_goal.quaternion.shape == (1, 4, 1, 1, 4)
    assert config_type.last_kwargs["transition_model"]["transition_model_cfg"]["horizon"] == 4
    assert (
        config_type.last_kwargs["transition_model"]["transition_model_cfg"]["control_space"]
        == "POSITION"
    )
    assert result.times_s == times
    assert np.allclose(result.positions, seed)


def test_joint_trajopt_propagates_teleport_dt_without_interpolation_inflation(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    solver_type, _ = _adapter_runtime(adapter)
    poses = _tool_poses(32, dt_s=0.01)
    times = tuple(pose.time_s for pose in poses)
    seed = tuple(tuple(0.001 * index for _ in environment.joint_names) for index in range(32))

    result = adapter.optimize_pose_path(poses, times, seed)

    solver = solver_type.instance
    assert solver.solve_kwargs["dt"] == pytest.approx(0.01)
    assert torch.allclose(
        solver.dt_at_seed_rollout,
        torch.full_like(solver.dt_at_seed_rollout, 0.01),
    )
    assert torch.allclose(
        solver.dt_at_interpolation,
        torch.full_like(solver.dt_at_interpolation, 0.01),
    )
    assert torch.allclose(
        solver.last_result.js_solution.dt,
        torch.full_like(solver.last_result.js_solution.dt, 0.01),
    )
    assert torch.allclose(
        solver.last_active_js.dt,
        torch.full_like(solver.last_active_js.dt, 0.01),
    )
    expected_dense = _locked_curobo_interpolated_step_count(
        0.01,
        0.005,
        len(poses),
    )
    default_inflated = _locked_curobo_interpolated_step_count(
        1.0,
        0.025,
        len(poses),
    )
    assert expected_dense == 2 * (len(poses) - 1) + 1
    assert default_inflated >= 40 * (len(poses) - 1) + 1
    assert solver.interpolated_step_count == expected_dense
    assert solver.interpolated_step_count < default_inflated
    assert result.times_s == times
    assert len(result.positions) == len(poses)
    assert np.allclose(result.positions, seed)


def test_fixed_time_interpolation_capacity_tracks_actual_horizon(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    configured = int(environment.planner["curobo"]["interpolation_buffer_size"])
    long_horizon = configured + 123
    interpolation_dt = adapter._fixed_time_interpolation_dt(0.01, 0.025)
    expected_steps_max = 2 * (long_horizon - 1) + 1

    assert interpolation_dt == pytest.approx(0.005)
    assert (
        adapter._fixed_time_interpolation_buffer_size(
            configured // 2,
            0.01,
            interpolation_dt,
        )
        == configured
    )
    assert (
        adapter._fixed_time_interpolation_buffer_size(
            long_horizon,
            0.01,
            interpolation_dt,
        )
        == expected_steps_max
    )


def test_fixed_time_interpolation_capacity_fails_closed_at_gpu_limit(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    interpolation_dt = adapter._fixed_time_interpolation_dt(0.01, 0.025)

    assert (
        adapter._fixed_time_interpolation_buffer_size(
            500_000,
            0.01,
            interpolation_dt,
        )
        == 999_999
    )
    with pytest.raises(PlanningError, match="more than 1000000 samples"):
        adapter._fixed_time_interpolation_buffer_size(
            500_001,
            0.01,
            interpolation_dt,
        )


def test_fixed_time_interpolation_uses_float32_exact_power_of_two_subdivision(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    base_dt = 0.107152

    interpolation_dt = adapter._fixed_time_interpolation_dt(base_dt, 0.025)
    actual_steps = adapter._curobo_linear_interpolation_steps_per_interval(
        base_dt,
        interpolation_dt,
    )

    assert interpolation_dt == pytest.approx(float(np.float32(base_dt) / np.float32(8)))
    assert interpolation_dt <= 0.025
    assert actual_steps == 8


def test_fixed_time_dense_metrics_detect_collision_between_safe_knots(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    solver_type, _ = _adapter_runtime(adapter, midpoint_collision=True)
    poses = _tool_poses(2, dt_s=0.01)
    seed = ((0.0,) * 6, (0.01,) * 6)

    with pytest.raises(PlanningError, match="trajectory optimization failed"):
        adapter.optimize_pose_path(poses, (0.0, 0.01), seed)

    solver = solver_type.instance
    assert solver.interpolation_steps_per_interval == 2
    assert solver.interpolated_step_count == 3
    assert solver.collision_sample_fractions == (0.0, 0.5, 1.0)
    assert solver.collision_samples == (False, True, False)
    assert solver.midpoint_collision_detected is True


def test_joint_trajopt_solver_failure_rejects_complete_path(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    _adapter_runtime(adapter, solver_success=False)
    poses = _tool_poses(3)
    seed = ((0.0,) * 6, (0.01,) * 6, (0.02,) * 6)

    with pytest.raises(PlanningError, match="trajectory optimization failed"):
        adapter.optimize_pose_path(poses, (0.0, 0.1, 0.2), seed)


@pytest.mark.parametrize("count_delta", [-1, 1])
def test_joint_trajopt_changed_constraint_count_is_rejected(
    repository_root,
    environment,
    count_delta,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    _adapter_runtime(adapter, result_count_delta=count_delta)
    poses = _tool_poses(3)
    seed = ((0.0,) * 6, (0.01,) * 6, (0.02,) * 6)

    with pytest.raises(PlanningError, match="constraint count"):
        adapter.optimize_pose_path(poses, (0.0, 0.1, 0.2), seed)


def test_joint_trajopt_any_fk_tolerance_failure_rejects_path(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    _adapter_runtime(adapter, fk_position_error_index=1)
    poses = _tool_poses(3)
    seed = ((0.0,) * 6, (0.01,) * 6, (0.02,) * 6)

    with pytest.raises(PlanningError, match="index 1"):
        adapter.optimize_pose_path(poses, (0.0, 0.1, 0.2), seed)


def test_horizon_tool_pose_cost_is_vectorized_and_differentiable() -> None:
    target_position = torch.zeros((1, 3, 1, 1, 3), dtype=torch.float32)
    target_quaternion = torch.zeros((1, 3, 1, 1, 4), dtype=torch.float32)
    target_quaternion[..., 0] = 1.0
    goal = _GoalToolPose(["tool0"], target_position, target_quaternion)
    cost = _TrajectoryTrackingToolPoseCost(_FakeCost(), torch, goal)
    current_position = torch.full(
        (1, 3, 1, 3),
        0.01,
        dtype=torch.float32,
        requires_grad=True,
    )
    current_quaternion = target_quaternion[:, :, :, 0, :].clone().requires_grad_(True)
    current = SimpleNamespace(
        tool_frames=["tool0"],
        position=current_position,
        quaternion=current_quaternion,
    )

    value, position_error, orientation_error, _ = cost.forward(current, goal)
    value.sum().backward()

    assert value.shape == (1, 3, 2)
    assert position_error.shape == (1, 3, 1)
    assert orientation_error.shape == (1, 3, 1)
    assert current_position.grad is not None
    assert torch.all(torch.isfinite(current_position.grad))


def test_true_time_velocity_regularizer_is_resampling_invariant() -> None:
    def total_cost(times: list[float]) -> float:
        position = torch.tensor(times, dtype=torch.float32).reshape(1, -1, 1)
        cost = _TrajectoryTrackingCSpaceCost(
            _FakeCost((1.0,)),
            torch,
            torch.zeros_like(position),
            torch.tensor(times, dtype=torch.float32),
            torch.ones((1, 1, 1), dtype=torch.float32),
            torch.ones((1, 1, 1), dtype=torch.float32),
            torch.ones((1, 1, 1), dtype=torch.float32),
            joint_distance_weight=0.0,
            velocity_weight=1.0,
            acceleration_weight=0.0,
            jerk_weight=0.0,
        )
        return float(cost.forward(SimpleNamespace(position=position)).sum())

    assert total_cost([0.0, 0.5, 1.0]) == pytest.approx(1.0)
    assert total_cost([0.0, 0.25, 0.5, 0.75, 1.0]) == pytest.approx(1.0)
