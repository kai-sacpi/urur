from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
import torch

from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
from hx_waypoint_curobo.errors import PlanningError
from hx_waypoint_curobo.models import TimedPose


class _CpuTorch:
    float32 = torch.float32

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
        if isinstance(data, list) and any(isinstance(item, np.ndarray) for item in data):
            data = np.asarray(data)
        return torch.tensor(data, *args, **self._cpu_kwargs(kwargs))

    def full(self, size, fill_value, *args, **kwargs):
        return torch.full(size, fill_value, *args, **self._cpu_kwargs(kwargs))


class _Pose:
    def __init__(self, position, quaternion) -> None:
        self.position = position
        self.quaternion = quaternion


class _GoalToolPose:
    def __init__(self, tool_frames=None, position=None, quaternion=None) -> None:
        self.tool_frames = tool_frames
        self.position = position
        self.quaternion = quaternion

    @classmethod
    def from_poses(cls, poses, ordered_tool_frames, num_goalset):
        del ordered_tool_frames, num_goalset
        return SimpleNamespace(pose=next(iter(poses.values())))


class _JointState:
    @classmethod
    def from_position(cls, position, joint_names=None):
        return SimpleNamespace(position=position, joint_names=joint_names)


class _FakeIKCfg:
    last_kwargs = None

    @classmethod
    def create(cls, **kwargs):
        cls.last_kwargs = kwargs
        return SimpleNamespace(**kwargs)


class _FakeIK:
    instance = None
    solutions = (
        np.asarray(
            [
                [0.020, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.050, 0.1, 0.1, 0.1, 0.1, 0.1],
            ],
            dtype=np.float32,
        ),
        np.asarray(
            [
                [0.030, 0.15, 0.15, 0.15, 0.15, 0.15],
                [0.080, 0.20, 0.20, 0.20, 0.20, 0.20],
            ],
            dtype=np.float32,
        ),
    )

    def __init__(self, config) -> None:
        self.config = config
        self.solve_calls = []
        limits = torch.tensor([[-2.0] * 6, [2.0] * 6], dtype=torch.float32)
        self.kinematics = SimpleNamespace(
            get_joint_limits=lambda: SimpleNamespace(position=limits)
        )
        _FakeIK.instance = self

    def solve_pose(self, **kwargs):
        call_index = len(self.solve_calls)
        self.solve_calls.append(kwargs)
        solution = torch.tensor(self.solutions[call_index]).unsqueeze(0)
        success = torch.ones((1, len(solution[0])), dtype=torch.bool)
        return SimpleNamespace(
            js_solution=SimpleNamespace(position=solution),
            success=success,
            feasible=success.clone(),
        )

    @staticmethod
    def compute_kinematics(state):
        identity = torch.eye(
            state.position.shape[-1],
            dtype=state.position.dtype,
            device=state.position.device,
        ).reshape(1, 1, 1, state.position.shape[-1], state.position.shape[-1])
        return SimpleNamespace(
            tool_jacobians=identity.repeat(len(state.position), 1, 1, 1, 1)
        )


class _FakeMotionPlannerCfg:
    last_kwargs = None

    @classmethod
    def create(cls, **kwargs):
        cls.last_kwargs = kwargs
        return SimpleNamespace(
            trajopt_solver_config=SimpleNamespace(interpolation_buffer_size=None)
        )


class _FakeMotionPlanner:
    instance = None

    def __init__(self, config) -> None:
        self.config = config
        self.warmup_calls = []
        _FakeMotionPlanner.instance = self

    def warmup(self, **kwargs):
        self.warmup_calls.append(kwargs)
        return True


class _FakeCollisionChecker:
    @staticmethod
    def get_kinematics(joint_positions):
        clearance = joint_positions[..., 0]
        spheres = torch.zeros((*clearance.shape, 1, 4), dtype=torch.float32)
        spheres[..., 0] = clearance.unsqueeze(-1)
        return SimpleNamespace(robot_spheres=spheres)

    @staticmethod
    def get_collision_constraint(spheres):
        clearance = spheres[..., 0, 0]
        radius = spheres[..., 0, 3]
        return torch.clamp(radius - clearance, min=0.0)

    @staticmethod
    def get_self_collision_distance(spheres):
        return torch.zeros(spheres.shape[:2], dtype=torch.float32)


def _poses() -> tuple[TimedPose, TimedPose]:
    return (
        TimedPose(
            time_s=0.0,
            frame_id="base_link",
            position_m=(0.4, 0.1, 0.3),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            phase="draw",
        ),
        TimedPose(
            time_s=0.1,
            frame_id="base_link",
            position_m=(0.401, 0.1, 0.3),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            phase="draw",
        ),
    )


def _adapter(repository_root, environment) -> CuroboAdapterV2:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    adapter._runtime = {
        "torch": _CpuTorch(),
        "InverseKinematics": _FakeIK,
        "InverseKinematicsCfg": _FakeIKCfg,
        "GoalToolPose": _GoalToolPose,
        "Pose": _Pose,
        "JointState": _JointState,
    }
    adapter._collision_checker = _FakeCollisionChecker()
    adapter._robot_config = lambda: {}
    adapter._scene_config = lambda: {}
    adapter._target_link = lambda: "tool0"
    return adapter


def test_ik_uses_previous_feasible_frontier_and_measures_clearance(
    repository_root,
    environment,
) -> None:
    adapter = _adapter(repository_root, environment)

    candidates = adapter.solve_ik_candidates(_poses(), candidates_per_pose=2)

    solver = _FakeIK.instance
    assert _FakeIKCfg.last_kwargs["max_batch_size"] == 1
    assert _FakeIKCfg.last_kwargs["use_cuda_graph"] is False
    assert _FakeIKCfg.last_kwargs["num_seeds"] == 32
    assert _FakeIKCfg.last_kwargs["position_tolerance"] == pytest.approx(0.001)
    assert _FakeIKCfg.last_kwargs["orientation_tolerance"] == pytest.approx(
        np.radians(0.5)
    )
    assert solver.solve_calls[0]["seed_config"] is None
    second_seed = solver.solve_calls[1]["seed_config"].detach().cpu().numpy()
    assert second_seed.shape == (1, 2, 6)
    assert np.allclose(second_seed[0], _FakeIK.solutions[0])
    assert [candidate.collision_distance_m for candidate in candidates[0]] == pytest.approx(
        [0.050, 0.020],
        abs=5.0e-6,
    )
    assert [candidate.collision_distance_m for candidate in candidates[1]] == pytest.approx(
        [0.080, 0.030],
        abs=5.0e-6,
    )


def test_prepare_motion_planner_disables_cuda_graphs(
    repository_root,
    environment,
) -> None:
    adapter = _adapter(repository_root, environment)
    adapter._runtime.update(
        {
            "MotionPlanner": _FakeMotionPlanner,
            "MotionPlannerCfg": _FakeMotionPlannerCfg,
        }
    )

    adapter.warmup()

    assert _FakeMotionPlannerCfg.last_kwargs["use_cuda_graph"] is False
    assert _FakeMotionPlannerCfg.last_kwargs[
        "optimizer_collision_activation_distance"
    ] == pytest.approx(0.005)
    assert _FakeMotionPlanner.instance.config.trajopt_solver_config.interpolation_buffer_size == (
        environment.planner["curobo"]["interpolation_buffer_size"]
    )
    assert _FakeMotionPlanner.instance.warmup_calls == [
        {"enable_graph": False, "num_warmup_iterations": 5}
    ]


def test_full_previous_frontier_keeps_random_seed_reserve_and_diversity(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    clustered = np.zeros((24, 6), dtype=np.float32)
    clustered[:, 0] = 0.05 + np.arange(24, dtype=np.float32) * 0.001
    clustered[1] = clustered[0]
    clustered[2] = clustered[0]
    exploratory = np.zeros((8, 6), dtype=np.float32)
    exploratory[:, 0] = 0.10
    exploratory[:, 1] = np.linspace(-1.4, 1.4, 8, dtype=np.float32)
    first_solutions = np.concatenate((clustered, exploratory), axis=0)
    second_solutions = first_solutions.copy()
    second_solutions[:, 0] += 0.0001
    monkeypatch.setattr(
        _FakeIK,
        "solutions",
        (first_solutions, second_solutions),
    )
    adapter = _adapter(repository_root, environment)

    adapter.solve_ik_candidates(_poses(), candidates_per_pose=32)

    solver = _FakeIK.instance
    previous_seeds = solver.solve_calls[1]["seed_config"].detach().cpu().numpy()[0]
    assert _FakeIKCfg.last_kwargs["num_seeds"] == 32
    assert previous_seeds.shape == (24, 6)
    assert 32 - len(previous_seeds) == environment.planner["ik"]["random_seed_reserve"]
    assert len(np.unique(previous_seeds, axis=0)) == len(previous_seeds)
    assert np.allclose(previous_seeds[0], first_solutions[0])
    assert float(np.max(np.abs(previous_seeds[:, 1]))) >= 1.0


def test_trajectory_optimizer_uses_guarded_internal_tolerances(
    repository_root,
    environment,
) -> None:
    class ConfigCaptured(RuntimeError):
        pass

    class CapturingTrajectoryOptimizerCfg:
        last_kwargs = None

        @classmethod
        def create(cls, **kwargs):
            cls.last_kwargs = kwargs
            raise ConfigCaptured

    adapter = _adapter(repository_root, environment)
    adapter._runtime.update(
        {
            "TrajectoryOptimizer": object,
            "TrajectoryOptimizerCfg": CapturingTrajectoryOptimizerCfg,
        }
    )
    poses = _poses()
    seed = ((0.0,) * 6, (0.01,) * 6)

    with pytest.raises(ConfigCaptured):
        adapter.optimize_pose_path(
            poses,
            tuple(pose.time_s for pose in poses),
            seed,
        )

    assert CapturingTrajectoryOptimizerCfg.last_kwargs["position_tolerance"] == pytest.approx(
        0.0008
    )
    assert CapturingTrajectoryOptimizerCfg.last_kwargs[
        "orientation_tolerance"
    ] == pytest.approx(np.radians(0.4))


def test_empty_clearance_filtered_frontier_fails_closed(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    adapter = _adapter(repository_root, environment)

    def no_solution(self, **kwargs):
        del self, kwargs
        solution = torch.zeros((1, 1, 6), dtype=torch.float32)
        failed = torch.zeros((1, 1), dtype=torch.bool)
        return SimpleNamespace(
            js_solution=SimpleNamespace(position=solution),
            success=failed,
            feasible=failed,
    )

    monkeypatch.setattr(_FakeIK, "solve_pose", no_solution)
    with pytest.raises(
        PlanningError,
        match=(
            r"pose index 0: raw accepted count=0, required clearance=0\.005 m"
        ),
    ):
        adapter.solve_ik_candidates(_poses(), candidates_per_pose=1)


def test_clearance_filter_removes_bad_candidates_before_seed_feedback(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    first_solutions = np.asarray(
        [
            [0.004, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.020, 0.1, 0.1, 0.1, 0.1, 0.1],
        ],
        dtype=np.float32,
    )
    second_solutions = np.asarray(
        [
            [0.030, 0.2, 0.2, 0.2, 0.2, 0.2],
            [0.040, 0.3, 0.3, 0.3, 0.3, 0.3],
        ],
        dtype=np.float32,
    )
    monkeypatch.setattr(
        _FakeIK,
        "solutions",
        (first_solutions, second_solutions),
    )
    adapter = _adapter(repository_root, environment)

    candidates = adapter.solve_ik_candidates(_poses(), candidates_per_pose=2)

    previous_seeds = _FakeIK.instance.solve_calls[1]["seed_config"].detach().cpu().numpy()
    assert previous_seeds.shape == (1, 1, 6)
    assert np.allclose(previous_seeds[0, 0], first_solutions[1])
    assert len(candidates[0]) == 1
    assert candidates[0][0].positions == pytest.approx(first_solutions[1])


def test_clearance_filter_reports_raw_candidate_count_and_threshold(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    below_threshold = np.asarray(
        [
            [0.0040, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0049, 0.1, 0.1, 0.1, 0.1, 0.1],
        ],
        dtype=np.float32,
    )
    monkeypatch.setattr(
        _FakeIK,
        "solutions",
        (below_threshold, below_threshold),
    )
    adapter = _adapter(repository_root, environment)

    with pytest.raises(
        PlanningError,
        match=(
            r"pose index 0: raw accepted count=2, required clearance=0\.005 m"
        ),
    ):
        adapter.solve_ik_candidates(_poses(), candidates_per_pose=2)


def test_ik_filters_candidates_that_do_not_extend_the_reachable_frontier(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    first = np.asarray(
        [
            [0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.05, 0.1, 0.1, 0.1, 0.1, 0.1],
        ],
        dtype=np.float32,
    )
    second = np.asarray(
        [
            [0.03, 0.1, 0.1, 0.1, 0.1, 0.1],
            [0.08, 1.0, 1.0, 1.0, 1.0, 1.0],
        ],
        dtype=np.float32,
    )
    monkeypatch.setattr(_FakeIK, "solutions", (first, second))
    adapter = _adapter(repository_root, environment)

    candidates = adapter.solve_ik_candidates(_poses(), candidates_per_pose=2)

    assert len(candidates[1]) == 1
    assert candidates[1][0].positions == pytest.approx(second[0])


def test_clearance_threshold_uses_one_scene_and_self_inflation_query(
    repository_root,
    environment,
) -> None:
    class RecordingCollisionChecker(_FakeCollisionChecker):
        scene_radii: list[np.ndarray] = []
        self_radii: list[np.ndarray] = []

        @classmethod
        def get_collision_constraint(cls, spheres):
            cls.scene_radii.append(spheres[..., 3].detach().cpu().numpy().copy())
            return super().get_collision_constraint(spheres)

        @classmethod
        def get_self_collision_distance(cls, spheres):
            cls.self_radii.append(spheres[..., 3].detach().cpu().numpy().copy())
            return super().get_self_collision_distance(spheres)

    adapter = _adapter(repository_root, environment)
    adapter._collision_checker = RecordingCollisionChecker()
    positions = np.asarray([[0.02, 0.0, 0.0, 0.0, 0.0, 0.0]], dtype=np.float32)

    mask = adapter._ik_clearance_threshold_mask(adapter._runtime, positions)

    assert mask.tolist() == [True]
    assert len(RecordingCollisionChecker.scene_radii) == 1
    assert len(RecordingCollisionChecker.self_radii) == 1
    assert RecordingCollisionChecker.scene_radii[0][0, 0, 0] == pytest.approx(0.005)
    assert RecordingCollisionChecker.self_radii[0][0, 0, 0] == pytest.approx(0.0025)


def test_missing_feasible_mask_fails_closed(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    adapter = _adapter(repository_root, environment)

    def missing_feasible(self, **kwargs):
        del self, kwargs
        solution = torch.zeros((1, 1, 6), dtype=torch.float32)
        return SimpleNamespace(
            js_solution=SimpleNamespace(position=solution),
            success=torch.ones((1, 1), dtype=torch.bool),
        )

    monkeypatch.setattr(_FakeIK, "solve_pose", missing_feasible)
    with pytest.raises(PlanningError, match="no feasible mask"):
        adapter.solve_ik_candidates(_poses(), candidates_per_pose=1)


def test_ik_result_joint_order_mismatch_fails_closed(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    adapter = _adapter(repository_root, environment)

    def wrong_joint_order(self, **kwargs):
        del self, kwargs
        solution = torch.zeros((1, 1, 6), dtype=torch.float32)
        success = torch.ones((1, 1), dtype=torch.bool)
        return SimpleNamespace(
            js_solution=SimpleNamespace(
                position=solution,
                joint_names=list(reversed(environment.joint_names)),
            ),
            success=success,
            feasible=success.clone(),
        )

    monkeypatch.setattr(_FakeIK, "solve_pose", wrong_joint_order)
    with pytest.raises(PlanningError, match="authoritative order"):
        adapter.solve_ik_candidates(_poses(), candidates_per_pose=1)


def test_candidate_clearance_queries_are_chunked(
    repository_root,
    environment,
) -> None:
    adapter = _adapter(repository_root, environment)

    class RecordingCollisionChecker(_FakeCollisionChecker):
        batch_sizes: list[int] = []

        @classmethod
        def get_kinematics(cls, joint_positions):
            cls.batch_sizes.append(int(joint_positions.shape[1]))
            return super().get_kinematics(joint_positions)

    adapter._collision_checker = RecordingCollisionChecker()
    positions = np.zeros((513, len(environment.joint_names)), dtype=np.float32)
    positions[:, 0] = 0.05

    clearances = adapter._candidate_collision_clearances(adapter._runtime, positions)

    assert RecordingCollisionChecker.batch_sizes == [256, 256, 1]
    assert clearances == pytest.approx(np.full(513, 0.05), abs=5.0e-6)


def _diagonal_tool_jacobians(positions: torch.Tensor) -> torch.Tensor:
    diagonal = torch.stack(
        (
            torch.abs(positions[:, 0]) + 0.25,
            torch.abs(positions[:, 1]) + 0.50,
            torch.abs(positions[:, 2]) + 0.75,
            torch.abs(positions[:, 3]) + 1.00,
            torch.abs(positions[:, 4]) + 1.25,
            torch.abs(positions[:, 5]) + 1.50,
        ),
        dim=-1,
    )
    return torch.diag_embed(diagonal).unsqueeze(1).unsqueeze(2)


class _BatchJacobianSolver:
    def __init__(self) -> None:
        self.batch_sizes: list[int] = []

    def compute_kinematics(self, state):
        self.batch_sizes.append(int(state.position.shape[0]))
        return SimpleNamespace(
            tool_jacobians=_diagonal_tool_jacobians(state.position)
        )


def _legacy_singularity_scores(positions: np.ndarray) -> np.ndarray:
    if len(positions) == 0:
        return np.empty((0,), dtype=float)
    jacobians = _diagonal_tool_jacobians(torch.as_tensor(positions)).reshape(
        -1,
        6,
        6,
    )
    scores = []
    for matrix in jacobians:
        singular_values = torch.linalg.svdvals(matrix)
        minimum = float(torch.min(singular_values).item())
        maximum = float(torch.max(singular_values).item())
        scores.append(maximum / max(minimum, 1.0e-6))
    return np.asarray(scores, dtype=float)


def test_candidate_singularity_batch_matches_legacy_per_candidate_formula(
    repository_root,
    environment,
) -> None:
    adapter = _adapter(repository_root, environment)
    positions = np.linspace(
        -0.6,
        0.7,
        37 * len(environment.joint_names),
        dtype=np.float32,
    ).reshape(37, len(environment.joint_names))
    solver = _BatchJacobianSolver()

    scores = adapter._candidate_singularity_scores(solver, positions)

    assert solver.batch_sizes == [37]
    assert scores == pytest.approx(_legacy_singularity_scores(positions), rel=1.0e-7)


@pytest.mark.parametrize(
    ("candidate_count", "expected_batch_sizes"),
    (
        (0, []),
        (1, [1]),
        (2048, [2048]),
        (2049, [2048, 1]),
        (4097, [2048, 2048, 1]),
    ),
)
def test_candidate_singularity_queries_use_bounded_chunks_without_reordering(
    repository_root,
    environment,
    candidate_count,
    expected_batch_sizes,
) -> None:
    adapter = _adapter(repository_root, environment)
    positions = np.zeros(
        (candidate_count, len(environment.joint_names)),
        dtype=np.float32,
    )
    positions[:, 0] = np.linspace(0.0, 0.75, candidate_count, dtype=np.float32)
    solver = _BatchJacobianSolver()

    scores = adapter._candidate_singularity_scores(solver, positions)

    assert solver.batch_sizes == expected_batch_sizes
    assert scores == pytest.approx(_legacy_singularity_scores(positions), rel=1.0e-7)


def test_candidate_singularity_preserves_near_singular_clamp_formula(
    repository_root,
    environment,
) -> None:
    jacobians = torch.diag_embed(
        torch.tensor(
            (
                (5.0e-7, 0.25, 0.50, 0.75, 0.90, 1.00),
                (0.0, 0.25, 0.50, 1.00, 1.50, 2.00),
            ),
            dtype=torch.float32,
        )
    ).unsqueeze(1).unsqueeze(2)

    class NearSingularSolver:
        @staticmethod
        def compute_kinematics(state):
            assert len(state.position) == 2
            return SimpleNamespace(tool_jacobians=jacobians)

    adapter = _adapter(repository_root, environment)
    positions = np.zeros((2, len(environment.joint_names)), dtype=np.float32)

    scores = adapter._candidate_singularity_scores(NearSingularSolver(), positions)

    assert scores == pytest.approx((1.0e6, 2.0e6), rel=1.0e-7)


def test_candidate_singularity_missing_jacobians_fail_closed(
    repository_root,
    environment,
) -> None:
    class NoneJacobianSolver:
        def __init__(self) -> None:
            self.batch_sizes: list[int] = []

        def compute_kinematics(self, state):
            self.batch_sizes.append(int(state.position.shape[0]))
            return SimpleNamespace(tool_jacobians=None)

    adapter = _adapter(repository_root, environment)
    positions = np.zeros((4097, len(environment.joint_names)), dtype=np.float32)
    solver = NoneJacobianSolver()

    with pytest.raises(PlanningError, match="no tool Jacobians"):
        adapter._candidate_singularity_scores(solver, positions)

    assert solver.batch_sizes == [2048]


@pytest.mark.parametrize(
    "positions",
    (
        np.zeros((2, 5), dtype=np.float32),
        np.zeros((2, 6, 1), dtype=np.float32),
        np.asarray([[0.0, 0.0, 0.0, np.nan, 0.0, 0.0]], dtype=np.float32),
    ),
)
def test_candidate_singularity_invalid_inputs_fail_closed(
    repository_root,
    environment,
    positions,
) -> None:
    adapter = _adapter(repository_root, environment)

    with pytest.raises(PlanningError, match="singularity input"):
        adapter._candidate_singularity_scores(_BatchJacobianSolver(), positions)


@pytest.mark.parametrize(
    "jacobians",
    (
        torch.zeros((2, 5, 6), dtype=torch.float32),
        torch.zeros((1, 6, 6), dtype=torch.float32),
        torch.zeros((1, 2, 1, 6, 6), dtype=torch.float32),
        torch.zeros((2, 1, 6, 6), dtype=torch.float32),
    ),
)
def test_candidate_singularity_malformed_jacobians_fail_closed(
    repository_root,
    environment,
    jacobians,
) -> None:
    class MalformedJacobianSolver:
        @staticmethod
        def compute_kinematics(state):
            del state
            return SimpleNamespace(tool_jacobians=jacobians)

    adapter = _adapter(repository_root, environment)
    positions = np.zeros((2, len(environment.joint_names)), dtype=np.float32)

    with pytest.raises(PlanningError, match="Jacobian shape"):
        adapter._candidate_singularity_scores(MalformedJacobianSolver(), positions)


def test_candidate_singularity_nonfinite_jacobians_fail_closed(
    repository_root,
    environment,
) -> None:
    class NonfiniteJacobianSolver:
        @staticmethod
        def compute_kinematics(state):
            jacobians = torch.zeros(
                (len(state.position), 1, 1, 6, len(environment.joint_names)),
                dtype=torch.float32,
            )
            jacobians[0, 0, 0, 0, 0] = torch.nan
            return SimpleNamespace(tool_jacobians=jacobians)

    adapter = _adapter(repository_root, environment)
    positions = np.zeros((2, len(environment.joint_names)), dtype=np.float32)

    with pytest.raises(PlanningError, match="non-finite.*Jacobians"):
        adapter._candidate_singularity_scores(NonfiniteJacobianSolver(), positions)


def test_candidate_singularity_fk_exception_fails_closed(
    repository_root,
    environment,
) -> None:
    class FailingJacobianSolver:
        @staticmethod
        def compute_kinematics(state):
            del state
            raise RuntimeError("synthetic FK failure")

    adapter = _adapter(repository_root, environment)
    positions = np.zeros((2, len(environment.joint_names)), dtype=np.float32)

    with pytest.raises(PlanningError, match="could not evaluate batched"):
        adapter._candidate_singularity_scores(FailingJacobianSolver(), positions)


def test_candidate_singularity_svd_exception_fails_closed(
    repository_root,
    environment,
) -> None:
    class FailingLinalg:
        @staticmethod
        def svdvals(matrices):
            del matrices
            raise RuntimeError("synthetic SVD failure")

    class FailingSvdTorch(_CpuTorch):
        linalg = FailingLinalg()

    adapter = _adapter(repository_root, environment)
    adapter._runtime["torch"] = FailingSvdTorch()
    positions = np.zeros((2, len(environment.joint_names)), dtype=np.float32)

    with pytest.raises(PlanningError, match="singularity SVD failed"):
        adapter._candidate_singularity_scores(_BatchJacobianSolver(), positions)


def test_solve_ik_keeps_flattened_singularity_indices_through_sorting(
    repository_root,
    environment,
    monkeypatch,
) -> None:
    first = np.asarray(
        [
            [0.05, 0.04, 0.0, 0.0, 0.0, 0.20],
            [0.05, 0.01, 0.0, 0.0, 0.0, 0.20],
        ],
        dtype=np.float32,
    )
    second = np.asarray(
        [
            [0.05, 0.03, 0.0, 0.0, 0.0, 0.20],
            [0.05, 0.02, 0.0, 0.0, 0.0, 0.20],
        ],
        dtype=np.float32,
    )
    batch_sizes: list[int] = []

    def scored_kinematics(state):
        batch_sizes.append(int(state.position.shape[0]))
        score = 1.0 + 100.0 * state.position[:, 1]
        diagonal = torch.ones(
            (len(state.position), len(environment.joint_names)),
            dtype=torch.float32,
        )
        diagonal[:, -1] = score
        return SimpleNamespace(
            tool_jacobians=torch.diag_embed(diagonal).unsqueeze(1).unsqueeze(2)
        )

    monkeypatch.setattr(_FakeIK, "solutions", (first, second))
    monkeypatch.setattr(_FakeIK, "compute_kinematics", staticmethod(scored_kinematics))
    adapter = _adapter(repository_root, environment)

    candidates = adapter.solve_ik_candidates(_poses(), candidates_per_pose=2)

    assert batch_sizes == [4]
    assert [len(candidate_set) for candidate_set in candidates] == [2, 2]
    assert np.allclose(
        [[candidate.positions[1] for candidate in group] for group in candidates],
        [[0.01, 0.04], [0.02, 0.03]],
        atol=1.0e-7,
    )
    assert np.allclose(
        [[candidate.singularity_score for candidate in group] for group in candidates],
        [[2.0, 5.0], [3.0, 4.0]],
        rtol=1.0e-6,
    )
