from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np
import yaml

from .contact_policy import ContactPolicy, ContactSphereClassification
from .environment_builder import EnvironmentBundle
from .errors import ConfigurationError, CuroboUnavailableError, PlanningError
from .math3d import matrix_pose, pose_matrix, quaternion_angular_distance_deg
from .models import CompiledTrajectory, IKCandidate, TimedPose
from .robot_tool_builder import RobotToolBuilder
from .scene_config import SceneConfigurationError, resolved_curobo_scene


_COLLISION_QUERY_CHUNK_SIZE = 256
_SINGULARITY_SCORE_CHUNK_SIZE = 2048
_IK_SEED_DEDUPLICATION_TOLERANCE_RAD = 1.0e-5
_MAX_INTERPOLATION_BUFFER_SIZE = 1_000_000


@dataclass(frozen=True)
class JointPath:
    times_s: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class PathValidationMetrics:
    max_tip_position_error_m: float
    max_tip_orientation_error_deg: float
    minimum_collision_distance_m: float
    max_scene_collision_cost: float
    max_self_collision_cost: float
    max_joint_bound_cost: float
    checked_sample_count: int


class _DelegatingCost:
    """Delegate the cuRobo cost lifecycle while replacing only ``forward``."""

    def __init__(self, delegate: object) -> None:
        self._delegate = delegate

    def __getattr__(self, name: str):
        return getattr(self._delegate, name)


class _FixedTimeTeleportStep:
    """Preserve cuRobo's teleport positions while supplying their real dt."""

    def __init__(self, delegate: object, base_dt: float) -> None:
        self._delegate = delegate
        self._base_dt = float(base_dt)

    def __getattr__(self, name: str):
        return getattr(self._delegate, name)

    def forward(self, *args, **kwargs):
        state = self._delegate.forward(*args, **kwargs)
        dt = getattr(state, "dt", None)
        if dt is None:
            raise PlanningError("cuRoboV2 teleport transition did not expose trajectory dt")
        try:
            dt.fill_(self._base_dt)
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise PlanningError(
                "cuRoboV2 teleport transition rejected the authoritative trajectory dt"
            ) from error
        return state


class _TrajectoryTrackingToolPoseCost(_DelegatingCost):
    """Vectorized per-horizon tool-pose cost for one fixed Pose sequence."""

    def __init__(self, delegate: object, torch_module: object, tracking_goal: object) -> None:
        super().__init__(delegate)
        self._torch = torch_module
        self._tracking_goal = tracking_goal

    def forward(
        self,
        current_tool_poses,
        goal_tool_poses,
        idxs_goal=None,
        **kwargs,
    ):
        del goal_tool_poses, idxs_goal, kwargs
        torch = self._torch
        target_position = self._tracking_goal.position
        target_quaternion = self._tracking_goal.quaternion
        if (
            target_position.ndim != 5
            or target_quaternion.ndim != 5
            or target_position.shape[0] != 1
            or target_position.shape[3] != 1
            or target_position.shape[-1] != 3
            or target_quaternion.shape[-1] != 4
        ):
            raise PlanningError(
                "cuRobo trajectory tracking goal must have shape [1,H,L,1,3/4]"
            )
        if current_tool_poses.tool_frames != self._tracking_goal.tool_frames:
            raise PlanningError("cuRobo trajectory tracking tool-frame order changed")
        current_position = current_tool_poses.position
        current_quaternion = current_tool_poses.quaternion
        expected_shape = (
            current_position.shape[0],
            target_position.shape[1],
            target_position.shape[2],
            3,
        )
        if tuple(current_position.shape) != expected_shape:
            raise PlanningError(
                "cuRobo trajectory rollout changed the authoritative Pose constraint count"
            )
        if tuple(current_quaternion.shape) != expected_shape[:-1] + (4,):
            raise PlanningError("cuRobo trajectory rollout returned invalid quaternion shape")

        target_position = target_position[:, :, :, 0, :]
        target_quaternion = target_quaternion[:, :, :, 0, :]
        position_error = torch.linalg.vector_norm(
            current_position - target_position,
            dim=-1,
        )
        current_quaternion = current_quaternion / torch.linalg.vector_norm(
            current_quaternion,
            dim=-1,
            keepdim=True,
        ).clamp_min(1.0e-12)
        target_quaternion = target_quaternion / torch.linalg.vector_norm(
            target_quaternion,
            dim=-1,
            keepdim=True,
        ).clamp_min(1.0e-12)
        quaternion_dot = torch.sum(
            current_quaternion * target_quaternion,
            dim=-1,
        ).abs().clamp(max=1.0)
        rotation_error = 2.0 * torch.atan2(
            torch.sqrt(torch.clamp(1.0 - quaternion_dot.square(), min=0.0)),
            quaternion_dot.clamp_min(1.0e-12),
        )

        weights = self._delegate._weight.reshape(-1)
        if weights.numel() != 2:
            raise PlanningError("cuRobo tool-pose cost weight shape changed")
        position_cost = weights[0] * position_error.square()
        # 4 * (1 - dot^2) is locally equivalent to squared angular error and
        # remains smooth around the quaternion sign ambiguity.
        rotation_cost = 4.0 * weights[1] * (1.0 - quaternion_dot.square())
        cost = torch.cat((position_cost, rotation_cost), dim=-1)
        goalset_index = torch.zeros(
            position_error.shape,
            device=position_error.device,
            dtype=torch.int32,
        )
        return cost, position_error, rotation_error, goalset_index


class _TrajectoryTrackingCSpaceCost(_DelegatingCost):
    """Add exact-time branch and smoothness regularization to cuRobo bounds."""

    def __init__(
        self,
        delegate: object,
        torch_module: object,
        seed_positions,
        times_s,
        velocity_limits,
        acceleration_limits,
        jerk_limits,
        *,
        joint_distance_weight: float,
        velocity_weight: float,
        acceleration_weight: float,
        jerk_weight: float,
    ) -> None:
        super().__init__(delegate)
        self._torch = torch_module
        self._seed_positions = seed_positions
        self._times_s = times_s
        self._velocity_limits = velocity_limits
        self._acceleration_limits = acceleration_limits
        self._jerk_limits = jerk_limits
        self._joint_distance_weight = float(joint_distance_weight)
        self._velocity_weight = float(velocity_weight)
        self._acceleration_weight = float(acceleration_weight)
        self._jerk_weight = float(jerk_weight)

    def forward(self, state_batch, *args, **kwargs):
        base_cost = self._delegate.forward(state_batch, *args, **kwargs)
        torch = self._torch
        positions = state_batch.position
        if tuple(positions.shape[1:]) != tuple(self._seed_positions.shape[1:]):
            raise PlanningError(
                "cuRobo trajectory rollout changed the authoritative joint-path shape"
            )
        if tuple(base_cost.shape) != tuple(positions.shape):
            raise PlanningError("cuRobo c-space cost returned an incompatible shape")

        seed_delta = positions - self._seed_positions
        horizon = positions.shape[1]
        duration = (self._times_s[-1] - self._times_s[0]).clamp_min(1.0e-9)
        regularization = (
            self._joint_distance_weight * seed_delta.square() / float(horizon)
        )

        if positions.shape[1] >= 2:
            dt = (self._times_s[1:] - self._times_s[:-1]).reshape(1, -1, 1)
            delta = positions[:, 1:] - positions[:, :-1]
            velocity = delta / dt
            normalized_velocity = velocity / self._velocity_limits
            velocity_cost = torch.cat(
                (
                    torch.zeros_like(positions[:, :1]),
                    normalized_velocity.square() * dt / duration,
                ),
                dim=1,
            )
            regularization = regularization + self._velocity_weight * velocity_cost
        else:
            velocity = None

        if velocity is not None and velocity.shape[1] >= 2:
            velocity_dt = 0.5 * (dt[:, 1:] + dt[:, :-1])
            acceleration = (velocity[:, 1:] - velocity[:, :-1]) / velocity_dt
            normalized_acceleration = acceleration / self._acceleration_limits
            acceleration_cost = torch.cat(
                (
                    torch.zeros_like(positions[:, :2]),
                    normalized_acceleration.square() * velocity_dt / duration,
                ),
                dim=1,
            )
            regularization = regularization + self._acceleration_weight * acceleration_cost
        else:
            acceleration = None

        if acceleration is not None and acceleration.shape[1] >= 2:
            acceleration_dt = 0.5 * (velocity_dt[:, 1:] + velocity_dt[:, :-1])
            jerk = (acceleration[:, 1:] - acceleration[:, :-1]) / acceleration_dt
            normalized_jerk = jerk / self._jerk_limits
            jerk_cost = torch.cat(
                (
                    torch.zeros_like(positions[:, :3]),
                    normalized_jerk.square() * acceleration_dt / duration,
                ),
                dim=1,
            )
            regularization = regularization + self._jerk_weight * jerk_cost
        return base_cost + regularization


class CuroboAdapter(Protocol):
    @property
    def revision(self) -> str: ...

    @property
    def joint_names(self) -> tuple[str, ...]: ...

    def warmup(self) -> None: ...

    def solve_ik_candidates(
        self,
        tool_poses: Sequence[TimedPose],
        candidates_per_pose: int,
    ) -> tuple[tuple[IKCandidate, ...], ...]: ...

    def optimize_pose_path(
        self,
        tool_poses: Sequence[TimedPose],
        authoritative_times_s: Sequence[float],
        seed_positions: Sequence[Sequence[float]],
    ) -> JointPath: ...

    def plan_cspace(
        self,
        start_positions: Sequence[float],
        goal_positions: Sequence[float],
    ) -> JointPath: ...

    def validate_joint_trajectory(
        self,
        trajectory: CompiledTrajectory,
    ) -> PathValidationMetrics: ...

    def validate_compiled_trajectory(
        self,
        trajectory: CompiledTrajectory,
        constrained_tip_poses: Sequence[TimedPose],
        constrained_positions: Sequence[Sequence[float]],
    ) -> PathValidationMetrics: ...


class CuroboAdapterV2:
    """Thin, fail-closed adapter for the locked cuRoboV2 public API.

    This is the only production module allowed to import ``curobo`` or ``torch``.
    Business logic talks to the :class:`CuroboAdapter` protocol instead.
    """

    def __init__(
        self,
        environment: EnvironmentBundle,
        lock_path: str | Path,
    ) -> None:
        self.environment = environment
        self.lock_path = Path(lock_path).expanduser().resolve()
        self._lock = self._load_lock()
        self._runtime: dict[str, object] | None = None
        self._ik_solver = None
        self._motion_planner = None
        self._trajopt_solver = None
        self._collision_checker = None

    @property
    def revision(self) -> str:
        return str(self._lock["commit"])

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.environment.joint_names

    def warmup(self) -> None:
        runtime = self._ensure_runtime()
        self._ensure_motion_planner(runtime)
        if not bool(self._motion_planner.warmup(enable_graph=False, num_warmup_iterations=5)):
            raise PlanningError("cuRoboV2 MotionPlanner warmup returned failure")

    def solve_ik_candidates(
        self,
        tool_poses: Sequence[TimedPose],
        candidates_per_pose: int,
    ) -> tuple[tuple[IKCandidate, ...], ...]:
        if not tool_poses:
            raise PlanningError("cuRobo IK received no tool poses")
        if (
            isinstance(candidates_per_pose, bool)
            or not isinstance(candidates_per_pose, int)
            or candidates_per_pose <= 0
        ):
            raise PlanningError("cuRobo IK candidates_per_pose must be a positive integer")
        if self._planner_value("ik.use_previous_solution_seed") is not True:
            raise PlanningError(
                "planner ik.use_previous_solution_seed must be true for continuous IK"
            )
        runtime = self._ensure_runtime()
        torch = runtime["torch"]
        InverseKinematics = runtime["InverseKinematics"]
        InverseKinematicsCfg = runtime["InverseKinematicsCfg"]
        GoalToolPose = runtime["GoalToolPose"]
        Pose = runtime["Pose"]
        solver_num_seeds = max(candidates_per_pose, 32)
        random_seed_reserve = self._ik_random_seed_reserve(solver_num_seeds)
        previous_seed_budget = solver_num_seeds - random_seed_reserve
        ik_position_tolerance = float(
            self._planner_value("validation.tip_position_tolerance_m")
        )
        ik_orientation_tolerance = float(
            np.radians(
                self._planner_value("validation.tip_orientation_tolerance_deg")
            )
        )

        config = InverseKinematicsCfg.create(
            robot=self._robot_config(),
            scene_model=self._scene_config(),
            num_seeds=solver_num_seeds,
            # Each target is solved in sequence so the preceding feasible
            # frontier can be supplied through the locked V2 seed_config API.
            # A bounded frontier leaves capacity for cuRobo to generate fresh
            # random seeds instead of deterministically exhausting all seeds
            # with solutions inherited from the previous Pose.
            max_batch_size=1,
            self_collision_check=True,
            # Keep the candidate generator at the authoritative hard tolerance.
            # Tightening IK itself reduced branch coverage and regressed the
            # known circle_01 path.  The guard band is applied only to the soft
            # fixed-time refinement, while final FK validation remains hard.
            position_tolerance=ik_position_tolerance,
            orientation_tolerance=ik_orientation_tolerance,
            optimizer_collision_activation_distance=float(
                self._planner_value("validation.minimum_collision_distance_m")
            ),
            # The previous frontier deliberately changes size after clearance
            # filtering, deduplication, and random-seed reservation.  The
            # pinned V2 metrics rollout replays a fixed-shape CUDA graph and
            # can execute an illegal instruction when that seed shape changes
            # between sequential Pose solves.  IK is offline, so keep graph
            # replay disabled and favor correct dynamic frontiers over this
            # optimization.
            use_cuda_graph=False,
        )
        self._ik_solver = InverseKinematics(config)
        target_link = self._target_link()
        previous_frontier = None
        previous_reachable_positions: np.ndarray | None = None
        solution_sets: list[np.ndarray] = []
        for pose_index, pose in enumerate(tool_poses):
            position = torch.tensor(
                [pose.position_m],
                device="cuda",
                dtype=torch.float32,
            )
            quaternion = torch.tensor(
                [self._xyzw_to_wxyz(pose.quaternion_xyzw)],
                device="cuda",
                dtype=torch.float32,
            )
            goals = GoalToolPose.from_poses(
                {target_link: Pose(position=position, quaternion=quaternion)},
                ordered_tool_frames=[target_link],
                num_goalset=1,
            )
            result = self._ik_solver.solve_pose(
                goal_tool_poses=goals,
                seed_config=previous_frontier,
                return_seeds=candidates_per_pose,
            )
            if result is None or getattr(result, "js_solution", None) is None:
                raise PlanningError(
                    f"cuRoboV2 IK returned no joint solutions for pose index {pose_index}"
                )
            self._validate_solver_joint_names(
                result.js_solution,
                f"IK result at pose index {pose_index}",
            )
            success_result = getattr(result, "success", None)
            if success_result is None:
                raise PlanningError(
                    f"cuRoboV2 IK returned no success mask for pose index {pose_index}"
                )
            feasible_result = getattr(result, "feasible", None)
            if feasible_result is None:
                raise PlanningError(
                    f"cuRoboV2 IK returned no feasible mask for pose index {pose_index}"
                )
            solution_position = getattr(result.js_solution, "position", None)
            if solution_position is None:
                raise PlanningError(
                    f"cuRoboV2 IK returned no joint tensor for pose index {pose_index}"
                )
            solution = solution_position.detach().cpu().numpy()
            if solution.ndim == 0:
                raise PlanningError(
                    f"cuRoboV2 IK returned invalid joints for pose index {pose_index}"
                )
            solution = solution.reshape(-1, solution.shape[-1])
            success = success_result.detach().cpu().numpy().reshape(-1)
            feasible = feasible_result.detach().cpu().numpy().reshape(-1)
            if len(solution) != len(success) or len(solution) != len(feasible):
                raise PlanningError(
                    f"cuRoboV2 IK returned misaligned results for pose index {pose_index}"
                )
            accepted = np.asarray(
                [
                    row
                    for row, did_succeed, is_feasible in zip(
                        solution,
                        success,
                        feasible,
                        strict=True,
                    )
                    if bool(did_succeed) and bool(is_feasible)
                ],
                dtype=np.float32,
            )
            if accepted.size == 0:
                accepted = np.empty((0, len(self.joint_names)), dtype=np.float32)
            if accepted.ndim != 2 or accepted.shape[1] != len(self.joint_names):
                raise PlanningError(
                    f"cuRoboV2 IK returned invalid joint width for pose index {pose_index}"
                )
            if not np.all(np.isfinite(accepted)):
                raise PlanningError(
                    f"cuRoboV2 IK returned non-finite joints for pose index {pose_index}"
                )
            raw_accepted_count = len(accepted)
            clearance_mask = self._ik_clearance_threshold_mask(runtime, accepted)
            accepted = accepted[clearance_mask]
            if len(accepted) == 0:
                minimum_clearance = self._minimum_collision_distance()
                raise PlanningError(
                    "cuRoboV2 IK has no candidate meeting measured scene/self clearance "
                    f"at pose index {pose_index}: raw accepted count={raw_accepted_count}, "
                    f"required clearance={minimum_clearance:.6g} m"
                )
            branch_mask = self._canonical_ik_branch_mask(accepted)
            accepted = accepted[branch_mask]
            if len(accepted) == 0:
                raise PlanningError(
                    "cuRoboV2 IK has no candidate in the configured canonical "
                    f"joint branch at pose index {pose_index}"
                )
            if previous_reachable_positions is not None:
                dt_s = float(pose.time_s - tool_poses[pose_index - 1].time_s)
                accepted, minimum_dynamic_ratio = (
                    self._temporally_reachable_ik_candidates(
                        accepted,
                        previous_reachable_positions,
                        dt_s,
                    )
                )
                if len(accepted) == 0:
                    raise PlanningError(
                        "cuRoboV2 IK has no clearance-valid candidate reachable from the "
                        f"continuous frontier at pose index {pose_index}, "
                        f"time_s={pose.time_s:.9f}: minimum hard-limit ratio="
                        f"{minimum_dynamic_ratio:.6g}"
                    )
            solution_sets.append(accepted)
            previous_reachable_positions = accepted
            if pose_index + 1 < len(tool_poses):
                previous_seeds = self._diversify_previous_ik_seeds(
                    accepted,
                    previous_seed_budget,
                )
                previous_frontier = torch.as_tensor(
                    previous_seeds,
                    device="cuda",
                    dtype=torch.float32,
                ).reshape(1, len(previous_seeds), len(self.joint_names))

        lower, upper = self._joint_position_limits(self._ik_solver)
        flattened = (
            np.concatenate(solution_sets, axis=0)
            if any(len(item) for item in solution_sets)
            else np.empty((0, len(self.joint_names)), dtype=np.float32)
        )
        clearances = self._candidate_collision_clearances(runtime, flattened)
        singularity_scores = self._candidate_singularity_scores(
            self._ik_solver,
            flattened,
        )

        candidate_sets: list[tuple[IKCandidate, ...]] = []
        candidate_index = 0
        for solution in solution_sets:
            candidates: list[IKCandidate] = []
            for q in solution:
                margin = float(np.min(np.minimum(q - lower, upper - q)))
                candidates.append(
                    IKCandidate(
                        positions=tuple(float(value) for value in q),
                        collision_distance_m=float(clearances[candidate_index]),
                        joint_limit_margin_rad=margin,
                        singularity_score=float(singularity_scores[candidate_index]),
                    )
                )
                candidate_index += 1
            # The selector still evaluates the full dynamic path.  Clearance
            # ordering makes the static candidate ranking truthful and
            # deterministic when dynamic costs tie.
            candidates.sort(
                key=lambda candidate: (
                    -candidate.collision_distance_m,
                    -candidate.joint_limit_margin_rad,
                    candidate.singularity_score,
                    candidate.positions,
                )
            )
            candidate_sets.append(tuple(candidates))
        return tuple(candidate_sets)

    def _canonical_ik_branch_mask(self, positions: np.ndarray) -> np.ndarray:
        values = np.asarray(positions, dtype=float)
        if values.ndim != 2 or values.shape[1] != len(self.joint_names):
            raise PlanningError("IK branch filtering received an invalid joint matrix")
        try:
            return self.environment.canonical_joint_branch_mask(values)
        except (ConfigurationError, KeyError, TypeError, ValueError) as error:
            raise PlanningError("configured canonical IK joint ranges are invalid") from error

    def _temporally_reachable_ik_candidates(
        self,
        current: np.ndarray,
        previous: np.ndarray,
        dt_s: float,
    ) -> tuple[np.ndarray, float]:
        """Keep only IK endpoints that extend a recursively reachable path."""

        expected_width = len(self.joint_names)
        if (
            current.ndim != 2
            or previous.ndim != 2
            or current.shape[1] != expected_width
            or previous.shape[1] != expected_width
            or len(current) == 0
            or len(previous) == 0
            or not np.all(np.isfinite(current))
            or not np.all(np.isfinite(previous))
            or not np.isfinite(dt_s)
            or dt_s <= 0.0
        ):
            raise PlanningError("continuous IK reachability input is invalid")
        velocity_limits = np.asarray(
            self.environment.robot_model["robot"]["limits"]["velocity_rad_s"],
            dtype=float,
        )
        branch_jump = self._planner_value("ik.reject_branch_jump_rad")
        if (
            velocity_limits.shape != (expected_width,)
            or np.any(~np.isfinite(velocity_limits))
            or np.any(velocity_limits <= 0.0)
            or isinstance(branch_jump, bool)
            or not isinstance(branch_jump, (int, float))
            or not np.isfinite(float(branch_jump))
            or float(branch_jump) <= 0.0
        ):
            raise PlanningError("continuous IK hard limits are invalid")
        allowed_delta = np.minimum(
            velocity_limits * dt_s,
            float(branch_jump),
        )
        delta = np.abs(current[:, None, :] - previous[None, :, :])
        edge_ratios = np.max(
            delta / allowed_delta.reshape(1, 1, -1),
            axis=-1,
        )
        reachable = np.any(edge_ratios <= 1.0, axis=1)
        return current[reachable], float(np.min(edge_ratios))

    def _candidate_collision_clearances(
        self,
        runtime: dict[str, object],
        positions: np.ndarray,
    ) -> np.ndarray:
        """Measure conservative scene/self clearance for IK candidates.

        The locked collision API exposes collision constraints, not a direct
        minimum-clearance scalar.  Inflating every scene-facing sphere by a
        candidate-specific probe distance turns that constraint into a
        clearance predicate.  Self-collision spheres are inflated by half the
        probe because both spheres in each pair grow.  Parallel bisection then
        returns a metric lower bound, capped at the checker's 1 m query range.
        """

        if len(positions) == 0:
            return np.empty((0,), dtype=float)
        expected_shape = (len(positions), len(self.joint_names))
        if positions.shape != expected_shape or not np.all(np.isfinite(positions)):
            raise PlanningError("cuRobo IK clearance input has an invalid shape")

        chunks: list[np.ndarray] = []
        for start in range(0, len(positions), _COLLISION_QUERY_CHUNK_SIZE):
            end = min(start + _COLLISION_QUERY_CHUNK_SIZE, len(positions))
            chunks.append(
                self._candidate_collision_clearances_chunk(
                    runtime,
                    positions[start:end],
                )
            )
        clearances = np.concatenate(chunks)
        if clearances.shape != (len(positions),) or not np.all(
            np.isfinite(clearances)
        ):
            raise PlanningError(
                "cuRoboV2 collision checker returned invalid candidate clearances"
            )
        return clearances

    def _minimum_collision_distance(self) -> float:
        minimum_clearance = self._planner_value(
            "validation.minimum_collision_distance_m"
        )
        if (
            isinstance(minimum_clearance, bool)
            or not isinstance(minimum_clearance, (int, float))
            or not np.isfinite(float(minimum_clearance))
            or float(minimum_clearance) <= 0.0
        ):
            raise PlanningError(
                "validation.minimum_collision_distance_m must be finite and positive"
            )
        return float(minimum_clearance)

    def _ik_clearance_threshold_mask(
        self,
        runtime: dict[str, object],
        positions: np.ndarray,
    ) -> np.ndarray:
        """Test the configured scene/self margin with one collision query."""

        if len(positions) == 0:
            return np.empty((0,), dtype=bool)
        expected_shape = (len(positions), len(self.joint_names))
        if positions.shape != expected_shape or not np.all(np.isfinite(positions)):
            raise PlanningError("cuRobo IK clearance input has an invalid shape")

        torch = runtime["torch"]
        checker, robot_spheres = self._candidate_robot_spheres(runtime, positions)
        probe = torch.full(
            (1, len(positions)),
            self._minimum_collision_distance(),
            device="cuda",
            dtype=torch.float32,
        )
        colliding = self._candidate_collision_at_probe(
            runtime,
            checker,
            robot_spheres,
            probe,
            len(positions),
        )
        mask = torch.logical_not(colliding).detach().cpu().numpy().reshape(-1)
        if mask.shape != (len(positions),):
            raise PlanningError(
                "cuRoboV2 collision checker returned an invalid clearance mask"
            )
        return mask.astype(bool)

    def _candidate_robot_spheres(
        self,
        runtime: dict[str, object],
        positions: np.ndarray,
    ):
        torch = runtime["torch"]
        checker = self._ensure_collision_checker(runtime)
        joint_positions = torch.as_tensor(
            positions,
            device="cuda",
            dtype=torch.float32,
        ).unsqueeze(0)
        try:
            state = checker.get_kinematics(joint_positions)
            robot_spheres = state.robot_spheres
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise PlanningError(
                "cuRoboV2 collision checker could not evaluate IK candidate kinematics"
            ) from error
        if (
            getattr(robot_spheres, "ndim", None) != 4
            or robot_spheres.shape[0] != 1
            or robot_spheres.shape[1] != len(positions)
            or robot_spheres.shape[-1] != 4
            or not bool(torch.all(torch.isfinite(robot_spheres)).item())
        ):
            raise PlanningError(
                "cuRoboV2 collision checker returned invalid candidate sphere geometry"
            )
        return checker, robot_spheres

    def _candidate_collision_at_probe(
        self,
        runtime: dict[str, object],
        checker,
        robot_spheres,
        probe,
        candidate_count: int,
    ):
        torch = runtime["torch"]
        if (
            tuple(probe.shape) != (1, candidate_count)
            or not bool(torch.all(torch.isfinite(probe)).item())
            or bool(torch.any(probe < 0.0).item())
        ):
            raise PlanningError("cuRoboV2 IK clearance probe is invalid")
        active_radius = robot_spheres[..., 3] >= 0.0
        scene_spheres = robot_spheres.clone()
        scene_spheres[..., 3] = torch.where(
            active_radius,
            robot_spheres[..., 3] + probe.unsqueeze(-1),
            robot_spheres[..., 3],
        )
        self_spheres = robot_spheres.clone()
        self_spheres[..., 3] = torch.where(
            active_radius,
            robot_spheres[..., 3] + 0.5 * probe.unsqueeze(-1),
            robot_spheres[..., 3],
        )
        try:
            scene_cost = checker.get_collision_constraint(scene_spheres)
            self_cost = checker.get_self_collision_distance(self_spheres)
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise PlanningError(
                "cuRoboV2 collision checker cannot measure IK candidate clearance"
            ) from error

        def reduce_cost(cost, label: str):
            if (
                getattr(cost, "ndim", None) is None
                or cost.ndim < 2
                or cost.shape[0] != 1
                or cost.shape[1] != candidate_count
            ):
                raise PlanningError(
                    f"cuRoboV2 {label} collision query returned an invalid shape"
                )
            reduced = cost.reshape(1, candidate_count, -1)
            reduced = torch.max(reduced, dim=-1).values
            if not bool(torch.all(torch.isfinite(reduced)).item()):
                raise PlanningError(
                    f"cuRoboV2 {label} collision query returned non-finite cost"
                )
            if bool(torch.any(reduced < -1.0e-8).item()):
                raise PlanningError(
                    f"cuRoboV2 {label} collision query returned negative cost"
                )
            return reduced

        scene = reduce_cost(scene_cost, "scene")
        self_collision = reduce_cost(self_cost, "self")
        return torch.logical_or(scene > 0.0, self_collision > 0.0)

    def _candidate_collision_clearances_chunk(
        self,
        runtime: dict[str, object],
        positions: np.ndarray,
    ) -> np.ndarray:
        """Measure one bounded GPU batch of conservative clearances."""

        torch = runtime["torch"]
        checker, robot_spheres = self._candidate_robot_spheres(runtime, positions)
        minimum_clearance = self._minimum_collision_distance()
        clearance_cap = max(1.0, minimum_clearance * 4.0)

        def collision_at(probe):
            return self._candidate_collision_at_probe(
                runtime,
                checker,
                robot_spheres,
                probe,
                len(positions),
            )

        initial_probe = torch.full(
            (1, len(positions)),
            minimum_clearance,
            device="cuda",
            dtype=torch.float32,
        )
        below_minimum = collision_at(initial_probe)
        lower = torch.where(
            below_minimum,
            torch.zeros_like(initial_probe),
            initial_probe,
        )
        upper = torch.where(
            below_minimum,
            initial_probe,
            torch.full_like(initial_probe, clearance_cap),
        )
        # 18 iterations resolve the 1 m capped interval to < 4 micrometres.
        for _ in range(18):
            probe = 0.5 * (lower + upper)
            colliding = collision_at(probe)
            lower = torch.where(colliding, lower, probe)
            upper = torch.where(colliding, probe, upper)
        return lower.detach().cpu().numpy().reshape(-1).astype(float)

    def optimize_pose_path(
        self,
        tool_poses: Sequence[TimedPose],
        authoritative_times_s: Sequence[float],
        seed_positions: Sequence[Sequence[float]],
    ) -> JointPath:
        if len(tool_poses) != len(seed_positions) or len(tool_poses) < 2:
            raise PlanningError("cuRobo trajectory optimization requires aligned pose and seed paths")
        times = np.asarray(authoritative_times_s, dtype=float)
        if len(times) != len(tool_poses) or np.any(np.diff(times) <= 0.0):
            raise PlanningError("authoritative trajectory time is invalid")
        runtime = self._ensure_runtime()
        torch = runtime["torch"]
        TrajectoryOptimizer = runtime["TrajectoryOptimizer"]
        TrajectoryOptimizerCfg = runtime["TrajectoryOptimizerCfg"]
        GoalToolPose = runtime["GoalToolPose"]
        JointState = runtime["JointState"]

        selected = np.asarray(seed_positions, dtype=float)
        expected_shape = (len(tool_poses), len(self.joint_names))
        if selected.shape != expected_shape or not np.all(np.isfinite(selected)):
            raise PlanningError("selected cuRobo IK seed path has an invalid shape")

        # The locked V2 GoalToolPose type advertises a horizon dimension, but its
        # built-in ToolPoseCost squeezes only horizon=1 before calling a kernel
        # that accepts a 4D static goal.  Keep the public terminal goal at H=1
        # and install a vectorized torch-autograd cost on the live cuRobo
        # rollouts.  This makes all authoritative poses participate in the same
        # joint-trajectory optimization without falling back to per-frame IK.
        target_link = self._target_link()
        goal_positions = torch.as_tensor(
            np.asarray([pose.position_m for pose in tool_poses], dtype=np.float32),
            device="cuda",
            dtype=torch.float32,
        ).reshape(1, len(tool_poses), 1, 1, 3)
        goal_quaternions = torch.as_tensor(
            np.asarray(
                [self._xyzw_to_wxyz(pose.quaternion_xyzw) for pose in tool_poses],
                dtype=np.float32,
            ),
            device="cuda",
            dtype=torch.float32,
        ).reshape(1, len(tool_poses), 1, 1, 4)
        tracking_goal = GoalToolPose(
            tool_frames=[target_link],
            position=goal_positions,
            quaternion=goal_quaternions,
        )
        terminal_goal = GoalToolPose(
            tool_frames=[target_link],
            position=goal_positions[:, -1:, :, :, :].contiguous(),
            quaternion=goal_quaternions[:, -1:, :, :, :].contiguous(),
        )

        base_dt = float((times[-1] - times[0]) / (len(times) - 1))
        internal_position_tolerance, internal_orientation_tolerance = (
            self._internal_pose_tolerances()
        )
        config = TrajectoryOptimizerCfg.create(
            robot=self._robot_config(),
            scene_model=self._scene_config(),
            num_seeds=1,
            max_batch_size=1,
            self_collision_check=True,
            position_tolerance=internal_position_tolerance,
            orientation_tolerance=internal_orientation_tolerance,
            optimizer_collision_activation_distance=float(
                self._planner_value("validation.minimum_collision_distance_m")
            ),
            use_cuda_graph=False,
            transition_model=self._transition_model(len(tool_poses), base_dt),
            minimum_trajectory_dt=base_dt,
            maximum_trajectory_dt=max(base_dt, 0.2),
        )
        # Keep cuRobo's dense interpolated collision/bound metrics.  The
        # controlled interval is never coarser than V2's configured default
        # and always includes an inter-knot midpoint.  Capacity follows the
        # locked calculate_traj_steps formula using the real fixed-time dt.
        config.interpolation_dt = self._fixed_time_interpolation_dt(
            base_dt,
            config.interpolation_dt,
        )
        config.interpolation_buffer_size = self._fixed_time_interpolation_buffer_size(
            len(tool_poses),
            base_dt,
            config.interpolation_dt,
        )
        self._trajopt_solver = TrajectoryOptimizer(config)
        self._install_fixed_time_teleport_dt(self._trajopt_solver, base_dt)
        seed = torch.as_tensor(
            selected.reshape(1, 1, len(tool_poses), len(self.joint_names)),
            device="cuda",
            dtype=torch.float32,
        )
        times_tensor = torch.as_tensor(times, device="cuda", dtype=torch.float32)
        self._install_trajectory_tracking_costs(
            self._trajopt_solver,
            runtime,
            tracking_goal,
            seed[:, 0],
            times_tensor,
        )
        current_state = JointState.from_position(
            seed[:, 0, 0, :],
            joint_names=list(self.joint_names),
        )
        # In cuRobo V2, current_state.dt drives PositionCSpaceCost's
        # single-step window against the same start state for every horizon
        # knot.  The trajectory dt is supplied separately to solve_pose; keep
        # current_state.dt unset so a valid multi-knot path is not collapsed
        # into one velocity-limited step from its start.
        result = self._trajopt_solver.solve_pose(
            goal_tool_poses=terminal_goal,
            current_state=current_state,
            seed_traj=seed,
            return_seeds=1,
            num_seeds=1,
            dt=base_dt,
            finetune_attempts=0,
        )
        if result is None or getattr(result, "success", None) is None:
            raise PlanningError("cuRoboV2 joint trajectory optimization returned no result")
        if not bool(result.success.reshape(-1)[0].item()):
            raise PlanningError("cuRoboV2 joint trajectory optimization failed")
        if getattr(result, "js_solution", None) is None:
            raise PlanningError("cuRoboV2 joint trajectory optimization returned no joint path")
        self._validate_fixed_time_solution_dt(result.js_solution, base_dt)
        self._validate_solver_joint_names(
            result.js_solution,
            "joint trajectory optimization result",
        )
        active = self._trajopt_solver.get_active_js(result.js_solution)
        self._validate_fixed_time_solution_dt(active, base_dt)
        self._validate_solver_joint_names(
            active,
            "active joint trajectory optimization result",
        )
        optimized = active.position.detach().cpu().numpy().reshape(
            -1, len(self.joint_names)
        )
        if optimized.shape != expected_shape or not np.all(np.isfinite(optimized)):
            raise PlanningError(
                "cuRoboV2 optimization changed the authoritative Pose constraint count"
            )

        jump_limit = float(self._planner_value("ik.reject_branch_jump_rad"))
        deltas = np.diff(optimized, axis=0)
        if deltas.size and float(np.max(np.abs(deltas))) > jump_limit:
            index = int(np.unravel_index(np.argmax(np.abs(deltas)), deltas.shape)[0] + 1)
            raise PlanningError(
                f"cuRoboV2 optimized path exceeds branch-jump limit at pose index {index}"
            )
        self._validate_optimized_tool_path(
            self._trajopt_solver,
            runtime,
            tool_poses,
            optimized,
        )

        return JointPath(
            times_s=tuple(float(value) for value in times),
            positions=tuple(
                tuple(float(value) for value in row) for row in optimized
            ),
        )

    def _install_trajectory_tracking_costs(
        self,
        solver,
        runtime: dict[str, object],
        tracking_goal,
        seed_positions,
        times_s,
    ) -> None:
        torch = runtime["torch"]
        dynamic_limits = self.environment.robot_model["robot"]["limits"]
        velocity_limits = torch.as_tensor(
            dynamic_limits["velocity_rad_s"],
            device="cuda",
            dtype=torch.float32,
        ).reshape(1, 1, -1)
        acceleration_limits = torch.as_tensor(
            dynamic_limits["acceleration_rad_s2"],
            device="cuda",
            dtype=torch.float32,
        ).reshape(1, 1, -1)
        jerk_limits = torch.as_tensor(
            dynamic_limits["jerk_rad_s3"],
            device="cuda",
            dtype=torch.float32,
        ).reshape(1, 1, -1)
        optimizer_rollouts = tuple(getattr(solver, "optimizer_rollouts", ()))
        if not optimizer_rollouts:
            raise PlanningError("cuRoboV2 trajectory optimizer exposed no optimizer rollout")
        all_rollouts = (
            getattr(solver, "metrics_rollout", None),
            getattr(solver, "auxiliary_rollout", None),
            *optimizer_rollouts,
        )
        tracking_optimizer_rollouts: set[int] = set()
        smooth_optimizer_rollouts: set[int] = set()
        seen_managers: set[int] = set()
        manager_names = (
            "cost_manager",
            "constraint_manager",
            "hybrid_cost_constraint_manager",
            "metrics_cost_manager",
            "metrics_constraint_manager",
            "metrics_hybrid_cost_constraint_manager",
            "metrics_convergence_manager",
        )
        for rollout in all_rollouts:
            if rollout is None:
                continue
            is_optimizer_rollout = any(rollout is item for item in optimizer_rollouts)
            for manager_name in manager_names:
                manager = getattr(rollout, manager_name, None)
                if manager is None or id(manager) in seen_managers:
                    continue
                seen_managers.add(id(manager))
                if manager.has_cost("tool_pose"):
                    manager.costs["tool_pose"] = _TrajectoryTrackingToolPoseCost(
                        manager.get_cost("tool_pose"),
                        torch,
                        tracking_goal,
                    )
                    if is_optimizer_rollout and manager_name == "cost_manager":
                        tracking_optimizer_rollouts.add(id(rollout))
                if (
                    is_optimizer_rollout
                    and manager_name == "cost_manager"
                    and manager.has_cost("cspace")
                ):
                    manager.costs["cspace"] = _TrajectoryTrackingCSpaceCost(
                        manager.get_cost("cspace"),
                        torch,
                        seed_positions,
                        times_s,
                        velocity_limits,
                        acceleration_limits,
                        jerk_limits,
                        joint_distance_weight=float(
                            self._planner_value("optimization.joint_distance_weight")
                        ),
                        velocity_weight=float(
                            self._planner_value("optimization.velocity_weight")
                        ),
                        acceleration_weight=float(
                            self._planner_value("optimization.acceleration_weight")
                        ),
                        jerk_weight=float(
                            self._planner_value("optimization.jerk_weight")
                        ),
                    )
                    smooth_optimizer_rollouts.add(id(rollout))
        expected_rollouts = {id(rollout) for rollout in optimizer_rollouts}
        if tracking_optimizer_rollouts != expected_rollouts:
            raise PlanningError(
                "cuRoboV2 optimizer rollout has no replaceable tool-pose cost"
            )
        if smooth_optimizer_rollouts != expected_rollouts:
            raise PlanningError("cuRoboV2 optimizer rollout has no c-space regularization cost")

    def _validate_optimized_tool_path(
        self,
        solver,
        runtime: dict[str, object],
        tool_poses: Sequence[TimedPose],
        positions: np.ndarray,
    ) -> None:
        torch = runtime["torch"]
        JointState = runtime["JointState"]
        state = JointState.from_position(
            torch.as_tensor(positions, device="cuda", dtype=torch.float32),
            joint_names=list(self.joint_names),
        )
        kinematics = solver.compute_kinematics(state)
        actual_pose = kinematics.tool_poses.get_link_pose(self._target_link())
        actual_positions = actual_pose.position.detach().cpu().numpy().reshape(-1, 3)
        actual_quaternions = actual_pose.quaternion.detach().cpu().numpy().reshape(-1, 4)
        if (
            len(actual_positions) != len(tool_poses)
            or len(actual_quaternions) != len(tool_poses)
        ):
            raise PlanningError(
                "cuRoboV2 FK validation changed the authoritative Pose constraint count"
            )
        position_tolerance = float(
            self._planner_value("validation.tip_position_tolerance_m")
        )
        orientation_tolerance = float(
            self._planner_value("validation.tip_orientation_tolerance_deg")
        )
        for index, (target, actual_position, actual_wxyz) in enumerate(
            zip(tool_poses, actual_positions, actual_quaternions, strict=True)
        ):
            position_error = float(
                np.linalg.norm(
                    np.asarray(target.position_m, dtype=float)
                    - np.asarray(actual_position, dtype=float)
                )
            )
            actual_xyzw = (
                float(actual_wxyz[1]),
                float(actual_wxyz[2]),
                float(actual_wxyz[3]),
                float(actual_wxyz[0]),
            )
            orientation_error = quaternion_angular_distance_deg(
                target.quaternion_xyzw,
                actual_xyzw,
            )
            if position_error > position_tolerance:
                raise PlanningError(
                    "cuRoboV2 optimized Pose position error exceeds tolerance at "
                    f"index {index}: {position_error:.6g} > {position_tolerance:.6g} m"
                )
            if orientation_error > orientation_tolerance:
                raise PlanningError(
                    "cuRoboV2 optimized Pose orientation error exceeds tolerance at "
                    f"index {index}: {orientation_error:.6g} > "
                    f"{orientation_tolerance:.6g} deg"
                )

    def plan_cspace(
        self,
        start_positions: Sequence[float],
        goal_positions: Sequence[float],
    ) -> JointPath:
        runtime = self._ensure_runtime()
        torch = runtime["torch"]
        JointState = runtime["JointState"]
        self._ensure_motion_planner(runtime)
        start = JointState.from_position(
            torch.tensor([start_positions], device="cuda", dtype=torch.float32),
            joint_names=list(self.joint_names),
        )
        goal = JointState.from_position(
            torch.tensor([goal_positions], device="cuda", dtype=torch.float32),
            joint_names=list(self.joint_names),
        )
        result = self._motion_planner.plan_cspace(goal_state=goal, current_state=start)
        if result is None or not bool(result.success.any().item()):
            raise PlanningError("cuRoboV2 c-space planning failed")
        plan = result.get_interpolated_plan()
        self._validate_solver_joint_names(plan, "c-space planning result")
        positions = plan.position.detach().cpu().numpy().reshape(-1, len(self.joint_names))
        dt_value = float(np.asarray(plan.dt.detach().cpu()).reshape(-1)[0])
        times = tuple(float(index * dt_value) for index in range(len(positions)))
        return JointPath(
            times_s=times,
            positions=tuple(tuple(float(value) for value in row) for row in positions),
        )

    def validate_joint_trajectory(
        self,
        trajectory: CompiledTrajectory,
    ) -> PathValidationMetrics:
        if trajectory.joint_names != self.joint_names:
            raise PlanningError("joint trajectory order does not match the cuRobo robot model")
        try:
            positions = np.asarray(
                [sample.positions for sample in trajectory.samples],
                dtype=np.float32,
            )
            velocities = np.asarray(
                [sample.velocities for sample in trajectory.samples],
                dtype=np.float32,
            )
            accelerations = np.asarray(
                [sample.accelerations for sample in trajectory.samples],
                dtype=np.float32,
            )
            times = np.asarray(
                [sample.time_s for sample in trajectory.samples],
                dtype=float,
            )
        except (TypeError, ValueError) as error:
            raise PlanningError("joint trajectory contains an invalid numeric value") from error
        expected_shape = (len(trajectory.samples), len(self.joint_names))
        if (
            positions.shape != expected_shape
            or velocities.shape != expected_shape
            or accelerations.shape != expected_shape
        ):
            raise PlanningError("joint trajectory has an invalid joint tensor shape")
        if (
            times.shape != (len(trajectory.samples),)
            or not np.all(np.isfinite(times))
            or not np.all(np.isfinite(positions))
            or not np.all(np.isfinite(velocities))
            or not np.all(np.isfinite(accelerations))
        ):
            raise PlanningError("joint trajectory contains a non-finite numeric value")

        runtime = self._ensure_runtime()
        torch = runtime["torch"]
        checker = self._ensure_collision_checker(runtime)

        def finite_query_max(cost, label: str, sample_count: int) -> float:
            if (
                getattr(cost, "ndim", None) is None
                or cost.ndim < 2
                or cost.shape[0] != 1
                or cost.shape[1] != sample_count
                or cost.numel() == 0
            ):
                raise PlanningError(
                    f"cuRoboV2 {label} query returned an invalid shape"
                )
            if not bool(torch.all(torch.isfinite(cost)).item()):
                raise PlanningError(
                    f"cuRoboV2 {label} query returned a non-finite cost"
                )
            result = float(torch.max(cost).item())
            if not np.isfinite(result):
                raise PlanningError(
                    f"cuRoboV2 {label} query returned a non-finite maximum"
                )
            return result

        max_scene_cost = 0.0
        max_self_cost = 0.0
        max_bound_cost = 0.0
        minimum_measured_clearance = float("inf")
        all_samples_meet_clearance = True
        for start in range(0, len(positions), _COLLISION_QUERY_CHUNK_SIZE):
            end = min(start + _COLLISION_QUERY_CHUNK_SIZE, len(positions))
            chunk = torch.as_tensor(
                positions[start:end],
                device="cuda",
            ).unsqueeze(0)
            try:
                state = checker.get_kinematics(chunk)
                scene_cost = checker.get_collision_distance(state)
                self_cost = checker.get_self_collision_distance(state.robot_spheres)
                bound_cost = checker.get_bound(chunk)
            except (AttributeError, RuntimeError, TypeError, ValueError) as error:
                raise PlanningError(
                    "cuRoboV2 final trajectory collision/bound query failed"
                ) from error
            sample_count = end - start
            max_scene_cost = max(
                max_scene_cost,
                finite_query_max(scene_cost, "scene collision", sample_count),
            )
            max_self_cost = max(
                max_self_cost,
                finite_query_max(self_cost, "self collision", sample_count),
            )
            max_bound_cost = max(
                max_bound_cost,
                finite_query_max(bound_cost, "joint bound", sample_count),
            )
            clearance_mask = self._ik_clearance_threshold_mask(
                runtime,
                positions[start:end],
            )
            all_samples_meet_clearance = all_samples_meet_clearance and bool(
                np.all(clearance_mask)
            )
            chunk_clearances = self._candidate_collision_clearances(
                runtime,
                positions[start:end],
            )
            minimum_measured_clearance = min(
                minimum_measured_clearance,
                float(np.min(chunk_clearances)),
            )

        required_clearance = float(
            self._planner_value("validation.minimum_collision_distance_m")
        )
        if (
            not np.isfinite(required_clearance)
            or required_clearance <= 0.0
            or not np.isfinite(minimum_measured_clearance)
        ):
            raise PlanningError("collision-clearance validation produced an invalid value")

        cost_tolerance = 1.0e-7
        if max_scene_cost > cost_tolerance:
            raise PlanningError(
                "final sampled trajectory violates the configured scene-clearance margin: "
                f"max cost={max_scene_cost:.6g}"
            )
        if max_self_cost > cost_tolerance:
            raise PlanningError(
                "final sampled trajectory contains a self-collision cost: "
                f"{max_self_cost:.6g}"
            )
        if max_bound_cost > cost_tolerance:
            raise PlanningError(
                "final sampled trajectory violates a joint position bound: "
                f"{max_bound_cost:.6g}"
            )
        if not all_samples_meet_clearance:
            raise PlanningError(
                "final sampled trajectory violates the conservative scene/self "
                "clearance margin: "
                f"{minimum_measured_clearance:.6g} < {required_clearance:.6g} m"
            )
        # The pass/fail decision above uses the exact same float32 collision
        # predicate as IK admission.  The bounded binary-search value is only
        # a reporting lower bound and can land a few micrometres below that
        # accepted probe when its 1 m search interval is unresolved.  Once the
        # threshold predicate passes, the configured threshold itself is the
        # tightest proven lower bound available for reporting.
        minimum_reported_clearance = max(
            minimum_measured_clearance,
            required_clearance,
        )
        return PathValidationMetrics(
            max_tip_position_error_m=0.0,
            max_tip_orientation_error_deg=0.0,
            minimum_collision_distance_m=minimum_reported_clearance,
            max_scene_collision_cost=max_scene_cost,
            max_self_collision_cost=max_self_cost,
            max_joint_bound_cost=max_bound_cost,
            checked_sample_count=len(positions),
        )

    def validate_compiled_trajectory(
        self,
        trajectory: CompiledTrajectory,
        constrained_tip_poses: Sequence[TimedPose],
        constrained_positions: Sequence[Sequence[float]],
    ) -> PathValidationMetrics:
        if len(constrained_tip_poses) != len(constrained_positions) or not constrained_tip_poses:
            raise PlanningError("compiled trajectory validation received misaligned pose constraints")
        contact_policy = ContactPolicy.from_environment(self.environment)
        runtime = self._ensure_runtime()
        torch = runtime["torch"]
        JointState = runtime["JointState"]
        checker = self._ensure_collision_checker(runtime)

        constraint_q = torch.as_tensor(
            np.asarray(constrained_positions, dtype=np.float32),
            device="cuda",
        )
        constraint_state = checker.kinematics.compute_kinematics(
            JointState.from_position(constraint_q, joint_names=list(self.joint_names))
        )
        actual_tool_pose = constraint_state.tool_poses.get_link_pose(self._target_link())
        actual_positions = actual_tool_pose.position.detach().cpu().numpy().reshape(-1, 3)
        actual_quaternions_wxyz = (
            actual_tool_pose.quaternion.detach().cpu().numpy().reshape(-1, 4)
        )
        if len(actual_positions) != len(constrained_tip_poses):
            raise PlanningError("cuRobo FK validation returned an unexpected constraint count")

        tool_to_tip = RobotToolBuilder(self.environment).tool0_to_tip
        position_errors: list[float] = []
        orientation_errors: list[float] = []
        actual_tip_positions: list[tuple[float, ...]] = []
        for target, actual_position, actual_wxyz in zip(
            constrained_tip_poses,
            actual_positions,
            actual_quaternions_wxyz,
            strict=True,
        ):
            actual_xyzw = (
                float(actual_wxyz[1]),
                float(actual_wxyz[2]),
                float(actual_wxyz[3]),
                float(actual_wxyz[0]),
            )
            actual_tip_matrix = pose_matrix(actual_position, actual_xyzw) @ tool_to_tip
            actual_tip_position, actual_tip_quaternion = matrix_pose(actual_tip_matrix)
            actual_tip_positions.append(actual_tip_position)
            position_errors.append(
                float(
                    np.linalg.norm(
                        np.asarray(actual_tip_position, dtype=float)
                        - np.asarray(target.position_m, dtype=float)
                    )
                )
            )
            orientation_errors.append(
                quaternion_angular_distance_deg(
                    actual_tip_quaternion,
                    target.quaternion_xyzw,
                )
            )

        max_position_error = max(position_errors)
        max_orientation_error = max(orientation_errors)
        position_tolerance = float(
            self._planner_value("validation.tip_position_tolerance_m")
        )
        orientation_tolerance = float(
            self._planner_value("validation.tip_orientation_tolerance_deg")
        )
        if max_position_error > position_tolerance:
            raise PlanningError(
                "final constrained-path FK position error exceeds tolerance: "
                f"{max_position_error:.6g} > {position_tolerance:.6g} m"
            )
        if max_orientation_error > orientation_tolerance:
            raise PlanningError(
                "final constrained-path FK orientation error exceeds tolerance: "
                f"{max_orientation_error:.6g} > {orientation_tolerance:.6g} deg"
            )

        contact_spheres = self._canonical_contact_spheres(
            constraint_state,
            len(constrained_tip_poses),
        )
        contact_classification = self._contact_sphere_classification(
            checker,
            contact_policy,
            contact_spheres.shape[1],
        )
        contact_metrics = contact_policy.validate_canonical_samples(
            actual_tip_positions,
            contact_spheres,
            contact_classification,
        )

        joint_metrics = self.validate_joint_trajectory(trajectory)
        return PathValidationMetrics(
            max_tip_position_error_m=max_position_error,
            max_tip_orientation_error_deg=max_orientation_error,
            minimum_collision_distance_m=min(
                joint_metrics.minimum_collision_distance_m,
                contact_metrics.minimum_non_tip_clearance_m,
            ),
            max_scene_collision_cost=joint_metrics.max_scene_collision_cost,
            max_self_collision_cost=joint_metrics.max_self_collision_cost,
            max_joint_bound_cost=joint_metrics.max_joint_bound_cost,
            checked_sample_count=joint_metrics.checked_sample_count,
        )

    @staticmethod
    def _canonical_contact_spheres(
        kinematics_state,
        sample_count: int,
    ) -> np.ndarray:
        try:
            sphere_tensor = kinematics_state.robot_spheres
            spheres = sphere_tensor.detach().cpu().numpy()
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise PlanningError(
                "cuRobo FK did not provide canonical collision sphere geometry"
            ) from error
        if (
            spheres.ndim != 4
            or spheres.shape[-1] != 4
            or spheres.shape[0] * spheres.shape[1] != sample_count
            or spheres.shape[2] <= 0
            or not np.all(np.isfinite(spheres))
        ):
            raise PlanningError(
                "cuRobo FK returned invalid canonical collision sphere geometry"
            )
        return np.asarray(
            spheres.reshape(sample_count, spheres.shape[2], 4),
            dtype=float,
        )

    def _contact_sphere_classification(
        self,
        checker,
        policy: ContactPolicy,
        runtime_sphere_count: int,
    ) -> ContactSphereClassification:
        try:
            kinematics_config = checker.kinematics.config.kinematics_config
            configured_total = int(kinematics_config.total_spheres)
            tool_indices = (
                kinematics_config.get_sphere_index_from_link_name(
                    policy.tool_parent_link
                )
                .detach()
                .cpu()
                .numpy()
            )
            tool_reference_spheres = (
                kinematics_config.get_reference_link_spheres(
                    policy.tool_parent_link
                )
                .detach()
                .cpu()
                .numpy()
            )
        except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as error:
            raise PlanningError(
                "cuRobo cannot expose the sphere/link metadata required by contact policy"
            ) from error
        if configured_total != runtime_sphere_count:
            raise PlanningError(
                "cuRobo runtime sphere count does not match contact-policy metadata"
            )
        return policy.classify_curobo_spheres(
            total_sphere_count=configured_total,
            tool_sphere_indices=tool_indices,
            tool_reference_spheres=tool_reference_spheres,
        )

    def _ensure_runtime(self) -> dict[str, object]:
        if self._runtime is not None:
            return self._runtime
        try:
            import curobo
            import torch
            from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
            from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
            from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
            from curobo.trajectory_optimizer import TrajectoryOptimizer, TrajectoryOptimizerCfg
            from curobo.types import GoalToolPose, JointState, Pose
        except ImportError as error:
            raise CuroboUnavailableError(
                "locked cuRoboV2 runtime is unavailable; install the pinned revision from curobo.lock"
            ) from error
        self._verify_runtime_identity(curobo)
        if not torch.cuda.is_available():
            raise CuroboUnavailableError("cuRoboV2 requires an available CUDA GPU")
        runtime_cuda = str(torch.version.cuda or "")
        locked_cuda = str(self._lock["cuda_toolkit"])
        if runtime_cuda != locked_cuda:
            raise CuroboUnavailableError(
                f"PyTorch CUDA runtime {runtime_cuda or 'unknown'} does not match "
                f"curobo.lock toolkit {locked_cuda}"
            )
        self._runtime = {
            "curobo": curobo,
            "torch": torch,
            "InverseKinematics": InverseKinematics,
            "InverseKinematicsCfg": InverseKinematicsCfg,
            "RobotCollisionChecker": RobotCollisionChecker,
            "RobotCollisionCheckerCfg": RobotCollisionCheckerCfg,
            "MotionPlanner": MotionPlanner,
            "MotionPlannerCfg": MotionPlannerCfg,
            "TrajectoryOptimizer": TrajectoryOptimizer,
            "TrajectoryOptimizerCfg": TrajectoryOptimizerCfg,
            "GoalToolPose": GoalToolPose,
            "JointState": JointState,
            "Pose": Pose,
        }
        return self._runtime

    def _ensure_collision_checker(self, runtime: dict[str, object]):
        if self._collision_checker is not None:
            return self._collision_checker
        RobotCollisionChecker = runtime["RobotCollisionChecker"]
        RobotCollisionCheckerCfg = runtime["RobotCollisionCheckerCfg"]
        minimum_clearance = float(
            self._planner_value("validation.minimum_collision_distance_m")
        )
        config = RobotCollisionCheckerCfg.load_from_config(
            robot_config=self._robot_config(),
            scene_model=self._scene_config(),
            collision_activation_distance=minimum_clearance,
            max_collision_distance=max(1.0, minimum_clearance * 4.0),
        )
        self._collision_checker = RobotCollisionChecker(config)
        return self._collision_checker

    def _verify_runtime_identity(self, curobo_module: object) -> None:
        commit = str(self._lock["commit"])
        expected_source = (
            self.lock_path.parent / ".deps" / f"curobo-{commit}"
        ).resolve()
        marker = expected_source / ".ur-waypoint-curobo-commit"
        module_file_value = getattr(curobo_module, "__file__", None)
        if not isinstance(module_file_value, str) or not module_file_value:
            raise CuroboUnavailableError("imported cuRobo module has no verifiable source path")
        module_file = Path(module_file_value).resolve()
        if expected_source != module_file and expected_source not in module_file.parents:
            raise CuroboUnavailableError(
                "imported cuRobo does not come from the verified locked source: "
                f"expected {expected_source}, got {module_file}"
            )
        try:
            marker_commit = marker.read_text(encoding="utf-8").strip()
        except OSError as error:
            raise CuroboUnavailableError(
                f"verified cuRobo source marker is unavailable: {marker}"
            ) from error
        if marker_commit != commit:
            raise CuroboUnavailableError(
                "verified cuRobo source marker does not match curobo.lock"
            )

    def _ensure_motion_planner(self, runtime: dict[str, object]) -> None:
        if self._motion_planner is not None:
            return
        MotionPlanner = runtime["MotionPlanner"]
        MotionPlannerCfg = runtime["MotionPlannerCfg"]
        config = MotionPlannerCfg.create(
            robot=self._robot_config(),
            scene_model=self._scene_config(),
            self_collision_check=True,
            num_ik_seeds=int(self._planner_value("ik.candidates_per_pose")),
            num_trajopt_seeds=4,
            position_tolerance=float(self._planner_value("validation.tip_position_tolerance_m")),
            orientation_tolerance=float(
                np.radians(self._planner_value("validation.tip_orientation_tolerance_deg"))
            ),
            optimizer_collision_activation_distance=float(
                self._planner_value("validation.minimum_collision_distance_m")
            ),
            # Prepare planning is a low-frequency safety operation.  The
            # locked cuRobo/PyTorch/CUDA stack can fault on the second
            # plan_cspace call when its graph-captured interpolation horizon
            # changes, poisoning the long-lived server CUDA context.  Keep
            # MotionPlanner eager so repeated prepare requests remain usable;
            # this does not change any authoritative source Pose or time.
            use_cuda_graph=False,
        )
        # MotionPlanner retimes every seed before ranking it.  A rejected seed
        # can exceed V2's default 5000-sample buffer.  Preallocate the explicit
        # environment capacity so a long authoritative horizon never triggers
        # a shape-changing CUDA graph rebuild or an implementation-level
        # steps_max rejection.  This is storage capacity only; it never changes
        # any source time or trajectory sample.
        config.trajopt_solver_config.interpolation_buffer_size = (
            self._interpolation_buffer_size()
        )
        self._motion_planner = MotionPlanner(config)

    def _interpolation_buffer_size(self) -> int:
        value = self._planner_value("curobo.interpolation_buffer_size")
        if isinstance(value, bool):
            raise PlanningError("curobo.interpolation_buffer_size must be an integer")
        try:
            result = int(value)
        except (TypeError, ValueError) as error:
            raise PlanningError(
                "curobo.interpolation_buffer_size must be an integer"
            ) from error
        if result != value or not 2 <= result <= _MAX_INTERPOLATION_BUFFER_SIZE:
            raise PlanningError(
                "curobo.interpolation_buffer_size must be between 2 and 1000000"
            )
        return result

    @staticmethod
    def _fixed_time_interpolation_dt(
        base_dt: float,
        maximum_interpolation_dt: float,
    ) -> float:
        """Choose an even, dense subdivision no coarser than cuRobo's default."""

        if (
            isinstance(maximum_interpolation_dt, bool)
            or not np.isfinite(base_dt)
            or not np.isfinite(maximum_interpolation_dt)
            or base_dt <= 0.0
            or maximum_interpolation_dt <= 0.0
        ):
            raise PlanningError("cuRobo fixed-time interpolation dt must be positive and finite")
        raw_dt = np.float32(base_dt)
        maximum_dt = np.float32(maximum_interpolation_dt)
        minimum_steps = max(
            2,
            int(np.ceil(float(raw_dt / maximum_dt))),
        )
        # Division by a power of two is an exact binary scaling operation.
        # This prevents float32 remainder noise in locked V2's
        # ``(raw + raw % interpolation) / interpolation`` formula from
        # turning an intended even subdivision into an odd ``steps + 1``.
        steps_per_interval = 1 << (minimum_steps - 1).bit_length()
        interpolation_dt = float(raw_dt / np.float32(steps_per_interval))
        actual_steps = CuroboAdapterV2._curobo_linear_interpolation_steps_per_interval(
            base_dt,
            interpolation_dt,
        )
        if (
            actual_steps != steps_per_interval
            or actual_steps < 2
            or actual_steps % 2
            or np.float32(interpolation_dt) > maximum_dt
        ):
            raise PlanningError(
                "cuRobo fixed-time interpolation cannot preserve dense midpoint sampling"
            )
        return interpolation_dt

    @staticmethod
    def _curobo_linear_interpolation_steps_per_interval(
        raw_dt: float,
        interpolation_dt: float,
    ) -> int:
        """Mirror locked V2 calculate_traj_steps for LINEAR_CUDA."""

        raw = np.float32(raw_dt)
        interpolation = np.float32(interpolation_dt)
        if (
            not np.isfinite(raw)
            or not np.isfinite(interpolation)
            or raw <= 0.0
            or interpolation <= 0.0
        ):
            raise PlanningError(
                "cuRobo fixed-time interpolation inputs must be positive and finite"
            )
        steps = int((raw + raw % interpolation) / interpolation)
        if steps <= 0:
            raise PlanningError("cuRobo fixed-time interpolation produced no trajectory steps")
        return steps

    def _fixed_time_interpolation_buffer_size(
        self,
        knot_count: int,
        base_dt: float,
        interpolation_dt: float,
    ) -> int:
        """Cover the real locked-V2 steps_max without changing any waypoint."""

        if isinstance(knot_count, bool) or not isinstance(knot_count, int) or knot_count < 2:
            raise PlanningError("fixed-time interpolation horizon must contain at least two knots")
        steps_per_interval = self._curobo_linear_interpolation_steps_per_interval(
            base_dt,
            interpolation_dt,
        )
        required = (knot_count - 1) * steps_per_interval + 1
        if required > _MAX_INTERPOLATION_BUFFER_SIZE:
            raise PlanningError(
                "fixed-time interpolation requires more than 1000000 samples"
            )
        return max(self._interpolation_buffer_size(), required)

    @staticmethod
    def _fixed_time_rollouts(solver) -> tuple[object, ...]:
        optimizer_rollouts = tuple(getattr(solver, "optimizer_rollouts", ()))
        rollouts: list[object] = [
            getattr(solver, "metrics_rollout", None),
            getattr(solver, "auxiliary_rollout", None),
            *optimizer_rollouts,
        ]
        additional = getattr(solver, "additional_metrics_rollouts", None)
        if isinstance(additional, dict):
            rollouts.extend(additional.values())
        return tuple(rollout for rollout in rollouts if rollout is not None)

    def _install_fixed_time_teleport_dt(self, solver, base_dt: float) -> None:
        """Patch the locked teleport omission before optimization metrics run."""

        metrics_rollout = getattr(solver, "metrics_rollout", None)
        required_metrics_transitions = (
            getattr(metrics_rollout, "transition_model", None),
            getattr(metrics_rollout, "metrics_transition_model", None),
        )
        installed_transition_ids: set[int] = set()
        for rollout in self._fixed_time_rollouts(solver):
            for transition_name in ("transition_model", "metrics_transition_model"):
                transition = getattr(rollout, transition_name, None)
                if transition is None or id(transition) in installed_transition_ids:
                    continue
                if getattr(transition, "teleport_mode", None) is not True:
                    continue
                for attribute in ("_rollout_step_fn", "_cmd_step_fn"):
                    delegate = getattr(transition, attribute, None)
                    if delegate is None:
                        raise PlanningError(
                            "cuRoboV2 teleport transition does not expose its state step"
                        )
                    setattr(
                        transition,
                        attribute,
                        _FixedTimeTeleportStep(delegate, base_dt),
                    )
                installed_transition_ids.add(id(transition))
        if any(
            transition is None or id(transition) not in installed_transition_ids
            for transition in required_metrics_transitions
        ):
            raise PlanningError(
                "cuRoboV2 metrics rollout is not using both required teleport transitions"
            )

    @staticmethod
    def _validate_fixed_time_solution_dt(joint_state: object, base_dt: float) -> None:
        """Verify real dt propagated before cuRobo's interpolated metrics."""

        dt = getattr(joint_state, "dt", None)
        if dt is None:
            raise PlanningError("cuRoboV2 fixed-time joint path did not expose trajectory dt")
        try:
            values = dt.detach().cpu().numpy()
        except (AttributeError, RuntimeError, TypeError, ValueError) as error:
            raise PlanningError(
                "cuRoboV2 fixed-time joint path exposed an invalid trajectory dt"
            ) from error
        if values.size == 0 or not np.all(np.isfinite(values)) or not np.allclose(
            values,
            base_dt,
            rtol=1.0e-6,
            atol=1.0e-9,
        ):
            raise PlanningError(
                "cuRoboV2 fixed-time joint path did not preserve authoritative trajectory dt"
            )

    def _robot_config(self) -> dict:
        robot = copy.deepcopy(dict(self.environment.curobo_robot))
        robot.pop("schema_version", None)
        if "robot_cfg" not in robot:
            raise PlanningError("robot/curobo_robot.yaml must contain robot_cfg")
        robot_cfg = robot["robot_cfg"]
        if not isinstance(robot_cfg, dict):
            raise PlanningError("robot/curobo_robot.yaml robot_cfg must be a mapping")
        kinematics = robot_cfg.setdefault("kinematics", {})
        if not isinstance(kinematics, dict):
            raise PlanningError("cuRobo robot kinematics must be a mapping")
        cspace = kinematics.get("cspace")
        curobo_joint_names = (
            cspace.get("joint_names") if isinstance(cspace, dict) else None
        )
        if (
            not isinstance(curobo_joint_names, list)
            or any(not isinstance(name, str) for name in curobo_joint_names)
            or tuple(curobo_joint_names) != self.joint_names
        ):
            raise PlanningError(
                "cuRobo cspace joint_names must exactly match robot.joint_names order"
            )
        asset_value = kinematics.get("asset_root_path", "robot/assets")
        if not isinstance(asset_value, str) or not asset_value.strip():
            raise PlanningError("robot asset_root_path must be a non-empty path")
        asset_root = Path(asset_value).expanduser()
        if not asset_root.is_absolute():
            asset_root = (self.environment.root / asset_root).resolve()
        else:
            asset_root = asset_root.resolve()
        if asset_root != self.environment.root and self.environment.root not in asset_root.parents:
            raise PlanningError("robot assets must remain inside the environment directory")

        urdf_value = kinematics.get("urdf_path")
        if not isinstance(urdf_value, str) or not urdf_value.strip():
            raise PlanningError("robot urdf_path must be a non-empty path")
        urdf_path = Path(urdf_value).expanduser()
        if not urdf_path.is_absolute():
            urdf_path = (asset_root / urdf_path).resolve()
        else:
            urdf_path = urdf_path.resolve()
        if asset_root != urdf_path.parent and asset_root not in urdf_path.parents:
            raise PlanningError("robot URDF must remain inside asset_root_path")
        if not urdf_path.is_file():
            raise PlanningError(
                f"generated robot URDF is missing: {urdf_path}; run generate_ur_assets.sh"
            )
        kinematics["asset_root_path"] = str(asset_root)
        kinematics["urdf_path"] = str(urdf_path)

        sphere_data = self.environment.collision_spheres.get("collision_spheres", {})
        kinematics.setdefault("collision_spheres", {}).update(copy.deepcopy(sphere_data))
        generated = RobotToolBuilder(self.environment).collision_spheres()
        tool_spheres = kinematics["collision_spheres"].setdefault(self._target_link(), [])
        tool_spheres.extend(
            {"center": list(sphere.center_m), "radius": sphere.radius_m} for sphere in generated
        )
        return robot

    def _scene_config(self) -> dict:
        try:
            return resolved_curobo_scene(self.environment)
        except SceneConfigurationError as error:
            raise PlanningError(f"cannot resolve the shared environment scene: {error}") from error

    def _target_link(self) -> str:
        return RobotToolBuilder(self.environment).parent_link

    def _ik_random_seed_reserve(self, solver_num_seeds: int) -> int:
        value = self._planner_value("ik.random_seed_reserve")
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value <= 0
            or value >= solver_num_seeds
        ):
            raise PlanningError(
                "planner ik.random_seed_reserve must be a positive integer smaller "
                "than the cuRobo IK seed count"
            )
        return value

    def _internal_pose_tolerances(self) -> tuple[float, float]:
        ratio = self._planner_value("validation.internal_tolerance_guard_ratio")
        if (
            isinstance(ratio, bool)
            or not isinstance(ratio, (int, float))
            or not np.isfinite(float(ratio))
            or not 0.0 < float(ratio) < 1.0
        ):
            raise PlanningError(
                "planner validation.internal_tolerance_guard_ratio must be finite and "
                "strictly between 0 and 1"
            )
        guard_ratio = float(ratio)
        return (
            float(self._planner_value("validation.tip_position_tolerance_m"))
            * guard_ratio,
            float(
                np.radians(
                    self._planner_value("validation.tip_orientation_tolerance_deg")
                )
            )
            * guard_ratio,
        )

    @staticmethod
    def _diversify_previous_ik_seeds(
        positions: np.ndarray,
        maximum_seed_count: int,
    ) -> np.ndarray:
        """Keep a deterministic, diverse subset when the previous frontier is full."""

        if len(positions) <= maximum_seed_count:
            return positions

        unique_rows: list[np.ndarray] = []
        for row in positions:
            if any(
                float(np.max(np.abs(row - existing)))
                <= _IK_SEED_DEDUPLICATION_TOLERANCE_RAD
                for existing in unique_rows
            ):
                continue
            unique_rows.append(row)
        unique = np.asarray(unique_rows, dtype=positions.dtype)
        if len(unique) <= maximum_seed_count:
            return unique

        # Preserve the solver's best-ranked first candidate, then greedily add
        # the candidate farthest from its nearest selected neighbor.  Sorting
        # the final indices restores the solver's original ranking among the
        # selected, diverse set.
        selected = [0]
        minimum_squared_distance = np.sum((unique - unique[0]) ** 2, axis=1)
        minimum_squared_distance[0] = -np.inf
        while len(selected) < maximum_seed_count:
            next_index = int(np.argmax(minimum_squared_distance))
            selected.append(next_index)
            next_squared_distance = np.sum(
                (unique - unique[next_index]) ** 2,
                axis=1,
            )
            minimum_squared_distance = np.minimum(
                minimum_squared_distance,
                next_squared_distance,
            )
            minimum_squared_distance[selected] = -np.inf
        selected.sort()
        return unique[np.asarray(selected, dtype=int)]

    def _planner_value(self, dotted_path: str):
        value = self.environment.planner
        for key in dotted_path.split("."):
            if not isinstance(value, dict) or key not in value:
                raise PlanningError(f"planner.yaml is missing {dotted_path}")
            value = value[key]
        return value

    def _load_lock(self) -> dict:
        try:
            data = yaml.safe_load(self.lock_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as error:
            raise CuroboUnavailableError(f"cannot read cuRobo lock: {error}") from error
        if not isinstance(data, dict) or data.get("schema_version") != 1:
            raise CuroboUnavailableError("curobo.lock schema_version must be 1")
        commit = str(data.get("commit", ""))
        if len(commit) != 40 or commit.lower() == "main":
            raise CuroboUnavailableError("curobo.lock must pin an immutable 40-character commit")
        return data

    def _joint_position_limits(self, solver) -> tuple[np.ndarray, np.ndarray]:
        limits = solver.kinematics.get_joint_limits().position.detach().cpu().numpy()
        return np.asarray(limits[0], dtype=float), np.asarray(limits[1], dtype=float)

    def _validate_solver_joint_names(self, joint_state: object, context: str) -> None:
        returned_names = getattr(joint_state, "joint_names", None)
        if returned_names is None:
            return
        if (
            not isinstance(returned_names, (list, tuple))
            or any(not isinstance(name, str) for name in returned_names)
            or tuple(returned_names) != self.joint_names
        ):
            raise PlanningError(
                f"cuRoboV2 {context} joint_names do not match the authoritative order"
            )

    def _candidate_singularity_scores(
        self,
        solver,
        positions: np.ndarray,
    ) -> np.ndarray:
        """Score IK candidates with bounded, order-preserving batched FK/SVD."""

        try:
            candidate_positions = np.asarray(positions, dtype=np.float32)
        except (OverflowError, TypeError, ValueError) as error:
            raise PlanningError("cuRobo IK singularity input is invalid") from error
        if (
            candidate_positions.ndim != 2
            or candidate_positions.shape[1] != len(self.joint_names)
            or not np.all(np.isfinite(candidate_positions))
        ):
            raise PlanningError("cuRobo IK singularity input has an invalid shape or value")
        candidate_count = candidate_positions.shape[0]
        if candidate_count == 0:
            return np.empty((0,), dtype=float)

        runtime = self._ensure_runtime()
        try:
            torch = runtime["torch"]
            JointState = runtime["JointState"]
        except (KeyError, TypeError) as error:
            raise PlanningError(
                "cuRobo IK singularity scoring runtime is incomplete"
            ) from error

        score_chunks: list[np.ndarray] = []
        joint_count = len(self.joint_names)
        expected_singular_value_count = min(6, joint_count)
        for start in range(0, candidate_count, _SINGULARITY_SCORE_CHUNK_SIZE):
            end = min(start + _SINGULARITY_SCORE_CHUNK_SIZE, candidate_count)
            chunk_count = end - start
            try:
                state = JointState.from_position(
                    torch.as_tensor(
                        candidate_positions[start:end],
                        device="cuda",
                        dtype=torch.float32,
                    ),
                    joint_names=list(self.joint_names),
                )
                kinematics = solver.compute_kinematics(state)
                jacobians = getattr(kinematics, "tool_jacobians", None)
            except (AttributeError, KeyError, RuntimeError, TypeError, ValueError) as error:
                raise PlanningError(
                    "cuRoboV2 could not evaluate batched IK candidate Jacobians"
                ) from error
            if jacobians is None:
                raise PlanningError(
                    "cuRoboV2 returned no tool Jacobians for IK candidate scoring"
                )

            try:
                jacobian_shape = tuple(int(value) for value in jacobians.shape)
            except (AttributeError, OverflowError, TypeError, ValueError) as error:
                raise PlanningError(
                    "cuRoboV2 returned an invalid IK candidate Jacobian shape"
                ) from error
            expected_jacobian_shape = (chunk_count, 1, 1, 6, joint_count)
            if jacobian_shape != expected_jacobian_shape:
                raise PlanningError(
                    "cuRoboV2 returned an invalid IK candidate Jacobian shape"
                )

            try:
                matrices = jacobians[:, 0, 0, :, :]
                jacobians_are_finite = bool(
                    torch.all(torch.isfinite(matrices)).item()
                )
            except (AttributeError, RuntimeError, TypeError, ValueError) as error:
                raise PlanningError(
                    "cuRoboV2 returned invalid IK candidate Jacobians"
                ) from error
            if not jacobians_are_finite:
                raise PlanningError(
                    "cuRoboV2 returned non-finite IK candidate Jacobians"
                )

            try:
                singular_values = torch.linalg.svdvals(matrices)
                extrema = torch.stack(
                    (
                        torch.min(singular_values, dim=-1).values,
                        torch.max(singular_values, dim=-1).values,
                    ),
                    dim=-1,
                )
                extrema_values = extrema.detach().cpu().numpy()
            except (AttributeError, RuntimeError, TypeError, ValueError) as error:
                raise PlanningError(
                    "cuRoboV2 batched IK candidate singularity SVD failed"
                ) from error
            if (
                tuple(singular_values.shape)
                != (chunk_count, expected_singular_value_count)
                or extrema_values.shape != (chunk_count, 2)
                or not np.all(np.isfinite(extrema_values))
            ):
                raise PlanningError(
                    "cuRoboV2 returned invalid IK candidate singularity values"
                )
            chunk_scores = np.asarray(
                [
                    float(maximum) / max(float(minimum), 1.0e-6)
                    for minimum, maximum in zip(
                        extrema_values[:, 0],
                        extrema_values[:, 1],
                        strict=True,
                    )
                ],
                dtype=float,
            )
            if chunk_scores.shape != (chunk_count,) or not np.all(
                np.isfinite(chunk_scores)
            ):
                raise PlanningError(
                    "cuRoboV2 produced invalid IK candidate singularity scores"
                )
            score_chunks.append(chunk_scores)

        scores = np.concatenate(score_chunks)
        if scores.shape != (candidate_count,) or not np.all(np.isfinite(scores)):
            raise PlanningError(
                "cuRoboV2 produced misaligned IK candidate singularity scores"
            )
        return scores

    @staticmethod
    def _xyzw_to_wxyz(quaternion: Sequence[float]) -> tuple[float, float, float, float]:
        x, y, z, w = quaternion
        return float(w), float(x), float(y), float(z)

    @staticmethod
    def _transition_model(knot_count: int, base_dt: float) -> dict:
        return {
            "transition_model_cfg": {
                # Direct position knots preserve the one-input-Pose/one-output-knot
                # contract.  cuRobo's transition model still supplies FK,
                # collision, interpolation, and joint-bound evaluation; the
                # adapter adds exact-time finite-difference regularization to the
                # live c-space cost because teleport parameterization itself does
                # not synthesize qd/qdd/qddd.
                "control_space": "POSITION",
                "dt_traj_params": {
                    "base_dt": base_dt,
                    "base_ratio": 1.0,
                    "max_dt": base_dt,
                },
                "horizon": knot_count,
                "interpolation_steps": 1,
                "state_filter_cfg": {
                    "enable": False,
                    "filter_coeff": {"acceleration": 0.0, "position": 1.0, "velocity": 1.0},
                },
                "teleport_mode": True,
                "vel_scale": 1.0,
                "return_full_act_buffer": True,
            }
        }
