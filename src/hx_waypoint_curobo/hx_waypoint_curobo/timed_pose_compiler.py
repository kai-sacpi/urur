from __future__ import annotations

from dataclasses import dataclass
from math import ceil, floor, radians
from pathlib import Path
from typing import Mapping, Sequence
from xml.etree import ElementTree

import numpy as np

from .continuous_ik_selector import (
    ContinuousIKSelector,
    SelectionLimits,
    SelectionWeights,
)
from .curobo_adapter_v2 import CuroboAdapter, JointPath, PathValidationMetrics
from .environment_builder import EnvironmentBundle, JointPositionDomain
from .errors import PlanningError, ValidationError
from .math3d import quaternion_angular_distance_deg, slerp_xyzw
from .models import CompiledTrajectory, TimedPose
from .robot_tool_builder import RobotToolBuilder
from .trajectory_optimizer import (
    DynamicLimits,
    ResamplePolicy,
    TrajectoryMetrics,
    TrajectoryOptimizer,
)


@dataclass(frozen=True)
class CompilationResult:
    trajectory: CompiledTrajectory
    source_poses: tuple[TimedPose, ...]
    resampled_source_poses: tuple[TimedPose, ...]
    metrics: TrajectoryMetrics
    validation_metrics: PathValidationMetrics
    curobo_revision: str


class CuroboTimedPathCompiler:
    """Compile only the authoritative source path before controller execution."""

    def __init__(self, environment: EnvironmentBundle, adapter: CuroboAdapter) -> None:
        self.environment = environment
        self.adapter = adapter
        if adapter.joint_names != environment.joint_names:
            raise PlanningError("cuRobo joint order does not match the environment robot model")

    def compile(self, source_poses: Sequence[TimedPose]) -> CompilationResult:
        source = tuple(source_poses)
        if len(source) < 2:
            raise PlanningError("timed path compilation requires at least two source poses")

        transformed = tuple(self.environment.transform_pose_to_base(pose) for pose in source)
        resampled = self._resample_authoritative_path(transformed)
        authoritative_times = tuple(float(pose.time_s) for pose in resampled)
        authoritative_phases = tuple(pose.phase for pose in resampled)
        tool_builder = RobotToolBuilder(self.environment)
        authoritative_tool_poses = tuple(
            tool_builder.tip_pose_to_tool_pose(pose) for pose in resampled
        )
        self._validate_endpoint_rest_dynamics(authoritative_tool_poses)
        self.adapter.warmup()

        candidate_count = int(self._planner_value("ik.candidates_per_pose"))
        candidate_sets = self.adapter.solve_ik_candidates(
            authoritative_tool_poses,
            candidates_per_pose=candidate_count,
        )
        if len(candidate_sets) != len(authoritative_tool_poses):
            raise PlanningError("cuRobo did not return one candidate set per pose")
        selector = self._selector()
        reference_positions = tuple(
            float(value)
            for value in self.environment.robot_model["robot"][
                "home_joint_positions"
            ]
        )
        selected = selector.select(
            authoritative_times,
            candidate_sets,
            reference_positions=reference_positions,
        )
        selected_path = self._freeze_hold_knots(
            JointPath(
                times_s=authoritative_times,
                positions=tuple(candidate.positions for candidate in selected),
            ),
            authoritative_phases,
        )
        selected_path, selected_winding_offsets = self._canonicalize_revolute_winding(
            selected_path,
            reference_positions,
            self.environment.joint_position_domains,
        )
        post_optimization_winding_offsets = (0.0,) * len(reference_positions)
        optimizer = self._trajectory_optimizer()
        try:
            trajectory, metrics = optimizer.build(
                self.environment.joint_names,
                selected_path.times_s,
                selected_path.positions,
                authoritative_phases,
            )
        except ValidationError as dynamics_error:
            # A per-knot IK seed already satisfies every authoritative Pose.
            # Preserve it when its fixed-time spline is dynamically feasible;
            # cuRobo's horizon Pose objective is a soft cost, so running it
            # unconditionally can move an otherwise valid intermediate knot to
            # the edge of the hard FK tolerance.  Use fixed-time trajopt only
            # when the direct seed path demonstrably violates dynamics.
            try:
                optimized = self.adapter.optimize_pose_path(
                    authoritative_tool_poses,
                    authoritative_times,
                    selected_path.positions,
                )
                if len(optimized.positions) != len(authoritative_tool_poses):
                    raise PlanningError(
                        "cuRobo optimization changed the authoritative constraint count"
                    )
                if optimized.times_s != authoritative_times:
                    raise PlanningError(
                        "cuRobo optimization changed authoritative constraint times"
                    )
                optimized = self._freeze_hold_knots(optimized, authoritative_phases)
                optimized, post_optimization_winding_offsets = (
                    self._canonicalize_revolute_winding(
                        optimized,
                        reference_positions,
                        self.environment.joint_position_domains,
                    )
                )
                trajectory, metrics = optimizer.build(
                    self.environment.joint_names,
                    optimized.times_s,
                    optimized.positions,
                    authoritative_phases,
                )
            except Exception as fallback_error:
                raise ValidationError(
                    f"{dynamics_error}; cuRobo fixed-time fallback also failed: "
                    f"{type(fallback_error).__name__}: {fallback_error}; authoritative "
                    "waypoint Pose/time constraints were not retimed, scaled, or dropped"
                ) from fallback_error
        trajectory = self._with_winding_metadata(
            trajectory,
            reference_positions,
            selected_winding_offsets,
            post_optimization_winding_offsets,
        )
        validation_poses, validation_positions = self._sample_constrained_validation_path(
            trajectory,
            resampled,
        )
        validation_metrics = self.adapter.validate_compiled_trajectory(
            trajectory,
            validation_poses,
            validation_positions,
        )
        return CompilationResult(
            trajectory=trajectory,
            source_poses=source,
            resampled_source_poses=resampled,
            metrics=metrics,
            validation_metrics=validation_metrics,
            curobo_revision=self.adapter.revision,
        )

    def _validate_endpoint_rest_dynamics(
        self,
        tool_poses: Sequence[TimedPose],
    ) -> None:
        """Reject a Pose/time sequence that cannot meet the endpoint rest contract.

        The canonical controller goal starts and ends with zero joint velocity
        and acceleration.  For a revolute chain, the tool orientation change is
        no greater than the sum of the absolute joint changes, while its linear
        displacement is bounded by those changes times conservative per-joint
        tool radii from the hash-bound URDF.  Starting from rest, each joint
        change over a window ``dt`` is independently bounded by its velocity,
        acceleration, and jerk limits.  These necessary conditions can reject
        an impossible authoritative time sequence before any GPU work, without
        choosing an IK branch or changing a waypoint.
        """

        if len(tool_poses) < 2:
            raise PlanningError("endpoint-rest validation requires at least two poses")
        limits = self.environment.robot_model["robot"]["limits"]
        velocity = np.asarray(limits["velocity_rad_s"], dtype=float)
        acceleration = np.asarray(limits["acceleration_rad_s2"], dtype=float)
        jerk = np.asarray(limits["jerk_rad_s3"], dtype=float)
        expected_shape = (len(self.environment.joint_names),)
        if (
            velocity.shape != expected_shape
            or acceleration.shape != expected_shape
            or jerk.shape != expected_shape
            or np.any(~np.isfinite(velocity))
            or np.any(~np.isfinite(acceleration))
            or np.any(~np.isfinite(jerk))
            or np.any(velocity <= 0.0)
            or np.any(acceleration <= 0.0)
            or np.any(jerk <= 0.0)
        ):
            raise PlanningError("robot dynamics limits are invalid")

        position_tolerance_m = float(
            self._planner_value("validation.tip_position_tolerance_m")
        )
        orientation_tolerance_rad = radians(
            float(self._planner_value("validation.tip_orientation_tolerance_deg"))
        )
        if not np.isfinite(position_tolerance_m) or position_tolerance_m < 0.0:
            raise PlanningError("tip position tolerance is invalid")
        if not np.isfinite(orientation_tolerance_rad) or orientation_tolerance_rad < 0.0:
            raise PlanningError("tip orientation tolerance is invalid")
        translation_radius_bounds_m = self._tool_translation_radius_bounds_m()
        if (
            translation_radius_bounds_m.shape != expected_shape
            or np.any(~np.isfinite(translation_radius_bounds_m))
            or np.any(translation_radius_bounds_m < 0.0)
        ):
            raise PlanningError("tool translation radius bounds are invalid")

        checks = (
            ("start", 0, range(1, len(tool_poses))),
            ("terminal", len(tool_poses) - 1, range(len(tool_poses) - 2, -1, -1)),
        )
        for boundary_name, boundary_index, comparison_indices in checks:
            boundary = tool_poses[boundary_index]
            for comparison_index in comparison_indices:
                comparison = tool_poses[comparison_index]
                duration_s = abs(float(comparison.time_s - boundary.time_s))
                if duration_s <= 0.0:
                    raise PlanningError("endpoint-rest validation received invalid pose times")
                target_translation_m = float(
                    np.linalg.norm(
                        np.asarray(comparison.position_m, dtype=float)
                        - np.asarray(boundary.position_m, dtype=float)
                    )
                )
                required_translation_m = max(
                    0.0,
                    target_translation_m - 2.0 * position_tolerance_m,
                )
                target_rotation_rad = radians(
                    quaternion_angular_distance_deg(
                        boundary.quaternion_xyzw,
                        comparison.quaternion_xyzw,
                    )
                )
                required_rotation_rad = max(
                    0.0,
                    target_rotation_rad - 2.0 * orientation_tolerance_rad,
                )
                maximum_joint_displacement = np.minimum.reduce(
                    (
                        velocity * duration_s,
                        0.5 * acceleration * duration_s**2,
                        jerk * duration_s**3 / 6.0,
                    )
                )
                maximum_rotation_bound_rad = float(
                    np.sum(maximum_joint_displacement)
                )
                maximum_translation_bound_m = float(
                    np.dot(
                        translation_radius_bounds_m,
                        maximum_joint_displacement,
                    )
                )
                translation_infeasible = (
                    required_translation_m > maximum_translation_bound_m + 1.0e-12
                )
                orientation_infeasible = (
                    required_rotation_rad > maximum_rotation_bound_rad + 1.0e-12
                )
                if translation_infeasible or orientation_infeasible:
                    violations = ",".join(
                        name
                        for name, active in (
                            ("translation", translation_infeasible),
                            ("orientation", orientation_infeasible),
                        )
                        if active
                    )
                    raise ValidationError(
                        "authoritative endpoint-rest dynamics are infeasible; trajectory "
                        "retiming is forbidden: "
                        f"{boundary_name}_time_s={boundary.time_s:.9f}, "
                        f"comparison_time_s={comparison.time_s:.9f}, "
                        f"window_s={duration_s:.9f}, "
                        f"violations={violations}, "
                        "required_tool_translation_change_m>="
                        f"{required_translation_m:.9g}, "
                        "maximum_tool_translation_bound_m="
                        f"{maximum_translation_bound_m:.9g}, "
                        "required_tool_orientation_change_rad>="
                        f"{required_rotation_rad:.9g}, "
                        "maximum_joint_displacement_bound_rad="
                        f"{maximum_rotation_bound_rad:.9g}; waypoint Pose and time_s are "
                        "hard constraints and cannot be retimed, scaled, or dropped"
                    )

    def _tool_translation_radius_bounds_m(self) -> np.ndarray:
        """Bound each revolute joint's distance to the configured tool origin.

        Walking the serial URDF chain from the tool toward the base lets us sum
        the norms of all already-seen fixed link translations.  By the triangle
        inequality, that suffix sum bounds the tool's lever arm about the next
        upstream revolute joint for every joint configuration.
        """

        generated = self.environment.asset_provenance.get("generated_urdf")
        if not isinstance(generated, Mapping):
            raise PlanningError(
                "generated robot URDF provenance is invalid for endpoint-rest validation"
            )
        relative_value = generated.get("path")
        if not isinstance(relative_value, str) or not relative_value:
            raise PlanningError(
                "generated robot URDF path is invalid for endpoint-rest validation"
            )
        relative_path = Path(relative_value)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise PlanningError(
                "generated robot URDF path is unsafe for endpoint-rest validation"
            )
        environment_root = self.environment.root.resolve()
        urdf_path = (environment_root / relative_path).resolve()
        try:
            urdf_path.relative_to(environment_root)
        except ValueError as error:
            raise PlanningError(
                "generated robot URDF escapes the environment during endpoint-rest validation"
            ) from error
        try:
            root = ElementTree.parse(urdf_path).getroot()
        except (OSError, ElementTree.ParseError) as error:
            raise PlanningError(
                f"cannot parse generated robot URDF for endpoint-rest validation: {error}"
            ) from error

        joints_by_child: dict[str, tuple[str, str, str, float]] = {}
        for element in root.findall("joint"):
            name = element.get("name")
            joint_type = element.get("type")
            parents = element.findall("parent")
            children = element.findall("child")
            origins = element.findall("origin")
            if (
                not isinstance(name, str)
                or not name
                or not isinstance(joint_type, str)
                or len(parents) != 1
                or len(children) != 1
                or len(origins) > 1
            ):
                raise PlanningError(
                    "generated robot URDF joint graph is invalid for endpoint-rest validation"
                )
            parent_link = parents[0].get("link")
            child_link = children[0].get("link")
            if (
                not isinstance(parent_link, str)
                or not parent_link
                or not isinstance(child_link, str)
                or not child_link
                or child_link in joints_by_child
            ):
                raise PlanningError(
                    "generated robot URDF parent/child graph is invalid for "
                    "endpoint-rest validation"
                )
            xyz_text = origins[0].get("xyz", "0 0 0") if origins else "0 0 0"
            try:
                translation = np.asarray(
                    [float(value) for value in xyz_text.split()],
                    dtype=float,
                )
            except (AttributeError, TypeError, ValueError) as error:
                raise PlanningError(
                    f"generated robot URDF joint {name} has an invalid origin"
                ) from error
            if translation.shape != (3,) or np.any(~np.isfinite(translation)):
                raise PlanningError(
                    f"generated robot URDF joint {name} has an invalid origin"
                )
            joints_by_child[child_link] = (
                parent_link,
                name,
                joint_type,
                float(np.linalg.norm(translation)),
            )

        robot = self.environment.robot_model.get("robot")
        if not isinstance(robot, Mapping):
            raise PlanningError("robot model is invalid for endpoint-rest validation")
        base_link = robot.get("base_link")
        tool_link = robot.get("tool_link")
        if (
            not isinstance(base_link, str)
            or not base_link
            or not isinstance(tool_link, str)
            or not tool_link
        ):
            raise PlanningError("robot base/tool links are invalid for endpoint-rest validation")

        tool_to_base_chain: list[tuple[str, str, str, float]] = []
        current_link = tool_link
        visited_links: set[str] = set()
        while current_link != base_link:
            if current_link in visited_links:
                raise PlanningError(
                    "generated robot URDF tool chain contains a cycle"
                )
            visited_links.add(current_link)
            joint = joints_by_child.get(current_link)
            if joint is None:
                raise PlanningError(
                    "generated robot URDF does not connect the configured tool to the base"
                )
            tool_to_base_chain.append(joint)
            current_link = joint[0]

        expected_names = set(self.environment.joint_names)
        radii_by_name: dict[str, float] = {}
        downstream_translation_m = 0.0
        for _, name, joint_type, origin_translation_m in tool_to_base_chain:
            if name in expected_names:
                if joint_type != "revolute" or name in radii_by_name:
                    raise PlanningError(
                        "endpoint-rest translation validation requires one serial "
                        f"revolute definition for joint {name}"
                    )
                radii_by_name[name] = downstream_translation_m
            downstream_translation_m += origin_translation_m

        missing = [name for name in self.environment.joint_names if name not in radii_by_name]
        if missing:
            raise PlanningError(
                "generated robot URDF tool chain is missing configured joints: "
                + ", ".join(missing)
            )
        return np.asarray(
            [radii_by_name[name] for name in self.environment.joint_names],
            dtype=float,
        )

    def _resample_authoritative_path(self, poses: Sequence[TimedPose]) -> tuple[TimedPose, ...]:
        max_translation = float(self._planner_value("timed_path.max_translation_step_m"))
        max_rotation = float(self._planner_value("timed_path.max_rotation_step_deg"))
        if max_translation <= 0.0 or max_rotation <= 0.0:
            raise PlanningError("timed path resampling limits must be positive")
        output: list[TimedPose] = [poses[0]]
        for start, end in zip(poses[:-1], poses[1:], strict=True):
            distance = float(
                np.linalg.norm(np.asarray(end.position_m) - np.asarray(start.position_m))
            )
            rotation = quaternion_angular_distance_deg(
                start.quaternion_xyzw,
                end.quaternion_xyzw,
            )
            steps = max(1, ceil(distance / max_translation), ceil(rotation / max_rotation))
            start_ns = int(round(start.time_s * 1.0e9))
            end_ns = int(round(end.time_s * 1.0e9))
            if end_ns - start_ns < steps:
                raise PlanningError(
                    "authoritative waypoint interval is too short for nanosecond-safe densification"
                )
            for step in range(1, steps):
                ratio = step / steps
                position = (1.0 - ratio) * np.asarray(start.position_m) + ratio * np.asarray(
                    end.position_m
                )
                quaternion = slerp_xyzw(start.quaternion_xyzw, end.quaternion_xyzw, ratio)
                output.append(
                    TimedPose(
                        time_s=(
                            start_ns
                            + ((end_ns - start_ns) * step + steps // 2) // steps
                        )
                        / 1.0e9,
                        frame_id=start.frame_id,
                        position_m=tuple(float(value) for value in position),
                        quaternion_xyzw=tuple(float(value) for value in quaternion),
                        phase=end.phase,
                    )
                )
            # Append the original object, not an interpolated reconstruction.
            # This preserves every source time and Pose value bit-for-bit after
            # frame conversion while still allowing additional constraints.
            output.append(end)
        return tuple(output)

    @staticmethod
    def _sample_constrained_validation_path(
        trajectory: CompiledTrajectory,
        constrained_tip_poses: Sequence[TimedPose],
    ) -> tuple[tuple[TimedPose, ...], tuple[tuple[float, ...], ...]]:
        if len(constrained_tip_poses) < 2:
            raise PlanningError("constrained validation path requires at least two poses")
        constraint_times = np.asarray(
            [pose.time_s for pose in constrained_tip_poses],
            dtype=float,
        )
        if float(constraint_times[0]) != 0.0 or np.any(np.diff(constraint_times) <= 0.0):
            raise PlanningError("constrained validation pose times are invalid")

        targets: list[TimedPose] = []
        positions: list[tuple[float, ...]] = []
        for sample in trajectory.samples:
            local_time = sample.time_s
            if local_time < -1.0e-9 or local_time > float(constraint_times[-1]) + 1.0e-9:
                continue
            local_time = float(np.clip(local_time, constraint_times[0], constraint_times[-1]))
            upper = int(np.searchsorted(constraint_times, local_time, side="right"))
            if upper == 0:
                target = constrained_tip_poses[0]
            elif upper >= len(constrained_tip_poses):
                target = constrained_tip_poses[-1]
            else:
                start = constrained_tip_poses[upper - 1]
                end = constrained_tip_poses[upper]
                duration = end.time_s - start.time_s
                ratio = (local_time - start.time_s) / duration
                start_position = np.asarray(start.position_m, dtype=float)
                end_position = np.asarray(end.position_m, dtype=float)
                position = (1.0 - ratio) * start_position + ratio * end_position
                target = TimedPose(
                    time_s=sample.time_s,
                    frame_id=start.frame_id,
                    position_m=tuple(float(value) for value in position),
                    quaternion_xyzw=slerp_xyzw(
                        start.quaternion_xyzw,
                        end.quaternion_xyzw,
                        ratio,
                    ),
                    phase=end.phase,
                )
            targets.append(target)
            positions.append(sample.positions)
        if len(targets) < 2:
            raise PlanningError("compiled trajectory contains no constrained samples to validate")
        return tuple(targets), tuple(positions)

    def _selector(self) -> ContinuousIKSelector:
        weights = self.environment.planner["optimization"]
        limits = self.environment.planner["ik"]
        return ContinuousIKSelector(
            weights=SelectionWeights(
                joint_distance=float(weights["joint_distance_weight"]),
                velocity=float(weights["velocity_weight"]),
                acceleration=float(weights["acceleration_weight"]),
                jerk=float(weights["jerk_weight"]),
                joint_limit=float(weights["joint_limit_weight"]),
                collision=float(weights["collision_weight"]),
                singularity=float(weights["singularity_weight"]),
            ),
            limits=SelectionLimits(
                reject_branch_jump_rad=float(limits["reject_branch_jump_rad"]),
                minimum_collision_distance_m=float(
                    self._planner_value("validation.minimum_collision_distance_m")
                ),
                maximum_velocity_rad_s=tuple(
                    float(value)
                    for value in self.environment.robot_model["robot"]["limits"][
                        "velocity_rad_s"
                    ]
                ),
                beam_width=int(limits.get("beam_width", 128)),
            ),
        )

    def _trajectory_optimizer(self) -> TrajectoryOptimizer:
        limits = self.environment.robot_model["robot"]["limits"]
        timing = self.environment.planner["limits"]
        return TrajectoryOptimizer(
            limits=DynamicLimits(
                velocity_rad_s=tuple(float(value) for value in limits["velocity_rad_s"]),
                acceleration_rad_s2=tuple(
                    float(value) for value in limits["acceleration_rad_s2"]
                ),
                jerk_rad_s3=tuple(float(value) for value in limits["jerk_rad_s3"]),
            ),
            policy=ResamplePolicy(
                nominal_dt_s=float(self._planner_value("timed_path.nominal_output_dt_s")),
                allow_uniform_time_scale=timing["allow_uniform_time_scale"],
                on_violation=str(timing["on_violation"]),
                max_uniform_time_scale=float(
                    self._planner_value("limits.max_uniform_time_scale")
                ),
            ),
        )

    @staticmethod
    def _canonicalize_revolute_winding(
        path: JointPath,
        reference_positions: Sequence[float],
        domains: Sequence[JointPositionDomain],
    ) -> tuple[JointPath, tuple[float, ...]]:
        """Apply only one constant full-turn offset per revolute joint.

        The same offset is applied to every knot, so this cannot wrap an
        individual waypoint or hide an internal full-turn discontinuity.  A
        candidate offset is eligible only when the complete path stays inside
        the hash-bound URDF limits and does not reduce its minimum limit margin.
        """

        positions = np.asarray(path.positions, dtype=float)
        reference = np.asarray(reference_positions, dtype=float)
        width = len(domains)
        if (
            positions.ndim != 2
            or positions.shape[0] < 2
            or positions.shape[1] != width
            or len(path.times_s) != positions.shape[0]
            or reference.shape != (width,)
            or not np.all(np.isfinite(positions))
            or not np.all(np.isfinite(reference))
        ):
            raise PlanningError("joint winding canonicalization input is invalid")

        tolerance = 1.0e-9
        offsets = np.zeros(width, dtype=float)
        for joint_index, domain in enumerate(domains):
            values = positions[:, joint_index]
            original_within_limits = not (
                np.any(values < domain.lower - tolerance)
                or np.any(values > domain.upper + tolerance)
            )
            if domain.period is None:
                continue
            period = float(domain.period)
            if not np.isfinite(period) or period <= 0.0:
                raise PlanningError(
                    f"joint winding period for {domain.name} is invalid"
                )

            minimum_k = int(
                ceil(float(np.max((domain.lower - values) / period)) - 1.0e-12)
            )
            maximum_k = int(
                floor(float(np.min((domain.upper - values) / period)) + 1.0e-12)
            )
            if minimum_k > maximum_k:
                # This helper is not a substitute for the final cuRobo joint-
                # bound validator.  Leave a non-canonicalizable invalid path
                # untouched so the normal compile failure remains diagnostic.
                continue
            original_margin = float(
                np.min(np.minimum(values - domain.lower, domain.upper - values))
            )
            candidates: list[tuple[float, float, int, int]] = []
            for turns in range(minimum_k, maximum_k + 1):
                shifted = values + turns * period
                margin = float(
                    np.min(
                        np.minimum(
                            shifted - domain.lower,
                            domain.upper - shifted,
                        )
                    )
                )
                if original_within_limits and margin + tolerance < original_margin:
                    continue
                candidates.append(
                    (
                        abs(float(shifted[0] - reference[joint_index])),
                        -margin,
                        abs(turns),
                        turns,
                    )
                )
            if not candidates:
                raise PlanningError(
                    "no constant full-turn offset preserves the complete path and "
                    f"joint-limit margin for {domain.name}"
                )
            turns = min(candidates)[-1]
            offsets[joint_index] = turns * period

        shifted_positions = positions + offsets.reshape(1, -1)
        original_delta = np.diff(positions, axis=0)
        shifted_delta = np.diff(shifted_positions, axis=0)
        if not np.allclose(original_delta, shifted_delta, rtol=0.0, atol=1.0e-12):
            raise PlanningError(
                "constant joint winding offset changed an internal trajectory delta"
            )
        return (
            JointPath(
                times_s=path.times_s,
                positions=tuple(
                    tuple(float(value) for value in row)
                    for row in shifted_positions
                ),
            ),
            tuple(float(value) for value in offsets),
        )

    @staticmethod
    def _with_winding_metadata(
        trajectory: CompiledTrajectory,
        reference_positions: Sequence[float],
        selected_offsets: Sequence[float],
        post_optimization_offsets: Sequence[float],
    ) -> CompiledTrajectory:
        metadata = dict(trajectory.metadata)
        metadata["joint_winding"] = {
            "policy": "constant_whole_path_nearest_home_without_margin_loss",
            "reference_positions_rad": [float(value) for value in reference_positions],
            "selected_seed_offsets_rad": [float(value) for value in selected_offsets],
            "post_optimization_offsets_rad": [
                float(value) for value in post_optimization_offsets
            ],
        }
        return CompiledTrajectory(
            joint_names=trajectory.joint_names,
            samples=trajectory.samples,
            metadata=metadata,
        )

    @staticmethod
    def _freeze_hold_knots(
        path: JointPath,
        phases: Sequence[str],
    ) -> JointPath:
        if len(path.positions) != len(phases):
            raise PlanningError("optimized path and phase arrays are not aligned")
        positions = list(path.positions)
        for index, phase in enumerate(phases):
            if phase != "hold":
                continue
            if index == 0:
                raise PlanningError("hold cannot be the first optimized knot")
            # A source hold repeats the preceding Cartesian pose. Reusing the
            # preceding collision-checked joint solution prevents independent
            # IK refinement from introducing motion between equivalent poses.
            positions[index] = positions[index - 1]
        return JointPath(times_s=path.times_s, positions=tuple(positions))

    def _planner_value(self, dotted_path: str):
        value = self.environment.planner
        for key in dotted_path.split("."):
            if not isinstance(value, dict) or key not in value:
                raise PlanningError(f"planner.yaml is missing {dotted_path}")
            value = value[key]
        return value
