from __future__ import annotations

from math import isfinite
from typing import Mapping, Sequence

from .curobo_adapter_v2 import CuroboAdapter
from .environment_builder import EnvironmentBundle
from .errors import PlanningError, ValidationError
from .models import CompiledTrajectory, JointSample
from .robot_kinematics import QuinticHermiteSegment
from .trajectory_optimizer import DynamicLimits, ResamplePolicy, TrajectoryOptimizer


class PreparePlanner:
    """Build one bounded, complete q/qd/qdd prepare trajectory with cuRobo c-space planning."""

    def __init__(self, environment: EnvironmentBundle, adapter: CuroboAdapter) -> None:
        self.environment = environment
        self.adapter = adapter
        if adapter.joint_names != environment.joint_names:
            raise PlanningError("prepare planner joint order does not match the environment")

    def plan(
        self,
        joint_names: Sequence[str],
        current_positions: Sequence[float],
        target_positions: Sequence[float],
    ) -> CompiledTrajectory:
        names = tuple(str(name) for name in joint_names)
        if names != self.environment.joint_names:
            raise PlanningError("prepare request joint_names do not exactly match the fixed environment")
        current = self._positions(current_positions, "current_positions")
        target = self._positions(target_positions, "target_positions")
        if max(abs(start - goal) for start, goal in zip(current, target, strict=True)) <= 1.0e-9:
            # A mandatory startup stage still needs positive collision evidence when the
            # robot is already at its configured target.  Return a short, strictly static
            # controller goal and run it through the same full joint/collision validator.
            zero = (0.0,) * len(names)
            hold_duration_s = float(
                self._value(self.environment.planner, "timed_path.nominal_output_dt_s")
            )
            trajectory = CompiledTrajectory(
                joint_names=names,
                samples=(
                    JointSample(0.0, current, zero, zero, "approach"),
                    JointSample(hold_duration_s, current, zero, zero, "approach"),
                ),
                metadata={
                    "prepare": {
                        "curobo_revision": self.adapter.revision,
                        "uniform_time_scale": 1.0,
                        "max_joint_velocity_ratio": 0.0,
                        "max_joint_acceleration_ratio": 0.0,
                        "max_joint_jerk_ratio": 0.0,
                        "stationary_collision_check": True,
                    }
                },
            )
            self._require_canonical_joint_branch(
                tuple(sample.positions for sample in trajectory.samples),
                "stationary",
            )
            self._validate_sent_payload_dynamics(trajectory)
            self.adapter.validate_joint_trajectory(trajectory)
            return trajectory

        path = self.adapter.plan_cspace(current, target)
        if len(path.times_s) != len(path.positions) or len(path.times_s) < 2:
            raise PlanningError("cuRobo prepare path has invalid time/position dimensions")
        self._require_canonical_joint_branch(path.positions, "cuRobo knot")
        # Prepare is a separately generated motion, not part of the authority
        # source CSV. Canonicalize its solver-generated clock to ROS Duration's
        # native nanosecond grid before deriving or validating qd/qdd.
        time_ns = tuple(int(round(float(value) * 1.0e9)) for value in path.times_s)
        if time_ns[0] != 0 or any(
            current_ns <= previous_ns
            for previous_ns, current_ns in zip(time_ns[:-1], time_ns[1:], strict=True)
        ):
            raise PlanningError("cuRobo prepare path collapses on the ROS nanosecond grid")
        canonical_times = tuple(value / 1.0e9 for value in time_ns)
        optimizer = self._optimizer()
        trajectory, metrics = optimizer.build(
            names,
            canonical_times,
            path.positions,
            ("approach",) * len(path.times_s),
        )
        self._require_endpoint(trajectory.samples[0].positions, current, "start")
        self._require_endpoint(trajectory.samples[-1].positions, target, "target")
        for label, sample in (("first", trajectory.samples[0]), ("last", trajectory.samples[-1])):
            if max(abs(value) for value in sample.velocities) > 1.0e-9:
                raise PlanningError(f"prepare {label} sample velocity is not zero")
            if max(abs(value) for value in sample.accelerations) > 1.0e-9:
                raise PlanningError(f"prepare {label} sample acceleration is not zero")
        self._require_canonical_joint_branch(
            tuple(sample.positions for sample in trajectory.samples),
            "resampled",
        )
        payload_ratios = self._validate_sent_payload_dynamics(trajectory)
        # The SciPy resampling step can curve between cuRobo's collision-checked
        # knots.  Recheck every emitted q/qd/qdd sample before the service is
        # allowed to return a prepare trajectory.
        self.adapter.validate_joint_trajectory(trajectory)
        metadata = dict(trajectory.metadata)
        metadata["prepare"] = {
            "curobo_revision": self.adapter.revision,
            "uniform_time_scale": metrics.uniform_time_scale,
            "max_joint_velocity_ratio": metrics.max_joint_velocity_ratio,
            "max_joint_acceleration_ratio": metrics.max_joint_acceleration_ratio,
            "max_joint_jerk_ratio": metrics.max_joint_jerk_ratio,
            "sent_payload_max_joint_velocity_ratio": payload_ratios[0],
            "sent_payload_max_joint_acceleration_ratio": payload_ratios[1],
            "sent_payload_max_joint_jerk_ratio": payload_ratios[2],
            "optional_sampling": {
                "scope": "generated_prepare_only",
                "minimum_optional_sample_spacing_s": optimizer.policy.minimum_optional_sample_spacing_s,
                "solver_knots_and_ros_nanoseconds_unchanged": True,
                "source_csv_modified": False,
            },
        }
        return CompiledTrajectory(
            joint_names=trajectory.joint_names,
            samples=trajectory.samples,
            metadata=metadata,
        )

    def _positions(self, values: Sequence[float], name: str) -> tuple[float, ...]:
        result = tuple(float(value) for value in values)
        if len(result) != len(self.environment.joint_names):
            raise PlanningError(
                f"prepare {name} must contain {len(self.environment.joint_names)} joints"
            )
        if not all(isfinite(value) for value in result):
            raise PlanningError(f"prepare {name} contains a non-finite value")
        return result

    def _require_canonical_joint_branch(
        self,
        positions: Sequence[Sequence[float]],
        label: str,
    ) -> None:
        try:
            mask = self.environment.canonical_joint_branch_mask(positions)
        except (KeyError, TypeError, ValueError) as error:
            raise PlanningError("prepare canonical joint branch configuration is invalid") from error
        rejected = [index for index, accepted in enumerate(mask) if not bool(accepted)]
        if rejected:
            raise PlanningError(
                f"prepare {label} sample {rejected[0]} leaves the configured "
                "canonical joint branch"
            )

    @staticmethod
    def _require_endpoint(
        actual: Sequence[float],
        expected: Sequence[float],
        label: str,
    ) -> None:
        error = max(abs(value - target) for value, target in zip(actual, expected, strict=True))
        if error > 1.0e-6:
            raise PlanningError(f"prepare trajectory {label} endpoint error is {error:.6g} rad")

    def _optimizer(self) -> TrajectoryOptimizer:
        robot_limits = self.environment.robot_model["robot"]["limits"]
        planner = self.environment.planner
        return TrajectoryOptimizer(
            DynamicLimits(
                velocity_rad_s=tuple(float(value) for value in robot_limits["velocity_rad_s"]),
                acceleration_rad_s2=tuple(
                    float(value) for value in robot_limits["acceleration_rad_s2"]
                ),
                jerk_rad_s3=tuple(float(value) for value in robot_limits["jerk_rad_s3"]),
            ),
            ResamplePolicy(
                nominal_dt_s=float(self._value(planner, "timed_path.nominal_output_dt_s")),
                minimum_optional_sample_spacing_s=float(
                    self._value(planner, "timed_path.nominal_output_dt_s")
                ) / 4.0,
                allow_uniform_time_scale=bool(
                    self._value(planner, "limits.allow_uniform_time_scale")
                ),
                on_violation=str(self._value(planner, "limits.on_violation")),
                max_uniform_time_scale=float(
                    self._value(planner, "limits.max_uniform_time_scale")
                ),
            ),
        )

    def _validate_sent_payload_dynamics(
        self, trajectory: CompiledTrajectory
    ) -> tuple[float, float, float]:
        """Check the actual controller's adjacent q/qd/qdd quintics, without repair."""
        limits = self.environment.robot_model["robot"]["limits"]
        limit_groups = (
            limits["velocity_rad_s"],
            limits["acceleration_rad_s2"],
            limits["jerk_rad_s3"],
        )
        maximum_ratios = [0.0, 0.0, 0.0]
        for segment_index, (start, end) in enumerate(
            zip(trajectory.samples[:-1], trajectory.samples[1:], strict=True)
        ):
            for joint_index, joint_name in enumerate(trajectory.joint_names):
                try:
                    segment = QuinticHermiteSegment.from_boundary_conditions(
                        duration_s=end.time_s - start.time_s,
                        start_position=start.positions[joint_index],
                        start_velocity=start.velocities[joint_index],
                        start_acceleration=start.accelerations[joint_index],
                        end_position=end.positions[joint_index],
                        end_velocity=end.velocities[joint_index],
                        end_acceleration=end.accelerations[joint_index],
                    )
                    for order, derivative in enumerate(("velocity", "acceleration", "jerk"), 1):
                        peak = segment.maximum_absolute(order)
                        limit = float(limit_groups[order - 1][joint_index])
                        if not isfinite(limit) or limit <= 0.0:
                            raise ValueError(f"invalid {derivative} limit")
                        ratio = peak.absolute_value / limit
                        maximum_ratios[order - 1] = max(maximum_ratios[order - 1], ratio)
                        if not isfinite(ratio) or ratio > 1.0 + 1.0e-9:
                            raise ValidationError(
                                "prepare sent payload quintic dynamics violates "
                                f"{derivative} limit: joint={joint_name}, "
                                f"segment={segment_index}, "
                                f"time_s={start.time_s + peak.time_s:.9f}, ratio={ratio:.9g}; "
                                "trajectory modification is forbidden"
                            )
                except ValueError as error:
                    raise ValidationError(
                        "prepare sent payload quintic segment is invalid: "
                        f"joint={joint_name}, segment={segment_index}: {error}"
                    ) from error
        return tuple(maximum_ratios)

    @staticmethod
    def _value(mapping: Mapping[str, object], dotted_path: str) -> object:
        value: object = mapping
        for key in dotted_path.split("."):
            if not isinstance(value, Mapping) or key not in value:
                raise PlanningError(f"missing prepare planner setting {dotted_path}")
            value = value[key]
        return value
