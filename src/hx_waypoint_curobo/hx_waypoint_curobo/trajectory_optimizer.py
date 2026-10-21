from __future__ import annotations

from dataclasses import dataclass
from math import ceil, isfinite
from typing import Sequence

import numpy as np
from scipy.interpolate import BPoly, CubicSpline, PPoly

from .errors import PlanningError, ValidationError
from .models import CompiledTrajectory, JointSample


@dataclass(frozen=True)
class DynamicLimits:
    velocity_rad_s: tuple[float, ...]
    acceleration_rad_s2: tuple[float, ...]
    jerk_rad_s3: tuple[float, ...]


@dataclass(frozen=True)
class ResamplePolicy:
    nominal_dt_s: float = 0.01
    allow_uniform_time_scale: bool = False
    on_violation: str = "reject"
    max_uniform_time_scale: float = 1.0
    minimum_optional_sample_spacing_s: float = 0.0


@dataclass(frozen=True)
class TrajectoryMetrics:
    original_duration_s: float
    final_duration_s: float
    uniform_time_scale: float
    max_joint_velocity_ratio: float
    max_joint_acceleration_ratio: float
    max_joint_jerk_ratio: float


@dataclass(frozen=True)
class _DerivativeExtrema:
    values: np.ndarray
    segment_indices: np.ndarray
    times_s: np.ndarray


class TrajectoryOptimizer:
    """Sample one C2 quintic curve and derive q/qd/qdd from that same curve."""

    def __init__(self, limits: DynamicLimits, policy: ResamplePolicy) -> None:
        self.limits = limits
        self.policy = policy
        if policy.nominal_dt_s <= 0.0:
            raise PlanningError("nominal_dt_s must be positive")
        nominal_dt_ns = int(round(policy.nominal_dt_s * 1.0e9))
        if (
            nominal_dt_ns <= 0
            or abs(policy.nominal_dt_s - nominal_dt_ns / 1.0e9) > 1.0e-15
        ):
            raise PlanningError("nominal_dt_s must be exactly representable in ROS nanoseconds")
        self._nominal_dt_ns = nominal_dt_ns
        optional_spacing = policy.minimum_optional_sample_spacing_s
        if (
            isinstance(optional_spacing, bool)
            or not isinstance(optional_spacing, (int, float))
            or not isfinite(optional_spacing)
            or not isfinite(optional_spacing * 1.0e9)
            or optional_spacing < 0.0
        ):
            raise PlanningError("minimum_optional_sample_spacing_s must be finite and non-negative")
        self._minimum_optional_spacing_ns = int(ceil(optional_spacing * 1.0e9))
        if not isinstance(policy.allow_uniform_time_scale, bool):
            raise PlanningError("allow_uniform_time_scale must be a boolean")
        if policy.on_violation != "reject":
            raise PlanningError(
                "on_violation must be reject because trajectory retiming is forbidden"
            )
        if policy.allow_uniform_time_scale:
            raise PlanningError(
                "allow_uniform_time_scale must be false because trajectory retiming is forbidden"
            )
        try:
            maximum_scale = float(policy.max_uniform_time_scale)
        except (TypeError, ValueError) as error:
            raise PlanningError(
                "max_uniform_time_scale must be exactly 1.0"
            ) from error
        if isinstance(policy.max_uniform_time_scale, bool):
            raise PlanningError("max_uniform_time_scale must be exactly 1.0")
        if not isfinite(maximum_scale) or abs(maximum_scale - 1.0) > 1.0e-12:
            raise PlanningError("max_uniform_time_scale must be exactly 1.0")

    def build(
        self,
        joint_names: Sequence[str],
        knot_times_s: Sequence[float],
        knot_positions: Sequence[Sequence[float]],
        knot_phases: Sequence[str],
    ) -> tuple[CompiledTrajectory, TrajectoryMetrics]:
        original_times = np.asarray(knot_times_s, dtype=float)
        positions = np.asarray(knot_positions, dtype=float).copy()
        phases = tuple(str(phase).strip().lower() for phase in knot_phases)
        if positions.ndim != 2 or positions.shape[0] != len(original_times):
            raise PlanningError("optimized knots must be an N x J matrix")
        if len(phases) != len(original_times) or np.any(np.diff(original_times) <= 0.0):
            raise PlanningError("optimized knot time and phase arrays are invalid")
        if float(original_times[0]) != 0.0:
            raise PlanningError("optimized knot time must start at zero")
        hold_knots = self._hold_knots(phases, positions)

        trajectory, ratios, extrema = self._sample(
            tuple(joint_names), original_times, positions, phases, hold_knots
        )
        if max(ratios) > 1.0 + 1.0e-9:
            velocity_limits = self._limit_array(
                self.limits.velocity_rad_s,
                positions.shape[1],
                "velocity",
            )
            acceleration_limits = self._limit_array(
                self.limits.acceleration_rad_s2,
                positions.shape[1],
                "acceleration",
            )
            jerk_limits = self._limit_array(
                self.limits.jerk_rad_s3,
                positions.shape[1],
                "jerk",
            )
            raise ValidationError(
                "authoritative input timing violates joint dynamics; trajectory retiming is "
                "forbidden: "
                + "; ".join(
                    (
                        self._format_derivative_peak(
                            "velocity",
                            "rad/s",
                            extrema[0],
                            velocity_limits,
                            joint_names,
                        ),
                        self._format_derivative_peak(
                            "acceleration",
                            "rad/s^2",
                            extrema[1],
                            acceleration_limits,
                            joint_names,
                        ),
                        self._format_derivative_peak(
                            "jerk",
                            "rad/s^3",
                            extrema[2],
                            jerk_limits,
                            joint_names,
                        ),
                    )
                )
            )

        metrics = TrajectoryMetrics(
            original_duration_s=float(original_times[-1]),
            final_duration_s=trajectory.duration_s,
            uniform_time_scale=1.0,
            max_joint_velocity_ratio=ratios[0],
            max_joint_acceleration_ratio=ratios[1],
            max_joint_jerk_ratio=ratios[2],
        )
        metadata = dict(trajectory.metadata)
        metadata["timing"] = {
            "original_duration_s": metrics.original_duration_s,
            "final_duration_s": metrics.final_duration_s,
            "uniform_time_scale": metrics.uniform_time_scale,
            "max_uniform_time_scale": 1.0,
        }
        return CompiledTrajectory(
            joint_names=trajectory.joint_names,
            samples=trajectory.samples,
            metadata=metadata,
        ), metrics

    def _sample(
        self,
        joint_names: tuple[str, ...],
        original_times: np.ndarray,
        positions: np.ndarray,
        phases: tuple[str, ...],
        hold_knots: np.ndarray,
    ) -> tuple[
        CompiledTrajectory,
        tuple[float, float, float],
        tuple[_DerivativeExtrema, _DerivativeExtrema, _DerivativeExtrema],
    ]:
        # The controller time type is integer nanoseconds.  Use that same grid
        # for interpolation, dynamics validation, CSV export, and ROS playback.
        time_ns = np.rint(original_times * 1.0e9).astype(np.int64)
        canonical_times = time_ns.astype(float) / 1.0e9
        if np.any(np.abs(original_times - canonical_times) > 1.0e-15):
            raise PlanningError("optimized knot times must be exact ROS nanoseconds")
        times = canonical_times
        stopped_knots = hold_knots.copy()
        stopped_knots[:-1] |= hold_knots[1:]
        curves: list[BPoly] = []
        for joint_index in range(positions.shape[1]):
            q = positions[:, joint_index]
            cubic = CubicSpline(times, q, bc_type=((1, 0.0), (1, 0.0)))
            qd = np.asarray(cubic(times, 1), dtype=float)
            qdd = np.asarray(cubic(times, 2), dtype=float)
            qd[0] = qd[-1] = 0.0
            qdd[0] = qdd[-1] = 0.0
            qd[stopped_knots] = 0.0
            qdd[stopped_knots] = 0.0
            derivatives = [[q[index], qd[index], qdd[index]] for index in range(len(times))]
            curves.append(BPoly.from_derivatives(times, derivatives))

        duration_ns = int(time_ns[-1])
        regular_ns = list(range(0, duration_ns, self._nominal_dt_ns))
        regular_ns.append(duration_ns)
        optional_ns = regular_ns
        if self._minimum_optional_spacing_ns:
            # Optional regular-grid extras are not authority knots.  Avoid
            # inventing tiny intervals beside unchanged solver knots, e.g.
            # nominal 0.1 beside float32 solver time 0.100000001.  Never snap,
            # remove, or move an original knot, however close two of them are.
            optional_ns = []
            for candidate_ns in regular_ns:
                right = int(np.searchsorted(time_ns, candidate_ns, side="left"))
                if right < len(time_ns) and candidate_ns == time_ns[right]:
                    continue
                if (
                    right > 0
                    and candidate_ns - int(time_ns[right - 1])
                    < self._minimum_optional_spacing_ns
                ) or (
                    right < len(time_ns)
                    and int(time_ns[right]) - candidate_ns
                    < self._minimum_optional_spacing_ns
                ):
                    continue
                if (
                    optional_ns
                    and candidate_ns - optional_ns[-1] < self._minimum_optional_spacing_ns
                ):
                    continue
                optional_ns.append(candidate_ns)
        sample_time_ns = np.asarray(sorted(set(optional_ns).union(int(v) for v in time_ns)))
        sample_times = sample_time_ns.astype(float) / 1.0e9
        q = np.column_stack([curve(sample_times, 0) for curve in curves])
        qd = np.column_stack([curve(sample_times, 1) for curve in curves])
        qdd = np.column_stack([curve(sample_times, 2) for curve in curves])
        for knot_index in np.flatnonzero(hold_knots):
            hold_samples = (sample_times >= times[knot_index - 1]) & (
                sample_times <= times[knot_index]
            )
            q[hold_samples] = positions[knot_index - 1]
            qd[hold_samples] = 0.0
            qdd[hold_samples] = 0.0

        samples: list[JointSample] = []
        for index, time_s in enumerate(sample_times):
            knot_index = int(np.searchsorted(times, time_s, side="left"))
            knot_index = min(knot_index, len(phases) - 1)
            samples.append(
                JointSample(
                    time_s=float(time_s),
                    positions=tuple(float(value) for value in q[index]),
                    velocities=tuple(float(value) for value in qd[index]),
                    accelerations=tuple(float(value) for value in qdd[index]),
                    phase=phases[knot_index],
                )
            )

        velocity_limits = self._limit_array(self.limits.velocity_rad_s, q.shape[1], "velocity")
        acceleration_limits = self._limit_array(
            self.limits.acceleration_rad_s2, q.shape[1], "acceleration"
        )
        jerk_limits = self._limit_array(self.limits.jerk_rad_s3, q.shape[1], "jerk")
        maximum_velocity = self._maximum_absolute_derivative(curves, 1)
        maximum_acceleration = self._maximum_absolute_derivative(curves, 2)
        maximum_jerk = self._maximum_absolute_derivative(curves, 3)
        ratios = (
            float(np.max(maximum_velocity.values / velocity_limits)),
            float(np.max(maximum_acceleration.values / acceleration_limits)),
            float(np.max(maximum_jerk.values / jerk_limits)),
        )
        return (
            CompiledTrajectory(joint_names=joint_names, samples=tuple(samples)),
            ratios,
            (maximum_velocity, maximum_acceleration, maximum_jerk),
        )

    @staticmethod
    def _maximum_absolute_derivative(
        curves: Sequence[BPoly],
        derivative_order: int,
    ) -> _DerivativeExtrema:
        """Evaluate every polynomial-segment extremum, not only output samples."""
        maxima = np.zeros(len(curves), dtype=float)
        maximum_segments = np.zeros(len(curves), dtype=np.int64)
        maximum_times = np.asarray(
            [float(curve.x[0]) for curve in curves],
            dtype=float,
        )
        for joint_index, curve in enumerate(curves):
            power = PPoly.from_bernstein_basis(curve)
            for interval in range(len(power.x) - 1):
                width = float(power.x[interval + 1] - power.x[interval])
                coefficients = np.polyder(power.c[:, interval], derivative_order)
                candidates = [0.0, width]
                stationary = np.trim_zeros(np.polyder(coefficients), trim="f")
                if len(stationary) > 1:
                    for root in np.roots(stationary):
                        if abs(float(root.imag)) > 1.0e-9:
                            continue
                        value = float(root.real)
                        if -1.0e-12 <= value <= width + 1.0e-12:
                            candidates.append(float(np.clip(value, 0.0, width)))
                values = np.polyval(coefficients, np.asarray(candidates, dtype=float))
                absolute_values = np.abs(values)
                candidate_index = int(np.argmax(absolute_values))
                candidate_maximum = float(absolute_values[candidate_index])
                if candidate_maximum > maxima[joint_index]:
                    maxima[joint_index] = candidate_maximum
                    maximum_segments[joint_index] = interval
                    maximum_times[joint_index] = float(
                        power.x[interval] + candidates[candidate_index]
                    )
        if (
            not np.all(np.isfinite(maxima))
            or not np.all(np.isfinite(maximum_times))
        ):
            raise PlanningError("trajectory derivative extrema are non-finite")
        return _DerivativeExtrema(
            values=maxima,
            segment_indices=maximum_segments,
            times_s=maximum_times,
        )

    @staticmethod
    def _format_derivative_peak(
        name: str,
        unit: str,
        extrema: _DerivativeExtrema,
        limits: np.ndarray,
        joint_names: Sequence[str],
    ) -> str:
        ratios = extrema.values / limits
        joint_index = int(np.argmax(ratios))
        return (
            f"{name}_ratio={ratios[joint_index]:.3f} "
            f"(joint={joint_names[joint_index]}, "
            f"segment={int(extrema.segment_indices[joint_index])}, "
            f"time_s={extrema.times_s[joint_index]:.9f}, "
            f"{name}={extrema.values[joint_index]:.9g} {unit}, "
            f"limit={limits[joint_index]:.9g} {unit})"
        )

    @staticmethod
    def _hold_knots(phases: tuple[str, ...], positions: np.ndarray) -> np.ndarray:
        hold_knots = np.asarray([phase == "hold" for phase in phases], dtype=bool)
        if len(hold_knots) and hold_knots[0]:
            raise PlanningError("hold cannot be the first optimized knot")
        for index in np.flatnonzero(hold_knots):
            if not np.allclose(
                positions[index],
                positions[index - 1],
                rtol=0.0,
                atol=1.0e-12,
            ):
                raise PlanningError(
                    f"hold knot {index} must repeat the previous joint position"
                )
            # Eliminate representational round-off so the corresponding
            # Hermite segment is mathematically constant.
            positions[index] = positions[index - 1]
        return hold_knots

    @staticmethod
    def _limit_array(values: Sequence[float], width: int, name: str) -> np.ndarray:
        result = np.asarray(values, dtype=float)
        if result.shape != (width,) or np.any(result <= 0.0):
            raise PlanningError(f"{name} limits must contain {width} positive values")
        return result
