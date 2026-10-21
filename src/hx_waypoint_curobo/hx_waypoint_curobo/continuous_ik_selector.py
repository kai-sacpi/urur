from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .errors import PlanningError
from .models import IKCandidate


def _joint_delta(current: np.ndarray, previous: np.ndarray) -> np.ndarray:
    """Return the numerical joint motion that a trajectory controller will execute.

    UR joints are bounded revolute joints, not continuous-angle commands.  Treating
    values that differ by 2*pi as identical can select a path whose exported joint
    positions contain a full-turn discontinuity even though its wrapped cost is
    near zero.
    """

    return current - previous


@dataclass(frozen=True)
class SelectionWeights:
    joint_distance: float = 1.0
    velocity: float = 0.1
    acceleration: float = 0.1
    jerk: float = 0.05
    joint_limit: float = 0.1
    collision: float = 1.0
    singularity: float = 0.1


@dataclass(frozen=True)
class SelectionLimits:
    reject_branch_jump_rad: float = 0.25
    minimum_collision_distance_m: float = 0.005
    maximum_velocity_rad_s: tuple[float, ...] | None = None
    beam_width: int = 128


@dataclass
class _State:
    score: float
    positions: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    indices: tuple[int, ...]


class ContinuousIKSelector:
    """Beam-search IK branch selector with dynamic continuity costs."""

    def __init__(
        self,
        weights: SelectionWeights | None = None,
        limits: SelectionLimits | None = None,
    ) -> None:
        self.weights = weights or SelectionWeights()
        self.limits = limits or SelectionLimits()

    def select(
        self,
        times_s: Sequence[float],
        candidate_sets: Sequence[Sequence[IKCandidate]],
        start_positions: Sequence[float] | None = None,
        reference_positions: Sequence[float] | None = None,
    ) -> tuple[IKCandidate, ...]:
        times = np.asarray(times_s, dtype=float)
        if len(times) != len(candidate_sets) or len(times) < 2:
            raise PlanningError("IK selection needs one non-empty candidate set per timed pose")
        if np.any(np.diff(times) <= 0.0):
            raise PlanningError("IK selection time must be strictly increasing")
        if any(not candidates for candidates in candidate_sets):
            failed = next(index for index, candidates in enumerate(candidate_sets) if not candidates)
            raise PlanningError(f"cuRobo returned no IK candidate for pose index {failed}")

        width = len(candidate_sets[0][0].positions)
        zero = np.zeros(width, dtype=float)
        reference = (
            None
            if reference_positions is None
            else np.asarray(reference_positions, dtype=float)
        )
        if reference is not None and (
            reference.shape != (width,) or not np.all(np.isfinite(reference))
        ):
            raise PlanningError("IK selection reference joint positions are invalid")
        states: list[_State] = []
        first_pose_clearance_eligible = 0
        for candidate_index, candidate in enumerate(candidate_sets[0]):
            if candidate.collision_distance_m < self.limits.minimum_collision_distance_m:
                continue
            first_pose_clearance_eligible += 1
            current = np.asarray(candidate.positions, dtype=float)
            if current.shape != (width,):
                raise PlanningError("inconsistent IK candidate joint width")
            if start_positions is None:
                distance = zero if reference is None else _joint_delta(current, reference)
            else:
                distance = _joint_delta(
                    current,
                    np.asarray(start_positions, dtype=float),
                )
                if (
                    np.max(np.abs(distance), initial=0.0)
                    > self.limits.reject_branch_jump_rad
                ):
                    continue
            score = self._static_cost(candidate) + self.weights.joint_distance * float(
                np.dot(distance, distance)
            )
            states.append(
                _State(
                    score=score,
                    positions=current,
                    velocity=zero.copy(),
                    acceleration=zero.copy(),
                    indices=(candidate_index,),
                )
            )
        if not states:
            if first_pose_clearance_eligible == 0:
                maximum_clearance = max(
                    candidate.collision_distance_m for candidate in candidate_sets[0]
                )
                raise PlanningError(
                    "all first-pose IK candidates violate the minimum collision "
                    f"clearance: maximum={maximum_clearance:.9g} m, "
                    f"required={self.limits.minimum_collision_distance_m:.9g} m"
                )
            raise PlanningError("all first-pose IK candidates violate the branch-jump limit")

        for pose_index in range(1, len(candidate_sets)):
            dt = float(times[pose_index] - times[pose_index - 1])
            clearance_eligible = tuple(
                candidate
                for candidate in candidate_sets[pose_index]
                if candidate.collision_distance_m
                >= self.limits.minimum_collision_distance_m
            )
            if not clearance_eligible:
                maximum_clearance = max(
                    candidate.collision_distance_m
                    for candidate in candidate_sets[pose_index]
                )
                raise PlanningError(
                    f"all IK candidates at pose index {pose_index} violate the minimum "
                    f"collision clearance: maximum={maximum_clearance:.9g} m, "
                    f"required={self.limits.minimum_collision_distance_m:.9g} m"
                )
            expanded: list[_State] = []
            for state in states:
                for candidate_index, candidate in enumerate(candidate_sets[pose_index]):
                    if candidate.collision_distance_m < self.limits.minimum_collision_distance_m:
                        continue
                    current = np.asarray(candidate.positions, dtype=float)
                    if current.shape != state.positions.shape:
                        raise PlanningError("inconsistent IK candidate joint width")
                    delta = _joint_delta(current, state.positions)
                    if np.max(np.abs(delta), initial=0.0) > self.limits.reject_branch_jump_rad:
                        continue
                    velocity = delta / dt
                    if self.limits.maximum_velocity_rad_s is not None:
                        limits = np.asarray(self.limits.maximum_velocity_rad_s, dtype=float)
                        if limits.shape != velocity.shape or np.any(np.abs(velocity) > limits):
                            continue
                    acceleration = (velocity - state.velocity) / dt
                    jerk = (acceleration - state.acceleration) / dt
                    dynamic_cost = (
                        self.weights.joint_distance * float(np.dot(delta, delta))
                        + self.weights.velocity * float(np.dot(velocity, velocity))
                        + self.weights.acceleration * float(np.dot(acceleration, acceleration))
                        + self.weights.jerk * float(np.dot(jerk, jerk))
                    )
                    expanded.append(
                        _State(
                            score=state.score + dynamic_cost + self._static_cost(candidate),
                            positions=current,
                            velocity=velocity,
                            acceleration=acceleration,
                            indices=(*state.indices, candidate_index),
                        )
                    )
            if not expanded:
                raise PlanningError(
                    f"no continuous IK branch reaches pose index {pose_index} within hard limits"
                )
            states = self._prune_diverse_frontier(expanded)

        best = min(states, key=lambda state: (state.score, state.indices))
        return tuple(
            candidate_sets[index][candidate_index]
            for index, candidate_index in enumerate(best.indices)
        )

    def _prune_diverse_frontier(self, expanded: Sequence[_State]) -> list[_State]:
        """Keep low-cost histories without collapsing onto one IK endpoint.

        A global top-k beam can be filled by many nearly identical histories
        that end at the same current-pose candidate.  That silently removes
        other feasible IK branches before a later waypoint needs them.  Select
        states round-robin by current endpoint, then by score, so every
        reachable endpoint gets one history before any endpoint gets a second.
        """

        width = max(1, self.limits.beam_width)
        ordered = sorted(expanded, key=lambda state: (state.score, state.indices))
        if len(ordered) <= width:
            return ordered

        by_endpoint: dict[int, list[_State]] = {}
        for state in ordered:
            by_endpoint.setdefault(state.indices[-1], []).append(state)

        selected: list[_State] = []
        depth = 0
        while len(selected) < width:
            layer = [
                histories[depth]
                for histories in by_endpoint.values()
                if depth < len(histories)
            ]
            if not layer:
                break
            layer.sort(key=lambda state: (state.score, state.indices))
            selected.extend(layer[: width - len(selected)])
            depth += 1
        return selected

    def _static_cost(self, candidate: IKCandidate) -> float:
        collision_cost = 1.0 / max(candidate.collision_distance_m, 1.0e-6)
        limit_cost = 1.0 / max(candidate.joint_limit_margin_rad, 1.0e-6)
        return (
            self.weights.collision * collision_cost
            + self.weights.joint_limit * limit_cost
            + self.weights.singularity * max(0.0, candidate.singularity_score)
        )
