from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import numpy as np

from .environment_builder import EnvironmentBundle
from .errors import PlanningError, WaypointError
from .robot_tool_builder import RobotToolBuilder


_SPHERE_GEOMETRY_TOLERANCE_M = 5.0e-6
_UNIT_NORMAL_TOLERANCE = 1.0e-9
_NUMERIC_DISTANCE_TOLERANCE_M = 1.0e-12


@dataclass(frozen=True)
class ContactSphereClassification:
    """Stable mapping from cuRobo's flattened sphere tensor to tool geometry."""

    total_sphere_count: int
    tip_sphere_index: int
    rod_sphere_indices: tuple[int, ...]

    @property
    def non_tip_sphere_indices(self) -> tuple[int, ...]:
        return tuple(
            index
            for index in range(self.total_sphere_count)
            if index != self.tip_sphere_index
        )


@dataclass(frozen=True)
class ContactValidationMetrics:
    checked_sample_count: int
    maximum_tip_penetration_m: float
    minimum_non_tip_clearance_m: float


@dataclass(frozen=True)
class ContactPolicy:
    """Fail-closed contact-surface contract in the robot base frame.

    The allowed contact is the mathematical TCP/rod-tip point.  cuRobo's rod
    approximation is attached to ``tool0`` as a sphere chain, so exactly one
    terminal sphere is identified from the configured tool0-to-tip transform
    and rod geometry.  Every other collision sphere remains non-contact
    geometry and must keep the configured signed clearance from the surface.
    A surface is infinite when no XY ranges are configured; otherwise the
    analytical gate applies only where a point or sphere projection overlaps
    the configured finite rectangle in the surface frame.
    """

    allowed_contact_link: str
    allowed_surface: str
    plane_normal_base: tuple[float, float, float]
    plane_offset_base_m: float
    plane_origin_base_m: tuple[float, float, float]
    plane_x_axis_base: tuple[float, float, float]
    plane_y_axis_base: tuple[float, float, float]
    surface_x_range_m: tuple[float, float] | None
    surface_y_range_m: tuple[float, float] | None
    maximum_tip_penetration_m: float
    minimum_non_tip_clearance_m: float
    tool_parent_link: str
    tip_translation_tool_m: tuple[float, float, float]
    expected_rod_spheres_tool: tuple[tuple[float, float, float, float], ...]
    terminal_rod_sphere_offset: int

    @classmethod
    def from_environment(cls, environment: EnvironmentBundle) -> ContactPolicy:
        zones = _mapping(environment.zones, "zones.yaml")
        contact = _mapping(zones.get("contact_policy"), "contact_policy")
        allowed_contact_link = _nonempty_string(
            contact.get("allowed_contact_link"),
            "contact_policy.allowed_contact_link",
        )
        allowed_surface = _nonempty_string(
            contact.get("allowed_surface"),
            "contact_policy.allowed_surface",
        )
        maximum_tip_penetration_m = _finite_scalar(
            contact.get("maximum_tip_penetration_m"),
            "contact_policy.maximum_tip_penetration_m",
            minimum=0.0,
            strict_minimum=False,
        )
        minimum_non_tip_clearance_m = _finite_scalar(
            contact.get("minimum_non_tip_clearance_m"),
            "contact_policy.minimum_non_tip_clearance_m",
            minimum=0.0,
            strict_minimum=True,
        )

        tool_builder = RobotToolBuilder(environment)
        try:
            tip_frame = tool_builder.tip_frame
            parent_link = tool_builder.parent_link
            tool_to_tip = np.asarray(tool_builder.tool0_to_tip, dtype=float)
            generated_spheres = tool_builder.collision_spheres()
        except (KeyError, TypeError, ValueError, WaypointError) as error:
            raise PlanningError(
                "contact policy cannot resolve the configured TCP/rod geometry"
            ) from error
        if allowed_contact_link != tip_frame:
            raise PlanningError(
                "contact_policy.allowed_contact_link must exactly match the configured "
                f"rod tip frame {tip_frame!r}"
            )
        if (
            tool_to_tip.shape != (4, 4)
            or not np.all(np.isfinite(tool_to_tip))
            or not np.allclose(tool_to_tip[3], (0.0, 0.0, 0.0, 1.0), atol=1.0e-12)
        ):
            raise PlanningError("contact policy received an invalid tool0-to-tip transform")
        tip_translation = tool_to_tip[:3, 3]

        expected_spheres = np.asarray(
            [(*sphere.center_m, sphere.radius_m) for sphere in generated_spheres],
            dtype=float,
        )
        if (
            expected_spheres.ndim != 2
            or expected_spheres.shape[0] < 2
            or expected_spheres.shape[1] != 4
            or not np.all(np.isfinite(expected_spheres))
            or np.any(expected_spheres[:, 3] <= 0.0)
            or any(sphere.link_name != parent_link for sphere in generated_spheres)
        ):
            raise PlanningError("contact policy received invalid generated rod spheres")
        tip_surface_error = (
            np.linalg.norm(expected_spheres[:, :3] - tip_translation, axis=1)
            - expected_spheres[:, 3]
        )
        terminal_offsets = np.flatnonzero(
            np.abs(tip_surface_error) <= _SPHERE_GEOMETRY_TOLERANCE_M
        )
        if len(terminal_offsets) != 1:
            raise PlanningError(
                "rod sphere chain must contain exactly one terminal sphere whose surface "
                "coincides with the configured rod tip"
            )

        analysis_zones = _mapping(zones.get("analysis_zones"), "analysis_zones")
        surface = _mapping(
            analysis_zones.get(allowed_surface),
            f"analysis_zones.{allowed_surface}",
        )
        frame_id = _nonempty_string(
            surface.get("frame_id"),
            f"analysis_zones.{allowed_surface}.frame_id",
        )
        normal_local = _finite_vector3(
            surface.get("normal_xyz"),
            f"analysis_zones.{allowed_surface}.normal_xyz",
        )
        normal_norm = float(np.linalg.norm(normal_local))
        if abs(normal_norm - 1.0) > _UNIT_NORMAL_TOLERANCE:
            raise PlanningError(
                f"analysis_zones.{allowed_surface}.normal_xyz must be a unit vector"
            )
        plane_offset_local = _finite_scalar(
            surface.get("offset_m"),
            f"analysis_zones.{allowed_surface}.offset_m",
        )
        surface_x_range = _optional_finite_range(
            surface.get("x_range_m"),
            f"analysis_zones.{allowed_surface}.x_range_m",
        )
        surface_y_range = _optional_finite_range(
            surface.get("y_range_m"),
            f"analysis_zones.{allowed_surface}.y_range_m",
        )
        if (surface_x_range is None) != (surface_y_range is None):
            raise PlanningError(
                f"analysis_zones.{allowed_surface} must define both x_range_m and "
                "y_range_m, or neither"
            )
        try:
            base_from_plane = np.asarray(environment._frame_to_base(frame_id), dtype=float)
        except (KeyError, TypeError, ValueError, WaypointError) as error:
            raise PlanningError(
                f"contact policy cannot resolve plane frame {frame_id!r} to the robot base"
            ) from error
        if (
            base_from_plane.shape != (4, 4)
            or not np.all(np.isfinite(base_from_plane))
            or not np.allclose(
                base_from_plane[:3, :3].T @ base_from_plane[:3, :3],
                np.eye(3),
                atol=1.0e-9,
            )
        ):
            raise PlanningError("contact policy plane transform is invalid")
        normal_base = base_from_plane[:3, :3] @ normal_local
        normal_base_norm = float(np.linalg.norm(normal_base))
        if (
            not np.isfinite(normal_base_norm)
            or abs(normal_base_norm - 1.0) > _UNIT_NORMAL_TOLERANCE
        ):
            raise PlanningError("contact policy base-frame plane normal is invalid")
        normal_base = normal_base / normal_base_norm
        plane_point_base = (
            base_from_plane[:3, 3]
            + base_from_plane[:3, :3] @ (normal_local * plane_offset_local)
        )
        plane_offset_base = float(np.dot(normal_base, plane_point_base))
        if not np.all(np.isfinite(normal_base)) or not np.isfinite(plane_offset_base):
            raise PlanningError("contact policy plane geometry is non-finite")

        return cls(
            allowed_contact_link=allowed_contact_link,
            allowed_surface=allowed_surface,
            plane_normal_base=tuple(float(value) for value in normal_base),
            plane_offset_base_m=plane_offset_base,
            plane_origin_base_m=tuple(
                float(value) for value in base_from_plane[:3, 3]
            ),
            plane_x_axis_base=tuple(
                float(value) for value in base_from_plane[:3, 0]
            ),
            plane_y_axis_base=tuple(
                float(value) for value in base_from_plane[:3, 1]
            ),
            surface_x_range_m=surface_x_range,
            surface_y_range_m=surface_y_range,
            maximum_tip_penetration_m=maximum_tip_penetration_m,
            minimum_non_tip_clearance_m=minimum_non_tip_clearance_m,
            tool_parent_link=parent_link,
            tip_translation_tool_m=tuple(float(value) for value in tip_translation),
            expected_rod_spheres_tool=tuple(
                tuple(float(value) for value in row) for row in expected_spheres
            ),
            terminal_rod_sphere_offset=int(terminal_offsets[0]),
        )

    def classify_curobo_spheres(
        self,
        *,
        total_sphere_count: int,
        tool_sphere_indices: Sequence[int],
        tool_reference_spheres: Sequence[Sequence[float]],
    ) -> ContactSphereClassification:
        """Match configured rod spheres to cuRobo metadata without assuming order."""

        if (
            isinstance(total_sphere_count, bool)
            or not isinstance(total_sphere_count, (int, np.integer))
            or int(total_sphere_count) <= 0
        ):
            raise PlanningError("cuRobo contact geometry has an invalid total sphere count")
        total_count = int(total_sphere_count)
        try:
            raw_indices = np.asarray(tool_sphere_indices)
            indices = raw_indices.astype(np.int64)
            reference = np.asarray(tool_reference_spheres, dtype=float)
        except (TypeError, ValueError) as error:
            raise PlanningError("cuRobo contact sphere metadata is invalid") from error
        if (
            raw_indices.ndim != 1
            or raw_indices.shape != indices.shape
            or not np.all(raw_indices == indices)
            or len(indices) == 0
            or len(set(int(value) for value in indices)) != len(indices)
            or np.any(indices < 0)
            or np.any(indices >= total_count)
        ):
            raise PlanningError("cuRobo tool sphere indices are invalid")
        if (
            reference.shape != (len(indices), 4)
            or not np.all(np.isfinite(reference))
            or np.any(reference[:, 3] <= 0.0)
        ):
            raise PlanningError("cuRobo tool reference sphere geometry is invalid")

        expected = np.asarray(self.expected_rod_spheres_tool, dtype=float)
        matched_global_indices: list[int] = []
        used_local_indices: set[int] = set()
        for expected_offset, expected_sphere in enumerate(expected):
            matches = np.flatnonzero(
                np.all(
                    np.abs(reference - expected_sphere)
                    <= _SPHERE_GEOMETRY_TOLERANCE_M,
                    axis=1,
                )
            )
            matches = np.asarray(
                [value for value in matches if int(value) not in used_local_indices],
                dtype=np.int64,
            )
            if len(matches) != 1:
                raise PlanningError(
                    "cuRobo tool spheres do not contain one unique match for configured "
                    f"rod sphere {expected_offset}"
                )
            local_index = int(matches[0])
            used_local_indices.add(local_index)
            matched_global_indices.append(int(indices[local_index]))

        tip_sphere_index = matched_global_indices[self.terminal_rod_sphere_offset]
        return ContactSphereClassification(
            total_sphere_count=total_count,
            tip_sphere_index=tip_sphere_index,
            rod_sphere_indices=tuple(matched_global_indices),
        )

    def compliant_sample_mask(
        self,
        tip_positions_base_m: Sequence[Sequence[float]],
        robot_spheres_base_m: Sequence[Sequence[Sequence[float]]],
        classification: ContactSphereClassification,
    ) -> np.ndarray:
        evaluation = self._evaluate_samples(
            tip_positions_base_m,
            robot_spheres_base_m,
            classification,
        )
        return (
            (
                evaluation[0]
                <= self.maximum_tip_penetration_m
                + _NUMERIC_DISTANCE_TOLERANCE_M
            )
            & (
                evaluation[1]
                <= self.maximum_tip_penetration_m
                + _NUMERIC_DISTANCE_TOLERANCE_M
            )
            & (
                evaluation[2]
                >= self.minimum_non_tip_clearance_m
                - _NUMERIC_DISTANCE_TOLERANCE_M
            )
            & (evaluation[3] <= _SPHERE_GEOMETRY_TOLERANCE_M)
            & (evaluation[4] <= _SPHERE_GEOMETRY_TOLERANCE_M)
        )

    def validate_canonical_samples(
        self,
        tip_positions_base_m: Sequence[Sequence[float]],
        robot_spheres_base_m: Sequence[Sequence[Sequence[float]]],
        classification: ContactSphereClassification,
    ) -> ContactValidationMetrics:
        (
            tip_penetration,
            terminal_surface_penetration,
            minimum_non_tip_clearance,
            tip_surface_distance_error,
            tip_plane_alignment_error,
            non_tip_minimum_indices,
        ) = self._evaluate_samples(
            tip_positions_base_m,
            robot_spheres_base_m,
            classification,
        )

        invalid_tip_geometry = np.flatnonzero(
            (tip_surface_distance_error > _SPHERE_GEOMETRY_TOLERANCE_M)
            | (tip_plane_alignment_error > _SPHERE_GEOMETRY_TOLERANCE_M)
        )
        if len(invalid_tip_geometry):
            sample_index = int(invalid_tip_geometry[0])
            raise PlanningError(
                "canonical contact validation cannot prove that only the configured rod "
                f"tip touches {self.allowed_surface!r} at sample {sample_index}"
            )
        excessive_penetration = np.flatnonzero(
            (
                tip_penetration
                > self.maximum_tip_penetration_m
                + _NUMERIC_DISTANCE_TOLERANCE_M
            )
            | (
                terminal_surface_penetration
                > self.maximum_tip_penetration_m
                + _NUMERIC_DISTANCE_TOLERANCE_M
            )
        )
        if len(excessive_penetration):
            sample_index = int(excessive_penetration[0])
            measured = max(
                float(tip_penetration[sample_index]),
                float(terminal_surface_penetration[sample_index]),
            )
            raise PlanningError(
                "canonical rod-tip penetration exceeds contact policy at sample "
                f"{sample_index}: {measured:.9g} > "
                f"{self.maximum_tip_penetration_m:.9g} m"
            )
        insufficient_clearance = np.flatnonzero(
            minimum_non_tip_clearance
            < self.minimum_non_tip_clearance_m
            - _NUMERIC_DISTANCE_TOLERANCE_M
        )
        if len(insufficient_clearance):
            sample_index = int(insufficient_clearance[0])
            sphere_index = int(non_tip_minimum_indices[sample_index])
            geometry = (
                "non-tip rod sphere"
                if sphere_index in classification.rod_sphere_indices
                else "robot sphere"
            )
            raise PlanningError(
                f"canonical {geometry} violates drawing-plane clearance at sample "
                f"{sample_index}, sphere {sphere_index}: "
                f"{minimum_non_tip_clearance[sample_index]:.9g} < "
                f"{self.minimum_non_tip_clearance_m:.9g} m"
            )

        return ContactValidationMetrics(
            checked_sample_count=len(tip_penetration),
            maximum_tip_penetration_m=max(
                float(np.max(tip_penetration)),
                float(np.max(terminal_surface_penetration)),
            ),
            minimum_non_tip_clearance_m=float(np.min(minimum_non_tip_clearance)),
        )

    def _evaluate_samples(
        self,
        tip_positions_base_m: Sequence[Sequence[float]],
        robot_spheres_base_m: Sequence[Sequence[Sequence[float]]],
        classification: ContactSphereClassification,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        try:
            tips = np.asarray(tip_positions_base_m, dtype=float)
            spheres = np.asarray(robot_spheres_base_m, dtype=float)
        except (TypeError, ValueError) as error:
            raise PlanningError("canonical contact geometry is not numeric") from error
        if tips.ndim != 2 or tips.shape[1:] != (3,) or len(tips) == 0:
            raise PlanningError("canonical rod-tip geometry must have shape N x 3")
        if (
            spheres.ndim != 3
            or spheres.shape[0] != len(tips)
            or spheres.shape[2] != 4
            or spheres.shape[1] != classification.total_sphere_count
        ):
            raise PlanningError("canonical collision sphere geometry must have shape N x S x 4")
        if not np.all(np.isfinite(tips)) or not np.all(np.isfinite(spheres)):
            raise PlanningError("canonical contact geometry contains a non-finite value")
        if np.any(spheres[:, :, 3] <= 0.0):
            raise PlanningError("canonical contact geometry contains a non-positive sphere radius")
        tip_index = classification.tip_sphere_index
        if not 0 <= tip_index < spheres.shape[1]:
            raise PlanningError("canonical contact geometry has an invalid tip sphere index")
        non_tip_indices = np.asarray(classification.non_tip_sphere_indices, dtype=np.int64)
        if len(non_tip_indices) == 0:
            raise PlanningError("canonical contact geometry contains no non-tip spheres")

        normal = np.asarray(self.plane_normal_base, dtype=float)
        tip_signed_distance = tips @ normal - self.plane_offset_base_m
        terminal = spheres[:, tip_index, :]
        terminal_center_signed_distance = (
            terminal[:, :3] @ normal - self.plane_offset_base_m
        )
        terminal_surface_signed_distance = (
            terminal_center_signed_distance - terminal[:, 3]
        )
        tip_surface_active = self._points_overlap_surface(tips)
        terminal_surface_active = self._spheres_overlap_surface(terminal)
        tip_penetration = np.where(
            tip_surface_active,
            np.maximum(0.0, -tip_signed_distance),
            0.0,
        )
        terminal_surface_penetration = np.where(
            terminal_surface_active,
            np.maximum(0.0, -terminal_surface_signed_distance),
            0.0,
        )

        expected_terminal_radius = self.expected_rod_spheres_tool[
            self.terminal_rod_sphere_offset
        ][3]
        radius_error = np.abs(terminal[:, 3] - expected_terminal_radius)
        tip_surface_distance_error = np.maximum(
            np.abs(np.linalg.norm(tips - terminal[:, :3], axis=1) - terminal[:, 3]),
            radius_error,
        )
        # The terminal sphere's plane-facing surface must coincide with the
        # mathematical rod tip only while either one is in the contact
        # neighbourhood.  Away from the surface a tilted rod legitimately has
        # a different closest point on the terminal sphere.  Penetration and
        # every non-tip clearance remain enforced independently below.
        contact_alignment_active = (
            tip_surface_active
            & terminal_surface_active
            & (
                (tip_signed_distance <= _SPHERE_GEOMETRY_TOLERANCE_M)
                | (
                    terminal_surface_signed_distance
                    <= _SPHERE_GEOMETRY_TOLERANCE_M
                )
            )
        )
        tip_plane_alignment_error = np.where(
            contact_alignment_active,
            np.abs(terminal_surface_signed_distance - tip_signed_distance),
            0.0,
        )

        non_tip = spheres[:, non_tip_indices, :]
        clearances = (
            non_tip[:, :, :3] @ normal
            - self.plane_offset_base_m
            - non_tip[:, :, 3]
        )
        active_non_tip = self._spheres_overlap_surface(non_tip)
        masked_clearances = np.where(active_non_tip, clearances, np.inf)
        minimum_offsets = np.argmin(masked_clearances, axis=1)
        active_samples = np.any(active_non_tip, axis=1)
        minimum_clearance = masked_clearances[
            np.arange(len(tips)), minimum_offsets
        ]
        minimum_clearance = np.where(
            active_samples,
            minimum_clearance,
            self.minimum_non_tip_clearance_m,
        )
        minimum_global_indices = np.where(
            active_samples,
            non_tip_indices[minimum_offsets],
            -1,
        )
        return (
            tip_penetration,
            terminal_surface_penetration,
            minimum_clearance,
            tip_surface_distance_error,
            tip_plane_alignment_error,
            minimum_global_indices,
        )

    def _points_overlap_surface(self, points_base_m: np.ndarray) -> np.ndarray:
        if self.surface_x_range_m is None or self.surface_y_range_m is None:
            return np.ones(points_base_m.shape[:-1], dtype=bool)
        local_x, local_y = self._surface_local_xy(points_base_m)
        return (
            (local_x >= self.surface_x_range_m[0] - _NUMERIC_DISTANCE_TOLERANCE_M)
            & (local_x <= self.surface_x_range_m[1] + _NUMERIC_DISTANCE_TOLERANCE_M)
            & (local_y >= self.surface_y_range_m[0] - _NUMERIC_DISTANCE_TOLERANCE_M)
            & (local_y <= self.surface_y_range_m[1] + _NUMERIC_DISTANCE_TOLERANCE_M)
        )

    def _spheres_overlap_surface(self, spheres_base_m: np.ndarray) -> np.ndarray:
        if self.surface_x_range_m is None or self.surface_y_range_m is None:
            return np.ones(spheres_base_m.shape[:-1], dtype=bool)
        local_x, local_y = self._surface_local_xy(spheres_base_m[..., :3])
        delta_x = np.maximum.reduce(
            (
                self.surface_x_range_m[0] - local_x,
                np.zeros_like(local_x),
                local_x - self.surface_x_range_m[1],
            )
        )
        delta_y = np.maximum.reduce(
            (
                self.surface_y_range_m[0] - local_y,
                np.zeros_like(local_y),
                local_y - self.surface_y_range_m[1],
            )
        )
        lateral_distance = np.hypot(delta_x, delta_y)
        return lateral_distance <= (
            spheres_base_m[..., 3] + _NUMERIC_DISTANCE_TOLERANCE_M
        )

    def _surface_local_xy(
        self,
        points_base_m: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        delta = points_base_m - np.asarray(self.plane_origin_base_m, dtype=float)
        local_x = delta @ np.asarray(self.plane_x_axis_base, dtype=float)
        local_y = delta @ np.asarray(self.plane_y_axis_base, dtype=float)
        return local_x, local_y


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlanningError(f"zones.yaml {path} must be a mapping")
    return value


def _nonempty_string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value or value.strip() != value:
        raise PlanningError(f"zones.yaml {path} must be a non-empty trimmed string")
    return value


def _finite_scalar(
    value: Any,
    path: str,
    *,
    minimum: float | None = None,
    strict_minimum: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, np.number)):
        raise PlanningError(f"zones.yaml {path} must be a finite number")
    result = float(value)
    if not np.isfinite(result):
        raise PlanningError(f"zones.yaml {path} must be a finite number")
    if minimum is not None and (
        result <= minimum if strict_minimum else result < minimum
    ):
        qualifier = "greater than" if strict_minimum else "at least"
        raise PlanningError(f"zones.yaml {path} must be {qualifier} {minimum}")
    return result


def _finite_vector3(value: Any, path: str) -> np.ndarray:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise PlanningError(f"zones.yaml {path} must contain three finite numbers")
    if any(isinstance(item, bool) for item in value):
        raise PlanningError(f"zones.yaml {path} must contain three finite numbers")
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise PlanningError(
            f"zones.yaml {path} must contain three finite numbers"
        ) from error
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise PlanningError(f"zones.yaml {path} must contain three finite numbers")
    return result


def _optional_finite_range(
    value: Any,
    path: str,
) -> tuple[float, float] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise PlanningError(f"zones.yaml {path} must contain two finite bounds")
    if any(isinstance(item, bool) for item in value):
        raise PlanningError(f"zones.yaml {path} must contain two finite bounds")
    try:
        lower, upper = (float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise PlanningError(
            f"zones.yaml {path} must contain two finite bounds"
        ) from error
    if not np.isfinite(lower) or not np.isfinite(upper) or lower >= upper:
        raise PlanningError(
            f"zones.yaml {path} must contain increasing finite bounds"
        )
    return lower, upper
