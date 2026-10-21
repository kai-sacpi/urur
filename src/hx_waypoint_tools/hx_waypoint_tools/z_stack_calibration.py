from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from io import StringIO
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.math3d import quaternion_to_matrix_xyzw, slerp_xyzw
from hx_waypoint_curobo.models import TimedPose

from .trajectory_generator import minimum_jerk_progress
from .yaw_calibration import write_authoritative_trajectory


Z_STACK_ENVIRONMENT_ID = "new_board_4cm_rod33365_up50_icassp_phase2"
Z_STACK_SENSOR_PLANE_Z_M = -0.007
Z_STACK_COLLISION_FACE_Z_M = -0.067
Z_STACK_CLEARANCE_BOUNDARY_Z_M = -0.072
Z_STACK_MINIMUM_TIP_DISTANCE_M = 0.065
Z_STACK_DISTANCES_M = (0.068, 0.076, 0.084, 0.092, 0.084, 0.076, 0.068)
Z_STACK_TILT_DEG = 25.0
Z_STACK_TRANSLATION_DURATION_S = 1.5
Z_STACK_TILT_DURATION_S = 4.5
Z_STACK_SAMPLE_PERIOD_S = 0.05
Z_STACK_SETTLE_DURATION_S = 1.0
Z_STACK_CAPTURE_DURATION_S = 3.0
Z_STACK_BASE_QUATERNION_XYZW = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class ZStackSite:
    site_id: str
    trajectory_id: str
    x_m: float
    y_m: float


@dataclass(frozen=True)
class ZStackVisit:
    visit_index: int
    visit_id: str
    stage: str
    distance_m: float
    tip_position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    arrival_time_s: float
    settle_end_time_s: float
    capture_start_time_s: float
    capture_end_time_s: float
    fit_role: str


@dataclass(frozen=True)
class ZStackExperiment:
    site: ZStackSite
    poses: tuple[TimedPose, ...]
    visits: tuple[ZStackVisit, ...]


@dataclass(frozen=True)
class RigidOffsetFit:
    board_bias_m: tuple[float, float, float]
    tool_offset_m: tuple[float, float, float]
    rmse_m: float
    maximum_residual_m: float
    rank: int
    condition_number: float


Z_STACK_SITES = (
    ZStackSite(
        site_id="center",
        trajectory_id="z_stack_tilt_calibration_center_up50_phase2_v3",
        x_m=0.0,
        y_m=0.0,
    ),
    ZStackSite(
        site_id="x50",
        trajectory_id="z_stack_tilt_calibration_x50_up50_phase2_v3",
        x_m=0.050,
        y_m=0.0,
    ),
)

Z_STACK_CAPTURE_WINDOW_COLUMNS = (
    "environment_id",
    "trajectory_id",
    "site_id",
    "visit_index",
    "visit_id",
    "stage",
    "distance_m",
    "tip_x_m",
    "tip_y_m",
    "tip_z_m",
    "tip_qx",
    "tip_qy",
    "tip_qz",
    "tip_qw",
    "arrival_time_s",
    "settle_end_time_s",
    "capture_start_time_s",
    "capture_end_time_s",
    "fit_role",
)


@dataclass(frozen=True)
class _VisitTarget:
    visit_id: str
    stage: str
    distance_m: float
    quaternion_xyzw: tuple[float, float, float, float]
    fit_role: str


def _axis_angle_quaternion_xyzw(
    axis_xyz: Sequence[float],
    angle_deg: float,
) -> tuple[float, float, float, float]:
    axis = np.asarray(tuple(float(value) for value in axis_xyz), dtype=float)
    if axis.shape != (3,) or not np.all(np.isfinite(axis)):
        raise ValueError("tilt axis must contain three finite values")
    norm = float(np.linalg.norm(axis))
    if norm <= 1.0e-12:
        raise ValueError("tilt axis must be non-zero")
    axis /= norm
    angle = math.radians(float(angle_deg))
    if not math.isfinite(angle):
        raise ValueError("tilt angle must be finite")
    sine = math.sin(0.5 * angle)
    quaternion = (*tuple(float(value * sine) for value in axis), math.cos(0.5 * angle))
    return quaternion  # type: ignore[return-value]


def _visit_targets(tilt_deg: float) -> tuple[_VisitTarget, ...]:
    normal = Z_STACK_BASE_QUATERNION_XYZW
    tilt_label = int(round(tilt_deg))
    tilt_x_positive = _axis_angle_quaternion_xyzw((1.0, 0.0, 0.0), tilt_deg)
    tilt_x_negative = _axis_angle_quaternion_xyzw((1.0, 0.0, 0.0), -tilt_deg)
    tilt_y_positive = _axis_angle_quaternion_xyzw((0.0, 1.0, 0.0), tilt_deg)
    tilt_y_negative = _axis_angle_quaternion_xyzw((0.0, 1.0, 0.0), -tilt_deg)
    distance_68, distance_76, distance_84, deepest, _, _, _ = Z_STACK_DISTANCES_M
    return (
        _VisitTarget("z068_out", "z_stack_out", distance_68, normal, "z_stack"),
        _VisitTarget("z076_out", "z_stack_out", distance_76, normal, "z_stack"),
        _VisitTarget("z084_out", "z_stack_out", distance_84, normal, "z_stack"),
        _VisitTarget("z092_normal", "z_stack_out", deepest, normal, "z_stack_and_rigid"),
        _VisitTarget(
            f"z092_rx_p{tilt_label}",
            "tilt",
            deepest,
            tilt_x_positive,
            "rigid",
        ),
        _VisitTarget("z092_normal_1", "tilt_return", deepest, normal, "rigid_drift"),
        _VisitTarget(
            f"z092_rx_n{tilt_label}",
            "tilt",
            deepest,
            tilt_x_negative,
            "rigid",
        ),
        _VisitTarget("z092_normal_2", "tilt_return", deepest, normal, "rigid_drift"),
        _VisitTarget(
            f"z092_ry_p{tilt_label}",
            "tilt",
            deepest,
            tilt_y_positive,
            "rigid",
        ),
        _VisitTarget("z092_normal_3", "tilt_return", deepest, normal, "rigid_drift"),
        _VisitTarget(
            f"z092_ry_n{tilt_label}",
            "tilt",
            deepest,
            tilt_y_negative,
            "rigid",
        ),
        _VisitTarget("z092_normal_4", "tilt_return", deepest, normal, "rigid_drift"),
        _VisitTarget(
            "z084_return", "z_stack_return", distance_84, normal, "z_stack"
        ),
        _VisitTarget(
            "z076_return", "z_stack_return", distance_76, normal, "z_stack"
        ),
        _VisitTarget(
            "z068_return", "z_stack_return", distance_68, normal, "z_stack"
        ),
    )


def _validated_seconds(value: float, label: str) -> int:
    seconds = float(value)
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    nanoseconds = int(round(seconds * 1_000_000_000.0))
    if not math.isclose(
        nanoseconds / 1_000_000_000.0,
        seconds,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError(f"{label} must be exactly representable in ROS nanoseconds")
    return nanoseconds


def validate_z_stack_tip_distances(distances_m: Sequence[float]) -> tuple[float, ...]:
    """Reject any requested tip distance at or inside the 65 mm boundary."""

    distances = tuple(float(value) for value in distances_m)
    if not distances or not all(math.isfinite(value) for value in distances):
        raise ValueError("Z-stack distances must contain finite values")
    if any(value <= Z_STACK_MINIMUM_TIP_DISTANCE_M for value in distances):
        raise ValueError("every Z-stack target must remain beyond the 65 mm hard boundary")
    return distances


def _validate_environment(environment_path: str | Path) -> tuple[Path, str]:
    root = Path(environment_path).expanduser().resolve()
    environment = EnvironmentBuilder().load(root)
    if environment.environment_id != Z_STACK_ENVIRONMENT_ID:
        raise ValueError(
            f"Z-stack calibration requires environment {Z_STACK_ENVIRONMENT_ID!r}"
        )
    sensor = environment.environment["calibration"]["sensor_geometry"]
    sensor_z = float(sensor["actual_sensor_plane_local_z_m"])
    keepout_min = float(sensor["keepout_local_z_min_m"])
    boundary = environment.zones["analysis_zones"][
        "sensor_keepout_clearance_boundary"
    ]
    boundary_z = float(boundary["normal_xyz"][2]) * float(boundary["offset_m"])
    minimum_distance = sensor_z - boundary_z
    expected = (
        sensor_z,
        keepout_min,
        boundary_z,
        minimum_distance,
    )
    authority = (
        Z_STACK_SENSOR_PLANE_Z_M,
        Z_STACK_COLLISION_FACE_Z_M,
        Z_STACK_CLEARANCE_BOUNDARY_Z_M,
        Z_STACK_MINIMUM_TIP_DISTANCE_M,
    )
    if any(
        not math.isclose(actual, required, rel_tol=0.0, abs_tol=1.0e-12)
        for actual, required in zip(expected, authority, strict=True)
    ):
        raise ValueError("Z-stack geometry does not match the frozen up50 environment")
    return root, environment.setup_config.work_frame.child_frame


def _build_experiment(
    site: ZStackSite,
    *,
    frame_id: str,
    tilt_deg: float,
    translation_duration_s: float,
    tilt_duration_s: float,
    sample_period_s: float,
    settle_duration_s: float,
    capture_duration_s: float,
) -> ZStackExperiment:
    translation_ns = _validated_seconds(
        translation_duration_s,
        "translation_duration_s",
    )
    tilt_ns = _validated_seconds(tilt_duration_s, "tilt_duration_s")
    sample_ns = _validated_seconds(sample_period_s, "sample_period_s")
    settle_ns = _validated_seconds(settle_duration_s, "settle_duration_s")
    capture_ns = _validated_seconds(capture_duration_s, "capture_duration_s")
    if translation_ns % sample_ns or tilt_ns % sample_ns:
        raise ValueError(
            "sample_period_s must divide translation and tilt durations exactly"
        )
    if not math.isclose(float(tilt_deg), 25.0, rel_tol=0.0, abs_tol=1.0e-12):
        raise ValueError("the authoritative Z-stack tilt must remain exactly 25 degrees")
    targets = _visit_targets(float(tilt_deg))
    validate_z_stack_tip_distances(tuple(target.distance_m for target in targets))

    poses: list[TimedPose] = []
    visits: list[ZStackVisit] = []
    current_time_ns = 0
    current_position = (
        site.x_m,
        site.y_m,
        Z_STACK_SENSOR_PLANE_Z_M - targets[0].distance_m,
    )
    current_quaternion = targets[0].quaternion_xyzw
    poses.append(
        TimedPose(
            time_s=0.0,
            frame_id=frame_id,
            position_m=current_position,
            quaternion_xyzw=current_quaternion,
            phase="draw",
        )
    )

    for visit_index, target in enumerate(targets):
        target_position = (
            site.x_m,
            site.y_m,
            Z_STACK_SENSOR_PLANE_Z_M - target.distance_m,
        )
        if visit_index:
            position_changed = any(
                not math.isclose(
                    target_position[axis],
                    current_position[axis],
                    rel_tol=0.0,
                    abs_tol=1.0e-15,
                )
                for axis in range(3)
            )
            orientation_dot = abs(
                sum(
                    current_quaternion[index] * target.quaternion_xyzw[index]
                    for index in range(4)
                )
            )
            orientation_changed = not math.isclose(
                orientation_dot,
                1.0,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
            if position_changed == orientation_changed:
                raise ValueError(
                    "each Z-stack leg must change exactly one of position or orientation"
                )
            move_ns = translation_ns if position_changed else tilt_ns
            start_time_ns = current_time_ns
            start_position = current_position
            start_quaternion = current_quaternion
            for offset_ns in range(sample_ns, move_ns + sample_ns, sample_ns):
                progress = minimum_jerk_progress(offset_ns / move_ns)
                position = tuple(
                    start_position[axis]
                    + progress * (target_position[axis] - start_position[axis])
                    for axis in range(3)
                )
                quaternion = tuple(
                    float(value)
                    for value in slerp_xyzw(
                        start_quaternion,
                        target.quaternion_xyzw,
                        progress,
                    )
                )
                poses.append(
                    TimedPose(
                        time_s=(start_time_ns + offset_ns) / 1_000_000_000.0,
                        frame_id=frame_id,
                        position_m=position,  # type: ignore[arg-type]
                        quaternion_xyzw=quaternion,  # type: ignore[arg-type]
                        phase="draw",
                    )
                )
            current_time_ns += move_ns
            current_position = target_position
            current_quaternion = target.quaternion_xyzw

        arrival_ns = current_time_ns
        settle_end_ns = arrival_ns + settle_ns
        capture_end_ns = settle_end_ns + capture_ns
        poses.append(
            TimedPose(
                time_s=settle_end_ns / 1_000_000_000.0,
                frame_id=frame_id,
                position_m=current_position,
                quaternion_xyzw=current_quaternion,
                phase="hold",
            )
        )
        poses.append(
            TimedPose(
                time_s=capture_end_ns / 1_000_000_000.0,
                frame_id=frame_id,
                position_m=current_position,
                quaternion_xyzw=current_quaternion,
                phase="hold",
            )
        )
        visits.append(
            ZStackVisit(
                visit_index=visit_index,
                visit_id=target.visit_id,
                stage=target.stage,
                distance_m=target.distance_m,
                tip_position_m=current_position,
                quaternion_xyzw=current_quaternion,
                arrival_time_s=arrival_ns / 1_000_000_000.0,
                settle_end_time_s=settle_end_ns / 1_000_000_000.0,
                capture_start_time_s=settle_end_ns / 1_000_000_000.0,
                capture_end_time_s=capture_end_ns / 1_000_000_000.0,
                fit_role=target.fit_role,
            )
        )
        current_time_ns = capture_end_ns

    return ZStackExperiment(site=site, poses=tuple(poses), visits=tuple(visits))


def build_z_stack_tilt_experiments(
    environment_path: str | Path,
    *,
    tilt_deg: float = Z_STACK_TILT_DEG,
    translation_duration_s: float = Z_STACK_TRANSLATION_DURATION_S,
    tilt_duration_s: float = Z_STACK_TILT_DURATION_S,
    sample_period_s: float = Z_STACK_SAMPLE_PERIOD_S,
    settle_duration_s: float = Z_STACK_SETTLE_DURATION_S,
    capture_duration_s: float = Z_STACK_CAPTURE_DURATION_S,
) -> tuple[ZStackExperiment, ...]:
    """Build two immutable, fully identifiable Z-stack-plus-tilt experiments."""

    _, frame_id = _validate_environment(environment_path)
    return tuple(
        _build_experiment(
            site,
            frame_id=frame_id,
            tilt_deg=tilt_deg,
            translation_duration_s=translation_duration_s,
            tilt_duration_s=tilt_duration_s,
            sample_period_s=sample_period_s,
            settle_duration_s=settle_duration_s,
            capture_duration_s=capture_duration_s,
        )
        for site in Z_STACK_SITES
    )


def rigid_offset_design_matrix(experiment: ZStackExperiment) -> np.ndarray:
    """Return the ``d = b + R o`` design used by the deepest tilt windows."""

    rows = []
    for visit in experiment.visits:
        if visit.fit_role not in {"z_stack_and_rigid", "rigid", "rigid_drift"}:
            continue
        rotation = quaternion_to_matrix_xyzw(visit.quaternion_xyzw)
        rows.append(np.hstack((np.eye(3, dtype=float), rotation)))
    return np.vstack(rows)


def fit_rigid_offsets(
    rotations_plane1_from_rod_tip: Sequence[Sequence[Sequence[float]]],
    tracker_minus_rod_tip_m: Sequence[Sequence[float]],
) -> RigidOffsetFit:
    """Solve ``d_i = b + R_i o`` from actual-FK rotations and tracker deltas."""

    rotations = np.asarray(rotations_plane1_from_rod_tip, dtype=float)
    deltas = np.asarray(tracker_minus_rod_tip_m, dtype=float)
    if (
        rotations.ndim != 3
        or rotations.shape[1:] != (3, 3)
        or deltas.shape != (len(rotations), 3)
        or len(rotations) < 3
        or not np.all(np.isfinite(rotations))
        or not np.all(np.isfinite(deltas))
    ):
        raise ValueError("rigid-offset fit requires finite N x 3 x 3 rotations and N x 3 deltas")
    for rotation in rotations:
        if not np.allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1.0e-8,
            rtol=0.0,
        ) or not math.isclose(
            float(np.linalg.det(rotation)),
            1.0,
            rel_tol=0.0,
            abs_tol=1.0e-8,
        ):
            raise ValueError("rigid-offset fit requires proper rotation matrices")
    design = np.vstack(
        [np.hstack((np.eye(3, dtype=float), rotation)) for rotation in rotations]
    )
    observation = deltas.reshape(-1)
    solution, _, rank, singular_values = np.linalg.lstsq(
        design,
        observation,
        rcond=None,
    )
    if rank != 6 or singular_values[-1] <= 1.0e-12:
        raise ValueError("actual orientations do not independently identify b and o")
    predicted = (design @ solution).reshape(-1, 3)
    residuals = deltas - predicted
    residual_norms = np.linalg.norm(residuals, axis=1)
    return RigidOffsetFit(
        board_bias_m=tuple(float(value) for value in solution[:3]),
        tool_offset_m=tuple(float(value) for value in solution[3:]),
        rmse_m=float(np.sqrt(np.mean(np.square(residual_norms)))),
        maximum_residual_m=float(np.max(residual_norms)),
        rank=int(rank),
        condition_number=float(singular_values[0] / singular_values[-1]),
    )


def _capture_window_csv(experiments: Iterable[ZStackExperiment]) -> bytes:
    stream = StringIO(newline="")
    writer = csv.DictWriter(
        stream,
        fieldnames=Z_STACK_CAPTURE_WINDOW_COLUMNS,
        lineterminator="\n",
    )
    writer.writeheader()
    for experiment in experiments:
        for visit in experiment.visits:
            writer.writerow(
                {
                    "environment_id": Z_STACK_ENVIRONMENT_ID,
                    "trajectory_id": experiment.site.trajectory_id,
                    "site_id": experiment.site.site_id,
                    "visit_index": visit.visit_index,
                    "visit_id": visit.visit_id,
                    "stage": visit.stage,
                    "distance_m": f"{visit.distance_m:.9f}",
                    "tip_x_m": f"{visit.tip_position_m[0]:.9f}",
                    "tip_y_m": f"{visit.tip_position_m[1]:.9f}",
                    "tip_z_m": f"{visit.tip_position_m[2]:.9f}",
                    "tip_qx": f"{visit.quaternion_xyzw[0]:.12f}",
                    "tip_qy": f"{visit.quaternion_xyzw[1]:.12f}",
                    "tip_qz": f"{visit.quaternion_xyzw[2]:.12f}",
                    "tip_qw": f"{visit.quaternion_xyzw[3]:.12f}",
                    "arrival_time_s": f"{visit.arrival_time_s:.9f}",
                    "settle_end_time_s": f"{visit.settle_end_time_s:.9f}",
                    "capture_start_time_s": f"{visit.capture_start_time_s:.9f}",
                    "capture_end_time_s": f"{visit.capture_end_time_s:.9f}",
                    "fit_role": visit.fit_role,
                }
            )
    return stream.getvalue().encode("utf-8")


def _write_immutable(path: Path, payload: bytes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink() or (path.exists() and not path.is_file()):
        raise FileExistsError(f"refusing to replace non-regular authority: {path}")
    if path.exists():
        if path.read_bytes() != payload:
            raise FileExistsError(f"refusing to overwrite differing authority: {path}")
        return path
    with path.open("xb") as stream:
        stream.write(payload)
    return path


def generate_z_stack_tilt_experiments(
    environment_path: str | Path,
) -> tuple[tuple[Path, ...], Path]:
    """Write both source CSVs and their authoritative capture-window table."""

    environment_root, _ = _validate_environment(environment_path)
    experiments = build_z_stack_tilt_experiments(environment_root)
    output_directory = environment_root / "trajectories"
    trajectory_paths = tuple(
        write_authoritative_trajectory(
            output_directory / f"{experiment.site.trajectory_id}.csv",
            experiment.poses,
        )
        for experiment in experiments
    )
    capture_path = _write_immutable(
        output_directory / "z_stack_tilt_calibration_capture_windows_v3.csv",
        _capture_window_csv(experiments),
    )
    return trajectory_paths, capture_path
