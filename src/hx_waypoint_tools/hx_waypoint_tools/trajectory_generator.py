from __future__ import annotations

import argparse
import csv
from bisect import bisect_left
from dataclasses import dataclass
from functools import lru_cache
from io import StringIO
from math import cos, exp, floor, isfinite, pi, sin, sqrt
from pathlib import Path
from typing import Sequence

from hx_waypoint_curobo.models import TimedPose
from hx_waypoint_curobo.trajectory_io import SOURCE_COLUMNS


ACCEPTANCE_TRAJECTORY_IDS = (
    "horizontal_line_mj",
    "circle_mj",
    "ellipse_mj",
    "shallow_s_mj",
)
ACCEPTANCE_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in ACCEPTANCE_TRAJECTORY_IDS
)
RECOVERY_TRAJECTORY_IDS = ("circle_mj_v2",)
RECOVERY_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in RECOVERY_TRAJECTORY_IDS
)
BAFE_STARTUP_TRAJECTORY_IDS = (
    "bafe_wave_3d",
    "bafe_helix_3d",
    "bafe_rounded_square_3d",
    "bafe_spiral_3d",
)
BAFE_STARTUP_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in BAFE_STARTUP_TRAJECTORY_IDS
)
BAFE_DOWNWARD_TRAJECTORY_IDS = tuple(
    f"{trajectory_id}_downward" for trajectory_id in BAFE_STARTUP_TRAJECTORY_IDS
)
BAFE_DOWNWARD_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in BAFE_DOWNWARD_TRAJECTORY_IDS
)
_BAFE_DOWNWARD_SOURCE_IDS = dict(
    zip(BAFE_DOWNWARD_TRAJECTORY_IDS, BAFE_STARTUP_TRAJECTORY_IDS, strict=True)
)
BAFE_DOWNWARD_MJ_TRAJECTORY_IDS = tuple(
    f"{trajectory_id}_downward_mj" for trajectory_id in BAFE_STARTUP_TRAJECTORY_IDS
)
BAFE_DOWNWARD_MJ_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in BAFE_DOWNWARD_MJ_TRAJECTORY_IDS
)
_BAFE_DOWNWARD_MJ_SOURCE_IDS = dict(
    zip(BAFE_DOWNWARD_MJ_TRAJECTORY_IDS, BAFE_STARTUP_TRAJECTORY_IDS, strict=True)
)
BAFE_GROUND_MJ_TRAJECTORY_IDS = tuple(
    f"{trajectory_id}_ground_mj" for trajectory_id in BAFE_STARTUP_TRAJECTORY_IDS
)
BAFE_GROUND_MJ_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in BAFE_GROUND_MJ_TRAJECTORY_IDS
)
_BAFE_GROUND_MJ_SOURCE_IDS = dict(
    zip(BAFE_GROUND_MJ_TRAJECTORY_IDS, BAFE_STARTUP_TRAJECTORY_IDS, strict=True)
)
BAFE_GROUND_MJ_V2_TRAJECTORY_IDS = tuple(
    f"{trajectory_id}_ground_mj_v2" for trajectory_id in BAFE_STARTUP_TRAJECTORY_IDS
)
BAFE_GROUND_MJ_V2_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in BAFE_GROUND_MJ_V2_TRAJECTORY_IDS
)
_BAFE_GROUND_MJ_V2_SOURCE_IDS = dict(
    zip(BAFE_GROUND_MJ_V2_TRAJECTORY_IDS, BAFE_STARTUP_TRAJECTORY_IDS, strict=True)
)
NEW_BOARD_FIG14_TRAJECTORY_IDS = (
    "fig14_figure8_3d_mj",
    "fig14_zigzag_s_curve_3d_mj",
)
NEW_BOARD_FIG14_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in NEW_BOARD_FIG14_TRAJECTORY_IDS
)
NEW_BOARD_CONTINUOUS_CORE_TRAJECTORY_IDS = (
    "bafe_continuous_wave_3d",
    "bafe_continuous_helix_3d",
    "bafe_continuous_rounded_square_3d",
    "bafe_continuous_figure8_3d",
    "bafe_continuous_lissajous_3d",
    "bafe_continuous_rosette_3d",
)
NEW_BOARD_CONTINUOUS_CANDIDATE_TRAJECTORY_IDS = (
    "bafe_continuous_trefoil_knot_3d",
    "bafe_continuous_clover_3d",
    "bafe_continuous_rounded_star_3d",
    "bafe_continuous_heart_3d",
    "bafe_continuous_spirograph_3d",
    "bafe_continuous_guilloche_3d",
    "bafe_continuous_seven_petal_rose_3d",
    "bafe_continuous_torus_knot_2_5_3d",
    "bafe_continuous_viviani_3d",
    "bafe_continuous_spherical_spiral_3d",
    "bafe_continuous_harmonograph_3d",
    "bafe_continuous_butterfly_3d",
    "bafe_continuous_superformula_flower_3d",
    "bafe_continuous_lorenz_3d",
    "bafe_continuous_celtic_knot_3d",
    "bafe_continuous_braided_helix_3d",
)
NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS = (
    NEW_BOARD_CONTINUOUS_CORE_TRAJECTORY_IDS
    + NEW_BOARD_CONTINUOUS_CANDIDATE_TRAJECTORY_IDS
)
NEW_BOARD_CONTINUOUS_PERSISTENT_MOTION_IDS = tuple(
    trajectory_id
    for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS
    if trajectory_id != "bafe_continuous_lorenz_3d"
)
NEW_BOARD_CONTINUOUS_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv" for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS
)
NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_TRAJECTORY_IDS = (
    "bafe_continuous_wave_3d",
    "bafe_continuous_helix_3d",
    "bafe_continuous_rounded_square_3d",
    "bafe_continuous_figure8_3d",
    "bafe_continuous_lissajous_3d",
    "bafe_continuous_trefoil_knot_3d",
    "bafe_continuous_heart_3d",
    "bafe_continuous_spirograph_3d",
    "bafe_continuous_spherical_spiral_3d",
    "bafe_continuous_celtic_knot_3d",
)
NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS = tuple(
    f"{trajectory_id}_zplus26mm"
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_TRAJECTORY_IDS
)
NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv"
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS
)
_NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_IDS = dict(
    zip(
        NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS,
        NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_TRAJECTORY_IDS,
        strict=True,
    )
)
NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_SOURCE_TRAJECTORY_IDS = (
    NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS
)
NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS = tuple(
    f"{trajectory_id}_up50mm"
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_SOURCE_TRAJECTORY_IDS
)
NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_FILENAMES = tuple(
    f"{trajectory_id}.csv"
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS
)
_NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_SOURCE_IDS = dict(
    zip(
        NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS,
        NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_SOURCE_TRAJECTORY_IDS,
        strict=True,
    )
)

FRAME_ID = "wall_draw_frame"
QUATERNION_XYZW = (0.0, 1.0, 0.0, 0.0)
DRAW_DURATION_NS = 30_000_000_000
HOLD_DURATION_NS = 3_000_000_000
DRAW_INTERVAL_COUNT = 120
DRAW_SAMPLE_COUNT = DRAW_INTERVAL_COUNT + 1
DRAW_INTERVAL_NS = DRAW_DURATION_NS // DRAW_INTERVAL_COUNT
RECOVERY_DRAW_DURATION_NS = 60_000_000_000
RECOVERY_DRAW_INTERVAL_NS = RECOVERY_DRAW_DURATION_NS // DRAW_INTERVAL_COUNT
BAFE_FRAME_ID = "plane1"
BAFE_REFERENCE_DURATION_S = 4.0
BAFE_DRAW_DURATION_NS = 4_800_000_000
BAFE_DRAW_INTERVAL_NS = 10_000_000
BAFE_DRAW_INTERVAL_COUNT = BAFE_DRAW_DURATION_NS // BAFE_DRAW_INTERVAL_NS
BAFE_DRAW_SAMPLE_COUNT = BAFE_DRAW_INTERVAL_COUNT + 1
BAFE_RAMP_DURATION_S = 0.5
BAFE_HOLD_DURATION_NS = 3_000_000_000
BAFE_REFERENCE_SAMPLE_COUNT = 401
BAFE_REFERENCE_Z_START_M = 0.05
BAFE_DOWNWARD_MJ_DRAW_DURATION_NS = 30_000_000_000
BAFE_DOWNWARD_MJ_DRAW_INTERVAL_NS = (
    BAFE_DOWNWARD_MJ_DRAW_DURATION_NS // BAFE_DRAW_INTERVAL_COUNT
)
BAFE_GROUND_MJ_DRAW_DURATION_NS = BAFE_DOWNWARD_MJ_DRAW_DURATION_NS
BAFE_GROUND_MJ_DRAW_INTERVAL_NS = BAFE_DOWNWARD_MJ_DRAW_INTERVAL_NS
BAFE_GROUND_MJ_V2_DRAW_DURATION_NS = 120_000_000_000
BAFE_GROUND_MJ_V2_DRAW_INTERVAL_NS = (
    BAFE_GROUND_MJ_V2_DRAW_DURATION_NS // BAFE_DRAW_INTERVAL_COUNT
)
NEW_BOARD_FRAME_ID = "plane1"
NEW_BOARD_QUATERNION_XYZW = (0.0, 0.0, 0.0, 1.0)
NEW_BOARD_DRAW_DURATION_NS = 36_000_000_000
NEW_BOARD_DRAW_INTERVAL_COUNT = 480
NEW_BOARD_DRAW_SAMPLE_COUNT = NEW_BOARD_DRAW_INTERVAL_COUNT + 1
NEW_BOARD_DRAW_INTERVAL_NS = (
    NEW_BOARD_DRAW_DURATION_NS // NEW_BOARD_DRAW_INTERVAL_COUNT
)
NEW_BOARD_HOLD_DURATION_NS = 3_000_000_000
NEW_BOARD_TRAJECTORY_CENTER_Z_M = -0.05
NEW_BOARD_CONTINUOUS_DURATION_NS = 30_000_000_000
NEW_BOARD_CONTINUOUS_INTERVAL_NS = 50_000_000
NEW_BOARD_CONTINUOUS_INTERVAL_COUNT = (
    NEW_BOARD_CONTINUOUS_DURATION_NS // NEW_BOARD_CONTINUOUS_INTERVAL_NS
)
NEW_BOARD_CONTINUOUS_SAMPLE_COUNT = NEW_BOARD_CONTINUOUS_INTERVAL_COUNT + 1
NEW_BOARD_CONTINUOUS_RAMP_DURATION_S = 0.6
NEW_BOARD_CONTINUOUS_ZPLUS26_OFFSET_M = 0.026
NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_OFFSET_M = -0.050
NEW_BOARD_ROUNDED_SQUARE_NORMALIZATION = 0.874719165351
NEW_BOARD_CANDIDATE_X_RADIUS_M = 0.044
NEW_BOARD_CANDIDATE_Y_RADIUS_M = 0.033
NEW_BOARD_CANDIDATE_Z_RADIUS_M = 0.007
NEW_BOARD_CANDIDATE_ARCLENGTH_SAMPLES = 8193
NEW_BOARD_CANDIDATE_REPEAT_COUNTS = {
    "bafe_continuous_trefoil_knot_3d": 5,
    "bafe_continuous_clover_3d": 7,
    "bafe_continuous_rounded_star_3d": 6,
    "bafe_continuous_heart_3d": 8,
    "bafe_continuous_spirograph_3d": 4,
    "bafe_continuous_guilloche_3d": 6,
    "bafe_continuous_seven_petal_rose_3d": 6,
    "bafe_continuous_torus_knot_2_5_3d": 4,
    "bafe_continuous_viviani_3d": 7,
    "bafe_continuous_spherical_spiral_3d": 4,
    "bafe_continuous_harmonograph_3d": 5,
    "bafe_continuous_butterfly_3d": 6,
    "bafe_continuous_superformula_flower_3d": 6,
    "bafe_continuous_lorenz_3d": 1,
    "bafe_continuous_celtic_knot_3d": 4,
    "bafe_continuous_braided_helix_3d": 6,
}
NEW_BOARD_CANDIDATE_PHASE_OFFSETS = {
    trajectory_id: (0.071 * (index + 1)) % 1.0
    for index, trajectory_id in enumerate(
        NEW_BOARD_CONTINUOUS_CANDIDATE_TRAJECTORY_IDS
    )
}
BAFE_DOWNWARD_MJ_VERTICAL_OFFSETS_M = {
    "bafe_wave_3d": -0.025,
    "bafe_helix_3d": 0.0,
    "bafe_rounded_square_3d": 0.0,
    "bafe_spiral_3d": 0.0,
}
# In the current lab_wall_a setup this plane1-relative orientation maps the
# tool0 -> rod_tip +Z axis to base_link -Z, so the physical rod points down.
BAFE_DOWNWARD_QUATERNION_XYZW = (
    0.0,
    -0.707106781187,
    0.0,
    0.707106781187,
)
# The measured p10 plane1 frame used by legacy_plane1_rod194 has local +Z
# aligned with world up (within the teach-pendant measurement tilt). Rotating
# the rod's local +Z to plane1 -Z therefore points the physical rod down while
# remaining exactly compatible with that environment's lower-plane policy.
BAFE_GROUND_QUATERNION_XYZW = QUATERNION_XYZW


@dataclass(frozen=True)
class _BafeTrajectorySpec:
    kind: str
    phase0_rad: float
    phase_rate_rad_s: float
    xy_rotation_rad: float
    parameters: tuple[float, ...]


@dataclass(frozen=True)
class NewBoardOrientationMotion:
    """Optional smooth tip-orientation motion for continuous trajectory exports."""

    enabled: bool = False
    amplitude_deg: float = 10.0
    cycle_count: int = 3

    def __post_init__(self) -> None:
        amplitude_deg = float(self.amplitude_deg)
        if not isfinite(amplitude_deg) or not 0.0 <= amplitude_deg <= 45.0:
            raise ValueError("amplitude_deg must be finite and within [0, 45]")
        if isinstance(self.cycle_count, bool) or not isinstance(self.cycle_count, int):
            raise ValueError("cycle_count must be an integer")
        if not 1 <= self.cycle_count <= 20:
            raise ValueError("cycle_count must be within [1, 20]")
        object.__setattr__(self, "enabled", bool(self.enabled))
        object.__setattr__(self, "amplitude_deg", amplitude_deg)


DEFAULT_NEW_BOARD_ORIENTATION_MOTION = NewBoardOrientationMotion()


# These fixed parameters reproduce the four representative trajectories used in
# the paper's frozen orientation-study figure. The robot export changes only the
# timing law: the same spatial segment is traversed with smooth static boundaries.
_BAFE_TRAJECTORY_SPECS = {
    "bafe_wave_3d": _BafeTrajectorySpec(
        kind="wave",
        phase0_rad=4.486157895869617,
        phase_rate_rad_s=1.5057718314740116,
        xy_rotation_rad=-3.283776937015545,
        parameters=(
            0.09470915134941868,
            0.02444780551618554,
            0.007249887916781189,
            0.4596296224060648,
            0.7696188186400252,
        ),
    ),
    "bafe_helix_3d": _BafeTrajectorySpec(
        kind="helix",
        phase0_rad=-4.389054925577096,
        phase_rate_rad_s=1.6188994372205394,
        xy_rotation_rad=-0.1421842834257532,
        parameters=(
            0.09539186699461535,
            0.0858931008457902,
            0.013161162242647352,
            1.203919280336985,
        ),
    ),
    "bafe_rounded_square_3d": _BafeTrajectorySpec(
        kind="rounded_square",
        phase0_rad=5.527340022883659,
        phase_rate_rad_s=1.3971882775582078,
        xy_rotation_rad=-3.283776937015545,
        parameters=(
            0.1016191005385011,
            0.10340591314600141,
            0.010139885465408527,
            1.0363478781935993,
        ),
    ),
    "bafe_spiral_3d": _BafeTrajectorySpec(
        kind="spiral",
        phase0_rad=1.8541273539817582,
        phase_rate_rad_s=1.478082858743553,
        xy_rotation_rad=3.3371224754579125,
        parameters=(
            0.09559739258679771,
            0.27481256471782395,
            -8.907663771386673,
            0.012762457858267533,
            1.4846507101649,
        ),
    ),
}


def _unit_interval(value: float, name: str) -> float:
    result = float(value)
    if not isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError(f"{name} must be finite and within [0, 1]")
    return result


def minimum_jerk_progress(unit_time: float) -> float:
    """Return quintic minimum-jerk progress for normalized time in ``[0, 1]``."""

    u = _unit_interval(unit_time, "unit_time")
    if u == 0.0:
        return 0.0
    if u == 1.0:
        return 1.0
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def minimum_jerk_velocity_factor(unit_time: float) -> float:
    """Return ``ds/du`` for :func:`minimum_jerk_progress`."""

    u = _unit_interval(unit_time, "unit_time")
    if u in (0.0, 1.0):
        return 0.0
    return 30.0 * u**2 * (1.0 - u) ** 2


def minimum_jerk_acceleration_factor(unit_time: float) -> float:
    """Return ``d²s/du²`` for :func:`minimum_jerk_progress`."""

    u = _unit_interval(unit_time, "unit_time")
    if u in (0.0, 1.0):
        return 0.0
    return 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u)


def _minimum_jerk_integral(unit_time: float) -> float:
    u = _unit_interval(unit_time, "unit_time")
    return 2.5 * u**4 - 3.0 * u**5 + u**6


def bafe_startup_progress(time_s: float) -> float:
    """Return the normalized BAFE path progress with smooth static boundaries."""

    value = float(time_s)
    duration_s = BAFE_DRAW_DURATION_NS / 1_000_000_000
    if not isfinite(value) or not 0.0 <= value <= duration_s:
        raise ValueError("time_s must lie within the BAFE draw interval")
    if value == 0.0:
        return 0.0
    if value == duration_s:
        return 1.0

    ramp_s = BAFE_RAMP_DURATION_S
    normalized_area_s = duration_s - ramp_s
    if value < ramp_s:
        return ramp_s * _minimum_jerk_integral(value / ramp_s) / normalized_area_s
    if value > duration_s - ramp_s:
        remaining = duration_s - value
        return 1.0 - ramp_s * _minimum_jerk_integral(
            remaining / ramp_s
        ) / normalized_area_s
    return (value - 0.5 * ramp_s) / normalized_area_s


def _bafe_local_shape(
    spec: _BafeTrajectorySpec,
    phase_rad: float,
) -> tuple[float, float, float]:
    if spec.kind == "wave":
        x_amplitude, y_amplitude, z_amplitude, y_phase, z_phase = spec.parameters
        x = x_amplitude * sin(phase_rad)
        y = y_amplitude * sin(4.0 * phase_rad + y_phase)
        z = z_amplitude * sin(5.0 * phase_rad + z_phase)
    elif spec.kind == "helix":
        x_radius, y_radius, z_amplitude, z_phase = spec.parameters
        x = x_radius * cos(phase_rad)
        y = y_radius * sin(phase_rad)
        z = z_amplitude * sin(2.0 * phase_rad + z_phase)
    elif spec.kind == "rounded_square":
        xy_scale, harmonic_weight, z_amplitude, z_phase = spec.parameters
        x = xy_scale * (
            cos(phase_rad) - harmonic_weight * cos(3.0 * phase_rad)
        )
        y = xy_scale * (
            sin(phase_rad) + harmonic_weight * sin(3.0 * phase_rad)
        )
        z = z_amplitude * sin(2.0 * phase_rad + z_phase)
    elif spec.kind == "spiral":
        xy_scale, modulation, radial_phase, z_amplitude, z_phase = spec.parameters
        radial = 1.0 + modulation * sin(phase_rad + radial_phase)
        x = xy_scale * radial * cos(phase_rad)
        y = xy_scale * radial * sin(phase_rad)
        z = z_amplitude * sin(2.0 * phase_rad + z_phase)
    else:
        raise ValueError(f"unknown BAFE trajectory kind: {spec.kind!r}")

    rotation_cos = cos(spec.xy_rotation_rad)
    rotation_sin = sin(spec.xy_rotation_rad)
    return (
        rotation_cos * x - rotation_sin * y,
        rotation_sin * x + rotation_cos * y,
        z,
    )


@lru_cache(maxsize=None)
def _bafe_reference_center_xy(trajectory_id: str) -> tuple[float, float]:
    spec = _BAFE_TRAJECTORY_SPECS[trajectory_id]
    sum_x = 0.0
    sum_y = 0.0
    for index in range(BAFE_REFERENCE_SAMPLE_COUNT):
        reference_time_s = BAFE_REFERENCE_DURATION_S * index / (
            BAFE_REFERENCE_SAMPLE_COUNT - 1
        )
        phase_rad = spec.phase0_rad + spec.phase_rate_rad_s * reference_time_s
        x, y, _ = _bafe_local_shape(spec, phase_rad)
        sum_x += x
        sum_y += y
    return (
        sum_x / BAFE_REFERENCE_SAMPLE_COUNT,
        sum_y / BAFE_REFERENCE_SAMPLE_COUNT,
    )


def bafe_startup_geometry_position(
    trajectory_id: str,
    progress: float,
) -> tuple[float, float, float]:
    """Evaluate one paper-matched BAFE trajectory in the ``plane1`` frame."""

    if trajectory_id not in BAFE_STARTUP_TRAJECTORY_IDS:
        raise ValueError(f"unknown BAFE startup trajectory_id: {trajectory_id!r}")
    s = _unit_interval(progress, "progress")
    spec = _BAFE_TRAJECTORY_SPECS[trajectory_id]
    phase_rad = spec.phase0_rad + (
        spec.phase_rate_rad_s * BAFE_REFERENCE_DURATION_S * s
    )
    x, y, z = _bafe_local_shape(spec, phase_rad)
    center_x, center_y = _bafe_reference_center_xy(trajectory_id)
    _, _, start_z = _bafe_local_shape(spec, spec.phase0_rad)
    return (
        x - center_x,
        y - center_y,
        BAFE_REFERENCE_Z_START_M + z - start_z,
    )


def acceptance_geometry_position(
    trajectory_id: str,
    progress: float,
) -> tuple[float, float, float]:
    """Evaluate one acceptance geometry at scalar progress in ``[0, 1]``."""

    s = _unit_interval(progress, "progress")
    if trajectory_id == "horizontal_line_mj":
        return (-0.04 + 0.08 * s, 0.0, 0.0)
    if trajectory_id in ("circle_mj", "circle_mj_v2"):
        if s in (0.0, 1.0):
            return (0.0, 0.035, 0.0)
        angle = 2.0 * pi * s
        return (0.035 * sin(angle), 0.035 * cos(angle), 0.0)
    if trajectory_id == "ellipse_mj":
        if s in (0.0, 1.0):
            return (0.0, 0.025, 0.0)
        angle = 2.0 * pi * s
        return (0.045 * sin(angle), 0.025 * cos(angle), 0.0)
    if trajectory_id == "shallow_s_mj":
        if s == 0.0:
            return (-0.04, 0.0, 0.0)
        if s == 1.0:
            return (0.04, 0.0, 0.0)
        return (-0.04 + 0.08 * s, 0.015 * sin(2.0 * pi * s), 0.0)
    raise ValueError(f"unknown acceptance trajectory_id: {trajectory_id!r}")


def new_board_fig14_geometry_position(
    trajectory_id: str,
    progress: float,
) -> tuple[float, float, float]:
    """Evaluate one clear, slow Fig. 14-inspired path in the ``plane1`` frame."""

    s = _unit_interval(progress, "progress")
    if trajectory_id == "fig14_figure8_3d_mj":
        # Three repetitions preserve the recognizable figure-eight while a
        # phase-shifted depth component supplies modest 3-D excitation.
        angle = 0.0 if s in (0.0, 1.0) else 6.0 * pi * s
        return (
            0.04 * sin(angle),
            0.025 * sin(2.0 * angle),
            NEW_BOARD_TRAJECTORY_CENTER_Z_M + 0.008 * sin(angle + 0.45),
        )
    if trajectory_id == "fig14_zigzag_s_curve_3d_mj":
        # A rising sinusoidal sweep is a continuous serpentine/S curve: there
        # are no piecewise corners and no phase-shift seam that could jump.
        periodic_s = 0.0 if s == 1.0 else s
        return (
            0.045 * sin(6.0 * pi * periodic_s),
            -0.03 + 0.06 * s,
            NEW_BOARD_TRAJECTORY_CENTER_Z_M
            + 0.007 * sin(4.0 * pi * periodic_s + 1.0),
        )
    raise ValueError(f"unknown new-board Fig. 14 trajectory_id: {trajectory_id!r}")


def new_board_continuous_phase_progress(time_s: float) -> float:
    """Return phase progress with smooth endpoint ramps and steady interior speed."""

    value = float(time_s)
    duration_s = NEW_BOARD_CONTINUOUS_DURATION_NS / 1_000_000_000
    if not isfinite(value) or not 0.0 <= value <= duration_s:
        raise ValueError("time_s must lie within the continuous trajectory interval")
    if value == 0.0:
        return 0.0
    if value == duration_s:
        return 1.0

    ramp_s = NEW_BOARD_CONTINUOUS_RAMP_DURATION_S
    traversed_s: float
    if value < ramp_s:
        traversed_s = ramp_s * _minimum_jerk_integral(value / ramp_s)
    elif value > duration_s - ramp_s:
        remaining_s = duration_s - value
        traversed_s = (
            duration_s
            - ramp_s
            - ramp_s * _minimum_jerk_integral(remaining_s / ramp_s)
        )
    else:
        traversed_s = value - 0.5 * ramp_s
    return traversed_s / (duration_s - ramp_s)


@lru_cache(maxsize=1)
def _lorenz_curve_samples() -> tuple[tuple[float, float, float], ...]:
    """Integrate one deterministic Lorenz path used by the open candidate."""

    sigma = 10.0
    rho = 28.0
    beta = 8.0 / 3.0
    dt_s = 0.003
    burn_in_steps = 2000
    retained_steps = 18000
    x, y, z = 0.1, 0.0, 0.0
    samples: list[tuple[float, float, float]] = []

    def derivative(
        state_x: float,
        state_y: float,
        state_z: float,
    ) -> tuple[float, float, float]:
        return (
            sigma * (state_y - state_x),
            state_x * (rho - state_z) - state_y,
            state_x * state_y - beta * state_z,
        )

    for step in range(burn_in_steps + retained_steps):
        k1 = derivative(x, y, z)
        k2 = derivative(
            x + 0.5 * dt_s * k1[0],
            y + 0.5 * dt_s * k1[1],
            z + 0.5 * dt_s * k1[2],
        )
        k3 = derivative(
            x + 0.5 * dt_s * k2[0],
            y + 0.5 * dt_s * k2[1],
            z + 0.5 * dt_s * k2[2],
        )
        k4 = derivative(
            x + dt_s * k3[0],
            y + dt_s * k3[1],
            z + dt_s * k3[2],
        )
        x += dt_s * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        y += dt_s * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        z += dt_s * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]) / 6.0
        if step >= burn_in_steps:
            samples.append((x, y, z))
    control_count = 32
    return tuple(
        samples[
            round(index * (len(samples) - 1) / (control_count - 1))
        ]
        for index in range(control_count)
    )


def _candidate_raw_curve(
    trajectory_id: str,
    unit_parameter: float,
) -> tuple[float, float, float]:
    """Evaluate an unscaled candidate curve on its canonical interval."""

    u = min(max(float(unit_parameter), 0.0), 1.0)
    theta = 2.0 * pi * u

    if trajectory_id == "bafe_continuous_trefoil_knot_3d":
        return (
            sin(theta) + 2.0 * sin(2.0 * theta),
            cos(theta) - 2.0 * cos(2.0 * theta),
            -sin(3.0 * theta),
        )
    if trajectory_id == "bafe_continuous_clover_3d":
        radius = 0.78 + 0.22 * cos(4.0 * theta)
        return (
            radius * cos(theta),
            radius * sin(theta),
            0.55 * sin(3.0 * theta + 0.45),
        )
    if trajectory_id == "bafe_continuous_rounded_star_3d":
        radius = 0.84 + 0.16 * cos(5.0 * theta)
        return (
            radius * cos(theta),
            radius * sin(theta),
            0.50 * sin(4.0 * theta + 0.35),
        )
    if trajectory_id == "bafe_continuous_heart_3d":
        return (
            sin(theta) ** 3,
            0.76 * cos(theta)
            - 0.22 * cos(2.0 * theta)
            - 0.05 * cos(3.0 * theta),
            0.45 * sin(3.0 * theta + 0.6),
        )
    if trajectory_id == "bafe_continuous_spirograph_3d":
        t = 6.0 * pi * u
        major_radius = 5.0
        minor_radius = 3.0
        pen_offset = 1.35
        ratio = (major_radius - minor_radius) / minor_radius
        return (
            (major_radius - minor_radius) * cos(t)
            + pen_offset * cos(ratio * t),
            (major_radius - minor_radius) * sin(t)
            - pen_offset * sin(ratio * t),
            0.75 * sin(t + 0.35),
        )
    if trajectory_id == "bafe_continuous_guilloche_3d":
        radius = 0.88 + 0.095 * cos(7.0 * theta) + 0.004 * cos(13.0 * theta)
        twist = 0.004 * sin(9.0 * theta)
        return (
            radius * cos(theta) - twist * sin(theta),
            radius * sin(theta) + twist * cos(theta),
            0.55 * sin(4.0 * theta + 0.4),
        )
    if trajectory_id == "bafe_continuous_seven_petal_rose_3d":
        radius = 0.90 + 0.10 * cos(7.0 * theta)
        return (
            radius * cos(theta),
            radius * sin(theta),
            0.45 * sin(5.0 * theta + 0.5),
        )
    if trajectory_id == "bafe_continuous_torus_knot_2_5_3d":
        tube = 0.28
        radial = 1.0 + tube * cos(5.0 * theta)
        return (
            radial * cos(2.0 * theta),
            radial * sin(2.0 * theta),
            tube * sin(5.0 * theta),
        )
    if trajectory_id == "bafe_continuous_viviani_3d":
        t = 4.0 * pi * u
        return (
            1.0 + cos(t),
            sin(t),
            2.0 * sin(0.5 * t),
        )
    if trajectory_id == "bafe_continuous_spherical_spiral_3d":
        latitude = 0.62 * sin(4.0 * theta)
        return (
            cos(latitude) * cos(3.0 * theta),
            cos(latitude) * sin(3.0 * theta),
            sin(latitude),
        )
    if trajectory_id == "bafe_continuous_harmonograph_3d":
        return (
            sin(2.0 * theta + 0.65) + 0.24 * sin(3.0 * theta - 0.35),
            sin(3.0 * theta) + 0.20 * sin(5.0 * theta + 1.0),
            0.50 * sin(4.0 * theta + 0.2),
        )
    if trajectory_id == "bafe_continuous_butterfly_3d":
        radial = exp(0.35 * cos(theta)) - 0.30 * cos(4.0 * theta)
        return (
            sin(theta) * radial,
            cos(theta) * radial,
            0.50 * sin(3.0 * theta + 0.4),
        )
    if trajectory_id == "bafe_continuous_superformula_flower_3d":
        m = 7.0
        n1 = 2.0
        n2 = 2.0
        n3 = 2.0
        term = (
            abs(cos(m * theta / 4.0)) ** n2
            + abs(sin(m * theta / 4.0)) ** n3
        )
        radius = max(term, 1.0e-12) ** (-1.0 / n1)
        return (
            radius * cos(theta),
            radius * sin(theta),
            0.50 * sin(5.0 * theta + 0.25),
        )
    if trajectory_id == "bafe_continuous_lorenz_3d":
        samples = _lorenz_curve_samples()
        coordinate = u * (len(samples) - 1)
        index = min(int(floor(coordinate)), len(samples) - 2)
        fraction = coordinate - index
        points = (
            samples[max(index - 1, 0)],
            samples[index],
            samples[min(index + 1, len(samples) - 1)],
            samples[min(index + 2, len(samples) - 1)],
        )
        return tuple(
            0.5
            * (
                2.0 * points[1][axis]
                + (-points[0][axis] + points[2][axis]) * fraction
                + (
                    2.0 * points[0][axis]
                    - 5.0 * points[1][axis]
                    + 4.0 * points[2][axis]
                    - points[3][axis]
                )
                * fraction**2
                + (
                    -points[0][axis]
                    + 3.0 * points[1][axis]
                    - 3.0 * points[2][axis]
                    + points[3][axis]
                )
                * fraction**3
            )
            for axis in range(3)
        )
    if trajectory_id == "bafe_continuous_celtic_knot_3d":
        return (
            sin(2.0 * theta + 0.5),
            sin(3.0 * theta) + 0.16 * sin(6.0 * theta + 0.3),
            0.60 * sin(4.0 * theta + 0.15),
        )
    if trajectory_id == "bafe_continuous_braided_helix_3d":
        tube = 0.20
        radial = (
            1.0
            + tube * cos(6.0 * theta)
            + 0.07 * cos(theta + 0.45)
        )
        return (
            radial * cos(theta),
            radial * sin(theta),
            tube * sin(6.0 * theta) + 0.06 * sin(theta + 0.8),
        )
    raise ValueError(f"unknown new-board candidate trajectory_id: {trajectory_id!r}")


@lru_cache(maxsize=None)
def _candidate_axis_normalization(
    trajectory_id: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    sample_count = 4097
    samples = [
        _candidate_raw_curve(trajectory_id, index / (sample_count - 1))
        for index in range(sample_count)
    ]
    minima = tuple(min(sample[axis] for sample in samples) for axis in range(3))
    maxima = tuple(max(sample[axis] for sample in samples) for axis in range(3))
    centers = tuple(0.5 * (minimum + maximum) for minimum, maximum in zip(minima, maxima))
    radii = tuple(
        max(0.5 * (maximum - minimum), 1.0e-12)
        for minimum, maximum in zip(minima, maxima)
    )
    return centers, radii


def _candidate_normalized_curve(
    trajectory_id: str,
    unit_parameter: float,
) -> tuple[float, float, float]:
    raw = _candidate_raw_curve(trajectory_id, unit_parameter)
    centers, radii = _candidate_axis_normalization(trajectory_id)
    return tuple(
        (raw[axis] - centers[axis]) / radii[axis]
        for axis in range(3)
    )


@lru_cache(maxsize=None)
def _candidate_arclength_table(
    trajectory_id: str,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    parameters = tuple(
        index / (NEW_BOARD_CANDIDATE_ARCLENGTH_SAMPLES - 1)
        for index in range(NEW_BOARD_CANDIDATE_ARCLENGTH_SAMPLES)
    )
    normalized = [
        _candidate_normalized_curve(trajectory_id, parameter)
        for parameter in parameters
    ]
    cumulative = [0.0]
    for left, right in zip(normalized, normalized[1:]):
        dx = NEW_BOARD_CANDIDATE_X_RADIUS_M * (right[0] - left[0])
        dy = NEW_BOARD_CANDIDATE_Y_RADIUS_M * (right[1] - left[1])
        dz = NEW_BOARD_CANDIDATE_Z_RADIUS_M * (right[2] - left[2])
        cumulative.append(cumulative[-1] + sqrt(dx * dx + dy * dy + dz * dz))
    total = cumulative[-1]
    if not isfinite(total) or total <= 0.0:
        raise ValueError(f"candidate curve has invalid arc length: {trajectory_id}")
    fractions = tuple(value / total for value in cumulative)
    return parameters, fractions


def _candidate_parameter_at_arclength(
    trajectory_id: str,
    arc_fraction: float,
) -> float:
    parameters, fractions = _candidate_arclength_table(trajectory_id)
    target = min(max(float(arc_fraction), 0.0), 1.0)
    right = bisect_left(fractions, target)
    if right <= 0:
        return parameters[0]
    if right >= len(fractions):
        return parameters[-1]
    left = right - 1
    span = fractions[right] - fractions[left]
    weight = 0.0 if span <= 0.0 else (target - fractions[left]) / span
    return parameters[left] + weight * (parameters[right] - parameters[left])


def _candidate_continuous_position(
    trajectory_id: str,
    progress: float,
) -> tuple[float, float, float]:
    repeat_count = NEW_BOARD_CANDIDATE_REPEAT_COUNTS[trajectory_id]
    phase_offset = NEW_BOARD_CANDIDATE_PHASE_OFFSETS[trajectory_id]
    if trajectory_id == "bafe_continuous_lorenz_3d":
        parameter = min(max(float(progress), 0.0), 1.0)
    else:
        parameter = phase_offset + repeat_count * float(progress)
        parameter -= floor(parameter)
        if progress == 1.0:
            parameter = phase_offset
    x, y, z = _candidate_normalized_curve(trajectory_id, parameter)
    geometry_scale = {
        "bafe_continuous_guilloche_3d": 1.01,
        "bafe_continuous_lorenz_3d": 1.0,
    }.get(trajectory_id, 1.0)
    return (
        geometry_scale * NEW_BOARD_CANDIDATE_X_RADIUS_M * x,
        geometry_scale * NEW_BOARD_CANDIDATE_Y_RADIUS_M * y,
        NEW_BOARD_TRAJECTORY_CENTER_Z_M
        + geometry_scale * NEW_BOARD_CANDIDATE_Z_RADIUS_M * z,
    )


def new_board_continuous_geometry_position(
    trajectory_id: str,
    time_s: float,
) -> tuple[float, float, float]:
    """Evaluate one continuously informative 30 s path in the ``plane1`` frame."""

    if trajectory_id not in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS:
        raise ValueError(
            f"unknown new-board continuous trajectory_id: {trajectory_id!r}"
        )
    progress = new_board_continuous_phase_progress(time_s)
    center_z = NEW_BOARD_TRAJECTORY_CENTER_Z_M

    if trajectory_id == "bafe_continuous_wave_3d":
        angle = 1.15 + 2.0 * pi * 7.0 * progress
        return (
            0.045 * sin(angle),
            0.014 * sin(3.0 * angle + 0.35),
            center_z + 0.006 * sin(3.0 * angle + 0.8),
        )
    if trajectory_id == "bafe_continuous_helix_3d":
        angle = 2.0 * pi * 8.0 * progress
        return (
            0.042 * cos(angle),
            0.034 * sin(angle),
            center_z + 0.007 * sin(3.0 * angle + 0.55),
        )
    if trajectory_id == "bafe_continuous_rounded_square_3d":
        angle = 0.75 + 2.0 * pi * 7.0 * progress
        harmonic_weight = 0.13
        normalization = NEW_BOARD_ROUNDED_SQUARE_NORMALIZATION
        return (
            0.045
            * (cos(angle) - harmonic_weight * cos(3.0 * angle))
            / normalization,
            0.034
            * (sin(angle) + harmonic_weight * sin(3.0 * angle))
            / normalization,
            center_z + 0.006 * sin(3.0 * angle + 0.7),
        )
    if trajectory_id == "bafe_continuous_figure8_3d":
        angle = 1.15 + 2.0 * pi * 7.0 * progress
        return (
            0.044 * sin(angle),
            0.026 * sin(2.0 * angle) + 0.003 * sin(3.0 * angle + 0.4),
            center_z + 0.007 * sin(3.0 * angle + 0.5),
        )
    if trajectory_id == "bafe_continuous_lissajous_3d":
        angle = 1.15 + 2.0 * pi * 4.0 * progress
        return (
            0.044 * sin(2.0 * angle + 0.4),
            0.033 * sin(3.0 * angle),
            center_z
            + 0.007 * sin(4.0 * angle + 0.9)
            + 0.0015 * sin(angle),
        )
    if trajectory_id == "bafe_continuous_rosette_3d":
        angle = 2.0 * pi * 8.0 * progress
        radius = 0.032 + 0.010 * cos(3.0 * angle)
        return (
            radius * cos(angle),
            0.82 * radius * sin(angle),
            center_z + 0.007 * sin(4.0 * angle + 0.25),
        )
    if trajectory_id in NEW_BOARD_CONTINUOUS_CANDIDATE_TRAJECTORY_IDS:
        return _candidate_continuous_position(trajectory_id, progress)
    raise AssertionError("unreachable continuous trajectory branch")


def new_board_continuous_quaternion(
    time_s: float,
    orientation_motion: NewBoardOrientationMotion | None = None,
) -> tuple[float, float, float, float]:
    """Evaluate an optional bounded tilt about the fixed new-board orientation."""

    profile = orientation_motion or DEFAULT_NEW_BOARD_ORIENTATION_MOTION
    progress = new_board_continuous_phase_progress(time_s)
    if not profile.enabled or profile.amplitude_deg == 0.0 or progress in (0.0, 1.0):
        return NEW_BOARD_QUATERNION_XYZW

    tilt_rad = (
        profile.amplitude_deg
        * pi
        / 180.0
        * sin(2.0 * pi * profile.cycle_count * progress)
    )
    axis_azimuth_rad = 2.0 * pi * progress + 0.35
    half_tilt = 0.5 * tilt_rad
    vector_scale = sin(half_tilt)
    return (
        cos(axis_azimuth_rad) * vector_scale,
        sin(axis_azimuth_rad) * vector_scale,
        0.0,
        cos(half_tilt),
    )


def build_acceptance_trajectory(trajectory_id: str) -> tuple[TimedPose, ...]:
    """Build one authoritative minimum-jerk trajectory, including its final hold."""

    if trajectory_id not in ACCEPTANCE_TRAJECTORY_IDS:
        raise ValueError(f"unknown acceptance trajectory_id: {trajectory_id!r}")

    poses: list[TimedPose] = []
    for index in range(DRAW_SAMPLE_COUNT):
        unit_time = index / DRAW_INTERVAL_COUNT
        poses.append(
            TimedPose(
                time_s=(index * DRAW_INTERVAL_NS) / 1_000_000_000,
                frame_id=FRAME_ID,
                position_m=acceptance_geometry_position(
                    trajectory_id,
                    minimum_jerk_progress(unit_time),
                ),
                quaternion_xyzw=QUATERNION_XYZW,
                phase="draw",
            )
        )

    final_draw_pose = poses[-1]
    poses.append(
        TimedPose(
            time_s=(DRAW_DURATION_NS + HOLD_DURATION_NS) / 1_000_000_000,
            frame_id=final_draw_pose.frame_id,
            position_m=final_draw_pose.position_m,
            quaternion_xyzw=final_draw_pose.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_acceptance_trajectories() -> dict[str, tuple[TimedPose, ...]]:
    """Build all authoritative acceptance trajectories in stable ID order."""

    return {
        trajectory_id: build_acceptance_trajectory(trajectory_id)
        for trajectory_id in ACCEPTANCE_TRAJECTORY_IDS
    }


def build_recovery_trajectory(trajectory_id: str) -> tuple[TimedPose, ...]:
    """Build a new, slower candidate without modifying a rejected authority file."""

    if trajectory_id not in RECOVERY_TRAJECTORY_IDS:
        raise ValueError(f"unknown recovery trajectory_id: {trajectory_id!r}")
    poses: list[TimedPose] = []
    for index in range(DRAW_SAMPLE_COUNT):
        unit_time = index / DRAW_INTERVAL_COUNT
        poses.append(
            TimedPose(
                time_s=(index * RECOVERY_DRAW_INTERVAL_NS) / 1_000_000_000,
                frame_id=FRAME_ID,
                position_m=acceptance_geometry_position(
                    trajectory_id,
                    minimum_jerk_progress(unit_time),
                ),
                quaternion_xyzw=QUATERNION_XYZW,
                phase="draw",
            )
        )
    final_draw_pose = poses[-1]
    poses.append(
        TimedPose(
            time_s=(RECOVERY_DRAW_DURATION_NS + HOLD_DURATION_NS) / 1_000_000_000,
            frame_id=final_draw_pose.frame_id,
            position_m=final_draw_pose.position_m,
            quaternion_xyzw=final_draw_pose.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_new_board_fig14_trajectory(
    trajectory_id: str,
) -> tuple[TimedPose, ...]:
    """Build one new-board authority with minimum-jerk timing and a static hold."""

    if trajectory_id not in NEW_BOARD_FIG14_TRAJECTORY_IDS:
        raise ValueError(
            f"unknown new-board Fig. 14 trajectory_id: {trajectory_id!r}"
        )

    poses: list[TimedPose] = []
    for index in range(NEW_BOARD_DRAW_SAMPLE_COUNT):
        time_ns = index * NEW_BOARD_DRAW_INTERVAL_NS
        poses.append(
            TimedPose(
                time_s=time_ns / 1_000_000_000,
                frame_id=NEW_BOARD_FRAME_ID,
                position_m=new_board_fig14_geometry_position(
                    trajectory_id,
                    minimum_jerk_progress(index / NEW_BOARD_DRAW_INTERVAL_COUNT),
                ),
                quaternion_xyzw=NEW_BOARD_QUATERNION_XYZW,
                phase="draw",
            )
        )

    final_draw_pose = poses[-1]
    poses.append(
        TimedPose(
            time_s=(
                NEW_BOARD_DRAW_DURATION_NS + NEW_BOARD_HOLD_DURATION_NS
            )
            / 1_000_000_000,
            frame_id=final_draw_pose.frame_id,
            position_m=final_draw_pose.position_m,
            quaternion_xyzw=final_draw_pose.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_new_board_fig14_trajectories() -> dict[str, tuple[TimedPose, ...]]:
    """Build both new-board Fig. 14-inspired trajectories in stable ID order."""

    return {
        trajectory_id: build_new_board_fig14_trajectory(trajectory_id)
        for trajectory_id in NEW_BOARD_FIG14_TRAJECTORY_IDS
    }


def build_new_board_continuous_trajectory(
    trajectory_id: str,
    *,
    orientation_motion: NewBoardOrientationMotion | None = None,
) -> tuple[TimedPose, ...]:
    """Build one 30 s authority with continuous motion and static endpoints."""

    if trajectory_id not in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS:
        raise ValueError(
            f"unknown new-board continuous trajectory_id: {trajectory_id!r}"
        )
    return tuple(
        TimedPose(
            time_s=(index * NEW_BOARD_CONTINUOUS_INTERVAL_NS) / 1_000_000_000,
            frame_id=NEW_BOARD_FRAME_ID,
            position_m=new_board_continuous_geometry_position(
                trajectory_id,
                (index * NEW_BOARD_CONTINUOUS_INTERVAL_NS) / 1_000_000_000,
            ),
            quaternion_xyzw=new_board_continuous_quaternion(
                (index * NEW_BOARD_CONTINUOUS_INTERVAL_NS) / 1_000_000_000,
                orientation_motion,
            ),
            phase="draw",
        )
        for index in range(NEW_BOARD_CONTINUOUS_SAMPLE_COUNT)
    )


def build_new_board_continuous_trajectories(
    *,
    orientation_motion: NewBoardOrientationMotion | None = None,
) -> dict[str, tuple[TimedPose, ...]]:
    """Build all continuously informative new-board trajectories in stable order."""

    return {
        trajectory_id: build_new_board_continuous_trajectory(
            trajectory_id,
            orientation_motion=orientation_motion,
        )
        for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS
    }


def build_new_board_continuous_zplus26_trajectory(
    trajectory_id: str,
) -> tuple[TimedPose, ...]:
    """Build a new authority by translating one sealed geometry +26 mm in Z."""

    source_trajectory_id = _NEW_BOARD_CONTINUOUS_ZPLUS26_SOURCE_IDS.get(
        trajectory_id
    )
    if source_trajectory_id is None:
        raise ValueError(
            f"unknown new-board +26 mm trajectory_id: {trajectory_id!r}"
        )
    return tuple(
        TimedPose(
            time_s=pose.time_s,
            frame_id=pose.frame_id,
            position_m=(
                pose.position_m[0],
                pose.position_m[1],
                pose.position_m[2] + NEW_BOARD_CONTINUOUS_ZPLUS26_OFFSET_M,
            ),
            quaternion_xyzw=pose.quaternion_xyzw,
            phase=pose.phase,
        )
        for pose in build_new_board_continuous_trajectory(source_trajectory_id)
    )


def build_new_board_continuous_zplus26_trajectories(
) -> dict[str, tuple[TimedPose, ...]]:
    """Build all selected +26 mm authorities in stable source-trajectory order."""

    return {
        trajectory_id: build_new_board_continuous_zplus26_trajectory(trajectory_id)
        for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS
    }


def build_new_board_continuous_zplus26_up50_trajectory(
    trajectory_id: str,
) -> tuple[TimedPose, ...]:
    """Translate one +26 mm authority by exactly -50 mm in plane1 Z."""

    source_trajectory_id = _NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_SOURCE_IDS.get(
        trajectory_id
    )
    if source_trajectory_id is None:
        raise ValueError(
            f"unknown new-board +26 mm/up 50 mm trajectory_id: {trajectory_id!r}"
        )
    return tuple(
        TimedPose(
            time_s=pose.time_s,
            frame_id=pose.frame_id,
            position_m=(
                pose.position_m[0],
                pose.position_m[1],
                pose.position_m[2]
                + NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_OFFSET_M,
            ),
            quaternion_xyzw=pose.quaternion_xyzw,
            phase=pose.phase,
        )
        for pose in build_new_board_continuous_zplus26_trajectory(
            source_trajectory_id
        )
    )


def build_new_board_continuous_zplus26_up50_trajectories(
) -> dict[str, tuple[TimedPose, ...]]:
    """Build all selected +26 mm authorities shifted upward by 50 mm."""

    return {
        trajectory_id: build_new_board_continuous_zplus26_up50_trajectory(
            trajectory_id
        )
        for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS
    }


def _build_bafe_trajectory(
    geometry_trajectory_id: str,
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[TimedPose, ...]:
    poses: list[TimedPose] = []
    for index in range(BAFE_DRAW_SAMPLE_COUNT):
        time_ns = index * BAFE_DRAW_INTERVAL_NS
        time_s = time_ns / 1_000_000_000
        poses.append(
            TimedPose(
                time_s=time_s,
                frame_id=BAFE_FRAME_ID,
                position_m=bafe_startup_geometry_position(
                    geometry_trajectory_id,
                    bafe_startup_progress(time_s),
                ),
                quaternion_xyzw=quaternion_xyzw,
                phase="draw",
            )
        )

    final_draw_pose = poses[-1]
    poses.append(
        TimedPose(
            time_s=(
                BAFE_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS
            ) / 1_000_000_000,
            frame_id=final_draw_pose.frame_id,
            position_m=final_draw_pose.position_m,
            quaternion_xyzw=final_draw_pose.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_bafe_startup_trajectory(trajectory_id: str) -> tuple[TimedPose, ...]:
    """Build one original BAFE candidate with static endpoint timing."""

    if trajectory_id not in BAFE_STARTUP_TRAJECTORY_IDS:
        raise ValueError(f"unknown BAFE startup trajectory_id: {trajectory_id!r}")
    return _build_bafe_trajectory(trajectory_id, QUATERNION_XYZW)


def build_bafe_startup_trajectories() -> dict[str, tuple[TimedPose, ...]]:
    """Build all four paper-matched BAFE trajectories in stable ID order."""

    return {
        trajectory_id: build_bafe_startup_trajectory(trajectory_id)
        for trajectory_id in BAFE_STARTUP_TRAJECTORY_IDS
    }


def build_bafe_downward_trajectory(trajectory_id: str) -> tuple[TimedPose, ...]:
    """Build a new BAFE candidate whose physical rod points toward the ground."""

    geometry_trajectory_id = _BAFE_DOWNWARD_SOURCE_IDS.get(trajectory_id)
    if geometry_trajectory_id is None:
        raise ValueError(f"unknown downward BAFE trajectory_id: {trajectory_id!r}")
    return _build_bafe_trajectory(
        geometry_trajectory_id,
        BAFE_DOWNWARD_QUATERNION_XYZW,
    )


def build_bafe_downward_trajectories() -> dict[str, tuple[TimedPose, ...]]:
    """Build all four downward-pointing replacement candidates in stable order."""

    return {
        trajectory_id: build_bafe_downward_trajectory(trajectory_id)
        for trajectory_id in BAFE_DOWNWARD_TRAJECTORY_IDS
    }


def build_bafe_downward_mj_trajectory(trajectory_id: str) -> tuple[TimedPose, ...]:
    """Build a slow minimum-jerk BAFE candidate with its rod pointing down."""

    geometry_trajectory_id = _BAFE_DOWNWARD_MJ_SOURCE_IDS.get(trajectory_id)
    if geometry_trajectory_id is None:
        raise ValueError(f"unknown downward minimum-jerk trajectory_id: {trajectory_id!r}")
    vertical_offset_m = BAFE_DOWNWARD_MJ_VERTICAL_OFFSETS_M[geometry_trajectory_id]
    poses: list[TimedPose] = []
    for index in range(BAFE_DRAW_SAMPLE_COUNT):
        time_ns = index * BAFE_DOWNWARD_MJ_DRAW_INTERVAL_NS
        position = bafe_startup_geometry_position(
            geometry_trajectory_id,
            minimum_jerk_progress(index / BAFE_DRAW_INTERVAL_COUNT),
        )
        poses.append(
            TimedPose(
                time_s=time_ns / 1_000_000_000,
                frame_id=BAFE_FRAME_ID,
                position_m=(
                    position[0] + vertical_offset_m,
                    position[1],
                    position[2],
                ),
                quaternion_xyzw=BAFE_DOWNWARD_QUATERNION_XYZW,
                phase="draw",
            )
        )

    final_draw_pose = poses[-1]
    poses.append(
        TimedPose(
            time_s=(
                BAFE_DOWNWARD_MJ_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS
            ) / 1_000_000_000,
            frame_id=final_draw_pose.frame_id,
            position_m=final_draw_pose.position_m,
            quaternion_xyzw=final_draw_pose.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_bafe_downward_mj_trajectories() -> dict[str, tuple[TimedPose, ...]]:
    """Build all slow downward BAFE candidates in stable order."""

    return {
        trajectory_id: build_bafe_downward_mj_trajectory(trajectory_id)
        for trajectory_id in BAFE_DOWNWARD_MJ_TRAJECTORY_IDS
    }


def build_bafe_ground_mj_trajectory(trajectory_id: str) -> tuple[TimedPose, ...]:
    """Build a p10-frame minimum-jerk candidate whose physical rod points down."""

    geometry_trajectory_id = _BAFE_GROUND_MJ_SOURCE_IDS.get(trajectory_id)
    if geometry_trajectory_id is None:
        raise ValueError(f"unknown ground minimum-jerk trajectory_id: {trajectory_id!r}")
    poses: list[TimedPose] = []
    for index in range(BAFE_DRAW_SAMPLE_COUNT):
        time_ns = index * BAFE_GROUND_MJ_DRAW_INTERVAL_NS
        poses.append(
            TimedPose(
                time_s=time_ns / 1_000_000_000,
                frame_id=BAFE_FRAME_ID,
                position_m=bafe_startup_geometry_position(
                    geometry_trajectory_id,
                    minimum_jerk_progress(index / BAFE_DRAW_INTERVAL_COUNT),
                ),
                quaternion_xyzw=BAFE_GROUND_QUATERNION_XYZW,
                phase="draw",
            )
        )

    final_draw_pose = poses[-1]
    poses.append(
        TimedPose(
            time_s=(
                BAFE_GROUND_MJ_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS
            ) / 1_000_000_000,
            frame_id=final_draw_pose.frame_id,
            position_m=final_draw_pose.position_m,
            quaternion_xyzw=final_draw_pose.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_bafe_ground_mj_trajectories() -> dict[str, tuple[TimedPose, ...]]:
    """Build all p10-frame downward BAFE candidates in stable ID order."""

    return {
        trajectory_id: build_bafe_ground_mj_trajectory(trajectory_id)
        for trajectory_id in BAFE_GROUND_MJ_TRAJECTORY_IDS
    }


def build_bafe_ground_mj_v2_trajectory(trajectory_id: str) -> tuple[TimedPose, ...]:
    """Build a slower p10-frame candidate for strict mock-recording evidence."""

    geometry_trajectory_id = _BAFE_GROUND_MJ_V2_SOURCE_IDS.get(trajectory_id)
    if geometry_trajectory_id is None:
        raise ValueError(
            f"unknown ground minimum-jerk v2 trajectory_id: {trajectory_id!r}"
        )
    poses: list[TimedPose] = []
    for index in range(BAFE_DRAW_SAMPLE_COUNT):
        time_ns = index * BAFE_GROUND_MJ_V2_DRAW_INTERVAL_NS
        poses.append(
            TimedPose(
                time_s=time_ns / 1_000_000_000,
                frame_id=BAFE_FRAME_ID,
                position_m=bafe_startup_geometry_position(
                    geometry_trajectory_id,
                    minimum_jerk_progress(index / BAFE_DRAW_INTERVAL_COUNT),
                ),
                quaternion_xyzw=BAFE_GROUND_QUATERNION_XYZW,
                phase="draw",
            )
        )

    final_draw_pose = poses[-1]
    poses.append(
        TimedPose(
            time_s=(
                BAFE_GROUND_MJ_V2_DRAW_DURATION_NS + BAFE_HOLD_DURATION_NS
            ) / 1_000_000_000,
            frame_id=final_draw_pose.frame_id,
            position_m=final_draw_pose.position_m,
            quaternion_xyzw=final_draw_pose.quaternion_xyzw,
            phase="hold",
        )
    )
    return tuple(poses)


def build_bafe_ground_mj_v2_trajectories() -> dict[str, tuple[TimedPose, ...]]:
    """Build all slower p10-frame BAFE candidates in stable ID order."""

    return {
        trajectory_id: build_bafe_ground_mj_v2_trajectory(trajectory_id)
        for trajectory_id in BAFE_GROUND_MJ_V2_TRAJECTORY_IDS
    }


def _render_trajectory_csv(poses: Sequence[TimedPose]) -> bytes:
    """Render authoritative poses to platform-independent UTF-8 CSV bytes."""

    stream = StringIO(newline="")
    writer = csv.DictWriter(
        stream,
        fieldnames=SOURCE_COLUMNS,
        lineterminator="\n",
    )
    writer.writeheader()
    for pose in poses:
        writer.writerow(
            {
                "time_s": f"{pose.time_s:.9f}",
                "frame_id": pose.frame_id,
                "tip_x_m": f"{pose.position_m[0]:.9f}",
                "tip_y_m": f"{pose.position_m[1]:.9f}",
                "tip_z_m": f"{pose.position_m[2]:.9f}",
                "tip_qx": f"{pose.quaternion_xyzw[0]:.12f}",
                "tip_qy": f"{pose.quaternion_xyzw[1]:.12f}",
                "tip_qz": f"{pose.quaternion_xyzw[2]:.12f}",
                "tip_qw": f"{pose.quaternion_xyzw[3]:.12f}",
                "phase": pose.phase,
            }
        )
    return stream.getvalue().encode("utf-8")


def render_acceptance_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one primary acceptance trajectory to canonical CSV bytes."""

    return _render_trajectory_csv(build_acceptance_trajectory(trajectory_id))


def render_recovery_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one replacement candidate while preserving all earlier CSV bytes."""

    return _render_trajectory_csv(build_recovery_trajectory(trajectory_id))


def render_new_board_fig14_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one new-board Fig. 14-inspired authority to canonical CSV bytes."""

    return _render_trajectory_csv(build_new_board_fig14_trajectory(trajectory_id))


def render_new_board_continuous_trajectory_csv(
    trajectory_id: str,
    *,
    orientation_motion: NewBoardOrientationMotion | None = None,
) -> bytes:
    """Render one continuously informative new-board authority."""

    return _render_trajectory_csv(
        build_new_board_continuous_trajectory(
            trajectory_id,
            orientation_motion=orientation_motion,
        )
    )


def render_new_board_continuous_zplus26_trajectory_csv(
    trajectory_id: str,
) -> bytes:
    """Render one +26 mm translated authority without changing its source."""

    return _render_trajectory_csv(
        build_new_board_continuous_zplus26_trajectory(trajectory_id)
    )


def render_new_board_continuous_zplus26_up50_trajectory_csv(
    trajectory_id: str,
) -> bytes:
    """Render one +26 mm authority translated by -50 mm in plane1 Z."""

    return _render_trajectory_csv(
        build_new_board_continuous_zplus26_up50_trajectory(trajectory_id)
    )


def render_bafe_startup_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one paper-matched BAFE trajectory to canonical source CSV bytes."""

    return _render_trajectory_csv(build_bafe_startup_trajectory(trajectory_id))


def render_bafe_downward_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one downward-pointing BAFE replacement candidate."""

    return _render_trajectory_csv(build_bafe_downward_trajectory(trajectory_id))


def render_bafe_downward_mj_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one slow downward minimum-jerk BAFE candidate."""

    return _render_trajectory_csv(build_bafe_downward_mj_trajectory(trajectory_id))


def render_bafe_ground_mj_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one p10-frame downward minimum-jerk BAFE candidate."""

    return _render_trajectory_csv(build_bafe_ground_mj_trajectory(trajectory_id))


def render_bafe_ground_mj_v2_trajectory_csv(trajectory_id: str) -> bytes:
    """Render one slower p10-frame downward minimum-jerk candidate."""

    return _render_trajectory_csv(build_bafe_ground_mj_v2_trajectory(trajectory_id))


def generate_acceptance_trajectories(output_directory: str | Path) -> tuple[Path, ...]:
    """Write exactly the four authoritative acceptance CSVs to ``output_directory``."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    generated_paths = []
    for trajectory_id in ACCEPTANCE_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        path.write_bytes(render_acceptance_trajectory_csv(trajectory_id))
        generated_paths.append(path)
    return tuple(generated_paths)


def generate_recovery_trajectories(output_directory: str | Path) -> tuple[Path, ...]:
    """Write new candidate files without opening or replacing earlier authorities."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    generated_paths = []
    for trajectory_id in RECOVERY_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        if path.exists():
            if path.read_bytes() != render_recovery_trajectory_csv(trajectory_id):
                raise FileExistsError(
                    f"refusing to overwrite authoritative recovery trajectory: {path}"
                )
        else:
            path.write_bytes(render_recovery_trajectory_csv(trajectory_id))
        generated_paths.append(path)
    return tuple(generated_paths)


def generate_new_board_fig14_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write new-board authorities without replacing any differing CSV bytes."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    planned_outputs: list[tuple[Path, bytes]] = []
    for trajectory_id in NEW_BOARD_FIG14_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_new_board_fig14_trajectory_csv(trajectory_id)
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise FileExistsError(
                "refusing to replace non-regular new-board Fig. 14 authority: "
                f"{path}"
            )
        if path.exists():
            if path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite authoritative new-board Fig. 14 "
                    f"trajectory: {path}"
                )
        planned_outputs.append((path, payload))

    for path, payload in planned_outputs:
        if path.exists():
            continue
        try:
            with path.open("xb") as stream:
                stream.write(payload)
        except FileExistsError:
            if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite concurrently created new-board Fig. 14 "
                    f"authority: {path}"
                ) from None
    return tuple(path for path, _ in planned_outputs)


def generate_new_board_continuous_trajectories(
    output_directory: str | Path,
    *,
    orientation_motion: NewBoardOrientationMotion | None = None,
) -> tuple[Path, ...]:
    """Write continuous new-board authorities without replacing differing bytes."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    planned_outputs: list[tuple[Path, bytes]] = []
    for trajectory_id in NEW_BOARD_CONTINUOUS_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_new_board_continuous_trajectory_csv(
            trajectory_id,
            orientation_motion=orientation_motion,
        )
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise FileExistsError(
                "refusing to replace non-regular continuous authority: " f"{path}"
            )
        if path.exists() and path.read_bytes() != payload:
            raise FileExistsError(
                "refusing to overwrite authoritative continuous trajectory: "
                f"{path}"
            )
        planned_outputs.append((path, payload))

    for path, payload in planned_outputs:
        if path.exists():
            continue
        try:
            with path.open("xb") as stream:
                stream.write(payload)
        except FileExistsError:
            if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite concurrently created continuous authority: "
                    f"{path}"
                ) from None
    return tuple(path for path, _ in planned_outputs)


def generate_new_board_continuous_zplus26_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write +26 mm authorities without replacing any differing CSV bytes."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    planned_outputs: list[tuple[Path, bytes]] = []
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_new_board_continuous_zplus26_trajectory_csv(trajectory_id)
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise FileExistsError(
                "refusing to replace non-regular +26 mm authority: " f"{path}"
            )
        if path.exists() and path.read_bytes() != payload:
            raise FileExistsError(
                "refusing to overwrite authoritative +26 mm trajectory: "
                f"{path}"
            )
        planned_outputs.append((path, payload))

    for path, payload in planned_outputs:
        if path.exists():
            continue
        try:
            with path.open("xb") as stream:
                stream.write(payload)
        except FileExistsError:
            if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite concurrently created +26 mm authority: "
                    f"{path}"
                ) from None
    return tuple(path for path, _ in planned_outputs)


def generate_new_board_continuous_zplus26_up50_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write the -50 mm Z translations without replacing differing bytes."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    planned_outputs: list[tuple[Path, bytes]] = []
    for trajectory_id in NEW_BOARD_CONTINUOUS_ZPLUS26_UP50_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_new_board_continuous_zplus26_up50_trajectory_csv(
            trajectory_id
        )
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise FileExistsError(
                "refusing to replace non-regular +26 mm/up 50 mm authority: "
                f"{path}"
            )
        if path.exists() and path.read_bytes() != payload:
            raise FileExistsError(
                "refusing to overwrite authoritative +26 mm/up 50 mm trajectory: "
                f"{path}"
            )
        planned_outputs.append((path, payload))

    for path, payload in planned_outputs:
        if path.exists():
            continue
        try:
            with path.open("xb") as stream:
                stream.write(payload)
        except FileExistsError:
            if path.is_symlink() or not path.is_file() or path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite concurrently created +26 mm/up 50 mm "
                    f"authority: {path}"
                ) from None
    return tuple(path for path, _ in planned_outputs)


def generate_bafe_startup_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write the four paper-matched BAFE source trajectories."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    generated_paths = []
    for trajectory_id in BAFE_STARTUP_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        path.write_bytes(render_bafe_startup_trajectory_csv(trajectory_id))
        generated_paths.append(path)
    return tuple(generated_paths)


def generate_bafe_downward_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write new downward candidates without rewriting an existing authority."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    generated_paths = []
    for trajectory_id in BAFE_DOWNWARD_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_bafe_downward_trajectory_csv(trajectory_id)
        if path.exists():
            if path.read_bytes() != payload:
                raise FileExistsError(
                    f"refusing to overwrite authoritative downward trajectory: {path}"
                )
        else:
            path.write_bytes(payload)
        generated_paths.append(path)
    return tuple(generated_paths)


def generate_bafe_downward_mj_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write slow downward candidates without rewriting an existing authority."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    generated_paths = []
    for trajectory_id in BAFE_DOWNWARD_MJ_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_bafe_downward_mj_trajectory_csv(trajectory_id)
        if path.exists():
            if path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite authoritative downward minimum-jerk "
                    f"trajectory: {path}"
                )
        else:
            path.write_bytes(payload)
        generated_paths.append(path)
    return tuple(generated_paths)


def generate_bafe_ground_mj_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write p10-frame downward candidates without replacing an authority."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    generated_paths = []
    for trajectory_id in BAFE_GROUND_MJ_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_bafe_ground_mj_trajectory_csv(trajectory_id)
        if path.exists():
            if path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite authoritative ground minimum-jerk "
                    f"trajectory: {path}"
                )
        else:
            path.write_bytes(payload)
        generated_paths.append(path)
    return tuple(generated_paths)


def generate_bafe_ground_mj_v2_trajectories(
    output_directory: str | Path,
) -> tuple[Path, ...]:
    """Write slower p10-frame candidates without replacing an authority."""

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    generated_paths = []
    for trajectory_id in BAFE_GROUND_MJ_V2_TRAJECTORY_IDS:
        path = destination / f"{trajectory_id}.csv"
        payload = render_bafe_ground_mj_v2_trajectory_csv(trajectory_id)
        if path.exists():
            if path.read_bytes() != payload:
                raise FileExistsError(
                    "refusing to overwrite authoritative ground minimum-jerk v2 "
                    f"trajectory: {path}"
                )
        else:
            path.write_bytes(payload)
        generated_paths.append(path)
    return tuple(generated_paths)


def circle(
    frame_id: str,
    center_m: Sequence[float],
    radius_m: float,
    duration_s: float,
    point_count: int,
    quaternion_xyzw: Sequence[float],
) -> tuple[TimedPose, ...]:
    if radius_m <= 0.0 or duration_s <= 0.0 or point_count < 4:
        raise ValueError("circle requires positive radius/duration and at least four points")
    cx, cy, cz = (float(value) for value in center_m)
    poses = []
    for index in range(point_count + 1):
        ratio = index / point_count
        angle = 2.0 * pi * ratio
        poses.append(
            TimedPose(
                time_s=duration_s * ratio,
                frame_id=frame_id,
                position_m=(cx + radius_m * sin(angle), cy + radius_m * cos(angle), cz),
                quaternion_xyzw=tuple(quaternion_xyzw),
                phase="draw",
            )
        )
    return tuple(poses)


def main(argv: Sequence[str] | None = None) -> int:
    """Regenerate the checked-in acceptance CSVs from a development checkout."""

    repository_root = Path(__file__).resolve().parents[3]
    default_output = repository_root / "environments" / "lab_wall_a" / "trajectories"
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument(
        "output_directory",
        nargs="?",
        type=Path,
        default=default_output,
        help="destination directory (defaults to the lab_wall_a trajectory directory)",
    )
    arguments = parser.parse_args(argv)
    generate_acceptance_trajectories(arguments.output_directory)
    generate_recovery_trajectories(arguments.output_directory)
    generate_bafe_startup_trajectories(arguments.output_directory)
    generate_bafe_downward_trajectories(arguments.output_directory)
    generate_bafe_downward_mj_trajectories(arguments.output_directory)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
