"""Fail-closed live TCP viewer and small collision-planned manual moves.

The module deliberately has no ROS imports at import time.  This keeps its
parsing and rigid-transform helpers testable on the development Mac; ROS 2 and
MoveIt messages are loaded only when :class:`RosTcpSession` is constructed.
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

import numpy as np

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.math3d import (
    inverse_pose,
    matrix_pose,
    normalize_quaternion_xyzw,
    pose_matrix,
    quaternion_angular_distance_deg,
)
from hx_waypoint_curobo.models import TimedPose
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_curobo.setup_config import rotvec_to_quaternion_xyzw

from .planning_scene_publisher import (
    _build_apply_request,
    load_planning_scene_definition,
)


MAX_JOG_STEP_M = 0.005
MAX_SERVO_LINEAR_SPEED_M_S = 0.020
MOVE_ACTION = "/move_action"
EXECUTE_TRAJECTORY_ACTION = "/execute_trajectory"
PLAYBACK_ACTION = "/hx_waypoint/play_recording"
GET_SCENE_SERVICE = "/get_planning_scene"
COMPUTE_IK_SERVICE = "/compute_ik"
LIST_CONTROLLERS_SERVICE = "/controller_manager/list_controllers"
SERVO_TWIST_TOPIC = "/servo_node/delta_twist_cmds"
SERVO_STATUS_TOPIC = "/servo_node/status"
SERVO_COMMAND_TYPE_SERVICE = "/servo_node/switch_command_type"
SERVO_PAUSE_SERVICE = "/servo_node/pause_servo"
UI_REFRESH_HZ = 5.0
UI_REFRESH_PERIOD_S = 1.0 / UI_REFRESH_HZ
MANUAL_MOVEIT_CONTROLLER = "scaled_joint_trajectory_controller"
MAX_TARGET_DERIVATION_JOINT_DRIFT_RAD = 5.0e-4
# The Jazzy MoveGroup kinematics capability forwards whole timeout seconds to
# the solver; a sub-second duration can therefore become zero at that layer.
MANUAL_IK_TIMEOUT_S = 1.0
MANUAL_JOINT_GOAL_TOLERANCE_RAD = 1.0e-4
SERVO_CANONICAL_MARGIN_RAD = math.radians(2.0)
_SERVO_FATAL_STATUS_CODES = frozenset((-1, 2, 5, 6))


@dataclass(frozen=True)
class AbsolutePoseCommand:
    frame: str
    position_m: tuple[float, float, float]
    rotation_vector_rad: tuple[float, float, float]

    @property
    def quaternion_xyzw(self) -> tuple[float, float, float, float]:
        return rotvec_to_quaternion_xyzw(self.rotation_vector_rad)


@dataclass(frozen=True)
class AbsolutePositionCommand:
    """Absolute rod-tip position while retaining the live tip orientation."""

    frame: str
    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class MotionDelta:
    translation_m: float
    rotation_deg: float


@dataclass(frozen=True)
class AttachedRodObservation:
    object_id: str
    link_name: str
    header_frame: str
    is_cylinder: bool
    dimensions_m: tuple[float, ...]
    center_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    touch_links: tuple[str, ...]


@dataclass(frozen=True)
class WorldCuboidObservation:
    object_id: str
    frame_id: str
    is_box: bool
    dimensions_m: tuple[float, ...]
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class SceneComparison:
    missing_world_ids: tuple[str, ...]
    missing_rod: bool
    rod_geometry_errors: tuple[str, ...]
    unexpected_world_ids: tuple[str, ...] = ()
    world_geometry_errors: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return (
            not self.missing_world_ids
            and not self.missing_rod
            and not self.rod_geometry_errors
            and not self.unexpected_world_ids
            and not self.world_geometry_errors
        )

    def error_message(self) -> str:
        parts: list[str] = []
        if self.missing_world_ids:
            parts.append("missing world objects: " + ", ".join(self.missing_world_ids))
        if self.missing_rod:
            parts.append("authoritative attached rod is missing")
        if self.rod_geometry_errors:
            parts.append("attached rod mismatch: " + "; ".join(self.rod_geometry_errors))
        if self.world_geometry_errors:
            parts.append("world geometry mismatch: " + "; ".join(self.world_geometry_errors))
        if self.unexpected_world_ids:
            parts.append(
                "unexpected world objects: " + ", ".join(self.unexpected_world_ids)
            )
        return "; ".join(parts) if parts else "scene matches"


@dataclass(frozen=True)
class JointSnapshot:
    received_monotonic_s: float
    positions_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...] | None


@dataclass(frozen=True)
class MotionWorkerSnapshot:
    """Thread-safe, renderable state of the one-command motion worker."""

    running: bool
    generation: int
    message: str
    error: str


# Stable, concise public name used by callers and pure unit tests.
PoseTarget = AbsolutePoseCommand


def parse_absolute_pose(text: str) -> AbsolutePoseCommand:
    """Parse the intentionally explicit ``base|plane1 p[x,y,z,rx,ry,rz]`` form."""

    value = text.strip()
    pieces = value.split(None, 1)
    if len(pieces) != 2 or pieces[0].lower() not in {"base", "plane1"}:
        raise ValueError("absolute pose must explicitly start with 'base' or 'plane1'")
    frame = pieces[0].lower()
    pose_text = pieces[1].strip()
    if not pose_text.startswith("p[") or not pose_text.endswith("]"):
        raise ValueError("pose must use UR syntax p[x,y,z,rx,ry,rz]")
    fields = [field.strip() for field in pose_text[2:-1].split(",")]
    if len(fields) != 6 or any(not field for field in fields):
        raise ValueError("UR pose must contain exactly six comma-separated numbers")
    try:
        numbers = tuple(float(field) for field in fields)
    except ValueError as error:
        raise ValueError("UR pose contains a non-number") from error
    if not all(math.isfinite(number) for number in numbers):
        raise ValueError("UR pose values must be finite")
    return AbsolutePoseCommand(
        frame=frame,
        position_m=numbers[:3],  # type: ignore[arg-type]
        rotation_vector_rad=numbers[3:],  # type: ignore[arg-type]
    )


def parse_pose_command(text: str) -> PoseTarget:
    return parse_absolute_pose(text)


def quaternion_to_rotvec_xyzw(values: Sequence[float]) -> tuple[float, float, float]:
    """Convert xyzw to the shortest canonical UR rotation vector."""

    quaternion = normalize_quaternion_xyzw(values)
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    vector_norm = float(np.linalg.norm(quaternion[:3]))
    if vector_norm <= 1.0e-15:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, float(quaternion[3]))
    return tuple(float(component * angle / vector_norm) for component in quaternion[:3])  # type: ignore[return-value]


def quaternion_to_rotvec(values: Sequence[float]) -> tuple[float, float, float]:
    return quaternion_to_rotvec_xyzw(values)


def compose_transform(parent_to_child: np.ndarray, child_to_target: np.ndarray) -> np.ndarray:
    left = np.asarray(parent_to_child, dtype=float)
    right = np.asarray(child_to_target, dtype=float)
    if left.shape != (4, 4) or right.shape != (4, 4):
        raise ValueError("transforms must be 4x4 matrices")
    return left @ right


def inverse_transform(transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix")
    return inverse_pose(matrix)


def ur_pose_text(transform: np.ndarray) -> str:
    position, quaternion = matrix_pose(transform)
    rotvec = quaternion_to_rotvec_xyzw(quaternion)
    values = (*position, *rotvec)
    return "p[" + ", ".join(f"{value:.6f}" for value in values) + "]"


def absolute_tip_target_in_base(
    command: AbsolutePoseCommand,
    base_to_plane1: np.ndarray,
) -> np.ndarray:
    target_in_frame = pose_matrix(command.position_m, command.quaternion_xyzw)
    if command.frame == "base":
        return target_in_frame
    if command.frame == "plane1":
        return np.asarray(base_to_plane1, dtype=float) @ target_in_frame
    raise ValueError(f"unsupported command frame {command.frame!r}")


def absolute_position_target_in_base(
    command: AbsolutePositionCommand,
    current_base_to_tip: np.ndarray,
    base_to_plane1: np.ndarray,
) -> np.ndarray:
    """Replace XYZ in ``command.frame`` and preserve the live tip orientation.

    The orientation is sampled as part of the same motion command that derives
    the target.  It is therefore never copied from stale UI telemetry.
    """

    if command.frame not in {"base", "plane1"}:
        raise ValueError(f"unsupported command frame {command.frame!r}")
    position = tuple(float(value) for value in command.position_m)
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        raise ValueError("absolute XYZ must contain exactly three finite values")
    current = np.asarray(current_base_to_tip, dtype=float)
    base_to_frame = (
        np.eye(4)
        if command.frame == "base"
        else np.asarray(base_to_plane1, dtype=float)
    )
    if (
        current.shape != (4, 4)
        or base_to_frame.shape != (4, 4)
        or not np.all(np.isfinite(current))
        or not np.all(np.isfinite(base_to_frame))
    ):
        raise ValueError("absolute-position transforms must be finite 4x4 matrices")
    frame_to_tip = inverse_pose(base_to_frame) @ current
    frame_to_tip = frame_to_tip.copy()
    frame_to_tip[:3, 3] = position
    return base_to_frame @ frame_to_tip


def rod_tip_target_to_tool0(
    tool_builder: RobotToolBuilder,
    base_to_rod_tip: np.ndarray,
    *,
    frame_id: str = "base",
) -> TimedPose:
    """Use the environment's authoritative tool transform for a MoveIt target."""

    position, quaternion = matrix_pose(base_to_rod_tip)
    return tool_builder.tip_pose_to_tool_pose(
        TimedPose(
            time_s=0.0,
            frame_id=frame_id,
            position_m=position,  # type: ignore[arg-type]
            quaternion_xyzw=quaternion,  # type: ignore[arg-type]
            phase="approach",
        )
    )


def tip_target_to_tool_target(
    tool_builder: RobotToolBuilder,
    base_to_rod_tip: np.ndarray,
    *,
    frame_id: str = "base",
) -> TimedPose:
    return rod_tip_target_to_tool0(
        tool_builder, base_to_rod_tip, frame_id=frame_id
    )


_JOG_KEYS: Mapping[str, tuple[float, float, float]] = {
    "a": (-1.0, 0.0, 0.0),
    "left": (-1.0, 0.0, 0.0),
    "d": (1.0, 0.0, 0.0),
    "right": (1.0, 0.0, 0.0),
    "w": (0.0, 1.0, 0.0),
    "up": (0.0, 1.0, 0.0),
    "s": (0.0, -1.0, 0.0),
    "down": (0.0, -1.0, 0.0),
}
_Z_JOG_KEYS = frozenset(("r", "pageup", "f", "pagedown"))


@dataclass(frozen=True)
class TerminalKeyCodes:
    """The curses special-key values, passed explicitly to keep decoding testable."""

    left: int
    right: int
    up: int
    down: int
    page_up: int
    page_down: int


def decode_terminal_key(key_code: int, special: TerminalKeyCodes) -> str | None:
    """Map one curses key code to a semantic command; never synthesize repeats."""

    special_commands = {
        special.left: "left",
        special.right: "right",
        special.up: "up",
        special.down: "down",
        special.page_up: "pageup",
        special.page_down: "pagedown",
    }
    if key_code in special_commands:
        return special_commands[key_code]
    if key_code == 27:
        return "escape"
    if key_code == ord(" "):
        return "arm"
    if key_code in (ord("\n"), ord("\r")):
        return "enter"
    if key_code in (8, 127):
        return "backspace"
    if 0 <= key_code <= 255:
        value = chr(key_code).lower()
        if value in {"a", "d", "w", "s", "r", "f"}:
            return value
        if value == "g":
            return "absolute"
        if value == "q":
            return "quit"
        if value == "t":
            return "toggle-frame"
        if value == "c":
            return "clear"
    return None


def jog_delta_for_key(
    key: str,
    step_m: float,
    jog_frame: str = "base",
) -> tuple[float, float, float]:
    step = float(step_m)
    if not math.isfinite(step) or step <= 0.0 or step > MAX_JOG_STEP_M:
        raise ValueError("jog step must be finite, positive, and no greater than 5 mm")
    normalized = key.strip().lower()
    if jog_frame not in {"base", "plane1"}:
        raise ValueError("jog frame must be base or plane1")
    if normalized in {"r", "pageup"}:
        # Work-frame Z is environment-specific.  In MLA only, -plane1 Z is
        # also physical raise; other environments must interpret the axis
        # label literally.
        return (0.0, 0.0, -step if jog_frame == "plane1" else step)
    if normalized in {"f", "pagedown"}:
        return (0.0, 0.0, step if jog_frame == "plane1" else -step)
    if normalized not in _JOG_KEYS:
        raise ValueError(f"unsupported jog key {key!r}")
    return tuple(step * component for component in _JOG_KEYS[normalized])  # type: ignore[return-value]


def servo_jog_velocity(
    key: str,
    jog_frame: str,
    speed_m_s: float,
) -> tuple[float, float, float]:
    """Return one bounded Cartesian Servo velocity in the selected frame.

    Only translation is exposed by the manual UI.  With angular velocity held
    exactly at zero, ``tool0`` and the rigidly attached ``rod_tip`` have the
    same translational velocity even though ``rod_tip`` is not a RobotModel
    link.  Plane-frame R/F retain this project's established physical-height
    convention: R is local -Z and F is local +Z.
    """

    speed = float(speed_m_s)
    if (
        not math.isfinite(speed)
        or speed <= 0.0
        or speed > MAX_SERVO_LINEAR_SPEED_M_S + 1.0e-12
    ):
        raise ValueError(
            "Servo jog speed must be finite, positive, and no greater than "
            f"{MAX_SERVO_LINEAR_SPEED_M_S * 1000.0:.0f} mm/s"
        )
    # The discrete helper has a deliberately smaller 5 mm upper bound, so use
    # its axis mapping with a unit scale rather than feeding it a velocity.
    unit_direction = jog_delta_for_key(key, 1.0e-3, jog_frame)
    return tuple(speed * component / 1.0e-3 for component in unit_direction)  # type: ignore[return-value]


def jog_tip_target_in_base(
    current_base_to_tip: np.ndarray,
    base_to_jog_frame: np.ndarray,
    delta_in_jog_frame_m: Sequence[float],
) -> np.ndarray:
    """Translate in the chosen frame while preserving the actual TCP orientation."""

    delta = np.asarray(delta_in_jog_frame_m, dtype=float)
    if delta.shape != (3,) or not np.all(np.isfinite(delta)):
        raise ValueError("jog delta must contain three finite values")
    base_to_frame = np.asarray(base_to_jog_frame, dtype=float)
    frame_to_tip = inverse_pose(base_to_frame) @ np.asarray(current_base_to_tip, dtype=float)
    frame_to_tip = frame_to_tip.copy()
    frame_to_tip[:3, 3] += delta
    return base_to_frame @ frame_to_tip


def jog_pose(
    current_base_to_tip: np.ndarray,
    base_to_jog_frame: np.ndarray,
    key: str,
    step_m: float,
    jog_frame: str = "base",
) -> np.ndarray:
    return jog_tip_target_in_base(
        current_base_to_tip,
        base_to_jog_frame,
        jog_delta_for_key(key, step_m, jog_frame),
    )


class OneMotionWorker:
    """Run at most one motion operation and deliberately provide no queue.

    Cancellation calls are repeated while the same generation remains active.
    This matters because Escape may be pressed during MoveIt's preflight, before
    an action goal handle exists; the later retry then cancels the actual goal.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._cancel_thread: threading.Thread | None = None
        self._generation = 0
        self._running = False
        self._message = "idle"
        self._error = ""
        self._cancel_requested_generation = 0
        self._cancel_event: threading.Event | None = None

    def snapshot(self) -> MotionWorkerSnapshot:
        with self._lock:
            cancel_running = (
                self._cancel_thread is not None
                and self._cancel_thread.is_alive()
            )
            return MotionWorkerSnapshot(
                running=self._running or cancel_running,
                generation=self._generation,
                message=self._message,
                error=self._error,
            )

    def start(
        self,
        operation: Callable[[threading.Event], MotionDelta],
        *,
        label: str,
    ) -> bool:
        """Start immediately or return False; a rejected operation is not queued."""

        with self._lock:
            if self._running or (
                self._cancel_thread is not None
                and self._cancel_thread.is_alive()
            ):
                return False
            self._generation += 1
            generation = self._generation
            self._running = True
            self._message = f"running: {label}"
            self._error = ""
            cancel_event = threading.Event()
            self._cancel_event = cancel_event
            thread = threading.Thread(
                target=self._run,
                args=(generation, operation, cancel_event),
                name=f"tcp-console-motion-{generation}",
                daemon=True,
            )
            self._thread = thread
            thread.start()
            return True

    def _run(
        self,
        generation: int,
        operation: Callable[[threading.Event], MotionDelta],
        cancel_event: threading.Event,
    ) -> None:
        try:
            delta = operation(cancel_event)
        except Exception as error:
            with self._lock:
                if generation == self._generation:
                    self._running = False
                    self._message = "motion failed"
                    self._error = str(error)
            return
        with self._lock:
            if generation == self._generation:
                self._running = False
                self._message = (
                    f"completed: {delta.translation_m * 1000.0:.3f} mm, "
                    f"{delta.rotation_deg:.3f} deg"
                )
                self._error = ""

    def _same_generation_running(self, generation: int) -> bool:
        with self._lock:
            return self._running and self._generation == generation

    def request_cancel(self, cancel_active: Callable[[], None]) -> bool:
        """Cancel the current generation asynchronously, including late goals."""

        with self._lock:
            if not self._running:
                return False
            generation = self._generation
            cancel_event = self._cancel_event
            if self._cancel_requested_generation == generation:
                return True
            self._cancel_requested_generation = generation
            self._message = "cancellation requested"
            if cancel_event is not None:
                cancel_event.set()
        cancel_thread = threading.Thread(
            target=self._cancel_until_finished,
            args=(generation, cancel_active),
            name=f"tcp-console-cancel-{generation}",
            daemon=True,
        )
        with self._lock:
            self._cancel_thread = cancel_thread
            cancel_thread.start()
        return True

    def _cancel_until_finished(
        self,
        generation: int,
        cancel_active: Callable[[], None],
    ) -> None:
        try:
            while self._same_generation_running(generation):
                try:
                    cancel_active()
                except Exception:
                    # The motion worker owns the final, user-visible MoveIt error.
                    pass
                time.sleep(0.05)
        finally:
            with self._lock:
                if self._cancel_thread is threading.current_thread():
                    self._cancel_thread = None

    def join(self, timeout_s: float | None) -> None:
        deadline = None if timeout_s is None else time.monotonic() + max(0.0, timeout_s)
        with self._lock:
            thread = self._thread
        if thread is not None:
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            thread.join(timeout=remaining)
        with self._lock:
            cancel_thread = self._cancel_thread
        if cancel_thread is not None and cancel_thread is not threading.current_thread():
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            cancel_thread.join(timeout=remaining)


def motion_delta(current: np.ndarray, target: np.ndarray) -> MotionDelta:
    current_matrix = np.asarray(current, dtype=float)
    target_matrix = np.asarray(target, dtype=float)
    if (
        current_matrix.shape != (4, 4)
        or target_matrix.shape != (4, 4)
        or not np.all(np.isfinite(current_matrix))
        or not np.all(np.isfinite(target_matrix))
    ):
        raise ValueError("motion transforms must be finite 4x4 matrices")
    current_position, current_quaternion = matrix_pose(current_matrix)
    target_position, target_quaternion = matrix_pose(target_matrix)
    delta = MotionDelta(
        translation_m=float(
            np.linalg.norm(np.asarray(target_position) - np.asarray(current_position))
        ),
        rotation_deg=quaternion_angular_distance_deg(
            current_quaternion, target_quaternion
        ),
    )
    if not math.isfinite(delta.translation_m) or not math.isfinite(delta.rotation_deg):
        raise ValueError("motion delta is not finite")
    return delta


def enforce_motion_delta(
    current: np.ndarray,
    target: np.ndarray,
    *,
    max_translation_m: float,
    max_rotation_deg: float,
) -> MotionDelta:
    delta = motion_delta(current, target)
    if delta.translation_m > max_translation_m + 1.0e-12:
        raise ValueError(
            f"single translation {delta.translation_m * 1000.0:.3f} mm exceeds "
            f"limit {max_translation_m * 1000.0:.3f} mm"
        )
    if delta.rotation_deg > max_rotation_deg + 1.0e-12:
        raise ValueError(
            f"single rotation {delta.rotation_deg:.3f} deg exceeds "
            f"limit {max_rotation_deg:.3f} deg"
        )
    return delta


def enforce_post_motion_result(
    current: np.ndarray,
    target: np.ndarray,
    actual: np.ndarray,
    *,
    max_position_error_m: float,
    max_orientation_error_deg: float,
) -> MotionDelta:
    """Validate independent actual FK and return the measured displacement."""

    actual_matrix = np.asarray(actual, dtype=float)
    if actual_matrix.shape != (4, 4) or not np.all(np.isfinite(actual_matrix)):
        raise RuntimeError("actual rod_tip FK is not a finite 4x4 transform")
    target_error = motion_delta(target, actual)
    if (
        target_error.translation_m > max_position_error_m
        or target_error.rotation_deg > max_orientation_error_deg
    ):
        raise RuntimeError(
            "motion completed but actual rod_tip error is "
            f"{target_error.translation_m * 1000.0:.3f} mm / "
            f"{target_error.rotation_deg:.3f} deg"
        )

    commanded_vector = np.asarray(target, dtype=float)[:3, 3] - np.asarray(
        current, dtype=float
    )[:3, 3]
    commanded_distance = float(np.linalg.norm(commanded_vector))
    if commanded_distance > 1.0e-12:
        actual_vector = np.asarray(actual, dtype=float)[:3, 3] - np.asarray(
            current, dtype=float
        )[:3, 3]
        progress_m = float(
            np.dot(actual_vector, commanded_vector / commanded_distance)
        )
        required_progress_m = max(
            0.0, commanded_distance - max_position_error_m
        )
        if progress_m + 1.0e-12 < required_progress_m:
            raise RuntimeError(
                "motion completed without enough actual FK progress along the "
                f"commanded direction ({progress_m * 1000.0:.3f} / "
                f"{required_progress_m * 1000.0:.3f} mm)"
            )
    return motion_delta(current, actual)


def manual_joint_path_bounds(
    joint_names: Sequence[str],
    start_positions_rad: Sequence[float],
    canonical_ranges_rad: Mapping[str, Sequence[float]],
    *,
    max_excursion_rad: float,
) -> tuple[tuple[float, float], ...]:
    """Return fail-closed per-joint path bounds around the actual start state.

    A Cartesian pose can have distant IK solutions. Intersecting a small
    start-centred interval with the environment's canonical branch limits keeps
    a one-millimetre jog from turning into an unrelated large joint motion.
    """

    if len(joint_names) != len(start_positions_rad):
        raise ValueError("joint names and start positions must have equal length")
    excursion = float(max_excursion_rad)
    if not math.isfinite(excursion) or excursion <= 0.0 or excursion > math.pi:
        raise ValueError("maximum joint excursion must be in (0, pi] radians")

    bounds: list[tuple[float, float]] = []
    for name, raw_position in zip(joint_names, start_positions_rad, strict=True):
        position = float(raw_position)
        if not math.isfinite(position):
            raise ValueError(f"start position for {name!r} is not finite")
        lower = position - excursion
        upper = position + excursion
        configured = canonical_ranges_rad.get(name)
        if configured is not None:
            if len(configured) != 2:
                raise ValueError(f"canonical range for {name!r} must have two values")
            canonical_lower = float(configured[0])
            canonical_upper = float(configured[1])
            if (
                not math.isfinite(canonical_lower)
                or not math.isfinite(canonical_upper)
                or canonical_lower >= canonical_upper
            ):
                raise ValueError(f"canonical range for {name!r} is invalid")
            if position < canonical_lower - 1.0e-9 or position > canonical_upper + 1.0e-9:
                raise RuntimeError(
                    f"current {name}={position:.6f} rad is outside the configured "
                    f"manual branch [{canonical_lower:.6f}, {canonical_upper:.6f}]"
                )
            lower = max(lower, canonical_lower)
            upper = min(upper, canonical_upper)
        if lower > position or upper < position:
            raise RuntimeError(f"no valid manual path interval remains for {name!r}")
        bounds.append((lower, upper))
    return tuple(bounds)


def _quaternion_equal(
    left: Sequence[float], right: Sequence[float], tolerance: float
) -> bool:
    q0 = normalize_quaternion_xyzw(left)
    q1 = normalize_quaternion_xyzw(right)
    return bool(
        min(np.linalg.norm(q0 - q1), np.linalg.norm(q0 + q1)) <= tolerance
    )


def compare_scene_inventory(
    expected_world_ids: Iterable[Any],
    expected_rod: Any,
    actual_world_ids: Iterable[str],
    actual_rods: Mapping[str, AttachedRodObservation],
    *,
    actual_cuboids: Mapping[str, WorldCuboidObservation] | None = None,
    tolerance: float = 1.0e-6,
) -> SceneComparison:
    """Compare the safety-relevant scene inventory and complete rod geometry."""

    expected_items = tuple(expected_world_ids)
    expected_cuboids = {
        str(item.object_id): item for item in expected_items if hasattr(item, "object_id")
    }
    expected_world = {
        str(item.object_id) if hasattr(item, "object_id") else str(item)
        for item in expected_items
    }
    actual_world = set(actual_world_ids)
    missing = tuple(sorted(expected_world - actual_world))
    unexpected = tuple(sorted(actual_world - expected_world))
    world_errors: list[str] = []
    if actual_cuboids is not None:
        for object_id, expected in expected_cuboids.items():
            actual = actual_cuboids.get(object_id)
            if actual is None:
                continue
            prefix = f"{object_id}: "
            if actual.frame_id != expected.frame_id:
                world_errors.append(prefix + f"frame {actual.frame_id!r} != {expected.frame_id!r}")
            if not actual.is_box:
                world_errors.append(prefix + "primitive is not BOX")
            if len(actual.dimensions_m) != 3 or not np.allclose(
                actual.dimensions_m, expected.dimensions_m, atol=tolerance, rtol=0.0
            ):
                world_errors.append(prefix + "dimensions differ")
            if not np.allclose(
                actual.position_m, expected.position_m, atol=tolerance, rtol=0.0
            ):
                world_errors.append(prefix + "position differs")
            try:
                quaternion_matches = _quaternion_equal(
                    actual.quaternion_xyzw, expected.quaternion_xyzw, tolerance
                )
            except Exception:
                quaternion_matches = False
            if not quaternion_matches:
                world_errors.append(prefix + "orientation differs")
    observation = actual_rods.get(str(expected_rod.object_id))
    if observation is None:
        return SceneComparison(
            missing, True, (), unexpected, tuple(world_errors)
        )

    errors: list[str] = []
    if observation.link_name != expected_rod.link_name:
        errors.append(f"link {observation.link_name!r} != {expected_rod.link_name!r}")
    if observation.header_frame != expected_rod.link_name:
        errors.append("collision-object frame is not the rod parent link")
    if not observation.is_cylinder:
        errors.append("primitive is not CYLINDER")
    expected_dimensions = (float(expected_rod.length_m), float(expected_rod.radius_m))
    if len(observation.dimensions_m) != 2 or not np.allclose(
        observation.dimensions_m, expected_dimensions, atol=tolerance, rtol=0.0
    ):
        errors.append(f"dimensions {observation.dimensions_m!r} != {expected_dimensions!r}")
    if not np.allclose(
        observation.center_m, expected_rod.center_in_link_m, atol=tolerance, rtol=0.0
    ):
        errors.append("center pose differs")
    try:
        rod_quaternion_matches = _quaternion_equal(
            observation.quaternion_xyzw, expected_rod.quaternion_xyzw, tolerance
        )
    except Exception:
        rod_quaternion_matches = False
    if not rod_quaternion_matches:
        errors.append("orientation differs")
    if set(observation.touch_links) != set(expected_rod.touch_links):
        errors.append("touch_links differ")
    return SceneComparison(
        missing, False, tuple(errors), unexpected, tuple(world_errors)
    )


def forbidden_acm_allowances(
    matrix: Any,
    *,
    world_object_ids: Iterable[str],
    attached_rod_id: str,
    rod_touch_links: Iterable[str],
) -> tuple[str, ...]:
    """Return collision allowances that would weaken the manual safety scene."""

    names = tuple(str(name) for name in matrix.entry_names)
    # MoveIt only applies an ACM from a PlanningScene diff when entry_names is
    # non-empty.  Accepting an empty matrix here would make the goal inherit a
    # potentially changed parent ACM and reopen the verify/execute race.
    if not names:
        return ("allowed collision matrix is empty and cannot be bound to the goal",)
    if len(set(names)) != len(names) or len(matrix.entry_values) != len(names):
        return ("allowed collision matrix has malformed names/rows",)
    rows: list[tuple[bool, ...]] = []
    for row in matrix.entry_values:
        values = tuple(bool(value) for value in row.enabled)
        if len(values) != len(names):
            return ("allowed collision matrix has a malformed row",)
        rows.append(values)
    if any(
        rows[row][column] != rows[column][row]
        for row in range(len(names))
        for column in range(len(names))
    ):
        return ("allowed collision matrix is not symmetric",)
    if len(matrix.default_entry_names) != len(matrix.default_entry_values):
        return ("allowed collision matrix has malformed defaults",)
    if len(set(str(name) for name in matrix.default_entry_names)) != len(
        matrix.default_entry_names
    ):
        return ("allowed collision matrix has duplicate defaults",)
    defaults = {
        str(name): bool(value)
        for name, value in zip(
            matrix.default_entry_names,
            matrix.default_entry_values,
            strict=True,
        )
    }

    world = set(str(value) for value in world_object_ids)
    touch_links = set(str(value) for value in rod_touch_links)
    protected = world | {str(attached_rod_id)}
    errors: set[str] = set()
    # A true default for *any* name is unsafe.  In particular, a robot-link
    # default can silently allow collision with a protected world object that
    # is not represented by an explicit matrix entry.
    for name, enabled in defaults.items():
        if enabled:
            errors.add(f"default collision allowance is true for {name!r}")

    for row_index, left in enumerate(names):
        for column_index in range(row_index, len(names)):
            if not rows[row_index][column_index]:
                continue
            right = names[column_index]
            if left == right:
                continue
            if left in world or right in world:
                protected_world = left if left in world else right
                other = right if left in world else left
                errors.add(
                    f"world object {protected_world!r} is allowed to collide with "
                    f"{other!r}"
                )
                continue
            if attached_rod_id in {left, right}:
                other = right if left == attached_rod_id else left
                if other not in touch_links:
                    errors.add(
                        f"attached rod is allowed to collide with non-touch link {other!r}"
                    )
    return tuple(sorted(errors))


def _validate_positive(value: float, label: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return number


class RosTcpSession:
    """ROS-facing state and one-at-a-time MoveGroup execution."""

    def __init__(
        self,
        environment: Any,
        scene: Any,
        args: argparse.Namespace,
        *,
        context: Any | None = None,
    ) -> None:
        # Every ROS import is intentionally local.
        import rclpy
        from action_msgs.msg import GoalStatus
        from controller_manager_msgs.srv import ListControllers
        from geometry_msgs.msg import Pose, TwistStamped
        from hx_waypoint_interfaces.action import PlayRecording
        from moveit_msgs.action import ExecuteTrajectory, MoveGroup
        from moveit_msgs.msg import (
            AttachedCollisionObject,
            CollisionObject,
            Constraints,
            JointConstraint,
            MoveItErrorCodes,
            OrientationConstraint,
            PlanningScene,
            PlanningSceneComponents,
            PositionConstraint,
            ServoStatus,
        )
        from moveit_msgs.srv import (
            ApplyPlanningScene,
            GetPlanningScene,
            GetPositionIK,
            ServoCommandType,
        )
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import JointState
        from shape_msgs.msg import SolidPrimitive
        from std_srvs.srv import SetBool
        from tf2_ros import Buffer, TransformListener

        self.rclpy = rclpy
        self.types = {
            "Pose": Pose,
            "TwistStamped": TwistStamped,
            "ApplyPlanningScene": ApplyPlanningScene,
            "AttachedCollisionObject": AttachedCollisionObject,
            "CollisionObject": CollisionObject,
            "PlanningScene": PlanningScene,
            "ListControllers": ListControllers,
            "MoveGroup": MoveGroup,
            "ExecuteTrajectory": ExecuteTrajectory,
            "MoveItErrorCodes": MoveItErrorCodes,
            "Constraints": Constraints,
            "JointConstraint": JointConstraint,
            "OrientationConstraint": OrientationConstraint,
            "PlanningSceneComponents": PlanningSceneComponents,
            "PositionConstraint": PositionConstraint,
            "ServoStatus": ServoStatus,
            "GetPlanningScene": GetPlanningScene,
            "GetPositionIK": GetPositionIK,
            "ServoCommandType": ServoCommandType,
            "SetBool": SetBool,
            "GoalStatus": GoalStatus,
            "SolidPrimitive": SolidPrimitive,
            "Duration": Duration,
            "Time": Time,
        }
        self.environment = environment
        self.scene = scene
        self.args = args
        self.context = context
        self.environment_id = str(environment.environment_id)
        self.tool_builder = RobotToolBuilder(environment)
        self.base_frame = environment.setup_config.work_frame.parent_frame
        self.plane_frame = environment.setup_config.work_frame.child_frame
        work = environment.setup_config.work_frame.parent_to_child
        self.base_to_plane1 = pose_matrix(
            work.translation_m, work.quaternion_xyzw
        )
        self.node = rclpy.create_node("hx_waypoint_tcp_console", context=context)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self._lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._joint_history: deque[JointSnapshot] = deque(maxlen=300)
        self._active_goal: Any | None = None
        self._active_result_future: Any | None = None
        self._active_goal_generation = 0
        self._active_action_label = "MoveIt"
        self._active_result_request_in_progress = False
        self._pending_submission_generation = 0
        self._goal_generation_counter = 0
        self._cancel_supervisors: dict[int, threading.Thread] = {}
        self._motion_state_unknown = False
        self._validated_acm: Any | None = None
        self._servo_publish_lock = threading.Lock()
        self._last_servo_status: tuple[float, int, str] | None = None
        self._servo_configured = False
        self._servo_paused = True
        self._servo_generation_counter = 0
        self._active_servo_generation = 0
        self._active_servo_frame: str | None = None
        self._active_servo_velocity_m_s: tuple[float, float, float] | None = None
        self._servo_start_positions_rad: tuple[float, ...] | None = None
        self._servo_command_lock_held = False
        self.node.create_subscription(
            JointState, args.joint_state_topic, self._joint_callback,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            ServoStatus,
            SERVO_STATUS_TOPIC,
            self._servo_status_callback,
            10,
        )
        self.scene_client = self.node.create_client(GetPlanningScene, GET_SCENE_SERVICE)
        self.ik_client = self.node.create_client(GetPositionIK, COMPUTE_IK_SERVICE)
        self.controller_client = self.node.create_client(
            ListControllers, LIST_CONTROLLERS_SERVICE
        )
        self.servo_command_type_client = self.node.create_client(
            ServoCommandType, SERVO_COMMAND_TYPE_SERVICE
        )
        self.servo_pause_client = self.node.create_client(
            SetBool, SERVO_PAUSE_SERVICE
        )
        self.servo_twist_publisher = self.node.create_publisher(
            TwistStamped, SERVO_TWIST_TOPIC, 10
        )
        self.move_client = ActionClient(self.node, MoveGroup, MOVE_ACTION)
        self.execute_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            EXECUTE_TRAJECTORY_ACTION,
        )
        # Merely finding this server means the authoritative runtime is active;
        # manual control must then fail closed rather than compete for the UR.
        self.playback_client = (
            ActionClient(self.node, PlayRecording, PLAYBACK_ACTION)
            if args.enable_motion else None
        )
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

    def _joint_callback(self, message: Any) -> None:
        by_name = {
            name: index for index, name in enumerate(message.name)
            if index < len(message.position)
        }
        if any(name not in by_name for name in self.environment.joint_names):
            return
        positions = tuple(
            float(message.position[by_name[name]]) for name in self.environment.joint_names
        )
        velocities: tuple[float, ...] | None = None
        if len(message.velocity) >= len(message.name):
            velocities = tuple(
                float(message.velocity[by_name[name]]) for name in self.environment.joint_names
            )
        if not all(math.isfinite(value) for value in positions):
            return
        if velocities is not None and not all(math.isfinite(value) for value in velocities):
            velocities = None
        snapshot = JointSnapshot(time.monotonic(), positions, velocities)
        with self._lock:
            self._joint_history.append(snapshot)

    def _servo_status_callback(self, message: Any) -> None:
        received = time.monotonic()
        code = int(message.code)
        text = str(message.message).strip()
        with self._lock:
            self._last_servo_status = (received, code, text)

    def latest_joints(self) -> JointSnapshot | None:
        with self._lock:
            return self._joint_history[-1] if self._joint_history else None

    def _settled_snapshot(self, *, after_s: float = 0.0) -> JointSnapshot | None:
        now = time.monotonic()
        with self._lock:
            history = tuple(self._joint_history)
        if not history or now - history[-1].received_monotonic_s > self.args.state_freshness_s:
            return None
        # A post-motion check must accumulate a wholly new settle window; it
        # cannot reuse quiet samples captured before MoveGroup completed.
        if after_s > 0.0 and now - after_s < self.args.settle_window_s:
            return None
        cutoff = now - self.args.settle_window_s
        window = tuple(
            sample for sample in history
            if sample.received_monotonic_s >= max(cutoff, after_s)
        )
        if not window or window[0].received_monotonic_s > cutoff + 0.08:
            return None
        # Missing velocities are unknown motion, never evidence of a settled arm.
        if any(sample.velocities_rad_s is None for sample in window):
            return None
        if any(
            max(abs(value) for value in sample.velocities_rad_s or ())
            > self.args.settled_velocity_rad_s
            for sample in window
        ):
            return None
        positions = np.asarray([sample.positions_rad for sample in window], dtype=float)
        if float(np.max(np.ptp(positions, axis=0))) > self.args.settled_position_span_rad:
            return None
        return history[-1]

    def wait_settled(self, *, timeout_s: float, after_s: float = 0.0) -> JointSnapshot:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            snapshot = self._settled_snapshot(after_s=after_s)
            if snapshot is not None:
                return snapshot
            time.sleep(0.03)
        raise RuntimeError("current JointState is stale or the robot is not settled")

    def wait_fresh_joint_state(self, *, timeout_s: float) -> JointSnapshot:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            snapshot = self.latest_joints()
            if (
                snapshot is not None
                and time.monotonic() - snapshot.received_monotonic_s
                <= self.args.state_freshness_s
            ):
                return snapshot
            time.sleep(0.03)
        raise RuntimeError("no fresh complete JointState was received")

    @staticmethod
    def _require_negligible_joint_drift(
        reference: JointSnapshot,
        current: JointSnapshot,
        *,
        label: str,
    ) -> None:
        drift = float(
            np.max(
                np.abs(
                    np.asarray(current.positions_rad, dtype=float)
                    - np.asarray(reference.positions_rad, dtype=float)
                )
            )
        )
        if drift > MAX_TARGET_DERIVATION_JOINT_DRIFT_RAD:
            raise RuntimeError(
                f"robot joint state changed by {drift:.6f} rad {label}; "
                "manual target derivation was invalidated"
            )

    @staticmethod
    def _ensure_not_cancelled(cancel_event: threading.Event | None) -> None:
        if cancel_event is not None and cancel_event.is_set():
            raise RuntimeError("manual motion was cancelled before controller submission")

    def unresolved_motion(self) -> bool:
        with self._lock:
            return (
                self._motion_state_unknown
                or self._active_goal is not None
                or getattr(self, "_pending_submission_generation", 0) != 0
            )

    def _publish_servo_twist(
        self,
        frame_id: str,
        velocity_m_s: Sequence[float],
    ) -> None:
        frame = str(frame_id).strip()
        velocity = tuple(float(value) for value in velocity_m_s)
        if frame not in {self.base_frame, self.plane_frame}:
            raise ValueError(
                f"Servo command frame must be {self.base_frame!r} or "
                f"{self.plane_frame!r}"
            )
        if len(velocity) != 3 or not all(math.isfinite(value) for value in velocity):
            raise ValueError("Servo velocity must contain exactly three finite values")
        if max(abs(value) for value in velocity) > MAX_SERVO_LINEAR_SPEED_M_S + 1.0e-12:
            raise ValueError(
                f"Servo velocity exceeds {MAX_SERVO_LINEAR_SPEED_M_S * 1000.0:.0f} mm/s"
            )
        message = self.types["TwistStamped"]()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = frame
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = velocity
        message.twist.angular.x = 0.0
        message.twist.angular.y = 0.0
        message.twist.angular.z = 0.0
        self.servo_twist_publisher.publish(message)

    def _call_servo_pause(self, paused: bool) -> None:
        if not self.servo_pause_client.wait_for_service(
            timeout_sec=self.args.service_timeout_s
        ):
            raise RuntimeError(f"MoveIt Servo service {SERVO_PAUSE_SERVICE} is unavailable")
        request = self.types["SetBool"].Request()
        request.data = bool(paused)
        response = self._await(
            self.servo_pause_client.call_async(request),
            self.args.service_timeout_s,
            "MoveIt Servo pause-state change",
        )
        if not bool(response.success):
            detail = str(getattr(response, "message", "")).strip()
            raise RuntimeError(
                "MoveIt Servo rejected the requested pause state"
                + (f": {detail}" if detail else "")
            )
        with self._lock:
            self._servo_paused = bool(paused)

    def _switch_servo_to_twist(self) -> None:
        if not self.servo_command_type_client.wait_for_service(
            timeout_sec=self.args.service_timeout_s
        ):
            raise RuntimeError(
                f"MoveIt Servo service {SERVO_COMMAND_TYPE_SERVICE} is unavailable"
            )
        request_type = self.types["ServoCommandType"].Request
        request = request_type()
        request.command_type = int(request_type.TWIST)
        response = self._await(
            self.servo_command_type_client.call_async(request),
            self.args.service_timeout_s,
            "MoveIt Servo command-type selection",
        )
        if not bool(response.success):
            raise RuntimeError("MoveIt Servo rejected Cartesian Twist commands")

    def _wait_for_acceptable_servo_status(self, *, after_s: float) -> None:
        deadline = time.monotonic() + self.args.service_timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                status = self._last_servo_status
            if status is not None and status[0] >= after_s:
                _, code, message = status
                if code in _SERVO_FATAL_STATUS_CODES:
                    raise RuntimeError(
                        f"MoveIt Servo is not safe to arm (status {code}: "
                        f"{message or 'no detail'})"
                    )
                return
            time.sleep(0.01)
        raise RuntimeError("MoveIt Servo did not publish a fresh status")

    def _require_servo_joint_envelope(
        self,
        *,
        start_positions_rad: Sequence[float] | None = None,
    ) -> JointSnapshot:
        snapshot = self.latest_joints()
        if (
            snapshot is None
            or time.monotonic() - snapshot.received_monotonic_s
            > self.args.state_freshness_s
        ):
            raise RuntimeError("Servo refused: actual JointState is stale")
        by_name = dict(zip(self.environment.joint_names, snapshot.positions_rad, strict=True))
        configured_ranges = self.environment.planner["ik"][
            "canonical_joint_ranges_rad"
        ]
        for name, limits in configured_ranges.items():
            lower, upper = (float(value) for value in limits)
            margin = min(SERVO_CANONICAL_MARGIN_RAD, (upper - lower) * 0.25)
            value = float(by_name[name])
            if value <= lower + margin or value >= upper - margin:
                raise RuntimeError(
                    f"Servo refused: {name}={value:.6f} rad reached the "
                    f"canonical-branch guard [{lower + margin:.6f}, "
                    f"{upper - margin:.6f}]"
                )
        if start_positions_rad is not None:
            start = np.asarray(tuple(start_positions_rad), dtype=float)
            current = np.asarray(snapshot.positions_rad, dtype=float)
            if start.shape != current.shape:
                raise RuntimeError("Servo start-state joint vector is malformed")
            excursion = float(np.max(np.abs(current - start)))
            maximum = math.radians(self.args.max_joint_excursion_deg)
            if excursion > maximum:
                raise RuntimeError(
                    f"Servo stopped: joint excursion {math.degrees(excursion):.3f} deg "
                    f"exceeds the per-gesture limit {self.args.max_joint_excursion_deg:.3f} deg"
                )
        return snapshot

    def _require_servo_fast_gate(
        self,
        *,
        start_positions_rad: Sequence[float] | None = None,
    ) -> JointSnapshot:
        with self._lock:
            configured = self._servo_configured
            paused = self._servo_paused
            status = self._last_servo_status
            action_active = (
                self._active_goal is not None
                or self._motion_state_unknown
                or self._pending_submission_generation != 0
            )
        if not configured or paused:
            raise RuntimeError("MoveIt Servo is not armed")
        if action_active:
            raise RuntimeError("MoveGroup owns motion; Servo command refused")
        if self.servo_twist_publisher.get_subscription_count() < 1:
            raise RuntimeError("MoveIt Servo Twist subscriber is unavailable")
        if status is None or time.monotonic() - status[0] > self.args.state_freshness_s:
            raise RuntimeError("MoveIt Servo status is stale")
        if status[1] in _SERVO_FATAL_STATUS_CODES:
            raise RuntimeError(
                f"MoveIt Servo stopped with status {status[1]}: "
                f"{status[2] or 'no detail'}"
            )
        if self.playback_client is not None:
            server_is_ready = getattr(self.playback_client, "server_is_ready", None)
            playback_present = (
                bool(server_is_ready())
                if callable(server_is_ready)
                else self.playback_client.wait_for_server(timeout_sec=0.0)
            )
            if playback_present:
                raise RuntimeError(
                    f"authoritative runtime {PLAYBACK_ACTION} appeared; Servo stopped"
                )
        return self._require_servo_joint_envelope(
            start_positions_rad=start_positions_rad
        )

    def configure_servo(self) -> None:
        """Pre-arm collision-monitored Servo before any operator button press."""

        if not self.args.enable_motion:
            return
        with self._lock:
            if self._servo_configured and not self._servo_paused:
                return
        self._runtime_absent()
        self.verify_required_controller()
        self.live_authoritative_transforms()
        self.verify_scene()
        self._require_servo_joint_envelope()
        deadline = time.monotonic() + self.args.service_timeout_s
        while (
            self.servo_twist_publisher.get_subscription_count() < 1
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
        if self.servo_twist_publisher.get_subscription_count() < 1:
            raise RuntimeError(f"MoveIt Servo topic {SERVO_TWIST_TOPIC} has no subscriber")
        try:
            self._call_servo_pause(True)
            self._switch_servo_to_twist()
            self._publish_servo_twist(self.plane_frame, (0.0, 0.0, 0.0))
            status_after = time.monotonic()
            self._call_servo_pause(False)
            self._publish_servo_twist(self.plane_frame, (0.0, 0.0, 0.0))
            self._wait_for_acceptable_servo_status(after_s=status_after)
        except BaseException:
            try:
                self._publish_servo_twist(self.plane_frame, (0.0, 0.0, 0.0))
                self._call_servo_pause(True)
            except BaseException:
                pass
            with self._lock:
                self._servo_configured = False
                self._servo_paused = True
            raise
        with self._lock:
            self._servo_configured = True
            self._servo_paused = False

    def servo_jog_active(self) -> bool:
        with self._lock:
            return self._active_servo_generation != 0

    def begin_servo_jog(
        self,
        key: str,
        jog_frame: str,
        speed_m_s: float,
    ) -> int:
        """Atomically reserve motion and publish the first non-zero Twist."""

        if not self.args.enable_motion:
            raise RuntimeError("read-only mode; restart with --enable-motion")
        velocity = servo_jog_velocity(key, jog_frame, speed_m_s)
        frame = self.base_frame if jog_frame == "base" else self.plane_frame
        if not self._command_lock.acquire(blocking=False):
            raise RuntimeError(
                "another manual command is already in progress; commands are never queued"
            )
        lock_retained = False
        try:
            snapshot = self._require_servo_fast_gate()
            with self._servo_publish_lock:
                with self._lock:
                    if self._active_servo_generation != 0:
                        raise RuntimeError("a Servo jog is already active")
                    self._servo_generation_counter += 1
                    generation = self._servo_generation_counter
                    self._active_servo_generation = generation
                    self._active_servo_frame = frame
                    self._active_servo_velocity_m_s = velocity
                    self._servo_start_positions_rad = snapshot.positions_rad
                    self._servo_command_lock_held = True
                self._publish_servo_twist(frame, velocity)
            lock_retained = True
            return generation
        finally:
            if not lock_retained:
                with self._lock:
                    self._active_servo_generation = 0
                    self._active_servo_frame = None
                    self._active_servo_velocity_m_s = None
                    self._servo_start_positions_rad = None
                    self._servo_command_lock_held = False
                self._command_lock.release()

    def refresh_servo_jog(self, generation: int) -> bool:
        """Refresh one still-owned non-zero command; stale generations do nothing."""

        with self._servo_publish_lock:
            with self._lock:
                if self._active_servo_generation != int(generation):
                    return False
                frame = self._active_servo_frame
                velocity = self._active_servo_velocity_m_s
                start = self._servo_start_positions_rad
            if frame is None or velocity is None or start is None:
                self._stop_servo_jog_locked(generation)
                raise RuntimeError("active Servo command state is incomplete")
            try:
                self._require_servo_fast_gate(start_positions_rad=start)
                # Recheck ownership immediately before the non-zero publish.
                with self._lock:
                    if self._active_servo_generation != int(generation):
                        return False
                self._publish_servo_twist(frame, velocity)
                return True
            except BaseException:
                # The caller will surface the reason, but the session itself
                # owns the safety action and must issue zero first.
                with self._lock:
                    still_owned = self._active_servo_generation == int(generation)
                if still_owned:
                    self._stop_servo_jog_locked(generation)
                raise

    def _stop_servo_jog_locked(self, generation: int | None) -> bool:
        """Stop while ``_servo_publish_lock`` is held, leaving zero as last command."""

        with self._lock:
            active = self._active_servo_generation
            if generation is not None and active != int(generation):
                return False
            frame = self._active_servo_frame or self.plane_frame
            had_active = active != 0
            release_command_lock = self._servo_command_lock_held
            self._active_servo_generation = 0
            self._active_servo_frame = None
            self._active_servo_velocity_m_s = None
            self._servo_start_positions_rad = None
            self._servo_command_lock_held = False
        try:
            if getattr(self, "_servo_configured", False):
                self._publish_servo_twist(frame, (0.0, 0.0, 0.0))
        finally:
            if release_command_lock:
                self._command_lock.release()
        return had_active

    def stop_servo_jog(self, generation: int | None = None) -> bool:
        """Immediately terminate a held intent and publish a fresh zero Twist."""

        with self._servo_publish_lock:
            return self._stop_servo_jog_locked(generation)

    def _pause_servo_for_move_group(self) -> bool:
        if not getattr(self, "_servo_configured", False):
            return False
        if self.servo_jog_active():
            raise RuntimeError("active Servo jog must be released before MoveGroup motion")
        self._publish_servo_twist(self.plane_frame, (0.0, 0.0, 0.0))
        self.wait_settled(timeout_s=self.args.state_timeout_s)
        self._call_servo_pause(True)
        return True

    def _resume_servo_after_move_group(self) -> None:
        self._publish_servo_twist(self.plane_frame, (0.0, 0.0, 0.0))
        status_after = time.monotonic()
        self._call_servo_pause(False)
        self._publish_servo_twist(self.plane_frame, (0.0, 0.0, 0.0))
        self._wait_for_acceptable_servo_status(after_s=status_after)
        with self._lock:
            self._servo_configured = True
            self._servo_paused = False

    def verify_manual_readiness(self, *, joint_state_timeout_s: float = 1.0) -> None:
        """Run the complete non-moving gate used before a UI reports READY.

        Every actual move repeats these checks at submission time.  This first
        pass exists so a graphical control cannot look ready while its
        controller, authoritative TFs, collision scene, or runtime exclusion
        is already known to be wrong.
        """

        self.wait_fresh_joint_state(timeout_s=joint_state_timeout_s)
        self._runtime_absent()
        self.verify_required_controller()
        self.live_authoritative_transforms()
        self.verify_scene()
        if self.args.enable_motion:
            if not self.move_client.wait_for_server(
                timeout_sec=self.args.service_timeout_s
            ):
                raise RuntimeError(f"MoveIt action {MOVE_ACTION} is unavailable")
            if not self.execute_client.wait_for_server(
                timeout_sec=self.args.service_timeout_s
            ):
                raise RuntimeError(
                    f"MoveIt action {EXECUTE_TRAJECTORY_ACTION} is unavailable"
                )
            self.configure_servo()

    def verify_required_controller(self) -> None:
        if not self.controller_client.wait_for_service(
            timeout_sec=self.args.service_timeout_s
        ):
            raise RuntimeError(
                f"controller service {LIST_CONTROLLERS_SERVICE} is unavailable"
            )
        request = self.types["ListControllers"].Request()
        response = self._await(
            self.controller_client.call_async(request),
            self.args.service_timeout_s,
            "controller-state verification",
        )
        states = {str(item.name): str(item.state) for item in response.controller}
        state = states.get(self.args.required_controller)
        if state != "active":
            raise RuntimeError(
                f"required MoveIt controller {self.args.required_controller!r} "
                f"is not active (state={state!r})"
            )

    @staticmethod
    def _transform_matrix(transform: Any) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return pose_matrix(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _lookup_matrix(self, parent: str, child: str, timeout_s: float) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            parent,
            child,
            self.types["Time"](),
            timeout=self.types["Duration"](seconds=timeout_s),
        )
        return self._transform_matrix(transform)

    @staticmethod
    def _pose_message_matrix(pose: Any) -> np.ndarray:
        return pose_matrix(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        )

    def live_authoritative_transforms(
        self, timeout_s: float = 1.0
    ) -> tuple[np.ndarray, np.ndarray]:
        """Require live static TFs and prove that they match setup.yaml."""

        live_base_to_plane = self._lookup_matrix(
            self.base_frame, self.plane_frame, timeout_s
        )
        live_tool0_to_tip = self._lookup_matrix(
            self.tool_builder.parent_link, self.tool_builder.tip_frame, timeout_s
        )
        for label, expected, actual in (
            ("base->plane1", self.base_to_plane1, live_base_to_plane),
            ("tool0->rod_tip", self.tool_builder.tool0_to_tip, live_tool0_to_tip),
        ):
            error = motion_delta(expected, actual)
            if error.translation_m > 1.0e-6 or error.rotation_deg > 1.0e-4:
                raise RuntimeError(
                    f"live TF {label} disagrees with setup.yaml by "
                    f"{error.translation_m * 1000.0:.6f} mm / "
                    f"{error.rotation_deg:.6f} deg"
                )
        return live_base_to_plane, live_tool0_to_tip

    def current_tip_matrix(
        self,
        timeout_s: float = 1.0,
        *,
        tool0_to_tip: np.ndarray | None = None,
    ) -> np.ndarray:
        live_tool0_to_tip = (
            self.live_authoritative_transforms(timeout_s)[1]
            if tool0_to_tip is None else np.asarray(tool0_to_tip, dtype=float)
        )
        base_to_tool0 = self._lookup_matrix(
            self.base_frame, self.tool_builder.parent_link, timeout_s
        )
        return base_to_tool0 @ live_tool0_to_tip

    def snapshot_text(self) -> str:
        live_base_to_plane1, live_tool0_to_tip = self.live_authoritative_transforms()
        base_to_tip = self.current_tip_matrix(tool0_to_tip=live_tool0_to_tip)
        plane_to_tip = inverse_pose(live_base_to_plane1) @ base_to_tip
        base_position, base_quaternion = matrix_pose(base_to_tip)
        plane_position, plane_quaternion = matrix_pose(plane_to_tip)
        joints = self.latest_joints()
        if joints is None:
            raise RuntimeError("actual joints are unavailable")
        joint_age_s = time.monotonic() - joints.received_monotonic_s
        if joint_age_s > self.args.state_freshness_s:
            raise RuntimeError(
                f"actual JointState is stale ({joint_age_s:.3f} s old)"
            )
        lines = [
            f"environment: {self.environment_id}",
            f"{self.base_frame} -> {self.tool_builder.tip_frame}: {ur_pose_text(base_to_tip)}",
            "  quaternion_xyzw: [" + ", ".join(f"{v:.9f}" for v in base_quaternion) + "]",
            f"{self.plane_frame} -> {self.tool_builder.tip_frame}: {ur_pose_text(plane_to_tip)}",
            "  quaternion_xyzw: [" + ", ".join(f"{v:.9f}" for v in plane_quaternion) + "]",
        ]
        lines.append(
            "actual joints rad: ["
            + ", ".join(f"{v:.6f}" for v in joints.positions_rad)
            + f"] (age {joint_age_s:.3f}s)"
        )
        lines.append(
            "actual joints deg: ["
            + ", ".join(f"{math.degrees(v):.3f}" for v in joints.positions_rad)
            + "]"
        )
        return "\n".join(lines)

    def _await(self, future: Any, timeout_s: float, label: str) -> Any:
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            raise RuntimeError(f"{label} timed out after {timeout_s:.1f} s")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def _scene_observation(self, attached: Any) -> AttachedRodObservation:
        collision = attached.object
        primitive = collision.primitives[0] if collision.primitives else None
        pose = collision.primitive_poses[0] if collision.primitive_poses else None
        combined_pose: tuple[tuple[float, ...], tuple[float, ...]] | None = None
        if pose is not None:
            combined_pose = matrix_pose(
                self._pose_message_matrix(collision.pose)
                @ self._pose_message_matrix(pose)
            )
        cylinder_type = self.types["SolidPrimitive"].CYLINDER
        return AttachedRodObservation(
            object_id=str(collision.id),
            link_name=str(attached.link_name),
            header_frame=str(collision.header.frame_id),
            is_cylinder=(
                len(collision.primitives) == 1
                and len(collision.primitive_poses) == 1
                and primitive is not None
                and int(primitive.type) == int(cylinder_type)
            ),
            dimensions_m=tuple(float(value) for value in primitive.dimensions) if primitive else (),
            center_m=(
                tuple(float(value) for value in combined_pose[0])
                if combined_pose is not None else (math.nan, math.nan, math.nan)
            ),
            quaternion_xyzw=(
                tuple(float(value) for value in combined_pose[1])
                if combined_pose is not None else (math.nan, math.nan, math.nan, math.nan)
            ),
            touch_links=tuple(str(value) for value in attached.touch_links),
        )

    def _world_observation(self, collision: Any) -> WorldCuboidObservation:
        primitive = collision.primitives[0] if collision.primitives else None
        pose = collision.primitive_poses[0] if collision.primitive_poses else None
        combined_pose: tuple[tuple[float, ...], tuple[float, ...]] | None = None
        if pose is not None:
            combined_pose = matrix_pose(
                self._pose_message_matrix(collision.pose)
                @ self._pose_message_matrix(pose)
            )
        box_type = self.types["SolidPrimitive"].BOX
        return WorldCuboidObservation(
            object_id=str(collision.id),
            frame_id=str(collision.header.frame_id),
            is_box=(
                len(collision.primitives) == 1
                and len(collision.primitive_poses) == 1
                and primitive is not None
                and int(primitive.type) == int(box_type)
            ),
            dimensions_m=(
                tuple(float(value) for value in primitive.dimensions)
                if primitive else ()
            ),
            position_m=(
                tuple(float(value) for value in combined_pose[0])
                if combined_pose is not None else (math.nan, math.nan, math.nan)
            ),
            quaternion_xyzw=(
                tuple(float(value) for value in combined_pose[1])
                if combined_pose is not None else (math.nan, math.nan, math.nan, math.nan)
            ),
        )

    def verify_scene(self) -> SceneComparison:
        with self._lock:
            self._validated_acm = None
        if not self.scene_client.wait_for_service(timeout_sec=self.args.service_timeout_s):
            raise RuntimeError(f"MoveIt service {GET_SCENE_SERVICE} is unavailable")
        request = self.types["GetPlanningScene"].Request()
        components = self.types["PlanningSceneComponents"]
        request.components.components = (
            components.WORLD_OBJECT_NAMES
            | components.WORLD_OBJECT_GEOMETRY
            | components.ROBOT_STATE_ATTACHED_OBJECTS
            | components.ALLOWED_COLLISION_MATRIX
        )
        response = self._await(
            self.scene_client.call_async(request), self.args.service_timeout_s,
            "planning-scene verification",
        )
        world_observations = {
            observation.object_id: observation
            for observation in (
                self._world_observation(item)
                for item in response.scene.world.collision_objects
            )
        }
        world_ids = tuple(world_observations)
        expected_cuboids: list[Any] = []
        for expected in self.scene.cuboids:
            actual = world_observations.get(expected.object_id)
            if actual is None or actual.frame_id == expected.frame_id:
                expected_cuboids.append(expected)
                continue
            expected_in_actual = self._lookup_matrix(
                actual.frame_id,
                expected.frame_id,
                self.args.service_timeout_s,
            ) @ pose_matrix(expected.position_m, expected.quaternion_xyzw)
            expected_position, expected_quaternion = matrix_pose(expected_in_actual)
            expected_cuboids.append(
                WorldCuboidObservation(
                    object_id=expected.object_id,
                    frame_id=actual.frame_id,
                    is_box=True,
                    dimensions_m=expected.dimensions_m,
                    position_m=expected_position,  # type: ignore[arg-type]
                    quaternion_xyzw=expected_quaternion,  # type: ignore[arg-type]
                )
            )
        rods = {
            observation.object_id: observation
            for observation in (
                self._scene_observation(item)
                for item in response.scene.robot_state.attached_collision_objects
            )
        }
        comparison = compare_scene_inventory(
            expected_cuboids,
            self.scene.attached_rod,
            world_ids,
            rods,
            actual_cuboids=world_observations,
        )
        if not comparison.ok:
            raise RuntimeError(comparison.error_message())
        acm_errors = forbidden_acm_allowances(
            response.scene.allowed_collision_matrix,
            world_object_ids=(cuboid.object_id for cuboid in self.scene.cuboids),
            attached_rod_id=self.scene.attached_rod.object_id,
            rod_touch_links=self.scene.attached_rod.touch_links,
        )
        if acm_errors:
            raise RuntimeError(
                "allowed collision matrix weakens the authoritative scene: "
                + "; ".join(acm_errors)
            )
        with self._lock:
            self._validated_acm = copy.deepcopy(
                response.scene.allowed_collision_matrix
            )
        return comparison

    def _runtime_absent(self) -> None:
        if self.playback_client is not None and self.playback_client.wait_for_server(
            timeout_sec=0.05
        ):
            raise RuntimeError(
                f"authoritative runtime {PLAYBACK_ACTION} is present; manual motion refused"
            )

    def _local_joint_constraints(
        self,
        start_positions_rad: Sequence[float],
        *,
        name: str,
    ) -> tuple[Any, tuple[tuple[float, float], ...]]:
        """Build the same installed-branch bounds for IK and path planning."""

        canonical_ranges = self.environment.planner["ik"][
            "canonical_joint_ranges_rad"
        ]
        path_bounds = manual_joint_path_bounds(
            self.environment.joint_names,
            start_positions_rad,
            canonical_ranges,
            max_excursion_rad=math.radians(self.args.max_joint_excursion_deg),
        )
        constraints = self.types["Constraints"]()
        constraints.name = name
        constraints.joint_constraints = []
        for name, start, (lower, upper) in zip(
            self.environment.joint_names,
            start_positions_rad,
            path_bounds,
            strict=True,
        ):
            joint = self.types["JointConstraint"]()
            joint.joint_name = name
            joint.position = float(start)
            joint.tolerance_below = float(start) - lower
            joint.tolerance_above = upper - float(start)
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints, path_bounds

    @staticmethod
    def _moveit_error_description(error_code: Any) -> str:
        values = [f"code {getattr(error_code, 'val', 'unknown')}"]
        message = str(getattr(error_code, "message", "") or "").strip()
        source = str(getattr(error_code, "source", "") or "").strip()
        if message:
            values.append(message)
        if source:
            values.append(f"source={source}")
        return ", ".join(values)

    def _canonicalize_local_ik_solution(
        self,
        raw_positions_rad: Sequence[float],
        start_positions_rad: Sequence[float],
        path_bounds_rad: Sequence[tuple[float, float]],
    ) -> tuple[float, ...]:
        """Choose only an exactly equivalent whole-turn representation.

        UR joints are bounded revolute joints in the sealed robot model.  Some
        IK plugins nevertheless report an equivalent value one full turn away
        from the supplied seed.  Selecting the in-domain winding nearest that
        seed changes neither the Cartesian target nor the physical joint
        configuration; it only makes the representation match the installed
        branch.  No approximate or non-periodic value is repaired.
        """

        joint_names = tuple(str(name) for name in self.environment.joint_names)
        domains = tuple(self.environment.joint_position_domains)
        if (
            len(raw_positions_rad) != len(joint_names)
            or len(start_positions_rad) != len(joint_names)
            or len(path_bounds_rad) != len(joint_names)
            or tuple(str(domain.name) for domain in domains) != joint_names
        ):
            raise RuntimeError("sealed joint domains do not match the MoveIt IK group")

        canonical: list[float] = []
        tolerance = 1.0e-9
        for name, raw, start, path_bounds, domain in zip(
            joint_names,
            raw_positions_rad,
            start_positions_rad,
            path_bounds_rad,
            domains,
            strict=True,
        ):
            value = float(raw)
            seed = float(start)
            lower = max(float(path_bounds[0]), float(domain.lower))
            upper = min(float(path_bounds[1]), float(domain.upper))
            if lower > upper:
                raise RuntimeError(f"no valid local joint interval remains for {name}")

            period = domain.period
            if period is None:
                candidate = value
            else:
                period_value = float(period)
                if not math.isfinite(period_value) or period_value <= 0.0:
                    raise RuntimeError(f"sealed winding period for {name} is invalid")
                minimum_turns = math.ceil(
                    (lower - value) / period_value - 1.0e-12
                )
                maximum_turns = math.floor(
                    (upper - value) / period_value + 1.0e-12
                )
                if minimum_turns > maximum_turns:
                    raise RuntimeError(
                        f"MoveIt IK result {name}={value:.6f} rad has no exactly "
                        "equivalent winding in the configured local/canonical interval"
                    )
                nearest_turns = round((seed - value) / period_value)
                turn_candidates = {
                    minimum_turns,
                    maximum_turns,
                    min(max(nearest_turns, minimum_turns), maximum_turns),
                }
                candidate = min(
                    (value + turns * period_value for turns in turn_candidates),
                    key=lambda item: abs(item - seed),
                )

            if candidate < lower - tolerance or candidate > upper + tolerance:
                raise RuntimeError(
                    f"MoveIt IK result {name}={value:.6f} rad leaves the configured "
                    f"local/canonical interval [{lower:.6f}, {upper:.6f}]"
                )
            canonical.append(float(candidate))
        return tuple(canonical)

    def _solve_collision_checked_ik(
        self,
        target_tool: TimedPose,
        start_positions_rad: Sequence[float],
    ) -> tuple[float, ...]:
        """Resolve one exact tool0 target without OMPL pose-goal sampling.

        The service may return only a collision-free solution in the same local
        joint branch as the actual seed.  Any malformed, distant, or
        non-canonical result is rejected.  Only an exactly equivalent whole-turn
        winding may be represented near the seed; the function never retries or
        approximates an IK result.
        """

        if not self.ik_client.wait_for_service(timeout_sec=self.args.service_timeout_s):
            raise RuntimeError(f"MoveIt service {COMPUTE_IK_SERVICE} is unavailable")

        request = self.types["GetPositionIK"].Request()
        ik_request = request.ik_request
        joint_names = tuple(str(name) for name in self.environment.joint_names)
        start_positions = tuple(float(value) for value in start_positions_rad)
        local_constraints, path_bounds = self._local_joint_constraints(
            start_positions,
            name="manual_local_joint_branch",
        )
        ik_request.group_name = self.args.move_group
        ik_request.robot_state.is_diff = True
        ik_request.robot_state.joint_state.name = list(joint_names)
        ik_request.robot_state.joint_state.position = list(start_positions)
        # Explicitly carry the authoritative attached rod in the IK seed.  The
        # world and ACM come from the just-verified live PlanningScene because
        # GetPositionIK has no PlanningScene-diff field.
        scene_diff = _build_apply_request(self.scene, self.types).scene
        ik_request.robot_state.attached_collision_objects = copy.deepcopy(
            scene_diff.robot_state.attached_collision_objects
        )
        ik_request.constraints = local_constraints
        ik_request.avoid_collisions = True
        ik_request.ik_link_name = self.tool_builder.parent_link
        ik_request.pose_stamped.header.frame_id = self.base_frame
        (
            ik_request.pose_stamped.pose.position.x,
            ik_request.pose_stamped.pose.position.y,
            ik_request.pose_stamped.pose.position.z,
        ) = target_tool.position_m
        (
            ik_request.pose_stamped.pose.orientation.x,
            ik_request.pose_stamped.pose.orientation.y,
            ik_request.pose_stamped.pose.orientation.z,
            ik_request.pose_stamped.pose.orientation.w,
        ) = target_tool.quaternion_xyzw
        ik_request.timeout = self.types["Duration"](
            seconds=MANUAL_IK_TIMEOUT_S
        ).to_msg()

        response = self._await(
            self.ik_client.call_async(request),
            self.args.service_timeout_s,
            "collision-checked inverse kinematics",
        )
        error_code = response.error_code
        if int(error_code.val) != int(self.types["MoveItErrorCodes"].SUCCESS):
            raise RuntimeError(
                "MoveIt collision-checked IK failed with "
                + self._moveit_error_description(error_code)
            )

        response_names = tuple(str(name) for name in response.solution.joint_state.name)
        response_positions = tuple(
            float(value) for value in response.solution.joint_state.position
        )
        if len(response_names) != len(response_positions):
            raise RuntimeError("MoveIt IK returned mismatched joint names and positions")
        if len(set(response_names)) != len(response_names):
            raise RuntimeError("MoveIt IK returned duplicate joint names")
        if set(response_names) != set(joint_names):
            missing = sorted(set(joint_names) - set(response_names))
            unexpected = sorted(set(response_names) - set(joint_names))
            raise RuntimeError(
                "MoveIt IK joint set does not exactly match the configured group "
                f"(missing={missing}, unexpected={unexpected})"
            )
        by_name = dict(zip(response_names, response_positions, strict=True))
        solution = tuple(by_name[name] for name in joint_names)
        if not all(math.isfinite(value) for value in solution):
            raise RuntimeError("MoveIt IK returned a non-finite joint position")
        return self._canonicalize_local_ik_solution(
            solution,
            start_positions,
            path_bounds,
        )

    def _build_goal(
        self,
        target_positions_rad: Sequence[float],
        start_positions_rad: Sequence[float],
    ) -> Any:
        types = self.types
        joint_names = tuple(str(name) for name in self.environment.joint_names)
        target_positions = tuple(float(value) for value in target_positions_rad)
        start_positions = tuple(float(value) for value in start_positions_rad)
        if (
            len(target_positions) != len(joint_names)
            or not all(math.isfinite(value) for value in target_positions)
        ):
            raise RuntimeError("manual IK target does not match the configured joints")

        constraints = types["Constraints"]()
        constraints.name = "manual_exact_collision_ik_joint_goal"
        constraints.joint_constraints = []
        for name, position in zip(joint_names, target_positions, strict=True):
            joint = types["JointConstraint"]()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_below = MANUAL_JOINT_GOAL_TOLERANCE_RAD
            joint.tolerance_above = MANUAL_JOINT_GOAL_TOLERANCE_RAD
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        path_constraints, _path_bounds = self._local_joint_constraints(
            start_positions,
            name="manual_local_joint_branch",
        )

        goal = types["MoveGroup"].Goal()
        goal.request.group_name = self.args.move_group
        goal.request.num_planning_attempts = self.args.planning_attempts
        goal.request.allowed_planning_time = self.args.allowed_planning_time_s
        goal.request.max_velocity_scaling_factor = self.args.velocity_scale
        goal.request.max_acceleration_scaling_factor = self.args.acceleration_scale
        goal.request.pipeline_id = self.args.pipeline_id
        goal.request.planner_id = self.args.planner_id
        goal.request.start_state.is_diff = True
        goal.request.start_state.joint_state.name = list(joint_names)
        goal.request.start_state.joint_state.position = list(start_positions)
        goal.request.goal_constraints = [constraints]
        goal.request.path_constraints = path_constraints
        # Keep the explicit start state and attached rod intact.  Jazzy's
        # combined plan-and-execute capability clears those request fields;
        # plan-only honours them, after which the exact validated trajectory is
        # submitted through MoveIt's ExecuteTrajectory action.
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        # Bind the same complete world geometry and attached rod to this goal,
        # closing the verify-then-plan gap if another node edits the scene.
        scene_diff = _build_apply_request(
            self.scene,
            self.types,
        ).scene
        with self._lock:
            validated_acm = copy.deepcopy(self._validated_acm)
        if validated_acm is None:
            raise RuntimeError(
                "no validated Allowed Collision Matrix is bound to this motion"
            )
        # Carry the exact validated ACM with the authoritative geometry.  A
        # concurrent node therefore cannot weaken collision policy in the gap
        # between verification and MoveGroup consuming this goal.
        scene_diff.allowed_collision_matrix = validated_acm
        goal.planning_options.planning_scene_diff = scene_diff
        return goal

    def _terminal_result_is_confirmed(self, wrapped: Any) -> bool:
        GoalStatus = self.types["GoalStatus"]
        try:
            status = int(wrapped.status)
        except (AttributeError, TypeError, ValueError):
            return False
        return status in {
            int(GoalStatus.STATUS_SUCCEEDED),
            int(GoalStatus.STATUS_CANCELED),
            int(GoalStatus.STATUS_ABORTED),
        }

    def _ensure_goal_tracking_fields(self) -> None:
        """Populate tracking fields for normal and lightweight test sessions."""

        # A few pure unit tests deliberately construct this class with
        # ``__new__`` so that importing this module never requires ROS.  Keep
        # the safety state lazy as well as initialising it in ``__init__``.
        if not hasattr(self, "_active_goal_generation"):
            self._active_goal_generation = 0
        if not hasattr(self, "_active_action_label"):
            self._active_action_label = "MoveIt"
        if not hasattr(self, "_active_result_request_in_progress"):
            self._active_result_request_in_progress = False
        if not hasattr(self, "_pending_submission_generation"):
            self._pending_submission_generation = 0
        if not hasattr(self, "_goal_generation_counter"):
            self._goal_generation_counter = 0
        if not hasattr(self, "_cancel_supervisors"):
            self._cancel_supervisors = {}

    def _begin_goal_submission(self) -> int:
        """Reserve ownership before calling ``send_goal_async``."""

        with self._lock:
            self._ensure_goal_tracking_fields()
            self._goal_generation_counter += 1
            generation = self._goal_generation_counter
            self._pending_submission_generation = generation
            self._motion_state_unknown = True
            return generation

    def _clear_pending_submission(self, generation: int) -> None:
        with self._lock:
            self._ensure_goal_tracking_fields()
            if self._pending_submission_generation != generation:
                return
            self._pending_submission_generation = 0
            if self._active_goal is None:
                self._motion_state_unknown = False

    def _clear_tracked_goal(
        self,
        handle: Any,
        generation: int | None = None,
    ) -> None:
        with self._lock:
            self._ensure_goal_tracking_fields()
            matches_generation = (
                generation is None
                or self._active_goal_generation == generation
            )
            if self._active_goal is handle and matches_generation:
                self._active_goal = None
                self._active_result_future = None
                self._active_goal_generation = 0
                self._active_action_label = "MoveIt"
                self._active_result_request_in_progress = False
                self._motion_state_unknown = (
                    self._pending_submission_generation != 0
                )

    def _tracked_result_finished(
        self,
        handle: Any,
        generation: int,
        future: Any,
    ) -> None:
        """Unlock only when GetResult itself confirms a terminal response."""

        try:
            wrapped = future.result()
        except BaseException:
            with self._lock:
                self._ensure_goal_tracking_fields()
                if (
                    self._active_goal is handle
                    and self._active_goal_generation == generation
                ):
                    self._motion_state_unknown = True
            self._ensure_cancel_supervisor(handle, generation)
            return
        if wrapped is None or not self._terminal_result_is_confirmed(wrapped):
            with self._lock:
                self._ensure_goal_tracking_fields()
                if (
                    self._active_goal is handle
                    and self._active_goal_generation == generation
                ):
                    self._motion_state_unknown = True
            self._ensure_cancel_supervisor(handle, generation)
            return
        self._clear_tracked_goal(handle, generation)

    def _track_goal(
        self,
        handle: Any,
        *,
        generation: int | None = None,
        action_label: str = "MoveIt",
    ) -> Any:
        # Own the accepted handle before any further action-client call.  If
        # get_result_async fails or the caller is interrupted, the handle stays
        # cancellable and the global motion lock remains engaged.
        with self._lock:
            self._ensure_goal_tracking_fields()
            if generation is None:
                self._goal_generation_counter += 1
                generation = self._goal_generation_counter
            elif self._pending_submission_generation not in (0, generation):
                raise RuntimeError("a different MoveIt submission owns the session")
            if (
                self._active_goal is not None
                and not (
                    self._active_goal is handle
                    and self._active_goal_generation == generation
                )
            ):
                raise RuntimeError("a different MoveIt goal already owns the session")
            if (
                self._active_goal is handle
                and self._active_goal_generation == generation
            ):
                if self._active_result_future is not None:
                    return self._active_result_future
                if self._active_result_request_in_progress:
                    raise RuntimeError("MoveIt goal adoption is already in progress")
            self._active_goal = handle
            self._active_result_future = None
            self._active_goal_generation = generation
            self._active_action_label = action_label
            self._active_result_request_in_progress = True
            if self._pending_submission_generation == generation:
                self._pending_submission_generation = 0
            self._motion_state_unknown = True
        try:
            result_future = handle.get_result_async()
        except BaseException:
            with self._lock:
                if (
                    self._active_goal is handle
                    and self._active_goal_generation == generation
                ):
                    self._active_result_request_in_progress = False
            self._ensure_cancel_supervisor(handle, generation)
            raise
        with self._lock:
            if (
                self._active_goal is handle
                and self._active_goal_generation == generation
            ):
                self._active_result_future = result_future
                self._active_result_request_in_progress = False
        result_future.add_done_callback(
            lambda future: self._tracked_result_finished(
                handle,
                generation,
                future,
            )
        )
        return result_future

    def _late_goal_submission(
        self,
        future: Any,
        generation: int | None = None,
        action_label: str = "MoveIt",
    ) -> None:
        """Cancel a goal whose response arrived after the submission timeout."""

        try:
            handle = future.result()
        except Exception:
            # A transport/client failure does not prove that the server never
            # accepted the request.  With no handle available it cannot be
            # cancelled, so retain the global lock.
            return
        if handle is None:
            return
        accepted = getattr(handle, "accepted", None)
        if accepted is False:
            if generation is None:
                with self._lock:
                    self._motion_state_unknown = False
            else:
                self._clear_pending_submission(generation)
            return
        if accepted is not True:
            return
        with self._lock:
            self._ensure_goal_tracking_fields()
            already_adopted = (
                self._active_goal is handle
                and (
                    generation is None
                    or self._active_goal_generation == generation
                )
            )
            if generation is not None and not already_adopted:
                if self._pending_submission_generation != generation:
                    return
        try:
            if not already_adopted:
                self._track_goal(
                    handle,
                    generation=generation,
                    action_label=action_label,
                )
        except BaseException:
            return
        with self._lock:
            adopted_generation = self._active_goal_generation
        # Make the first cancellation request synchronously.  Besides reducing
        # latency, this guarantees ownership has turned into an actual cancel
        # request before a late-response callback returns to its executor.
        self._request_goal_cancel(handle, adopted_generation)
        self._ensure_cancel_supervisor(handle, adopted_generation)

    def _arm_late_goal_submission(
        self,
        future: Any,
        generation: int,
        action_label: str,
    ) -> None:
        """Idempotently hand an interrupted submission to the late adopter."""

        future.add_done_callback(
            lambda completed: self._late_goal_submission(
                completed,
                generation,
                action_label,
            )
        )

    def _await_goal_submission(
        self,
        future: Any,
        *,
        action_label: str = "MoveGroup",
        generation: int | None = None,
    ) -> Any:
        # Once a send request exists, a missing response is not proof that the
        # server rejected it.  Only explicit rejection or a confirmed terminal
        # GetResult response may release this lock.
        if generation is None:
            generation = self._begin_goal_submission()
        deadline = time.monotonic() + self.args.service_timeout_s
        try:
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
        except BaseException:
            # A Python signal can interrupt the polling loop after DDS has
            # accepted the send request but before the goal handle reaches this
            # thread.  The late callback must own and cancel that handle.
            with self._lock:
                self._motion_state_unknown = True
            self._arm_late_goal_submission(future, generation, action_label)
            raise
        if not future.done():
            with self._lock:
                self._motion_state_unknown = True
            self._arm_late_goal_submission(future, generation, action_label)
            raise RuntimeError(
                f"{action_label} goal response timed out; motion is locked until the "
                "late response is cancelled and reaches a terminal state"
            )
        try:
            handle = future.result()
        except Exception as error:
            with self._lock:
                self._motion_state_unknown = True
            raise RuntimeError(
                f"{action_label} goal submission failed with unknown server state; "
                "manual motion remains locked"
            ) from error
        if handle is None:
            raise RuntimeError(
                f"{action_label} goal submission returned no handle; manual motion "
                "remains locked"
            )
        accepted = getattr(handle, "accepted", None)
        if accepted is False:
            self._clear_pending_submission(generation)
            return handle, None
        if accepted is not True:
            raise RuntimeError(
                f"{action_label} goal response has an ambiguous acceptance state; "
                "manual motion remains locked"
            )
        try:
            result_future = self._track_goal(
                handle,
                generation=generation,
                action_label=action_label,
            )
        except BaseException as error:
            raise RuntimeError(
                f"{action_label} accepted the goal but result tracking failed; manual "
                "motion remains locked"
            ) from error
        return handle, result_future

    def _goal_is_owned(self, handle: Any, generation: int) -> bool:
        with self._lock:
            self._ensure_goal_tracking_fields()
            return (
                self._active_goal is handle
                and self._active_goal_generation == generation
            )

    def _request_goal_cancel(self, handle: Any, generation: int) -> bool:
        """Request cancellation only for the still-owned goal generation."""

        if not self._goal_is_owned(handle, generation):
            return False
        try:
            response = self._await(
                handle.cancel_goal_async(),
                2.0,
                "MoveIt cancellation",
            )
        except BaseException:
            with self._lock:
                if (
                    self._active_goal is handle
                    and self._active_goal_generation == generation
                ):
                    self._motion_state_unknown = True
            return False
        # A late CancelGoal response belongs only to the captured generation.
        # It must never relock (or otherwise affect) a subsequent goal.
        if not self._goal_is_owned(handle, generation):
            return False
        return bool(getattr(response, "goals_canceling", ()))

    def _cancel_supervisor_main(
        self,
        handle: Any,
        generation: int,
    ) -> None:
        """Retain goal ownership and retry cancellation until GetResult is terminal."""

        try:
            while self._goal_is_owned(handle, generation):
                with self._lock:
                    current_future = (
                        self._active_result_future
                        if (
                            self._active_goal is handle
                            and self._active_goal_generation == generation
                        )
                        else None
                    )
                    result_request_in_progress = (
                        self._active_result_request_in_progress
                        if self._active_goal is handle
                        and self._active_goal_generation == generation
                        else False
                    )
                if current_future is not None and current_future.done():
                    try:
                        completed = current_future.result()
                    except BaseException:
                        completed = None
                    if (
                        completed is not None
                        and self._terminal_result_is_confirmed(completed)
                    ):
                        self._clear_tracked_goal(handle, generation)
                        break

                self._request_goal_cancel(handle, generation)
                # GetResult creation/future transport can itself fail.  Retrying
                # it is safe and gives the session another opportunity to
                # receive the authoritative terminal result while cancellation
                # continues in the background.  Never race the initial adopter.
                if (
                    not result_request_in_progress
                    and (current_future is None or current_future.done())
                ):
                    with self._lock:
                        if (
                            self._active_goal is handle
                            and self._active_goal_generation == generation
                            and not self._active_result_request_in_progress
                        ):
                            self._active_result_request_in_progress = True
                            may_request_result = True
                        else:
                            may_request_result = False
                    if not may_request_result:
                        threading.Event().wait(0.1)
                        continue
                    try:
                        replacement = handle.get_result_async()
                    except BaseException:
                        replacement = None
                    with self._lock:
                        if (
                            self._active_goal is handle
                            and self._active_goal_generation == generation
                        ):
                            self._active_result_request_in_progress = False
                            if (
                                replacement is not None
                                and replacement is not current_future
                            ):
                                self._active_result_future = replacement
                                bind_replacement = True
                            else:
                                bind_replacement = False
                        else:
                            bind_replacement = False
                    if bind_replacement:
                        replacement.add_done_callback(
                            lambda future: self._tracked_result_finished(
                                handle,
                                generation,
                                future,
                            )
                        )
                # Do not use the polling sleep helper here: this supervisor is
                # the last owner after the foreground operation has unwound and
                # must survive signal/test interruption of ordinary sleeps.
                threading.Event().wait(0.1)
        finally:
            with self._lock:
                self._ensure_goal_tracking_fields()
                if self._cancel_supervisors.get(generation) is threading.current_thread():
                    del self._cancel_supervisors[generation]

    def _ensure_cancel_supervisor(self, handle: Any, generation: int) -> None:
        """Start one session-owned cancellation supervisor for this exact goal."""

        if generation <= 0:
            return
        with self._lock:
            self._ensure_goal_tracking_fields()
            if not (
                self._active_goal is handle
                and self._active_goal_generation == generation
            ):
                return
            existing = self._cancel_supervisors.get(generation)
            if existing is not None and existing.is_alive():
                return
            thread = threading.Thread(
                target=self._cancel_supervisor_main,
                args=(handle, generation),
                name=f"tcp-console-goal-supervisor-{generation}",
                daemon=True,
            )
            self._cancel_supervisors[generation] = thread
            thread.start()

    def cancel_active(self) -> bool:
        with self._lock:
            self._ensure_goal_tracking_fields()
            handle = self._active_goal
            generation = self._active_goal_generation
        if handle is None:
            return False
        return self._request_goal_cancel(handle, generation)

    def _await_terminal_result(
        self,
        result_future: Any,
        cancel_event: threading.Event | None,
        *,
        action_label: str = "MoveGroup",
    ) -> Any:
        normal_deadline = time.monotonic() + self.args.motion_timeout_s
        cancel_deadline: float | None = None
        next_cancel_s = 0.0
        cancellation_reason = ""
        while True:
            now = time.monotonic()
            if cancel_event is not None and cancel_event.is_set() and cancel_deadline is None:
                cancellation_reason = "operator cancellation"
                self.cancel_active()
                next_cancel_s = time.monotonic() + 0.5
                cancel_deadline = now + self.args.cancel_confirmation_timeout_s
            elif (
                not result_future.done()
                and now >= normal_deadline
                and cancel_deadline is None
            ):
                cancellation_reason = "motion timeout"
                self.cancel_active()
                next_cancel_s = time.monotonic() + 0.5
                cancel_deadline = now + self.args.cancel_confirmation_timeout_s
            elif cancel_deadline is not None and now >= next_cancel_s:
                self.cancel_active()
                next_cancel_s = time.monotonic() + 0.5
            if result_future.done():
                break
            if cancel_deadline is not None and now >= cancel_deadline:
                with self._lock:
                    if self._active_result_future is result_future:
                        self._motion_state_unknown = True
                        handle = self._active_goal
                        generation = self._active_goal_generation
                    else:
                        handle = None
                        generation = 0
                if handle is not None:
                    self._ensure_cancel_supervisor(handle, generation)
                raise RuntimeError(
                    f"{cancellation_reason} was not confirmed by a terminal "
                    f"{action_label} result; manual motion remains locked"
                )
            time.sleep(0.02)
        try:
            wrapped = result_future.result()
        except BaseException as error:
            with self._lock:
                if self._active_result_future is result_future:
                    self._motion_state_unknown = True
                    handle = self._active_goal
                    generation = self._active_goal_generation
                else:
                    handle = None
                    generation = 0
            if handle is not None:
                self._ensure_cancel_supervisor(handle, generation)
            raise RuntimeError(
                f"{action_label} GetResult failed; manual motion remains locked"
            ) from error
        if wrapped is None or not self._terminal_result_is_confirmed(wrapped):
            with self._lock:
                if self._active_result_future is result_future:
                    self._motion_state_unknown = True
                    handle = self._active_goal
                    generation = self._active_goal_generation
                else:
                    handle = None
                    generation = 0
            if handle is not None:
                self._ensure_cancel_supervisor(handle, generation)
            raise RuntimeError(
                f"{action_label} GetResult did not confirm SUCCEEDED, CANCELED, or "
                "ABORTED; manual motion remains locked"
            )
        with self._lock:
            if self._active_result_future is result_future:
                terminal_handle = self._active_goal
                terminal_generation = self._active_goal_generation
            else:
                terminal_handle = None
                terminal_generation = 0
        if terminal_handle is not None:
            self._clear_tracked_goal(terminal_handle, terminal_generation)
        if cancellation_reason:
            # Cancellation/timeout is sticky.  A terminal result confirms that
            # this client once again owns the outcome; even SUCCEEDED must not
            # turn the interrupted operation into success or let plan-only
            # continue on to ExecuteTrajectory.
            raise RuntimeError(
                f"{cancellation_reason} was confirmed by terminal "
                f"{action_label} result; the interrupted motion is failed closed"
            )
        return wrapped

    def _run_tracked_action(
        self,
        client: Any,
        goal: Any,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
        pre_submit: Callable[[], None] | None = None,
    ) -> Any:
        """Submit one owned MoveIt action and retain ownership until terminal."""

        if not client.wait_for_server(timeout_sec=self.args.service_timeout_s):
            raise RuntimeError(f"MoveIt action {action_label} is unavailable")
        self._ensure_not_cancelled(cancel_event)
        if pre_submit is not None:
            pre_submit()
            self._ensure_not_cancelled(cancel_event)
        with self._lock:
            self._ensure_goal_tracking_fields()
        generation = self._begin_goal_submission()
        send_future: Any | None = None
        try:
            try:
                send_future = client.send_goal_async(goal)
            except BaseException as error:
                raise RuntimeError(
                    f"{action_label} submission raised with unknown server state; "
                    "manual motion remains locked"
                ) from error

            handle, result_future = self._await_goal_submission(
                send_future,
                action_label=action_label,
                generation=generation,
            )
        except BaseException:
            # Covers the otherwise dangerous interval after send_goal_async()
            # returned but before this thread stored/adopted the accepted goal.
            # If the future is already complete, add_done_callback invokes the
            # adopter immediately; otherwise it owns the eventual response.
            if send_future is not None:
                self._arm_late_goal_submission(
                    send_future,
                    generation,
                    action_label,
                )
            raise
        if result_future is None:
            raise RuntimeError(f"MoveIt rejected the {action_label} goal")
        terminal_confirmed = False
        try:
            wrapped = self._await_terminal_result(
                result_future,
                cancel_event,
                action_label=action_label,
            )
            terminal_confirmed = True
            return wrapped
        except BaseException:
            # A done future may contain an exception, None, or a nonterminal
            # response.  ``done()`` alone is never evidence that robot motion
            # stopped, so cancellation is attempted for every failure path.
            self.cancel_active()
            with self._lock:
                if (
                    self._active_goal is handle
                    and self._active_goal_generation == generation
                ):
                    self._motion_state_unknown = True
            self._ensure_cancel_supervisor(handle, generation)
            raise
        finally:
            if terminal_confirmed:
                self._clear_tracked_goal(handle, generation)

    def _validate_planned_trajectory(
        self,
        trajectory: Any,
        start_positions_rad: Sequence[float],
        target_positions_rad: Sequence[float],
    ) -> None:
        """Validate the exact MoveIt plan before it can reach execution."""

        expected_names = tuple(str(name) for name in self.environment.joint_names)
        names = tuple(str(name) for name in trajectory.joint_trajectory.joint_names)
        if len(set(names)) != len(names) or set(names) != set(expected_names):
            raise RuntimeError(
                "MoveIt planned trajectory joint set does not exactly match the "
                "configured group"
            )
        points = tuple(trajectory.joint_trajectory.points)
        if not points:
            raise RuntimeError("MoveIt returned an empty planned trajectory")
        multi_dof = getattr(trajectory, "multi_dof_joint_trajectory", None)
        if multi_dof is not None and (
            tuple(getattr(multi_dof, "joint_names", ()))
            or tuple(getattr(multi_dof, "points", ()))
        ):
            raise RuntimeError("manual UR plan unexpectedly contains multi-DOF joints")

        start = tuple(float(value) for value in start_positions_rad)
        target = tuple(float(value) for value in target_positions_rad)
        if (
            len(start) != len(expected_names)
            or len(target) != len(expected_names)
            or not all(math.isfinite(value) for value in (*start, *target))
        ):
            raise RuntimeError("planned trajectory validation received invalid endpoints")
        path_bounds = manual_joint_path_bounds(
            expected_names,
            start,
            self.environment.planner["ik"]["canonical_joint_ranges_rad"],
            max_excursion_rad=math.radians(self.args.max_joint_excursion_deg),
        )
        name_indices = {name: index for index, name in enumerate(names)}
        ordered_rows: list[tuple[float, ...]] = []
        times_s: list[float] = []
        width = len(names)
        for point_index, point in enumerate(points):
            positions = tuple(float(value) for value in point.positions)
            if len(positions) != width or not all(
                math.isfinite(value) for value in positions
            ):
                raise RuntimeError(
                    f"MoveIt planned trajectory point {point_index} has invalid positions"
                )
            for field_name in ("velocities", "accelerations", "effort"):
                values = tuple(float(value) for value in getattr(point, field_name))
                if len(values) not in {0, width} or not all(
                    math.isfinite(value) for value in values
                ):
                    raise RuntimeError(
                        f"MoveIt planned trajectory point {point_index} has invalid "
                        f"{field_name}"
                    )
            duration = point.time_from_start
            seconds = int(duration.sec)
            nanoseconds = int(duration.nanosec)
            if seconds < 0 or nanoseconds < 0 or nanoseconds >= 1_000_000_000:
                raise RuntimeError(
                    f"MoveIt planned trajectory point {point_index} has invalid time"
                )
            times_s.append(float(seconds) + float(nanoseconds) * 1.0e-9)
            ordered = tuple(positions[name_indices[name]] for name in expected_names)
            for name, value, (lower, upper) in zip(
                expected_names,
                ordered,
                path_bounds,
                strict=True,
            ):
                if value < lower - 1.0e-9 or value > upper + 1.0e-9:
                    raise RuntimeError(
                        f"MoveIt planned trajectory point {point_index} leaves the "
                        f"local/canonical interval for {name}"
                    )
            ordered_rows.append(ordered)
        if any(
            right <= left
            for left, right in zip(times_s, times_s[1:])
        ):
            raise RuntimeError("MoveIt planned trajectory times are not strictly increasing")
        start_error = max(
            abs(actual - expected)
            for actual, expected in zip(ordered_rows[0], start, strict=True)
        )
        if start_error > MAX_TARGET_DERIVATION_JOINT_DRIFT_RAD:
            raise RuntimeError(
                f"MoveIt planned trajectory start differs by {start_error:.6f} rad "
                "from the verified actual start"
            )
        target_error = max(
            abs(actual - expected)
            for actual, expected in zip(ordered_rows[-1], target, strict=True)
        )
        if target_error > MANUAL_JOINT_GOAL_TOLERANCE_RAD + 1.0e-9:
            raise RuntimeError(
                f"MoveIt planned trajectory endpoint differs by {target_error:.6f} rad "
                "from the collision-checked IK target"
            )

    def _build_execute_goal(self, trajectory: Any) -> Any:
        goal = self.types["ExecuteTrajectory"].Goal()
        # Execute the exact RobotTrajectory object that passed validation.
        # Reconstructing or copying it here would make identity-based tests and
        # audits unable to prove that no trajectory mutation happened between
        # planning and execution.
        goal.trajectory = trajectory
        goal.controller_names = [self.args.required_controller]
        return goal

    def _execute_tip_target(
        self,
        target_builder: Callable[[np.ndarray, np.ndarray], np.ndarray],
        *,
        cancel_event: threading.Event | None,
        max_translation_m: float | None = None,
    ) -> MotionDelta:
        if not self._command_lock.acquire(blocking=False):
            raise RuntimeError(
                "another manual command is already in progress; commands are never queued"
            )
        servo_paused = False
        try:
            servo_paused = self._pause_servo_for_move_group()
            if max_translation_m is None:
                result = self._execute_tip_target_reserved(
                    target_builder,
                    cancel_event=cancel_event,
                )
            else:
                result = self._execute_tip_target_reserved(
                    target_builder,
                    cancel_event=cancel_event,
                    max_translation_m=max_translation_m,
                )
        except BaseException as motion_error:
            if servo_paused and not self.unresolved_motion():
                try:
                    self._resume_servo_after_move_group()
                except BaseException as servo_error:
                    with self._lock:
                        self._servo_configured = False
                        self._servo_paused = True
                    raise RuntimeError(
                        f"{motion_error}; MoveIt Servo could not be safely re-armed: "
                        f"{servo_error}"
                    ) from motion_error
            raise
        else:
            if servo_paused:
                self._resume_servo_after_move_group()
            return result
        finally:
            self._command_lock.release()

    def _execute_tip_target_reserved(
        self,
        target_builder: Callable[[np.ndarray, np.ndarray], np.ndarray],
        *,
        cancel_event: threading.Event | None,
        max_translation_m: float | None = None,
    ) -> MotionDelta:
        if not self.args.enable_motion:
            raise RuntimeError("read-only mode; restart with --enable-motion")
        with self._lock:
            if self._active_goal is not None or self._motion_state_unknown:
                raise RuntimeError(
                    "a MoveGroup goal is active or unresolved; commands are never queued"
                )
        self._ensure_not_cancelled(cancel_event)
        self._runtime_absent()
        self.verify_required_controller()
        self._ensure_not_cancelled(cancel_event)
        self.live_authoritative_transforms()
        self._ensure_not_cancelled(cancel_event)
        self.verify_scene()
        if not self.move_client.wait_for_server(timeout_sec=self.args.service_timeout_s):
            raise RuntimeError(f"MoveIt action {MOVE_ACTION} is unavailable")
        if not self.execute_client.wait_for_server(
            timeout_sec=self.args.service_timeout_s
        ):
            raise RuntimeError(
                f"MoveIt action {EXECUTE_TRAJECTORY_ACTION} is unavailable"
            )
        self._ensure_not_cancelled(cancel_event)
        self.verify_required_controller()
        self._runtime_absent()
        self._ensure_not_cancelled(cancel_event)
        # All slower service/discovery preflights are complete.  Establish the
        # settled start state only now, then derive the jog from its live TCP.
        pre_derivation_snapshot = self.wait_settled(
            timeout_s=self.args.state_timeout_s
        )
        self._ensure_not_cancelled(cancel_event)
        derivation_started_s = time.monotonic()
        live_base_to_plane1, live_tool0_to_tip = self.live_authoritative_transforms()
        current = self.current_tip_matrix(tool0_to_tip=live_tool0_to_tip)
        target_base_to_tip = target_builder(current, live_base_to_plane1)
        delta = enforce_motion_delta(
            current,
            target_base_to_tip,
            max_translation_m=(
                self.args.max_translation_m
                if max_translation_m is None
                else max_translation_m
            ),
            max_rotation_deg=self.args.max_rotation_deg,
        )
        target_tool = rod_tip_target_to_tool0(
            self.tool_builder, target_base_to_tip, frame_id=self.base_frame
        )
        # Prove the robot remained quiet throughout TF sampling and target
        # derivation, then use the newest actual joints for local path bounds.
        start_snapshot = self.wait_settled(
            timeout_s=self.args.state_timeout_s,
            after_s=derivation_started_s,
        )
        self._require_negligible_joint_drift(
            pre_derivation_snapshot,
            start_snapshot,
            label="while live TCP and target were derived",
        )
        self._ensure_not_cancelled(cancel_event)
        # This final action discovery check intentionally sits immediately
        # before submission.  The dedicated manual launch omits the runtime;
        # if a stray runtime is nevertheless discoverable, fail closed.
        self._runtime_absent()
        self._ensure_not_cancelled(cancel_event)
        submission_snapshot = self._settled_snapshot()
        if submission_snapshot is None:
            raise RuntimeError(
                "robot stopped being settled immediately before MoveGroup submission"
            )
        self._require_negligible_joint_drift(
            start_snapshot,
            submission_snapshot,
            label="immediately before MoveGroup submission",
        )
        target_positions = self._solve_collision_checked_ik(
            target_tool,
            submission_snapshot.positions_rad,
        )
        self._ensure_not_cancelled(cancel_event)
        # IK is a bounded service call, but it still creates a time window in
        # which the runtime could appear or the physical arm could move.  Bind
        # the joint-space plan only after both facts are checked again.
        self._runtime_absent()
        self._ensure_not_cancelled(cancel_event)
        post_ik_snapshot = self._settled_snapshot()
        if post_ik_snapshot is None:
            raise RuntimeError(
                "robot stopped being settled while collision IK was solved"
            )
        self._require_negligible_joint_drift(
            submission_snapshot,
            post_ik_snapshot,
            label="while collision IK was solved",
        )
        goal = self._build_goal(
            target_positions,
            post_ik_snapshot.positions_rad,
        )
        self._ensure_not_cancelled(cancel_event)
        planned = self._run_tracked_action(
            self.move_client,
            goal,
            action_label="MoveGroup planning",
            cancel_event=cancel_event,
        )
        plan_result = planned.result
        if int(plan_result.error_code.val) != int(
            self.types["MoveItErrorCodes"].SUCCESS
        ):
            raise RuntimeError(
                "MoveIt joint-space planning failed with "
                + self._moveit_error_description(plan_result.error_code)
                + f" (action status {planned.status})"
            )
        if int(planned.status) != int(self.types["GoalStatus"].STATUS_SUCCEEDED):
            raise RuntimeError(
                f"MoveIt planning ended with terminal status {planned.status}; "
                "manual motion was not executed"
            )
        trajectory = plan_result.planned_trajectory
        self._validate_planned_trajectory(
            trajectory,
            post_ik_snapshot.positions_rad,
            target_positions,
        )
        plan_finished_at = time.monotonic()

        # The exact plan was produced with the explicit start state and the
        # complete authoritative scene.  Before execution, require the live
        # controller, runtime exclusion, scene, attached rod, ACM, and physical
        # joints to remain unchanged.
        self._ensure_not_cancelled(cancel_event)
        self.verify_required_controller()
        self._runtime_absent()
        self.verify_scene()
        execution_start = self.wait_settled(
            timeout_s=self.args.state_timeout_s,
            after_s=plan_finished_at,
        )
        self._require_negligible_joint_drift(
            post_ik_snapshot,
            execution_start,
            label="while the collision-checked joint trajectory was planned",
        )
        # Compare the controller's fresh start directly with the exact plan's
        # first point.  Two individually tolerated offsets relative to the old
        # snapshot must not add up across the plan/execute boundary.
        self._validate_planned_trajectory(
            trajectory,
            execution_start.positions_rad,
            target_positions,
        )
        self._ensure_not_cancelled(cancel_event)

        def final_execute_gate() -> None:
            # Action discovery itself is a bounded wait.  Recheck every live
            # execution prerequisite after that wait and take the joint sample
            # last, immediately before send_goal_async().
            self.verify_required_controller()
            self._ensure_not_cancelled(cancel_event)
            self.verify_scene()
            self._ensure_not_cancelled(cancel_event)
            self._runtime_absent()
            final_start = self._settled_snapshot()
            if final_start is None:
                raise RuntimeError(
                    "robot stopped being settled immediately before "
                    "ExecuteTrajectory submission"
                )
            self._require_negligible_joint_drift(
                execution_start,
                final_start,
                label="immediately before ExecuteTrajectory submission",
            )
            self._validate_planned_trajectory(
                trajectory,
                final_start.positions_rad,
                target_positions,
            )

        execute_goal = self._build_execute_goal(trajectory)
        executed = self._run_tracked_action(
            self.execute_client,
            execute_goal,
            action_label="ExecuteTrajectory",
            cancel_event=cancel_event,
            pre_submit=final_execute_gate,
        )
        execute_result = executed.result
        if int(execute_result.error_code.val) != int(
            self.types["MoveItErrorCodes"].SUCCESS
        ):
            raise RuntimeError(
                "MoveIt trajectory execution failed with "
                + self._moveit_error_description(execute_result.error_code)
                + f" (action status {executed.status})"
            )
        if int(executed.status) != int(
            self.types["GoalStatus"].STATUS_SUCCEEDED
        ):
            raise RuntimeError(
                f"MoveIt trajectory execution ended with terminal status "
                f"{executed.status}; manual motion did not succeed"
            )
        finished_at = time.monotonic()

        self.wait_settled(timeout_s=self.args.state_timeout_s, after_s=finished_at)
        actual = self.current_tip_matrix()
        return enforce_post_motion_result(
            current,
            target_base_to_tip,
            actual,
            max_position_error_m=self.args.post_position_tolerance_m,
            max_orientation_error_deg=self.args.post_orientation_tolerance_deg,
        )

    def move_tip(
        self,
        target_base_to_tip: np.ndarray,
        *,
        cancel_event: threading.Event | None = None,
    ) -> MotionDelta:
        target = np.asarray(target_base_to_tip, dtype=float).copy()
        return self._execute_tip_target(
            lambda _current, _base_to_plane1: target,
            cancel_event=cancel_event,
        )

    def move_absolute(
        self,
        command: AbsolutePoseCommand,
        *,
        cancel_event: threading.Event | None = None,
    ) -> MotionDelta:
        return self._execute_tip_target(
            lambda _current, base_to_plane1: absolute_tip_target_in_base(
                command, base_to_plane1
            ),
            cancel_event=cancel_event,
        )

    def move_absolute_position(
        self,
        command: AbsolutePositionCommand,
        *,
        cancel_event: threading.Event | None = None,
    ) -> MotionDelta:
        """Move to absolute XYZ while retaining orientation at dispatch time."""

        return self._execute_tip_target(
            lambda current, base_to_plane1: absolute_position_target_in_base(
                command,
                current,
                base_to_plane1,
            ),
            cancel_event=cancel_event,
            max_translation_m=self.args.max_absolute_translation_m,
        )

    def move_jog(
        self,
        key: str,
        jog_frame: str,
        *,
        step_m: float | None = None,
        cancel_event: threading.Event | None = None,
    ) -> MotionDelta:
        # Capture the requested step in this command instead of reading a
        # mutable UI setting later in the motion thread.  The terminal keeps
        # using its validated startup default; graphical callers may select a
        # different step for each explicit button click.
        command_step_m = self.args.jog_step_m if step_m is None else float(step_m)
        command_delta = jog_delta_for_key(key, command_step_m, jog_frame)
        largest_position_tolerance_m = max(
            self.args.goal_position_tolerance_m,
            self.args.post_position_tolerance_m,
        )
        if largest_position_tolerance_m * 2.0 >= command_step_m:
            raise ValueError(
                "jog step must be more than twice the goal and post-position "
                "tolerances so success proves actual TCP motion"
            )

        def build_target(current: np.ndarray, base_to_plane1: np.ndarray) -> np.ndarray:
            frame_transform = np.eye(4) if jog_frame == "base" else base_to_plane1
            return jog_tip_target_in_base(current, frame_transform, command_delta)

        return self._execute_tip_target(build_target, cancel_event=cancel_event)

    def close(self) -> None:
        """Cancel and retain ROS ownership until every submitted goal is terminal.

        Destroying the node while a goal is unresolved would turn a client-side
        timeout into an unobserved robot action.  This method therefore refuses
        teardown and keeps retrying cancellation.  The UR pendant remains the
        physical emergency-stop authority if the ROS graph is unavailable.
        """

        # A Servo stream is not an action and therefore has no CancelGoal
        # handshake.  Clear its generation and put a fresh zero command on the
        # wire before waiting on any MoveGroup action below.
        try:
            self.stop_servo_jog()
        except BaseException:
            pass
        next_cancel_s = 0.0
        next_notice_s = time.monotonic() + self.args.cancel_confirmation_timeout_s
        while True:
            try:
                if not self.unresolved_motion():
                    break
                now = time.monotonic()
                if now >= next_cancel_s:
                    self.cancel_active()
                    next_cancel_s = time.monotonic() + 0.5
                if time.monotonic() >= next_notice_s:
                    print(
                        "tcp-console: MoveGroup goal state is unresolved; refusing ROS "
                        "shutdown and retaining the motion lock. Keep the UR pendant "
                        "ready; use its stop/emergency-stop if needed.",
                        file=sys.stderr,
                        flush=True,
                    )
                    next_notice_s = time.monotonic() + 5.0
                time.sleep(0.05)
            except KeyboardInterrupt:
                print(
                    "tcp-console: shutdown interrupt ignored while robot motion "
                    "state is unresolved",
                    file=sys.stderr,
                    flush=True,
                )
        # Once no goal is owned, supervisors are guaranteed to be on their way
        # out.  Join them before destroying ROS resources so none can issue a
        # stale CancelGoal/GetResult request against a torn-down node.
        with self._lock:
            self._ensure_goal_tracking_fields()
            supervisors = tuple(self._cancel_supervisors.values())
        for supervisor in supervisors:
            if supervisor is threading.current_thread():
                continue
            while supervisor.is_alive():
                try:
                    supervisor.join(timeout=0.25)
                except KeyboardInterrupt:
                    print(
                        "tcp-console: shutdown interrupt ignored while the MoveIt "
                        "cancellation supervisor is releasing ROS resources",
                        file=sys.stderr,
                        flush=True,
                    )
        if getattr(self, "_servo_configured", False):
            try:
                self._publish_servo_twist(
                    self.plane_frame, (0.0, 0.0, 0.0)
                )
                self.wait_settled(timeout_s=self.args.state_timeout_s)
                self._call_servo_pause(True)
            except BaseException:
                # The GUI-owned launch process group is stopped immediately
                # after the ROS session closes.  The Servo input watchdog and
                # the zero command remain the primary halt mechanisms here.
                pass
        self.executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self.node.destroy_node()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="tcp-console")
    parser.add_argument("--workspace", default=".")
    parser.add_argument("--env", required=True)
    parser.add_argument("--once", action="store_true", help="print one live snapshot and exit")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--jog-frame", choices=("base", "plane1"), default="plane1")
    parser.add_argument("--jog-step-mm", type=float, default=1.0)
    parser.add_argument("--max-translation-mm", type=float, default=50.0)
    parser.add_argument("--max-absolute-translation-mm", type=float, default=150.0)
    parser.add_argument("--max-rotation-deg", type=float, default=10.0)
    parser.add_argument("--max-joint-excursion-deg", type=float, default=20.0)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    # The matching manual bringup activates the UR MoveIt controller selected
    # by its installed controller map.  This is not user-overridable because a
    # name-only override could silently break the launch/MoveIt contract.
    parser.set_defaults(required_controller=MANUAL_MOVEIT_CONTROLLER)
    parser.add_argument("--move-group", default="ur_manipulator")
    parser.add_argument("--pipeline-id", default="ompl")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--allowed-planning-time", dest="allowed_planning_time_s", type=float, default=10.0)
    parser.add_argument("--motion-timeout", dest="motion_timeout_s", type=float, default=60.0)
    parser.add_argument(
        "--cancel-confirmation-timeout",
        dest="cancel_confirmation_timeout_s",
        type=float,
        default=10.0,
    )
    parser.add_argument("--service-timeout", dest="service_timeout_s", type=float, default=5.0)
    parser.add_argument("--state-timeout", dest="state_timeout_s", type=float, default=5.0)
    parser.add_argument("--state-freshness", dest="state_freshness_s", type=float, default=0.5)
    parser.add_argument("--settle-window", dest="settle_window_s", type=float, default=0.25)
    parser.add_argument("--settled-velocity", dest="settled_velocity_rad_s", type=float, default=0.02)
    parser.add_argument("--settled-position-span", dest="settled_position_span_rad", type=float, default=0.003)
    parser.add_argument("--velocity-scale", type=float, default=0.10)
    parser.add_argument("--acceleration-scale", type=float, default=0.10)
    parser.add_argument("--goal-position-tolerance-mm", type=float, default=0.2)
    parser.add_argument("--goal-orientation-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--post-position-tolerance-mm", type=float, default=0.4)
    parser.add_argument("--post-orientation-tolerance-deg", type=float, default=1.0)
    return parser


def build_argument_parser() -> argparse.ArgumentParser:
    return _build_parser()


def _validate_args(args: argparse.Namespace) -> None:
    if (
        not args.required_controller
        or any(
            character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-"
            for character in args.required_controller
        )
    ):
        raise ValueError("--required-controller must be a plain controller name")
    args.jog_step_m = _validate_positive(args.jog_step_mm, "--jog-step-mm") / 1000.0
    if args.jog_step_m > MAX_JOG_STEP_M:
        raise ValueError("--jog-step-mm has a hard maximum of 5 mm")
    args.max_translation_m = _validate_positive(
        args.max_translation_mm, "--max-translation-mm"
    ) / 1000.0
    args.max_absolute_translation_m = _validate_positive(
        args.max_absolute_translation_mm, "--max-absolute-translation-mm"
    ) / 1000.0
    args.max_rotation_deg = _validate_positive(args.max_rotation_deg, "--max-rotation-deg")
    args.max_joint_excursion_deg = _validate_positive(
        args.max_joint_excursion_deg, "--max-joint-excursion-deg"
    )
    if args.max_joint_excursion_deg > 180.0:
        raise ValueError("--max-joint-excursion-deg cannot exceed 180 degrees")
    args.goal_position_tolerance_m = _validate_positive(
        args.goal_position_tolerance_mm, "--goal-position-tolerance-mm"
    ) / 1000.0
    args.post_position_tolerance_m = _validate_positive(
        args.post_position_tolerance_mm, "--post-position-tolerance-mm"
    ) / 1000.0
    for label, tolerance_m in (
        ("--goal-position-tolerance-mm", args.goal_position_tolerance_m),
        ("--post-position-tolerance-mm", args.post_position_tolerance_m),
    ):
        if tolerance_m * 2.0 >= args.jog_step_m:
            raise ValueError(
                f"{label} must be less than half --jog-step-mm so a successful "
                "jog proves actual TCP motion"
            )
    for name in (
        "goal_orientation_tolerance_deg", "post_orientation_tolerance_deg",
        "allowed_planning_time_s", "motion_timeout_s", "service_timeout_s",
        "cancel_confirmation_timeout_s",
        "state_timeout_s", "state_freshness_s", "settle_window_s",
        "settled_velocity_rad_s", "settled_position_span_rad",
    ):
        setattr(args, name, _validate_positive(getattr(args, name), f"--{name.replace('_', '-')}"))
    if args.planning_attempts <= 0:
        raise ValueError("--planning-attempts must be positive")
    for name in ("velocity_scale", "acceleration_scale"):
        value = _validate_positive(getattr(args, name), f"--{name.replace('_', '-')}")
        if value > 1.0:
            raise ValueError(f"--{name.replace('_', '-')} cannot exceed 1")


def _line_help_text(jog_frame: str, step_mm: float) -> str:
    vertical = (
        "R/PageUp=-Z, F/PageDown=+Z (MLA: physical raise/lower)"
        if jog_frame == "plane1"
        else "R/PageUp=+Z, F/PageDown=-Z (base vertical axis)"
    )
    return (
        f"jog frame={jog_frame}, step={step_mm:.3f} mm\n"
        "  A/left=-X, D/right=+X, W/up=+Y, S/down=-Y\n"
        f"  {vertical}; T toggles jog frame\n"
        "  G base p[x,y,z,rx,ry,rz] | G plane1 p[...] (explicit frame required)\n"
        "  SPACE then Enter arms exactly the next move; C clears error; Enter refreshes; Q quits"
    )


def _interactive_line(session: RosTcpSession, args: argparse.Namespace) -> int:
    """Original line-oriented interface retained for pipes and non-TTY shells."""

    armed = False
    jog_frame = args.jog_frame
    error = ""
    while True:
        print("\n" + session.snapshot_text())
        print(_line_help_text(jog_frame, args.jog_step_mm))
        print("mode: " + ("MOTION ENABLED" if args.enable_motion else "READ ONLY"))
        print("arm: " + ("ARMED FOR ONE MOVE" if armed else "disarmed"))
        if error:
            print("ERROR: " + error)
        try:
            raw = input("tcp> ")
        except EOFError:
            return 0
        command = raw.strip()
        lower = command.lower()
        if lower in {"q", "quit", "exit"}:
            return 0
        if lower in {"c", "clear"}:
            error = ""
            continue
        if lower in {"t", "frame"}:
            armed = False
            jog_frame = "base" if jog_frame == "plane1" else "plane1"
            error = "jog frame changed; motion disarmed"
            continue
        if raw == " " or lower == "arm":
            if not args.enable_motion:
                error = "read-only mode; --enable-motion was not supplied"
            else:
                armed = not armed
                error = ""
            continue
        if not command:
            continue
        try:
            if not armed:
                raise RuntimeError("motion is disarmed; enter one SPACE then Enter first")
            # One-shot arming is consumed even if preflight or planning refuses.
            armed = False
            if lower in _JOG_KEYS or lower in _Z_JOG_KEYS:
                moved = session.move_jog(lower, jog_frame)
            elif lower.startswith("g "):
                moved = session.move_absolute(
                    parse_absolute_pose(command[2:])
                )
            else:
                raise ValueError("unknown command")
            error = ""
            print(
                f"completed: {moved.translation_m * 1000.0:.3f} mm, "
                f"{moved.rotation_deg:.3f} deg"
            )
        except (ValueError, RuntimeError) as exc:
            armed = False
            error = str(exc)


def _curses_help_lines(jog_frame: str, step_mm: float) -> tuple[str, ...]:
    vertical = (
        "R/PageUp=-Z, F/PageDown=+Z (MLA: physical raise/lower)"
        if jog_frame == "plane1"
        else "R/PageUp=+Z, F/PageDown=-Z (base vertical axis)"
    )
    return (
        f"Jog: frame={jog_frame}, step={step_mm:.3f} mm",
        "A/Left=-X   D/Right=+X   W/Up=+Y   S/Down=-Y",
        vertical,
        "Space=arm one move   G=absolute rod_tip pose   T=toggle jog frame",
        "Esc=disarm/cancel   C=clear UI error   Q=quit",
        "Absolute syntax after G: base p[x,y,z,rx,ry,rz] or plane1 p[...]",
    )


def _draw_curses_screen(
    screen: Any,
    curses_module: Any,
    *,
    snapshot_text: str,
    snapshot_error: str,
    args: argparse.Namespace,
    jog_frame: str,
    armed: bool,
    worker: MotionWorkerSnapshot,
    ui_error: str,
    input_buffer: str | None,
) -> None:
    """Render a whole frame defensively, including on very small terminals."""

    height, width = screen.getmaxyx()
    screen.erase()
    lines = [
        f"Live rod_tip TCP console ({UI_REFRESH_HZ:.0f} Hz)",
        "",
        *snapshot_text.splitlines(),
    ]
    if snapshot_error:
        lines.append("SNAPSHOT ERROR: " + snapshot_error)
    lines.extend(
        (
            "",
            "mode: " + ("MOTION ENABLED" if args.enable_motion else "READ ONLY"),
            "arm: " + ("ARMED FOR EXACTLY ONE MOVE" if armed else "disarmed"),
            "motion: " + worker.message,
        )
    )
    if worker.running:
        lines.append("motion keys are ignored while this command is running (no queue)")
    if worker.error:
        lines.append("MOTION ERROR: " + worker.error)
    if ui_error:
        lines.append("UI ERROR: " + ui_error)
    lines.extend(("", *_curses_help_lines(jog_frame, args.jog_step_mm)))
    if input_buffer is not None:
        lines.extend(("", "target> " + input_buffer))

    if width > 1:
        for row, line in enumerate(lines[:height]):
            try:
                screen.addnstr(row, 0, line, width - 1)
            except curses_module.error:
                pass
    try:
        curses_module.curs_set(1 if input_buffer is not None else 0)
    except curses_module.error:
        pass
    screen.refresh()


def _start_curses_jog(
    worker: OneMotionWorker,
    session: RosTcpSession,
    *,
    key: str,
    jog_frame: str,
) -> bool:
    def operation(cancel_event: threading.Event) -> MotionDelta:
        return session.move_jog(key, jog_frame, cancel_event=cancel_event)

    return worker.start(operation, label=f"{jog_frame} {key} jog")


def _start_curses_absolute(
    worker: OneMotionWorker,
    session: RosTcpSession,
    command: AbsolutePoseCommand,
) -> bool:
    def operation(cancel_event: threading.Event) -> MotionDelta:
        return session.move_absolute(command, cancel_event=cancel_event)

    return worker.start(operation, label=f"absolute {command.frame} rod_tip pose")


def _interactive_curses_loop(
    screen: Any,
    session: RosTcpSession,
    args: argparse.Namespace,
    curses_module: Any,
) -> int:
    """Responsive 5 Hz display with one unqueued background motion at a time."""

    screen.keypad(True)
    # Poll faster than the display cadence so Esc remains responsive.
    screen.timeout(40)
    special = TerminalKeyCodes(
        left=curses_module.KEY_LEFT,
        right=curses_module.KEY_RIGHT,
        up=curses_module.KEY_UP,
        down=curses_module.KEY_DOWN,
        page_up=curses_module.KEY_PPAGE,
        page_down=curses_module.KEY_NPAGE,
    )
    worker = OneMotionWorker()
    armed = False
    jog_frame = args.jog_frame
    ui_error = ""
    input_buffer: str | None = None
    snapshot = "waiting for first live snapshot..."
    snapshot_error = ""
    next_snapshot_s = 0.0
    previous_worker_running = False
    quit_pending = False

    try:
        while True:
            now = time.monotonic()
            if now >= next_snapshot_s:
                try:
                    snapshot = session.snapshot_text()
                    snapshot_error = ""
                except Exception as error:
                    snapshot_error = str(error)
                # Avoid catch-up bursts if a TF lookup itself took over 200 ms.
                next_snapshot_s = time.monotonic() + UI_REFRESH_PERIOD_S

            worker_state = worker.snapshot()
            if previous_worker_running and not worker_state.running:
                # Clear terminal-driver autorepeat and every key typed while
                # the command was active. None may become a later motion.
                curses_module.flushinp()
            previous_worker_running = worker_state.running
            if (
                quit_pending
                and not worker_state.running
                and not session.unresolved_motion()
            ):
                return 0
            _draw_curses_screen(
                screen,
                curses_module,
                snapshot_text=snapshot,
                snapshot_error=snapshot_error,
                args=args,
                jog_frame=jog_frame,
                armed=armed,
                worker=worker_state,
                ui_error=ui_error,
                input_buffer=input_buffer,
            )
            key_code = screen.getch()
            if key_code == -1:
                continue

            if input_buffer is not None:
                if key_code == 27:
                    input_buffer = None
                    armed = False
                    ui_error = "absolute input cancelled; motion disarmed"
                    continue
                if key_code in (ord("\n"), ord("\r"), curses_module.KEY_ENTER):
                    pose_text = input_buffer
                    input_buffer = None
                    try:
                        command = parse_absolute_pose(pose_text)
                        if not armed:
                            raise RuntimeError("motion is disarmed; press Space first")
                        armed = False
                        if not _start_curses_absolute(worker, session, command):
                            raise RuntimeError(
                                "motion command ignored because another motion is running"
                            )
                        ui_error = ""
                    except (ValueError, RuntimeError) as error:
                        armed = False
                        ui_error = str(error)
                    continue
                if key_code in (8, 127, curses_module.KEY_BACKSPACE):
                    input_buffer = input_buffer[:-1]
                    continue
                if 32 <= key_code <= 126 and len(input_buffer) < 255:
                    input_buffer += chr(key_code)
                continue

            command = decode_terminal_key(key_code, special)
            if command is None:
                continue
            worker_state = worker.snapshot()
            if command == "quit":
                armed = False
                cancel_requested = worker.request_cancel(session.cancel_active)
                if session.unresolved_motion() and not cancel_requested:
                    threading.Thread(
                        target=session.cancel_active,
                        name="tcp-console-final-cancel",
                        daemon=True,
                    ).start()
                    cancel_requested = True
                if cancel_requested:
                    quit_pending = True
                    ui_error = "cancelling current motion before exit"
                    continue
                return 0
            if command == "escape":
                armed = False
                if worker.request_cancel(session.cancel_active):
                    ui_error = "cancellation requested; motion remains locked until it ends"
                elif session.unresolved_motion():
                    threading.Thread(
                        target=session.cancel_active,
                        name="tcp-console-retry-cancel",
                        daemon=True,
                    ).start()
                    ui_error = "retrying cancellation of unresolved MoveGroup goal"
                else:
                    ui_error = "motion disarmed"
                continue
            if command == "clear":
                ui_error = ""
                continue
            if command == "toggle-frame":
                armed = False
                jog_frame = "base" if jog_frame == "plane1" else "plane1"
                ui_error = "jog frame changed; motion disarmed"
                continue
            if command == "arm":
                if quit_pending:
                    continue
                if worker_state.running or session.unresolved_motion():
                    ui_error = "arm key ignored while motion is running"
                elif not args.enable_motion:
                    ui_error = "read-only mode; --enable-motion was not supplied"
                else:
                    armed = True
                    ui_error = ""
                continue
            if command == "absolute":
                if quit_pending:
                    continue
                if worker_state.running or session.unresolved_motion():
                    ui_error = "G ignored while motion is running (no queue)"
                else:
                    input_buffer = ""
                    ui_error = ""
                continue
            if command in _JOG_KEYS or command in _Z_JOG_KEYS:
                if quit_pending:
                    continue
                if worker_state.running or session.unresolved_motion():
                    ui_error = "motion key ignored while motion is running (no queue)"
                    continue
                if not armed:
                    ui_error = "motion is disarmed; press Space first"
                    continue
                # One-shot arming is consumed by this one command, regardless
                # of whether planning subsequently accepts the target.
                armed = False
                if _start_curses_jog(
                    worker,
                    session,
                    key=command,
                    jog_frame=jog_frame,
                ):
                    ui_error = ""
                else:
                    ui_error = "motion command ignored because another motion is running"
    finally:
        armed = False
        worker.request_cancel(session.cancel_active)
        # Do not let session.close() tear ROS down while a preflight thread can
        # still cross into goal submission.  All real preflight waits are
        # bounded; keep consuming Ctrl-C as another cancellation request until
        # this one unqueued operation has actually returned.
        while worker.snapshot().running:
            try:
                worker.join(timeout_s=0.25)
            except KeyboardInterrupt:
                worker.request_cancel(session.cancel_active)


def _interactive_curses(session: RosTcpSession, args: argparse.Namespace) -> int:
    import curses

    return int(
        curses.wrapper(_interactive_curses_loop, session, args, curses)
    )


def _interactive(session: RosTcpSession, args: argparse.Namespace) -> int:
    if sys.stdin.isatty() and sys.stdout.isatty():
        return _interactive_curses(session, args)
    return _interactive_line(session, args)


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        _validate_args(args)
        workspace = Path(args.workspace).expanduser().resolve()
        scene = load_planning_scene_definition(workspace, args.env)
        if scene.environment_id != args.env:
            raise RuntimeError(
                f"selected environment {args.env!r} resolved to unexpected "
                f"identity {scene.environment_id!r}"
            )
        environment = EnvironmentBuilder().load(
            workspace / "environments" / args.env
        )
        if environment.environment_id != args.env:
            raise RuntimeError(
                f"environment bundle identity {environment.environment_id!r} "
                f"does not match --env {args.env!r}"
            )
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))

    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
    except ImportError as error:
        print(f"tcp-console requires a sourced ROS 2/MoveIt workspace: {error}")
        return 2

    # Keep Python's KeyboardInterrupt behavior but prevent rclpy's SIGINT
    # handler from shutting the context down before close() can cancel and
    # observe a terminal MoveGroup result.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    session: RosTcpSession | None = None
    try:
        session = RosTcpSession(environment, scene, args)
        # Viewing is valid while the arm is moving.  Settled state is required
        # only by move_tip(), immediately before a manual motion is submitted.
        session.wait_fresh_joint_state(timeout_s=args.state_timeout_s)
        if args.once:
            print(session.snapshot_text())
            return 0
        return _interactive(session, args)
    except KeyboardInterrupt:
        if session is not None:
            session.cancel_active()
        print("\nmanual TCP console cancelled")
        return 130
    except Exception as error:
        print(f"tcp-console failed: {error}")
        return 2
    finally:
        if session is not None:
            session.close()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
