from __future__ import annotations

import math
import sys
import threading
from concurrent.futures import Future
from dataclasses import replace
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

import hx_waypoint_tools.tcp_console as tcp_console
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder, JointPositionDomain
from hx_waypoint_curobo.math3d import matrix_pose, pose_matrix
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_tools.tcp_console import (
    MAX_JOG_STEP_M,
    UI_REFRESH_PERIOD_S,
    AttachedRodObservation,
    MotionDelta,
    OneMotionWorker,
    TerminalKeyCodes,
    WorldCuboidObservation,
    absolute_tip_target_in_base,
    build_argument_parser,
    compare_scene_inventory,
    compose_transform,
    decode_terminal_key,
    enforce_post_motion_result,
    forbidden_acm_allowances,
    inverse_transform,
    jog_delta_for_key,
    jog_pose,
    manual_joint_path_bounds,
    parse_pose_command,
    quaternion_to_rotvec,
    rotvec_to_quaternion_xyzw,
    tip_target_to_tool_target,
)
from hx_waypoint_tools.planning_scene_publisher import load_planning_scene_definition


def _assert_quaternion_equivalent(
    actual: tuple[float, ...],
    expected: tuple[float, ...],
    *,
    abs_tolerance: float = 1.0e-12,
) -> None:
    left = np.asarray(actual, dtype=float)
    right = np.asarray(expected, dtype=float)
    assert min(np.linalg.norm(left - right), np.linalg.norm(left + right)) <= abs_tolerance


@pytest.mark.parametrize("frame", ["base", "plane1", "BASE", "Plane1"])
def test_parse_absolute_pose_requires_and_preserves_explicit_supported_frame(
    frame: str,
) -> None:
    command = parse_pose_command(f"  {frame}  p[0.1, -0.2, 0.3, 0.4, -0.5, 0.6]  ")

    assert command.frame == frame.lower()
    assert command.position_m == pytest.approx((0.1, -0.2, 0.3))
    assert command.rotation_vector_rad == pytest.approx((0.4, -0.5, 0.6))


@pytest.mark.parametrize(
    "text",
    [
        "p[0,0,0,0,0,0]",
        "map p[0,0,0,0,0,0]",
        "base [0,0,0,0,0,0]",
        "base p[0,0,0,0,0]",
        "base p[0,0,0,0,0,0,0]",
        "base p[nan,0,0,0,0,0]",
        "plane1 p[0,0,0,0,inf,0]",
    ],
)
def test_parse_absolute_pose_rejects_missing_frame_ambiguous_or_nonfinite_input(
    text: str,
) -> None:
    with pytest.raises(ValueError):
        parse_pose_command(text)


@pytest.mark.parametrize(
    "rotation_vector",
    [
        (0.0, 0.0, 0.0),
        (0.1, -0.2, 0.3),
        (math.pi, 0.0, 0.0),
        (0.0, -math.pi, 0.0),
    ],
)
def test_rotvec_and_quaternion_conversion_round_trip(
    rotation_vector: tuple[float, float, float],
) -> None:
    quaternion = rotvec_to_quaternion_xyzw(rotation_vector)
    recovered = quaternion_to_rotvec(quaternion)
    recovered_quaternion = rotvec_to_quaternion_xyzw(recovered)

    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)
    _assert_quaternion_equivalent(recovered_quaternion, quaternion)


def test_rigid_transform_inverse_and_composition_return_identity() -> None:
    transform = pose_matrix(
        (0.23, -0.41, 0.17),
        rotvec_to_quaternion_xyzw((0.6, -0.2, 0.4)),
    )

    inverse = inverse_transform(transform)

    assert compose_transform(transform, inverse) == pytest.approx(np.eye(4), abs=1.0e-12)
    assert compose_transform(inverse, transform) == pytest.approx(np.eye(4), abs=1.0e-12)


def test_plane1_absolute_pose_composes_into_base_and_base_pose_does_not() -> None:
    base_to_plane1 = pose_matrix(
        (0.4, -0.2, 0.1),
        rotvec_to_quaternion_xyzw((0.0, 0.0, math.pi / 2.0)),
    )
    plane_command = parse_pose_command("plane1 p[0.1,0.0,-0.07,0,0,0]")
    base_command = parse_pose_command("base p[0.1,0.0,-0.07,0,0,0]")

    expected_plane_target = base_to_plane1 @ pose_matrix(
        plane_command.position_m,
        plane_command.quaternion_xyzw,
    )
    assert absolute_tip_target_in_base(
        plane_command, base_to_plane1
    ) == pytest.approx(expected_plane_target, abs=1.0e-12)
    assert absolute_tip_target_in_base(base_command, base_to_plane1) == pytest.approx(
        pose_matrix(base_command.position_m, base_command.quaternion_xyzw),
        abs=1.0e-12,
    )


def test_rod_tip_target_is_converted_to_tool0_with_authoritative_mla_tcp(
    repository_root: Path,
) -> None:
    environment = EnvironmentBuilder().load(repository_root / "environments" / "mla")
    tool_builder = RobotToolBuilder(environment)
    tip_quaternion = rotvec_to_quaternion_xyzw((math.pi / 2.0, 0.0, 0.0))
    base_to_tip = pose_matrix((0.1, -0.2, 0.4), tip_quaternion)

    target = tip_target_to_tool_target(tool_builder, base_to_tip)
    target_matrix = pose_matrix(target.position_m, target.quaternion_xyzw)

    assert target.frame_id == "base"
    assert target.phase == "approach"
    assert target.time_s == 0.0
    assert target_matrix @ tool_builder.tool0_to_tip == pytest.approx(
        base_to_tip,
        abs=1.0e-12,
    )
    assert target.position_m == pytest.approx((0.1, 0.13365, 0.4), abs=1.0e-12)
    _assert_quaternion_equivalent(target.quaternion_xyzw, tip_quaternion)


@pytest.mark.parametrize(
    ("key", "expected_axis"),
    [
        ("a", (-1.0, 0.0, 0.0)),
        ("right", (1.0, 0.0, 0.0)),
        ("w", (0.0, 1.0, 0.0)),
        ("down", (0.0, -1.0, 0.0)),
        ("r", (0.0, 0.0, 1.0)),
        ("pagedown", (0.0, 0.0, -1.0)),
    ],
)
def test_jog_key_changes_exactly_one_axis(
    key: str,
    expected_axis: tuple[float, float, float],
) -> None:
    step_m = 0.001

    delta = jog_delta_for_key(key, step_m)

    assert delta == pytest.approx(tuple(step_m * value for value in expected_axis))
    assert sum(abs(value) > 0.0 for value in delta) == 1


@pytest.mark.parametrize("step_m", [0.0, -0.001, 0.005000001, math.nan, math.inf])
def test_jog_step_rejects_nonpositive_nonfinite_or_more_than_five_mm(
    step_m: float,
) -> None:
    with pytest.raises(ValueError, match="5 mm|finite"):
        jog_delta_for_key("a", step_m)


def test_jog_accepts_exact_five_mm_hard_limit_and_preserves_tip_orientation() -> None:
    base_to_jog_frame = pose_matrix(
        (0.2, -0.1, 0.3),
        rotvec_to_quaternion_xyzw((0.0, 0.0, math.pi / 2.0)),
    )
    current_tip = pose_matrix(
        (0.4, 0.2, -0.07),
        rotvec_to_quaternion_xyzw((0.2, -0.3, 0.4)),
    )
    delta = jog_delta_for_key("d", MAX_JOG_STEP_M)
    target = jog_pose(current_tip, base_to_jog_frame, "d", MAX_JOG_STEP_M)
    current_position, current_quaternion = matrix_pose(current_tip)
    target_position, target_quaternion = matrix_pose(target)

    expected_base_delta = base_to_jog_frame[:3, :3] @ np.asarray(delta)
    assert np.asarray(target_position) - np.asarray(current_position) == pytest.approx(
        expected_base_delta,
        abs=1.0e-12,
    )
    _assert_quaternion_equivalent(target_quaternion, current_quaternion)


def test_mla_plane1_raise_is_negative_z_and_lower_is_positive_z() -> None:
    assert jog_delta_for_key("r", 0.001, "plane1") == pytest.approx(
        (0.0, 0.0, -0.001)
    )
    assert jog_delta_for_key("f", 0.001, "plane1") == pytest.approx(
        (0.0, 0.0, 0.001)
    )


def test_one_mm_jog_cannot_report_success_when_actual_fk_did_not_move() -> None:
    current = np.eye(4)
    target = np.eye(4)
    target[0, 3] = 0.001

    with pytest.raises(RuntimeError, match="actual rod_tip error|enough actual FK"):
        enforce_post_motion_result(
            current,
            target,
            current,
            max_position_error_m=0.0004,
            max_orientation_error_deg=0.5,
        )


def test_post_motion_result_reports_measured_fk_not_commanded_distance() -> None:
    current = np.eye(4)
    target = np.eye(4)
    target[0, 3] = 0.001
    actual = np.eye(4)
    actual[0, 3] = 0.0008

    measured = enforce_post_motion_result(
        current,
        target,
        actual,
        max_position_error_m=0.0004,
        max_orientation_error_deg=0.5,
    )

    assert measured.translation_m == pytest.approx(0.0008)


@pytest.mark.parametrize("nonfinite", [math.nan, math.inf, -math.inf])
def test_post_motion_result_rejects_nonfinite_actual_fk(nonfinite: float) -> None:
    current = np.eye(4)
    target = np.eye(4)
    actual = np.eye(4)
    actual[0, 3] = nonfinite

    with pytest.raises(RuntimeError, match="actual|FK|finite"):
        enforce_post_motion_result(
            current,
            target,
            actual,
            max_position_error_m=0.0004,
            max_orientation_error_deg=0.5,
        )


def test_session_jog_captures_explicit_gui_step_without_mutating_cli_default() -> None:
    session = tcp_console.RosTcpSession.__new__(tcp_console.RosTcpSession)
    session.args = SimpleNamespace(
        jog_step_m=0.001,
        goal_position_tolerance_m=0.0002,
        post_position_tolerance_m=0.0004,
    )
    captured_target: list[np.ndarray] = []

    def execute(target_builder, *, cancel_event):
        current = np.eye(4)
        target = target_builder(current, np.eye(4))
        captured_target.append(target)
        return tcp_console.motion_delta(current, target)

    session._execute_tip_target = execute

    delta = session.move_jog("d", "base", step_m=0.005)

    assert captured_target[0][:3, 3] == pytest.approx((0.005, 0.0, 0.0))
    assert delta.translation_m == pytest.approx(0.005)
    assert session.args.jog_step_m == pytest.approx(0.001)


def test_position_only_absolute_move_uses_its_independent_translation_limit() -> None:
    session = tcp_console.RosTcpSession.__new__(tcp_console.RosTcpSession)
    session.args = SimpleNamespace(max_absolute_translation_m=0.150)
    captured_limit: list[float] = []
    captured_target: list[np.ndarray] = []

    def execute(
        target_builder,
        *,
        cancel_event,
        max_translation_m,
    ):
        assert cancel_event is None
        captured_limit.append(max_translation_m)
        current = np.eye(4)
        target = target_builder(current, np.eye(4))
        captured_target.append(target)
        return tcp_console.motion_delta(current, target)

    session._execute_tip_target = execute
    command = tcp_console.AbsolutePositionCommand(
        frame="base",
        position_m=(0.0855, 0.0, 0.0),
    )

    delta = session.move_absolute_position(command)

    assert captured_limit == pytest.approx([0.150])
    assert captured_target[0][:3, 3] == pytest.approx((0.0855, 0.0, 0.0))
    assert delta.translation_m == pytest.approx(0.0855)


def test_manual_joint_bounds_prevent_distant_ik_and_intersect_mla_branch() -> None:
    names = ("shoulder_pan_joint", "elbow_joint")
    start = (-3.0, 0.5)
    excursion = math.radians(20.0)

    bounds = manual_joint_path_bounds(
        names,
        start,
        {"shoulder_pan_joint": (-math.pi, 0.0)},
        max_excursion_rad=excursion,
    )

    assert bounds[0] == pytest.approx((-math.pi, -3.0 + excursion))
    assert bounds[1] == pytest.approx((0.5 - excursion, 0.5 + excursion))
    with pytest.raises(RuntimeError, match="outside the configured manual branch"):
        manual_joint_path_bounds(
            names,
            (0.1, 0.5),
            {"shoulder_pan_joint": (-math.pi, 0.0)},
            max_excursion_rad=excursion,
        )


def test_cli_defaults_to_read_only_one_mm_jog() -> None:
    args = build_argument_parser().parse_args(["--env", "mla"])

    assert args.enable_motion is False
    assert args.once is False
    assert args.jog_frame == "plane1"
    assert args.jog_step_mm == pytest.approx(1.0)
    assert args.max_translation_mm == pytest.approx(50.0)
    assert args.max_absolute_translation_mm == pytest.approx(150.0)
    assert args.max_joint_excursion_deg == pytest.approx(20.0)
    assert args.required_controller == "scaled_joint_trajectory_controller"


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_absolute_translation_limit_must_be_finite_and_positive(value: str) -> None:
    args = build_argument_parser().parse_args(
        ["--env", "mla", "--max-absolute-translation-mm", value]
    )

    with pytest.raises(ValueError, match="max-absolute-translation-mm.*finite.*positive"):
        tcp_console._validate_args(args)


def test_curses_key_decoder_supports_letters_arrows_pages_and_safety_keys() -> None:
    keys = TerminalKeyCodes(
        left=1001,
        right=1002,
        up=1003,
        down=1004,
        page_up=1005,
        page_down=1006,
    )

    assert UI_REFRESH_PERIOD_S == pytest.approx(0.2)
    assert decode_terminal_key(ord("A"), keys) == "a"
    assert decode_terminal_key(ord("f"), keys) == "f"
    assert decode_terminal_key(keys.left, keys) == "left"
    assert decode_terminal_key(keys.up, keys) == "up"
    assert decode_terminal_key(keys.page_up, keys) == "pageup"
    assert decode_terminal_key(keys.page_down, keys) == "pagedown"
    assert decode_terminal_key(ord(" "), keys) == "arm"
    assert decode_terminal_key(ord("G"), keys) == "absolute"
    assert decode_terminal_key(27, keys) == "escape"
    assert decode_terminal_key(ord("Q"), keys) == "quit"


def test_motion_worker_rejects_concurrent_command_without_queueing() -> None:
    worker = OneMotionWorker()
    started = threading.Event()
    release = threading.Event()
    second_called = threading.Event()

    def first(_cancel_event: threading.Event) -> MotionDelta:
        started.set()
        assert release.wait(timeout=1.0)
        return MotionDelta(translation_m=0.001, rotation_deg=0.0)

    def second(_cancel_event: threading.Event) -> MotionDelta:
        second_called.set()
        return MotionDelta(translation_m=0.002, rotation_deg=0.0)

    assert worker.start(first, label="first") is True
    assert started.wait(timeout=1.0)
    assert worker.start(second, label="second") is False
    release.set()
    worker.join(timeout_s=1.0)

    snapshot = worker.snapshot()
    assert snapshot.running is False
    assert snapshot.message == "completed: 1.000 mm, 0.000 deg"
    assert second_called.is_set() is False


def test_motion_worker_escape_style_cancel_retries_until_late_goal_can_cancel() -> None:
    worker = OneMotionWorker()
    operation_started = threading.Event()
    cancelled = threading.Event()
    cancel_call_count = 0
    cancel_lock = threading.Lock()

    def operation(cancel_event: threading.Event) -> MotionDelta:
        operation_started.set()
        assert cancel_event.wait(timeout=1.0)
        assert cancelled.wait(timeout=1.0)
        raise RuntimeError("cancelled by operator")

    def cancel_active() -> None:
        nonlocal cancel_call_count
        with cancel_lock:
            cancel_call_count += 1
            if cancel_call_count >= 2:
                cancelled.set()

    assert worker.start(operation, label="preflight then move") is True
    assert operation_started.wait(timeout=1.0)
    assert worker.request_cancel(cancel_active) is True
    worker.join(timeout_s=1.0)

    snapshot = worker.snapshot()
    assert cancel_call_count >= 2
    assert snapshot.running is False
    assert snapshot.error == "cancelled by operator"


def _bare_action_session(*, service_timeout_s: float = 0.001) -> object:
    session = tcp_console.RosTcpSession.__new__(tcp_console.RosTcpSession)
    session.args = SimpleNamespace(
        enable_motion=True,
        service_timeout_s=service_timeout_s,
    )
    session._lock = threading.Lock()
    session._command_lock = threading.Lock()
    session._active_goal = None
    session._active_result_future = None
    session._motion_state_unknown = False
    session.types = {
        "GoalStatus": SimpleNamespace(
            STATUS_UNKNOWN=0,
            STATUS_ACCEPTED=1,
            STATUS_EXECUTING=2,
            STATUS_CANCELING=3,
            STATUS_SUCCEEDED=4,
            STATUS_CANCELED=5,
            STATUS_ABORTED=6,
        )
    }
    session.execute_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
        send_goal_async=lambda _goal: (_ for _ in ()).throw(
            AssertionError("unexpected ExecuteTrajectory submission")
        ),
    )
    return session


def test_session_atomically_rejects_a_second_manual_command_without_queueing() -> None:
    session = _bare_action_session()
    first_entered = threading.Event()
    release_first = threading.Event()
    first_errors: list[BaseException] = []

    def reserved(
        _target_builder: object,
        *,
        cancel_event: threading.Event | None,
    ) -> MotionDelta:
        assert cancel_event is None
        first_entered.set()
        assert release_first.wait(timeout=1.0)
        return MotionDelta(translation_m=0.0, rotation_deg=0.0)

    session._execute_tip_target_reserved = reserved

    def run_first() -> None:
        try:
            session._execute_tip_target(lambda _current, _frame: np.eye(4), cancel_event=None)
        except BaseException as error:  # pragma: no cover - asserted below
            first_errors.append(error)

    thread = threading.Thread(target=run_first)
    thread.start()
    assert first_entered.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="already in progress|never queued"):
        session._execute_tip_target(
            lambda _current, _frame: np.eye(4),
            cancel_event=None,
        )

    release_first.set()
    thread.join(timeout=1.0)
    assert thread.is_alive() is False
    assert first_errors == []


class _FakeGoalHandle:
    def __init__(self, *, cancellation_accepted: bool = True) -> None:
        self.accepted = True
        self.result_future: Future[object] = Future()
        self.cancel_call_count = 0
        self.cancellation_accepted = cancellation_accepted

    def get_result_async(self) -> Future[object]:
        return self.result_future

    def cancel_goal_async(self) -> Future[object]:
        self.cancel_call_count += 1
        future: Future[object] = Future()
        future.set_result(
            SimpleNamespace(
                goals_canceling=(object(),) if self.cancellation_accepted else (),
            )
        )
        return future


def _wrapped_action_result(status: int, *, moveit_code: int = 1) -> object:
    return SimpleNamespace(
        status=status,
        result=SimpleNamespace(error_code=SimpleNamespace(val=moveit_code)),
    )


def test_escape_during_preflight_never_submits_a_move_group_goal() -> None:
    session = _bare_action_session()
    preflight_entered = threading.Event()
    release_preflight = threading.Event()
    submitted = threading.Event()

    session._runtime_absent = lambda: None

    def blocking_controller_preflight() -> None:
        preflight_entered.set()
        assert release_preflight.wait(timeout=1.0)

    session.verify_required_controller = blocking_controller_preflight
    session.move_client = SimpleNamespace(
        send_goal_async=lambda _goal: submitted.set(),
    )
    worker = OneMotionWorker()

    assert worker.start(
        lambda cancel_event: session._execute_tip_target(
            lambda _current, _base_to_plane1: np.eye(4),
            cancel_event=cancel_event,
        ),
        label="preflight",
    )
    assert preflight_entered.wait(timeout=1.0)
    assert worker.request_cancel(session.cancel_active)
    release_preflight.set()
    worker.join(timeout_s=1.0)

    snapshot = worker.snapshot()
    assert snapshot.running is False
    assert "cancelled before controller submission" in snapshot.error
    assert submitted.is_set() is False
    assert session.unresolved_motion() is False


def test_late_accepted_goal_after_response_timeout_keeps_lock_until_terminal() -> None:
    session = _bare_action_session()
    goal_response: Future[object] = Future()

    with pytest.raises(RuntimeError, match="goal response timed out.*motion is locked"):
        session._await_goal_submission(goal_response)

    assert session.unresolved_motion() is True
    with pytest.raises(RuntimeError, match="active or unresolved.*never queued"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: np.eye(4),
            cancel_event=None,
        )

    late_handle = _FakeGoalHandle()
    goal_response.set_result(late_handle)

    assert late_handle.cancel_call_count == 1
    assert session._active_goal is late_handle
    assert session.unresolved_motion() is True
    with pytest.raises(RuntimeError, match="active or unresolved.*never queued"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: np.eye(4),
            cancel_event=None,
        )

    late_handle.result_future.set_result(_wrapped_action_result(4))

    assert session.unresolved_motion() is False
    assert session._active_goal is None


def test_goal_response_wait_interrupted_by_baseexception_still_adopts_late_goal(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    goal_response: Future[object] = Future()

    def interrupt_wait(_seconds: float) -> None:
        raise KeyboardInterrupt

    monkeypatch.setattr(tcp_console.time, "sleep", interrupt_wait)

    with pytest.raises(KeyboardInterrupt):
        session._await_goal_submission(goal_response)

    assert session.unresolved_motion() is True
    late_handle = _FakeGoalHandle()
    goal_response.set_result(late_handle)

    # A request can be accepted by the server after the caller has been
    # interrupted.  The late callback must own and cancel that handle rather
    # than leave an untracked robot action behind.
    assert late_handle.cancel_call_count == 1
    assert session._active_goal is late_handle
    assert session.unresolved_motion() is True

    late_handle.result_future.set_result(_wrapped_action_result(4))

    assert session._active_goal is None
    assert session.unresolved_motion() is False


def test_late_goal_response_exception_after_timeout_remains_fail_closed() -> None:
    session = _bare_action_session()
    goal_response: Future[object] = Future()

    with pytest.raises(RuntimeError, match="goal response timed out.*motion is locked"):
        session._await_goal_submission(goal_response)

    goal_response.set_exception(RuntimeError("late action transport failure"))

    assert session.unresolved_motion() is True
    with pytest.raises(RuntimeError, match="active or unresolved.*never queued"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: np.eye(4),
            cancel_event=None,
        )


def test_late_empty_goal_response_after_timeout_remains_fail_closed() -> None:
    session = _bare_action_session()
    goal_response: Future[object] = Future()

    with pytest.raises(RuntimeError, match="goal response timed out.*motion is locked"):
        session._await_goal_submission(goal_response)

    goal_response.set_result(None)

    assert session.unresolved_motion() is True


def test_immediate_goal_submission_exception_is_also_fail_closed() -> None:
    session = _bare_action_session()
    goal_response: Future[object] = Future()
    goal_response.set_exception(RuntimeError("action transport failure"))

    with pytest.raises(RuntimeError, match="unknown server state.*remains locked"):
        session._await_goal_submission(goal_response)

    assert session.unresolved_motion() is True


def test_accepted_goal_result_exception_does_not_release_motion_lock() -> None:
    session = _bare_action_session()
    handle = _FakeGoalHandle()
    session._track_goal(handle)

    handle.result_future.set_exception(RuntimeError("GetResult transport failure"))

    assert session._active_goal is handle
    assert session.unresolved_motion() is True


def test_accepted_handle_get_result_failure_is_tracked_fail_closed() -> None:
    session = _bare_action_session()

    class BrokenResultHandle:
        accepted = True

        def get_result_async(self) -> object:
            raise RuntimeError("failed to create GetResult request")

    handle = BrokenResultHandle()
    try:
        session._track_goal(handle)
    except RuntimeError:
        pass

    assert session._active_goal is handle
    assert session.unresolved_motion() is True


@pytest.mark.parametrize("status", [0, 1, 2, 3])
def test_nonterminal_goal_result_status_does_not_release_motion_lock(
    status: int,
) -> None:
    session = _bare_action_session()
    handle = _FakeGoalHandle()
    session._track_goal(handle)

    handle.result_future.set_result(_wrapped_action_result(status))

    assert session._active_goal is handle
    assert session.unresolved_motion() is True


@pytest.mark.parametrize("status", [4, 5, 6])
def test_terminal_goal_result_status_releases_motion_lock(status: int) -> None:
    session = _bare_action_session()
    handle = _FakeGoalHandle()
    session._track_goal(handle)

    handle.result_future.set_result(_wrapped_action_result(status))

    assert session._active_goal is None
    assert session.unresolved_motion() is False


def test_rejected_goal_cancellation_keeps_motion_locked_until_terminal() -> None:
    session = _bare_action_session()
    handle = _FakeGoalHandle(cancellation_accepted=False)
    session._track_goal(handle)

    assert session.cancel_active() is False
    assert handle.cancel_call_count == 1
    assert session._active_goal is handle
    assert session.unresolved_motion() is True

    handle.result_future.set_result(_wrapped_action_result(5))

    assert session.unresolved_motion() is False
    assert session._active_goal is None


def test_late_cancel_transport_failure_cannot_relock_a_terminal_goal() -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    cancel_requested = threading.Event()
    cancel_future: Future[object] = Future()

    class DelayedCancelHandle(_FakeGoalHandle):
        def cancel_goal_async(self) -> Future[object]:
            self.cancel_call_count += 1
            cancel_requested.set()
            return cancel_future

    handle = DelayedCancelHandle()
    session._track_goal(handle)
    cancel_results: list[bool] = []

    thread = threading.Thread(
        target=lambda: cancel_results.append(session.cancel_active()),
        daemon=True,
    )
    thread.start()
    assert cancel_requested.wait(timeout=1.0)

    # GetResult wins the race and conclusively clears the tracked action.
    handle.result_future.set_result(_wrapped_action_result(4))
    assert session._active_goal is None
    assert session.unresolved_motion() is False

    # A delayed CancelGoal transport error is now stale evidence.  It must not
    # change the already-terminal action back to an unresolved global lock.
    cancel_future.set_exception(RuntimeError("late CancelGoal transport failure"))
    thread.join(timeout=1.0)

    assert thread.is_alive() is False
    assert cancel_results == [False]
    assert session._active_goal is None
    assert session.unresolved_motion() is False


def test_operator_cancel_is_sticky_even_if_get_result_says_succeeded() -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.motion_timeout_s = 1.0
    session.args.cancel_confirmation_timeout_s = 0.2
    handle = _FakeGoalHandle()
    result_future = session._track_goal(handle)
    cancel_event = threading.Event()
    cancel_event.set()

    timer = threading.Timer(
        0.02,
        lambda: handle.result_future.set_result(_wrapped_action_result(4)),
    )
    timer.start()
    with pytest.raises(RuntimeError, match="operator cancellation.*failed closed"):
        session._await_terminal_result(result_future, cancel_event)
    timer.join(timeout=1.0)

    assert handle.cancel_call_count >= 1
    assert session.unresolved_motion() is False


def test_motion_timeout_is_sticky_even_if_late_result_says_succeeded() -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.motion_timeout_s = 0.01
    session.args.cancel_confirmation_timeout_s = 0.2
    handle = _FakeGoalHandle()
    result_future = session._track_goal(handle)

    timer = threading.Timer(
        0.03,
        lambda: handle.result_future.set_result(_wrapped_action_result(4)),
    )
    timer.start()
    with pytest.raises(RuntimeError, match="motion timeout.*failed closed"):
        session._await_terminal_result(result_future, None)
    timer.join(timeout=1.0)

    assert handle.cancel_call_count >= 1
    assert session.unresolved_motion() is False


def test_baseexception_after_goal_adoption_still_cancels_owned_goal() -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.motion_timeout_s = 1.0
    session.args.cancel_confirmation_timeout_s = 0.1
    handle = _FakeGoalHandle()
    response: Future[object] = Future()
    response.set_result(handle)
    client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
        send_goal_async=lambda _goal: response,
    )
    original_await = session._await_goal_submission

    def adopt_then_interrupt(future: object, **kwargs: object) -> object:
        original_await(future, **kwargs)
        raise KeyboardInterrupt

    session._await_goal_submission = adopt_then_interrupt

    with pytest.raises(KeyboardInterrupt):
        session._run_tracked_action(
            client,
            object(),
            action_label="MoveGroup planning",
            cancel_event=None,
        )

    assert session._active_goal is handle
    assert handle.cancel_call_count >= 1
    assert session.unresolved_motion() is True
    handle.result_future.set_result(_wrapped_action_result(5))
    assert session.unresolved_motion() is False


@pytest.mark.parametrize("invalid_result", ["exception", "none", "nonterminal"])
def test_done_but_invalid_get_result_is_cancelled_and_kept_owned(
    invalid_result: str,
) -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.motion_timeout_s = 1.0
    session.args.cancel_confirmation_timeout_s = 0.05
    handle = _FakeGoalHandle()
    if invalid_result == "exception":
        handle.result_future.set_exception(RuntimeError("GetResult transport failure"))
    elif invalid_result == "none":
        handle.result_future.set_result(None)
    else:
        handle.result_future.set_result(_wrapped_action_result(2))
    response: Future[object] = Future()
    response.set_result(handle)
    client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
        send_goal_async=lambda _goal: response,
    )

    with pytest.raises(RuntimeError, match="GetResult"):
        session._run_tracked_action(
            client,
            object(),
            action_label="MoveGroup planning",
            cancel_event=None,
        )

    deadline = tcp_console.time.monotonic() + 0.5
    while handle.cancel_call_count == 0 and tcp_console.time.monotonic() < deadline:
        threading.Event().wait(0.01)
    assert handle.cancel_call_count >= 1
    assert session._active_goal is handle
    assert session.unresolved_motion() is True
    # End the deliberately invalid fake generation so its daemon supervisor
    # cannot leak work into later tests.
    session._clear_tracked_goal(handle)


def test_cancel_supervisor_survives_confirmation_timeout_until_terminal() -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.motion_timeout_s = 1.0
    session.args.cancel_confirmation_timeout_s = 0.03
    handle = _FakeGoalHandle()
    result_future = session._track_goal(handle)
    cancel_event = threading.Event()
    cancel_event.set()

    with pytest.raises(RuntimeError, match="not confirmed.*remains locked"):
        session._await_terminal_result(result_future, cancel_event)

    first_count = handle.cancel_call_count
    threading.Event().wait(0.22)
    assert handle.cancel_call_count > first_count
    assert session._active_goal is handle
    assert session.unresolved_motion() is True

    handle.result_future.set_result(_wrapped_action_result(5))
    deadline = tcp_console.time.monotonic() + 0.5
    while session.unresolved_motion() and tcp_console.time.monotonic() < deadline:
        threading.Event().wait(0.01)
    assert session.unresolved_motion() is False

    # The old supervisor is generation-bound and cannot cancel a later goal.
    next_handle = _FakeGoalHandle()
    session._track_goal(next_handle)
    threading.Event().wait(0.15)
    assert next_handle.cancel_call_count == 0
    next_handle.result_future.set_result(_wrapped_action_result(4))


def test_close_keeps_ros_resources_alive_until_unresolved_motion_clears() -> None:
    session = _bare_action_session()
    session.args.cancel_confirmation_timeout_s = 0.05
    cancel_attempted = threading.Event()
    shutdown_called = threading.Event()
    spin_joined = threading.Event()
    node_destroyed = threading.Event()
    close_finished = threading.Event()

    session._motion_state_unknown = True
    session.cancel_active = lambda: cancel_attempted.set() or False
    session.executor = SimpleNamespace(
        shutdown=lambda **_kwargs: shutdown_called.set()
    )
    session._spin_thread = SimpleNamespace(
        join=lambda **_kwargs: spin_joined.set()
    )
    session.node = SimpleNamespace(destroy_node=lambda: node_destroyed.set())

    def close_session() -> None:
        session.close()
        close_finished.set()

    close_thread = threading.Thread(target=close_session, daemon=True)
    close_thread.start()
    assert cancel_attempted.wait(timeout=1.0)

    # The old implementation tore ROS down after one confirmation timeout.
    # Waiting beyond that interval proves an unresolved action still keeps the
    # executor alive so its late response/result callbacks can run.
    assert close_finished.wait(timeout=0.12) is False
    assert shutdown_called.is_set() is False
    assert spin_joined.is_set() is False
    assert node_destroyed.is_set() is False

    with session._lock:
        session._motion_state_unknown = False
    close_thread.join(timeout=1.0)

    assert close_finished.is_set()
    assert shutdown_called.is_set()
    assert spin_joined.is_set()
    assert node_destroyed.is_set()


def test_close_ignores_keyboard_interrupt_from_cancel_and_keeps_ros_alive() -> None:
    session = _bare_action_session()
    session.args.cancel_confirmation_timeout_s = 0.05
    interrupt_raised = threading.Event()
    cancel_retried = threading.Event()
    shutdown_called = threading.Event()
    node_destroyed = threading.Event()
    close_finished = threading.Event()
    cancel_call_count = 0

    session._motion_state_unknown = True

    def interrupted_cancel() -> bool:
        nonlocal cancel_call_count
        cancel_call_count += 1
        if cancel_call_count == 1:
            interrupt_raised.set()
            raise KeyboardInterrupt
        cancel_retried.set()
        return False

    session.cancel_active = interrupted_cancel
    session.executor = SimpleNamespace(
        shutdown=lambda **_kwargs: shutdown_called.set()
    )
    session._spin_thread = SimpleNamespace(join=lambda **_kwargs: None)
    session.node = SimpleNamespace(destroy_node=lambda: node_destroyed.set())

    def close_session() -> None:
        session.close()
        close_finished.set()

    close_thread = threading.Thread(target=close_session, daemon=True)
    close_thread.start()
    assert interrupt_raised.wait(timeout=1.0)
    assert cancel_retried.wait(timeout=1.0)

    assert close_finished.wait(timeout=0.08) is False
    assert shutdown_called.is_set() is False
    assert node_destroyed.is_set() is False

    with session._lock:
        session._motion_state_unknown = False
    close_thread.join(timeout=1.0)

    assert close_finished.is_set()
    assert shutdown_called.is_set()
    assert node_destroyed.is_set()


def test_motion_uses_settled_snapshot_and_live_tcp_after_all_slow_preflights(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.state_timeout_s = 0.1
    session.args.max_translation_m = 0.05
    session.args.max_rotation_deg = 10.0
    session.args.motion_timeout_s = 0.2
    session.args.cancel_confirmation_timeout_s = 0.1
    session.args.post_position_tolerance_m = 0.001
    session.args.post_orientation_tolerance_deg = 0.1
    session.base_frame = "base"
    session.tool_builder = object()
    session.types["MoveItErrorCodes"] = SimpleNamespace(SUCCESS=1)

    stale_positions = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    final_positions = (2.0, 2.0, 2.0, 2.0, 2.0, 2.0)
    final_preflights_complete = False
    runtime_checks = 0
    build_start_positions: list[tuple[float, ...]] = []
    current_tip_preflight_states: list[bool] = []

    def runtime_absent() -> None:
        nonlocal runtime_checks, final_preflights_complete
        runtime_checks += 1
        # The second runtime/controller discovery pass is the last potentially
        # slow preflight before establishing the authoritative start snapshot.
        if runtime_checks == 2:
            final_preflights_complete = True

    def wait_settled(*, timeout_s: float, after_s: float = 0.0) -> object:
        assert timeout_s == pytest.approx(0.1)
        if after_s > 0.0:
            return SimpleNamespace(positions_rad=final_positions)
        positions = final_positions if final_preflights_complete else stale_positions
        return SimpleNamespace(positions_rad=positions)

    def current_tip_matrix(**_kwargs: object) -> np.ndarray:
        current_tip_preflight_states.append(final_preflights_complete)
        return np.eye(4)

    def build_goal(_target: object, positions: tuple[float, ...]) -> object:
        build_start_positions.append(tuple(positions))
        return object()

    session.move_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
    )
    session._runtime_absent = runtime_absent
    session.verify_required_controller = lambda: None
    session.live_authoritative_transforms = lambda: (np.eye(4), np.eye(4))
    session.verify_scene = lambda: None
    session.wait_settled = wait_settled
    session._settled_snapshot = lambda: SimpleNamespace(
        positions_rad=final_positions
    )
    session.current_tip_matrix = current_tip_matrix
    solved_seeds: list[tuple[float, ...]] = []

    def solve_ik(_target: object, positions: tuple[float, ...]) -> tuple[float, ...]:
        solved_seeds.append(tuple(positions))
        return final_positions

    session._solve_collision_checked_ik = solve_ik
    session._build_goal = build_goal
    trajectory = object()
    session._validate_planned_trajectory = lambda _trajectory, _start, _target: None
    session._build_execute_goal = lambda exact: SimpleNamespace(trajectory=exact)

    def run_action(
        client: object,
        _goal: object,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
        pre_submit: object | None = None,
    ) -> object:
        assert cancel_event is None
        if client is session.move_client:
            assert pre_submit is None
            assert action_label == "MoveGroup planning"
            return SimpleNamespace(
                status=4,
                result=SimpleNamespace(
                    error_code=SimpleNamespace(val=1),
                    planned_trajectory=trajectory,
                ),
            )
        assert client is session.execute_client
        assert action_label == "ExecuteTrajectory"
        assert callable(pre_submit)
        pre_submit()
        return _wrapped_action_result(4)

    session._run_tracked_action = run_action
    monkeypatch.setattr(
        tcp_console,
        "rod_tip_target_to_tool0",
        lambda _builder, _target, *, frame_id: SimpleNamespace(frame_id=frame_id),
    )

    result = session._execute_tip_target(
        lambda current, _base_to_plane1: current.copy(),
        cancel_event=None,
    )

    assert result == MotionDelta(translation_m=0.0, rotation_deg=0.0)
    assert solved_seeds == [final_positions]
    assert build_start_positions == [final_positions]
    assert current_tip_preflight_states
    assert all(current_tip_preflight_states)


def test_motion_rejects_final_joint_drift_before_building_or_submitting_goal(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.state_timeout_s = 0.1
    session.args.max_translation_m = 0.05
    session.args.max_rotation_deg = 10.0
    session.base_frame = "base"
    session.tool_builder = object()
    built = threading.Event()
    submitted = threading.Event()
    stable_positions = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    drifted_positions = (0.000501, 0.0, 0.0, 0.0, 0.0, 0.0)

    session._runtime_absent = lambda: None
    session.verify_required_controller = lambda: None
    session.live_authoritative_transforms = lambda: (np.eye(4), np.eye(4))
    session.verify_scene = lambda: None
    session.wait_settled = lambda **_kwargs: SimpleNamespace(
        positions_rad=stable_positions
    )
    # This is the authoritative, zero-wait snapshot after the final runtime
    # discovery check. It exceeds the 5e-4 rad drift ceiling by 1e-6 rad.
    session._settled_snapshot = lambda: SimpleNamespace(
        positions_rad=drifted_positions
    )
    session.current_tip_matrix = lambda **_kwargs: np.eye(4)
    session._build_goal = lambda _target, _positions: built.set()
    session.move_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
        send_goal_async=lambda _goal: submitted.set(),
    )
    monkeypatch.setattr(
        tcp_console,
        "rod_tip_target_to_tool0",
        lambda _builder, _target, *, frame_id: SimpleNamespace(frame_id=frame_id),
    )

    with pytest.raises(
        RuntimeError,
        match="changed by 0.000501 rad immediately before MoveGroup submission",
    ):
        session._execute_tip_target(
            lambda current, _base_to_plane1: current.copy(),
            cancel_event=None,
        )

    assert built.is_set() is False
    assert submitted.is_set() is False
    assert session.unresolved_motion() is False


def test_ik_failure_stops_before_goal_build_or_move_group_submission(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.state_timeout_s = 0.1
    session.args.max_translation_m = 0.05
    session.args.max_rotation_deg = 10.0
    session.base_frame = "base"
    session.tool_builder = object()
    positions = (-1.7, -1.2, -1.4, -1.6, 1.4, 1.5)
    built = threading.Event()
    submitted = threading.Event()

    session._runtime_absent = lambda: None
    session.verify_required_controller = lambda: None
    session.live_authoritative_transforms = lambda: (np.eye(4), np.eye(4))
    session.verify_scene = lambda: None
    session.wait_settled = lambda **_kwargs: SimpleNamespace(
        positions_rad=positions
    )
    session._settled_snapshot = lambda: SimpleNamespace(positions_rad=positions)
    session.current_tip_matrix = lambda **_kwargs: np.eye(4)
    session._solve_collision_checked_ik = lambda _target, _seed: (_ for _ in ()).throw(
        RuntimeError("collision-aware IK failed")
    )
    session._build_goal = lambda _target, _start: built.set()
    session.move_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
        send_goal_async=lambda _goal: submitted.set(),
    )
    monkeypatch.setattr(
        tcp_console,
        "rod_tip_target_to_tool0",
        lambda _builder, _target, *, frame_id: SimpleNamespace(frame_id=frame_id),
    )

    with pytest.raises(RuntimeError, match="collision-aware IK failed"):
        session._execute_tip_target(
            lambda current, _base_to_plane1: current.copy(),
            cancel_event=None,
        )

    assert built.is_set() is False
    assert submitted.is_set() is False
    assert session.unresolved_motion() is False


def test_successful_joint_goal_is_still_verified_against_original_rod_tip_target(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.state_timeout_s = 0.1
    session.args.max_translation_m = 0.05
    session.args.max_rotation_deg = 10.0
    session.args.motion_timeout_s = 0.2
    session.args.cancel_confirmation_timeout_s = 0.1
    session.args.post_position_tolerance_m = 0.0004
    session.args.post_orientation_tolerance_deg = 0.1
    session.base_frame = "base"
    session.tool_builder = object()
    session.types["MoveItErrorCodes"] = SimpleNamespace(SUCCESS=1)
    positions = (-1.7, -1.2, -1.4, -1.6, 1.4, 1.5)
    solved = tuple(value + 0.001 for value in positions)
    current_tip = np.eye(4)
    target_tip = np.eye(4)
    target_tip[0, 3] = 0.002
    actual_tip = target_tip.copy()
    actual_tip[1, 3] = 0.0001
    tip_reads = iter((current_tip, actual_tip))
    tool_target = SimpleNamespace(
        # Intentionally unrelated to the rod-tip target. If post-checking ever
        # drifts to this tool0 pose, this test must fail.
        position_m=(0.5, 0.5, 0.5),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id="base",
    )
    ik_inputs: list[tuple[object, tuple[float, ...]]] = []
    goal_inputs: list[tuple[tuple[float, ...], tuple[float, ...]]] = []

    session._runtime_absent = lambda: None
    session.verify_required_controller = lambda: None
    session.live_authoritative_transforms = lambda: (np.eye(4), np.eye(4))
    session.verify_scene = lambda: None
    session.wait_settled = lambda **_kwargs: SimpleNamespace(
        positions_rad=positions
    )
    session._settled_snapshot = lambda: SimpleNamespace(positions_rad=positions)
    session.current_tip_matrix = lambda **_kwargs: next(tip_reads)

    def solve_ik(target: object, seed: tuple[float, ...]) -> tuple[float, ...]:
        ik_inputs.append((target, tuple(seed)))
        return solved

    def build_goal(target: tuple[float, ...], start: tuple[float, ...]) -> object:
        goal_inputs.append((tuple(target), tuple(start)))
        return object()

    session._solve_collision_checked_ik = solve_ik
    session._build_goal = build_goal
    session.move_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
    )
    trajectory = object()
    session._validate_planned_trajectory = lambda _trajectory, _start, _target: None
    session._build_execute_goal = lambda exact: SimpleNamespace(trajectory=exact)

    def run_action(
        client: object,
        _goal: object,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
        pre_submit: object | None = None,
    ) -> object:
        assert cancel_event is None
        if client is session.move_client:
            assert pre_submit is None
            assert action_label == "MoveGroup planning"
            return SimpleNamespace(
                status=4,
                result=SimpleNamespace(
                    error_code=SimpleNamespace(val=1),
                    planned_trajectory=trajectory,
                ),
            )
        assert client is session.execute_client
        assert action_label == "ExecuteTrajectory"
        assert callable(pre_submit)
        pre_submit()
        return _wrapped_action_result(4)

    session._run_tracked_action = run_action
    monkeypatch.setattr(
        tcp_console,
        "rod_tip_target_to_tool0",
        lambda _builder, _target, *, frame_id: tool_target,
    )

    measured = session._execute_tip_target(
        lambda _current, _base_to_plane1: target_tip.copy(),
        cancel_event=None,
    )

    assert ik_inputs == [(tool_target, positions)]
    assert goal_inputs == [(solved, positions)]
    assert measured.translation_m == pytest.approx(math.hypot(0.002, 0.0001))


class _ManualJointState:
    def __init__(self) -> None:
        self.name: list[str] = []
        self.position: list[float] = []


class _ManualRobotState:
    def __init__(self) -> None:
        self.joint_state = _ManualJointState()
        self.attached_collision_objects: list[object] = []
        self.is_diff = False


class _ManualPose:
    def __init__(self) -> None:
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


class _ManualPoseStamped:
    def __init__(self) -> None:
        self.header = SimpleNamespace(frame_id="")
        self.pose = _ManualPose()


class _ManualJointConstraint:
    def __init__(self) -> None:
        self.joint_name = ""
        self.position = 0.0
        self.tolerance_below = 0.0
        self.tolerance_above = 0.0
        self.weight = 0.0


class _ManualConstraints:
    def __init__(self) -> None:
        self.name = ""
        self.joint_constraints: list[_ManualJointConstraint] = []
        self.position_constraints: list[object] = []
        self.orientation_constraints: list[object] = []


class _ManualGetPositionIK:
    class Request:
        def __init__(self) -> None:
            self.ik_request = SimpleNamespace(
                group_name="",
                robot_state=_ManualRobotState(),
                constraints=_ManualConstraints(),
                avoid_collisions=False,
                ik_link_name="",
                pose_stamped=_ManualPoseStamped(),
                timeout=None,
                attempts=0,
            )


class _ManualDuration:
    def __init__(self, *, seconds: float) -> None:
        self.seconds = seconds

    def to_msg(self) -> object:
        return SimpleNamespace(seconds=self.seconds)


class _ManualMoveGroup:
    class Goal:
        def __init__(self) -> None:
            self.request = SimpleNamespace(start_state=_ManualRobotState())
            self.planning_options = SimpleNamespace()


class _ManualExecuteTrajectory:
    class Goal:
        def __init__(self) -> None:
            self.trajectory: object | None = None
            self.controller_names: list[str] = []


def _manual_motion_types() -> dict[str, object]:
    return {
        "Pose": _ManualPose,
        "PoseStamped": _ManualPoseStamped,
        "RobotState": _ManualRobotState,
        "GetPositionIK": _ManualGetPositionIK,
        "Duration": _ManualDuration,
        "Constraints": _ManualConstraints,
        "JointConstraint": _ManualJointConstraint,
        "MoveGroup": _ManualMoveGroup,
        "ExecuteTrajectory": _ManualExecuteTrajectory,
        "MoveItErrorCodes": SimpleNamespace(SUCCESS=1),
    }


def _duration_message(seconds: float) -> SimpleNamespace:
    whole = math.floor(seconds)
    return SimpleNamespace(
        sec=int(whole),
        nanosec=int(round((seconds - whole) * 1.0e9)),
    )


def _trajectory_point(
    positions: tuple[float, ...],
    time_s: float,
    *,
    velocities: tuple[float, ...] | None = None,
    accelerations: tuple[float, ...] | None = None,
    effort: tuple[float, ...] | None = None,
) -> SimpleNamespace:
    return SimpleNamespace(
        positions=list(positions),
        velocities=[] if velocities is None else list(velocities),
        accelerations=[] if accelerations is None else list(accelerations),
        effort=[] if effort is None else list(effort),
        time_from_start=_duration_message(time_s),
    )


def _planned_trajectory(
    names: tuple[str, ...],
    points: tuple[SimpleNamespace, ...],
) -> SimpleNamespace:
    return SimpleNamespace(
        joint_trajectory=SimpleNamespace(
            joint_names=list(names),
            points=list(points),
        ),
        multi_dof_joint_trajectory=SimpleNamespace(
            joint_names=[],
            points=[],
        ),
    )


def _manual_ik_session(
    response: object,
    *,
    joint_names: tuple[str, ...] = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    ),
) -> tuple[object, list[object]]:
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.move_group = "ur_manipulator"
    session.args.max_joint_excursion_deg = 20.0
    session.args.allowed_planning_time_s = 1.25
    session.base_frame = "base"
    session.tool_builder = SimpleNamespace(parent_link="tool0")
    session.environment = SimpleNamespace(
        joint_names=joint_names,
        joint_position_domains=tuple(
            JointPositionDomain(
                name=name,
                lower=-math.tau,
                upper=math.tau,
                period=math.tau,
            )
            for name in joint_names
        ),
        planner={
            "ik": {
                "canonical_joint_ranges_rad": {
                    "shoulder_pan_joint": (-math.pi, 0.0),
                }
            }
        },
    )
    session.scene = object()
    session.types.update(_manual_motion_types())
    requests: list[object] = []

    def call_async(request: object) -> Future[object]:
        requests.append(request)
        future: Future[object] = Future()
        future.set_result(response)
        return future

    session.ik_client = SimpleNamespace(
        wait_for_service=lambda **_kwargs: True,
        call_async=call_async,
    )
    return session, requests


def _ik_response(names: tuple[str, ...], positions: tuple[float, ...], code: int = 1) -> object:
    return SimpleNamespace(
        error_code=SimpleNamespace(val=code),
        solution=SimpleNamespace(
            joint_state=SimpleNamespace(name=list(names), position=list(positions))
        ),
    )


def test_collision_checked_ik_uses_actual_seed_attached_rod_and_exact_tool_pose(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    )
    seed = (-1.7, -1.2, -1.4)
    # Deliberately shuffle the response and add one full winding to every
    # value. The accepted result must be reordered and canonicalized back to
    # the actual local branch around the seed.
    response = _ik_response(
        (names[2], names[0], names[1]),
        (seed[2] + math.tau, seed[0] + math.tau, seed[1] + math.tau),
    )
    session, requests = _manual_ik_session(response, joint_names=names)
    attached_rod = "authoritative_attached_rod"
    monkeypatch.setattr(
        tcp_console,
        "_build_apply_request",
        lambda _scene, _types: SimpleNamespace(
            scene=SimpleNamespace(
                robot_state=SimpleNamespace(
                    attached_collision_objects=[attached_rod],
                )
            )
        ),
    )
    target = SimpleNamespace(
        position_m=(0.31, -0.27, 0.42),
        quaternion_xyzw=(0.1, -0.2, 0.3, 0.9273618495),
    )

    solved = session._solve_collision_checked_ik(target, seed)

    assert solved == pytest.approx(seed)
    assert len(requests) == 1
    request = requests[0].ik_request
    assert request.group_name == "ur_manipulator"
    assert request.avoid_collisions is True
    assert request.ik_link_name == "tool0"
    assert request.robot_state.is_diff is True
    assert tuple(request.robot_state.joint_state.name) == names
    assert tuple(request.robot_state.joint_state.position) == pytest.approx(seed)
    assert request.robot_state.attached_collision_objects == [attached_rod]
    assert request.pose_stamped.header.frame_id == "base"
    assert (
        request.pose_stamped.pose.position.x,
        request.pose_stamped.pose.position.y,
        request.pose_stamped.pose.position.z,
    ) == pytest.approx(target.position_m)
    assert (
        request.pose_stamped.pose.orientation.x,
        request.pose_stamped.pose.orientation.y,
        request.pose_stamped.pose.orientation.z,
        request.pose_stamped.pose.orientation.w,
    ) == pytest.approx(target.quaternion_xyzw)
    assert request.constraints.name == "manual_local_joint_branch"
    assert [item.joint_name for item in request.constraints.joint_constraints] == list(
        names
    )


@pytest.mark.parametrize(
    ("returned_names", "returned_positions", "message"),
    [
        (
            ("shoulder_pan_joint", "shoulder_lift_joint"),
            (-1.7, -1.2),
            "joint (names|set)",
        ),
        (
            (
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "unexpected_joint",
            ),
            (-1.7, -1.2, -1.4, 0.0),
            "joint (names|set)",
        ),
        (
            ("shoulder_pan_joint", "shoulder_pan_joint", "elbow_joint"),
            (-1.7, -1.2, -1.4),
            "joint names",
        ),
        (
            ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"),
            (-1.7, math.nan, -1.4),
            "finite",
        ),
    ],
)
def test_collision_checked_ik_rejects_nonexact_names_and_nonfinite_values(
    monkeypatch: pytest.MonkeyPatch,
    returned_names: tuple[str, ...],
    returned_positions: tuple[float, ...],
    message: str,
) -> None:
    session, _requests = _manual_ik_session(
        _ik_response(returned_names, returned_positions)
    )
    monkeypatch.setattr(
        tcp_console,
        "_build_apply_request",
        lambda _scene, _types: SimpleNamespace(
            scene=SimpleNamespace(
                robot_state=SimpleNamespace(attached_collision_objects=[])
            )
        ),
    )

    with pytest.raises(RuntimeError, match=message):
        session._solve_collision_checked_ik(
            SimpleNamespace(
                position_m=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
            (-1.7, -1.2, -1.4),
        )


def test_collision_checked_ik_rejects_solution_outside_twenty_degree_local_branch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    seed = (-1.7, -1.2, -1.4)
    solution = (seed[0], seed[1] + math.radians(20.0) + 1.0e-6, seed[2])
    session, _requests = _manual_ik_session(
        _ik_response(
            (
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
            ),
            solution,
        )
    )
    monkeypatch.setattr(
        tcp_console,
        "_build_apply_request",
        lambda _scene, _types: SimpleNamespace(
            scene=SimpleNamespace(
                robot_state=SimpleNamespace(attached_collision_objects=[])
            )
        ),
    )

    with pytest.raises(RuntimeError, match="20|local|excursion"):
        session._solve_collision_checked_ik(
            SimpleNamespace(
                position_m=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
            seed,
        )


def test_collision_checked_ik_does_not_wrap_a_nonperiodic_joint(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    )
    seed = (-1.7, -1.2, -1.4)
    response = _ik_response(names, (seed[0], seed[1] + math.tau, seed[2]))
    session, _requests = _manual_ik_session(response, joint_names=names)
    session.environment.joint_position_domains = (
        JointPositionDomain(names[0], -math.tau, math.tau, math.tau),
        JointPositionDomain(names[1], -math.tau, math.tau, None),
        JointPositionDomain(names[2], -math.tau, math.tau, math.tau),
    )
    monkeypatch.setattr(
        tcp_console,
        "_build_apply_request",
        lambda _scene, _types: SimpleNamespace(
            scene=SimpleNamespace(
                robot_state=SimpleNamespace(attached_collision_objects=[])
            )
        ),
    )

    with pytest.raises(RuntimeError, match="local|canonical|domain|bounds"):
        session._solve_collision_checked_ik(
            SimpleNamespace(
                position_m=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
            seed,
        )


@pytest.mark.parametrize(
    ("response", "message"),
    [
        (
            _ik_response(
                (
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                ),
                (-1.7, -1.2, -1.4),
                code=-31,
            ),
            "IK|code|-31",
        ),
        (
            _ik_response(
                (
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                ),
                (0.1, -1.2, -1.4),
            ),
            "canonical|branch",
        ),
    ],
)
def test_collision_checked_ik_rejects_moveit_failure_and_canonical_branch_escape(
    monkeypatch: pytest.MonkeyPatch,
    response: object,
    message: str,
) -> None:
    session, _requests = _manual_ik_session(response)
    monkeypatch.setattr(
        tcp_console,
        "_build_apply_request",
        lambda _scene, _types: SimpleNamespace(
            scene=SimpleNamespace(
                robot_state=SimpleNamespace(attached_collision_objects=[])
            )
        ),
    )

    with pytest.raises(RuntimeError, match=message):
        session._solve_collision_checked_ik(
            SimpleNamespace(
                position_m=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            ),
            (-0.1, -1.2, -1.4),
        )


def test_move_group_goal_uses_ik_joint_constraints_and_preserves_scene_and_acm(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session = _bare_action_session()
    session.args.max_joint_excursion_deg = 20.0
    session.args.move_group = "ur_manipulator"
    session.args.planning_attempts = 2
    session.args.allowed_planning_time_s = 1.0
    session.args.velocity_scale = 0.1
    session.args.acceleration_scale = 0.1
    session.args.pipeline_id = "ompl"
    session.args.planner_id = "RRTConnectkConfigDefault"
    session.base_frame = "base"
    session.tool_builder = SimpleNamespace(parent_link="tool0")
    names = ("shoulder_pan_joint", "shoulder_lift_joint")
    session.environment = SimpleNamespace(
        joint_names=names,
        planner={
            "ik": {
                "canonical_joint_ranges_rad": {
                    "shoulder_pan_joint": (-math.pi, 0.0)
                }
            }
        },
    )
    session.scene = object()
    session.types = _manual_motion_types()
    validated_acm = SimpleNamespace(
        entry_names=["sensor_board_keepout", "tool0"],
        entry_values=[
            SimpleNamespace(enabled=[False, False]),
            SimpleNamespace(enabled=[False, False]),
        ],
        default_entry_names=[],
        default_entry_values=[],
    )
    session._validated_acm = validated_acm
    attached_rod = "authoritative_attached_rod"
    scene_diff = SimpleNamespace(
        allowed_collision_matrix=None,
        robot_state=SimpleNamespace(attached_collision_objects=[attached_rod]),
        world=SimpleNamespace(collision_objects=[object()]),
    )
    monkeypatch.setattr(
        tcp_console,
        "_build_apply_request",
        lambda _scene, _types: SimpleNamespace(scene=scene_diff),
    )

    start = (-1.0, -1.2)
    target = (-0.95, -1.15)
    goal = session._build_goal(
        target,
        start,
    )

    constraints = goal.request.goal_constraints
    assert len(constraints) == 1
    assert constraints[0].position_constraints == []
    assert constraints[0].orientation_constraints == []
    assert [item.joint_name for item in constraints[0].joint_constraints] == list(names)
    assert [item.position for item in constraints[0].joint_constraints] == pytest.approx(
        target
    )
    assert tuple(goal.request.start_state.joint_state.name) == names
    assert tuple(goal.request.start_state.joint_state.position) == pytest.approx(start)
    assert goal.request.start_state.is_diff is True
    assert goal.planning_options.plan_only is True
    assert goal.planning_options.look_around is False
    assert goal.planning_options.replan is False
    assert [
        item.joint_name for item in goal.request.path_constraints.joint_constraints
    ] == list(names)
    assert goal.planning_options.planning_scene_diff is scene_diff
    assert scene_diff.robot_state.attached_collision_objects == [attached_rod]
    assert scene_diff.world.collision_objects
    bound_acm = goal.planning_options.planning_scene_diff.allowed_collision_matrix
    assert bound_acm is not None
    assert bound_acm == validated_acm
    assert bound_acm is not validated_acm
    assert bound_acm.entry_names


def test_execute_goal_preserves_validated_trajectory_and_pins_required_controller() -> None:
    session = _bare_action_session()
    session.args.required_controller = "scaled_joint_trajectory_controller"
    session.types = _manual_motion_types()
    trajectory = _planned_trajectory(
        ("shoulder_pan_joint",),
        (
            _trajectory_point((-1.0,), 0.0),
            _trajectory_point((-0.9,), 1.0),
        ),
    )

    goal = session._build_execute_goal(trajectory)

    assert goal.trajectory is trajectory
    assert goal.controller_names == ["scaled_joint_trajectory_controller"]


def test_planned_joint_trajectory_accepts_exact_start_target_and_local_path() -> None:
    names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    )
    session, _requests = _manual_ik_session(
        _ik_response(names, (-1.7, -1.2, -1.4)),
        joint_names=names,
    )
    start = (-1.7, -1.2, -1.4)
    target = (-1.65, -1.15, -1.35)
    trajectory = _planned_trajectory(
        names,
        (
            _trajectory_point(start, 0.0, velocities=(0.0, 0.0, 0.0)),
            _trajectory_point(
                (-1.68, -1.18, -1.38),
                0.5,
                velocities=(0.1, 0.1, 0.1),
                accelerations=(0.2, 0.2, 0.2),
            ),
            _trajectory_point(target, 1.0, velocities=(0.0, 0.0, 0.0)),
        ),
    )

    session._validate_planned_trajectory(trajectory, start, target)


@pytest.mark.parametrize(
    ("trajectory_builder", "expected_message"),
    [
        (
            lambda names, start, target: _planned_trajectory(
                names[:-1],
                (
                    _trajectory_point(start[:-1], 0.0),
                    _trajectory_point(target[:-1], 1.0),
                ),
            ),
            "joint|name",
        ),
        (
            lambda names, start, target: _planned_trajectory(
                (names[0], names[0], names[2]),
                (
                    _trajectory_point(start, 0.0),
                    _trajectory_point(target, 1.0),
                ),
            ),
            "joint|name|duplicate",
        ),
        (
            lambda names, start, target: _planned_trajectory(names, ()),
            "empty|point|trajectory",
        ),
        (
            lambda names, start, target: _planned_trajectory(
                names,
                (
                    _trajectory_point(start[:-1], 0.0),
                    _trajectory_point(target, 1.0),
                ),
            ),
            "position|joint|length",
        ),
        (
            lambda names, start, target: _planned_trajectory(
                names,
                (
                    _trajectory_point(start, 0.0),
                    _trajectory_point((target[0], math.nan, target[2]), 1.0),
                ),
            ),
            "finite|position",
        ),
        (
            lambda names, start, target: _planned_trajectory(
                names,
                (
                    _trajectory_point(
                        start,
                        0.0,
                        velocities=(0.0, math.inf, 0.0),
                    ),
                    _trajectory_point(target, 1.0),
                ),
            ),
            "finite|velocity|point",
        ),
        (
            lambda names, start, target: _planned_trajectory(
                names,
                (
                    _trajectory_point(start, 0.0),
                    _trajectory_point(target, 0.0),
                ),
            ),
            "time|increas|duration",
        ),
        (
            lambda names, start, target: _planned_trajectory(
                names,
                (
                    _trajectory_point(
                        (start[0] + 1.0e-3, start[1], start[2]),
                        0.0,
                    ),
                    _trajectory_point(target, 1.0),
                ),
            ),
            "first|start",
        ),
        (
            lambda names, start, target: _planned_trajectory(
                names,
                (
                    _trajectory_point(start, 0.0),
                    _trajectory_point(
                        (target[0], target[1] + 1.0e-3, target[2]),
                        1.0,
                    ),
                ),
            ),
            "last|target|IK",
        ),
    ],
    ids=(
        "wrong-joint-set",
        "duplicate-joint-name",
        "empty-points",
        "wrong-position-length",
        "nonfinite-position",
        "nonfinite-velocity",
        "nonincreasing-time",
        "first-not-actual-start",
        "last-not-ik-target",
    ),
)
def test_planned_joint_trajectory_rejects_malformed_or_changed_data(
    trajectory_builder: object,
    expected_message: str,
) -> None:
    names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    )
    start = (-1.7, -1.2, -1.4)
    target = (-1.65, -1.15, -1.35)
    session, _requests = _manual_ik_session(
        _ik_response(names, target),
        joint_names=names,
    )
    build = trajectory_builder
    trajectory = build(names, start, target)  # type: ignore[operator]

    with pytest.raises(RuntimeError, match=expected_message):
        session._validate_planned_trajectory(trajectory, start, target)


@pytest.mark.parametrize(
    "middle",
    [
        # Escapes shoulder_pan's environment canonical interval [-pi, 0].
        (0.01, -1.18, -1.38),
        # Escapes the explicit +/-20 degree branch around shoulder_lift.
        (-1.68, -1.2 + math.radians(20.0) + 1.0e-6, -1.38),
    ],
    ids=("canonical-range", "twenty-degree-local-branch"),
)
def test_planned_joint_trajectory_rejects_any_path_point_outside_sealed_branch(
    middle: tuple[float, ...],
) -> None:
    names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    )
    start = (-1.7, -1.2, -1.4)
    target = (-1.65, -1.15, -1.35)
    session, _requests = _manual_ik_session(
        _ik_response(names, target),
        joint_names=names,
    )
    trajectory = _planned_trajectory(
        names,
        (
            _trajectory_point(start, 0.0),
            _trajectory_point(middle, 0.5),
            _trajectory_point(target, 1.0),
        ),
    )

    with pytest.raises(RuntimeError, match="canonical|local|branch|bounds|path"):
        session._validate_planned_trajectory(trajectory, start, target)


def _split_execution_session(
    monkeypatch: pytest.MonkeyPatch,
    *,
    current_tip: np.ndarray | None = None,
    target_tip: np.ndarray | None = None,
    actual_tip: np.ndarray | None = None,
) -> tuple[object, tuple[str, ...], tuple[float, ...], tuple[float, ...]]:
    names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    )
    start = (-1.7, -1.2, -1.4)
    target = (-1.65, -1.15, -1.35)
    session = _bare_action_session(service_timeout_s=0.1)
    session.args.state_timeout_s = 0.1
    session.args.max_translation_m = 0.05
    session.args.max_rotation_deg = 10.0
    session.args.motion_timeout_s = 0.2
    session.args.cancel_confirmation_timeout_s = 0.1
    session.args.post_position_tolerance_m = 0.0004
    session.args.post_orientation_tolerance_deg = 0.1
    session.args.max_joint_excursion_deg = 20.0
    session.args.required_controller = "scaled_joint_trajectory_controller"
    session.base_frame = "base"
    session.tool_builder = object()
    session.environment = SimpleNamespace(
        joint_names=names,
        joint_position_domains=tuple(
            JointPositionDomain(
                name=name,
                lower=-math.tau,
                upper=math.tau,
                period=math.tau,
            )
            for name in names
        ),
        planner={
            "ik": {
                "canonical_joint_ranges_rad": {
                    "shoulder_pan_joint": (-math.pi, 0.0),
                }
            }
        },
    )
    session.types.update(_manual_motion_types())
    session.scene = object()
    session._runtime_absent = lambda: None
    session.verify_required_controller = lambda: None
    session.live_authoritative_transforms = lambda: (np.eye(4), np.eye(4))
    session.verify_scene = lambda: None
    session.wait_settled = lambda **_kwargs: SimpleNamespace(positions_rad=start)
    session._settled_snapshot = lambda: SimpleNamespace(positions_rad=start)
    current = np.eye(4) if current_tip is None else current_tip.copy()
    requested = current.copy() if target_tip is None else target_tip.copy()
    actual = requested.copy() if actual_tip is None else actual_tip.copy()
    tip_reads = iter((current, actual))
    session.current_tip_matrix = lambda **_kwargs: next(tip_reads)
    session._solve_collision_checked_ik = lambda _target, seed: (
        target if tuple(seed) == start else (_ for _ in ()).throw(
            AssertionError("IK must use the explicit actual start")
        )
    )
    session.move_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
    )
    session.execute_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
    )
    monkeypatch.setattr(
        tcp_console,
        "rod_tip_target_to_tool0",
        lambda _builder, _target, *, frame_id: SimpleNamespace(frame_id=frame_id),
    )
    session._test_target_tip = requested
    return session, names, start, target


def test_absolute_xyz_allows_85mm_to_reach_ik_but_rejects_over_150mm(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class ReachedCollisionCheckedIk(RuntimeError):
        pass

    session = _bare_action_session(service_timeout_s=0.1)
    session.args.state_timeout_s = 0.1
    session.args.max_translation_m = 0.050
    session.args.max_absolute_translation_m = 0.150
    session.args.max_rotation_deg = 10.0
    session.base_frame = "base"
    session.tool_builder = object()
    positions = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    session._runtime_absent = lambda: None
    session.verify_required_controller = lambda: None
    session.live_authoritative_transforms = lambda: (np.eye(4), np.eye(4))
    session.verify_scene = lambda: None
    session.wait_settled = lambda **_kwargs: SimpleNamespace(positions_rad=positions)
    session._settled_snapshot = lambda: SimpleNamespace(positions_rad=positions)
    session.current_tip_matrix = lambda **_kwargs: np.eye(4)
    ik_calls: list[object] = []

    def reached_ik(target: object, _seed: object) -> tuple[float, ...]:
        ik_calls.append(target)
        raise ReachedCollisionCheckedIk("planning gate reached")

    session._solve_collision_checked_ik = reached_ik
    session.move_client = SimpleNamespace(wait_for_server=lambda **_kwargs: True)
    session.execute_client = SimpleNamespace(wait_for_server=lambda **_kwargs: True)
    monkeypatch.setattr(
        tcp_console,
        "rod_tip_target_to_tool0",
        lambda _builder, _target, *, frame_id: SimpleNamespace(frame_id=frame_id),
    )

    with pytest.raises(ReachedCollisionCheckedIk, match="planning gate reached"):
        session.move_absolute_position(
            tcp_console.AbsolutePositionCommand("base", (0.0855, 0.0, 0.0))
        )
    assert len(ik_calls) == 1

    with pytest.raises(ValueError, match=r"150\.001 mm exceeds limit 150\.000 mm"):
        session.move_absolute_position(
            tcp_console.AbsolutePositionCommand("base", (0.150001, 0.0, 0.0))
        )
    assert len(ik_calls) == 1

    # The terminal's full-pose command intentionally retains the older 50 mm
    # single-command ceiling; only the GUI's position-only path is widened.
    with pytest.raises(ValueError, match=r"85\.500 mm exceeds limit 50\.000 mm"):
        session.move_absolute(parse_pose_command("base p[0.0855,0,0,0,0,0]"))
    assert len(ik_calls) == 1


def test_manual_motion_plans_only_then_executes_the_same_validated_trajectory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    current_tip = np.eye(4)
    target_tip = np.eye(4)
    target_tip[0, 3] = 0.002
    actual_tip = target_tip.copy()
    actual_tip[1, 3] = 0.0001
    session, names, start, target = _split_execution_session(
        monkeypatch,
        current_tip=current_tip,
        target_tip=target_tip,
        actual_tip=actual_tip,
    )
    trajectory = _planned_trajectory(
        names,
        (
            _trajectory_point(start, 0.0),
            _trajectory_point((-1.68, -1.18, -1.38), 0.5),
            _trajectory_point(target, 1.0),
        ),
    )
    plan_goal = SimpleNamespace(
        planning_options=SimpleNamespace(plan_only=True),
    )
    build_goal_calls: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
    ordering: list[str] = []
    session.verify_scene = lambda: ordering.append("scene")

    def build_goal(
        target_positions: tuple[float, ...],
        start_positions: tuple[float, ...],
    ) -> object:
        build_goal_calls.append((tuple(target_positions), tuple(start_positions)))
        return plan_goal

    session._build_goal = build_goal
    action_calls: list[tuple[object, object, str]] = []

    def run_action(
        client: object,
        goal: object,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
        pre_submit: object | None = None,
    ) -> object:
        assert cancel_event is None
        action_calls.append((client, goal, action_label))
        if client is session.move_client:
            assert pre_submit is None
            ordering.append("plan")
            assert goal is plan_goal
            assert goal.planning_options.plan_only is True
            return SimpleNamespace(
                status=4,
                result=SimpleNamespace(
                    error_code=SimpleNamespace(val=1),
                    planned_trajectory=trajectory,
                ),
            )
        assert client is session.execute_client
        assert callable(pre_submit)
        pre_submit()
        ordering.append("execute")
        assert goal.trajectory is trajectory
        assert goal.controller_names == [
            "scaled_joint_trajectory_controller"
        ]
        return _wrapped_action_result(4)

    session._run_tracked_action = run_action

    measured = session._execute_tip_target(
        lambda _current, _base_to_plane1: target_tip.copy(),
        cancel_event=None,
    )

    assert build_goal_calls == [(target, start)]
    assert [call[2] for call in action_calls] == [
        "MoveGroup planning",
        "ExecuteTrajectory",
    ]
    assert action_calls[0][0] is session.move_client
    assert action_calls[1][0] is session.execute_client
    assert ordering == ["scene", "plan", "scene", "scene", "execute"]
    assert measured.translation_m == pytest.approx(math.hypot(0.002, 0.0001))


def test_move_group_plan_failure_never_calls_execute_trajectory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session, _names, _start, _target = _split_execution_session(monkeypatch)
    session._build_goal = lambda _target, _start: SimpleNamespace(
        planning_options=SimpleNamespace(plan_only=True)
    )
    execution_attempted = threading.Event()

    def run_action(
        client: object,
        _goal: object,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
    ) -> object:
        assert cancel_event is None
        if client is session.execute_client:
            execution_attempted.set()
            raise AssertionError("ExecuteTrajectory must not receive a failed plan")
        assert action_label == "MoveGroup planning"
        return SimpleNamespace(
            status=4,
            result=SimpleNamespace(
                error_code=SimpleNamespace(val=-1),
                planned_trajectory=None,
            ),
        )

    session._run_tracked_action = run_action

    with pytest.raises(RuntimeError, match="plan|MoveIt|failure|code"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: session._test_target_tip.copy(),
            cancel_event=None,
        )

    assert execution_attempted.is_set() is False


@pytest.mark.parametrize(
    "trajectory_kind",
    ["empty", "malformed"],
)
def test_empty_or_malformed_plan_never_calls_execute_trajectory(
    monkeypatch: pytest.MonkeyPatch,
    trajectory_kind: str,
) -> None:
    session, names, start, target = _split_execution_session(monkeypatch)
    trajectory = (
        _planned_trajectory(names, ())
        if trajectory_kind == "empty"
        else _planned_trajectory(
            names,
            (
                _trajectory_point(start, 0.0),
                _trajectory_point((target[0], math.nan, target[2]), 1.0),
            ),
        )
    )
    session._build_goal = lambda _target, _start: SimpleNamespace(
        planning_options=SimpleNamespace(plan_only=True)
    )
    execution_attempted = threading.Event()

    def run_action(
        client: object,
        _goal: object,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
    ) -> object:
        assert cancel_event is None
        if client is session.execute_client:
            execution_attempted.set()
            raise AssertionError("ExecuteTrajectory must not receive a malformed plan")
        assert action_label == "MoveGroup planning"
        return SimpleNamespace(
            status=4,
            result=SimpleNamespace(
                error_code=SimpleNamespace(val=1),
                planned_trajectory=trajectory,
            ),
        )

    session._run_tracked_action = run_action

    with pytest.raises(RuntimeError, match="trajectory|point|finite|empty"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: session._test_target_tip.copy(),
            cancel_event=None,
        )

    assert execution_attempted.is_set() is False


def test_actual_start_drift_after_planning_never_calls_execute_trajectory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session, names, start, target = _split_execution_session(monkeypatch)
    trajectory = _planned_trajectory(
        names,
        (
            _trajectory_point(start, 0.0),
            _trajectory_point(target, 1.0),
        ),
    )
    session._build_goal = lambda _target, _start: SimpleNamespace(
        planning_options=SimpleNamespace(plan_only=True)
    )
    plan_completed = threading.Event()
    execution_attempted = threading.Event()
    drifted = (start[0] + 0.000501, *start[1:])

    def snapshot() -> object:
        positions = drifted if plan_completed.is_set() else start
        return SimpleNamespace(positions_rad=positions)

    session._settled_snapshot = snapshot
    session.wait_settled = lambda **_kwargs: snapshot()

    def run_action(
        client: object,
        _goal: object,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
    ) -> object:
        assert cancel_event is None
        if client is session.execute_client:
            execution_attempted.set()
            raise AssertionError("ExecuteTrajectory must not run after start drift")
        assert action_label == "MoveGroup planning"
        plan_completed.set()
        return SimpleNamespace(
            status=4,
            result=SimpleNamespace(
                error_code=SimpleNamespace(val=1),
                planned_trajectory=trajectory,
            ),
        )

    session._run_tracked_action = run_action

    with pytest.raises(RuntimeError, match="changed|drift|start|0.000501"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: session._test_target_tip.copy(),
            cancel_event=None,
        )

    assert execution_attempted.is_set() is False


def test_planned_first_point_must_match_the_rechecked_live_execution_start(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session, names, start, target = _split_execution_session(monkeypatch)
    # Each offset is individually at the 0.0005 rad drift ceiling relative to
    # the planning start, but in opposite directions.  Checking them only
    # against the older start would permit a 0.001 rad discontinuity at send.
    planned_first = (start[0] + 0.0005, *start[1:])
    live_execution_start = (start[0] - 0.0005, *start[1:])
    trajectory = _planned_trajectory(
        names,
        (
            _trajectory_point(planned_first, 0.0),
            _trajectory_point(target, 1.0),
        ),
    )
    session._build_goal = lambda _target, _start: SimpleNamespace(
        planning_options=SimpleNamespace(plan_only=True)
    )
    plan_completed = threading.Event()
    execution_attempted = threading.Event()

    def wait_settled(**_kwargs: object) -> object:
        positions = live_execution_start if plan_completed.is_set() else start
        return SimpleNamespace(positions_rad=positions)

    session.wait_settled = wait_settled

    def run_action(
        client: object,
        _goal: object,
        *,
        action_label: str,
        cancel_event: threading.Event | None,
    ) -> object:
        assert cancel_event is None
        if client is session.execute_client:
            execution_attempted.set()
            raise AssertionError(
                "ExecuteTrajectory must not bridge a live-start discontinuity"
            )
        assert action_label == "MoveGroup planning"
        plan_completed.set()
        return SimpleNamespace(
            status=4,
            result=SimpleNamespace(
                error_code=SimpleNamespace(val=1),
                planned_trajectory=trajectory,
            ),
        )

    session._run_tracked_action = run_action

    with pytest.raises(RuntimeError, match="first|start|differ|0.001"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: session._test_target_tip.copy(),
            cancel_event=None,
        )

    assert execution_attempted.is_set() is False


def test_final_start_drift_during_execute_server_wait_prevents_trajectory_send(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session, names, start, target = _split_execution_session(monkeypatch)
    trajectory = _planned_trajectory(
        names,
        (
            _trajectory_point(start, 0.0),
            _trajectory_point(target, 1.0),
        ),
    )
    plan_goal = SimpleNamespace(planning_options=SimpleNamespace(plan_only=True))
    session._build_goal = lambda _target, _start: plan_goal
    drifted = (start[0] + 0.000501, *start[1:])
    drift_during_final_wait = threading.Event()
    execute_sent = threading.Event()

    def completed_goal_response(wrapped: object) -> Future[object]:
        handle = _FakeGoalHandle()
        handle.result_future.set_result(wrapped)
        response: Future[object] = Future()
        response.set_result(handle)
        return response

    plan_wrapped = SimpleNamespace(
        status=4,
        result=SimpleNamespace(
            error_code=SimpleNamespace(val=1),
            planned_trajectory=trajectory,
        ),
    )
    session.move_client = SimpleNamespace(
        wait_for_server=lambda **_kwargs: True,
        send_goal_async=lambda goal: (
            completed_goal_response(plan_wrapped)
            if goal is plan_goal
            else (_ for _ in ()).throw(AssertionError("unexpected MoveGroup goal"))
        ),
    )

    execute_wait_count = 0

    def wait_for_execute_server(**_kwargs: object) -> bool:
        nonlocal execute_wait_count
        execute_wait_count += 1
        # The first call is the early availability preflight.  The second is
        # immediately inside the eventual ExecuteTrajectory submission path.
        if execute_wait_count == 2:
            drift_during_final_wait.set()
        return True

    def send_execute(_goal: object) -> Future[object]:
        execute_sent.set()
        return completed_goal_response(_wrapped_action_result(4))

    session.execute_client = SimpleNamespace(
        wait_for_server=wait_for_execute_server,
        send_goal_async=send_execute,
    )

    def snapshot() -> object:
        positions = drifted if drift_during_final_wait.is_set() else start
        return SimpleNamespace(positions_rad=positions)

    session._settled_snapshot = snapshot
    session.wait_settled = lambda **_kwargs: snapshot()

    with pytest.raises(RuntimeError, match="changed|drift|start|0.000501"):
        session._execute_tip_target(
            lambda _current, _base_to_plane1: session._test_target_tip.copy(),
            cancel_event=None,
        )

    assert execute_wait_count == 2
    assert execute_sent.is_set() is False


@pytest.mark.parametrize(
    "mismatch",
    ["frame_id", "is_box", "dimensions_m", "position_m", "quaternion_xyzw"],
)
def test_scene_inventory_compares_complete_world_cuboid_geometry(
    repository_root: Path,
    mismatch: str,
) -> None:
    scene = load_planning_scene_definition(repository_root, "mla")
    actual_cuboids = {
        cuboid.object_id: WorldCuboidObservation(
            object_id=cuboid.object_id,
            frame_id=cuboid.frame_id,
            is_box=True,
            dimensions_m=cuboid.dimensions_m,
            position_m=cuboid.position_m,
            quaternion_xyzw=cuboid.quaternion_xyzw,
        )
        for cuboid in scene.cuboids
    }
    rod = scene.attached_rod
    actual_rods = {
        rod.object_id: AttachedRodObservation(
            object_id=rod.object_id,
            link_name=rod.link_name,
            header_frame=rod.link_name,
            is_cylinder=True,
            dimensions_m=(rod.length_m, rod.radius_m),
            center_m=rod.center_in_link_m,
            quaternion_xyzw=rod.quaternion_xyzw,
            touch_links=rod.touch_links,
        )
    }
    exact = compare_scene_inventory(
        scene.cuboids,
        rod,
        actual_cuboids,
        actual_rods,
        actual_cuboids=actual_cuboids,
    )
    assert exact.ok

    first_id = scene.cuboids[0].object_id
    original = actual_cuboids[first_id]
    mismatch_value: object
    if mismatch == "frame_id":
        mismatch_value = "wrong_frame"
    elif mismatch == "is_box":
        mismatch_value = False
    elif mismatch == "dimensions_m":
        mismatch_value = (original.dimensions_m[0] + 0.01, *original.dimensions_m[1:])
    elif mismatch == "position_m":
        mismatch_value = (original.position_m[0] + 0.01, *original.position_m[1:])
    else:
        mismatch_value = rotvec_to_quaternion_xyzw((0.1, 0.0, 0.0))
    actual_cuboids[first_id] = replace(original, **{mismatch: mismatch_value})

    comparison = compare_scene_inventory(
        scene.cuboids,
        rod,
        actual_cuboids,
        actual_rods,
        actual_cuboids=actual_cuboids,
    )

    assert comparison.ok is False
    assert comparison.missing_world_ids == ()
    assert any(first_id in error for error in comparison.world_geometry_errors)


def test_acm_rejects_world_or_non_touch_rod_allowances() -> None:
    names = ("sensor_board_keepout", "hx_waypoint_attached_rod", "tool0", "wrist_3_link")

    def matrix_with(*allowed_pairs: tuple[str, str]) -> SimpleNamespace:
        enabled = [[False for _ in names] for _ in names]
        for left, right in allowed_pairs:
            left_index = names.index(left)
            right_index = names.index(right)
            enabled[left_index][right_index] = True
            enabled[right_index][left_index] = True
        return SimpleNamespace(
            entry_names=list(names),
            entry_values=[SimpleNamespace(enabled=row) for row in enabled],
            default_entry_names=[],
            default_entry_values=[],
        )

    expected_touch = forbidden_acm_allowances(
        matrix_with(("hx_waypoint_attached_rod", "tool0")),
        world_object_ids=("sensor_board_keepout",),
        attached_rod_id="hx_waypoint_attached_rod",
        rod_touch_links=("tool0",),
    )
    assert expected_touch == ()

    errors = forbidden_acm_allowances(
        matrix_with(
            ("sensor_board_keepout", "wrist_3_link"),
            ("hx_waypoint_attached_rod", "wrist_3_link"),
        ),
        world_object_ids=("sensor_board_keepout",),
        attached_rod_id="hx_waypoint_attached_rod",
        rod_touch_links=("tool0",),
    )
    assert any("sensor_board_keepout" in error for error in errors)
    assert any("non-touch link" in error for error in errors)


def test_acm_rejects_any_true_default_even_for_an_unprotected_name() -> None:
    names = ("sensor_board_keepout", "hx_waypoint_attached_rod", "tool0")
    matrix = SimpleNamespace(
        entry_names=list(names),
        entry_values=[
            SimpleNamespace(enabled=[False for _ in names]) for _ in names
        ],
        default_entry_names=["unrelated_robot_link"],
        default_entry_values=[True],
    )

    errors = forbidden_acm_allowances(
        matrix,
        world_object_ids=("sensor_board_keepout",),
        attached_rod_id="hx_waypoint_attached_rod",
        rod_touch_links=("tool0",),
    )

    assert errors
    assert any(
        "default collision allowance" in error
        and "unrelated_robot_link" in error
        for error in errors
    )


def test_acm_rejects_duplicate_default_names() -> None:
    names = ("sensor_board_keepout", "hx_waypoint_attached_rod", "tool0")
    matrix = SimpleNamespace(
        entry_names=list(names),
        entry_values=[
            SimpleNamespace(enabled=[False for _ in names]) for _ in names
        ],
        default_entry_names=["tool0", "tool0"],
        default_entry_values=[False, False],
    )

    errors = forbidden_acm_allowances(
        matrix,
        world_object_ids=("sensor_board_keepout",),
        attached_rod_id="hx_waypoint_attached_rod",
        rod_touch_links=("tool0",),
    )

    assert errors == ("allowed collision matrix has duplicate defaults",)


def test_acm_rejects_an_empty_explicit_matrix() -> None:
    matrix = SimpleNamespace(
        entry_names=[],
        entry_values=[],
        default_entry_names=[],
        default_entry_values=[],
    )

    errors = forbidden_acm_allowances(
        matrix,
        world_object_ids=("sensor_board_keepout",),
        attached_rod_id="hx_waypoint_attached_rod",
        rod_touch_links=("tool0",),
    )

    assert errors
    assert any("empty" in error.lower() for error in errors)


def test_once_read_only_snapshot_does_not_require_robot_to_be_settled(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    events: list[str] = []

    class FakeBuilder:
        def load(self, _path: Path) -> object:
            return SimpleNamespace(environment_id="mla")

    class FakeSession:
        def __init__(self, _environment: object, _scene: object, _args: object) -> None:
            events.append("session")

        def wait_settled(self, **_kwargs: object) -> None:
            raise AssertionError("read-only viewing must not require settled JointState")

        def wait_fresh_joint_state(self, **_kwargs: object) -> None:
            events.append("fresh")

        def snapshot_text(self) -> str:
            return "live tcp snapshot"

        def close(self) -> None:
            events.append("closed")

    fake_rclpy = ModuleType("rclpy")
    fake_rclpy.init = lambda **_kwargs: events.append("init")  # type: ignore[attr-defined]
    fake_rclpy.shutdown = lambda: events.append("shutdown")  # type: ignore[attr-defined]
    fake_rclpy_signals = ModuleType("rclpy.signals")
    fake_rclpy_signals.SignalHandlerOptions = SimpleNamespace(NO=object())  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.signals", fake_rclpy_signals)
    monkeypatch.setattr(tcp_console, "EnvironmentBuilder", FakeBuilder)
    monkeypatch.setattr(
        tcp_console,
        "load_planning_scene_definition",
        lambda _workspace, _environment_id: SimpleNamespace(environment_id="mla"),
    )
    monkeypatch.setattr(tcp_console, "RosTcpSession", FakeSession)

    result = tcp_console.main(["--env", "mla", "--once"])

    assert result == 0
    assert "live tcp snapshot" in capsys.readouterr().out
    assert events == ["init", "session", "fresh", "closed", "shutdown"]
