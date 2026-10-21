from __future__ import annotations

import importlib
import inspect
import os
import signal
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest


def test_module_import_does_not_import_qt_or_ros() -> None:
    for name in tuple(sys.modules):
        if name == "hx_waypoint_tools.tcp_gui":
            del sys.modules[name]
        if name == "python_qt_binding" or name.startswith("python_qt_binding."):
            del sys.modules[name]
        if name == "rclpy" or name.startswith("rclpy."):
            del sys.modules[name]

    module = importlib.import_module("hx_waypoint_tools.tcp_gui")

    assert module.GUI_REFRESH_HZ == pytest.approx(5.0)
    assert not any(
        name == "python_qt_binding" or name.startswith("python_qt_binding.")
        for name in sys.modules
    )
    assert not any(name == "rclpy" or name.startswith("rclpy.") for name in sys.modules)


def test_discovers_only_path_safe_environment_bundles(tmp_path: Path) -> None:
    from hx_waypoint_tools.tcp_gui import discover_environment_ids

    environments = tmp_path / "environments"
    (environments / "mla").mkdir(parents=True)
    (environments / "mla" / "setup.yaml").write_text("environment_id: mla\n")
    (environments / "lab_wall-a").mkdir()
    (environments / "lab_wall-a" / "setup.yaml").write_text(
        "environment_id: lab_wall-a\n"
    )
    (environments / "not_a_bundle").mkdir()
    (environments / "bad.name").mkdir()
    (environments / "bad.name" / "setup.yaml").write_text("bad\n")
    (environments / ".hidden").mkdir()
    (environments / ".hidden" / "setup.yaml").write_text("bad\n")

    assert discover_environment_ids(tmp_path) == ("lab_wall-a", "mla")


@pytest.mark.parametrize("value", ["192.168.1.99", "192.168.1.1", "10.0.0.2"])
def test_robot_ip_accepts_literal_ipv4(value: str) -> None:
    from hx_waypoint_tools.tcp_gui import validate_robot_ip

    assert validate_robot_ip(f" {value} ") == value


@pytest.mark.parametrize(
    "value",
    ["", "ur.local", "192.168.1", "192.168.1.999", "::1", "192.168.1.99; reboot"],
)
def test_robot_ip_rejects_non_literal_ipv4(value: str) -> None:
    from hx_waypoint_tools.tcp_gui import validate_robot_ip

    with pytest.raises(ValueError, match="IPv4"):
        validate_robot_ip(value)


def test_manual_real_launch_command_is_argument_vector_with_environment_contract(
    tmp_path: Path,
) -> None:
    from hx_waypoint_tools.tcp_gui import LaunchSpec, build_real_launch_command

    workspace = tmp_path / "workspace with spaces"
    spec = LaunchSpec(
        workspace=workspace,
        environment_id="mla",
        robot_ip="192.168.1.99",
        launch_rviz=True,
    )

    command = build_real_launch_command(spec)

    assert command[:4] == (
        "ros2",
        "launch",
        "hx_waypoint_bringup",
        "real.launch.py",
    )
    assert f"workspace:={workspace.resolve()}" in command
    assert "environment_id:=mla" in command
    assert "robot_ip:=192.168.1.99" in command
    assert "launch_rviz:=true" in command
    assert "manual_control:=true" in command
    assert all(" " not in argument or argument.startswith("workspace:=") for argument in command)


def test_existing_graph_probe_detects_servers_but_ignores_action_client(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    outputs = iter(
        (
            SimpleNamespace(stdout="Services count: 1\n", stderr=""),
            SimpleNamespace(
                stdout="Action clients: 1\nAction servers: 0\n", stderr=""
            ),
        )
    )
    calls: list[tuple[str, ...]] = []

    def fake_run(command, **kwargs):
        calls.append(tuple(command))
        assert kwargs["shell"] is False
        assert kwargs["timeout"] == pytest.approx(2.0)
        return next(outputs)

    monkeypatch.setattr(tcp_gui.subprocess, "run", fake_run)

    assert tcp_gui.find_existing_control_graph() == ("controller_manager",)
    assert calls[0][-1] == "/controller_manager/list_controllers"
    assert calls[1][-1] == "/hx_waypoint/play_recording"


@pytest.mark.parametrize("environment_id", ["../mla", "mla/x", "bad.name", "", "."])
def test_launch_command_rejects_unsafe_environment_id(
    tmp_path: Path, environment_id: str
) -> None:
    from hx_waypoint_tools.tcp_gui import LaunchSpec, build_real_launch_command

    with pytest.raises(ValueError, match="environment"):
        build_real_launch_command(
            LaunchSpec(tmp_path, environment_id, "192.168.1.99", False)
        )


def test_gui_absolute_position_accepts_three_independent_xyz_fields_only() -> None:
    from hx_waypoint_tools.tcp_console import AbsolutePositionCommand
    from hx_waypoint_tools.tcp_gui import parse_gui_absolute_position

    command = parse_gui_absolute_position("plane1", " .1 ", "-.2", "-.07")

    assert isinstance(command, AbsolutePositionCommand)
    assert command.frame == "plane1"
    assert command.position_m == pytest.approx((0.1, -0.2, -0.07))


def test_gui_absolute_translation_limit_defaults_to_150mm_and_is_configurable() -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    defaults = tcp_gui._build_parser().parse_args([])
    configured = tcp_gui._build_parser().parse_args(
        ["--max-absolute-translation-mm", "120"]
    )

    assert defaults.max_absolute_translation_mm == pytest.approx(150.0)
    assert configured.max_absolute_translation_mm == pytest.approx(120.0)
    assert '"--max-absolute-translation-mm"' in inspect.getsource(
        tcp_gui._open_ros_session
    )


@pytest.mark.parametrize("frame", ["base", "plane1"])
def test_gui_absolute_position_accepts_each_supported_frame(frame: str) -> None:
    from hx_waypoint_tools.tcp_gui import parse_gui_absolute_position

    command = parse_gui_absolute_position(frame, "0", "0.025", "-0.081")

    assert command.frame == frame
    assert command.position_m == pytest.approx((0.0, 0.025, -0.081))


@pytest.mark.parametrize(
    ("frame", "x", "y", "z"),
    [
        ("plane1", "", "0", "0"),
        ("plane1", "0", "   ", "0"),
        ("plane1", "0", "0", ""),
        ("plane1", "p[0.1,0.2,0.3,0,0,0]", "0", "0"),
        ("plane1", "one", "0", "0"),
        ("plane1", "nan", "0", "0"),
        ("plane1", "0", "inf", "0"),
        ("map", "0", "0", "0"),
    ],
)
def test_gui_absolute_position_rejects_missing_full_pose_nonfinite_or_bad_frame(
    frame: str, x: str, y: str, z: str
) -> None:
    from hx_waypoint_tools.tcp_gui import parse_gui_absolute_position

    with pytest.raises(ValueError):
        parse_gui_absolute_position(frame, x, y, z)


@pytest.mark.parametrize("frame", ["base", "plane1"])
def test_absolute_position_target_replaces_only_xyz_and_preserves_live_orientation(
    frame: str,
) -> None:
    import numpy as np

    from hx_waypoint_curobo.math3d import pose_matrix
    from hx_waypoint_tools.tcp_console import absolute_position_target_in_base
    from hx_waypoint_tools.tcp_gui import parse_gui_absolute_position

    # Deliberately use non-identity, different work-frame and live TCP
    # rotations.  "Keep current orientation" means the resulting physical
    # base-frame orientation must be bit-for-bit derived from the live pose;
    # it must not silently become the work-frame orientation.
    base_to_plane1 = pose_matrix(
        (-0.06528, -0.49473, 0.18244),
        (0.3692189854, 0.9293397736, -0.0000510015, 0.0022188758),
    )
    current_base_to_tip = pose_matrix(
        (0.31, -0.22, 0.48),
        (0.1825741858, -0.3651483717, 0.5477225575, 0.7302967433),
    )
    position = (0.032, -0.041, -0.078)
    command = parse_gui_absolute_position(
        frame, *(f"{component:.9f}" for component in position)
    )

    target = absolute_position_target_in_base(
        command, current_base_to_tip, base_to_plane1
    )

    expected_position = (
        np.asarray(position)
        if frame == "base"
        else base_to_plane1[:3, :3] @ np.asarray(position)
        + base_to_plane1[:3, 3]
    )
    assert target[:3, 3] == pytest.approx(expected_position, abs=1.0e-12)
    assert target[:3, :3] == pytest.approx(
        current_base_to_tip[:3, :3], abs=1.0e-12
    )


def test_qt_window_wires_xyz_fields_to_position_only_motion() -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    # Qt/ROS are intentionally unavailable in the lightweight unit-test
    # process, so keep this as a narrow wiring contract on the lazy window
    # factory.  The transform and parser behavior are tested independently
    # above.
    source = inspect.getsource(tcp_gui._make_window_class)
    assert 'self.absolute_xyz_edits: dict[str, Any] = {}' in source
    assert 'for axis in ("X", "Y", "Z")' in source
    assert 'QtWidgets.QLabel(f"{axis} · m")' in source
    assert "parse_gui_absolute_position(" in source
    assert "session.move_absolute_position(" in source
    assert "self.absolute_edit" not in source
    assert "session.move_absolute(command" not in source


def test_teardown_gate_refuses_running_worker_or_unresolved_ros_goal() -> None:
    from hx_waypoint_tools.tcp_gui import safe_to_teardown_ros

    assert safe_to_teardown_ros(SimpleNamespace(running=False), False)
    assert not safe_to_teardown_ros(SimpleNamespace(running=True), False)
    assert not safe_to_teardown_ros(SimpleNamespace(running=False), True)


def test_manual_ready_wait_retries_complete_gate_before_reporting_ready() -> None:
    from hx_waypoint_tools.tcp_gui import wait_until_manual_session_ready

    class Session:
        calls = 0

        def verify_manual_readiness(self, *, joint_state_timeout_s):
            assert joint_state_timeout_s == pytest.approx(1.0)
            self.calls += 1
            if self.calls < 3:
                raise RuntimeError("controller is not active")

    session = Session()
    wait_until_manual_session_ready(session, timeout_s=1.0)

    assert session.calls == 3


def test_manual_ready_wait_stops_when_gui_startup_is_cancelled() -> None:
    from hx_waypoint_tools.tcp_gui import wait_until_manual_session_ready

    cancelled = threading.Event()
    cancelled.set()

    with pytest.raises(RuntimeError, match="cancelled"):
        wait_until_manual_session_ready(
            SimpleNamespace(), cancel_event=cancelled, timeout_s=1.0
        )


@pytest.mark.parametrize(
    ("key", "expected"),
    [("W", "w"), ("A", "a"), ("S", "s"), ("D", "d"), ("R", "r"), ("F", "f")],
)
def test_keyboard_jog_accepts_only_explicit_non_repeated_unfocused_enabled_key(
    key: str, expected: str
) -> None:
    from hx_waypoint_tools.tcp_gui import keyboard_jog_command

    assert keyboard_jog_command(
        key,
        motion_enabled=True,
        text_input_focused=False,
        auto_repeat=False,
        motion_busy=False,
    ) == expected
    assert keyboard_jog_command(
        key,
        motion_enabled=False,
        text_input_focused=False,
        auto_repeat=False,
        motion_busy=False,
    ) is None
    assert keyboard_jog_command(
        key,
        motion_enabled=True,
        text_input_focused=True,
        auto_repeat=False,
        motion_busy=False,
    ) is None
    assert keyboard_jog_command(
        key,
        motion_enabled=True,
        text_input_focused=False,
        auto_repeat=True,
        motion_busy=False,
    ) is None
    assert keyboard_jog_command(
        key,
        motion_enabled=True,
        text_input_focused=False,
        auto_repeat=False,
        motion_busy=True,
    ) is None


def test_qt_window_wires_servo_press_refresh_and_immediate_zero_stops() -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    source = inspect.getsource(tcp_gui._make_window_class)
    assert "button.pressed.connect(" in source
    assert "button.released.connect(" in source
    assert "QtCore.QEvent.KeyRelease" in source
    assert "QtCore.QEvent.WindowDeactivate" in source
    assert "self.installEventFilter(self)" in source
    # Press starts Servo immediately; the normal GUI timer refreshes the same
    # non-zero command while held.  There is no release grace and no sequence
    # of collision-planned OneMotionWorker jogs.
    assert "SERVO_COMMAND_HZ" in source
    assert "_publish_held_jog" in source
    assert "session.begin_servo_jog(" in source
    assert "session.refresh_servo_jog(" in source
    assert "session.stop_servo_jog(" in source
    assert "HOLD_RELEASE_GRACE_MS" not in source
    assert "_continue_jog_hold" not in source
    assert "session.move_jog(" not in source

    # Every authority/lifecycle escape path converges on the same zero-command
    # operation.  A held intent must be cleared before that zero is published,
    # so a later timer tick cannot resurrect motion.
    for reason in (
        "window focus lost",
        "motion authority disabled",
        "operator cancellation",
        "manual stack stop",
        "servo publish failed",
    ):
        assert reason in source
    assert "def _clear_jog_hold(" in source
    assert "self.runtime.session.stop_servo_jog(" in source


def test_successful_absolute_dispatch_returns_focus_to_keyboard_jog() -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    source = inspect.getsource(tcp_gui._make_window_class)
    constructor = source.split("def __init__", 1)[1].split("\n        def ", 1)[0]
    dispatch = source.split("def _dispatch_absolute", 1)[1].split(
        "\n        def ", 1
    )[0]

    assert "self.setFocusPolicy(QtCore.Qt.StrongFocus)" in constructor
    assert "if self.worker.start(" in dispatch
    accepted = dispatch.split("if self.worker.start(", 1)[1]
    assert "for edit in self.absolute_xyz_edits.values():" in accepted
    assert "edit.clearFocus()" in accepted
    assert "self.setFocus(QtCore.Qt.OtherFocusReason)" in accepted


def test_qt_servo_speed_selector_is_exactly_5_10_20_mm_per_second() -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    source = inspect.getsource(tcp_gui._make_window_class)
    assert 'addItems(("5 mm/s", "10 mm/s", "20 mm/s"))' in source
    assert "self.speed_combo.setCurrentIndex(1)" in source
    assert "(5.0, 10.0, 20.0)" in source
    assert "def _speed_m_s(" in source
    assert 'addItems(("1 mm", "2 mm", "5 mm"))' not in source


@pytest.mark.parametrize(
    ("key", "frame", "expected"),
    (
        ("w", "base", (0.0, 0.01, 0.0)),
        ("s", "base", (0.0, -0.01, 0.0)),
        ("a", "base", (-0.01, 0.0, 0.0)),
        ("d", "base", (0.01, 0.0, 0.0)),
        ("r", "base", (0.0, 0.0, 0.01)),
        ("f", "base", (0.0, 0.0, -0.01)),
        ("r", "plane1", (0.0, 0.0, -0.01)),
        ("f", "plane1", (0.0, 0.0, 0.01)),
    ),
)
def test_servo_jog_velocity_has_exact_signed_frame_semantics(
    key: str, frame: str, expected: tuple[float, float, float]
) -> None:
    from hx_waypoint_tools.tcp_console import servo_jog_velocity

    assert servo_jog_velocity(key, frame, 0.01) == pytest.approx(expected)


@pytest.mark.parametrize("speed", [0.0, -0.001, 0.0200001, float("nan")])
def test_servo_jog_velocity_rejects_nonpositive_nonfinite_or_excess_speed(
    speed: float,
) -> None:
    from hx_waypoint_tools.tcp_console import servo_jog_velocity

    with pytest.raises(ValueError):
        servo_jog_velocity("w", "base", speed)


def test_servo_gui_contract_uses_50hz_stream_and_not_motion_worker_for_jog() -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    assert tcp_gui.SERVO_COMMAND_HZ == pytest.approx(50.0)
    source = inspect.getsource(tcp_gui._make_window_class)
    begin = source.split("def _begin_jog_hold", 1)[1].split("\n        def ", 1)[0]
    publish = source.split("def _publish_held_jog", 1)[1].split("\n        def ", 1)[0]
    assert "begin_servo_jog(" in begin
    assert "self.worker.start(" not in begin
    assert "refresh_servo_jog(" in publish
    assert "self.worker.start(" not in publish


def test_servo_session_exposes_generation_scoped_begin_refresh_and_zero_stop() -> None:
    from hx_waypoint_tools.tcp_console import RosTcpSession

    configure = inspect.signature(RosTcpSession.configure_servo)
    begin = inspect.signature(RosTcpSession.begin_servo_jog)
    refresh = inspect.signature(RosTcpSession.refresh_servo_jog)
    stop = inspect.signature(RosTcpSession.stop_servo_jog)
    assert tuple(configure.parameters)[0] == "self"
    assert {"key", "jog_frame", "speed_m_s"} <= set(begin.parameters)
    assert tuple(refresh.parameters) == ("self", "generation")
    assert tuple(stop.parameters) == ("self", "generation")
    assert stop.parameters["generation"].default is None
    assert hasattr(RosTcpSession, "servo_jog_active")


def test_vertical_labels_only_call_minus_plane_z_physical_raise_in_mla() -> None:
    from hx_waypoint_tools.tcp_gui import vertical_axis_labels

    mla = vertical_axis_labels("plane1", "mla")
    wall = vertical_axis_labels("plane1", "lab_wall_a")
    base = vertical_axis_labels("base", "lab_wall_a")

    assert mla.r_button == "R  −Z"
    assert "物理抬高" in mla.explanation
    assert wall.r_button == "R  −Z"
    assert "不一定是物理上下" in wall.explanation
    assert base.r_button == "R  +Z"


class _ReadablePipe:
    def __iter__(self):
        return iter(("launch started\n",))

    def close(self) -> None:
        pass


class _FakeProcess:
    def __init__(self) -> None:
        self.pid = 4321
        self.stdout = _ReadablePipe()
        self.returncode = None
        self.wait_calls: list[float | None] = []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        self.returncode = 0
        return 0


class _StubbornProcess(_FakeProcess):
    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        self.returncode = -int(signal.SIGKILL)
        return self.returncode


def test_managed_launch_starts_without_shell_in_a_new_process_group(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    captured: dict[str, object] = {}
    process = _FakeProcess()

    def fake_popen(command, **kwargs):
        captured["command"] = command
        captured.update(kwargs)
        return process

    monkeypatch.setattr(tcp_gui.subprocess, "Popen", fake_popen)
    logs: list[str] = []
    launch = tcp_gui.ManagedLaunchProcess(log_callback=logs.append)

    launch.start(("ros2", "launch", "pkg", "real.launch.py"), cwd=tmp_path)
    launch.join_log_reader(1.0)

    assert captured["command"] == ["ros2", "launch", "pkg", "real.launch.py"]
    assert captured["shell"] is False
    assert captured["start_new_session"] is True
    assert captured["cwd"] == str(tmp_path.resolve())
    assert "launch started" in logs


def test_managed_launch_stops_the_owned_process_group(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    process = _FakeProcess()
    monkeypatch.setattr(tcp_gui.subprocess, "Popen", lambda *_args, **_kwargs: process)
    signals: list[tuple[int, signal.Signals]] = []
    group_alive = {"value": True}

    def fake_killpg(pid, sent_signal):
        signals.append((pid, sent_signal))
        group_alive["value"] = False

    monkeypatch.setattr(os, "killpg", fake_killpg)
    monkeypatch.setattr(
        tcp_gui, "process_group_exists", lambda _pgid: group_alive["value"]
    )
    launch = tcp_gui.ManagedLaunchProcess()
    launch.start(("ros2", "launch", "pkg", "real.launch.py"), cwd=tmp_path)

    assert launch.stop() == 0
    assert signals == [(4321, signal.SIGINT)]
    assert process.wait_calls == [1.0]


def test_managed_launch_escalates_only_its_owned_group_until_it_is_gone(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    process = _StubbornProcess()
    monkeypatch.setattr(tcp_gui.subprocess, "Popen", lambda *_args, **_kwargs: process)
    signals: list[tuple[int, signal.Signals]] = []
    group_alive = {"value": True}

    def fake_killpg(pid, sent_signal):
        signals.append((pid, sent_signal))
        if sent_signal == signal.SIGKILL:
            group_alive["value"] = False

    monkeypatch.setattr(os, "killpg", fake_killpg)
    monkeypatch.setattr(
        tcp_gui, "process_group_exists", lambda _pgid: group_alive["value"]
    )
    clock = {"value": 0.0}

    def fake_monotonic():
        clock["value"] += 10.0
        return clock["value"]

    monkeypatch.setattr(tcp_gui.time, "monotonic", fake_monotonic)
    monkeypatch.setattr(tcp_gui.time, "sleep", lambda _seconds: None)
    launch = tcp_gui.ManagedLaunchProcess()
    launch.start(("ros2", "launch", "pkg", "real.launch.py"), cwd=tmp_path)

    assert launch.stop() == -int(signal.SIGKILL)
    assert signals == [
        (4321, signal.SIGINT),
        (4321, signal.SIGTERM),
        (4321, signal.SIGKILL),
    ]
    assert process.wait_calls == [1.0]


def test_managed_launch_still_stops_children_after_launch_leader_exits(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    process = _FakeProcess()
    process.returncode = 0
    monkeypatch.setattr(tcp_gui.subprocess, "Popen", lambda *_args, **_kwargs: process)
    group_alive = {"value": True}
    signals: list[signal.Signals] = []

    def fake_killpg(_pid, sent_signal):
        signals.append(sent_signal)
        group_alive["value"] = False

    monkeypatch.setattr(os, "killpg", fake_killpg)
    monkeypatch.setattr(
        tcp_gui, "process_group_exists", lambda _pgid: group_alive["value"]
    )
    launch = tcp_gui.ManagedLaunchProcess()
    launch.start(("ros2", "launch", "pkg", "real.launch.py"), cwd=tmp_path)

    assert launch.stop() == 0
    assert signals == [signal.SIGINT]


@pytest.mark.skipif(os.name != "posix", reason="process-group semantics are POSIX-only")
def test_managed_launch_reaps_real_leader_instead_of_mistaking_zombie_for_group(
    tmp_path: Path,
) -> None:
    from hx_waypoint_tools.tcp_gui import ManagedLaunchProcess, process_group_exists

    launch = ManagedLaunchProcess()
    pid = launch.start(
        (sys.executable, "-c", "import time; time.sleep(60)"), cwd=tmp_path
    )

    code = launch.stop(timeout_s=1.0)

    assert code is not None
    assert not process_group_exists(pid)


def test_bootstrap_result_requires_explicit_consumption_and_ack_after_thread_exit() -> None:
    from hx_waypoint_tools.tcp_gui import BootstrapAttempt, bootstrap_is_quiescent

    class FinishedThread:
        @staticmethod
        def is_alive() -> bool:
            return False

    runtime = object()
    attempt = BootstrapAttempt(7, threading.Event())
    attempt.publish(runtime=runtime)

    # This models a session_ready signal that is queued after its producer has
    # returned.  Thread.is_alive() alone must never allow window destruction.
    assert not bootstrap_is_quiescent(FinishedThread(), attempt)
    outcome = attempt.consume()
    assert outcome.runtime is runtime
    assert not bootstrap_is_quiescent(FinishedThread(), attempt)

    attempt.acknowledge()
    assert bootstrap_is_quiescent(FinishedThread(), attempt)
    with pytest.raises(RuntimeError, match="already consumed"):
        attempt.consume()


def test_sigint_and_sigterm_are_deferred_to_one_qt_window_close() -> None:
    from hx_waypoint_tools.tcp_gui import install_qt_close_signal_handlers

    class FakeSignalApi:
        SIGINT = 2
        SIGTERM = 15

        def __init__(self) -> None:
            self.original = {self.SIGINT: object(), self.SIGTERM: object()}
            self.handlers = dict(self.original)

        def getsignal(self, sent_signal):
            return self.handlers[sent_signal]

        def signal(self, sent_signal, handler):
            previous = self.handlers[sent_signal]
            self.handlers[sent_signal] = handler
            return previous

    class Window:
        close_calls = 0

        def close(self) -> None:
            self.close_calls += 1

    signal_api = FakeSignalApi()
    window = Window()
    bridge = install_qt_close_signal_handlers(
        SimpleNamespace(), window, signal_api=signal_api
    )

    signal_api.handlers[signal_api.SIGINT](signal_api.SIGINT, None)
    signal_api.handlers[signal_api.SIGTERM](signal_api.SIGTERM, None)
    assert window.close_calls == 0
    assert bridge.consume_requested()
    window.close()
    assert window.close_calls == 1
    assert not bridge.consume_requested()
    bridge.restore()
    assert signal_api.handlers == signal_api.original


def test_retained_qt_window_survives_embedded_main_until_destroyed() -> None:
    import hx_waypoint_tools.tcp_gui as tcp_gui

    class DestroyedSignal:
        def __init__(self) -> None:
            self.callback = None

        def connect(self, callback) -> None:
            self.callback = callback

        def emit(self) -> None:
            assert self.callback is not None
            self.callback()

    window = SimpleNamespace(destroyed=DestroyedSignal())
    released: list[bool] = []
    release = tcp_gui.retain_qt_window(
        window, on_release=lambda: released.append(True)
    )

    assert tcp_gui._ACTIVE_WINDOWS[id(window)] is window
    window.destroyed.emit()
    assert id(window) not in tcp_gui._ACTIVE_WINDOWS
    assert released == [True]

    release()
    assert released == [True]
