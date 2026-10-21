"""Qt desktop control surface for the environment-bound manual TCP session.

Qt and ROS are deliberately imported only after :func:`main` is called.  The
pure launch, validation and lifecycle helpers therefore remain testable on a
development machine without a ROS or Qt installation.
"""

from __future__ import annotations

import argparse
import ipaddress
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Sequence

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder

from .planning_scene_publisher import load_planning_scene_definition
from .tcp_console import (
    AbsolutePositionCommand,
    MotionDelta,
    OneMotionWorker,
    RosTcpSession,
    build_argument_parser as build_console_argument_parser,
    parse_absolute_pose,
)
from .tcp_console import _validate_args as _validate_console_args


GUI_REFRESH_HZ = 5.0
GUI_REFRESH_PERIOD_S = 1.0 / GUI_REFRESH_HZ
MOTION_MONITOR_HZ = 20.0
SERVO_COMMAND_HZ = 50.0
DEFAULT_ROBOT_IP = "192.168.1.99"
DEFAULT_ENVIRONMENT = "mla"
_ENVIRONMENT_ID = re.compile(r"[A-Za-z0-9_-]+")
_KEYBOARD_JOG = frozenset(("w", "a", "s", "d", "r", "f"))
_ACTIVE_WINDOWS: dict[int, Any] = {}


@dataclass(frozen=True)
class LaunchSpec:
    workspace: Path
    environment_id: str
    robot_ip: str
    launch_rviz: bool


@dataclass(frozen=True)
class VerticalAxisLabels:
    r_button: str
    f_button: str
    explanation: str


class JogHoldState:
    """One physical hold intent; its Servo generation lives in the window."""

    def __init__(self) -> None:
        self._source: str | None = None
        self._key: str | None = None
        self._epoch = 0

    @property
    def active(self) -> bool:
        return self._source is not None

    @property
    def source(self) -> str | None:
        return self._source

    @property
    def key(self) -> str | None:
        return self._key

    @property
    def epoch(self) -> int:
        return self._epoch

    def press(self, source: str, key: str) -> bool:
        normalized_source = str(source).strip()
        normalized_key = str(key).strip().lower()
        if not normalized_source:
            raise ValueError("jog hold source must not be empty")
        if normalized_key not in _KEYBOARD_JOG:
            raise ValueError(f"unsupported jog hold key {key!r}")
        if self.active:
            return self._source == normalized_source and self._key == normalized_key
        self._epoch += 1
        self._source = normalized_source
        self._key = normalized_key
        return True

    def release(self, source: str) -> bool:
        if not self.active or self._source != str(source).strip():
            return False
        self._epoch += 1
        self._source = None
        self._key = None
        return True

    def clear(self) -> bool:
        was_active = self.active
        if was_active:
            self._epoch += 1
        self._source = None
        self._key = None
        return was_active

@dataclass(frozen=True)
class BootstrapOutcome:
    runtime: Any | None
    error: str


class BootstrapAttempt:
    """One bootstrap result with explicit queued-signal ownership transfer.

    A producer thread publishes exactly once.  The Qt thread consumes exactly
    once and acknowledges only after it either owns the runtime or has handled
    the failure.  Thread termination alone is deliberately not an
    acknowledgement: its Qt signal may still be waiting in the event queue.
    """

    def __init__(self, generation: int, cancel_event: threading.Event) -> None:
        self.generation = int(generation)
        self.cancel_event = cancel_event
        self._lock = threading.Lock()
        self._outcome: BootstrapOutcome | None = None
        self._consumed = False
        self._acknowledged = threading.Event()

    def publish(self, *, runtime: Any | None = None, error: str = "") -> None:
        if (runtime is None) == (not str(error)):
            raise ValueError("bootstrap outcome must contain exactly one result")
        with self._lock:
            if self._outcome is not None:
                raise RuntimeError("bootstrap outcome was already published")
            self._outcome = BootstrapOutcome(runtime=runtime, error=str(error))

    def consume(self) -> BootstrapOutcome:
        with self._lock:
            if self._outcome is None:
                raise RuntimeError("bootstrap outcome is not ready")
            if self._consumed:
                raise RuntimeError("bootstrap outcome was already consumed")
            self._consumed = True
            return self._outcome

    def acknowledge(self) -> None:
        with self._lock:
            if not self._consumed:
                raise RuntimeError("bootstrap outcome must be consumed before ack")
        self._acknowledged.set()

    def acknowledged(self) -> bool:
        return self._acknowledged.is_set()


def bootstrap_is_quiescent(
    thread: threading.Thread | None, attempt: BootstrapAttempt | None
) -> bool:
    """Return true only after the producer and queued result are both done."""

    thread_live = bool(thread is not None and thread.is_alive())
    result_acknowledged = attempt is None or attempt.acknowledged()
    return not thread_live and result_acknowledged


class QtCloseSignalBridge:
    """Turn POSIX termination signals into GUI-thread close requests."""

    def __init__(self, signal_api: Any = signal) -> None:
        self._signal_api = signal_api
        self._previous: dict[Any, Any] = {}
        self._installed = False
        # Python signal handlers run on the main thread and may interrupt
        # arbitrary Python code.  A plain boolean avoids taking the lock hidden
        # inside threading.Event from a re-entrant signal handler.
        self._requested = False

    def install(self) -> "QtCloseSignalBridge":
        if self._installed:
            return self
        for sent_signal in (self._signal_api.SIGINT, self._signal_api.SIGTERM):
            self._previous[sent_signal] = self._signal_api.getsignal(sent_signal)
            self._signal_api.signal(sent_signal, self._handle)
        self._installed = True
        return self

    def _handle(self, _signum: int, _frame: Any) -> None:
        # Python's signal handler only flips a main-thread bit.  The existing
        # 5 Hz Qt timer consumes it and calls QWidget.close() on the GUI thread.
        self._requested = True

    def consume_requested(self) -> bool:
        if not self._requested:
            return False
        self._requested = False
        return True

    def restore(self) -> None:
        if not self._installed:
            return
        for sent_signal, previous in self._previous.items():
            if self._signal_api.getsignal(sent_signal) == self._handle:
                self._signal_api.signal(sent_signal, previous)
        self._installed = False


def install_qt_close_signal_handlers(
    QtCore: Any, window: Any, *, signal_api: Any = signal
) -> QtCloseSignalBridge:
    """Install signal handlers consumed by the window's Qt timer."""

    del QtCore  # Kept in the API to make the GUI-thread boundary explicit.
    bridge = QtCloseSignalBridge(signal_api=signal_api).install()
    window._qt_close_signal_bridge = bridge
    return bridge


def retain_qt_window(
    window: Any, *, on_release: Callable[[], None] | None = None
) -> Callable[[], None]:
    """Keep a top-level window alive when called under an existing QApplication."""

    window_id = id(window)
    _ACTIVE_WINDOWS[window_id] = window
    released = False

    def release(*_args: Any) -> None:
        nonlocal released
        if released:
            return
        released = True
        _ACTIVE_WINDOWS.pop(window_id, None)
        if on_release is not None:
            on_release()

    window.destroyed.connect(release)
    return release


def vertical_axis_labels(jog_frame: str, environment_id: str) -> VerticalAxisLabels:
    if jog_frame == "base":
        return VerticalAxisLabels("R  +Z", "F  −Z", "base: R = +Z；F = −Z")
    if jog_frame != "plane1":
        raise ValueError("jog frame must be base or plane1")
    explanation = (
        "MLA: R = −Z 物理抬高；F = +Z 靠近传感器板"
        if environment_id == "mla"
        else "R/F 严格表示 plane1 −Z/+Z；该方向不一定是物理上下"
    )
    return VerticalAxisLabels("R  −Z", "F  +Z", explanation)


def validate_environment_id(value: str) -> str:
    environment_id = str(value).strip()
    if not _ENVIRONMENT_ID.fullmatch(environment_id):
        raise ValueError(
            "environment name may contain only letters, numbers, '_' and '-'"
        )
    return environment_id


def validate_robot_ip(value: str) -> str:
    text = str(value).strip()
    try:
        address = ipaddress.ip_address(text)
    except ValueError as error:
        raise ValueError("robot IP must be a literal IPv4 address") from error
    if address.version != 4:
        raise ValueError("robot IP must be a literal IPv4 address")
    return str(address)


def discover_environment_ids(workspace: Path | str) -> tuple[str, ...]:
    root = Path(workspace).expanduser().resolve() / "environments"
    if not root.is_dir():
        return ()
    return tuple(
        sorted(
            item.name
            for item in root.iterdir()
            if item.is_dir()
            and _ENVIRONMENT_ID.fullmatch(item.name)
            and (item / "setup.yaml").is_file()
        )
    )


def build_real_launch_command(spec: LaunchSpec) -> tuple[str, ...]:
    workspace = Path(spec.workspace).expanduser().resolve()
    environment_id = validate_environment_id(spec.environment_id)
    robot_ip = validate_robot_ip(spec.robot_ip)
    return (
        "ros2",
        "launch",
        "hx_waypoint_bringup",
        "real.launch.py",
        f"workspace:={workspace}",
        f"environment_id:={environment_id}",
        f"robot_ip:={robot_ip}",
        f"launch_rviz:={'true' if spec.launch_rviz else 'false'}",
        "manual_control:=true",
    )


def parse_gui_absolute_pose(frame: str, text: str):
    value = str(text).strip()
    if not value:
        raise ValueError("enter p[x,y,z,rx,ry,rz]")
    first = value.split(None, 1)[0].lower()
    explicit = first in {"base", "plane1"}
    return parse_absolute_pose(value if explicit else f"{frame} {value}")


def parse_gui_absolute_position(
    frame: str,
    x_text: str,
    y_text: str,
    z_text: str,
) -> AbsolutePositionCommand:
    """Parse the GUI's three plain XYZ fields; values are metres."""

    normalized_frame = str(frame).strip().lower()
    if normalized_frame not in {"base", "plane1"}:
        raise ValueError("coordinate frame must be base or plane1")
    fields = (str(x_text).strip(), str(y_text).strip(), str(z_text).strip())
    if any(not value for value in fields):
        raise ValueError("X, Y and Z are all required")
    try:
        position = tuple(float(value) for value in fields)
    except ValueError as error:
        raise ValueError("X, Y and Z must be numbers in metres") from error
    if not all(math.isfinite(value) for value in position):
        raise ValueError("X, Y and Z must be finite")
    return AbsolutePositionCommand(
        frame=normalized_frame,
        position_m=position,  # type: ignore[arg-type]
    )


def safe_to_teardown_ros(worker_snapshot: Any, session_unresolved: bool) -> bool:
    return not bool(worker_snapshot.running) and not bool(session_unresolved)


def keyboard_jog_command(
    key_text: str,
    *,
    motion_enabled: bool,
    text_input_focused: bool,
    auto_repeat: bool,
    motion_busy: bool,
) -> str | None:
    key = str(key_text).lower()
    if (
        key not in _KEYBOARD_JOG
        or not motion_enabled
        or text_input_focused
        or auto_repeat
        or motion_busy
    ):
        return None
    return key


def process_group_exists(pgid: int) -> bool:
    """Return whether the GUI-owned POSIX process group still has members."""

    try:
        os.killpg(int(pgid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def find_existing_control_graph(*, timeout_s: float = 2.0) -> tuple[str, ...]:
    """Fail-closed probe for a control stack that this GUI does not own."""

    probes = (
        (
            "controller_manager",
            ("ros2", "service", "info", "/controller_manager/list_controllers"),
            re.compile(r"Services count:\s*([1-9][0-9]*)"),
        ),
        (
            "authoritative playback runtime",
            ("ros2", "action", "info", "/hx_waypoint/play_recording"),
            re.compile(r"Action servers:\s*([1-9][0-9]*)"),
        ),
    )
    active: list[str] = []
    for label, command, server_pattern in probes:
        try:
            result = subprocess.run(
                command,
                stdin=subprocess.DEVNULL,
                capture_output=True,
                text=True,
                shell=False,
                timeout=max(0.1, float(timeout_s)),
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RuntimeError(
                f"could not prove the ROS graph is free before launch: {error}"
            ) from error
        output = f"{result.stdout}\n{result.stderr}"
        if server_pattern.search(output):
            active.append(label)
    return tuple(active)


class ManagedLaunchProcess:
    """Own one ros2 launch process and its complete independent process group."""

    def __init__(self, log_callback: Callable[[str], None] | None = None) -> None:
        self._log_callback = log_callback or (lambda _line: None)
        self._process: Any | None = None
        self._pgid: int | None = None
        self._reader_thread: threading.Thread | None = None
        self._lock = threading.Lock()

    @property
    def pid(self) -> int | None:
        with self._lock:
            return None if self._process is None else int(self._process.pid)

    def is_running(self) -> bool:
        with self._lock:
            process = self._process
            pgid = self._pgid
        if process is not None:
            # poll() reaps an exited launch leader.  Otherwise a leader-only
            # zombie can make killpg(pgid, 0) look like a live process group.
            process.poll()
        return pgid is not None and process_group_exists(pgid)

    def returncode(self) -> int | None:
        with self._lock:
            process = self._process
        return None if process is None else process.poll()

    def start(self, command: Sequence[str], *, cwd: Path | str) -> int:
        arguments = [str(item) for item in command]
        if not arguments:
            raise ValueError("launch command is empty")
        with self._lock:
            if self._process is not None:
                self._process.poll()
            if self._pgid is not None and process_group_exists(self._pgid):
                raise RuntimeError("a launch process is already running")
            process = subprocess.Popen(
                arguments,
                cwd=str(Path(cwd).expanduser().resolve()),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                shell=False,
                start_new_session=True,
            )
            self._process = process
            self._pgid = int(process.pid)
            reader = threading.Thread(
                target=self._read_output,
                args=(process,),
                name="tcp-gui-launch-log",
                daemon=True,
            )
            self._reader_thread = reader
            reader.start()
            return int(process.pid)

    def _read_output(self, process: Any) -> None:
        stream = process.stdout
        if stream is None:
            return
        try:
            for raw_line in stream:
                line = str(raw_line).rstrip("\r\n")
                if line:
                    self._log_callback(line)
        finally:
            try:
                stream.close()
            except Exception:
                pass

    def join_log_reader(self, timeout_s: float) -> None:
        with self._lock:
            reader = self._reader_thread
        if reader is not None:
            reader.join(max(0.0, float(timeout_s)))

    def stop(self, timeout_s: float = 8.0) -> int | None:
        with self._lock:
            process = self._process
            pgid = self._pgid
        if process is None:
            return None
        if pgid is None:
            return None

        escalation = (
            (signal.SIGINT, max(0.1, float(timeout_s)), "SIGINT"),
            (signal.SIGTERM, 3.0, "SIGTERM"),
            (signal.SIGKILL, 2.0, "SIGKILL"),
        )

        def group_alive_after_reap() -> bool:
            process.poll()
            return process_group_exists(pgid)

        for index, (sent_signal, wait_s, name) in enumerate(escalation):
            if not group_alive_after_reap():
                break
            if index:
                self._log_callback(
                    f"launch process group is still alive; sending {name}"
                )
            try:
                os.killpg(pgid, sent_signal)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + wait_s
            while group_alive_after_reap() and time.monotonic() < deadline:
                time.sleep(0.05)
        if group_alive_after_reap():
            self._log_callback(
                "launch process group still exists after SIGKILL; stop is unconfirmed"
            )
            return None
        try:
            code = process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            code = process.poll()
        if code is None:
            self._log_callback(
                "launch group is gone but its leader exit status is unconfirmed"
            )
            return None
        self.join_log_reader(1.0)
        return int(code)


class StatusPoller:
    """One background snapshot loop; slow TF reads can never pile up."""

    def __init__(
        self,
        session: Any,
        snapshot_callback: Callable[[str], None],
        error_callback: Callable[[str], None],
        *,
        period_s: float = GUI_REFRESH_PERIOD_S,
    ) -> None:
        self._session = session
        self._snapshot_callback = snapshot_callback
        self._error_callback = error_callback
        self._period_s = float(period_s)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._run,
            name="tcp-gui-status-5hz",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                self._snapshot_callback(self._session.snapshot_text())
            except Exception as error:
                self._error_callback(str(error))
            remaining = self._period_s - (time.monotonic() - started)
            self._stop_event.wait(max(0.0, remaining))

    def stop(self) -> None:
        self._stop_event.set()

    def join(self, timeout_s: float | None = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout_s)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()


@dataclass
class RosSessionRuntime:
    session: Any
    rclpy: Any
    context: Any


def wait_until_manual_session_ready(
    session: Any,
    *,
    cancel_event: threading.Event | None = None,
    timeout_s: float = 45.0,
) -> None:
    """Wait for every non-moving manual-control gate, with bounded retries."""

    deadline = time.monotonic() + max(0.1, float(timeout_s))
    last_error = "manual-control services have not appeared"
    while time.monotonic() < deadline:
        if cancel_event is not None and cancel_event.is_set():
            raise RuntimeError("manual stack startup was cancelled")
        try:
            session.verify_manual_readiness(joint_state_timeout_s=1.0)
            return
        except Exception as error:
            last_error = str(error)
            if "authoritative runtime" in last_error:
                raise RuntimeError(last_error) from error
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        if cancel_event is None:
            time.sleep(min(0.25, remaining))
        elif cancel_event.wait(min(0.25, remaining)):
            raise RuntimeError("manual stack startup was cancelled")
    raise RuntimeError(
        f"manual stack did not become ready within {float(timeout_s):.1f} s; "
        f"last gate: {last_error}"
    )


def _open_ros_session(
    workspace: Path,
    environment_id: str,
    *,
    max_absolute_translation_mm: float = 150.0,
    cancel_event: threading.Event | None = None,
) -> RosSessionRuntime:
    """Load one authoritative environment and initialize ROS in this process."""

    scene = load_planning_scene_definition(workspace, environment_id)
    environment = EnvironmentBuilder().load(workspace / "environments" / environment_id)
    if scene.environment_id != environment_id or environment.environment_id != environment_id:
        raise RuntimeError("selected environment identity does not match its bundle")

    parser = build_console_argument_parser()
    args = parser.parse_args(
        [
            "--workspace",
            str(workspace),
            "--env",
            environment_id,
            "--enable-motion",
            "--max-absolute-translation-mm",
            str(max_absolute_translation_mm),
        ]
    )
    _validate_console_args(args)

    import rclpy
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions

    context = Context()
    rclpy.init(
        args=None,
        context=context,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    session: RosTcpSession | None = None
    try:
        session = RosTcpSession(environment, scene, args, context=context)
        wait_until_manual_session_ready(session, cancel_event=cancel_event)
        return RosSessionRuntime(session=session, rclpy=rclpy, context=context)
    except BaseException:
        if session is not None and not session.unresolved_motion():
            session.close()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)
        raise


def _qt_stylesheet() -> str:
    return """
QMainWindow, QWidget { background: #0b1220; color: #dbeafe; }
QWidget { font-family: "IBM Plex Sans", "Noto Sans"; font-size: 13px; }
QFrame#Header { background: #101a2d; border-bottom: 1px solid #263755; }
QLabel#Title { color: #f8fafc; font-size: 21px; font-weight: 700; }
QLabel#Kicker { color: #60a5fa; font-size: 11px; font-weight: 700; }
QLabel#Section { color: #93c5fd; font-size: 11px; font-weight: 700; }
QLabel#Hint { color: #8292ab; font-size: 11px; }
QLabel#StatusGood { color: #6ee7b7; background: #102a2a; border: 1px solid #1f5b50;
                    border-radius: 10px; padding: 4px 9px; font-weight: 700; }
QLabel#StatusIdle { color: #a8b5ca; background: #172033; border: 1px solid #334155;
                    border-radius: 10px; padding: 4px 9px; font-weight: 700; }
QLabel#StatusWarn { color: #fbbf24; background: #2b2516; border: 1px solid #66531f;
                    border-radius: 10px; padding: 4px 9px; font-weight: 700; }
QFrame#Card { background: #111b2e; border: 1px solid #243653; border-radius: 8px; }
QLineEdit, QComboBox, QPlainTextEdit { background: #0c1526; color: #e6edf8;
    border: 1px solid #344765; border-radius: 5px; padding: 7px;
    selection-background-color: #2563eb; }
QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus { border: 1px solid #60a5fa; }
QPushButton { background: #1a2942; color: #dbeafe; border: 1px solid #355071;
    border-radius: 5px; padding: 8px 12px; font-weight: 650; }
QPushButton:hover { background: #213754; border-color: #4d7198; }
QPushButton:pressed { background: #14243a; }
QPushButton:disabled { color: #55657c; background: #111a29; border-color: #243044; }
QPushButton#Primary { background: #2563eb; color: white; border-color: #3b82f6; }
QPushButton#Primary:hover { background: #2f72e5; }
QPushButton#Danger { background: #311a24; color: #fca5a5; border-color: #6b293b; }
QPushButton#Jog { min-width: 72px; min-height: 34px; font-family: "JetBrains Mono", monospace; }
QCheckBox { color: #cbd5e1; spacing: 8px; }
QCheckBox::indicator { width: 34px; height: 18px; }
QCheckBox::indicator:unchecked { background: #25344a; border: 1px solid #4a5f7c;
    border-radius: 9px; }
QCheckBox::indicator:checked { background: #2563eb; border: 1px solid #60a5fa; border-radius: 9px; }
QSplitter::handle { background: #243653; width: 1px; }
QScrollBar:vertical { background: #0c1526; width: 10px; }
QScrollBar::handle:vertical { background: #344765; min-height: 22px; border-radius: 4px; }
"""


def _make_window_class(QtCore: Any, QtGui: Any, QtWidgets: Any):
    Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal

    class Bridge(QtCore.QObject):
        log_message = Signal(str)
        session_ready = Signal(int, object)
        session_failed = Signal(int, object)
        snapshot_ready = Signal(int, str)
        snapshot_failed = Signal(int, str)
        teardown_done = Signal(int, str)

    class TcpControlWindow(QtWidgets.QMainWindow):
        def __init__(self, args: argparse.Namespace) -> None:
            super().__init__()
            self.args = args
            self.workspace = Path(args.workspace).expanduser().resolve()
            self.bridge = Bridge()
            self.bridge.log_message.connect(self._append_log)
            self.bridge.session_ready.connect(self._on_session_ready)
            self.bridge.session_failed.connect(self._on_session_failed)
            self.bridge.snapshot_ready.connect(self._on_snapshot)
            self.bridge.snapshot_failed.connect(self._on_snapshot_error)
            self.bridge.teardown_done.connect(self._on_teardown_done)
            self.launch: ManagedLaunchProcess | None = None
            self.runtime: RosSessionRuntime | None = None
            self.poller: StatusPoller | None = None
            self.worker = OneMotionWorker()
            self.jog_hold = JogHoldState()
            self._held_jog_frame: str | None = None
            self._held_jog_speed_m_s: float | None = None
            self._held_servo_generation: int | None = None
            self._generation = 0
            self._bootstrap_thread: threading.Thread | None = None
            self._bootstrap_cancel = threading.Event()
            self._bootstrap_attempt: BootstrapAttempt | None = None
            self._teardown_active = False
            self._pending_stop = False
            self._stop_failed = False
            self._close_requested = False
            self._reported_motion_generation = 0
            self._last_launch_running = False
            self._build_ui()
            self.setStyleSheet(_qt_stylesheet())
            self.setWindowTitle("HX · Manual TCP Control")
            self.resize(1180, 780)
            # The top-level window is the deliberate keyboard-jog target.
            # Absolute XYZ editors keep normal text focus while being edited;
            # after one accepted absolute move we explicitly hand focus back
            # here so WASDRF no longer remains trapped in the last QLineEdit.
            self.setFocusPolicy(QtCore.Qt.StrongFocus)
            # Child controls such as a non-editable combo can otherwise consume
            # letter keys before QMainWindow sees them.  Filtering every widget
            # gives WASDRF one consistent path while the focus gate below keeps
            # all text-entry controls completely isolated from jog commands.
            for child in self.findChildren(QtWidgets.QWidget):
                child.installEventFilter(self)
            self.installEventFilter(self)

            self.timer = QtCore.QTimer(self)
            self.timer.setInterval(int(1000.0 / GUI_REFRESH_HZ))
            self.timer.timeout.connect(self._tick)
            self.timer.start()
            self.motion_timer = QtCore.QTimer(self)
            self.motion_timer.setInterval(int(1000.0 / MOTION_MONITOR_HZ))
            self.motion_timer.timeout.connect(self._consume_motion_terminal)
            self.motion_timer.start()
            # Servo refresh belongs to the GUI thread.  If Qt stalls or the
            # process disappears, MoveIt Servo's 100 ms input watchdog stops
            # the arm instead of letting a background publisher run on.
            self.servo_timer = QtCore.QTimer(self)
            self.servo_timer.setInterval(int(1000.0 / SERVO_COMMAND_HZ))
            self.servo_timer.timeout.connect(self._publish_held_jog)
            self.servo_timer.start()
            self._update_controls()

        def _card(self, title: str) -> tuple[Any, Any]:
            frame = QtWidgets.QFrame()
            frame.setObjectName("Card")
            layout = QtWidgets.QVBoxLayout(frame)
            layout.setContentsMargins(14, 12, 14, 14)
            layout.setSpacing(9)
            label = QtWidgets.QLabel(title.upper())
            label.setObjectName("Section")
            layout.addWidget(label)
            return frame, layout

        def _build_ui(self) -> None:
            root = QtWidgets.QWidget()
            outer = QtWidgets.QVBoxLayout(root)
            outer.setContentsMargins(0, 0, 0, 0)
            outer.setSpacing(0)

            header = QtWidgets.QFrame()
            header.setObjectName("Header")
            header_layout = QtWidgets.QHBoxLayout(header)
            header_layout.setContentsMargins(20, 13, 20, 13)
            title_box = QtWidgets.QVBoxLayout()
            kicker = QtWidgets.QLabel("ENVIRONMENT-BOUND UR5 CONTROL")
            kicker.setObjectName("Kicker")
            title = QtWidgets.QLabel("Live rod_tip console")
            title.setObjectName("Title")
            title_box.addWidget(kicker)
            title_box.addWidget(title)
            header_layout.addLayout(title_box)
            header_layout.addStretch(1)
            self.launch_status = QtWidgets.QLabel("LAUNCH OFFLINE")
            self.launch_status.setObjectName("StatusIdle")
            self.ros_status = QtWidgets.QLabel("ROS OFFLINE")
            self.ros_status.setObjectName("StatusIdle")
            self.motion_status = QtWidgets.QLabel("MOTION DISABLED")
            self.motion_status.setObjectName("StatusIdle")
            header_layout.addWidget(self.launch_status)
            header_layout.addWidget(self.ros_status)
            header_layout.addWidget(self.motion_status)
            outer.addWidget(header)

            body = QtWidgets.QWidget()
            body_layout = QtWidgets.QHBoxLayout(body)
            body_layout.setContentsMargins(14, 14, 14, 14)
            body_layout.setSpacing(12)

            left = QtWidgets.QWidget()
            left.setFixedWidth(386)
            left_layout = QtWidgets.QVBoxLayout(left)
            left_layout.setContentsMargins(0, 0, 0, 0)
            left_layout.setSpacing(10)

            connection, connection_layout = self._card("Connection")
            form = QtWidgets.QFormLayout()
            form.setSpacing(8)
            self.environment_combo = QtWidgets.QComboBox()
            environments = discover_environment_ids(self.workspace)
            self.environment_combo.addItems(environments)
            preferred = (
                self.args.env
                if self.args.env in environments
                else DEFAULT_ENVIRONMENT
            )
            index = self.environment_combo.findText(preferred)
            if index >= 0:
                self.environment_combo.setCurrentIndex(index)
            self.environment_combo.currentTextChanged.connect(
                self._update_axis_labels
            )
            self.robot_ip_edit = QtWidgets.QLineEdit(self.args.robot_ip)
            self.robot_ip_edit.setPlaceholderText(DEFAULT_ROBOT_IP)
            self.rviz_check = QtWidgets.QCheckBox("Open RViz")
            self.rviz_check.setChecked(not self.args.no_rviz)
            form.addRow("Environment", self.environment_combo)
            form.addRow("Robot IP", self.robot_ip_edit)
            form.addRow("Visualization", self.rviz_check)
            connection_layout.addLayout(form)
            self.launch_button = QtWidgets.QPushButton("START MANUAL STACK")
            self.launch_button.setObjectName("Primary")
            self.launch_button.clicked.connect(self._launch_clicked)
            connection_layout.addWidget(self.launch_button)
            left_layout.addWidget(connection)

            safety, safety_layout = self._card("Motion authority")
            self.enable_motion = QtWidgets.QCheckBox("启用运动（持续有效）")
            self.enable_motion.setChecked(False)
            self.enable_motion.toggled.connect(self._motion_toggle_changed)
            safety_layout.addWidget(self.enable_motion)
            safety_hint = QtWidgets.QLabel(
                "开启后连续有效，不再需要按 Space。关闭会取消当前动作；"
                "运动中不排队。"
            )
            safety_hint.setWordWrap(True)
            safety_hint.setObjectName("Hint")
            safety_layout.addWidget(safety_hint)
            row = QtWidgets.QHBoxLayout()
            self.cancel_button = QtWidgets.QPushButton("CANCEL ACTIVE MOTION")
            self.cancel_button.setObjectName("Danger")
            self.cancel_button.clicked.connect(self._cancel_motion)
            row.addWidget(self.cancel_button)
            safety_layout.addLayout(row)
            left_layout.addWidget(safety)

            jog, jog_layout = self._card("Jog rod_tip")
            selectors = QtWidgets.QHBoxLayout()
            self.jog_frame_combo = QtWidgets.QComboBox()
            self.jog_frame_combo.addItems(("plane1", "base"))
            self.jog_frame_combo.currentTextChanged.connect(
                self._update_axis_labels
            )
            self.speed_combo = QtWidgets.QComboBox()
            self.speed_combo.addItems(("5 mm/s", "10 mm/s", "20 mm/s"))
            self.speed_combo.setCurrentIndex(1)
            selectors.addWidget(QtWidgets.QLabel("Frame"))
            selectors.addWidget(self.jog_frame_combo, 1)
            selectors.addWidget(QtWidgets.QLabel("Speed"))
            selectors.addWidget(self.speed_combo, 1)
            jog_layout.addLayout(selectors)

            grid = QtWidgets.QGridLayout()
            grid.setHorizontalSpacing(7)
            grid.setVerticalSpacing(7)
            definitions = (
                ("W  +Y", "w", 0, 1),
                ("A  −X", "a", 1, 0),
                ("S  −Y", "s", 1, 1),
                ("D  +X", "d", 1, 2),
                ("R  RAISE", "r", 2, 0),
                ("F  LOWER", "f", 2, 2),
            )
            self.jog_buttons: list[Any] = []
            self.jog_button_by_key: dict[str, Any] = {}
            for text, key, row_index, column in definitions:
                button = QtWidgets.QPushButton(text)
                button.setObjectName("Jog")
                # A clicked motion button must not retain keyboard focus: Qt's
                # default Space handling would otherwise repeat the last jog.
                button.setFocusPolicy(QtCore.Qt.NoFocus)
                button.pressed.connect(
                    lambda k=key: self._begin_jog_hold(f"button:{k}", k)
                )
                button.released.connect(
                    lambda k=key: self._release_jog_hold(f"button:{k}")
                )
                grid.addWidget(button, row_index, column)
                self.jog_buttons.append(button)
                self.jog_button_by_key[key] = button
            jog_layout.addLayout(grid)
            self.jog_hint = QtWidgets.QLabel()
            self.jog_hint.setWordWrap(True)
            self.jog_hint.setObjectName("Hint")
            jog_layout.addWidget(self.jog_hint)
            left_layout.addWidget(jog)

            absolute, absolute_layout = self._card("Absolute rod_tip target")
            frame_row = QtWidgets.QHBoxLayout()
            self.absolute_frame_combo = QtWidgets.QComboBox()
            self.absolute_frame_combo.addItems(("plane1", "base"))
            frame_row.addWidget(QtWidgets.QLabel("Coordinate frame"))
            frame_row.addWidget(self.absolute_frame_combo, 1)
            absolute_layout.addLayout(frame_row)

            xyz_row = QtWidgets.QHBoxLayout()
            xyz_row.setSpacing(7)
            self.absolute_xyz_edits: dict[str, Any] = {}
            for axis in ("X", "Y", "Z"):
                field_box = QtWidgets.QVBoxLayout()
                axis_label = QtWidgets.QLabel(f"{axis} · m")
                axis_label.setObjectName("Hint")
                edit = QtWidgets.QLineEdit()
                edit.setPlaceholderText("0.000000")
                edit.setAlignment(QtCore.Qt.AlignRight)
                field_box.addWidget(axis_label)
                field_box.addWidget(edit)
                xyz_row.addLayout(field_box, 1)
                self.absolute_xyz_edits[axis.lower()] = edit
            absolute_layout.addLayout(xyz_row)
            orientation_hint = QtWidgets.QLabel(
                "只输入杆尖 X / Y / Z；执行时自动保持当下姿态。"
            )
            orientation_hint.setWordWrap(True)
            orientation_hint.setObjectName("Hint")
            absolute_layout.addWidget(orientation_hint)
            self.absolute_button = QtWidgets.QPushButton(
                "MOVE TO XYZ · KEEP CURRENT ORIENTATION"
            )
            self.absolute_button.setFocusPolicy(QtCore.Qt.NoFocus)
            self.absolute_button.clicked.connect(self._dispatch_absolute)
            absolute_layout.addWidget(self.absolute_button)
            left_layout.addWidget(absolute)
            left_layout.addStretch(1)

            right_split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
            telemetry, telemetry_layout = self._card(
                f"Live TCP + joints · {GUI_REFRESH_HZ:.0f} Hz"
            )
            self.telemetry = QtWidgets.QPlainTextEdit()
            self.telemetry.setReadOnly(True)
            self.telemetry.setFocusPolicy(QtCore.Qt.NoFocus)
            self.telemetry.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
            fixed_font = QtGui.QFont("JetBrains Mono")
            fixed_font.setStyleHint(QtGui.QFont.Monospace)
            self.telemetry.setFont(fixed_font)
            self.telemetry.setPlainText("Start the manual stack to receive live state.")
            telemetry_layout.addWidget(self.telemetry)
            right_split.addWidget(telemetry)

            logs, logs_layout = self._card("Operation log")
            self.log = QtWidgets.QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setFocusPolicy(QtCore.Qt.NoFocus)
            self.log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
            self.log.setFont(fixed_font)
            logs_layout.addWidget(self.log)
            right_split.addWidget(logs)
            right_split.setSizes((470, 220))

            body_layout.addWidget(left)
            body_layout.addWidget(right_split, 1)
            outer.addWidget(body, 1)
            self.setCentralWidget(root)
            self._update_axis_labels()

        def _update_axis_labels(self, _value: str = "") -> None:
            if not hasattr(self, "jog_button_by_key"):
                return
            frame = self.jog_frame_combo.currentText()
            environment_id = self.environment_combo.currentText()
            labels = vertical_axis_labels(frame, environment_id)
            self.jog_button_by_key["r"].setText(labels.r_button)
            self.jog_button_by_key["f"].setText(labels.f_button)
            self.jog_hint.setText(
                "KEYBOARD  W/A/S/D = ±Y/±X   ·   "
                + labels.explanation
                + "\n按下立即移动，按住持续移动，松开立即停止；"
                "全程由 MoveIt Servo 检查碰撞。"
                "输入框聚焦时键盘点动停用。"
            )

        def _append_log(self, message: str) -> None:
            clean = str(message).strip()
            if not clean:
                return
            timestamp = time.strftime("%H:%M:%S")
            self.log.appendPlainText(f"{timestamp}  {clean}")
            bar = self.log.verticalScrollBar()
            bar.setValue(bar.maximum())

        def _set_pill(self, label: Any, text: str, state: str) -> None:
            label.setText(text)
            object_name = (
                "StatusGood"
                if state == "good"
                else "StatusWarn"
                if state == "warn"
                else "StatusIdle"
            )
            label.setObjectName(object_name)
            label.style().unpolish(label)
            label.style().polish(label)

        def _launch_clicked(self) -> None:
            if self.launch is not None or self.runtime is not None:
                self._request_stop()
            else:
                self._start_launch()

        def _start_launch(self) -> None:
            if self._teardown_active or (
                self._bootstrap_thread is not None and self._bootstrap_thread.is_alive()
            ) or (
                self._bootstrap_attempt is not None
                and not self._bootstrap_attempt.acknowledged()
            ):
                self._append_log("start ignored while the previous stack is closing")
                return
            try:
                environment_id = validate_environment_id(
                    self.environment_combo.currentText()
                )
                robot_ip = validate_robot_ip(self.robot_ip_edit.text())
                existing = find_existing_control_graph()
                if existing:
                    raise RuntimeError(
                        "existing ROS control stack detected: "
                        + ", ".join(existing)
                        + "; stop it before starting manual control"
                    )
                if not (self.workspace / "environments" / environment_id / "setup.yaml").is_file():
                    raise ValueError(f"environment bundle {environment_id!r} is missing")
                spec = LaunchSpec(
                    self.workspace,
                    environment_id,
                    robot_ip,
                    self.rviz_check.isChecked(),
                )
                command = build_real_launch_command(spec)
            except (OSError, RuntimeError, ValueError) as error:
                self._append_log(f"START REFUSED · {error}")
                return

            self._generation += 1
            generation = self._generation
            self._bootstrap_cancel = threading.Event()
            self.launch = ManagedLaunchProcess(self.bridge.log_message.emit)
            try:
                pid = self.launch.start(command, cwd=self.workspace)
            except Exception as error:
                self._append_log(f"LAUNCH FAILED · {error}")
                self.launch = None
                return
            attempt = BootstrapAttempt(generation, self._bootstrap_cancel)
            self._bootstrap_attempt = attempt
            self._pending_stop = False
            self._stop_failed = False
            self._last_launch_running = True
            self.environment_combo.setEnabled(False)
            self.robot_ip_edit.setEnabled(False)
            self.rviz_check.setEnabled(False)
            self.launch_button.setText("STOP MANUAL STACK")
            self._append_log(
                f"manual stack PID {pid} · env={environment_id} · robot={robot_ip}"
            )
            self._set_pill(self.launch_status, "LAUNCH STARTING", "warn")

            def bootstrap() -> None:
                try:
                    runtime = _open_ros_session(
                        self.workspace,
                        environment_id,
                        max_absolute_translation_mm=(
                            self.args.max_absolute_translation_mm
                        ),
                        cancel_event=self._bootstrap_cancel,
                    )
                except BaseException as error:
                    attempt.publish(error=str(error) or type(error).__name__)
                    self.bridge.session_failed.emit(generation, attempt)
                    return
                attempt.publish(runtime=runtime)
                self.bridge.session_ready.emit(generation, attempt)

            self._bootstrap_thread = threading.Thread(
                target=bootstrap,
                name="tcp-gui-ros-bootstrap",
                daemon=True,
            )
            self._bootstrap_thread.start()
            self._update_controls()

        def _on_session_ready(self, generation: int, attempt: BootstrapAttempt) -> None:
            try:
                outcome = attempt.consume()
            except RuntimeError as error:
                self._append_log(f"BOOTSTRAP RESULT ERROR · {error}")
                return
            runtime = outcome.runtime
            if runtime is None:
                self._append_log("BOOTSTRAP RESULT ERROR · ready result has no runtime")
                attempt.acknowledge()
                return
            launch_live = bool(self.launch is not None and self.launch.is_running())
            if generation != self._generation or attempt is not self._bootstrap_attempt:
                self._append_log("stale bootstrap generation refused")
                self._close_stale_runtime(runtime)
                attempt.acknowledge()
                return
            self.runtime = runtime
            attempt.acknowledge()
            if self._pending_stop or self._teardown_active or not launch_live:
                self._pending_stop = True
                self._append_log("ROS session became ready after stop; folding it into teardown")
                self._begin_teardown_if_safe()
                return
            self.poller = StatusPoller(
                runtime.session,
                lambda value: self.bridge.snapshot_ready.emit(generation, value),
                lambda message: self.bridge.snapshot_failed.emit(
                    generation, message
                ),
            )
            self.poller.start()
            self._append_log("ROS session ready · live rod_tip state connected")
            self._set_pill(self.ros_status, "ROS READY", "good")
            self._update_controls()

        @staticmethod
        def _close_stale_runtime(runtime: Any) -> None:
            if not runtime.session.unresolved_motion():
                runtime.session.close()
                if runtime.rclpy.ok(context=runtime.context):
                    runtime.rclpy.shutdown(context=runtime.context)

        def _on_session_failed(
            self, generation: int, attempt: BootstrapAttempt
        ) -> None:
            try:
                outcome = attempt.consume()
            except RuntimeError as error:
                self._append_log(f"BOOTSTRAP RESULT ERROR · {error}")
                return
            attempt.acknowledge()
            if generation != self._generation or attempt is not self._bootstrap_attempt:
                return
            message = outcome.error or "unknown bootstrap failure"
            self._append_log(f"ROS SESSION FAILED · {message}")
            self._set_pill(self.ros_status, "ROS ERROR", "warn")
            self.enable_motion.setChecked(False)
            self._pending_stop = True
            self._bootstrap_cancel.set()
            # A failed bootstrap has no useful control session.  Keeping its
            # GUI-owned driver/MoveIt launch would only leave another stale
            # stack and may conflict with the formal playback runtime.
            self._begin_teardown_if_safe()
            self._update_controls()

        def _on_snapshot(self, generation: int, value: str) -> None:
            if generation != self._generation or self._pending_stop:
                return
            self.telemetry.setPlainText(value)
            self._set_pill(self.ros_status, "ROS READY", "good")

        def _on_snapshot_error(self, generation: int, message: str) -> None:
            if generation != self._generation or self._pending_stop:
                return
            self._clear_jog_hold("live state unavailable")
            self.enable_motion.setChecked(False)
            self._set_pill(self.ros_status, "STATE WAIT", "warn")
            self.telemetry.setPlainText("Live state unavailable:\n" + message)

        def _motion_toggle_changed(self, enabled: bool) -> None:
            if not enabled:
                self._clear_jog_hold("motion authority disabled")
            self._append_log(
                "motion authority ENABLED"
                if enabled
                else "motion authority disabled"
            )
            if not enabled and self._motion_busy():
                self._cancel_motion()
            self._update_controls()

        def _motion_busy(self) -> bool:
            unresolved = bool(
                self.runtime is not None and self.runtime.session.unresolved_motion()
            )
            servo_active = bool(
                self.runtime is not None
                and self.runtime.session.servo_jog_active()
            )
            return self.worker.snapshot().running or unresolved or servo_active

        def _can_dispatch(self) -> bool:
            if not self.enable_motion.isChecked():
                self._append_log("command ignored · turn on 启用运动 first")
                return False
            if self.runtime is None:
                self._append_log("command ignored · ROS session is not ready")
                return False
            if self._pending_stop or self._motion_busy():
                self._append_log("command ignored · motion is active; no queue")
                return False
            return True

        def _speed_m_s(self) -> float:
            return (5.0, 10.0, 20.0)[self.speed_combo.currentIndex()] / 1000.0

        def _clear_jog_hold(self, reason: str = "") -> bool:
            # Clear the intent and generation before publishing zero.  A Qt
            # timer event already queued behind this call can therefore never
            # resurrect the old non-zero command.
            had_hold = self.jog_hold.clear()
            generation = self._held_servo_generation
            self._held_jog_frame = None
            self._held_jog_speed_m_s = None
            self._held_servo_generation = None
            stopped = False
            stop_error = ""
            if self.runtime is not None and (
                had_hold
                or generation is not None
                or self.runtime.session.servo_jog_active()
            ):
                try:
                    stopped = bool(
                        self.runtime.session.stop_servo_jog(generation)
                    )
                except BaseException as error:
                    stop_error = str(error) or type(error).__name__
            if reason and (had_hold or stopped or generation is not None):
                self._append_log(f"JOG HOLD RELEASED · {reason}")
            if stop_error:
                self._append_log(f"SERVO STOP ERROR · {stop_error}")
            self._update_controls()
            return had_hold or stopped

        def _begin_jog_hold(self, source: str, key: str) -> bool:
            if self.jog_hold.active:
                # Native keyboard repeat and duplicate mouse press events are
                # idempotent.  A second direction never replaces the one that
                # physically remains held.
                return self.jog_hold.press(source, key)
            if not self._can_dispatch():
                return False
            if not self.jog_hold.press(source, key):
                return False
            self._held_jog_frame = self.jog_frame_combo.currentText()
            self._held_jog_speed_m_s = self._speed_m_s()
            session = self.runtime.session
            try:
                # This synchronous call performs only bounded in-memory gates
                # and publishes the first non-zero Twist before pressed()
                # returns.  Slow IK/MoveGroup planning is never on the jog path.
                generation = session.begin_servo_jog(
                    key,
                    self._held_jog_frame,
                    self._held_jog_speed_m_s,
                )
            except BaseException as error:
                self._clear_jog_hold("servo start failed")
                self._append_log(
                    f"JOG REFUSED · {str(error) or type(error).__name__}"
                )
                return False
            self._held_servo_generation = int(generation)
            self._append_log(
                f"SERVO JOG · {self._held_jog_frame} · {key.upper()} · "
                f"{self._held_jog_speed_m_s * 1000.0:.0f} mm/s"
            )
            self._update_controls()
            return True

        def _release_jog_hold(self, source: str) -> None:
            if not self.jog_hold.active or self.jog_hold.source != str(source).strip():
                return
            self._clear_jog_hold("input released · zero velocity sent")

        def _publish_held_jog(self) -> None:
            generation = self._held_servo_generation
            if (
                generation is None
                or not self.jog_hold.active
                or self.runtime is None
            ):
                return
            try:
                refreshed = self.runtime.session.refresh_servo_jog(generation)
            except BaseException as error:
                self._clear_jog_hold("servo publish failed")
                self._append_log(
                    f"SERVO STOPPED · {str(error) or type(error).__name__}"
                )
                return
            if not refreshed:
                self._clear_jog_hold("servo generation no longer owns motion")

        def _dispatch_absolute(self) -> None:
            if not self._can_dispatch():
                return
            try:
                command = parse_gui_absolute_position(
                    self.absolute_frame_combo.currentText(),
                    self.absolute_xyz_edits["x"].text(),
                    self.absolute_xyz_edits["y"].text(),
                    self.absolute_xyz_edits["z"].text(),
                )
            except ValueError as error:
                self._append_log(f"ABSOLUTE REFUSED · {error}")
                return
            session = self.runtime.session

            def operation(cancel_event: threading.Event) -> MotionDelta:
                return session.move_absolute_position(
                    command,
                    cancel_event=cancel_event,
                )

            if self.worker.start(
                operation,
                label=f"absolute {command.frame} rod_tip XYZ",
            ):
                for edit in self.absolute_xyz_edits.values():
                    edit.clearFocus()
                self.setFocus(QtCore.Qt.OtherFocusReason)
                x, y, z = command.position_m
                self._append_log(
                    f"ABSOLUTE XYZ · {command.frame} · "
                    f"X={x:.6f} Y={y:.6f} Z={z:.6f} m · keep live orientation"
                )
            else:
                self._append_log("absolute target ignored · another motion is running")
            self._update_controls()

        def _cancel_motion(self) -> None:
            self._clear_jog_hold("operator cancellation")
            if self.runtime is None:
                return
            requested = self.worker.request_cancel(self.runtime.session.cancel_active)
            if not requested and self.runtime.session.unresolved_motion():
                threading.Thread(
                    target=self.runtime.session.cancel_active,
                    name="tcp-gui-cancel-unresolved",
                    daemon=True,
                ).start()
                requested = True
            self._append_log(
                "cancellation requested" if requested else "no active motion to cancel"
            )
            self._update_controls()

        def _request_stop(self) -> None:
            self._clear_jog_hold("manual stack stop")
            self.enable_motion.setChecked(False)
            self._pending_stop = True
            self._stop_failed = False
            self._bootstrap_cancel.set()
            self._cancel_motion()
            self._append_log("stop requested · waiting for a terminal motion state")
            self._begin_teardown_if_safe()
            self._update_controls()

        def _begin_teardown_if_safe(self) -> None:
            if self._teardown_active:
                return
            if not bootstrap_is_quiescent(
                self._bootstrap_thread, self._bootstrap_attempt
            ):
                return
            unresolved = bool(
                self.runtime is not None and self.runtime.session.unresolved_motion()
            )
            if not safe_to_teardown_ros(self.worker.snapshot(), unresolved):
                return
            if self.runtime is None and self.poller is None and self.launch is None:
                self._finish_pending_stop_if_quiescent()
                return
            self._teardown_active = True
            generation = self._generation
            runtime = self.runtime
            poller = self.poller
            launch = self.launch

            def teardown() -> None:
                errors: list[str] = []
                poller_alive = False
                if poller is not None:
                    try:
                        poller.stop()
                        poller.join(2.5)
                        poller_alive = poller.is_alive()
                    except BaseException as caught:
                        poller_alive = True
                        errors.append(f"status poller stop failed: {caught}")
                if launch is not None:
                    try:
                        code = launch.stop()
                        if code is None or launch.is_running():
                            errors.append(
                                "owned launch process group termination is unconfirmed"
                            )
                    except BaseException as caught:
                        errors.append(f"launch stop failed: {caught}")
                if poller is not None and poller_alive:
                    try:
                        # Driver shutdown normally releases a TF lookup that
                        # was still completing during the first bounded join.
                        poller.join(2.5)
                        poller_alive = poller.is_alive()
                    except BaseException as caught:
                        errors.append(f"status poller final join failed: {caught}")
                if poller_alive:
                    errors.append(
                        "status poller is still using the ROS node; ROS teardown refused"
                    )
                elif runtime is not None:
                    try:
                        runtime.session.close()
                        if runtime.rclpy.ok(context=runtime.context):
                            runtime.rclpy.shutdown(context=runtime.context)
                    except BaseException as caught:
                        errors.append(f"ROS session stop failed: {caught}")
                self.bridge.teardown_done.emit(generation, "; ".join(errors))

            threading.Thread(
                target=teardown,
                name="tcp-gui-safe-teardown",
                daemon=True,
            ).start()

        def _on_teardown_done(self, generation: int, error: str) -> None:
            if generation != self._generation:
                return
            self._teardown_active = False
            self.enable_motion.setChecked(False)
            if error:
                self._pending_stop = False
                self._stop_failed = True
                self._set_pill(self.launch_status, "STOP UNCONFIRMED", "warn")
                self._set_pill(self.ros_status, "TEARDOWN ERROR", "warn")
                self._append_log(f"STOP ERROR · {error}")
                self.launch_button.setText("RETRY SAFE STOP")
            else:
                self._stop_failed = False
                self.runtime = None
                self.poller = None
                self.launch = None
                self._set_pill(self.launch_status, "LAUNCH OFFLINE", "idle")
                self._set_pill(self.ros_status, "ROS OFFLINE", "idle")
                self._append_log("manual stack stopped")
                self.environment_combo.setEnabled(True)
                self.robot_ip_edit.setEnabled(True)
                self.rviz_check.setEnabled(True)
                self.launch_button.setText("START MANUAL STACK")
                self._last_launch_running = False
            self._update_controls()
            if not error:
                self._finish_pending_stop_if_quiescent()

        def _finish_pending_stop_if_quiescent(self) -> None:
            if (
                not self._pending_stop
                or self._teardown_active
                or self._stop_failed
                or self.runtime is not None
                or self.poller is not None
                or self.launch is not None
                or not bootstrap_is_quiescent(
                    self._bootstrap_thread, self._bootstrap_attempt
                )
            ):
                return
            self._pending_stop = False
            self._update_controls()
            if self._close_requested:
                QtCore.QTimer.singleShot(0, self.close)

        def _update_controls(self) -> None:
            launch_running = bool(self.launch is not None and self.launch.is_running())
            session_ready = self.runtime is not None
            busy = self._motion_busy()
            authority = self.enable_motion.isChecked()
            can_move = (
                launch_running
                and session_ready
                and authority
                and not busy
                and not self._pending_stop
                and not self._stop_failed
            )
            jog_available = (
                launch_running
                and session_ready
                and authority
                and not self._pending_stop
                and not self._stop_failed
            )
            self.enable_motion.setEnabled(
                session_ready and not self._pending_stop and not self._stop_failed
            )
            for key, button in self.jog_button_by_key.items():
                active_mouse_button = (
                    self.jog_hold.source == f"button:{key}"
                    and self.jog_hold.key == key
                )
                # A pressed mouse button must remain enabled until Qt delivers
                # released(); all other directions stay unavailable while a
                # hold or motion owns the single-command worker.
                button.setEnabled(
                    jog_available
                    and (
                        active_mouse_button
                        or (not busy and not self.jog_hold.active)
                    )
                )
            self.absolute_button.setEnabled(can_move and not self.jog_hold.active)
            self.cancel_button.setEnabled(session_ready and busy)
            controls_unlocked = not busy and not self.jog_hold.active
            self.jog_frame_combo.setEnabled(controls_unlocked)
            self.speed_combo.setEnabled(controls_unlocked)
            self.absolute_frame_combo.setEnabled(controls_unlocked)
            for edit in self.absolute_xyz_edits.values():
                edit.setEnabled(controls_unlocked)
            if busy:
                self._set_pill(self.motion_status, "MOTION ACTIVE", "warn")
            elif authority and session_ready:
                self._set_pill(self.motion_status, "MOTION ENABLED", "good")
            else:
                self._set_pill(self.motion_status, "MOTION DISABLED", "idle")
            if launch_running:
                self._set_pill(self.launch_status, "LAUNCH ONLINE", "good")

        def _tick(self) -> None:
            signal_bridge = getattr(self, "_qt_close_signal_bridge", None)
            if signal_bridge is not None and signal_bridge.consume_requested():
                self.close()
            launch_running = bool(self.launch is not None and self.launch.is_running())
            if self._last_launch_running and not launch_running and not self._teardown_active:
                code = None if self.launch is None else self.launch.returncode()
                self._append_log(f"launch exited unexpectedly · code={code}")
                self._request_stop()
            self._last_launch_running = launch_running

            self._consume_motion_terminal()
            if self._pending_stop:
                self._begin_teardown_if_safe()
            self._update_controls()
            bootstrap_quiescent = bootstrap_is_quiescent(
                self._bootstrap_thread, self._bootstrap_attempt
            )
            if (
                self._close_requested
                and self.runtime is None
                and self.launch is None
                and bootstrap_quiescent
                and not self._teardown_active
            ):
                self.close()

        def _consume_motion_terminal(self) -> None:
            state = self.worker.snapshot()
            if (
                not state.running
                and state.generation > 0
                and state.generation > self._reported_motion_generation
            ):
                self._reported_motion_generation = state.generation
                self._append_log(
                    f"MOTION FAILED · {state.error}" if state.error else state.message
                )
                if state.error:
                    self._clear_jog_hold("motion failed")
            self._update_controls()

        def _keyboard_command(self, event: Any) -> str | None:
            focus = self.focusWidget()
            text_input_focused = bool(
                isinstance(focus, QtWidgets.QLineEdit)
                or (
                    isinstance(
                        focus, (QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)
                    )
                    and not focus.isReadOnly()
                )
                or (isinstance(focus, QtWidgets.QComboBox) and focus.isEditable())
            )
            return keyboard_jog_command(
                event.text(),
                motion_enabled=self.enable_motion.isChecked(),
                text_input_focused=text_input_focused,
                auto_repeat=event.isAutoRepeat(),
                motion_busy=self._motion_busy(),
            )

        def _editable_text_has_focus(self) -> bool:
            focus = self.focusWidget()
            return bool(
                isinstance(focus, QtWidgets.QLineEdit)
                or (
                    isinstance(
                        focus, (QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)
                    )
                    and not focus.isReadOnly()
                )
                or (isinstance(focus, QtWidgets.QComboBox) and focus.isEditable())
            )

        def _handle_keyboard_jog_press(self, event: Any) -> bool:
            key = str(event.text()).lower()
            if key not in _KEYBOARD_JOG or self._editable_text_has_focus():
                return False
            if event.isAutoRepeat():
                return self.jog_hold.source == "keyboard" and self.jog_hold.key == key
            return self._begin_jog_hold("keyboard", key)

        def _handle_keyboard_jog_release(self, event: Any) -> bool:
            key = str(event.text()).lower()
            if key not in _KEYBOARD_JOG:
                return False
            if event.isAutoRepeat():
                return self.jog_hold.source == "keyboard" and self.jog_hold.key == key
            if self.jog_hold.source != "keyboard" or self.jog_hold.key != key:
                return False
            self._release_jog_hold("keyboard")
            return True

        def eventFilter(self, watched: Any, event: Any) -> bool:
            if event.type() == QtCore.QEvent.KeyPress:
                if self._handle_keyboard_jog_press(event):
                    event.accept()
                    return True
            elif event.type() == QtCore.QEvent.KeyRelease:
                if self._handle_keyboard_jog_release(event):
                    event.accept()
                    return True
            elif event.type() in (QtCore.QEvent.WindowDeactivate, QtCore.QEvent.Hide):
                self._clear_jog_hold("window focus lost")
            return super().eventFilter(watched, event)

        def keyPressEvent(self, event: Any) -> None:
            if self._handle_keyboard_jog_press(event):
                event.accept()
                return
            super().keyPressEvent(event)

        def keyReleaseEvent(self, event: Any) -> None:
            if self._handle_keyboard_jog_release(event):
                event.accept()
                return
            super().keyReleaseEvent(event)

        def closeEvent(self, event: Any) -> None:
            launch_live = bool(self.launch is not None and self.launch.is_running())
            bootstrap_quiescent = bootstrap_is_quiescent(
                self._bootstrap_thread, self._bootstrap_attempt
            )
            if (
                self.runtime is None
                and not launch_live
                and bootstrap_quiescent
                and not self._teardown_active
            ):
                event.accept()
                return
            event.ignore()
            self._close_requested = True
            if not self._pending_stop:
                self._append_log("window close requested · safe cancellation/teardown started")
                self._request_stop()
            elif self.runtime is not None and self.runtime.session.unresolved_motion():
                self._append_log(
                    "close remains blocked: MoveGroup state is unresolved; ROS is retained"
                )

    return TcpControlWindow


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="tcp-gui")
    parser.add_argument("--workspace", default=".")
    parser.add_argument("--env", default=DEFAULT_ENVIRONMENT)
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP)
    parser.add_argument("--max-absolute-translation-mm", type=float, default=150.0)
    parser.add_argument("--no-rviz", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        args.workspace = str(Path(args.workspace).expanduser().resolve())
        validate_robot_ip(args.robot_ip)
        if (
            not math.isfinite(args.max_absolute_translation_mm)
            or args.max_absolute_translation_mm <= 0.0
        ):
            raise ValueError("--max-absolute-translation-mm must be finite and positive")
        if not discover_environment_ids(args.workspace):
            raise ValueError(f"no environment bundles found under {args.workspace}/environments")
    except (OSError, ValueError) as error:
        print(f"tcp-gui: {error}", file=sys.stderr)
        return 2

    try:
        from python_qt_binding import QtCore, QtGui, QtWidgets
    except ImportError as error:
        print(
            "tcp-gui requires python_qt_binding from a sourced ROS 2 desktop install: "
            f"{error}",
            file=sys.stderr,
        )
        return 2

    application = QtWidgets.QApplication.instance()
    owns_application = application is None
    if application is None:
        application = QtWidgets.QApplication(sys.argv[:1])
    application.setApplicationName("HX Manual TCP Control")
    Window = _make_window_class(QtCore, QtGui, QtWidgets)
    window = Window(args)
    # An embedded QApplication belongs to its host, including its process-wide
    # signal policy.  The standalone CLI owns and restores its handlers.
    signal_bridge = (
        install_qt_close_signal_handlers(QtCore, window)
        if owns_application
        else None
    )
    release_window = retain_qt_window(
        window,
        on_release=None if signal_bridge is None else signal_bridge.restore,
    )
    window.show()
    if not owns_application:
        return 0
    execute = getattr(application, "exec", None) or application.exec_
    try:
        return int(execute())
    finally:
        if signal_bridge is not None:
            signal_bridge.restore()
        release_window()


if __name__ == "__main__":
    raise SystemExit(main())
