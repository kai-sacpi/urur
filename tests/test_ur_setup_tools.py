from __future__ import annotations

import math
import socket
from collections.abc import Iterable

import pytest

from hx_waypoint_curobo.setup_config import rotvec_to_quaternion_xyzw
from hx_waypoint_tools.ur_setup_tools import (
    DEFAULT_DASHBOARD_PORT,
    DEFAULT_ROBOT_IP,
    MAX_DASHBOARD_LINE_BYTES,
    URSetupError,
    extract_teach_pendant_pose_from_dashboard_response,
    parse_teach_pendant_pose,
    plane_transform_from_pose,
    query_dashboard_variable,
    static_transform_publisher_command,
)


class FakeSocket:
    def __init__(
        self,
        incoming: bytes = b"Connected: Universal Robots Dashboard Server\nopt1 = 42\n",
        *,
        recv_error: BaseException | None = None,
        send_error: BaseException | None = None,
    ) -> None:
        self.incoming = bytearray(incoming)
        self.recv_error = recv_error
        self.send_error = send_error
        self.sent: list[bytes] = []
        self.timeout: float | None = None
        self.closed = False

    def settimeout(self, value: float) -> None:
        self.timeout = value

    def recv(self, bufsize: int) -> bytes:
        if self.recv_error is not None:
            error, self.recv_error = self.recv_error, None
            raise error
        if not self.incoming:
            return b""
        result = bytes(self.incoming[:bufsize])
        del self.incoming[:bufsize]
        return result

    def sendall(self, data: bytes) -> None:
        if self.send_error is not None:
            raise self.send_error
        self.sent.append(data)

    def close(self) -> None:
        self.closed = True


class FakeConnectionFactory:
    def __init__(self, connections: Iterable[FakeSocket]) -> None:
        self.connections = iter(connections)
        self.calls: list[tuple[tuple[str, int], float]] = []

    def __call__(self, address: tuple[str, int], timeout: float) -> FakeSocket:
        self.calls.append((address, timeout))
        return next(self.connections)


def test_parse_teach_pendant_pose_accepts_finite_ur_pose() -> None:
    pose = parse_teach_pendant_pose(
        " p[-0.03413, -0.38168, 0.33594, -1.25152, 2.88032, -0.01303] \n"
    )

    assert pose == pytest.approx(
        (-0.03413, -0.38168, 0.33594, -1.25152, 2.88032, -0.01303)
    )


@pytest.mark.parametrize(
    "text",
    [
        "[-0.1,0,0,0,0,0]",
        "p[0,0,0,0,0]",
        "p[0,0,0,0,0,0,0]",
        "p[0,0,0,0,0,nan]",
        "p[0,0,0,0,0,inf]",
        "p[0,0,0,0,0,1_0]",
        "p[0,0,0,0,0,0] trailing",
        "p[0;0;0;0;0;0]",
    ],
)
def test_parse_teach_pendant_pose_rejects_ambiguous_or_nonfinite_input(text: str) -> None:
    with pytest.raises(URSetupError, match="six finite|p\\["):
        parse_teach_pendant_pose(text)


@pytest.mark.parametrize(
    "response",
    [
        "VariableValue: p[-0.1, -0.2, 0.3, 0, 0, 1.57]",
        "opt1 = p[-0.1,-0.2,0.3,0,0,1.57]",
    ],
)
def test_extract_pose_from_supported_dashboard_response_wrappers(response: str) -> None:
    assert extract_teach_pendant_pose_from_dashboard_response(response) == pytest.approx(
        (-0.1, -0.2, 0.3, 0.0, 0.0, 1.57)
    )


@pytest.mark.parametrize(
    "response",
    [
        "VariableValue: 42",
        "VariableValue: p[0,0,0,0,0,0] and p[1,1,1,0,0,0]",
        "VariableValue: p[0,0,0,0,0]",
        "",
    ],
)
def test_extract_pose_from_dashboard_response_fails_closed(response: str) -> None:
    with pytest.raises(URSetupError, match="exactly one|six finite"):
        extract_teach_pendant_pose_from_dashboard_response(response)


def test_plane_transform_reuses_authoritative_rotvec_conversion() -> None:
    pose = (-0.03413, -0.38168, 0.33594, -1.25152, 2.88032, -0.01303)

    transform = plane_transform_from_pose(pose)

    assert transform.translation_m == pose[:3]
    assert transform.quaternion_xyzw == pytest.approx(
        rotvec_to_quaternion_xyzw(pose[3:]), abs=1.0e-15
    )
    assert transform.setup_yaml_pose_xyz_rotvec == pose
    assert math.sqrt(sum(value * value for value in transform.quaternion_xyzw)) == pytest.approx(
        1.0, abs=1.0e-15
    )


@pytest.mark.parametrize(
    "pose",
    [
        (0.0, 0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 0.0, 0.0, float("nan")),
        (0.0, 0.0, 0.0, 0.0, False, 0.0),
    ],
)
def test_plane_transform_rejects_invalid_programmatic_pose(pose) -> None:
    with pytest.raises(URSetupError, match="six finite"):
        plane_transform_from_pose(pose)


def test_static_transform_command_uses_jazzy_long_options() -> None:
    pose = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)

    command = static_transform_publisher_command(pose)

    assert command == (
        "ros2 run tf2_ros static_transform_publisher "
        "--x 1.0 --y 2.0 --z 3.0 "
        "--qx 0.0 --qy 0.0 --qz 0.0 --qw 1.0 "
        "--frame-id base --child-frame-id plane1"
    )


@pytest.mark.parametrize("frame", ["", "base link", "base;shutdown", "/base", "base/child"])
def test_static_transform_command_rejects_unsafe_frame_names(frame: str) -> None:
    with pytest.raises(URSetupError, match="safe ROS frame"):
        static_transform_publisher_command((0.0,) * 6, parent_frame=frame)


def test_dashboard_query_uses_defaults_and_exact_command_bytes() -> None:
    connection = FakeSocket()
    factory = FakeConnectionFactory([connection])

    result = query_dashboard_variable(connection_factory=factory)

    assert factory.calls == [((DEFAULT_ROBOT_IP, DEFAULT_DASHBOARD_PORT), 3.0)]
    assert connection.timeout == 3.0
    assert connection.sent == [b"getVariable opt1\n"]
    assert connection.closed is True
    assert result.variable == "opt1"
    assert result.banner == "Connected: Universal Robots Dashboard Server"
    assert result.response == "opt1 = 42"


def test_dashboard_query_supports_safe_variable_and_connection_override() -> None:
    connection = FakeSocket(incoming=b"welcome\r\ncustom_2 = True\r\n")
    factory = FakeConnectionFactory([connection])

    result = query_dashboard_variable(
        robot_ip="127.0.0.1",
        variable="custom_2",
        port=30000,
        timeout_s=0.25,
        connection_factory=factory,
    )

    assert factory.calls == [(('127.0.0.1', 30000), 0.25)]
    assert connection.sent == [b"getVariable custom_2\n"]
    assert result.banner == "welcome"
    assert result.response == "custom_2 = True"


@pytest.mark.parametrize(
    "variable",
    ["", "1opt", "opt 1", "opt1\nshutdown", "opt.1", "中文"],
)
def test_dashboard_query_rejects_unsafe_variable_without_connecting(variable: str) -> None:
    factory = FakeConnectionFactory([])

    with pytest.raises(URSetupError, match="safe identifier"):
        query_dashboard_variable(variable=variable, connection_factory=factory)

    assert factory.calls == []


@pytest.mark.parametrize(
    ("incoming", "message"),
    [
        (b"", "before the welcome"),
        (b"welcome\n", "before the response"),
        (b"welcome\n\n", "response line is empty"),
        (b"welcome\n\xff\n", "not valid UTF-8"),
        (b"x" * (MAX_DASHBOARD_LINE_BYTES + 1), "exceeds"),
    ],
)
def test_dashboard_query_fails_closed_on_incomplete_or_oversized_lines(
    incoming: bytes, message: str
) -> None:
    connection = FakeSocket(incoming=incoming)

    with pytest.raises(URSetupError, match=message):
        query_dashboard_variable(connection_factory=FakeConnectionFactory([connection]))

    assert connection.closed is True


def test_dashboard_query_translates_socket_timeout_and_closes() -> None:
    connection = FakeSocket(recv_error=socket.timeout("timed out"))

    with pytest.raises(URSetupError, match="welcome could not be read"):
        query_dashboard_variable(connection_factory=FakeConnectionFactory([connection]))

    assert connection.closed is True


def test_dashboard_query_fails_closed_when_send_fails() -> None:
    connection = FakeSocket(incoming=b"welcome\n", send_error=BrokenPipeError("closed"))

    with pytest.raises(URSetupError, match="could not be sent"):
        query_dashboard_variable(connection_factory=FakeConnectionFactory([connection]))

    assert connection.closed is True


def test_dashboard_query_translates_connection_failure() -> None:
    def fail_connection(address: tuple[str, int], timeout: float) -> FakeSocket:
        raise ConnectionRefusedError("refused")

    with pytest.raises(URSetupError, match="could not connect"):
        query_dashboard_variable(connection_factory=fail_connection)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"robot_ip": " 192.168.1.99"}, "robot_ip"),
        ({"port": 0}, "port"),
        ({"port": True}, "port"),
        ({"timeout_s": 0.0}, "timeout"),
        ({"timeout_s": float("inf")}, "timeout"),
    ],
)
def test_dashboard_query_validates_connection_parameters_before_connecting(
    kwargs: dict[str, object], message: str
) -> None:
    factory = FakeConnectionFactory([])

    with pytest.raises(URSetupError, match=message):
        query_dashboard_variable(connection_factory=factory, **kwargs)

    assert factory.calls == []
