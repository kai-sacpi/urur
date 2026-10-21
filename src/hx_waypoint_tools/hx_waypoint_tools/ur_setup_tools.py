"""Small, fail-closed helpers for configuring a UR work frame.

The pose accepted here is the six-value ``p[x,y,z,rx,ry,rz]`` representation
shown by a Universal Robots teach pendant.  Positions are metres and the last
three values form a rotation vector in radians.
"""

from __future__ import annotations

import math
import re
import socket
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol

from hx_waypoint_curobo.setup_config import rotvec_to_quaternion_xyzw


DEFAULT_ROBOT_IP = "192.168.1.99"
DEFAULT_DASHBOARD_PORT = 29999
DEFAULT_DASHBOARD_TIMEOUT_S = 3.0
MAX_DASHBOARD_LINE_BYTES = 4096

_NUMBER = re.compile(
    r"[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[eE][+-]?\d+)?"
)
_VARIABLE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_FRAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_POSE_LITERAL = re.compile(r"p\[[^\]\r\n]*\]")

TeachPendantPose = tuple[float, float, float, float, float, float]


class URSetupError(ValueError):
    """Raised when a setup input or Dashboard response is not trustworthy."""


class _DashboardSocket(Protocol):
    def settimeout(self, value: float) -> None: ...

    def recv(self, bufsize: int) -> bytes: ...

    def sendall(self, data: bytes) -> None: ...

    def close(self) -> None: ...


ConnectionFactory = Callable[[tuple[str, int], float], _DashboardSocket]


@dataclass(frozen=True)
class PlaneTransform:
    """The teach-pendant pose and its equivalent ROS transform."""

    pose_xyz_rotvec: TeachPendantPose
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    @property
    def setup_yaml_pose_xyz_rotvec(self) -> TeachPendantPose:
        """Value for ``work_frame.teach_pendant_pose_xyz_rotvec`` in setup.yaml."""

        return self.pose_xyz_rotvec


@dataclass(frozen=True)
class DashboardVariableResult:
    """One Dashboard greeting and the response to one variable query."""

    variable: str
    banner: str
    response: str


def parse_teach_pendant_pose(text: str) -> TeachPendantPose:
    """Parse exactly one finite UR ``p[x,y,z,rx,ry,rz]`` pose."""

    if not isinstance(text, str):
        raise URSetupError("teach-pendant pose must be text in p[x,y,z,rx,ry,rz] form")
    stripped = text.strip()
    if not stripped.startswith("p[") or not stripped.endswith("]"):
        raise URSetupError("teach-pendant pose must be text in p[x,y,z,rx,ry,rz] form")

    fields = stripped[2:-1].split(",")
    if len(fields) != 6:
        raise URSetupError("teach-pendant pose must contain exactly six finite numbers")

    values: list[float] = []
    for field in fields:
        token = field.strip()
        if not _NUMBER.fullmatch(token):
            raise URSetupError("teach-pendant pose must contain exactly six finite numbers")
        value = float(token)
        if not math.isfinite(value):
            raise URSetupError("teach-pendant pose must contain exactly six finite numbers")
        values.append(value)

    return tuple(values)  # type: ignore[return-value]


def extract_teach_pendant_pose_from_dashboard_response(
    response: str,
) -> TeachPendantPose:
    """Extract exactly one UR pose literal from a Dashboard response line."""

    if not isinstance(response, str):
        raise URSetupError("Dashboard response must be text containing one p[...] pose")
    matches = _POSE_LITERAL.findall(response)
    if len(matches) != 1:
        raise URSetupError(
            "Dashboard response must contain exactly one p[x,y,z,rx,ry,rz] pose"
        )
    return parse_teach_pendant_pose(matches[0])


def _validated_pose(pose: Sequence[float]) -> TeachPendantPose:
    if isinstance(pose, (str, bytes)) or not isinstance(pose, Sequence) or len(pose) != 6:
        raise URSetupError("pose must contain exactly six finite numbers")
    values: list[float] = []
    for item in pose:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise URSetupError("pose must contain exactly six finite numbers")
        value = float(item)
        if not math.isfinite(value):
            raise URSetupError("pose must contain exactly six finite numbers")
        values.append(value)
    return tuple(values)  # type: ignore[return-value]


def plane_transform_from_pose(pose: Sequence[float]) -> PlaneTransform:
    """Convert a validated teach-pendant pose to a normalized ROS transform."""

    validated = _validated_pose(pose)
    return PlaneTransform(
        pose_xyz_rotvec=validated,
        translation_m=validated[:3],
        quaternion_xyzw=rotvec_to_quaternion_xyzw(validated[3:]),
    )


def static_transform_publisher_command(
    pose: Sequence[float],
    parent_frame: str = "base",
    child_frame: str = "plane1",
) -> str:
    """Build a copyable ROS 2 Jazzy static-transform command."""

    if not isinstance(parent_frame, str) or not _FRAME.fullmatch(parent_frame):
        raise URSetupError("parent_frame is not a safe ROS frame identifier")
    if not isinstance(child_frame, str) or not _FRAME.fullmatch(child_frame):
        raise URSetupError("child_frame is not a safe ROS frame identifier")
    transform = plane_transform_from_pose(pose)
    x, y, z = transform.translation_m
    qx, qy, qz, qw = transform.quaternion_xyzw
    arguments = (
        "ros2",
        "run",
        "tf2_ros",
        "static_transform_publisher",
        "--x",
        repr(x),
        "--y",
        repr(y),
        "--z",
        repr(z),
        "--qx",
        repr(qx),
        "--qy",
        repr(qy),
        "--qz",
        repr(qz),
        "--qw",
        repr(qw),
        "--frame-id",
        parent_frame,
        "--child-frame-id",
        child_frame,
    )
    return " ".join(arguments)


def _read_dashboard_line(connection: _DashboardSocket, label: str) -> str:
    payload = bytearray()
    while len(payload) <= MAX_DASHBOARD_LINE_BYTES:
        try:
            chunk = connection.recv(1)
        except (OSError, TimeoutError) as exc:
            raise URSetupError(f"Dashboard {label} could not be read: {exc}") from exc
        if not chunk:
            raise URSetupError(f"Dashboard closed the connection before the {label} line")
        if chunk == b"\n":
            try:
                result = payload.rstrip(b"\r").decode("utf-8", errors="strict")
            except UnicodeDecodeError as exc:
                raise URSetupError(f"Dashboard {label} is not valid UTF-8") from exc
            if not result:
                raise URSetupError(f"Dashboard {label} line is empty")
            return result
        payload.extend(chunk)
    raise URSetupError(
        f"Dashboard {label} exceeds {MAX_DASHBOARD_LINE_BYTES} bytes"
    )


def query_dashboard_variable(
    robot_ip: str = DEFAULT_ROBOT_IP,
    variable: str = "opt1",
    port: int = DEFAULT_DASHBOARD_PORT,
    timeout_s: float = DEFAULT_DASHBOARD_TIMEOUT_S,
    connection_factory: ConnectionFactory = socket.create_connection,
) -> DashboardVariableResult:
    """Query one safe global variable through the UR Dashboard TCP server."""

    if not isinstance(robot_ip, str) or not robot_ip or robot_ip.strip() != robot_ip:
        raise URSetupError("robot_ip must be a non-empty address without surrounding spaces")
    if not isinstance(variable, str) or not _VARIABLE.fullmatch(variable):
        raise URSetupError("Dashboard variable is not a safe identifier")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise URSetupError("Dashboard port must be an integer from 1 through 65535")
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
        raise URSetupError("Dashboard timeout must be a finite positive number")
    timeout = float(timeout_s)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise URSetupError("Dashboard timeout must be a finite positive number")

    connection: _DashboardSocket | None = None
    try:
        try:
            connection = connection_factory((robot_ip, port), timeout)
            connection.settimeout(timeout)
        except (OSError, TimeoutError) as exc:
            message = f"could not connect to UR Dashboard at {robot_ip}:{port}: {exc}"
            raise URSetupError(message) from exc

        banner = _read_dashboard_line(connection, "welcome")
        command = f"getVariable {variable}\n".encode("ascii")
        try:
            connection.sendall(command)
        except (OSError, TimeoutError) as exc:
            raise URSetupError(f"Dashboard variable query could not be sent: {exc}") from exc
        response = _read_dashboard_line(connection, "response")
        return DashboardVariableResult(variable=variable, banner=banner, response=response)
    finally:
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass
