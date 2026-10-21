"""Dependency-free URDF kinematics and quintic trajectory verification helpers.

The recording inspector needs to validate sealed artifacts without importing
cuRobo, NumPy, SciPy, ROS, or a platform-specific URDF library.  This module
therefore implements only the small, fail-closed subset required by that path:

* fixed and bounded revolute URDF joints;
* serial-chain forward kinematics in the manifest joint order;
* an explicit caller-provided ``tool0`` to ``rod_tip`` transform; and
* continuous extrema of one quintic Hermite segment.

Polynomial roots are isolated deterministically on a bounded interval.  No
sample grid is used to certify position, velocity, acceleration, or jerk.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, cos, isfinite, pi, sin, sqrt
from typing import Iterable, Sequence
from xml.etree import ElementTree


_MAX_URDF_BYTES = 16 * 1024 * 1024
_QUATERNION_NORM_TOLERANCE = 1.0e-3
_ROOT_X_TOLERANCE = 1.0e-13
_ROOT_VALUE_TOLERANCE = 1.0e-12

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


def _finite_tuple(values: Sequence[float], size: int, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(isfinite(value) for value in result):
        raise ValueError(f"{label} must contain exactly {size} finite values")
    return result


def _normalized_quaternion(values: Sequence[float], label: str) -> Quaternion:
    quaternion = _finite_tuple(values, 4, label)
    norm = sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-15 or abs(norm - 1.0) > _QUATERNION_NORM_TOLERANCE:
        raise ValueError(f"{label} must be normalized")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


@dataclass(frozen=True)
class Pose:
    """A finite rigid pose using ROS ``xyzw`` quaternion order."""

    position_m: Vector3
    quaternion_xyzw: Quaternion = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _finite_tuple(self.position_m, 3, "position_m"))
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _normalized_quaternion(self.quaternion_xyzw, "quaternion_xyzw"),
        )


@dataclass(frozen=True)
class PoseError:
    """Translation and shortest-path orientation error between two poses."""

    position_m: float
    orientation_rad: float

    @property
    def orientation_deg(self) -> float:
        return self.orientation_rad * 180.0 / pi


@dataclass(frozen=True)
class JointPositionLimit:
    """One bounded revolute joint domain from the sealed URDF."""

    name: str
    lower_rad: float
    upper_rad: float

    def __post_init__(self) -> None:
        if (
            not self.name
            or self.name.strip() != self.name
            or any(character.isspace() for character in self.name)
        ):
            raise ValueError("joint limit name must be a non-empty URDF identifier")
        if (
            not isfinite(self.lower_rad)
            or not isfinite(self.upper_rad)
            or self.lower_rad >= self.upper_rad
        ):
            raise ValueError(f"joint {self.name} must have finite increasing position limits")

    def contains(self, value_rad: float, *, tolerance_rad: float = 0.0) -> bool:
        value = float(value_rad)
        tolerance = float(tolerance_rad)
        return (
            isfinite(value)
            and isfinite(tolerance)
            and tolerance >= 0.0
            and self.lower_rad - tolerance <= value <= self.upper_rad + tolerance
        )

    def violation_rad(self, value_rad: float) -> float:
        value = float(value_rad)
        if not isfinite(value):
            return float("inf")
        return max(self.lower_rad - value, value - self.upper_rad, 0.0)


@dataclass(frozen=True)
class PolynomialExtremum:
    """One exact-candidate extremum of a physical-time derivative."""

    time_s: float
    value: float

    @property
    def absolute_value(self) -> float:
        return abs(self.value)


@dataclass(frozen=True)
class _Joint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin: Matrix4
    axis: Vector3
    limit: JointPositionLimit | None


def _identity_matrix() -> Matrix4:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matrix_multiply(left: Matrix4, right: Matrix4) -> Matrix4:
    rows = []
    for row in range(4):
        rows.append(
            tuple(
                sum(left[row][inner] * right[inner][column] for inner in range(4))
                for column in range(4)
            )
        )
    return tuple(rows)  # type: ignore[return-value]


def _rpy_matrix(roll: float, pitch: float, yaw: float, translation: Vector3) -> Matrix4:
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, translation[0]),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, translation[1]),
        (-sp, cp * sr, cp * cr, translation[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _axis_angle_matrix(axis: Vector3, angle: float) -> Matrix4:
    x, y, z = axis
    cosine = cos(angle)
    sine = sin(angle)
    complement = 1.0 - cosine
    return (
        (
            cosine + x * x * complement,
            x * y * complement - z * sine,
            x * z * complement + y * sine,
            0.0,
        ),
        (
            y * x * complement + z * sine,
            cosine + y * y * complement,
            y * z * complement - x * sine,
            0.0,
        ),
        (
            z * x * complement - y * sine,
            z * y * complement + x * sine,
            cosine + z * z * complement,
            0.0,
        ),
        (0.0, 0.0, 0.0, 1.0),
    )


def _quaternion_matrix(quaternion_xyzw: Quaternion, translation: Vector3) -> Matrix4:
    x, y, z, w = quaternion_xyzw
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            translation[0],
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            translation[1],
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
            translation[2],
        ),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matrix_quaternion(matrix: Matrix4) -> Quaternion:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = sqrt(max(0.0, trace + 1.0)) * 2.0
        quaternion = (
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            0.25 * scale,
        )
    elif matrix[0][0] >= matrix[1][1] and matrix[0][0] >= matrix[2][2]:
        scale = sqrt(max(0.0, 1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2])) * 2.0
        quaternion = (
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[2][1] - matrix[1][2]) / scale,
        )
    elif matrix[1][1] >= matrix[2][2]:
        scale = sqrt(max(0.0, 1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2])) * 2.0
        quaternion = (
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
        )
    else:
        scale = sqrt(max(0.0, 1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1])) * 2.0
        quaternion = (
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    return _normalized_quaternion(quaternion, "computed quaternion")


def _matrix_pose(matrix: Matrix4) -> Pose:
    return Pose(
        position_m=(matrix[0][3], matrix[1][3], matrix[2][3]),
        quaternion_xyzw=_matrix_quaternion(matrix),
    )


def _pose_matrix(pose: Pose) -> Matrix4:
    return _quaternion_matrix(pose.quaternion_xyzw, pose.position_m)


def compose_poses(parent_to_intermediate: Pose, intermediate_to_child: Pose) -> Pose:
    """Compose two rigid transforms in parent-to-child order."""

    return _matrix_pose(
        _matrix_multiply(_pose_matrix(parent_to_intermediate), _pose_matrix(intermediate_to_child))
    )


def inverse_pose(pose: Pose) -> Pose:
    """Invert a rigid transform without introducing a second math convention."""

    matrix = _pose_matrix(pose)
    rotation_transpose = tuple(
        tuple(matrix[column][row] for column in range(3)) for row in range(3)
    )
    translation = tuple(matrix[row][3] for row in range(3))
    inverse_translation = tuple(
        -sum(rotation_transpose[row][column] * translation[column] for column in range(3))
        for row in range(3)
    )
    inverse: Matrix4 = (
        (
            rotation_transpose[0][0],
            rotation_transpose[0][1],
            rotation_transpose[0][2],
            inverse_translation[0],
        ),
        (
            rotation_transpose[1][0],
            rotation_transpose[1][1],
            rotation_transpose[1][2],
            inverse_translation[1],
        ),
        (
            rotation_transpose[2][0],
            rotation_transpose[2][1],
            rotation_transpose[2][2],
            inverse_translation[2],
        ),
        (0.0, 0.0, 0.0, 1.0),
    )
    return _matrix_pose(inverse)


def quaternion_angular_error_rad(left: Sequence[float], right: Sequence[float]) -> float:
    """Return the shortest angular distance between two quaternion orientations."""

    first = _normalized_quaternion(left, "left quaternion")
    second = _normalized_quaternion(right, "right quaternion")
    dot = abs(sum(a * b for a, b in zip(first, second, strict=True)))
    if 1.0 - dot <= 1.0e-15:
        return 0.0
    return 2.0 * acos(min(1.0, max(-1.0, dot)))


def pose_error(actual: Pose, expected: Pose) -> PoseError:
    """Return Euclidean translation and shortest-path orientation errors."""

    delta = tuple(
        actual_value - expected_value
        for actual_value, expected_value in zip(
            actual.position_m,
            expected.position_m,
            strict=True,
        )
    )
    return PoseError(
        position_m=sqrt(sum(value * value for value in delta)),
        orientation_rad=quaternion_angular_error_rad(
            actual.quaternion_xyzw,
            expected.quaternion_xyzw,
        ),
    )


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _identifier(value: object, label: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string")
    result = value.strip()
    if not result or result != value or any(character.isspace() for character in result):
        raise ValueError(f"{label} must be a non-empty URDF identifier")
    return result


def _single_child(element: ElementTree.Element, name: str) -> ElementTree.Element | None:
    children = [child for child in element if _local_name(child.tag) == name]
    if len(children) > 1:
        raise ValueError(f"URDF {element.tag} contains duplicate {name} elements")
    return children[0] if children else None


def _xml_vector(raw: str | None, default: Vector3, label: str) -> Vector3:
    if raw is None:
        return default
    try:
        values = tuple(float(value) for value in raw.split())
    except ValueError as error:
        raise ValueError(f"{label} must contain exactly 3 finite values") from error
    return _finite_tuple(values, 3, label)  # type: ignore[return-value]


def _origin_matrix(joint: ElementTree.Element, label: str) -> Matrix4:
    origin = _single_child(joint, "origin")
    if origin is None:
        return _identity_matrix()
    xyz = _xml_vector(origin.attrib.get("xyz"), (0.0, 0.0, 0.0), f"{label} origin xyz")
    rpy = _xml_vector(origin.attrib.get("rpy"), (0.0, 0.0, 0.0), f"{label} origin rpy")
    return _rpy_matrix(rpy[0], rpy[1], rpy[2], xyz)


class RobotKinematics:
    """Fail-closed fixed/revolute kinematics parsed from immutable URDF bytes."""

    def __init__(
        self,
        *,
        base_link: str,
        joint_names: tuple[str, ...],
        links: frozenset[str],
        joints: tuple[_Joint, ...],
    ) -> None:
        self.base_link = base_link
        self.joint_names = joint_names
        self._links = links
        self._joints = joints
        self._joint_by_name = {joint.name: joint for joint in joints}
        self._parent_joint_by_child = {joint.child_link: joint for joint in joints}
        missing = [name for name in joint_names if name not in self._joint_by_name]
        if missing:
            raise ValueError("URDF is missing manifest joints: " + ", ".join(missing))
        self.position_limits = tuple(
            self._required_limit(self._joint_by_name[name]) for name in joint_names
        )

    @classmethod
    def from_urdf_bytes(
        cls,
        urdf_bytes: bytes,
        *,
        base_link: str,
        joint_names: Sequence[str],
    ) -> RobotKinematics:
        """Parse bounded serial-chain kinematics without resolving external resources."""

        if not isinstance(urdf_bytes, bytes) or not urdf_bytes:
            raise ValueError("URDF must be non-empty bytes")
        if len(urdf_bytes) > _MAX_URDF_BYTES:
            raise ValueError("URDF exceeds the safe parser size limit")
        try:
            urdf_text = urdf_bytes.decode("utf-8-sig")
        except UnicodeDecodeError as error:
            raise ValueError("URDF must be valid UTF-8") from error
        lowered = urdf_text.lower()
        if "<!doctype" in lowered or "<!entity" in lowered:
            raise ValueError("URDF DTD and entity declarations are forbidden")
        try:
            root = ElementTree.fromstring(urdf_text)
        except ElementTree.ParseError as error:
            raise ValueError(f"URDF XML is malformed: {error}") from error
        if _local_name(root.tag) != "robot":
            raise ValueError("URDF root element must be robot")

        parsed_base = _identifier(base_link, "base_link")
        requested_names = tuple(_identifier(name, "joint name") for name in joint_names)
        if not requested_names or len(set(requested_names)) != len(requested_names):
            raise ValueError("joint_names must be non-empty and unique")

        links: set[str] = set()
        for element in root:
            if _local_name(element.tag) != "link":
                continue
            name = _identifier(element.attrib.get("name"), "link name")
            if name in links:
                raise ValueError(f"URDF contains duplicate link {name}")
            links.add(name)
        if parsed_base not in links:
            raise ValueError(f"URDF does not contain base link {parsed_base}")

        joints: list[_Joint] = []
        joint_names_seen: set[str] = set()
        child_links_seen: set[str] = set()
        for element in root:
            if _local_name(element.tag) != "joint":
                continue
            name = _identifier(element.attrib.get("name"), "joint name")
            if name in joint_names_seen:
                raise ValueError(f"URDF contains duplicate joint {name}")
            joint_names_seen.add(name)
            joint_type = element.attrib.get("type")
            if joint_type not in {"fixed", "revolute"}:
                raise ValueError(f"URDF joint {name} has unsupported type {joint_type!r}")
            parent_element = _single_child(element, "parent")
            child_element = _single_child(element, "child")
            if parent_element is None or child_element is None:
                raise ValueError(f"URDF joint {name} must declare one parent and child")
            parent = _identifier(parent_element.attrib.get("link"), f"joint {name} parent")
            child = _identifier(child_element.attrib.get("link"), f"joint {name} child")
            if parent == child or parent not in links or child not in links:
                raise ValueError(f"URDF joint {name} has invalid parent/child links")
            if child in child_links_seen:
                raise ValueError(f"URDF link {child} has more than one parent joint")
            child_links_seen.add(child)

            axis: Vector3 = (1.0, 0.0, 0.0)
            limit = None
            if joint_type == "revolute":
                axis_element = _single_child(element, "axis")
                if axis_element is not None:
                    axis = _xml_vector(
                        axis_element.attrib.get("xyz"),
                        (1.0, 0.0, 0.0),
                        f"joint {name} axis",
                    )
                axis_norm = sqrt(sum(value * value for value in axis))
                if axis_norm <= 1.0e-15:
                    raise ValueError(f"URDF joint {name} axis must be non-zero")
                axis = tuple(value / axis_norm for value in axis)  # type: ignore[assignment]
                limit_element = _single_child(element, "limit")
                if limit_element is None:
                    raise ValueError(f"bounded revolute joint {name} is missing limit")
                try:
                    lower = float(limit_element.attrib["lower"])
                    upper = float(limit_element.attrib["upper"])
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(f"URDF joint {name} has invalid position limits") from error
                limit = JointPositionLimit(name=name, lower_rad=lower, upper_rad=upper)
            joints.append(
                _Joint(
                    name=name,
                    joint_type=joint_type,
                    parent_link=parent,
                    child_link=child,
                    origin=_origin_matrix(element, f"joint {name}"),
                    axis=axis,
                    limit=limit,
                )
            )

        result = cls(
            base_link=parsed_base,
            joint_names=requested_names,
            links=frozenset(links),
            joints=tuple(joints),
        )
        result._validate_tree_and_manifest_order()
        return result

    @staticmethod
    def _required_limit(joint: _Joint) -> JointPositionLimit:
        if joint.joint_type != "revolute" or joint.limit is None:
            raise ValueError(f"manifest joint {joint.name} must be a bounded revolute joint")
        return joint.limit

    def _chain_to_link(self, target_link: str) -> tuple[_Joint, ...]:
        if target_link not in self._links:
            raise ValueError(f"URDF does not contain requested frame {target_link}")
        if target_link == self.base_link:
            return ()
        reversed_chain: list[_Joint] = []
        seen = {target_link}
        current = target_link
        while current != self.base_link:
            joint = self._parent_joint_by_child.get(current)
            if joint is None:
                raise ValueError(
                    f"URDF has no serial chain from {self.base_link} to {target_link}"
                )
            reversed_chain.append(joint)
            current = joint.parent_link
            if current in seen:
                raise ValueError("URDF kinematic graph contains a cycle")
            seen.add(current)
        return tuple(reversed(reversed_chain))

    def _validate_tree_and_manifest_order(self) -> None:
        for link in self._links:
            current = link
            seen = {current}
            while current in self._parent_joint_by_child:
                current = self._parent_joint_by_child[current].parent_link
                if current in seen:
                    raise ValueError("URDF kinematic graph contains a cycle")
                seen.add(current)
        last_joint = self._joint_by_name[self.joint_names[-1]]
        chain_names = tuple(
            joint.name
            for joint in self._chain_to_link(last_joint.child_link)
            if joint.joint_type == "revolute"
        )
        if chain_names != self.joint_names:
            raise ValueError("URDF serial-chain joint order does not match manifest joint_names")

    def forward_kinematics(
        self,
        joint_positions_rad: Sequence[float],
        frame_names: Sequence[str],
        *,
        tool_to_tip: Pose | None = None,
        tool_frame: str = "tool0",
        tip_frame: str = "rod_tip",
    ) -> dict[str, Pose]:
        """Compute requested frame poses in ``base_link`` coordinates.

        URDF joint transforms are applied as ``origin * axis_rotation(q)``.
        ``tip_frame`` is synthetic and is appended from ``tool_frame`` using the
        caller-provided ``tool_to_tip`` transform.
        """

        positions = _finite_tuple(
            joint_positions_rad,
            len(self.joint_names),
            "joint_positions_rad",
        )
        requested = tuple(_identifier(name, "frame name") for name in frame_names)
        if not requested or len(set(requested)) != len(requested):
            raise ValueError("frame_names must be non-empty and unique")
        q_by_name = dict(zip(self.joint_names, positions, strict=True))

        def urdf_pose(frame: str) -> Pose:
            transform = _identity_matrix()
            for joint in self._chain_to_link(frame):
                transform = _matrix_multiply(transform, joint.origin)
                if joint.joint_type == "revolute":
                    if joint.name not in q_by_name:
                        raise ValueError(
                            f"requested frame {frame} depends on non-manifest joint {joint.name}"
                        )
                    transform = _matrix_multiply(
                        transform,
                        _axis_angle_matrix(joint.axis, q_by_name[joint.name]),
                    )
            return _matrix_pose(transform)

        output: dict[str, Pose] = {}
        for frame in requested:
            if frame == tip_frame:
                if tool_to_tip is None:
                    raise ValueError(f"requested {tip_frame} requires caller tool_to_tip pose")
                output[frame] = compose_poses(urdf_pose(tool_frame), tool_to_tip)
            else:
                output[frame] = urdf_pose(frame)
        return output


def _trim_polynomial(coefficients: Sequence[float]) -> tuple[float, ...]:
    result = list(_finite_tuple(coefficients, len(coefficients), "polynomial coefficients"))
    if not result:
        raise ValueError("polynomial coefficients must not be empty")
    scale = max(1.0, max(abs(value) for value in result))
    tolerance = 1.0e-15 * scale
    while len(result) > 1 and abs(result[-1]) <= tolerance:
        result.pop()
    return tuple(result)


def _polynomial_value(coefficients: Sequence[float], value: float) -> float:
    result = 0.0
    for coefficient in reversed(coefficients):
        result = result * value + coefficient
    return result


def _polynomial_derivative(coefficients: Sequence[float]) -> tuple[float, ...]:
    if len(coefficients) <= 1:
        return (0.0,)
    return tuple(index * coefficients[index] for index in range(1, len(coefficients)))


def _deduplicate_sorted(values: Iterable[float], tolerance: float) -> tuple[float, ...]:
    result: list[float] = []
    for value in sorted(values):
        if not result or abs(value - result[-1]) > tolerance:
            result.append(value)
        else:
            result[-1] = 0.5 * (result[-1] + value)
    return tuple(result)


def real_polynomial_roots_on_interval(
    coefficients_ascending: Sequence[float],
    lower: float,
    upper: float,
) -> tuple[float, ...]:
    """Isolate all real roots of a degree-at-most-four polynomial on an interval."""

    coefficients = _trim_polynomial(coefficients_ascending)
    left = float(lower)
    right = float(upper)
    if (
        len(coefficients) > 5
        or not isfinite(left)
        or not isfinite(right)
        or left > right
    ):
        raise ValueError("root isolation requires degree <= 4 and a finite ordered interval")
    degree = len(coefficients) - 1
    if degree == 0:
        return ()
    scale = max(1.0, sum(abs(value) for value in coefficients))
    value_tolerance = _ROOT_VALUE_TOLERANCE * scale
    if degree == 1:
        root = -coefficients[0] / coefficients[1]
        if left - _ROOT_X_TOLERANCE <= root <= right + _ROOT_X_TOLERANCE:
            return (min(right, max(left, root)),)
        return ()

    critical = real_polynomial_roots_on_interval(
        _polynomial_derivative(coefficients),
        left,
        right,
    )
    boundaries = _deduplicate_sorted((left, *critical, right), _ROOT_X_TOLERANCE)
    roots: list[float] = []
    for boundary in boundaries:
        if abs(_polynomial_value(coefficients, boundary)) <= value_tolerance:
            roots.append(boundary)
    for interval_left, interval_right in zip(boundaries[:-1], boundaries[1:], strict=True):
        left_value = _polynomial_value(coefficients, interval_left)
        right_value = _polynomial_value(coefficients, interval_right)
        if left_value * right_value >= 0.0:
            continue
        low = interval_left
        high = interval_right
        low_value = left_value
        for _ in range(100):
            midpoint = 0.5 * (low + high)
            midpoint_value = _polynomial_value(coefficients, midpoint)
            if high - low <= _ROOT_X_TOLERANCE:
                low = high = midpoint
                break
            if low_value * midpoint_value <= 0.0:
                high = midpoint
            else:
                low = midpoint
                low_value = midpoint_value
        roots.append(0.5 * (low + high))
    return _deduplicate_sorted(
        (min(right, max(left, root)) for root in roots),
        1.0e-10,
    )


@dataclass(frozen=True)
class QuinticHermiteSegment:
    """One quintic segment parameterized on normalized time ``u in [0, 1]``."""

    duration_s: float
    coefficients_u: tuple[float, float, float, float, float, float]

    def __post_init__(self) -> None:
        duration = float(self.duration_s)
        coefficients = _finite_tuple(self.coefficients_u, 6, "quintic coefficients")
        if not isfinite(duration) or duration <= 0.0:
            raise ValueError("quintic duration_s must be finite and positive")
        object.__setattr__(self, "duration_s", duration)
        object.__setattr__(self, "coefficients_u", coefficients)

    @classmethod
    def from_boundary_conditions(
        cls,
        *,
        duration_s: float,
        start_position: float,
        start_velocity: float,
        start_acceleration: float,
        end_position: float,
        end_velocity: float,
        end_acceleration: float,
    ) -> QuinticHermiteSegment:
        values = _finite_tuple(
            (
                duration_s,
                start_position,
                start_velocity,
                start_acceleration,
                end_position,
                end_velocity,
                end_acceleration,
            ),
            7,
            "quintic boundary conditions",
        )
        duration, q0, v0, a0, q1, v1, a1 = values
        if duration <= 0.0:
            raise ValueError("quintic duration_s must be positive")
        c0 = q0
        c1 = v0 * duration
        c2 = 0.5 * a0 * duration * duration
        delta_position = q1 - (c0 + c1 + c2)
        delta_velocity = v1 * duration - (c1 + 2.0 * c2)
        delta_acceleration = a1 * duration * duration - 2.0 * c2
        c3 = 10.0 * delta_position - 4.0 * delta_velocity + 0.5 * delta_acceleration
        c4 = -15.0 * delta_position + 7.0 * delta_velocity - delta_acceleration
        c5 = 6.0 * delta_position - 3.0 * delta_velocity + 0.5 * delta_acceleration
        return cls(duration_s=duration, coefficients_u=(c0, c1, c2, c3, c4, c5))

    def _derivative_coefficients_u(self, derivative_order: int) -> tuple[float, ...]:
        if derivative_order not in range(0, 6):
            raise ValueError("quintic derivative order must be between 0 and 5")
        coefficients: tuple[float, ...] = self.coefficients_u
        for _ in range(derivative_order):
            coefficients = _polynomial_derivative(coefficients)
        return coefficients

    def evaluate(self, time_s: float, derivative_order: int = 0) -> float:
        """Evaluate position or a physical-time derivative at ``time_s``."""

        time = float(time_s)
        tolerance = 1.0e-12 * max(1.0, self.duration_s)
        if (
            not isfinite(time)
            or time < -tolerance
            or time > self.duration_s + tolerance
        ):
            raise ValueError("quintic evaluation time is outside the segment")
        time = min(self.duration_s, max(0.0, time))
        coefficients = self._derivative_coefficients_u(derivative_order)
        result = _polynomial_value(coefficients, time / self.duration_s)
        for _ in range(derivative_order):
            result /= self.duration_s
            if not isfinite(result):
                raise ValueError("quintic physical-time derivative is not representable")
        return result

    def candidate_extrema(self, derivative_order: int) -> tuple[PolynomialExtremum, ...]:
        """Return every endpoint/stationary candidate for derivative order 0..3."""

        if derivative_order not in range(0, 4):
            raise ValueError("extrema are supported for position through jerk only")
        next_derivative = self._derivative_coefficients_u(derivative_order + 1)
        roots_u = real_polynomial_roots_on_interval(next_derivative, 0.0, 1.0)
        candidate_u = _deduplicate_sorted(
            (0.0, 1.0, *roots_u),
            1.0e-10,
        )
        return tuple(
            PolynomialExtremum(
                time_s=normalized_time * self.duration_s,
                value=self.evaluate(normalized_time * self.duration_s, derivative_order),
            )
            for normalized_time in candidate_u
        )

    def maximum_absolute(self, derivative_order: int) -> PolynomialExtremum:
        """Return the exact-candidate maximum absolute derivative on the segment."""

        candidates = self.candidate_extrema(derivative_order)
        return max(candidates, key=lambda item: item.absolute_value)
