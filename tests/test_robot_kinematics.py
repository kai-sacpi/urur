from __future__ import annotations

from math import pi, sqrt
from pathlib import Path

import pytest

from hx_waypoint_curobo.robot_kinematics import (
    Pose,
    QuinticHermiteSegment,
    RobotKinematics,
    compose_poses,
    inverse_pose,
    pose_error,
    real_polynomial_roots_on_interval,
)


SIMPLE_URDF = b"""\
<robot name="test">
  <link name="base"/>
  <link name="mount"/>
  <link name="arm"/>
  <link name="flange"/>
  <link name="tool0"/>
  <joint name="base_mount" type="fixed">
    <parent link="base"/><child link="mount"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="joint_a" type="revolute">
    <parent link="mount"/><child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 2"/><limit lower="-2" upper="3"/>
  </joint>
  <joint name="arm_flange" type="fixed">
    <parent link="arm"/><child link="flange"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="flange_tool" type="fixed">
    <parent link="flange"/><child link="tool0"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_urdf_limits_fk_flange_tool_and_appended_tip() -> None:
    model = RobotKinematics.from_urdf_bytes(
        SIMPLE_URDF,
        base_link="base",
        joint_names=("joint_a",),
    )

    assert model.position_limits[0].lower_rad == -2.0
    assert model.position_limits[0].upper_rad == 3.0
    assert model.position_limits[0].contains(3.0)
    assert model.position_limits[0].violation_rad(3.25) == 0.25

    frames = model.forward_kinematics(
        (pi / 2.0,),
        ("arm", "flange", "tool0", "rod_tip"),
        tool_to_tip=Pose((0.5, 0.0, 0.0)),
    )
    assert frames["arm"].position_m == pytest.approx((1.0, 0.0, 0.0), abs=1.0e-12)
    assert frames["flange"].position_m == pytest.approx((1.0, 1.0, 0.0), abs=1.0e-12)
    assert frames["tool0"].position_m == pytest.approx((1.0, 1.0, 1.0), abs=1.0e-12)
    assert frames["rod_tip"].position_m == pytest.approx((1.0, 1.5, 1.0), abs=1.0e-12)


def test_manifest_order_and_unsafe_xml_are_rejected() -> None:
    with pytest.raises(ValueError, match="DTD"):
        RobotKinematics.from_urdf_bytes(
            b'<!DOCTYPE robot [<!ENTITY x "boom">]><robot name="x"/>',
            base_link="base",
            joint_names=("joint_a",),
        )

    two_joint = SIMPLE_URDF.replace(
        b'<joint name="arm_flange" type="fixed">',
        b'<joint name="joint_b" type="revolute">',
    ).replace(
        b'<origin xyz="1 0 0" rpy="0 0 0"/>\n  </joint>\n  <joint name="flange_tool"',
        b'<origin xyz="1 0 0" rpy="0 0 0"/>\n'
        b'    <axis xyz="0 1 0"/><limit lower="-1" upper="1"/>\n'
        b'  </joint>\n  <joint name="flange_tool"',
    )
    with pytest.raises(ValueError, match="joint order"):
        RobotKinematics.from_urdf_bytes(
            two_joint,
            base_link="base",
            joint_names=("joint_b", "joint_a"),
        )


def test_real_urdf_parses_manifest_limits_and_frames(repository_root: Path) -> None:
    urdf = (
        repository_root / "environments/lab_wall_a/robot/assets/ur5.urdf"
    ).read_bytes()
    joint_names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    model = RobotKinematics.from_urdf_bytes(
        urdf,
        base_link="base_link",
        joint_names=joint_names,
    )
    frames = model.forward_kinematics(
        (0.0, -pi / 2.0, pi / 2.0, -pi / 2.0, -pi / 2.0, 0.0),
        ("shoulder_link", "wrist_3_link", "flange", "tool0", "rod_tip"),
        tool_to_tip=Pose((0.0, 0.0, 0.264)),
    )

    assert tuple(limit.name for limit in model.position_limits) == joint_names
    assert all(limit.lower_rad < limit.upper_rad for limit in model.position_limits)
    assert set(frames) == {"shoulder_link", "wrist_3_link", "flange", "tool0", "rod_tip"}
    assert all(
        all(abs(value) < 10.0 for value in pose.position_m)
        for pose in frames.values()
    )


def test_pose_composition_and_shortest_orientation_error() -> None:
    first = Pose((1.0, 0.0, 0.0), (0.0, 0.0, sqrt(0.5), sqrt(0.5)))
    second = Pose((1.0, 0.0, 0.0))
    composed = compose_poses(first, second)

    assert composed.position_m == pytest.approx((1.0, 1.0, 0.0), abs=1.0e-12)
    quaternion = composed.quaternion_xyzw
    error = pose_error(
        composed,
        Pose(
            (1.0, 1.0, 0.0),
            (-quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]),
        ),
    )
    assert error.position_m == pytest.approx(0.0)
    assert error.orientation_rad == pytest.approx(0.0, abs=1.0e-12)
    identity = compose_poses(composed, inverse_pose(composed))
    assert identity.position_m == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-12)
    assert identity.quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-12)


def test_quintic_boundary_conditions_and_continuous_extrema() -> None:
    segment = QuinticHermiteSegment.from_boundary_conditions(
        duration_s=1.0,
        start_position=0.0,
        start_velocity=0.0,
        start_acceleration=0.0,
        end_position=1.0,
        end_velocity=0.0,
        end_acceleration=0.0,
    )

    assert segment.coefficients_u == pytest.approx((0.0, 0.0, 0.0, 10.0, -15.0, 6.0))
    for derivative in range(3):
        assert segment.evaluate(0.0, derivative) == pytest.approx(0.0)
    assert segment.evaluate(1.0, 0) == pytest.approx(1.0)
    assert segment.evaluate(1.0, 1) == pytest.approx(0.0)
    assert segment.evaluate(1.0, 2) == pytest.approx(0.0)

    velocity_peak = segment.maximum_absolute(1)
    assert velocity_peak.time_s == pytest.approx(0.5, abs=1.0e-10)
    assert velocity_peak.value == pytest.approx(1.875, rel=1.0e-10)
    assert segment.maximum_absolute(3).absolute_value == pytest.approx(60.0, rel=1.0e-10)

    acceleration_candidates = segment.candidate_extrema(2)
    interior_times = [item.time_s for item in acceleration_candidates if 0.0 < item.time_s < 1.0]
    expected = ((3.0 - sqrt(3.0)) / 6.0, (3.0 + sqrt(3.0)) / 6.0)
    assert interior_times == pytest.approx(expected, abs=1.0e-10)


def test_quintic_extrema_preserve_very_short_segment_endpoints() -> None:
    segment = QuinticHermiteSegment.from_boundary_conditions(
        duration_s=1.0e-12,
        start_position=0.0,
        start_velocity=0.0,
        start_acceleration=0.0,
        end_position=1.0,
        end_velocity=0.0,
        end_acceleration=0.0,
    )

    assert tuple(item.time_s for item in segment.candidate_extrema(0)) == pytest.approx(
        (0.0, 1.0e-12),
        abs=1.0e-24,
    )
    assert segment.maximum_absolute(0).value == pytest.approx(1.0)
    assert segment.maximum_absolute(3).absolute_value == pytest.approx(6.0e37)

    unrepresentable = QuinticHermiteSegment.from_boundary_conditions(
        duration_s=1.0e-110,
        start_position=0.0,
        start_velocity=0.0,
        start_acceleration=0.0,
        end_position=1.0,
        end_velocity=0.0,
        end_acceleration=0.0,
    )
    with pytest.raises(ValueError, match="not representable"):
        unrepresentable.maximum_absolute(3)


def test_quartic_root_isolation_finds_even_multiplicity_roots() -> None:
    # (x - 0.2)^2 * (x - 0.8)^2, ascending coefficient order.
    roots = real_polynomial_roots_on_interval(
        (0.0256, -0.32, 1.32, -2.0, 1.0),
        0.0,
        1.0,
    )

    assert roots == pytest.approx((0.2, 0.8), abs=1.0e-9)
