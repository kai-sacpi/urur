from __future__ import annotations

import copy
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from hx_waypoint_curobo.contact_policy import ContactPolicy
from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2, PathValidationMetrics
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import PlanningError
from hx_waypoint_curobo.math3d import pose_matrix
from hx_waypoint_curobo.models import CompiledTrajectory, JointSample, TimedPose
from hx_waypoint_curobo.robot_tool_builder import RobotToolBuilder
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv


class _CpuTorch:
    @staticmethod
    def as_tensor(data, *args, **kwargs):
        kwargs = dict(kwargs)
        if kwargs.get("device") == "cuda":
            kwargs["device"] = "cpu"
        return torch.as_tensor(data, *args, **kwargs)


class _JointState:
    @classmethod
    def from_position(cls, position, joint_names=None):
        return SimpleNamespace(position=position, joint_names=joint_names)


def _classification(policy: ContactPolicy):
    expected = np.asarray(policy.expected_rod_spheres_tool, dtype=float)
    tool_reference = np.vstack(
        (
            np.asarray([[0.0, 0.0, 0.03, 0.035]], dtype=float),
            expected,
        )
    )
    tool_indices = np.arange(1, 1 + len(tool_reference), dtype=np.int64)
    total_sphere_count = len(tool_reference) + 1
    classification = policy.classify_curobo_spheres(
        total_sphere_count=total_sphere_count,
        tool_sphere_indices=tool_indices,
        tool_reference_spheres=tool_reference,
    )
    return classification, tool_indices, tool_reference


def _safe_geometry(policy: ContactPolicy, sample_count: int = 2):
    classification, _, _ = _classification(policy)
    plane_normal = np.asarray(policy.plane_normal_base, dtype=float)
    plane_point = plane_normal * policy.plane_offset_base_m
    tips = np.repeat(plane_point.reshape(1, 3), sample_count, axis=0)
    spheres = np.zeros(
        (sample_count, classification.total_sphere_count, 4),
        dtype=float,
    )
    spheres[:, :, :3] = plane_point + plane_normal * 0.10
    spheres[:, :, 3] = 0.01
    terminal_radius = policy.expected_rod_spheres_tool[
        policy.terminal_rod_sphere_offset
    ][3]
    spheres[:, classification.tip_sphere_index, :3] = (
        tips + plane_normal * terminal_radius
    )
    spheres[:, classification.tip_sphere_index, 3] = terminal_radius
    return tips, spheres, classification


def test_lab_wall_plane_points_toward_robot_side(environment) -> None:
    policy = ContactPolicy.from_environment(environment)

    assert policy.allowed_contact_link == "rod_tip"
    assert policy.allowed_surface == "drawing_plane"
    assert policy.plane_normal_base == pytest.approx((-1.0, 0.0, 0.0), abs=1.0e-12)
    assert policy.plane_offset_base_m == pytest.approx(-0.65, abs=1.0e-12)


def test_circle_01_source_and_rod_chain_satisfy_contact_policy(
    repository_root,
    environment,
) -> None:
    policy = ContactPolicy.from_environment(environment)
    classification, _, _ = _classification(policy)
    source = read_timed_pose_csv(
        repository_root
        / "environments"
        / "lab_wall_a"
        / "trajectories"
        / "circle_01.csv"
    )
    base_tip_poses = tuple(environment.transform_pose_to_base(pose) for pose in source)
    tool_builder = RobotToolBuilder(environment)
    generated = tool_builder.collision_spheres()
    tips = np.asarray([pose.position_m for pose in base_tip_poses], dtype=float)
    plane_normal = np.asarray(policy.plane_normal_base, dtype=float)
    plane_point = plane_normal * policy.plane_offset_base_m
    spheres = np.zeros(
        (len(source), classification.total_sphere_count, 4),
        dtype=float,
    )
    spheres[:, :, :3] = plane_point + plane_normal * 0.10
    spheres[:, :, 3] = 0.01
    for sample_index, tip_pose in enumerate(base_tip_poses):
        tool_pose = tool_builder.tip_pose_to_tool_pose(tip_pose)
        base_from_tool = pose_matrix(
            tool_pose.position_m,
            tool_pose.quaternion_xyzw,
        )
        for sphere_index, sphere in zip(
            classification.rod_sphere_indices,
            generated,
            strict=True,
        ):
            center = base_from_tool @ np.asarray((*sphere.center_m, 1.0), dtype=float)
            spheres[sample_index, sphere_index, :3] = center[:3]
            spheres[sample_index, sphere_index, 3] = sphere.radius_m

    metrics = policy.validate_canonical_samples(tips, spheres, classification)

    assert metrics.checked_sample_count == len(source)
    assert metrics.maximum_tip_penetration_m < 1.0e-12
    assert metrics.minimum_non_tip_clearance_m >= 0.005


def test_tip_penetration_at_limit_passes_and_excess_is_rejected(environment) -> None:
    policy = ContactPolicy.from_environment(environment)
    tips, spheres, classification = _safe_geometry(policy, sample_count=1)
    normal = np.asarray(policy.plane_normal_base, dtype=float)
    radius = spheres[0, classification.tip_sphere_index, 3]

    tips[0] -= normal * policy.maximum_tip_penetration_m
    spheres[0, classification.tip_sphere_index, :3] = tips[0] + normal * radius
    policy.validate_canonical_samples(tips, spheres, classification)

    tips[0] -= normal * 1.0e-7
    spheres[0, classification.tip_sphere_index, :3] = tips[0] + normal * radius
    with pytest.raises(PlanningError, match="rod-tip penetration"):
        policy.validate_canonical_samples(tips, spheres, classification)


def test_tilted_terminal_sphere_is_allowed_only_while_safely_off_surface(
    repository_root,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root
        / "environments"
        / "new_board_4cm_rod33365_up50_icassp_phase2"
    )
    policy = ContactPolicy.from_environment(environment)
    tips, spheres, classification = _safe_geometry(policy, sample_count=1)
    normal = np.asarray(policy.plane_normal_base, dtype=float)
    plane_origin = np.asarray(policy.plane_origin_base_m, dtype=float)
    plane_point = plane_origin + normal * (
        policy.plane_offset_base_m - float(np.dot(normal, plane_origin))
    )
    terminal_index = classification.tip_sphere_index
    radius = spheres[0, terminal_index, 3]
    tangent = np.asarray(policy.plane_x_axis_base, dtype=float)
    tilted_back_direction = normal * np.cos(np.pi / 6.0) + tangent * np.sin(
        np.pi / 6.0
    )

    # At the Z-stack's deepest point the mathematical tip and the complete
    # terminal sphere are well above the boundary.  The sphere's plane-facing
    # point is not the mathematical tip, which is valid away from contact.
    tips[0] = plane_point + normal * 0.027
    spheres[0, terminal_index, :3] = tips[0] + tilted_back_direction * radius
    policy.validate_canonical_samples(tips, spheres, classification)

    # At the plane, the same tilted terminal sphere would touch through a
    # non-tip point first and must remain fail-closed.
    tips[0] = plane_point
    spheres[0, terminal_index, :3] = tips[0] + tilted_back_direction * radius
    with pytest.raises(
        PlanningError,
        match="only the configured rod tip touches|rod-tip penetration",
    ):
        policy.validate_canonical_samples(tips, spheres, classification)


def test_non_tip_rod_sphere_below_clearance_is_rejected(environment) -> None:
    policy = ContactPolicy.from_environment(environment)
    tips, spheres, classification = _safe_geometry(policy, sample_count=1)
    normal = np.asarray(policy.plane_normal_base, dtype=float)
    plane_point = normal * policy.plane_offset_base_m
    non_tip_rod_index = next(
        index
        for index in classification.rod_sphere_indices
        if index != classification.tip_sphere_index
    )
    radius = spheres[0, non_tip_rod_index, 3]
    spheres[0, non_tip_rod_index, :3] = (
        plane_point
        + normal * (radius + policy.minimum_non_tip_clearance_m - 1.0e-7)
    )

    with pytest.raises(PlanningError, match="non-tip rod sphere"):
        policy.validate_canonical_samples(tips, spheres, classification)


def test_robot_sphere_below_clearance_is_rejected(environment) -> None:
    policy = ContactPolicy.from_environment(environment)
    tips, spheres, classification = _safe_geometry(policy, sample_count=1)
    normal = np.asarray(policy.plane_normal_base, dtype=float)
    plane_point = normal * policy.plane_offset_base_m
    robot_index = next(
        index
        for index in classification.non_tip_sphere_indices
        if index not in classification.rod_sphere_indices
    )
    radius = spheres[0, robot_index, 3]
    spheres[0, robot_index, :3] = (
        plane_point
        + normal * (radius + policy.minimum_non_tip_clearance_m - 1.0e-7)
    )

    with pytest.raises(PlanningError, match="robot sphere"):
        policy.validate_canonical_samples(tips, spheres, classification)


def test_finite_board_ignores_external_shoulder_but_checks_overlapping_edges(
    repository_root,
) -> None:
    environment = EnvironmentBuilder().load(
        repository_root / "environments" / "new_board_4cm_rod33365"
    )
    policy = ContactPolicy.from_environment(environment)
    classification, _, _ = _classification(policy)
    base_from_plane = environment._frame_to_base("plane1")

    def plane_point(x: float, y: float, z: float) -> np.ndarray:
        return (base_from_plane @ np.asarray((x, y, z, 1.0), dtype=float))[:3]

    tips = np.asarray([plane_point(0.0, 0.0, -0.03)], dtype=float)
    spheres = np.zeros((1, classification.total_sphere_count, 4), dtype=float)
    spheres[0, :, :3] = plane_point(0.0, 0.0, -0.10)
    spheres[0, :, 3] = 0.01
    terminal_radius = policy.expected_rod_spheres_tool[
        policy.terminal_rod_sphere_offset
    ][3]
    normal = np.asarray(policy.plane_normal_base, dtype=float)
    spheres[0, classification.tip_sphere_index, :3] = (
        tips[0] + normal * terminal_radius
    )
    spheres[0, classification.tip_sphere_index, 3] = terminal_radius
    robot_index = next(
        index
        for index in classification.non_tip_sphere_indices
        if index not in classification.rod_sphere_indices
    )

    # Reproduce the ubuntu24 failure. The shoulder is behind the plane, but its
    # projection is outside the 40 x 40 cm sensor board and must not be rejected.
    spheres[0, robot_index, :3] = plane_point(
        0.27526385,
        0.38631537,
        0.09273267,
    )
    spheres[0, robot_index, 3] = 0.085
    policy.validate_canonical_samples(tips, spheres, classification)

    # A sphere whose center is outside the rectangle still overlaps the board
    # edge when its radius reaches into the configured XY range.
    spheres[0, robot_index, :3] = plane_point(0.205, 0.0, -0.0299)
    spheres[0, robot_index, 3] = 0.01
    with pytest.raises(PlanningError, match="robot sphere"):
        policy.validate_canonical_samples(tips, spheres, classification)

    # Move the same sphere beyond one radius from the edge; the finite-board
    # analytical gate no longer applies there.
    spheres[0, robot_index, :3] = plane_point(0.211, 0.0, -0.0299)
    policy.validate_canonical_samples(tips, spheres, classification)


def test_adapter_reports_contact_clearance_in_path_metrics(
    repository_root,
    environment,
) -> None:
    policy = ContactPolicy.from_environment(environment)
    tips, spheres, classification = _safe_geometry(policy, sample_count=2)
    _, tool_indices, tool_reference = _classification(policy)
    normal = np.asarray(policy.plane_normal_base, dtype=float)
    plane_point = normal * policy.plane_offset_base_m
    spheres[:, 0, :3] = plane_point + normal * 0.017
    spheres[:, 0, 3] = 0.01

    tip_quaternion = (0.0, 2.0**-0.5, 0.0, 2.0**-0.5)
    constrained_tip_poses = tuple(
        TimedPose(
            time_s=float(index),
            frame_id=environment.base_frame,
            position_m=tuple(float(value) for value in tip),
            quaternion_xyzw=tip_quaternion,
            phase="draw",
        )
        for index, tip in enumerate(tips)
    )
    tool_builder = RobotToolBuilder(environment)
    actual_tool_poses = tuple(
        tool_builder.tip_pose_to_tool_pose(pose) for pose in constrained_tip_poses
    )
    actual_tool_position = torch.tensor(
        [pose.position_m for pose in actual_tool_poses],
        dtype=torch.float32,
    )
    actual_tool_quaternion = torch.tensor(
        [
            (
                pose.quaternion_xyzw[3],
                pose.quaternion_xyzw[0],
                pose.quaternion_xyzw[1],
                pose.quaternion_xyzw[2],
            )
            for pose in actual_tool_poses
        ],
        dtype=torch.float32,
    )
    actual_pose = SimpleNamespace(
        position=actual_tool_position,
        quaternion=actual_tool_quaternion,
    )
    state = SimpleNamespace(
        tool_poses=SimpleNamespace(get_link_pose=lambda _: actual_pose),
        robot_spheres=torch.tensor(spheres[:, None, :, :], dtype=torch.float32),
    )
    kinematics_config = SimpleNamespace(
        total_spheres=classification.total_sphere_count,
        get_sphere_index_from_link_name=lambda _: torch.tensor(tool_indices),
        get_reference_link_spheres=lambda _: torch.tensor(
            tool_reference,
            dtype=torch.float32,
        ),
    )
    kinematics = SimpleNamespace(
        config=SimpleNamespace(kinematics_config=kinematics_config),
        compute_kinematics=lambda _: state,
    )
    checker = SimpleNamespace(kinematics=kinematics)
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    adapter._runtime = {"torch": _CpuTorch(), "JointState": _JointState}
    adapter._collision_checker = checker
    adapter.validate_joint_trajectory = lambda _: PathValidationMetrics(
        max_tip_position_error_m=0.0,
        max_tip_orientation_error_deg=0.0,
        minimum_collision_distance_m=0.02,
        max_scene_collision_cost=0.0,
        max_self_collision_cost=0.0,
        max_joint_bound_cost=0.0,
        checked_sample_count=2,
    )
    zero = (0.0,) * len(environment.joint_names)
    trajectory = CompiledTrajectory(
        joint_names=environment.joint_names,
        samples=(
            JointSample(0.0, zero, zero, zero, "draw"),
            JointSample(1.0, zero, zero, zero, "draw"),
        ),
    )

    metrics = adapter.validate_compiled_trajectory(
        trajectory,
        constrained_tip_poses,
        (zero, zero),
    )

    assert metrics.minimum_collision_distance_m == pytest.approx(0.007, abs=1.0e-7)


@pytest.mark.parametrize(
    "sphere_tensor",
    [
        torch.zeros((2, 3, 4), dtype=torch.float32),
        torch.zeros((1, 1, 3, 4), dtype=torch.float32),
        torch.full((1, 2, 3, 4), float("nan"), dtype=torch.float32),
    ],
)
def test_adapter_contact_sphere_shape_and_values_fail_closed(sphere_tensor) -> None:
    with pytest.raises(PlanningError, match="collision sphere geometry"):
        CuroboAdapterV2._canonical_contact_spheres(
            SimpleNamespace(robot_spheres=sphere_tensor),
            sample_count=2,
        )


def test_adapter_contact_sphere_metadata_fails_closed(
    repository_root,
    environment,
) -> None:
    adapter = CuroboAdapterV2(environment, repository_root / "curobo.lock")
    policy = ContactPolicy.from_environment(environment)

    with pytest.raises(PlanningError, match="sphere/link metadata"):
        adapter._contact_sphere_classification(
            SimpleNamespace(),
            policy,
            runtime_sphere_count=1,
        )


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda zones: zones["contact_policy"].pop(
                "maximum_tip_penetration_m"
            ),
            "maximum_tip_penetration_m",
        ),
        (
            lambda zones: zones["contact_policy"].__setitem__(
                "minimum_non_tip_clearance_m", float("nan")
            ),
            "minimum_non_tip_clearance_m",
        ),
        (
            lambda zones: zones["contact_policy"].__setitem__(
                "allowed_contact_link", "tool0"
            ),
            "allowed_contact_link",
        ),
        (
            lambda zones: zones["analysis_zones"]["drawing_plane"].__setitem__(
                "normal_xyz", [0.0, 0.0, 2.0]
            ),
            "unit vector",
        ),
        (
            lambda zones: zones["analysis_zones"]["drawing_plane"].__setitem__(
                "offset_m", float("inf")
            ),
            "offset_m",
        ),
        (
            lambda zones: zones["analysis_zones"]["drawing_plane"].__setitem__(
                "x_range_m", [-0.2, 0.2]
            ),
            "both x_range_m and y_range_m",
        ),
        (
            lambda zones: zones["analysis_zones"]["drawing_plane"].update(
                {
                    "x_range_m": [0.2, -0.2],
                    "y_range_m": [-0.2, 0.2],
                }
            ),
            "increasing finite bounds",
        ),
    ],
)
def test_invalid_contact_configuration_fails_closed(environment, mutate, message) -> None:
    zones = copy.deepcopy(environment.zones)
    mutate(zones)

    with pytest.raises(PlanningError, match=message):
        ContactPolicy.from_environment(replace(environment, zones=zones))


def test_tcp_and_rod_geometry_must_identify_one_terminal_tip_sphere(environment) -> None:
    tcp = copy.deepcopy(environment.tcp)
    tcp["rod"]["length_m"] = 0.20

    with pytest.raises(PlanningError, match="exactly one terminal sphere"):
        ContactPolicy.from_environment(replace(environment, tcp=tcp))


def test_missing_curobo_rod_sphere_mapping_fails_closed(environment) -> None:
    policy = ContactPolicy.from_environment(environment)
    _, tool_indices, tool_reference = _classification(policy)

    with pytest.raises(PlanningError, match="configured rod sphere"):
        policy.classify_curobo_spheres(
            total_sphere_count=len(tool_reference) + 1,
            tool_sphere_indices=tool_indices[:-1],
            tool_reference_spheres=tool_reference[:-1],
        )


def test_nonfinite_runtime_contact_geometry_fails_closed(environment) -> None:
    policy = ContactPolicy.from_environment(environment)
    tips, spheres, classification = _safe_geometry(policy, sample_count=1)
    spheres[0, 0, 0] = float("nan")

    with pytest.raises(PlanningError, match="non-finite"):
        policy.validate_canonical_samples(tips, spheres, classification)
