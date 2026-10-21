from __future__ import annotations

import csv
import math
import os
import shutil
from pathlib import Path
from typing import Any

import pytest
import yaml

from conftest import FakeCuroboAdapter, copy_environment_without_recordings

import hx_waypoint_curobo.recording_store as recording_store_module
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import ValidationError
from hx_waypoint_curobo.math3d import matrix_pose
from hx_waypoint_curobo.models import CompiledTrajectory, JointSample, TimedPose
from hx_waypoint_curobo.recording_store import RecordingBuilder, RecordingInspector
from hx_waypoint_curobo.robot_kinematics import (
    Pose,
    QuinticHermiteSegment,
    RobotKinematics,
    compose_poses,
    inverse_pose,
    pose_error,
)
from hx_waypoint_curobo.timed_pose_compiler import CuroboTimedPathCompiler
from hx_waypoint_curobo.trajectory_exporter import TrajectoryExporter
from hx_waypoint_curobo.trajectory_io import (
    read_joint_trajectory_csv,
    read_timed_pose_csv,
    sha256_file,
    write_timed_pose_csv,
)


_SIMULATION_FRAME_IDS = (
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tool0",
    "rod_tip",
)


def _read_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _write_yaml(path: Path, value: dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _reseal(path: Path) -> None:
    checksum_lines = [
        f"{sha256_file(candidate)}  {candidate.relative_to(path)}"
        for candidate in sorted(path.rglob("*"))
        if candidate.is_file()
        and not candidate.is_symlink()
        and candidate.name != "sha256sums.txt"
    ]
    (path / "sha256sums.txt").write_text(
        "\n".join(checksum_lines) + "\n",
        encoding="utf-8",
    )


def test_orientation_audit_comparison_absorbs_only_sub_arcsecond_acos_noise() -> None:
    assert recording_store_module._orientation_audit_metric_matches(0.0, 5.4e-6)
    assert recording_store_module._orientation_audit_metric_matches(1.9e-5, 0.0)
    assert not recording_store_module._orientation_audit_metric_matches(0.0, 1.0e-3)


def _bind_joint_hash(path: Path) -> None:
    digest = sha256_file(path / "joint_trajectory.csv")
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["joint_trajectory_sha256"] = digest
    _write_yaml(report_path, report)
    manifest_path = path / "manifest.yaml"
    manifest = _read_yaml(manifest_path)
    manifest["hashes"]["joint_trajectory"] = digest
    _write_yaml(manifest_path, manifest)
    _reseal(path)


def _bind_source_hash(path: Path) -> None:
    digest = sha256_file(path / "source_tip_trajectory.csv")
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["source_trajectory_sha256"] = digest
    _write_yaml(report_path, report)
    manifest_path = path / "manifest.yaml"
    manifest = _read_yaml(manifest_path)
    manifest["hashes"]["source_trajectory"] = digest
    _write_yaml(manifest_path, manifest)
    _reseal(path)


def _bind_validation(path: Path, validation: dict[str, Any]) -> None:
    _write_yaml(path / "validation_report.yaml", validation)
    manifest_path = path / "manifest.yaml"
    manifest = _read_yaml(manifest_path)
    manifest["validation"] = validation
    _write_yaml(manifest_path, manifest)
    _reseal(path)


def _bind_environment_hash(path: Path, relative: str) -> None:
    digest = sha256_file(path / "environment_snapshot" / relative)
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["environment_hashes"][relative] = digest
    _write_yaml(report_path, report)
    manifest_path = path / "manifest.yaml"
    manifest = _read_yaml(manifest_path)
    manifest["hashes"][relative] = digest
    _write_yaml(manifest_path, manifest)
    _reseal(path)


def _sampling_metrics(times: tuple[float, ...], sample_dt_s: float) -> dict[str, Any]:
    maximum_interval = 0.0
    maximum_jitter = 0.0
    missing_samples = 0
    for before, current in zip(times[:-1], times[1:], strict=True):
        interval = current - before
        maximum_interval = max(maximum_interval, interval)
        if interval <= sample_dt_s:
            nominal_steps = 1
            jitter = 0.0
        else:
            nominal_steps = max(1, math.floor(interval / sample_dt_s + 0.5))
            jitter = abs(interval - nominal_steps * sample_dt_s)
        maximum_jitter = max(maximum_jitter, jitter)
        missing_samples += nominal_steps - 1
    return {
        "sample_count": len(times),
        "first_trajectory_time_s": times[0],
        "last_trajectory_time_s": times[-1],
        "coverage_duration_s": times[-1] - times[0],
        "max_time_interval_s": maximum_interval,
        "max_time_jitter_s": maximum_jitter,
        "missing_sample_estimate": missing_samples,
    }


def _fixture_robot_audit(
    environment_path: Path,
    joint_names: tuple[str, ...],
) -> tuple[RobotKinematics, Pose, Pose]:
    robot_model = _read_yaml(environment_path / "robot" / "robot_model.yaml")["robot"]
    provenance = _read_yaml(environment_path / "robot" / "asset_provenance.yaml")
    urdf_path = environment_path / provenance["generated_urdf"]["path"]
    kinematics = RobotKinematics.from_urdf_bytes(
        urdf_path.read_bytes(),
        base_link=robot_model["base_link"],
        joint_names=joint_names,
    )
    environment = EnvironmentBuilder().load(environment_path)
    tcp = environment.tcp["tool"]
    tcp_translation = tuple(float(value) for value in tcp["translation_m"])
    tcp_quaternion = tuple(float(value) for value in tcp["quaternion_xyzw"])
    tool_to_tip = Pose(
        (tcp_translation[0], tcp_translation[1], tcp_translation[2]),
        (
            tcp_quaternion[0],
            tcp_quaternion[1],
            tcp_quaternion[2],
            tcp_quaternion[3],
        ),
    )
    wall_translation, wall_quaternion = matrix_pose(
        environment._frame_to_base("wall_draw_frame")
    )
    base_to_wall = Pose(
        wall_translation,
        wall_quaternion,
    )
    return kinematics, tool_to_tip, base_to_wall


def _continuous_ratios(
    trajectory: CompiledTrajectory,
    limits: dict[str, Any],
) -> tuple[float, float, float]:
    maxima = [[0.0] * len(trajectory.joint_names) for _ in range(3)]
    for start, end in zip(trajectory.samples[:-1], trajectory.samples[1:], strict=True):
        for joint in range(len(trajectory.joint_names)):
            segment = QuinticHermiteSegment.from_boundary_conditions(
                duration_s=end.time_s - start.time_s,
                start_position=start.positions[joint],
                start_velocity=start.velocities[joint],
                start_acceleration=start.accelerations[joint],
                end_position=end.positions[joint],
                end_velocity=end.velocities[joint],
                end_acceleration=end.accelerations[joint],
            )
            for derivative in range(1, 4):
                maxima[derivative - 1][joint] = max(
                    maxima[derivative - 1][joint],
                    segment.maximum_absolute(derivative).absolute_value,
    )
    limit_keys = ("velocity_rad_s", "acceleration_rad_s2", "jerk_rad_s3")
    ratios = tuple(
        max(
            value / float(limit)
            for value, limit in zip(maximum, limits[key], strict=True)
        )
        for maximum, key in zip(maxima, limit_keys, strict=True)
    )
    return ratios[0], ratios[1], ratios[2]


def _source_fk_errors(
    source: tuple[TimedPose, ...],
    trajectory: CompiledTrajectory,
    kinematics: RobotKinematics,
    tool_to_tip: Pose,
    base_to_wall: Pose,
) -> tuple[float, float]:
    samples = {round(sample.time_s * 1.0e9): sample for sample in trajectory.samples}
    maximum_position_error = 0.0
    maximum_orientation_error = 0.0
    for pose in source:
        sample = samples[round(pose.time_s * 1.0e9)]
        actual = kinematics.forward_kinematics(
            sample.positions,
            ("rod_tip",),
            tool_to_tip=tool_to_tip,
        )["rod_tip"]
        expected = compose_poses(
            base_to_wall,
            Pose(pose.position_m, pose.quaternion_xyzw),
        )
        error = pose_error(actual, expected)
        maximum_position_error = max(maximum_position_error, error.position_m)
        maximum_orientation_error = max(maximum_orientation_error, error.orientation_deg)
    return maximum_position_error, maximum_orientation_error


def _build_recording(tmp_path: Path, repository_root: Path) -> Path:
    environment_path = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        environment_path,
    )
    environment = EnvironmentBuilder().load(environment_path)
    joint_names = environment.joint_names
    home = tuple(
        float(value)
        for value in environment.robot_model["robot"]["home_joint_positions"]
    )
    goal_offsets = (0.05, -0.04, 0.03, -0.02, 0.01, 0.04)
    joint_curves = tuple(
        QuinticHermiteSegment.from_boundary_conditions(
            duration_s=1.0,
            start_position=start,
            start_velocity=0.0,
            start_acceleration=0.0,
            end_position=start + offset,
            end_velocity=0.0,
            end_acceleration=0.0,
        )
        for start, offset in zip(home, goal_offsets, strict=True)
    )
    samples = tuple(
        JointSample(
            time_s=index / 100.0,
            positions=tuple(curve.evaluate(index / 100.0, 0) for curve in joint_curves),
            velocities=tuple(curve.evaluate(index / 100.0, 1) for curve in joint_curves),
            accelerations=tuple(curve.evaluate(index / 100.0, 2) for curve in joint_curves),
            phase="draw",
        )
        for index in range(101)
    )
    trajectory = CompiledTrajectory(
        joint_names=joint_names,
        samples=samples,
        metadata={
            "joint_winding": {
                "policy": "constant_whole_path_nearest_home_without_margin_loss",
                "reference_positions_rad": list(home),
                "selected_seed_offsets_rad": [0.0] * len(joint_names),
                "post_optimization_offsets_rad": [0.0] * len(joint_names),
            }
        },
    )
    path = environment_path / "recordings" / "circle_01__20260717_120000"
    path.mkdir()
    exporter = TrajectoryExporter()
    exporter.write_joint_trajectory(path / "joint_trajectory.csv", trajectory)
    kinematics, tool_to_tip, base_to_wall = _fixture_robot_audit(
        environment_path,
        joint_names,
    )
    fk_samples = tuple(
        kinematics.forward_kinematics(
            sample.positions,
            _SIMULATION_FRAME_IDS,
            tool_to_tip=tool_to_tip,
        )
        for sample in samples
    )
    sample_by_time = {
        round(sample.time_s * 1.0e9): (sample, frames)
        for sample, frames in zip(samples, fk_samples, strict=True)
    }
    wall_from_base = inverse_pose(base_to_wall)
    fk_source_rows: list[TimedPose] = []
    for sample in samples:
        time_s = sample.time_s
        tip_in_wall = compose_poses(
            wall_from_base,
            sample_by_time[round(time_s * 1.0e9)][1]["rod_tip"],
        )
        fk_source_rows.append(
            TimedPose(
                time_s=time_s,
                frame_id="wall_draw_frame",
                position_m=tip_in_wall.position_m,
                quaternion_xyzw=tip_in_wall.quaternion_xyzw,
                phase="draw",
            )
        )
    write_timed_pose_csv(path / "source_tip_trajectory.csv", fk_source_rows)
    source = read_timed_pose_csv(path / "source_tip_trajectory.csv")
    actual_columns = (
        "ros_time_s",
        "trajectory_time_s",
        *(f"{name}_pos" for name in joint_names),
        *(f"{name}_vel" for name in joint_names),
    )

    def actual_rows():
        for sample in samples:
            row: dict[str, object] = {
                "ros_time_s": 1000.0 + sample.time_s,
                "trajectory_time_s": sample.time_s,
            }
            for name, value in zip(joint_names, sample.positions, strict=True):
                row[f"{name}_pos"] = value
            for name, value in zip(joint_names, sample.velocities, strict=True):
                row[f"{name}_vel"] = value
            yield row

    exporter.write_rows(path / "sim_actual_joint.csv", actual_columns, actual_rows())

    controller_columns = ["ros_time_s", "trajectory_time_s", "speed_scaling"]
    for prefix in ("reference", "feedback", "error"):
        controller_columns.extend(f"{prefix}_{name}_pos" for name in joint_names)
        controller_columns.extend(f"{prefix}_{name}_vel" for name in joint_names)

    def controller_rows():
        for sample in samples:
            row: dict[str, object] = {
                "ros_time_s": 1000.0 + sample.time_s,
                "trajectory_time_s": sample.time_s,
                "speed_scaling": 1.0,
            }
            for prefix in ("reference", "feedback"):
                for name, value in zip(joint_names, sample.positions, strict=True):
                    row[f"{prefix}_{name}_pos"] = value
                for name, value in zip(joint_names, sample.velocities, strict=True):
                    row[f"{prefix}_{name}_vel"] = value
            for name in joint_names:
                row[f"error_{name}_pos"] = 0.0
                row[f"error_{name}_vel"] = 0.0
            yield row

    exporter.write_rows(
        path / "sim_controller_state.csv",
        controller_columns,
        controller_rows(),
    )

    frame_columns = (
        "ros_time_s",
        "trajectory_time_s",
        "frame_id",
        "x_m",
        "y_m",
        "z_m",
        "qx",
        "qy",
        "qz",
        "qw",
    )

    def frame_rows():
        for sample, frames in zip(samples, fk_samples, strict=True):
            for frame_id in _SIMULATION_FRAME_IDS:
                pose = frames[frame_id]
                yield {
                    "ros_time_s": 1000.0 + sample.time_s,
                    "trajectory_time_s": sample.time_s,
                    "frame_id": frame_id,
                    "x_m": pose.position_m[0],
                    "y_m": pose.position_m[1],
                    "z_m": pose.position_m[2],
                    "qx": pose.quaternion_xyzw[0],
                    "qy": pose.quaternion_xyzw[1],
                    "qz": pose.quaternion_xyzw[2],
                    "qw": pose.quaternion_xyzw[3],
                }

    exporter.write_rows(path / "joint_frames.csv", frame_columns, frame_rows())
    tip_columns = (
        "ros_time_s",
        "trajectory_time_s",
        "x_m",
        "y_m",
        "z_m",
        "qx",
        "qy",
        "qz",
        "qw",
    )
    exporter.write_rows(
        path / "sim_actual_tip.csv",
        tip_columns,
        (
            {
                "ros_time_s": 1000.0 + sample.time_s,
                "trajectory_time_s": sample.time_s,
                "x_m": frames["rod_tip"].position_m[0],
                "y_m": frames["rod_tip"].position_m[1],
                "z_m": frames["rod_tip"].position_m[2],
                "qx": frames["rod_tip"].quaternion_xyzw[0],
                "qy": frames["rod_tip"].quaternion_xyzw[1],
                "qz": frames["rod_tip"].quaternion_xyzw[2],
                "qw": frames["rod_tip"].quaternion_xyzw[3],
            }
            for sample, frames in zip(samples, fk_samples, strict=True)
        ),
    )

    snapshot = path / "environment_snapshot"
    for relative in environment.hashes:
        source_path = environment_path / relative
        target = snapshot / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, target)

    sealed_trajectory = read_joint_trajectory_csv(
        path / "joint_trajectory.csv",
        joint_names,
    )
    dynamic_limits = environment.robot_model["robot"]["limits"]
    velocity_ratio, acceleration_ratio, jerk_ratio = _continuous_ratios(
        sealed_trajectory,
        dynamic_limits,
    )
    canonical_tip_position_error, canonical_tip_orientation_error = _source_fk_errors(
        source,
        sealed_trajectory,
        kinematics,
        tool_to_tip,
        base_to_wall,
    )
    actual_tip_source_position_error, actual_tip_source_orientation_error = (
        _source_fk_errors(
            source,
            trajectory,
            kinematics,
            tool_to_tip,
            base_to_wall,
        )
    )
    source_count = len(source)
    point_count = len(trajectory.samples)
    curobo_revision = FakeCuroboAdapter(environment).revision
    compile_report = {
        "schema_version": 2,
        "compiler_contract_revision": "fixed_time_waypoint_v2",
        "environment_id": environment.environment_id,
        "trajectory_id": "circle_01",
        "curobo_revision": curobo_revision,
        "source_trajectory_sha256": sha256_file(path / "source_tip_trajectory.csv"),
        "joint_trajectory_sha256": sha256_file(path / "joint_trajectory.csv"),
        "environment_hashes": dict(environment.hashes),
        "duration_s": trajectory.duration_s,
        "point_count": point_count,
        "source_waypoint_count": source_count,
        "matched_source_waypoint_count": source_count,
        "source_duration_s": source[-1].time_s,
        "max_source_time_error_s": 0.0,
        "uniform_time_scale": 1.0,
        "max_joint_velocity_ratio": velocity_ratio,
        "max_joint_acceleration_ratio": acceleration_ratio,
        "max_joint_jerk_ratio": jerk_ratio,
        "max_tip_position_error_m": canonical_tip_position_error,
        "max_tip_orientation_error_deg": canonical_tip_orientation_error,
        "minimum_collision_distance_m": 0.02,
        "max_scene_collision_cost": 0.0,
        "max_self_collision_cost": 0.0,
        "max_joint_bound_cost": 0.0,
        "checked_sample_count": point_count,
        "joint_winding": dict(trajectory.metadata["joint_winding"]),
    }
    _write_yaml(path / "compile_report.yaml", compile_report)

    command_interfaces = sorted(
        f"{joint_name}/position" for joint_name in trajectory.joint_names
    )
    state_interfaces = sorted(
        interface
        for joint_name in trajectory.joint_names
        for interface in (f"{joint_name}/position", f"{joint_name}/velocity")
    )
    recording_policy = environment.recording["recording"]
    simulation_policy = recording_policy["validation"]
    sample_dt_s = float(recording_policy["sample_dt_s"])
    sample_times = tuple(sample.time_s for sample in samples)
    sampling = _sampling_metrics(sample_times, sample_dt_s)
    assert sampling["missing_sample_estimate"] <= simulation_policy["max_missing_samples"]
    assert sampling["max_time_jitter_s"] <= simulation_policy["max_time_jitter_s"]
    joint_frame_streams = [
        {
            "frame_id": frame_id,
            **sampling,
            "max_quaternion_norm_error": 0.0,
        }
        for frame_id in sorted(_SIMULATION_FRAME_IDS)
    ]
    validation = {
        "schema_version": 1,
        "simulation_completed": True,
        "controller_result": "succeeded",
        "actual_joint_recorded": True,
        "controller_state_recorded": True,
        "joint_frames_recorded": True,
        "actual_tip_recorded": True,
        "point_count": point_count,
        "source_waypoint_count": source_count,
        "matched_source_waypoint_count": source_count,
        "checked_sample_count": point_count,
        "duration_s": trajectory.duration_s,
        "source_duration_s": source[-1].time_s,
        "max_source_time_error_s": 0.0,
        "uniform_time_scale": 1.0,
        "max_joint_velocity_ratio": velocity_ratio,
        "max_joint_acceleration_ratio": acceleration_ratio,
        "max_joint_jerk_ratio": jerk_ratio,
        "max_tip_position_error_m": canonical_tip_position_error,
        "max_tip_orientation_error_deg": canonical_tip_orientation_error,
        "minimum_collision_distance_m": 0.02,
        "max_scene_collision_cost": 0.0,
        "max_self_collision_cost": 0.0,
        "max_joint_bound_cost": 0.0,
        "sample_dt_s": sample_dt_s,
        "max_missing_samples_allowed": simulation_policy["max_missing_samples"],
        "max_time_jitter_s_allowed": simulation_policy["max_time_jitter_s"],
        "require_action_success": simulation_policy["require_action_success"],
        "actual_joint_sample_count": sampling["sample_count"],
        "actual_joint_first_trajectory_time_s": sampling[
            "first_trajectory_time_s"
        ],
        "actual_joint_last_trajectory_time_s": sampling["last_trajectory_time_s"],
        "actual_joint_coverage_duration_s": sampling["coverage_duration_s"],
        "actual_joint_max_time_interval_s": sampling["max_time_interval_s"],
        "actual_joint_max_time_jitter_s": sampling["max_time_jitter_s"],
        "actual_joint_missing_sample_estimate": sampling[
            "missing_sample_estimate"
        ],
        "controller_sample_count": sampling["sample_count"],
        "controller_first_trajectory_time_s": sampling[
            "first_trajectory_time_s"
        ],
        "controller_last_trajectory_time_s": sampling["last_trajectory_time_s"],
        "controller_coverage_duration_s": sampling["coverage_duration_s"],
        "controller_max_time_interval_s": sampling["max_time_interval_s"],
        "controller_max_time_jitter_s": sampling["max_time_jitter_s"],
        "controller_missing_sample_estimate": sampling["missing_sample_estimate"],
        "joint_frame_sample_count": sampling["sample_count"]
        * len(_SIMULATION_FRAME_IDS),
        "joint_frame_stream_count": len(_SIMULATION_FRAME_IDS),
        "max_joint_frame_quaternion_norm_error": 0.0,
        "joint_frame_streams": joint_frame_streams,
        "actual_tip_sample_count": sampling["sample_count"],
        "actual_tip_first_trajectory_time_s": sampling["first_trajectory_time_s"],
        "actual_tip_last_trajectory_time_s": sampling["last_trajectory_time_s"],
        "actual_tip_coverage_duration_s": sampling["coverage_duration_s"],
        "actual_tip_max_time_interval_s": sampling["max_time_interval_s"],
        "actual_tip_max_time_jitter_s": sampling["max_time_jitter_s"],
        "actual_tip_missing_sample_estimate": sampling["missing_sample_estimate"],
        "actual_tip_max_quaternion_norm_error": 0.0,
        "max_quaternion_norm_error_allowed": simulation_policy[
            "max_quaternion_norm_error"
        ],
        "max_completion_overrun_s_allowed": simulation_policy[
            "max_completion_overrun_s"
        ],
        "max_speed_scaling_error": 0.0,
        "max_controller_reference_position_error_rad": 0.0,
        "max_controller_reference_position_error_rad_allowed": simulation_policy[
            "max_controller_reference_position_error_rad"
        ],
        "max_controller_reference_velocity_error_rad_s": 0.0,
        "max_controller_reference_velocity_error_rad_s_allowed": simulation_policy[
            "max_controller_reference_velocity_error_rad_s"
        ],
        "max_joint_position_tracking_error_rad": 0.0,
        "max_joint_position_tracking_error_rad_allowed": simulation_policy[
            "max_joint_position_tracking_error_rad"
        ],
        "max_joint_velocity_tracking_error_rad_s": 0.0,
        "max_joint_velocity_tracking_error_rad_s_allowed": simulation_policy[
            "max_joint_velocity_tracking_error_rad_s"
        ],
        "max_actual_controller_position_error_rad": 0.0,
        "max_actual_controller_position_error_rad_allowed": simulation_policy[
            "max_actual_controller_position_error_rad"
        ],
        "max_actual_controller_velocity_error_rad_s": 0.0,
        "max_actual_controller_velocity_error_rad_s_allowed": simulation_policy[
            "max_actual_controller_velocity_error_rad_s"
        ],
        "max_frame_fk_position_error_m": 0.0,
        "max_frame_fk_position_error_m_allowed": simulation_policy[
            "max_frame_fk_position_error_m"
        ],
        "max_frame_fk_orientation_error_deg": 0.0,
        "max_frame_fk_orientation_error_deg_allowed": simulation_policy[
            "max_frame_fk_orientation_error_deg"
        ],
        "max_tip_fk_position_error_m": 0.0,
        "max_tip_fk_position_error_m_allowed": simulation_policy[
            "max_frame_fk_position_error_m"
        ],
        "max_tip_fk_orientation_error_deg": 0.0,
        "max_tip_fk_orientation_error_deg_allowed": simulation_policy[
            "max_frame_fk_orientation_error_deg"
        ],
        "max_tip_source_position_error_m": actual_tip_source_position_error,
        "max_tip_source_position_error_m_allowed": environment.planner["validation"][
            "tip_position_tolerance_m"
        ],
        "max_tip_source_orientation_error_deg": actual_tip_source_orientation_error,
        "max_tip_source_orientation_error_deg_allowed": environment.planner["validation"][
            "tip_orientation_tolerance_deg"
        ],
        "simulation_execution_contract_revision": 1,
        "speed_scaling_evidence": (
            "locked_standard_unscaled_joint_trajectory_controller"
        ),
        "simulation_controller_name": "joint_trajectory_controller",
        "simulation_controller_type": (
            "joint_trajectory_controller/JointTrajectoryController"
        ),
        "simulation_controller_state": "active",
        "simulation_controller_claimed_interfaces": command_interfaces,
        "simulation_controller_required_command_interfaces": command_interfaces,
        "simulation_controller_required_state_interfaces": state_interfaces,
        "simulation_hardware_name": environment.robot_type,
        "simulation_hardware_type": "system",
        "simulation_hardware_plugin": "mock_components/GenericSystem",
        "simulation_hardware_state_id": 3,
        "simulation_hardware_state_label": "active",
    }
    _write_yaml(path / "validation_report.yaml", validation)

    manifest = {
        "schema_version": 1,
        "recording_id": path.name,
        "status": "validated",
        "environment_id": environment.environment_id,
        "trajectory_id": "circle_01",
        "robot": {
            "type": environment.robot_type,
            "joint_names": list(trajectory.joint_names),
        },
        "trajectory": {
            "duration_s": trajectory.duration_s,
            "nominal_dt_s": samples[1].time_s - samples[0].time_s,
            "point_count": point_count,
            "source_waypoint_count": source_count,
            "matched_source_waypoint_count": source_count,
            "source_duration_s": source[-1].time_s,
            "uniform_time_scale": 1.0,
            "max_source_time_error_s": 0.0,
            "max_joint_velocity_ratio": velocity_ratio,
            "max_joint_acceleration_ratio": acceleration_ratio,
            "max_joint_jerk_ratio": jerk_ratio,
            "interpolation": "spline",
            "fields": ["position", "velocity", "acceleration"],
        },
        "validation": validation,
        "hashes": {
            **dict(environment.hashes),
            "source_trajectory": sha256_file(path / "source_tip_trajectory.csv"),
            "joint_trajectory": sha256_file(path / "joint_trajectory.csv"),
            "curobo_revision": curobo_revision,
        },
    }
    _write_yaml(path / "manifest.yaml", manifest)
    _reseal(path)
    return path


def _forge_self_consistent_joint_shift(path: Path, delta_rad: float) -> None:
    manifest_path = path / "manifest.yaml"
    manifest = _read_yaml(manifest_path)
    joint_names = tuple(manifest["robot"]["joint_names"])
    shifted_joint = joint_names[0]

    trajectory_path = path / "joint_trajectory.csv"
    with trajectory_path.open("r", encoding="utf-8", newline="") as stream:
        trajectory_rows = list(csv.reader(stream))
    trajectory_column = trajectory_rows[0].index(f"{shifted_joint}_pos")
    for row in trajectory_rows[1:]:
        row[trajectory_column] = str(float(row[trajectory_column]) + delta_rad)
    with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(trajectory_rows)

    actual_path = path / "sim_actual_joint.csv"
    with actual_path.open("r", encoding="utf-8", newline="") as stream:
        actual_rows = list(csv.reader(stream))
    actual_column = actual_rows[0].index(f"{shifted_joint}_pos")
    for row in actual_rows[1:]:
        row[actual_column] = str(float(row[actual_column]) + delta_rad)
    with actual_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(actual_rows)

    controller_path = path / "sim_controller_state.csv"
    with controller_path.open("r", encoding="utf-8", newline="") as stream:
        controller_rows = list(csv.reader(stream))
    for prefix in ("reference", "feedback"):
        column = controller_rows[0].index(f"{prefix}_{shifted_joint}_pos")
        for row in controller_rows[1:]:
            row[column] = str(float(row[column]) + delta_rad)
    with controller_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(controller_rows)

    snapshot = path / "environment_snapshot"
    kinematics, tool_to_tip, base_to_wall = _fixture_robot_audit(snapshot, joint_names)
    position_columns = [actual_rows[0].index(f"{name}_pos") for name in joint_names]
    actual_states = tuple(
        (
            float(row[actual_rows[0].index("trajectory_time_s")]),
            tuple(float(row[column]) for column in position_columns),
        )
        for row in actual_rows[1:]
    )
    fk_states = tuple(
        (
            time_s,
            kinematics.forward_kinematics(
                positions,
                _SIMULATION_FRAME_IDS,
                tool_to_tip=tool_to_tip,
            ),
        )
        for time_s, positions in actual_states
    )
    exporter = TrajectoryExporter()
    exporter.write_rows(
        path / "joint_frames.csv",
        (
            "ros_time_s",
            "trajectory_time_s",
            "frame_id",
            "x_m",
            "y_m",
            "z_m",
            "qx",
            "qy",
            "qz",
            "qw",
        ),
        (
            {
                "ros_time_s": 1000.0 + time_s,
                "trajectory_time_s": time_s,
                "frame_id": frame_id,
                "x_m": frames[frame_id].position_m[0],
                "y_m": frames[frame_id].position_m[1],
                "z_m": frames[frame_id].position_m[2],
                "qx": frames[frame_id].quaternion_xyzw[0],
                "qy": frames[frame_id].quaternion_xyzw[1],
                "qz": frames[frame_id].quaternion_xyzw[2],
                "qw": frames[frame_id].quaternion_xyzw[3],
            }
            for time_s, frames in fk_states
            for frame_id in _SIMULATION_FRAME_IDS
        ),
    )
    exporter.write_rows(
        path / "sim_actual_tip.csv",
        (
            "ros_time_s",
            "trajectory_time_s",
            "x_m",
            "y_m",
            "z_m",
            "qx",
            "qy",
            "qz",
            "qw",
        ),
        (
            {
                "ros_time_s": 1000.0 + time_s,
                "trajectory_time_s": time_s,
                "x_m": frames["rod_tip"].position_m[0],
                "y_m": frames["rod_tip"].position_m[1],
                "z_m": frames["rod_tip"].position_m[2],
                "qx": frames["rod_tip"].quaternion_xyzw[0],
                "qy": frames["rod_tip"].quaternion_xyzw[1],
                "qz": frames["rod_tip"].quaternion_xyzw[2],
                "qw": frames["rod_tip"].quaternion_xyzw[3],
            }
            for time_s, frames in fk_states
        ),
    )

    source = read_timed_pose_csv(path / "source_tip_trajectory.csv")
    fk_by_time = {round(time_s * 1.0e9): frames for time_s, frames in fk_states}
    wall_from_base = inverse_pose(base_to_wall)
    shifted_source: list[TimedPose] = []
    for pose in source:
        tip_in_wall = compose_poses(
            wall_from_base,
            fk_by_time[round(pose.time_s * 1.0e9)]["rod_tip"],
        )
        shifted_source.append(
            TimedPose(
                time_s=pose.time_s,
                frame_id=pose.frame_id,
                position_m=tip_in_wall.position_m,
                quaternion_xyzw=tip_in_wall.quaternion_xyzw,
                phase=pose.phase,
            )
        )
    write_timed_pose_csv(path / "source_tip_trajectory.csv", shifted_source)

    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["joint_trajectory_sha256"] = sha256_file(trajectory_path)
    report["source_trajectory_sha256"] = sha256_file(
        path / "source_tip_trajectory.csv"
    )
    report["max_tip_position_error_m"] = 0.0
    report["max_tip_orientation_error_deg"] = 0.0
    _write_yaml(report_path, report)

    validation = _read_yaml(path / "validation_report.yaml")
    for key in (
        "max_tip_position_error_m",
        "max_tip_orientation_error_deg",
        "max_actual_controller_position_error_rad",
        "max_actual_controller_velocity_error_rad_s",
        "max_frame_fk_position_error_m",
        "max_frame_fk_orientation_error_deg",
        "max_tip_fk_position_error_m",
        "max_tip_fk_orientation_error_deg",
        "max_tip_source_position_error_m",
        "max_tip_source_orientation_error_deg",
    ):
        validation[key] = 0.0
    _write_yaml(path / "validation_report.yaml", validation)
    manifest["validation"] = validation
    manifest["hashes"]["joint_trajectory"] = report["joint_trajectory_sha256"]
    manifest["hashes"]["source_trajectory"] = report["source_trajectory_sha256"]
    _write_yaml(manifest_path, manifest)
    _reseal(path)


def test_strict_recording_inspection(tmp_path: Path, repository_root: Path) -> None:
    path = _build_recording(tmp_path, repository_root)
    assert path.name == "circle_01__20260717_120000"
    assert not (path.parent / f".{path.name}.staging").exists()
    inspection = RecordingInspector().inspect(path)
    assert inspection.manifest["status"] == "validated"
    assert inspection.trajectory.samples
    assert (path / "environment_snapshot" / "robot" / "curobo_robot.yaml").is_file()
    assert (path / "environment_snapshot" / "robot" / "assets" / "ur5.urdf").is_file()
    assert "robot/assets/ur5.urdf" in inspection.manifest["hashes"]


def test_python_recording_finalization_is_disabled(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    environment_path = tmp_path / "lab_wall_a"
    copy_environment_without_recordings(
        repository_root / "environments" / "lab_wall_a",
        environment_path,
    )
    environment = EnvironmentBuilder().load(environment_path)
    source = read_timed_pose_csv(environment_path / "trajectories" / "circle_01.csv")
    result = CuroboTimedPathCompiler(environment, FakeCuroboAdapter(environment)).compile(source)
    builder = RecordingBuilder(environment, "circle_01")
    builder.begin()

    with pytest.raises(ValidationError, match="direct Python recording finalization is disabled"):
        builder.finalize(
            result.trajectory,
            {"simulation_completed": True, "controller_result": "succeeded"},
            result.curobo_revision,
        )


def test_hash_tamper_is_rejected(tmp_path: Path, repository_root: Path) -> None:
    path = _build_recording(tmp_path, repository_root)
    with (path / "joint_trajectory.csv").open("a", encoding="utf-8") as stream:
        stream.write("tampered\n")
    with pytest.raises(ValidationError, match="checksum mismatch"):
        RecordingInspector().inspect(path)


def test_only_validated_status_can_play(tmp_path: Path, repository_root: Path) -> None:
    path = _build_recording(tmp_path, repository_root)
    manifest_path = path / "manifest.yaml"
    manifest = _read_yaml(manifest_path)
    manifest["status"] = "building"
    _write_yaml(manifest_path, manifest)
    _reseal(path)
    with pytest.raises(ValidationError, match="status=validated"):
        RecordingInspector().inspect(path)


def test_legacy_compile_report_schema_one_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["schema_version"] = 1
    _write_yaml(report_path, report)
    _reseal(path)
    with pytest.raises(ValidationError, match="legacy compile_report schema"):
        RecordingInspector().inspect(path)


def test_wrong_joint_hash_in_compile_report_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["joint_trajectory_sha256"] = "0" * 64
    _write_yaml(report_path, report)
    _reseal(path)
    with pytest.raises(ValidationError, match="joint_trajectory_sha256"):
        RecordingInspector().inspect(path)


def test_snapshot_environment_hash_mismatch_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    planner_path = path / "environment_snapshot" / "planner.yaml"
    planner_path.write_bytes(planner_path.read_bytes() + b"\n")
    _reseal(path)
    with pytest.raises(ValidationError, match="environment_snapshot hash mismatch"):
        RecordingInspector().inspect(path)


def test_snapshot_setup_hash_mismatch_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    setup_path = path / "environment_snapshot" / "setup.yaml"
    setup_path.write_bytes(setup_path.read_bytes() + b"\n# changed setup bytes\n")
    _reseal(path)

    with pytest.raises(ValidationError, match="environment_snapshot hash mismatch"):
        RecordingInspector().inspect(path)


def test_generated_urdf_provenance_hash_must_match_the_sealed_file(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    provenance_path = path / "environment_snapshot" / "robot" / "asset_provenance.yaml"
    provenance = _read_yaml(provenance_path)
    provenance["generated_urdf"]["sha256"] = "0" * 64
    _write_yaml(provenance_path, provenance)
    _bind_environment_hash(path, "robot/asset_provenance.yaml")

    with pytest.raises(ValidationError, match="generated URDF SHA-256"):
        RecordingInspector().inspect(path)


def test_generated_urdf_is_rechecked_against_its_initial_file_seal(
    tmp_path: Path,
    repository_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    inspector = RecordingInspector()
    original_reader = inspector._read_relative_file
    mutated = False

    def mutate_before_dynamic_capture(
        recording_fd: int,
        relative: str,
        *,
        capture: bool,
    ) -> tuple[Any, bytes]:
        nonlocal mutated
        if (
            relative == "environment_snapshot/robot/assets/ur5.urdf"
            and capture
            and not mutated
        ):
            urdf_path = path / relative
            urdf_path.write_bytes(urdf_path.read_bytes() + b"\n")
            mutated = True
        return original_reader(recording_fd, relative, capture=capture)

    monkeypatch.setattr(inspector, "_read_relative_file", mutate_before_dynamic_capture)
    with pytest.raises(ValidationError, match="changed after the artifact snapshot"):
        inspector.inspect(path)


def test_non_integer_joint_winding_offset_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["joint_winding"]["selected_seed_offsets_rad"][0] = 0.5
    _write_yaml(report_path, report)
    _reseal(path)
    with pytest.raises(ValidationError, match="integer 2pi turn"):
        RecordingInspector().inspect(path)


@pytest.mark.parametrize(
    ("row_index", "message"),
    [(1, "first point"), (-1, "last point")],
)
def test_canonical_endpoint_derivatives_must_be_zero(
    tmp_path: Path,
    repository_root: Path,
    row_index: int,
    message: str,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    trajectory_path = path / "joint_trajectory.csv"
    with trajectory_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    velocity_column = rows[0].index("shoulder_pan_joint_vel")
    rows[row_index][velocity_column] = "0.01"
    with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _bind_joint_hash(path)
    with pytest.raises(ValidationError, match=message):
        RecordingInspector().inspect(path)


def test_canonical_hold_must_be_stationary(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    trajectory_path = path / "joint_trajectory.csv"
    with trajectory_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    position_column = rows[0].index("shoulder_pan_joint_pos")
    rows[2][position_column] = str(float(rows[1][position_column]) + 0.1)
    rows[2][-1] = "hold"
    with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _bind_joint_hash(path)
    with pytest.raises(ValidationError, match="hold phase must keep joint positions fixed"):
        RecordingInspector().inspect(path)


def test_canonical_hold_derivatives_must_be_zero(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    trajectory_path = path / "joint_trajectory.csv"
    with trajectory_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    position_columns = [
        index for index, name in enumerate(rows[0]) if name.endswith("_pos")
    ]
    for column in position_columns:
        rows[2][column] = rows[1][column]
    rows[2][rows[0].index("shoulder_pan_joint_vel")] = "0.01"
    rows[2][-1] = "hold"
    with trajectory_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _bind_joint_hash(path)
    with pytest.raises(ValidationError, match="hold sample"):
        RecordingInspector().inspect(path)


def test_continuous_ratios_are_recomputed_from_the_sealed_quintics(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["max_joint_velocity_ratio"] = 0.0
    _write_yaml(report_path, report)
    validation = _read_yaml(path / "validation_report.yaml")
    validation["max_joint_velocity_ratio"] = 0.0
    _write_yaml(path / "validation_report.yaml", validation)
    manifest = _read_yaml(path / "manifest.yaml")
    manifest["trajectory"]["max_joint_velocity_ratio"] = 0.0
    manifest["validation"] = validation
    _write_yaml(path / "manifest.yaml", manifest)
    _reseal(path)

    with pytest.raises(ValidationError, match="max_joint_velocity_ratio"):
        RecordingInspector().inspect(path)


def test_canonical_audit_accepts_only_bounded_cross_backend_orientation_roundoff() -> None:
    audit = recording_store_module._CanonicalAuditMetrics(
        max_joint_velocity_ratio=0.01,
        max_joint_acceleration_ratio=0.02,
        max_joint_jerk_ratio=0.0291225,
        max_tip_position_error_m=4.1e-6,
        max_tip_orientation_error_deg=2.1e-5,
    )
    metrics = {
        "max_joint_velocity_ratio": 0.01,
        "max_joint_acceleration_ratio": 0.02,
        "max_joint_jerk_ratio": 0.0291148,
        "max_tip_position_error_m": 4.3e-6,
        "max_tip_orientation_error_deg": 3.8e-5,
    }

    RecordingInspector._reconcile_canonical_audit(audit, {"metrics": metrics})

    metrics["max_joint_jerk_ratio"] = 0.0
    with pytest.raises(ValidationError, match="max_joint_jerk_ratio"):
        RecordingInspector._reconcile_canonical_audit(audit, {"metrics": metrics})

    metrics["max_joint_jerk_ratio"] = 0.0291148
    metrics["max_tip_orientation_error_deg"] = 1.0e-3
    with pytest.raises(ValidationError, match="max_tip_orientation_error_deg"):
        RecordingInspector._reconcile_canonical_audit(audit, {"metrics": metrics})


def test_self_consistent_shifted_trajectory_still_fails_urdf_bounds(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    _forge_self_consistent_joint_shift(path, 10.0)

    with pytest.raises(ValidationError, match="sealed URDF position bounds"):
        RecordingInspector().inspect(path)


def test_unchecksummed_extra_regular_file_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    (path / "unsealed.txt").write_text("not protected\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="does not protect every recording file"):
        RecordingInspector().inspect(path)


def test_duplicate_checksum_path_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    checksum_path = path / "sha256sums.txt"
    lines = checksum_path.read_text(encoding="utf-8").splitlines()
    checksum_path.write_text("\n".join([*lines, lines[0]]) + "\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="duplicate path"):
        RecordingInspector().inspect(path)


def test_unsafe_checksum_path_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    checksum_path = path / "sha256sums.txt"
    lines = checksum_path.read_text(encoding="utf-8").splitlines()
    digest = lines[0].split(maxsplit=1)[0]
    lines[0] = f"{digest}  ../manifest.yaml"
    checksum_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="unsafe path"):
        RecordingInspector().inspect(path)


def test_invalid_checksum_hash_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    checksum_path = path / "sha256sums.txt"
    lines = checksum_path.read_text(encoding="utf-8").splitlines()
    relative = lines[0][64:]
    lines[0] = "z" * 64 + relative
    checksum_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="invalid SHA-256 digest"):
        RecordingInspector().inspect(path)


def test_symlink_artifact_is_rejected(tmp_path: Path, repository_root: Path) -> None:
    path = _build_recording(tmp_path, repository_root)
    (path / "manifest_link.yaml").symlink_to(path / "manifest.yaml")
    with pytest.raises(ValidationError, match="must not contain symlinks"):
        RecordingInspector().inspect(path)


def test_recording_directory_symlink_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    alias = path.parent / "recording_alias"
    alias.symlink_to(path, target_is_directory=True)
    with pytest.raises(ValidationError, match="symlink"):
        RecordingInspector().inspect(alias)


def test_recording_directory_fd_is_closed_on_failure(
    tmp_path: Path,
    repository_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    opened: list[int] = []

    def fail_snapshot(
        self: RecordingInspector,
        recording_fd: int,
        *,
        capture_paths: frozenset[str],
    ) -> None:
        del self, capture_paths
        opened.append(recording_fd)
        raise ValidationError("forced snapshot failure")

    monkeypatch.setattr(RecordingInspector, "_snapshot_artifact", fail_snapshot)
    with pytest.raises(ValidationError, match="forced snapshot failure"):
        RecordingInspector().inspect(path)
    assert opened
    with pytest.raises(OSError):
        os.fstat(opened[0])


def test_artifact_change_during_inspection_is_rejected(
    tmp_path: Path,
    repository_root: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    original_reader = recording_store_module.read_timed_pose_csv

    def mutate_after_read(source_path: str | Path) -> tuple[Any, ...]:
        poses = original_reader(source_path)
        (path / "sim_actual_joint.csv").write_text("changed\n", encoding="utf-8")
        return poses

    monkeypatch.setattr(
        recording_store_module,
        "read_timed_pose_csv",
        mutate_after_read,
    )
    with pytest.raises(ValidationError, match="checksum mismatch"):
        RecordingInspector().inspect(path)


def test_runtime_recorded_flags_are_required(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    validation = _read_yaml(path / "validation_report.yaml")
    validation["actual_tip_recorded"] = False
    _bind_validation(path, validation)
    with pytest.raises(ValidationError, match="actual_tip_recorded must be true"):
        RecordingInspector().inspect(path)


def test_simulation_sample_count_is_recomputed_from_csv(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    validation = _read_yaml(path / "validation_report.yaml")
    validation["actual_joint_sample_count"] += 1
    _bind_validation(path, validation)
    with pytest.raises(ValidationError, match="actual_joint_sample_count"):
        RecordingInspector().inspect(path)


def test_simulation_speed_scaling_is_recomputed_from_csv(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    controller_path = path / "sim_controller_state.csv"
    with controller_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    speed_scaling_column = rows[0].index("speed_scaling")
    rows[1][speed_scaling_column] = "0.9"
    with controller_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _reseal(path)
    with pytest.raises(ValidationError, match="speed_scaling must remain exactly 1.0"):
        RecordingInspector().inspect(path)


def test_controller_tracking_error_is_recomputed_from_csv(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    controller_path = path / "sim_controller_state.csv"
    with controller_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    feedback_column = rows[0].index("feedback_shoulder_pan_joint_pos")
    rows[1][feedback_column] = str(float(rows[1][feedback_column]) + 0.5)
    with controller_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _reseal(path)
    with pytest.raises(ValidationError, match="position tracking error exceeds"):
        RecordingInspector().inspect(path)


def test_actual_joint_state_is_compared_with_controller_feedback(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    actual_path = path / "sim_actual_joint.csv"
    with actual_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    position_column = rows[0].index("shoulder_pan_joint_pos")
    rows[1][position_column] = "123"
    with actual_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _reseal(path)

    with pytest.raises(ValidationError, match="actual joint positions"):
        RecordingInspector().inspect(path)


def test_controller_reference_is_compared_with_canonical_trajectory(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    controller_path = path / "sim_controller_state.csv"
    with controller_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    reference_column = rows[0].index("reference_shoulder_pan_joint_pos")
    feedback_column = rows[0].index("feedback_shoulder_pan_joint_pos")
    shifted = str(float(rows[1][reference_column]) + 0.5)
    rows[1][reference_column] = shifted
    rows[1][feedback_column] = shifted
    with controller_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _reseal(path)
    with pytest.raises(ValidationError, match="canonical positions"):
        RecordingInspector().inspect(path)


def test_joint_frame_stream_set_is_recomputed_from_csv(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    frames_path = path / "joint_frames.csv"
    with frames_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    frame_column = rows[0].index("frame_id")
    for row in rows[1:]:
        if row[frame_column] == "rod_tip":
            row[frame_column] = "tool0"
    with frames_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _reseal(path)
    with pytest.raises(ValidationError, match="exactly the eight required frame streams"):
        RecordingInspector().inspect(path)


def test_joint_frame_poses_are_recomputed_from_actual_joint_fk(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    frames_path = path / "joint_frames.csv"
    with frames_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    for row in rows[1:]:
        for key, value in (
            ("x_m", "0"),
            ("y_m", "0"),
            ("z_m", "0"),
            ("qx", "0"),
            ("qy", "0"),
            ("qz", "0"),
            ("qw", "1"),
        ):
            row[rows[0].index(key)] = value
    with frames_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _reseal(path)

    with pytest.raises(ValidationError, match="joint_frames.csv position"):
        RecordingInspector().inspect(path)


def test_actual_tip_pose_is_recomputed_from_actual_joint_fk(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    tip_path = path / "sim_actual_tip.csv"
    with tip_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    x_column = rows[0].index("x_m")
    for row in rows[1:]:
        row[x_column] = str(float(row[x_column]) + 0.1)
    with tip_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)
    _reseal(path)

    with pytest.raises(ValidationError, match="sim_actual_tip.csv position"):
        RecordingInspector().inspect(path)


def test_source_pose_is_compared_with_time_aligned_actual_tip(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    source_path = path / "source_tip_trajectory.csv"
    with source_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.reader(stream))
    x_column = rows[0].index("tip_x_m")
    for row in rows[1:]:
        row[x_column] = str(float(row[x_column]) + 0.02)
    with source_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream).writerows(rows)

    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report["source_trajectory_sha256"] = sha256_file(source_path)
    report["max_tip_position_error_m"] = 0.02
    _write_yaml(report_path, report)
    validation = _read_yaml(path / "validation_report.yaml")
    validation["max_tip_position_error_m"] = 0.02
    _write_yaml(path / "validation_report.yaml", validation)
    manifest = _read_yaml(path / "manifest.yaml")
    manifest["hashes"]["source_trajectory"] = report["source_trajectory_sha256"]
    manifest["validation"] = validation
    _write_yaml(path / "manifest.yaml", manifest)
    _reseal(path)

    with pytest.raises(ValidationError, match="actual rod_tip position"):
        RecordingInspector().inspect(path)


def test_recorded_tip_pose_is_interpolated_to_exact_source_time() -> None:
    start = Pose(
        position_m=(0.0, 0.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    finish = Pose(
        position_m=(0.008, -0.004, 0.002),
        quaternion_xyzw=(0.0, 0.0, math.sin(0.004), math.cos(0.004)),
    )

    interpolated = RecordingInspector._interpolate_pose_stream_at_time(
        (0.0, 0.008),
        (start, finish),
        0.004,
        0.012,
    )

    assert interpolated.position_m == pytest.approx((0.004, -0.002, 0.001))
    assert interpolated.quaternion_xyzw == pytest.approx(
        (0.0, 0.0, math.sin(0.002), math.cos(0.002)),
        abs=1.0e-12,
    )


def test_simulation_allowed_thresholds_match_snapshot(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    validation = _read_yaml(path / "validation_report.yaml")
    validation["max_time_jitter_s_allowed"] = 0.001
    _bind_validation(path, validation)
    with pytest.raises(ValidationError, match="max_time_jitter_s_allowed"):
        RecordingInspector().inspect(path)


def test_fk_measured_metric_is_recomputed_from_sealed_telemetry(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    validation = _read_yaml(path / "validation_report.yaml")
    validation["max_frame_fk_position_error_m"] = 0.0005
    _bind_validation(path, validation)

    with pytest.raises(ValidationError, match="max_frame_fk_position_error_m"):
        RecordingInspector().inspect(path)


def test_fk_allowed_metric_matches_the_sealed_recording_policy(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    validation = _read_yaml(path / "validation_report.yaml")
    validation["max_tip_fk_orientation_error_deg_allowed"] = 0.2
    _bind_validation(path, validation)

    with pytest.raises(
        ValidationError,
        match="max_tip_fk_orientation_error_deg_allowed",
    ):
        RecordingInspector().inspect(path)


@pytest.mark.parametrize(
    "keys",
    [
        ("record_controller_actual",),
        ("record_joint_states",),
        ("record_joint_frames",),
        ("record_tip_state",),
        ("validation", "require_action_success"),
    ],
)
def test_snapshot_recording_must_enable_all_simulation_gates(
    tmp_path: Path,
    repository_root: Path,
    keys: tuple[str, ...],
) -> None:
    path = _build_recording(tmp_path, repository_root)
    recording_path = path / "environment_snapshot" / "recording.yaml"
    document = _read_yaml(recording_path)
    target = document["recording"]
    for key in keys[:-1]:
        target = target[key]
    target[keys[-1]] = False
    _write_yaml(recording_path, document)
    _bind_environment_hash(path, "recording.yaml")
    with pytest.raises(ValidationError, match="recording.yaml"):
        RecordingInspector().inspect(path)


def test_manifest_nominal_dt_matches_sealed_trajectory(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    manifest = _read_yaml(path / "manifest.yaml")
    manifest["trajectory"]["nominal_dt_s"] *= 2.0
    _write_yaml(path / "manifest.yaml", manifest)
    _reseal(path)
    with pytest.raises(ValidationError, match="nominal_dt_s"):
        RecordingInspector().inspect(path)


def test_simulation_controller_identity_mismatch_is_rejected(
    tmp_path: Path,
    repository_root: Path,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    validation_path = path / "validation_report.yaml"
    validation = _read_yaml(validation_path)
    validation["simulation_controller_name"] = "unsafe_controller"
    _write_yaml(validation_path, validation)
    manifest_path = path / "manifest.yaml"
    manifest = _read_yaml(manifest_path)
    manifest["validation"] = validation
    _write_yaml(manifest_path, manifest)
    _reseal(path)
    with pytest.raises(ValidationError, match="simulation_controller_name"):
        RecordingInspector().inspect(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("max_joint_velocity_ratio", 1.1, "outside"),
        ("checked_sample_count", 1, "checked_sample_count"),
    ],
)
def test_invalid_compile_metrics_are_rejected(
    tmp_path: Path,
    repository_root: Path,
    field: str,
    value: float,
    message: str,
) -> None:
    path = _build_recording(tmp_path, repository_root)
    report_path = path / "compile_report.yaml"
    report = _read_yaml(report_path)
    report[field] = value
    _write_yaml(report_path, report)
    _reseal(path)
    with pytest.raises(ValidationError, match=message):
        RecordingInspector().inspect(path)
