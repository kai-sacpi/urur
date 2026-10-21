from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import pytest
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv
from hx_waypoint_tools.tracker_capture import (
    TRACKER_RAW_COLUMNS,
    TrackerCaptureConfig,
    TrackerCaptureSession,
    TrackerCaptureSummary,
    TrackerSample,
)
from hx_waypoint_tools.yaw_calibration import (
    build_yaw_calibration_trajectory,
    build_yaw_sweep_trajectory,
    build_yaw_warmup_wave,
    finalize_yaw_tracker_capture,
    fit_yaw_offsets,
    read_yaw_calibration_metadata,
    resolve_yaw_calibration_target,
    write_authoritative_trajectory,
)

from hx_waypoint_tools import yaw_calibration


def _rotated_z_axis(quaternion_xyzw: tuple[float, ...]) -> tuple[float, float, float]:
    x, y, z, w = quaternion_xyzw
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


def test_environment_target_uses_confirmed_sensor_plane_and_distance(
    repository_root: Path,
) -> None:
    environment = repository_root / "environments/new_board_4cm_rod33365"

    center = resolve_yaw_calibration_target(environment)
    offset = resolve_yaw_calibration_target(
        environment,
        sensor_distance_m=0.070,
        offset_xy_m=(0.050, -0.010),
    )

    assert center.frame_id == "plane1"
    assert center.sensor_plane_local_z_m == pytest.approx(0.043, abs=1.0e-12)
    assert center.position_m == pytest.approx((0.0, 0.0, -0.0245), abs=1.0e-12)
    assert offset.position_m == pytest.approx((0.050, -0.010, -0.027), abs=1.0e-12)


def test_full_yaw_sweep_keeps_xyz_and_axis_fixed_with_static_boundaries() -> None:
    poses = build_yaw_sweep_trajectory(
        position_m=(0.0, 0.0, -0.0245),
        base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert len(poses) == 722
    assert poses[0].time_s == 0.0
    assert poses[360].time_s == pytest.approx(18.0)
    assert poses[720].time_s == pytest.approx(36.0)
    assert poses[-1].time_s == pytest.approx(38.0)
    assert all(pose.position_m == (0.0, 0.0, -0.0245) for pose in poses)
    assert poses[0].quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert poses[360].quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, -1.0))
    assert poses[720].quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert poses[-1].phase == "hold"
    assert poses[-1].position_m == poses[-2].position_m
    assert poses[-1].quaternion_xyzw == poses[-2].quaternion_xyzw
    reference_axis = _rotated_z_axis(poses[0].quaternion_xyzw)
    for pose in poses:
        assert _rotated_z_axis(pose.quaternion_xyzw) == pytest.approx(
            reference_axis, abs=1.0e-12
        )


def test_half_yaw_sweep_reaches_180_and_returns_without_quaternion_jump() -> None:
    poses = build_yaw_sweep_trajectory(
        position_m=(0.05, 0.0, -0.0245),
        base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        sweep_deg=180.0,
    )

    assert poses[360].quaternion_xyzw == pytest.approx((0.0, 0.0, 1.0, 0.0))
    assert poses[720].quaternion_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0))
    for left, right in zip(poses[:720], poses[1:721], strict=True):
        dot = sum(
            first * second
            for first, second in zip(
                left.quaternion_xyzw,
                right.quaternion_xyzw,
                strict=True,
            )
        )
        assert dot > 0.999


def test_warmup_is_a_separate_small_closed_wave() -> None:
    center = (0.0, 0.0, -0.0245)
    poses = build_yaw_warmup_wave(
        position_m=center,
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert poses[0].position_m == pytest.approx(center)
    assert poses[-2].position_m == pytest.approx(center, abs=1.0e-12)
    assert poses[-1].phase == "hold"
    assert max(abs(pose.position_m[0]) for pose in poses) <= 0.005 + 1.0e-12
    assert max(abs(pose.position_m[1]) for pose in poses) <= 0.0025 + 1.0e-12
    assert all(pose.position_m[2] == center[2] for pose in poses)


@pytest.mark.parametrize("sweep_deg", [180.0, 360.0])
def test_combined_authority_keeps_warmup_and_yaw_in_one_playback(
    tmp_path: Path,
    sweep_deg: float,
) -> None:
    center = (0.0, 0.0, -0.0245)
    poses = build_yaw_calibration_trajectory(
        position_m=center,
        base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        sweep_deg=sweep_deg,
    )
    path = tmp_path / "combined.csv"
    write_authoritative_trajectory(path, poses)
    metadata = read_yaw_calibration_metadata(path)

    assert len(poses) == 843
    assert poses[120].time_s == pytest.approx(6.0)
    assert poses[120].position_m == pytest.approx(center, abs=1.0e-12)
    assert poses[121].phase == "hold"
    assert poses[121].time_s == pytest.approx(8.0)
    assert poses[122].phase == "draw"
    assert poses[-1].phase == "hold"
    assert poses[-1].time_s == pytest.approx(46.0)
    assert metadata.yaw_start_s == pytest.approx(8.0)
    assert metadata.turnaround_s == pytest.approx(26.0)
    assert metadata.yaw_end_s == pytest.approx(44.0)
    assert metadata.source_end_s == pytest.approx(46.0)
    assert metadata.intended_sweep_deg == pytest.approx(sweep_deg)
    assert metadata.fixed_tip_position_m == pytest.approx(center)


def test_authority_writer_is_idempotent_and_refuses_different_bytes(
    tmp_path: Path,
) -> None:
    path = tmp_path / "yaw.csv"
    poses = build_yaw_sweep_trajectory(
        position_m=(0.0, 0.0, -0.0245),
        base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        sweep_deg=180.0,
    )

    assert write_authoritative_trajectory(path, poses) == path
    original = path.read_bytes()
    assert write_authoritative_trajectory(path, poses) == path
    assert path.read_bytes() == original
    assert len(read_timed_pose_csv(path)) == len(poses)

    changed = build_yaw_sweep_trajectory(
        position_m=(0.001, 0.0, -0.0245),
        base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        sweep_deg=180.0,
    )
    with pytest.raises(FileExistsError, match="authoritative"):
        write_authoritative_trajectory(path, changed)


@pytest.mark.parametrize(
    ("filename", "position_m", "sweep_deg"),
    [
        ("yaw_calibration_center_yaw_180deg.csv", (0.0, 0.0, -0.0245), 180.0),
        ("yaw_calibration_center_yaw_360deg.csv", (0.0, 0.0, -0.0245), 360.0),
        ("yaw_calibration_x50_yaw_180deg.csv", (0.05, 0.0, -0.0245), 180.0),
        ("yaw_calibration_x50_yaw_360deg.csv", (0.05, 0.0, -0.0245), 360.0),
    ],
)
def test_checked_in_calibration_authorities_match_deterministic_generator(
    repository_root: Path,
    tmp_path: Path,
    filename: str,
    position_m: tuple[float, float, float],
    sweep_deg: float,
) -> None:
    generated = tmp_path / filename
    write_authoritative_trajectory(
        generated,
        build_yaw_calibration_trajectory(
            position_m=position_m,
            base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            sweep_deg=sweep_deg,
        ),
    )
    checked_in = (
        repository_root
        / "environments/new_board_4cm_rod33365/trajectories"
        / filename
    )

    assert checked_in.read_bytes() == generated.read_bytes()


def test_yaw_fit_recovers_board_center_and_tool_eccentricity() -> None:
    forward = np.linspace(0.0, 2.0 * math.pi, 361)
    theta = np.concatenate((forward, forward[-2::-1]))
    board = np.asarray((0.0124, -0.0031))
    tool = np.asarray((0.0042, 0.0017))
    deltas = []
    for angle in theta:
        rotation = np.asarray(
            (
                (math.cos(angle), -math.sin(angle)),
                (math.sin(angle), math.cos(angle)),
            )
        )
        xy = board + rotation @ tool
        deltas.append((xy[0], xy[1], 0.0065))

    fit = fit_yaw_offsets(theta, deltas)

    assert fit.board_bias_xy_m == pytest.approx(board, abs=1.0e-12)
    assert fit.tool_offset_xy_m == pytest.approx(tool, abs=1.0e-12)
    assert fit.combined_bz_plus_oz_m == pytest.approx(0.0065, abs=1.0e-12)
    assert fit.board_bias_radius_m == pytest.approx(np.linalg.norm(board), abs=1.0e-12)
    assert fit.tool_offset_radius_m == pytest.approx(np.linalg.norm(tool), abs=1.0e-12)
    assert fit.xy_rmse_m < 1.0e-12
    assert fit.maximum_residual_m < 1.0e-12


def test_tracker_config_requires_absolute_topic_and_positive_limits() -> None:
    config = TrackerCaptureConfig(topic="/tracker/pose")
    assert config.frame_id == "plane1"
    assert config.max_mapping_offset_s == pytest.approx(0.05)

    with pytest.raises(ValueError, match="absolute"):
        TrackerCaptureConfig(topic="tracker/pose")
    with pytest.raises(ValueError, match="positive"):
        TrackerCaptureConfig(topic="/tracker/pose", preflight_timeout_s=0.0)


def test_tracker_preflight_recognizes_only_pose_stamped_publishers(tmp_path: Path) -> None:
    class Endpoint:
        def __init__(self, topic_type: str) -> None:
            self.topic_type = topic_type

    class Node:
        def __init__(self, topic_type: str) -> None:
            self.topic_type = topic_type

        def get_publishers_info_by_topic(self, topic: str):
            assert topic == "/tracker/pose"
            return [Endpoint(self.topic_type)]

    config = TrackerCaptureConfig(topic="/tracker/pose")
    valid = TrackerCaptureSession(tmp_path, Node("geometry_msgs/msg/PoseStamped"), config)
    wrong = TrackerCaptureSession(tmp_path, Node("geometry_msgs/msg/PointStamped"), config)

    assert valid._pose_stamped_publisher_ready() is True
    assert wrong._pose_stamped_publisher_ready() is False


def test_finalizer_maps_full_capture_and_fits_only_yaw_interval(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recording = tmp_path / "recording"
    snapshot = recording / "environment_snapshot"
    run_directory = tmp_path / "run"
    snapshot.mkdir(parents=True)
    run_directory.mkdir()
    joint_names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    (recording / "manifest.yaml").write_text(
        "robot:\n  joint_names:\n"
        + "".join(f"    - {name}\n" for name in joint_names),
        encoding="utf-8",
    )
    source_path = recording / "source_tip_trajectory.csv"
    write_authoritative_trajectory(
        source_path,
        build_yaw_calibration_trajectory(
            position_m=(0.0, 0.0, -0.0245),
            base_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            sweep_deg=360.0,
        ),
    )

    sample_period_s = 0.05
    sample_count = int(round(46.0 / sample_period_s)) + 1
    sync_origin_ns = 10_000_000_000
    sync_step_ns = 50_000_000
    robot_path = run_directory / "execution_telemetry.csv"
    robot_columns = ("sync_time_ns", "trajectory_time_s") + tuple(
        f"{name}_pos" for name in joint_names
    )
    with robot_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=robot_columns)
        writer.writeheader()
        for index in range(sample_count):
            writer.writerow(
                {
                    "sync_time_ns": sync_origin_ns + index * sync_step_ns,
                    "trajectory_time_s": f"{index * sample_period_s:.9f}",
                    **{f"{name}_pos": "0" for name in joint_names},
                }
            )

    def yaw_at(time_s: float) -> float:
        if time_s < 8.0 or time_s > 44.0:
            return 0.0
        if time_s <= 26.0:
            return 2.0 * math.pi * yaw_calibration.minimum_jerk_progress(
                (time_s - 8.0) / 18.0
            )
        return 2.0 * math.pi * (
            1.0
            - yaw_calibration.minimum_jerk_progress((time_s - 26.0) / 18.0)
        )

    def fake_actual_poses(*, rows, indices, joint_names, snapshot):
        del joint_names, snapshot
        result = {}
        for index in set(indices):
            yaw = yaw_at(float(rows[index]["trajectory_time_s"]))
            result[index] = yaw_calibration.Pose(
                (0.0, 0.0, -0.0245),
                (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)),
            )
        return result

    monkeypatch.setattr(yaw_calibration, "_actual_plane1_tip_poses", fake_actual_poses)
    board = (0.0124, -0.0031)
    tool = (0.0042, 0.0017)
    tracker_samples = []
    for index in range(sample_count):
        time_s = index * sample_period_s
        yaw = yaw_at(time_s)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        delta_x = board[0] + cosine * tool[0] - sine * tool[1]
        delta_y = board[1] + sine * tool[0] + cosine * tool[1]
        tracker_samples.append(
            TrackerSample(
                tracker_rx_sync_time_ns=sync_origin_ns + index * sync_step_ns,
                tracker_rx_ros_time_ns=20_000_000_000 + index * sync_step_ns,
                tracker_message_ros_time_ns=30_000_000_000 + index * sync_step_ns,
                frame_id="plane1",
                position_m=(delta_x, delta_y, -0.0245 + 0.0065),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
        )
    staging_path = tmp_path / "tracker.staging.csv"
    with staging_path.open("w", encoding="utf-8", newline="") as stream:
        csv.DictWriter(stream, fieldnames=TRACKER_RAW_COLUMNS).writeheader()
    summary = TrackerCaptureSummary(
        config=TrackerCaptureConfig(topic="/tracker/pose"),
        staging_path=staging_path,
        samples=tuple(tracker_samples),
        invalid_message_count=0,
        started_sync_time_ns=sync_origin_ns,
        stopped_sync_time_ns=sync_origin_ns + (sample_count - 1) * sync_step_ns,
    )

    result = finalize_yaw_tracker_capture(summary, robot_path, recording)

    assert result.sample_count == sample_count
    assert result.fit.board_bias_xy_m == pytest.approx(board, abs=1.0e-10)
    assert result.fit.tool_offset_xy_m == pytest.approx(tool, abs=1.0e-10)
    assert result.fit.combined_bz_plus_oz_m == pytest.approx(0.0065, abs=1.0e-12)
    assert result.fit.xy_rmse_m < 1.0e-10
    assert result.tracker_raw_path.is_file()
    assert result.tracker_telemetry_path.is_file()
    assert result.report_path.is_file()
    assert not staging_path.exists()
