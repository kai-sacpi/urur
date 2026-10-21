from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from hx_waypoint_curobo.environment_builder import EnvironmentBundle
from hx_waypoint_curobo.errors import WaypointError
from hx_waypoint_curobo.trajectory_io import sha256_file
from hx_waypoint_tools.cli import _compile_report_payload


def test_compile_report_binds_source_environment_and_dynamic_metrics(
    tmp_path: Path,
    environment: EnvironmentBundle,
) -> None:
    source = tmp_path / "source_tip_trajectory.csv"
    source.write_text("immutable source bytes\n", encoding="utf-8")
    joint = tmp_path / "joint_trajectory.csv"
    joint.write_text("immutable joint bytes\n", encoding="utf-8")
    result = SimpleNamespace(
        curobo_revision="a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9",
        source_poses=(SimpleNamespace(time_s=0.0), SimpleNamespace(time_s=3.5)),
        trajectory=SimpleNamespace(
            duration_s=3.5,
            samples=(SimpleNamespace(time_s=0.0), SimpleNamespace(time_s=3.5)),
            metadata={
                "joint_winding": {
                    "policy": "constant_whole_path_nearest_home_without_margin_loss",
                    "reference_positions_rad": [0.0] * 6,
                    "selected_seed_offsets_rad": [0.0] * 6,
                    "post_optimization_offsets_rad": [0.0] * 6,
                }
            },
        ),
        metrics=SimpleNamespace(
            uniform_time_scale=1.0,
            max_joint_velocity_ratio=0.4,
            max_joint_acceleration_ratio=0.5,
            max_joint_jerk_ratio=0.6,
        ),
        validation_metrics=SimpleNamespace(
            max_tip_position_error_m=0.0002,
            max_tip_orientation_error_deg=0.2,
            minimum_collision_distance_m=0.01,
            max_scene_collision_cost=0.0,
            max_self_collision_cost=0.0,
            max_joint_bound_cost=0.0,
            checked_sample_count=2,
        ),
    )

    report = _compile_report_payload(environment, "line", result, source, joint)

    assert report["schema_version"] == 2
    assert report["compiler_contract_revision"] == "fixed_time_waypoint_v2"
    assert report["source_trajectory_sha256"] == sha256_file(source)
    assert report["joint_trajectory_sha256"] == sha256_file(joint)
    assert report["environment_hashes"] == dict(environment.hashes)
    assert report["environment_hashes"]["planner.yaml"] == environment.hashes["planner.yaml"]
    assert report["source_waypoint_count"] == 2
    assert report["matched_source_waypoint_count"] == 2
    assert report["source_duration_s"] == 3.5
    assert report["max_source_time_error_s"] == 0.0
    assert report["uniform_time_scale"] == 1.0
    assert report["max_joint_velocity_ratio"] == 0.4
    assert report["max_joint_acceleration_ratio"] == 0.5
    assert report["max_joint_jerk_ratio"] == 0.6
    assert report["max_tip_position_error_m"] == 0.0002
    assert report["max_tip_orientation_error_deg"] == 0.2
    assert report["minimum_collision_distance_m"] == 0.01
    assert report["max_scene_collision_cost"] == 0.0
    assert report["max_self_collision_cost"] == 0.0
    assert report["max_joint_bound_cost"] == 0.0
    assert report["checked_sample_count"] == 2
    assert report["joint_winding"] == result.trajectory.metadata["joint_winding"]

    canonical_audit = SimpleNamespace(
        max_joint_velocity_ratio=0.41,
        max_joint_acceleration_ratio=0.51,
        max_joint_jerk_ratio=0.61,
        max_tip_position_error_m=0.00021,
        max_tip_orientation_error_deg=0.21,
    )
    serialized_report = _compile_report_payload(
        environment,
        "line",
        result,
        source,
        joint,
        canonical_audit=canonical_audit,
    )
    assert serialized_report["max_joint_velocity_ratio"] == 0.41
    assert serialized_report["max_joint_acceleration_ratio"] == 0.51
    assert serialized_report["max_joint_jerk_ratio"] == 0.61
    assert serialized_report["max_tip_position_error_m"] == 0.00021
    assert serialized_report["max_tip_orientation_error_deg"] == 0.21


def test_compile_report_rejects_any_time_scale(
    tmp_path: Path,
    environment: EnvironmentBundle,
) -> None:
    source = tmp_path / "source_tip_trajectory.csv"
    source.write_text("immutable source bytes\n", encoding="utf-8")
    result = SimpleNamespace(
        metrics=SimpleNamespace(uniform_time_scale=1.0001),
    )

    with pytest.raises(WaypointError, match="must be exactly 1.0"):
        _compile_report_payload(environment, "line", result, source, source)


def test_compile_report_rejects_missing_authoritative_timestamp(
    tmp_path: Path,
    environment: EnvironmentBundle,
) -> None:
    source = tmp_path / "source_tip_trajectory.csv"
    source.write_text("immutable source bytes\n", encoding="utf-8")
    result = SimpleNamespace(
        curobo_revision="a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9",
        source_poses=(SimpleNamespace(time_s=0.0), SimpleNamespace(time_s=1.0)),
        trajectory=SimpleNamespace(
            duration_s=1.0,
            samples=(SimpleNamespace(time_s=0.0), SimpleNamespace(time_s=0.5)),
        ),
        metrics=SimpleNamespace(uniform_time_scale=1.0),
    )

    with pytest.raises(WaypointError, match="every authoritative source time exactly"):
        _compile_report_payload(environment, "line", result, source, source)
