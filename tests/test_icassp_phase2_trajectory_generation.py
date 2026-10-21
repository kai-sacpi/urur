from __future__ import annotations

import csv
import hashlib
from pathlib import Path

import numpy as np
import pytest
import yaml

import hx_waypoint_tools.icassp_phase2 as phase2_module
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv
from hx_waypoint_tools.icassp_phase2 import (
    PHASE2_ENVIRONMENT_ID,
    Phase2GenerationError,
    generate_phase2_authorities,
)


CONDITION_COLUMNS = (
    "condition_id",
    "phase",
    "location_id",
    "direction_id",
    "p_A_x_m",
    "p_A_y_m",
    "p_A_z_m",
    "moment_q_A_x",
    "moment_q_A_y",
    "moment_q_A_z",
    "quat_A_M_x",
    "quat_A_M_y",
    "quat_A_M_z",
    "quat_A_M_w",
    "settle_s",
    "record_window_s",
    "use_in_main_statistics",
    "status",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_conditions(
    path: Path,
    *,
    preview: bool = False,
    position: tuple[float, float, float] = (0.02, -0.03, 0.13),
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CONDITION_COLUMNS)
        writer.writeheader()
        writer.writerow(
            {
                "condition_id": "MAIN_C_DIF1_Z13",
                "phase": (
                    "held_out_nominal_preview" if preview else "hardware_frozen_main"
                ),
                "location_id": "C",
                "direction_id": "C_DIF1",
                "p_A_x_m": f"{position[0]:.9f}",
                "p_A_y_m": f"{position[1]:.9f}",
                "p_A_z_m": f"{position[2]:.9f}",
                "moment_q_A_x": "0.000000000",
                "moment_q_A_y": "0.000000000",
                "moment_q_A_z": "1.000000000",
                "quat_A_M_x": f"{quaternion[0]:.12f}",
                "quat_A_M_y": f"{quaternion[1]:.12f}",
                "quat_A_M_z": f"{quaternion[2]:.12f}",
                "quat_A_M_w": f"{quaternion[3]:.12f}",
                "settle_s": "1.0",
                "record_window_s": "2.0",
                "use_in_main_statistics": "0" if preview else "1",
                "status": (
                    "DO_NOT_FREEZE_AS_FINAL_UNTIL_SIGMA_HW_RECOMPUTE"
                    if preview
                    else "HARDWARE_FROZEN"
                ),
            }
        )


def _registration_data(
    *,
    registered: bool = True,
    improper: bool = False,
) -> dict[str, object]:
    rotation = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    if improper:
        rotation[0][0] = -1.0
    plane1_from_array = [
        [*rotation[0], 0.01],
        [*rotation[1], 0.02],
        [*rotation[2], -0.01],
        [0.0, 0.0, 0.0, 1.0],
    ]
    rod_tip_from_magnet = [
        [0.0, -1.0, 0.0, 0.001],
        [1.0, 0.0, 0.0, 0.002],
        [0.0, 0.0, 1.0, 0.005],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return {
        "schema_version": 1,
        "environment_id": PHASE2_ENVIRONMENT_ID,
        "status": "HARDWARE_FROZEN" if registered else "UNREGISTERED",
        "use_in_main_statistics": bool(registered),
        "transform_convention": "T_X_from_Y_column_vectors_metres",
        "magnet_id": "test_magnet",
        "polarity": "+Z_M_is_recorded_moment_q_A",
        "roll_policy": "frozen_in_quat_A_M",
        "T_plane1_from_array": plane1_from_array if registered else None,
        "T_rod_tip_from_magnet": rod_tip_from_magnet if registered else None,
    }


def _write_registration(
    path: Path,
    *,
    registered: bool = True,
    improper: bool = False,
) -> None:
    path.write_text(
        yaml.safe_dump(
            _registration_data(registered=registered, improper=improper),
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def _environment_directory(repository_root: Path) -> Path:
    return (
        repository_root
        / "environments"
        / "new_board_4cm_rod33365_up50_icassp_phase2"
    )


def test_formal_generation_applies_registered_se3_and_writes_sealed_bundle(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    conditions = tmp_path / "source" / "conditions.csv"
    registration = tmp_path / "source" / "registration.yaml"
    output_root = tmp_path / "authorities"
    _write_conditions(conditions)
    _write_registration(registration)

    result = generate_phase2_authorities(
        conditions_path=conditions,
        registration_path=registration,
        environment_directory=_environment_directory(repository_root),
        output_root=output_root,
        expected_condition_count=1,
    )

    assert result.mode == "formal"
    assert result.mode_directory == output_root / "formal"
    assert len(result.authority_paths) == 1
    authority_path = result.authority_paths[0]
    assert authority_path.name == "MAIN_C_DIF1_Z13.csv"
    poses = read_timed_pose_csv(authority_path)
    assert [pose.time_s for pose in poses] == [0.0, 1.0, 3.0]
    assert [pose.phase for pose in poses] == ["draw", "hold", "hold"]
    assert {pose.frame_id for pose in poses} == {"plane1"}

    # T_P_R = T_P_A @ T_A_M @ inv(T_R_M). T_P_A uses Rx(pi), while
    # T_R_M uses Rz(pi/2), so this also protects the non-commuting rotation
    # order.  -Rz(-pi/2)t_R_M = [-.002, .001, -.005], then T_P_A gives
    # p_P_R=[.028, .049, -.135] and R_P_R=Rx(pi)Rz(-pi/2).
    assert poses[0].position_m == pytest.approx((0.028, 0.049, -0.135), abs=1.0e-12)
    assert poses[0].quaternion_xyzw == pytest.approx(
        (2.0**-0.5, 2.0**-0.5, 0.0, 0.0),
        abs=1e-12,
    )
    assert all(pose.position_m == poses[0].position_m for pose in poses)
    assert all(pose.quaternion_xyzw == poses[0].quaternion_xyzw for pose in poses)

    with authority_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    pose_fields = (
        "frame_id",
        "tip_x_m",
        "tip_y_m",
        "tip_z_m",
        "tip_qx",
        "tip_qy",
        "tip_qz",
        "tip_qw",
    )
    assert all(
        tuple(row[field] for field in pose_fields)
        == tuple(rows[0][field] for field in pose_fields)
        for row in rows
    )

    copied_conditions = result.mode_directory / "inputs" / "conditions.csv"
    copied_registration = result.mode_directory / "inputs" / "registration.yaml"
    assert copied_conditions.read_bytes() == conditions.read_bytes()
    assert copied_registration.read_bytes() == registration.read_bytes()

    with result.index_path.open("r", encoding="utf-8", newline="") as stream:
        index_rows = list(csv.DictReader(stream))
    assert len(index_rows) == 1
    index_row = index_rows[0]
    assert index_row["authority_file"] == authority_path.name
    assert index_row["authority_sha256"] == _sha256(authority_path)
    assert index_row["input_conditions_sha256"] == _sha256(conditions)
    assert index_row["registration_sha256"] == _sha256(registration)
    assert index_row["environment_id"] == PHASE2_ENVIRONMENT_ID
    assert index_row["mode"] == "formal"
    assert index_row["use_in_main_statistics"] == "true"

    manifest = yaml.safe_load(result.manifest_path.read_text(encoding="utf-8"))
    environment = EnvironmentBuilder().load(_environment_directory(repository_root))
    assert manifest["environment_id"] == PHASE2_ENVIRONMENT_ID
    assert manifest["mode"] == "formal"
    assert manifest["use_in_main_statistics"] is True
    assert manifest["condition_count"] == manifest["expected_condition_count"] == 1
    assert manifest["input_conditions"] == {
        "path": "inputs/conditions.csv",
        "sha256": _sha256(conditions),
    }
    assert manifest["registration"]["path"] == "inputs/registration.yaml"
    assert manifest["registration"]["sha256"] == _sha256(registration)
    assert manifest["registration"]["status"] == "HARDWARE_FROZEN"
    assert manifest["registration"]["magnet_id"] == "test_magnet"
    assert manifest["environment"]["id"] == PHASE2_ENVIRONMENT_ID
    assert manifest["environment"]["hashes"] == dict(environment.hashes)
    assert manifest["environment"]["tree_sha256"] == index_row["environment_sha256"]
    expected_tree_payload = b"".join(
        f"{relative}\0{digest}\n".encode("utf-8")
        for relative, digest in sorted(environment.hashes.items())
    )
    assert index_row["environment_sha256"] == hashlib.sha256(
        expected_tree_payload
    ).hexdigest()
    assert manifest["authority_index"] == {
        "path": "phase2_authority_index.csv",
        "sha256": _sha256(result.index_path),
    }
    assert manifest["entries"] == [
        {
            "condition_id": "MAIN_C_DIF1_Z13",
            "authority_file": authority_path.name,
            "authority_sha256": _sha256(authority_path),
            "mode": "formal",
            "use_in_main_statistics": True,
        }
    ]


def test_preview_is_rejected_by_default_and_isolated_when_explicit(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    conditions = tmp_path / "conditions.csv"
    registration = tmp_path / "registration.yaml"
    output_root = tmp_path / "authorities"
    _write_conditions(conditions, preview=True)
    _write_registration(registration)

    with pytest.raises(Phase2GenerationError, match="preview.*--exploratory"):
        generate_phase2_authorities(
            conditions_path=conditions,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=output_root,
            expected_condition_count=1,
        )
    assert not output_root.exists()

    result = generate_phase2_authorities(
        conditions_path=conditions,
        registration_path=registration,
        environment_directory=_environment_directory(repository_root),
        output_root=output_root,
        expected_condition_count=1,
        exploratory=True,
    )
    assert result.mode == "exploratory"
    assert result.mode_directory == output_root / "exploratory"
    assert result.authority_paths[0].name == "MAIN_C_DIF1_Z13__exploratory.csv"
    assert not (output_root / "formal").exists()
    manifest = yaml.safe_load(result.manifest_path.read_text(encoding="utf-8"))
    assert manifest["mode"] == "exploratory"
    assert manifest["use_in_main_statistics"] is False
    assert manifest["source_contains_preview"] is True
    with result.index_path.open("r", encoding="utf-8", newline="") as stream:
        row = next(csv.DictReader(stream))
    assert row["mode"] == "exploratory"
    assert row["use_in_main_statistics"] == "false"
    assert "exploratory" in row["authority_file"]


@pytest.mark.parametrize(
    ("registered", "improper", "message"),
    (
        (False, False, "T_plane1_from_array.*registered"),
        (True, True, "proper SE\\(3\\).*determinant"),
    ),
)
def test_registration_is_fail_closed(
    repository_root: Path,
    tmp_path: Path,
    registered: bool,
    improper: bool,
    message: str,
) -> None:
    conditions = tmp_path / "conditions.csv"
    registration = tmp_path / "registration.yaml"
    _write_conditions(conditions)
    _write_registration(registration, registered=registered, improper=improper)

    with pytest.raises(Phase2GenerationError, match=message):
        generate_phase2_authorities(
            conditions_path=conditions,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=tmp_path / "output",
            expected_condition_count=1,
        )


def test_quaternion_count_and_tcp_floor_are_hard_gates(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    registration = tmp_path / "registration.yaml"
    _write_registration(registration)

    bad_quaternion = tmp_path / "bad_quaternion.csv"
    _write_conditions(bad_quaternion, quaternion=(0.0, 0.0, 0.0, 2.0))
    with pytest.raises(Phase2GenerationError, match="unit quaternion"):
        generate_phase2_authorities(
            conditions_path=bad_quaternion,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=tmp_path / "bad_quaternion_output",
            expected_condition_count=1,
        )

    valid = tmp_path / "valid.csv"
    _write_conditions(valid)
    with pytest.raises(Phase2GenerationError, match="condition count"):
        generate_phase2_authorities(
            conditions_path=valid,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=tmp_path / "bad_count_output",
            expected_condition_count=94,
        )

    unsafe_registration = tmp_path / "unsafe_registration.yaml"
    unsafe = _registration_data()
    unsafe["T_plane1_from_array"] = np.eye(4).tolist()
    unsafe_registration.write_text(
        yaml.safe_dump(unsafe, sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(Phase2GenerationError, match="TCP floor.*-0.072"):
        generate_phase2_authorities(
            conditions_path=valid,
            registration_path=unsafe_registration,
            environment_directory=_environment_directory(repository_root),
            output_root=tmp_path / "unsafe_floor_output",
            expected_condition_count=1,
        )


def test_generation_is_idempotent_but_never_overwrites_different_authority(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    conditions = tmp_path / "conditions.csv"
    registration = tmp_path / "registration.yaml"
    output_root = tmp_path / "authorities"
    _write_conditions(conditions)
    _write_registration(registration)
    arguments = {
        "conditions_path": conditions,
        "registration_path": registration,
        "environment_directory": _environment_directory(repository_root),
        "output_root": output_root,
        "expected_condition_count": 1,
    }

    first = generate_phase2_authorities(**arguments)
    second = generate_phase2_authorities(**arguments)
    assert second == first

    authority_path = first.authority_paths[0]
    authority_path.write_bytes(b"different authoritative bytes\n")
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        generate_phase2_authorities(**arguments)
    assert authority_path.read_bytes() == b"different authoritative bytes\n"


def test_condition_id_rejects_runtime_unsafe_dot(
    repository_root: Path,
    tmp_path: Path,
) -> None:
    conditions = tmp_path / "conditions.csv"
    registration = tmp_path / "registration.yaml"
    _write_conditions(conditions)
    text = conditions.read_text(encoding="utf-8").replace(
        "MAIN_C_DIF1_Z13",
        "MAIN.C.DIF1.Z13",
    )
    conditions.write_text(text, encoding="utf-8")
    _write_registration(registration)

    with pytest.raises(Phase2GenerationError, match="safe authority filename"):
        generate_phase2_authorities(
            conditions_path=conditions,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=tmp_path / "output",
            expected_condition_count=1,
        )


def test_failed_atomic_publish_never_exposes_partial_mode_directory(
    repository_root: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    conditions = tmp_path / "conditions.csv"
    registration = tmp_path / "registration.yaml"
    output_root = tmp_path / "authorities"
    _write_conditions(conditions)
    _write_registration(registration)

    def fail_rename(source: Path, destination: Path) -> None:
        raise OSError(f"injected rename failure: {source} -> {destination}")

    monkeypatch.setattr(phase2_module.os, "rename", fail_rename)
    with pytest.raises(FileExistsError, match="atomically publish"):
        generate_phase2_authorities(
            conditions_path=conditions,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=output_root,
            expected_condition_count=1,
        )

    assert not (output_root / "formal").exists()
    assert list(output_root.glob(".formal.staging-*")) == []


@pytest.mark.parametrize(
    "reserved_id",
    ("phase2_authority_index", "PHASE2_AUTHORITY_INDEX"),
)
def test_condition_id_cannot_collide_with_bundle_metadata(
    repository_root: Path,
    tmp_path: Path,
    reserved_id: str,
) -> None:
    conditions = tmp_path / "conditions.csv"
    registration = tmp_path / "registration.yaml"
    _write_conditions(conditions)
    conditions.write_text(
        conditions.read_text(encoding="utf-8").replace(
            "MAIN_C_DIF1_Z13",
            reserved_id,
        ),
        encoding="utf-8",
    )
    _write_registration(registration)

    with pytest.raises(Phase2GenerationError, match="reserved authority bundle path"):
        generate_phase2_authorities(
            conditions_path=conditions,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=tmp_path / "output",
            expected_condition_count=1,
        )
    assert not (tmp_path / "output" / "formal").exists()


@pytest.mark.parametrize(
    "duplicate_yaml",
    (
        "status: HARDWARE_FROZEN\n",
        "T_plane1_from_array: null\n",
    ),
)
def test_registration_rejects_duplicate_authority_keys(
    repository_root: Path,
    tmp_path: Path,
    duplicate_yaml: str,
) -> None:
    conditions = tmp_path / "conditions.csv"
    registration = tmp_path / "registration.yaml"
    _write_conditions(conditions)
    _write_registration(registration)
    registration.write_text(
        registration.read_text(encoding="utf-8") + duplicate_yaml,
        encoding="utf-8",
    )

    with pytest.raises(
        Phase2GenerationError,
        match=r"(?s)cannot parse.*duplicate key",
    ):
        generate_phase2_authorities(
            conditions_path=conditions,
            registration_path=registration,
            environment_directory=_environment_directory(repository_root),
            output_root=tmp_path / "output",
            expected_condition_count=1,
        )


def test_checked_in_registration_template_is_deliberately_unregistered(
    repository_root: Path,
) -> None:
    template = (
        _environment_directory(repository_root)
        / "experiment_registration.template.yaml"
    )
    data = yaml.safe_load(template.read_text(encoding="utf-8"))

    assert data["environment_id"] == PHASE2_ENVIRONMENT_ID
    assert data["status"] == "UNREGISTERED"
    assert data["use_in_main_statistics"] is False
    assert data["T_plane1_from_array"] is None
    assert data["T_rod_tip_from_magnet"] is None


def test_checked_in_exploratory_registration_reproduces_previous_center_pose(
    repository_root: Path,
) -> None:
    registration_path = (
        _environment_directory(repository_root)
        / "experiment_registration.exploratory.yaml"
    )
    data = yaml.safe_load(registration_path.read_text(encoding="utf-8"))
    transform_P_A = np.asarray(data["T_plane1_from_array"], dtype=float)
    transform_R_M = np.asarray(data["T_rod_tip_from_magnet"], dtype=float)

    assert data["status"] == "EXPLORATORY"
    assert data["use_in_main_statistics"] is False
    assert np.linalg.det(transform_P_A[:3, :3]) == pytest.approx(1.0, abs=1.0e-12)
    assert np.linalg.det(transform_R_M[:3, :3]) == pytest.approx(1.0, abs=1.0e-12)
    assert transform_P_A[:3, 3] == pytest.approx((0.0, 0.0, -0.007), abs=1.0e-12)
    assert transform_P_A[:3, :3] @ np.asarray([0.0, 0.0, 1.0]) == pytest.approx(
        (0.0, 0.0, -1.0),
        abs=1.0e-12,
    )
    assert transform_R_M[:3, :3] @ np.asarray([0.0, 0.0, 1.0]) == pytest.approx(
        (0.0, 0.0, -1.0),
        abs=1.0e-12,
    )

    # At array center with q=+Z_A, the two inherited Rx(pi) rotations cancel.
    # This reproduces the historical rod_tip identity orientation while the
    # magnet center lies 110 mm above the current sensor surface.
    transform_A_M = np.eye(4)
    transform_A_M[2, 3] = 0.11
    transform_P_R = transform_P_A @ transform_A_M @ np.linalg.inv(transform_R_M)
    assert transform_P_R[:3, :3] == pytest.approx(np.eye(3), abs=1.0e-12)
    assert transform_P_R[:3, 3] == pytest.approx((0.0, 0.0, -0.117), abs=1.0e-12)
