from __future__ import annotations

import csv
import os
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from pathlib import Path

import pytest
import yaml

import scripts.build_icassp_phase2_recordings as builder
from hx_waypoint_tools.icassp_phase2 import generate_phase2_authorities


def _write_csv(path: Path, fieldnames: Sequence[str], rows: Sequence[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _make_workspace(root: Path) -> Path:
    workspace = root / "workspace"
    environment = workspace / "environments" / builder.PHASE2_ENVIRONMENT_ID
    (environment / "trajectories").mkdir(parents=True)
    (environment / "recordings").mkdir()
    (environment / "meshes").mkdir()
    (environment / "robot" / "assets").mkdir(parents=True)
    (workspace / "runs").mkdir()
    commit = "a" * 40
    source_tree = "b" * 40
    (workspace / "curobo.lock").write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "commit": commit,
                "source_tree": source_tree,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    curobo_source = workspace / ".deps" / f"curobo-{commit}"
    (curobo_source / "curobo").mkdir(parents=True)
    (curobo_source / "curobo" / "__init__.py").write_text(
        "# test locked cuRobo package\n",
        encoding="utf-8",
    )
    (curobo_source / ".ur-waypoint-curobo-commit").write_text(
        commit + "\n",
        encoding="utf-8",
    )
    (curobo_source / ".ur-waypoint-curobo-tree").write_text(
        source_tree + "\n",
        encoding="utf-8",
    )
    required = (
        *builder.ROOT_CONFIG_FILES,
        *builder.ROBOT_CONFIG_FILES,
        *builder.ROBOT_DATA_FILES,
    )
    for relative in required:
        path = environment / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"schema_version": 1, "fixture": relative}
        if relative == "environment.yaml":
            payload["environment_id"] = builder.PHASE2_ENVIRONMENT_ID
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return workspace


def _make_authority_bundle(
    root: Path,
    workspace: Path,
    *,
    exploratory: bool = False,
    preview: bool = False,
) -> tuple[Path, Path]:
    mode = "exploratory" if exploratory else "formal"
    bundle_root = root / "authority" / mode
    inputs = bundle_root / "inputs"
    inputs.mkdir(parents=True)
    condition_id = "MAIN_C_DIF1_Z11"
    conditions_path = inputs / "conditions.csv"
    phase = "held_out_nominal_preview" if preview else "hardware_frozen_main"
    status = "DO_NOT_FREEZE_AS_FINAL" if preview else "HARDWARE_FROZEN"
    _write_csv(
        conditions_path,
        ("condition_id", "phase", "status", "use_in_main_statistics"),
        [
            {
                "condition_id": condition_id,
                "phase": phase,
                "status": status,
                "use_in_main_statistics": "false" if exploratory else "true",
            }
        ],
    )
    registration_path = inputs / "registration.yaml"
    registration_path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "environment_id": builder.PHASE2_ENVIRONMENT_ID,
                "status": "EXPLORATORY" if exploratory else "HARDWARE_FROZEN",
                "use_in_main_statistics": not exploratory,
                "magnet_id": "test_magnet",
                "polarity": "+Z_M_is_recorded_moment_q_A",
                "roll_policy": "frozen_in_quat_A_M",
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    suffix = "__exploratory.csv" if exploratory else ".csv"
    authority_name = f"{condition_id}{suffix}"
    authority_path = bundle_root / authority_name
    pose = {
        "frame_id": "plane1",
        "tip_x_m": "0.010000000000",
        "tip_y_m": "-0.020000000000",
        "tip_z_m": "-0.120000000000",
        "tip_qx": "0.000000000000",
        "tip_qy": "0.000000000000",
        "tip_qz": "0.000000000000",
        "tip_qw": "1.000000000000",
    }
    _write_csv(
        authority_path,
        builder.SOURCE_COLUMNS,
        [
            {"time_s": "0.000000000", **pose, "phase": "draw"},
            {"time_s": "1.000000000", **pose, "phase": "hold"},
            {"time_s": "3.000000000", **pose, "phase": "hold"},
        ],
    )

    environment_root = workspace / "environments" / builder.PHASE2_ENVIRONMENT_ID
    environment_hashes = builder._environment_hashes(environment_root)
    environment_sha256 = builder._environment_tree_sha256(environment_hashes)
    conditions_sha256 = builder._sha256_file(conditions_path)
    registration_sha256 = builder._sha256_file(registration_path)
    authority_sha256 = builder._sha256_file(authority_path)
    source_phase = "held_out_nominal_preview" if preview else "hardware_frozen_main"
    source_status = "DO_NOT_FREEZE_AS_FINAL" if preview else "HARDWARE_FROZEN"
    index_row = {
        "condition_id": condition_id,
        "authority_file": authority_name,
        "authority_sha256": authority_sha256,
        "input_conditions_sha256": conditions_sha256,
        "registration_sha256": registration_sha256,
        "environment_id": builder.PHASE2_ENVIRONMENT_ID,
        "environment_sha256": environment_sha256,
        "mode": mode,
        "use_in_main_statistics": "0" if exploratory else "1",
        "source_phase": source_phase,
        "source_status": source_status,
        "location_id": "C",
        "direction_id": "C_DIF1",
        "moment_q_A_x": "0",
        "moment_q_A_y": "0",
        "moment_q_A_z": "1",
        "quat_A_M_x": "0",
        "quat_A_M_y": "0",
        "quat_A_M_z": "0",
        "quat_A_M_w": "1",
        **pose,
    }
    index_columns = (
        *builder.AUTHORITY_REQUIRED_COLUMNS,
        "location_id",
        "direction_id",
        "moment_q_A_x",
        "moment_q_A_y",
        "moment_q_A_z",
        "quat_A_M_x",
        "quat_A_M_y",
        "quat_A_M_z",
        "quat_A_M_w",
    )
    # Preserve column uniqueness while allowing the builder's required subset.
    index_columns = tuple(dict.fromkeys(index_columns))
    index_path = bundle_root / builder.AUTHORITY_INDEX_NAME
    _write_csv(index_path, index_columns, [index_row])
    manifest_path = bundle_root / builder.AUTHORITY_MANIFEST_NAME
    manifest = {
        "schema_version": 1,
        "manifest_type": "icassp_phase2_authority_bundle",
        "environment_id": builder.PHASE2_ENVIRONMENT_ID,
        "mode": mode,
        "use_in_main_statistics": not exploratory,
        "source_contains_preview": preview,
        "condition_count": 1,
        "expected_condition_count": 1,
        "input_conditions": {
            "path": "inputs/conditions.csv",
            "sha256": conditions_sha256,
        },
        "registration": {
            "path": "inputs/registration.yaml",
            "sha256": registration_sha256,
            "status": "EXPLORATORY" if exploratory else "HARDWARE_FROZEN",
            "magnet_id": "test_magnet",
            "polarity": "+Z_M_is_recorded_moment_q_A",
            "roll_policy": "frozen_in_quat_A_M",
        },
        "environment": {
            "id": builder.PHASE2_ENVIRONMENT_ID,
            "tree_sha256": environment_sha256,
            "hashes": environment_hashes,
        },
        "authority_index": {
            "path": builder.AUTHORITY_INDEX_NAME,
            "sha256": builder._sha256_file(index_path),
        },
        "entries": [
            {
                "condition_id": condition_id,
                "authority_file": authority_name,
                "authority_sha256": authority_sha256,
                "mode": mode,
                "use_in_main_statistics": not exploratory,
            }
        ],
    }
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    return index_path, manifest_path


def _args(
    workspace: Path,
    index_path: Path,
    output_root: Path,
    *,
    exploratory: bool = False,
    recordings_root: Path | None = None,
) -> object:
    argv = [
        "--workspace",
        str(workspace),
        "--authority-index",
        str(index_path),
        "--output-root",
        str(output_root),
    ]
    if exploratory:
        argv.append("--exploratory")
    if recordings_root is not None:
        argv.extend(("--recordings-root", str(recordings_root)))
    return builder.build_parser().parse_args(argv)


@contextmanager
def _fake_simulation(*_args: object, **_kwargs: object) -> Iterator[None]:
    yield


def _mock_successful_commands(
    monkeypatch: pytest.MonkeyPatch,
    workspace: Path,
) -> list[tuple[str, ...]]:
    commands: list[tuple[str, ...]] = []
    environment = workspace / "environments" / builder.PHASE2_ENVIRONMENT_ID

    def fake_run(
        command: Sequence[str],
        _log_path: Path,
        _timeout_s: float,
        **_kwargs: object,
    ) -> object:
        commands.append(tuple(command))
        if "build-recording" in command:
            trajectory_id = command[command.index("--trajectory") + 1]
            recording_id = f"{trajectory_id}__20260828_120000"
            recording = environment / "recordings" / recording_id
            recording.mkdir()
            source = environment / "trajectories" / f"{trajectory_id}.csv"
            (recording / "source_tip_trajectory.csv").write_bytes(source.read_bytes())
            (recording / "manifest.yaml").write_text(
                yaml.safe_dump(
                    {
                        "schema_version": 1,
                        "recording_id": recording_id,
                        "status": "validated",
                        "environment_id": builder.PHASE2_ENVIRONMENT_ID,
                        "trajectory_id": trajectory_id,
                        "hashes": builder._environment_hashes(environment),
                    },
                    sort_keys=False,
                ),
                encoding="utf-8",
            )
            return builder.CommandResult(0, f"{recording}\n", "")
        assert "inspect" in command
        return builder.CommandResult(0, "status: validated\n", "")

    monkeypatch.setattr(builder, "_simulation_session", _fake_simulation)
    monkeypatch.setattr(builder, "_run_logged", fake_run)
    monkeypatch.setattr(builder, "_action_server_count", lambda *_args, **_kwargs: 1)
    return commands


def _read_build_index(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def test_formal_batch_builds_mock_inspects_and_writes_atomic_resume_index(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(tmp_path, workspace)
    output_root = tmp_path / "data" / "phase2-build"
    commands = _mock_successful_commands(monkeypatch, workspace)

    assert builder.run_batch(_args(workspace, index_path, output_root)) == 0

    rows = _read_build_index(output_root / "formal" / builder.BUILD_INDEX_NAME)
    assert len(rows) == 1
    assert rows[0]["status"] == "success"
    assert rows[0]["mode"] == "formal"
    assert rows[0]["recording_id"].startswith("icassp_phase2__MAIN_C_DIF1_Z11__")
    assert any("build-recording" in command for command in commands)
    assert any("inspect" in command for command in commands)
    assert all("play" not in command for command in commands)
    manifest = yaml.safe_load(
        (output_root / "formal" / builder.BUILD_MANIFEST_NAME).read_text(encoding="utf-8")
    )
    assert manifest["complete"] is True
    assert manifest["hardware_motion_permitted"] is False


def test_same_hash_valid_success_is_inspected_and_skipped_on_resume(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(tmp_path, workspace)
    output_root = tmp_path / "output"
    first_commands = _mock_successful_commands(monkeypatch, workspace)
    args = _args(workspace, index_path, output_root)
    assert builder.run_batch(args) == 0
    assert any("build-recording" in command for command in first_commands)
    batch_manifest = output_root / "formal" / builder.BUILD_MANIFEST_NAME
    batch_manifest.unlink()

    second_commands: list[tuple[str, ...]] = []

    def inspect_only(
        command: Sequence[str],
        _log_path: Path,
        _timeout_s: float,
        **_kwargs: object,
    ) -> object:
        second_commands.append(tuple(command))
        assert "inspect" in command
        return builder.CommandResult(0, "status: validated\n", "")

    monkeypatch.setattr(builder, "_run_logged", inspect_only)
    assert builder.run_batch(args) == 0
    assert second_commands and all("inspect" in command for command in second_commands)
    assert len(_read_build_index(output_root / "formal" / builder.BUILD_INDEX_NAME)) == 1
    assert batch_manifest.is_file()


def test_formal_mode_rejects_preview_before_any_subprocess(tmp_path: Path) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(tmp_path, workspace, preview=True)

    with pytest.raises(builder.Phase2BuildError, match="formal.*preview"):
        builder.run_batch(_args(workspace, index_path, tmp_path / "output"))


def test_exploratory_preview_requires_flag_and_is_isolated(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(
        tmp_path, workspace, exploratory=True, preview=True
    )
    output_root = tmp_path / "data"
    commands = _mock_successful_commands(monkeypatch, workspace)

    with pytest.raises(builder.Phase2BuildError, match="mode must be formal"):
        builder.run_batch(_args(workspace, index_path, output_root))

    assert builder.run_batch(
        _args(workspace, index_path, output_root, exploratory=True)
    ) == 0
    rows = _read_build_index(output_root / "exploratory" / builder.BUILD_INDEX_NAME)
    assert rows[0]["mode"] == "exploratory"
    assert rows[0]["trajectory_id"].startswith("icassp_phase2_exploratory__")
    assert any("build-recording" in command for command in commands)
    assert not (output_root / "formal" / builder.BUILD_INDEX_NAME).exists()


def test_exploratory_source_conditions_may_omit_main_statistics_column(
    tmp_path: Path,
) -> None:
    conditions = tmp_path / "nominal_preview.csv"
    _write_csv(
        conditions,
        ("condition_id", "phase", "status"),
        [
            {
                "condition_id": "MAIN_C_DIF1_Z11",
                "phase": "held_out_nominal_preview",
                "status": "DO_NOT_FREEZE_AS_FINAL_UNTIL_SIGMA_HW_RECOMPUTE",
            }
        ],
    )
    entry = builder.AuthorityEntry(
        condition_id="MAIN_C_DIF1_Z11",
        authority_path=tmp_path / "unused.csv",
        authority_sha256="0" * 64,
        input_conditions_sha256="1" * 64,
        registration_sha256="2" * 64,
        environment_sha256="3" * 64,
        mode="exploratory",
        source_phase="held_out_nominal_preview",
        source_status="DO_NOT_FREEZE_AS_FINAL_UNTIL_SIGMA_HW_RECOMPUTE",
        frame_id="plane1",
        tip_pose=(0.0, 0.0, -0.117, 0.0, 0.0, 0.0, 1.0),
    )

    builder._validate_conditions_input(
        conditions,
        (entry,),
        mode="exploratory",
        declared_contains_preview=True,
    )


def test_failed_build_event_is_retained_for_a_later_resume(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(tmp_path, workspace)
    output_root = tmp_path / "output"

    def fail_build(
        command: Sequence[str],
        _log_path: Path,
        _timeout_s: float,
        **_kwargs: object,
    ) -> object:
        assert "build-recording" in command
        return builder.CommandResult(7, "", "mock compile failed")

    monkeypatch.setattr(builder, "_simulation_session", _fake_simulation)
    monkeypatch.setattr(builder, "_run_logged", fail_build)
    monkeypatch.setattr(builder, "_action_server_count", lambda *_args, **_kwargs: 1)

    assert builder.run_batch(_args(workspace, index_path, output_root)) == 2
    rows = _read_build_index(output_root / "formal" / builder.BUILD_INDEX_NAME)
    assert [row["status"] for row in rows] == ["failed"]
    assert "exit 7" in rows[0]["message"]


def test_changed_environment_hash_rejects_stale_authority_bundle(tmp_path: Path) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(tmp_path, workspace)
    environment = workspace / "environments" / builder.PHASE2_ENVIRONMENT_ID
    (environment / "zones.yaml").write_text(
        "schema_version: 1\nfixture: changed-after-authority\n", encoding="utf-8"
    )

    with pytest.raises(builder.Phase2BuildError, match="environment tree hash is stale"):
        builder.run_batch(_args(workspace, index_path, tmp_path / "output"))


def test_noncanonical_recordings_root_is_rejected_without_symlink_workaround(
    tmp_path: Path,
) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(tmp_path, workspace)
    external = tmp_path / "data" / "recordings"
    external.mkdir(parents=True)

    with pytest.raises(builder.Phase2BuildError, match="canonical recordings store"):
        builder.run_batch(
            _args(
                workspace,
                index_path,
                tmp_path / "output",
                recordings_root=external,
            )
        )


def test_actual_generator_bundle_is_accepted_by_builder_contract(tmp_path: Path) -> None:
    repository_root = Path(__file__).parents[1]
    environment = (
        repository_root
        / "environments"
        / builder.PHASE2_ENVIRONMENT_ID
    )
    conditions = tmp_path / "source" / "conditions.csv"
    conditions.parent.mkdir(parents=True)
    condition_columns = (
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
    _write_csv(
        conditions,
        condition_columns,
        [
            {
                "condition_id": "MAIN_C_DIF1_Z13",
                "phase": "hardware_frozen_main",
                "location_id": "C",
                "direction_id": "C_DIF1",
                "p_A_x_m": "0.020000000",
                "p_A_y_m": "-0.030000000",
                "p_A_z_m": "0.130000000",
                "moment_q_A_x": "0.000000000",
                "moment_q_A_y": "0.000000000",
                "moment_q_A_z": "1.000000000",
                "quat_A_M_x": "0.000000000000",
                "quat_A_M_y": "0.000000000000",
                "quat_A_M_z": "0.000000000000",
                "quat_A_M_w": "1.000000000000",
                "settle_s": "1.0",
                "record_window_s": "2.0",
                "use_in_main_statistics": "1",
                "status": "HARDWARE_FROZEN",
            }
        ],
    )
    registration = tmp_path / "source" / "registration.yaml"
    registration.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "environment_id": builder.PHASE2_ENVIRONMENT_ID,
                "status": "HARDWARE_FROZEN",
                "use_in_main_statistics": True,
                "transform_convention": "T_X_from_Y_column_vectors_metres",
                "magnet_id": "integration_test_magnet",
                "polarity": "+Z_M_is_recorded_moment_q_A",
                "roll_policy": "frozen_in_quat_A_M",
                "T_plane1_from_array": [
                    [1.0, 0.0, 0.0, 0.01],
                    [0.0, -1.0, 0.0, 0.02],
                    [0.0, 0.0, -1.0, -0.01],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                "T_rod_tip_from_magnet": [
                    [1.0, 0.0, 0.0, 0.001],
                    [0.0, 1.0, 0.0, 0.002],
                    [0.0, 0.0, 1.0, 0.005],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    generated = generate_phase2_authorities(
        conditions_path=conditions,
        registration_path=registration,
        environment_directory=environment,
        output_root=tmp_path / "generated",
        expected_condition_count=1,
    )
    environment_hashes = builder._environment_hashes(environment)

    bundle = builder.load_authority_bundle(
        generated.index_path,
        generated.manifest_path,
        expected_mode="formal",
        environment_id=builder.PHASE2_ENVIRONMENT_ID,
        environment_hashes=environment_hashes,
        environment_sha256=builder._environment_tree_sha256(environment_hashes),
    )

    assert [entry.condition_id for entry in bundle.entries] == ["MAIN_C_DIF1_Z13"]
    assert bundle.mode == "formal"


def test_builder_condition_ids_match_runtime_and_reject_dot() -> None:
    entry = builder.AuthorityEntry(
        condition_id="MAIN.BAD",
        authority_path=Path("unused.csv"),
        authority_sha256="0" * 64,
        input_conditions_sha256="1" * 64,
        registration_sha256="2" * 64,
        environment_sha256="3" * 64,
        mode="formal",
        source_phase="hardware_frozen_main",
        source_status="HARDWARE_FROZEN",
        frame_id="plane1",
        tip_pose=(0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 1.0),
    )

    with pytest.raises(builder.Phase2BuildError, match="unsafe generated trajectory_id"):
        builder._trajectory_id(entry)


def test_dedicated_ros_environment_clears_external_discovery(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("ROS_DISCOVERY_SERVER", "192.0.2.1:11811")
    monkeypatch.setenv("ROS_STATIC_PEERS", "192.0.2.2")

    curobo_source = Path("/verified/curobo-source")
    environment = builder._dedicated_ros_environment(222, curobo_source)

    assert environment["ROS_DOMAIN_ID"] == "222"
    assert environment["ROS_LOCALHOST_ONLY"] == "1"
    assert environment["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"
    assert environment["PYTHONPATH"].split(os.pathsep)[0] == str(curobo_source)
    assert "ROS_DISCOVERY_SERVER" not in environment
    assert "ROS_STATIC_PEERS" not in environment


def test_output_symlink_and_concurrent_builder_lock_are_rejected(tmp_path: Path) -> None:
    workspace = _make_workspace(tmp_path)
    index_path, _ = _make_authority_bundle(tmp_path, workspace)
    real_output = tmp_path / "real-output"
    real_output.mkdir()
    output_alias = tmp_path / "output-alias"
    output_alias.symlink_to(real_output, target_is_directory=True)

    with pytest.raises(builder.Phase2BuildError, match="symlink"):
        builder.run_batch(_args(workspace, index_path, output_alias))

    lock_path = tmp_path / "exclusive.lock"
    with builder._exclusive_lock(lock_path):
        with pytest.raises(builder.Phase2BuildError, match="another Phase-2 builder"):
            with builder._exclusive_lock(lock_path):
                pass


def test_tipdraw_allowlist_rejects_playback() -> None:
    with pytest.raises(builder.Phase2BuildError, match="offline tipdraw allowlist"):
        builder._validate_offline_tipdraw_command(
            (
                "ros2",
                "run",
                "hx_waypoint_tools",
                "tipdraw",
                "--workspace",
                "/tmp/workspace",
                "play",
                "--env",
                builder.PHASE2_ENVIRONMENT_ID,
                "--recording",
                "recording",
            )
        )
