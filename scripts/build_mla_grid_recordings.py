#!/usr/bin/env python3
"""Build and inspect all reachable MLA grid recordings in one mock ROS session.

This orchestrator never invokes ``tipdraw play`` and never connects to a real
robot.  It validates the 54-row reachability authority, starts mock hardware in
an isolated ROS domain, and runs compile -> mock recording -> inspect for each
reachable point.
"""

from __future__ import annotations

import argparse
import csv
import io
import math
import os
import re
import secrets
import signal
import subprocess
import tempfile
import time
from collections.abc import Iterator, Mapping, Sequence
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

import yaml
from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.errors import WaypointError
from hx_waypoint_curobo.recording_store import RecordingInspector
from hx_waypoint_curobo.trajectory_io import read_timed_pose_csv
from hx_waypoint_tools.mla_grid import (
    EXPECTED_TARGET_COUNT,
    MLA_GRID_ENVIRONMENT_ID,
    REACHABILITY_FIELDS,
    STATIC_DURATION_S,
    ordered_grid_targets,
    sha256_file,
)

BUILD_FIELDS = (
    "event_sequence",
    "timestamp_utc",
    "order",
    "condition_id",
    "trajectory_id",
    "reachability_sha256",
    "authority_sha256",
    "status",
    "recording_id",
    "recording_path",
    "recording_manifest_sha256",
    "build_log_path",
    "inspect_log_path",
    "message",
)
SELECTED_FIELDS = (
    "order",
    "condition_id",
    "trajectory_id",
    "authority_sha256",
    "recording_id",
    "recording_relative_path",
    "recording_manifest_sha256",
    "build_status",
)
_SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*\Z")


class MlaGridBuildError(RuntimeError):
    """The mock build cannot proceed without weakening its isolation contract."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _read_csv(path: Path) -> tuple[tuple[str, ...], list[dict[str, str]]]:
    if path.is_symlink() or not path.is_file():
        raise MlaGridBuildError(f"CSV must be a regular non-symlink file: {path}")
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise MlaGridBuildError(f"invalid CSV header: {path}")
        rows: list[dict[str, str]] = []
        for line_number, row in enumerate(reader, start=2):
            if None in row or any(value is None for value in row.values()):
                raise MlaGridBuildError(f"invalid CSV row {line_number}: {path}")
            rows.append({str(key): str(value) for key, value in row.items()})
    return tuple(reader.fieldnames), rows


def _atomic_csv(
    path: Path,
    rows: Sequence[Mapping[str, object]],
    *,
    fieldnames: Sequence[str] = BUILD_FIELDS,
) -> None:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as output:
            output.write(stream.getvalue())
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_yaml(path: Path, value: Mapping[str, object]) -> None:
    content = yaml.safe_dump(dict(value), sort_keys=False).encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _load_reachability(
    workspace: Path,
    path: Path,
) -> tuple[list[dict[str, str]], str]:
    fields, rows = _read_csv(path)
    if fields != REACHABILITY_FIELDS:
        raise MlaGridBuildError("reachability header does not match the MLA grid contract")
    if len(rows) != EXPECTED_TARGET_COUNT:
        raise MlaGridBuildError(
            f"reachability must contain exactly {EXPECTED_TARGET_COUNT} rows; got {len(rows)}"
        )
    expected = ordered_grid_targets()
    environment_root = workspace / "environments" / MLA_GRID_ENVIRONMENT_ID
    for target, row in zip(expected, rows, strict=True):
        if int(row["order"]) != target.order or row["condition_id"] != target.condition_id:
            raise MlaGridBuildError(
                f"reachability order/identity mismatch at row {target.order}"
            )
        if row["environment_id"] != MLA_GRID_ENVIRONMENT_ID:
            raise MlaGridBuildError(f"wrong environment for {target.condition_id}")
        actual_position = tuple(
            float(row[name])
            for name in (
                "tip_x_plane1_m",
                "tip_y_plane1_m",
                "tip_z_plane1_m",
            )
        )
        if any(
            abs(actual - required) > 1.0e-12
            for actual, required in zip(
                actual_position,
                target.tip_position_plane1_m,
                strict=True,
            )
        ):
            raise MlaGridBuildError(f"target position was changed: {target.condition_id}")
        if row["status"] not in {"REACHABLE", "UNREACHABLE"}:
            raise MlaGridBuildError(f"invalid reachability status: {target.condition_id}")
        if row["status"] == "UNREACHABLE":
            if row["trajectory_id"] or row["authority_sha256"]:
                raise MlaGridBuildError(
                    f"unreachable target exposes a substitute authority: {target.condition_id}"
                )
            continue
        trajectory_id = row["trajectory_id"]
        if trajectory_id != target.condition_id or _SAFE_ID.fullmatch(trajectory_id) is None:
            raise MlaGridBuildError(f"unsafe trajectory identity: {trajectory_id!r}")
        authority_path = environment_root / "trajectories" / f"{trajectory_id}.csv"
        if authority_path.is_symlink() or not authority_path.is_file():
            raise MlaGridBuildError(f"authority is missing: {authority_path}")
        if sha256_file(authority_path) != row["authority_sha256"]:
            raise MlaGridBuildError(f"authority hash mismatch: {trajectory_id}")
        poses = read_timed_pose_csv(authority_path)
        if [pose.time_s for pose in poses] != [0.0, 1.0, STATIC_DURATION_S]:
            raise MlaGridBuildError(f"authority timestamps changed: {trajectory_id}")
        first = poses[0]
        if any(
            pose.position_m != first.position_m
            or pose.quaternion_xyzw != first.quaternion_xyzw
            for pose in poses[1:]
        ):
            raise MlaGridBuildError(f"authority is not strictly static: {trajectory_id}")
    return rows, sha256_file(path)


def _locked_curobo_source(workspace: Path) -> Path:
    lock_path = workspace / "curobo.lock"
    if not lock_path.is_file():
        raise MlaGridBuildError(f"missing cuRobo lock: {lock_path}")
    lock = yaml.safe_load(lock_path.read_text(encoding="utf-8"))
    if not isinstance(lock, Mapping) or not isinstance(lock.get("commit"), str):
        raise MlaGridBuildError("curobo.lock is invalid")
    source = workspace / ".deps" / f"curobo-{lock['commit']}"
    if source.is_symlink() or not (source / "curobo" / "__init__.py").is_file():
        raise MlaGridBuildError(f"locked cuRobo source is unavailable: {source}")
    return source.resolve()


def _select_domain(requested: int | None) -> int:
    caller = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if requested is not None:
        if not 0 <= requested <= 232 or requested == caller:
            raise MlaGridBuildError(
                "--ros-domain-id must be within 0..232 and differ from the caller domain"
            )
        return requested
    choices = [value for value in range(101, 233) if value != caller]
    return choices[secrets.randbelow(len(choices))]


def _ros_environment(workspace: Path, domain_id: int) -> dict[str, str]:
    environment = dict(os.environ)
    environment.pop("ROS_DISCOVERY_SERVER", None)
    environment.pop("ROS_STATIC_PEERS", None)
    environment.update(
        {
            "ROS_DOMAIN_ID": str(domain_id),
            "ROS2CLI_NO_DAEMON": "1",
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        }
    )
    parts = [str(_locked_curobo_source(workspace))]
    venv_site_packages = sorted(
        (workspace / ".venv" / "lib").glob("python*/site-packages")
    )
    if len(venv_site_packages) > 1:
        raise MlaGridBuildError("workspace .venv has ambiguous Python site-packages")
    if venv_site_packages:
        parts.append(str(venv_site_packages[0].resolve()))
    inherited = environment.get("PYTHONPATH", "")
    if inherited:
        parts.append(inherited)
    environment["PYTHONPATH"] = os.pathsep.join(parts)
    return environment


def _ros_nodes(environment: Mapping[str, str], spin_time: float) -> list[str]:
    command = [
        "ros2",
        "node",
        "list",
        "--no-daemon",
        "--spin-time",
        str(spin_time),
    ]
    try:
        result = subprocess.run(
            command,
            text=True,
            capture_output=True,
            timeout=max(5.0, spin_time + 2.0),
            check=False,
            env=dict(environment),
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise MlaGridBuildError(f"ROS graph query failed: {error}") from error
    if result.returncode != 0:
        raise MlaGridBuildError(f"ROS graph query failed: {result.stderr.strip()}")
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def _assert_empty_domain(environment: Mapping[str, str]) -> None:
    for _ in range(2):
        nodes = _ros_nodes(environment, 0.5)
        if nodes:
            raise MlaGridBuildError(
                "dedicated mock ROS domain is not empty; refusing cross-talk: "
                + ",".join(nodes)
            )
        time.sleep(0.25)


def _stop_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    for signal_number, timeout_s in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0)):
        try:
            os.killpg(process.pid, signal_number)
            process.wait(timeout=timeout_s)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            continue
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass


@contextmanager
def _simulation_session(
    workspace: Path,
    launch_log: Path,
    environment: Mapping[str, str],
    launch_timeout_s: float,
) -> Iterator[None]:
    _assert_empty_domain(environment)
    command = [
        "ros2",
        "launch",
        "hx_waypoint_bringup",
        "simulation.launch.py",
        "ur_type:=ur5",
        f"environment_id:={MLA_GRID_ENVIRONMENT_ID}",
        f"workspace:={workspace}",
        "launch_rviz:=false",
    ]
    launch_log.parent.mkdir(parents=True, exist_ok=True)
    with launch_log.open("w", encoding="utf-8") as stream:
        stream.write("$ " + " ".join(command) + "\n")
        stream.flush()
        process = subprocess.Popen(
            command,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
            env=dict(environment),
        )
        try:
            deadline = time.monotonic() + launch_timeout_s
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise MlaGridBuildError(
                        f"mock simulation exited early; see {launch_log}"
                    )
                nodes = _ros_nodes(environment, 1.0)
                if nodes.count("/hx_waypoint_runtime") > 1:
                    raise MlaGridBuildError("multiple mock runtime nodes were discovered")
                if nodes.count("/joint_trajectory_controller") > 1:
                    raise MlaGridBuildError("multiple mock controllers were discovered")
                if (
                    nodes.count("/hx_waypoint_runtime") == 1
                    and nodes.count("/joint_trajectory_controller") == 1
                ):
                    break
                time.sleep(1.0)
            else:
                raise MlaGridBuildError(
                    f"mock simulation was not ready in {launch_timeout_s}s; see {launch_log}"
                )
            yield
        finally:
            _stop_process_group(process)


def _run_logged(
    command: Sequence[str],
    log_path: Path,
    timeout_s: float,
    environment: Mapping[str, str],
) -> subprocess.CompletedProcess[str]:
    try:
        result = subprocess.run(
            list(command),
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
            env=dict(environment),
        )
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout if isinstance(error.stdout, str) else ""
        stderr = error.stderr if isinstance(error.stderr, str) else ""
        result = subprocess.CompletedProcess(command, 124, stdout, stderr + "\nTIMEOUT")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as stream:
        stream.write("$ " + " ".join(command) + "\n")
        stream.write(result.stdout)
        if result.stderr:
            stream.write("\n[stderr]\n" + result.stderr)
        stream.write(f"\n[exit={result.returncode}]\n")
    return result


def _validate_recording(
    workspace: Path,
    trajectory_id: str,
    authority_sha256: str,
    recording_path: Path,
) -> tuple[str, str]:
    expected_parent = (
        workspace / "environments" / MLA_GRID_ENVIRONMENT_ID / "recordings"
    ).resolve()
    if recording_path.is_symlink() or not recording_path.is_dir():
        raise MlaGridBuildError(f"recording is not a real directory: {recording_path}")
    if recording_path.resolve().parent != expected_parent:
        raise MlaGridBuildError(f"recording escaped the environment store: {recording_path}")
    try:
        inspection = RecordingInspector().inspect(recording_path)
    except WaypointError as error:
        raise MlaGridBuildError(
            f"recording inspect failed: {recording_path.name}: {error}"
        ) from error
    manifest = inspection.manifest
    if (
        manifest.get("status") != "validated"
        or manifest.get("environment_id") != MLA_GRID_ENVIRONMENT_ID
        or manifest.get("trajectory_id") != trajectory_id
        or manifest.get("recording_id") != recording_path.name
    ):
        raise MlaGridBuildError(f"recording identity mismatch: {recording_path.name}")
    source = recording_path / "source_tip_trajectory.csv"
    if sha256_file(source) != authority_sha256:
        raise MlaGridBuildError(f"recording source hash mismatch: {recording_path.name}")
    manifest_path = recording_path / "manifest.yaml"
    return recording_path.name, sha256_file(manifest_path)


def _recover_recording(
    workspace: Path,
    trajectory_id: str,
    authority_sha256: str,
) -> Path | None:
    root = workspace / "environments" / MLA_GRID_ENVIRONMENT_ID / "recordings"
    for candidate in sorted(root.glob(f"{trajectory_id}__*"), reverse=True):
        try:
            _validate_recording(workspace, trajectory_id, authority_sha256, candidate)
        except (OSError, ValueError, MlaGridBuildError):
            continue
        return candidate
    return None


def _event(
    *,
    sequence: int,
    reachability_sha256: str,
    row: Mapping[str, str],
    status: str,
    build_log: Path,
    inspect_log: Path,
    message: str,
    recording_path: Path | None = None,
    recording_manifest_sha256: str = "",
) -> dict[str, object]:
    return {
        "event_sequence": sequence,
        "timestamp_utc": _utc_now(),
        "order": row["order"],
        "condition_id": row["condition_id"],
        "trajectory_id": row["trajectory_id"],
        "reachability_sha256": reachability_sha256,
        "authority_sha256": row["authority_sha256"],
        "status": status,
        "recording_id": recording_path.name if recording_path else "",
        "recording_path": str(recording_path) if recording_path else "",
        "recording_manifest_sha256": recording_manifest_sha256,
        "build_log_path": str(build_log),
        "inspect_log_path": str(inspect_log),
        "message": message.replace("\n", " "),
    }


def _latest_successes(
    index_path: Path,
    reachability_sha256: str,
) -> tuple[list[dict[str, str]], dict[str, dict[str, str]]]:
    if not index_path.exists():
        return [], {}
    fields, rows = _read_csv(index_path)
    if fields != BUILD_FIELDS:
        raise MlaGridBuildError("existing build index has an incompatible schema")
    if any(row["reachability_sha256"] != reachability_sha256 for row in rows):
        raise MlaGridBuildError("existing build index belongs to another authority revision")
    latest: dict[str, dict[str, str]] = {}
    for row in rows:
        latest[row["condition_id"]] = row
    return rows, latest


def run(args: argparse.Namespace) -> int:
    workspace = Path(args.workspace).expanduser().resolve()
    reachability_path = Path(args.reachability).expanduser().resolve()
    output_directory = Path(args.output_directory).expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    environment = EnvironmentBuilder().load(
        workspace / "environments" / MLA_GRID_ENVIRONMENT_ID
    )
    if environment.environment_id != MLA_GRID_ENVIRONMENT_ID:
        raise MlaGridBuildError("workspace MLA environment identity is invalid")
    authority_rows, reachability_sha256 = _load_reachability(
        workspace,
        reachability_path,
    )
    reachable = [row for row in authority_rows if row["status"] == "REACHABLE"]
    index_path = output_directory / "recording_build_index.csv"
    manifest_path = output_directory / "recording_build_manifest.yaml"
    events, latest = _latest_successes(index_path, reachability_sha256)
    pending: list[dict[str, str]] = []
    for row in reachable:
        prior = latest.get(row["condition_id"])
        if prior is not None and prior["status"] in {"SUCCESS", "RECOVERED"}:
            path = Path(prior["recording_path"])
            try:
                _validate_recording(
                    workspace,
                    row["trajectory_id"],
                    row["authority_sha256"],
                    path,
                )
            except (OSError, ValueError, MlaGridBuildError):
                pending.append(row)
            else:
                continue
        else:
            pending.append(row)

    domain_id = _select_domain(args.ros_domain_id)
    ros_environment = _ros_environment(workspace, domain_id)
    failures = 0
    if pending:
        with _simulation_session(
            workspace,
            output_directory / "simulation.launch.log",
            ros_environment,
            args.launch_timeout,
        ):
            for batch_index, row in enumerate(pending, start=1):
                trajectory_id = row["trajectory_id"]
                build_log = output_directory / "logs" / f"{trajectory_id}.build.log"
                inspect_log = output_directory / "logs" / f"{trajectory_id}.inspect.log"
                print(
                    f"[{batch_index:02d}/{len(pending):02d}] {trajectory_id}",
                    flush=True,
                )
                recording_path = _recover_recording(
                    workspace,
                    trajectory_id,
                    row["authority_sha256"],
                )
                try:
                    status = "RECOVERED"
                    message = "reused an already validated recording with identical authority"
                    if recording_path is None:
                        command = [
                            "ros2",
                            "run",
                            "hx_waypoint_tools",
                            "tipdraw",
                            "--workspace",
                            str(workspace),
                            "build-recording",
                            "--env",
                            MLA_GRID_ENVIRONMENT_ID,
                            "--trajectory",
                            trajectory_id,
                            "--external-runtime",
                        ]
                        result = _run_logged(
                            command,
                            build_log,
                            args.build_timeout,
                            ros_environment,
                        )
                        if result.returncode != 0:
                            raise MlaGridBuildError(
                                f"build-recording exited {result.returncode}; see {build_log}"
                            )
                        output_lines = [
                            line.strip() for line in result.stdout.splitlines() if line.strip()
                        ]
                        if not output_lines:
                            raise MlaGridBuildError("build-recording returned no recording path")
                        recording_path = Path(output_lines[-1]).expanduser().resolve()
                        status = "SUCCESS"
                        message = "GPU compile, mock recording, and inspect succeeded"

                    inspect_command = [
                        "ros2",
                        "run",
                        "hx_waypoint_tools",
                        "tipdraw",
                        "--workspace",
                        str(workspace),
                        "inspect",
                        "--env",
                        MLA_GRID_ENVIRONMENT_ID,
                        "--recording",
                        recording_path.name,
                    ]
                    inspected = _run_logged(
                        inspect_command,
                        inspect_log,
                        args.inspect_timeout,
                        ros_environment,
                    )
                    if inspected.returncode != 0:
                        raise MlaGridBuildError(
                            f"inspect exited {inspected.returncode}; see {inspect_log}"
                        )
                    _, manifest_sha256 = _validate_recording(
                        workspace,
                        trajectory_id,
                        row["authority_sha256"],
                        recording_path,
                    )
                    events.append(
                        _event(
                            sequence=len(events) + 1,
                            reachability_sha256=reachability_sha256,
                            row=row,
                            status=status,
                            build_log=build_log,
                            inspect_log=inspect_log,
                            message=message,
                            recording_path=recording_path,
                            recording_manifest_sha256=manifest_sha256,
                        )
                    )
                    print(f"  {status}: {recording_path.name}", flush=True)
                except (OSError, ValueError, MlaGridBuildError) as error:
                    failures += 1
                    events.append(
                        _event(
                            sequence=len(events) + 1,
                            reachability_sha256=reachability_sha256,
                            row=row,
                            status="FAILED",
                            build_log=build_log,
                            inspect_log=inspect_log,
                            message=str(error),
                        )
                    )
                    print(f"  FAILED: {error}", flush=True)
                _atomic_csv(index_path, events)
                if failures and args.fail_fast:
                    break

    latest_by_condition: dict[str, Mapping[str, object]] = {}
    for event in events:
        latest_by_condition[str(event["condition_id"])] = event
    validated_count = sum(
        event["status"] in {"SUCCESS", "RECOVERED"}
        for condition_id, event in latest_by_condition.items()
        if any(row["condition_id"] == condition_id for row in reachable)
    )
    selected_rows: list[dict[str, object]] = []
    for authority in reachable:
        event = latest_by_condition.get(authority["condition_id"])
        if event is None or event["status"] not in {"SUCCESS", "RECOVERED"}:
            continue
        recording_path = Path(str(event["recording_path"])).resolve()
        selected_rows.append(
            {
                "order": authority["order"],
                "condition_id": authority["condition_id"],
                "trajectory_id": authority["trajectory_id"],
                "authority_sha256": authority["authority_sha256"],
                "recording_id": event["recording_id"],
                "recording_relative_path": str(recording_path.relative_to(workspace)),
                "recording_manifest_sha256": event["recording_manifest_sha256"],
                "build_status": event["status"],
            }
        )
    selected_path = output_directory / "selected_recordings.csv"
    _atomic_csv(selected_path, selected_rows, fieldnames=SELECTED_FIELDS)
    manifest = {
        "schema_version": 1,
        "manifest_type": "mla_grid_54_recording_build",
        "updated_utc": _utc_now(),
        "environment_id": MLA_GRID_ENVIRONMENT_ID,
        "environment_hashes": dict(sorted(environment.hashes.items())),
        "reachability": {
            "path": str(reachability_path),
            "sha256": reachability_sha256,
            "total_target_count": EXPECTED_TARGET_COUNT,
            "reachable_count": len(reachable),
            "unreachable_count": EXPECTED_TARGET_COUNT - len(reachable),
        },
        "mock_isolation": {
            "ros_domain_id": domain_id,
            "ros_localhost_only": True,
            "real_robot_playback_invoked": False,
        },
        "recordings": {
            "validated_count": validated_count,
            "pending_or_failed_count": len(reachable) - validated_count,
            "index_path": str(index_path),
            "index_sha256": sha256_file(index_path) if index_path.exists() else "",
            "selected_index_path": str(selected_path),
            "selected_index_sha256": sha256_file(selected_path),
        },
    }
    _atomic_yaml(manifest_path, manifest)
    if failures or validated_count != len(reachable):
        print(
            f"completed with {len(reachable) - validated_count} pending/failed recording(s); "
            f"index: {index_path}",
            flush=True,
        )
        return 2
    print(f"all reachable recordings validated: {validated_count}; index: {index_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=repository_root)
    parser.add_argument("--reachability", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int)
    parser.add_argument("--launch-timeout", type=float, default=180.0)
    parser.add_argument("--build-timeout", type=float, default=900.0)
    parser.add_argument("--inspect-timeout", type=float, default=120.0)
    parser.add_argument("--fail-fast", action="store_true")
    return parser


def main() -> int:
    parser = build_parser()
    arguments = parser.parse_args()
    for name in ("launch_timeout", "build_timeout", "inspect_timeout"):
        value = float(getattr(arguments, name))
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    try:
        return run(arguments)
    except (MlaGridBuildError, OSError, ValueError) as error:
        print(f"build_mla_grid_recordings.py: {error}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
