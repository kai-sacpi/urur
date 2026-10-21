"""Audit native actual-joint telemetry without modifying or resampling its rows."""
from __future__ import annotations

import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
from typing import Any


FK_TOLERANCE_M = .001
LABEL = "nominal_rod_tip_not_magnetic_ground_truth"


def native_columns(joints: tuple[str, ...]) -> list[str]:
    return ["ros_time_s", "sync_time_ns", "ros_time_ns", "trajectory_time_s",
            "trajectory_time_ns", "state", *[f"desired_{j}_pos" for j in joints],
            *[f"{j}_pos" for j in joints], *[f"{j}_vel" for j in joints],
            "tip_x", "tip_y", "tip_z", "tip_vx", "tip_vy", "tip_vz",
            "actual_tip_speed_mps", "tip_wx", "tip_wy", "tip_wz",
            "actual_tip_angular_speed_radps", "desired_tip_speed_mps",
            "position_error_m", "speed_scaling"]


def _ns(value: str, name: str, *, positive: bool = True) -> int:
    if not re.fullmatch(r"[0-9]+", value) or (positive and int(value) == 0):
        raise ValueError(f"{name} must be {'positive' if positive else 'nonnegative'} integer ns")
    return int(value)


def _finite(values: list[str] | list[float], name: str) -> list[float]:
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains nonfinite values")
    return result


def audit_actual_telemetry(path: Path, motion: Any, output_dir: Path) -> dict[str, Any]:
    """Recompute FK from each actual q; planned columns never substitute for q.

    The source clock is JointState *receipt* time, not its source header.  The
    original CSV and all native row/timestamp associations remain untouched.
    """
    path, output_dir = Path(path).resolve(), Path(output_dir).resolve()
    reference = output_dir / "actual_fk_reference.csv"
    report = output_dir / "actual_telemetry_audit.json"
    if reference.exists() or report.exists():
        raise FileExistsError("actual telemetry audit outputs already exist; refusing overwrite")
    output_dir.mkdir(parents=True, exist_ok=True)
    result: dict[str, Any] = {
        "status": "failed", "passed": False, "raw_csv": str(path),
        "label_reference_type": LABEL, "actual_joint_source": "/joint_states",
        "actual_tip_frame": "rod_tip", "native_fk_reference_frame": "base_link",
        "derived_fk_reference_frames": ["base_link", "plane1"],
        "sync_clock": "host CLOCK_MONOTONIC JointState receipt time, not source header",
        "time_mapping": "native rows and times preserved; no resampling or correction",
        "fk_tolerance_m": FK_TOLERANCE_M,
    }
    try:
        raw = path.read_bytes()  # Exactly one snapshot is parsed and hashed.
        result["raw_sha256"] = hashlib.sha256(raw).hexdigest()
        joints = tuple(motion.environment.joint_names)
        if len(joints) != 6 or len(set(joints)) != 6:
            raise ValueError("actual telemetry requires six unique environment joints")
        reader = csv.reader(io.StringIO(raw.decode("utf-8"), newline=""), strict=True)
        columns = next(reader, [])
        if len(set(columns)) != len(columns) or columns != native_columns(joints):
            raise ValueError("native actual telemetry schema/order differs; no planned fallback")
        actual_names = [f"{j}_{kind}" for kind in ("pos", "vel") for j in joints]
        derived_columns = ["sync_time_ns", "ros_time_ns", "trajectory_time_s",
                           "trajectory_time_ns", "state", *actual_names]
        for frame in ("base_link", "plane1"):
            derived_columns.extend(f"{frame}_tip_{axis}" for axis in
                                   ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"))
        derived = []
        previous_sync = previous_ros = 0
        previous_trajectory = -1
        maximum_error = maximum_gap_ns = 0.
        for number, values in enumerate(reader, 2):
            if len(values) != len(columns):
                raise ValueError(f"native row {number} has incorrect width")
            row = dict(zip(columns, values, strict=True))
            sync, ros = (_ns(row[name], name) for name in ("sync_time_ns", "ros_time_ns"))
            trajectory_ns = _ns(row["trajectory_time_ns"], "trajectory_time_ns", positive=False)
            numeric = _finite([row[name] for name in columns if name != "state"], f"row {number}")
            del numeric
            trajectory_s = float(row["trajectory_time_s"])
            if (sync <= previous_sync or ros < previous_ros or trajectory_ns < previous_trajectory
                    or trajectory_s < 0 or abs(trajectory_s - trajectory_ns * 1e-9) > 1e-9):
                raise ValueError(f"native row {number} has invalid clock progression")
            if previous_sync:
                maximum_gap_ns = max(maximum_gap_ns, sync - previous_sync)
            previous_sync, previous_ros, previous_trajectory = sync, ros, trajectory_ns
            q = _finite([row[f"{j}_pos"] for j in joints], "actual joints")
            fk = motion.tip_fk(q)
            output = {name: row[name] for name in derived_columns if name in row}
            for frame in ("base_link", "plane1"):
                position = _finite(list(fk[f"{frame}_position_m"]), f"{frame} actual FK")
                quaternion = _finite(list(fk[f"{frame}_quaternion_xyzw"]), f"{frame} actual FK")
                if len(position) != 3 or len(quaternion) != 4 or abs(math.hypot(*quaternion) - 1.) > 1e-6:
                    raise ValueError("actual FK returned invalid position/quaternion")
                output.update(zip((f"{frame}_tip_{axis}" for axis in
                                   ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")),
                                  [*position, *quaternion], strict=True))
            error = math.dist([float(row[f"tip_{axis}"]) for axis in "xyz"], fk["base_link_position_m"])
            maximum_error = max(maximum_error, error)
            if error > FK_TOLERANCE_M:
                raise ValueError(f"actual native tip differs from actual-joint FK by {error} m")
            derived.append(output)
        if len(derived) < 2:
            raise ValueError("actual telemetry needs at least two native rows")
        with reference.open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=derived_columns)
            writer.writeheader()
            writer.writerows(derived)
        result.update(status="passed", passed=True, sample_count=len(derived),
                      actual_joint_columns=actual_names, reference_csv=str(reference),
                      reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
                      max_native_fk_error_m=maximum_error, max_sync_gap_s=maximum_gap_ns * 1e-9,
                      first_sync_time_ns=int(derived[0]["sync_time_ns"]),
                      last_sync_time_ns=int(derived[-1]["sync_time_ns"]))
    except BaseException as error:
        result.update(status="failed", passed=False, error=f"{type(error).__name__}: {error}")
        if not report.exists():
            with report.open("x") as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
                stream.write("\n")
        raise
    with report.open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return result
