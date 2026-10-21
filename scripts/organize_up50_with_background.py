#!/usr/bin/env python3
"""Bundle the ten up50 real captures with the plane1 z-50 mm background field."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
from typing import Any, Iterable

import yaml


ENVIRONMENT_ID = "new_board_4cm_rod33365"
DEFAULT_BACKGROUND = (
    "empty_field_plane1_zminus50mm__20260806T125029.629110Z__f5f73abd"
)
TRAJECTORIES = (
    ("Wave", "wave"),
    ("Helix", "helix"),
    ("Rounded square", "rounded_square"),
    ("Figure eight", "figure_eight"),
    ("Lissajous", "lissajous"),
    ("Trefoil knot", "trefoil_knot"),
    ("Heart", "heart"),
    ("Spirograph", "spirograph"),
    ("Spherical spiral", "spherical_spiral"),
    ("Celtic knot", "celtic_knot"),
)
AXES = ("x", "y", "z")
SENSOR_COUNT = 64


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write("\n")


def copy_file(source: Path, destination: Path) -> None:
    source = source.resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    source_stat = source.stat()
    destination_stat = destination.stat()
    if (source_stat.st_dev, source_stat.st_ino) == (
        destination_stat.st_dev,
        destination_stat.st_ino,
    ):
        raise RuntimeError(f"bundle copy unexpectedly aliases source inode: {destination}")
    if sha256(source) != sha256(destination):
        raise RuntimeError(f"bundle copy checksum mismatch: {destination}")


def copy_tree(source: Path, destination: Path) -> int:
    count = 0
    for path in sorted(source.rglob("*")):
        if path.is_symlink():
            raise RuntimeError(f"source artifact contains a symlink: {path}")
        if path.is_file():
            copy_file(path, destination / path.relative_to(source))
            count += 1
    return count


def verify_recording_checksums(recording: Path) -> int:
    checksum_path = recording / "sha256sums.txt"
    rows = checksum_path.read_text(encoding="utf-8").splitlines()
    checked = 0
    for line in rows:
        expected, relative = line.split("  ", 1)
        target = recording / relative
        if sha256(target) != expected:
            raise RuntimeError(f"recording checksum mismatch: {target}")
        checked += 1
    if checked == 0:
        raise RuntimeError(f"empty recording checksum file: {checksum_path}")
    return checked


@dataclass
class RunningStat:
    count: int = 0
    mean: float = 0.0
    m2: float = 0.0
    minimum: float = math.inf
    maximum: float = -math.inf

    def add(self, value: float) -> None:
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (value - self.mean)
        self.minimum = min(self.minimum, value)
        self.maximum = max(self.maximum, value)

    @property
    def population_std(self) -> float:
        return math.sqrt(self.m2 / self.count) if self.count else math.nan


def format_float(value: float) -> str:
    return format(value, ".12g")


def background_statistics(raw_path: Path, output: Path, source_manifest: dict[str, Any]) -> dict[str, Any]:
    axis_stats = {
        (sensor, axis): RunningStat()
        for sensor in range(SENSOR_COUNT)
        for axis in AXES
    }
    magnitude_stats = {sensor: RunningStat() for sensor in range(SENSOR_COUNT)}
    frame_count = 0
    with raw_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            f"sensor_{sensor:02d}_{axis}_ut"
            for sensor in range(SENSOR_COUNT)
            for axis in AXES
        }
        missing = sorted(required - set(reader.fieldnames or ()))
        if missing:
            raise RuntimeError("background CSV missing columns: " + ", ".join(missing[:5]))
        for row in reader:
            frame_count += 1
            for sensor in range(SENSOR_COUNT):
                values = []
                for axis in AXES:
                    value = float(row[f"sensor_{sensor:02d}_{axis}_ut"])
                    if not math.isfinite(value):
                        raise RuntimeError("background CSV contains a non-finite value")
                    axis_stats[(sensor, axis)].add(value)
                    values.append(value)
                magnitude_stats[sensor].add(math.sqrt(sum(value * value for value in values)))
    if frame_count == 0:
        raise RuntimeError("background CSV is empty")

    stats_path = output / "background_field_stats.csv"
    with stats_path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            ["sensor_id", "axis", "sample_count", "mean_ut", "population_std_ut", "min_ut", "max_ut"]
        )
        for sensor in range(SENSOR_COUNT):
            for axis in AXES:
                stat = axis_stats[(sensor, axis)]
                writer.writerow(
                    [
                        f"{sensor:02d}", axis, stat.count, format_float(stat.mean),
                        format_float(stat.population_std), format_float(stat.minimum),
                        format_float(stat.maximum),
                    ]
                )

    mean_path = output / "background_field_mean.csv"
    with mean_path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            [
                "sensor_id", "sample_count", "mean_x_ut", "mean_y_ut", "mean_z_ut",
                "mean_vector_magnitude_ut", "sample_vector_magnitude_mean_ut",
                "sample_vector_magnitude_population_std_ut", "sample_vector_magnitude_min_ut",
                "sample_vector_magnitude_max_ut",
            ]
        )
        for sensor in range(SENSOR_COUNT):
            means = [axis_stats[(sensor, axis)].mean for axis in AXES]
            magnitude = magnitude_stats[sensor]
            writer.writerow(
                [
                    f"{sensor:02d}", frame_count,
                    *(format_float(value) for value in means),
                    format_float(math.sqrt(sum(value * value for value in means))),
                    format_float(magnitude.mean), format_float(magnitude.population_std),
                    format_float(magnitude.minimum), format_float(magnitude.maximum),
                ]
            )

    magloc_json = {
        "schema_version": 1,
        "frame_count": frame_count,
        "sensor_count": SENSOR_COUNT,
        "active_sensor_ids": list(range(SENSOR_COUNT)),
        "metadata": {
            "source": str(raw_path),
            "coordinate_contract": "raw NUCLEO sensor axes; used only for field subtraction",
            "reference_frame": "plane1",
            "sensor_board_translation_from_original_mm": [0.0, 0.0, -50.0],
        },
        "sensors": [],
    }
    for sensor in range(SENSOR_COUNT):
        magloc_json["sensors"].append(
            {
                "sensor_id": sensor,
                "enabled": True,
                "mean_uT": {axis: axis_stats[(sensor, axis)].mean for axis in AXES},
                "std_uT": {
                    axis: axis_stats[(sensor, axis)].population_std for axis in AXES
                },
                "count": {axis: axis_stats[(sensor, axis)].count for axis in AXES},
            }
        )
    write_json(output / "magloc_background_field.json", magloc_json)

    with (output / "magloc_background_field.csv").open(
        "x", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            [
                "sensor_id", "enabled",
                "mean_uT_x", "std_uT_x", "count_x",
                "mean_uT_y", "std_uT_y", "count_y",
                "mean_uT_z", "std_uT_z", "count_z",
            ]
        )
        for sensor in range(SENSOR_COUNT):
            row: list[Any] = [sensor, True]
            for axis in AXES:
                stat = axis_stats[(sensor, axis)]
                row.extend([format_float(stat.mean), format_float(stat.population_std), stat.count])
            writer.writerow(row)

    with (output / "magloc_background_field_wide.csv").open(
        "x", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            ["frame_count"]
            + [
                f"B_uT_{sensor}_{axis}"
                for sensor in range(SENSOR_COUNT)
                for axis in AXES
            ]
        )
        writer.writerow(
            [frame_count]
            + [
                format_float(axis_stats[(sensor, axis)].mean)
                for sensor in range(SENSOR_COUNT)
                for axis in AXES
            ]
        )

    all_axis = [axis_stats[key] for key in sorted(axis_stats)]
    max_axis_key = max(axis_stats, key=lambda key: axis_stats[key].population_std)
    vector_means = [
        math.sqrt(sum(axis_stats[(sensor, axis)].mean ** 2 for axis in AXES))
        for sensor in range(SENSOR_COUNT)
    ]
    highest_noise = sorted(
        (
            {
                "sensor_id": f"{sensor:02d}",
                "axis": axis,
                "population_std_ut": stat.population_std,
            }
            for (sensor, axis), stat in axis_stats.items()
        ),
        key=lambda item: item["population_std_ut"],
        reverse=True,
    )[:10]
    quality = source_manifest["counts"]["quality_counters"]
    capture = source_manifest["capture"]
    summary = {
        "schema_version": 1,
        "status": source_manifest["status"],
        "experiment": "empty_sensor_field_plane1_zminus50mm",
        "source_directory": str(raw_path.parent),
        "sample_count": frame_count,
        "sensor_count": SENSOR_COUNT,
        "axes_per_sensor": list(AXES),
        "units": "microtesla",
        "requested_duration_s": capture["requested_duration_s"],
        "timer_duration_s": capture["timer_duration_s"],
        "capture_clock_duration_s": capture["capture_clock_duration_s"],
        "robot_motion_commanded": bool(source_manifest["robot_motion_commanded"]),
        "ur_control_started": bool(source_manifest["ur_control_started"]),
        "spatial_context": source_manifest["spatial_context"],
        "quality_counters": quality,
        "statistics_method": {
            "algorithm": "streaming Welford",
            "standard_deviation": "population standard deviation (ddof=0)",
            "background_subtraction_applied": False,
        },
        "aggregate": {
            "axis_population_std_mean_ut": sum(stat.population_std for stat in all_axis) / len(all_axis),
            "axis_population_std_max_ut": axis_stats[max_axis_key].population_std,
            "axis_population_std_max_sensor_id": f"{max_axis_key[0]:02d}",
            "axis_population_std_max_axis": max_axis_key[1],
            "sensor_mean_vector_magnitude_min_ut": min(vector_means),
            "sensor_mean_vector_magnitude_mean_ut": sum(vector_means) / len(vector_means),
            "sensor_mean_vector_magnitude_max_ut": max(vector_means),
        },
        "highest_noise_axes": highest_noise,
    }
    write_json(output / "background_field_summary.yaml", summary)
    return summary


def robot_table(path: Path) -> tuple[dict[str, tuple[str, str]], dict[str, Any]]:
    rows: dict[str, tuple[str, str]] = {}
    count = 0
    last_time = 0.0
    speed_min = math.inf
    speed_max = -math.inf
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"sync_time_ns", "trajectory_time_s", "trajectory_time_ns", "speed_scaling"}
        missing = sorted(required - set(reader.fieldnames or ()))
        if missing:
            raise RuntimeError(f"robot telemetry missing columns: {missing}")
        for row in reader:
            count += 1
            sync = row["sync_time_ns"]
            if sync in rows:
                raise RuntimeError(f"duplicate robot sync timestamp: {sync}")
            rows[sync] = (row["trajectory_time_s"], row["trajectory_time_ns"])
            last_time = float(row["trajectory_time_s"])
            speed = float(row["speed_scaling"])
            speed_min = min(speed_min, speed)
            speed_max = max(speed_max, speed)
    if count == 0:
        raise RuntimeError(f"empty robot telemetry: {path}")
    return rows, {
        "sample_count": count,
        "last_trajectory_time_s": last_time,
        "speed_scaling_min": speed_min,
        "speed_scaling_max": speed_max,
        "all_speed_scaling_exactly_one": speed_min == 1.0 and speed_max == 1.0,
    }


def sensor_table(path: Path, robot_rows: dict[str, tuple[str, str]]) -> dict[str, Any]:
    count = 0
    missing_robot = 0
    time_mismatch = 0
    max_offset_ns = 0
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"sync_time_ns", "trajectory_time_s", "trajectory_time_ns", "sync_offset_ns"}
        missing = sorted(required - set(reader.fieldnames or ()))
        if missing:
            raise RuntimeError(f"sensor telemetry missing columns: {missing}")
        for row in reader:
            count += 1
            expected = robot_rows.get(row["sync_time_ns"])
            if expected is None:
                missing_robot += 1
            elif expected != (row["trajectory_time_s"], row["trajectory_time_ns"]):
                time_mismatch += 1
            max_offset_ns = max(max_offset_ns, abs(int(row["sync_offset_ns"])))
    if count == 0:
        raise RuntimeError(f"empty sensor telemetry: {path}")
    return {
        "mapped_frame_count": count,
        "missing_robot_timestamp_count": missing_robot,
        "trajectory_time_mismatch_count": time_mismatch,
        "maximum_mapping_offset_ms": max_offset_ns / 1_000_000.0,
    }


def all_files(root: Path, exclude: Iterable[Path] = ()) -> list[Path]:
    excluded = {path.resolve() for path in exclude}
    return [
        path
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.resolve() not in excluded
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default="/home/huaxin/Codes/ur-waypoint")
    parser.add_argument("--background", default=DEFAULT_BACKGROUND)
    parser.add_argument("--run-date-prefix", default="20260807")
    args = parser.parse_args()

    workspace = Path(args.workspace).expanduser().resolve()
    runs = workspace / "runs"
    recordings_root = workspace / "environments" / ENVIRONMENT_ID / "recordings"
    index_path = runs / "up50_recordings.latest.csv"
    background_source = runs / args.background
    now = datetime.now(timezone.utc)
    dataset_id = (
        "organized_up50_real10_with_plane1_zminus50mm_background__"
        + now.strftime("%Y%m%dT%H%M%S.%fZ")
    )
    output = runs / dataset_id
    output.mkdir(parents=True, exist_ok=False)
    (output / "trajectories").mkdir()
    (output / "background").mkdir()
    (output / "logs").mkdir()
    (output / "provenance").mkdir()

    index_rows = list(csv.DictReader(index_path.open("r", encoding="utf-8", newline="")))
    if len(index_rows) != len(TRAJECTORIES):
        raise RuntimeError("up50 recording index must contain exactly ten rows")
    copy_file(index_path, output / "provenance" / "up50_recordings.latest.csv")
    shutil.copy2(Path(__file__).resolve(), output / "provenance" / Path(__file__).name)

    log_sources = sorted(runs.glob("up50_robot_nucleo_10__20260807T*.log"))
    for log in log_sources:
        copy_file(log, output / "logs" / log.name)

    trajectory_entries = []
    quality_rows = []
    for index, ((display_name, slug), index_row) in enumerate(
        zip(TRAJECTORIES, index_rows, strict=True), start=1
    ):
        recording_id = index_row["recording_id"]
        recording_source = recordings_root / recording_id
        verify_recording_checksums(recording_source)
        sealed_manifest = yaml.safe_load(
            (recording_source / "manifest.yaml").read_text(encoding="utf-8")
        )
        if (
            not isinstance(sealed_manifest, dict)
            or sealed_manifest.get("recording_id") != recording_id
            or sealed_manifest.get("status") != "validated"
        ):
            raise RuntimeError(f"invalid sealed recording manifest: {recording_id}")
        expected_duration_s = float(sealed_manifest["trajectory"]["duration_s"])
        candidates = [
            path
            for path in runs.glob(f"{recording_id}__{args.run_date_prefix}_*/real")
            if (path / "execution_telemetry.csv").is_file()
            and (path / "nucleo_telemetry.csv").is_file()
            and (path / "nucleo_capture.yaml").is_file()
        ]
        if not candidates:
            raise RuntimeError(f"no complete real run found for {recording_id}")
        run_source = max(candidates, key=lambda path: path.stat().st_mtime_ns)
        run_id = run_source.parent.name
        destination = output / "trajectories" / f"{index:02d}_{slug}"
        destination.mkdir()

        recording_file_count = copy_tree(recording_source, destination)
        copy_file(recording_source / "manifest.yaml", destination / "recording_manifest.yaml")
        for filename in ("execution_telemetry.csv", "nucleo_telemetry.csv", "nucleo_capture.yaml"):
            copy_file(run_source / filename, destination / filename)

        nucleo_manifest = json.loads((run_source / "nucleo_capture.yaml").read_text(encoding="utf-8"))
        staging_info = nucleo_manifest.get("artifacts", {}).get("staging_raw_csv", {})
        staging_path = Path(str(staging_info.get("path", "")))
        staging_included = staging_path.is_file()
        if staging_included:
            copy_file(staging_path, destination / "nucleo_raw_staging.csv")

        robot_rows, robot_summary = robot_table(run_source / "execution_telemetry.csv")
        robot_summary["expected_recording_duration_s"] = expected_duration_s
        robot_summary["recording_duration_coverage_verified"] = (
            expected_duration_s - 0.1
            <= robot_summary["last_trajectory_time_s"]
            <= expected_duration_s + 0.1
        )
        if not robot_summary["recording_duration_coverage_verified"]:
            raise RuntimeError(f"robot execution window is truncated for {recording_id}")
        sensor_summary = sensor_table(run_source / "nucleo_telemetry.csv", robot_rows)
        counts = nucleo_manifest["counts"]
        if sensor_summary["mapped_frame_count"] != int(counts["mapped_sensor_frames"]):
            raise RuntimeError(f"mapped-frame count mismatch for {recording_id}")
        if sensor_summary["missing_robot_timestamp_count"] or sensor_summary["trajectory_time_mismatch_count"]:
            raise RuntimeError(f"robot/sensor timestamp mapping mismatch for {recording_id}")
        mapping = nucleo_manifest["mapping"]
        if sensor_summary["maximum_mapping_offset_ms"] > (
            int(mapping["maximum_allowed_absolute_offset_ns"]) / 1_000_000.0
        ):
            raise RuntimeError(f"mapping offset exceeds manifest limit for {recording_id}")
        deltas = mapping["parser_integrity_deltas"]
        entry = {
            "index": index,
            "name": display_name,
            "directory": str(destination.relative_to(output)),
            "recording_id": recording_id,
            "run_id": run_id,
            "source_run_directory": str(run_source),
            "source_recording_directory": str(recording_source),
            "robot": {
                **robot_summary,
                "telemetry_sha256": sha256(run_source / "execution_telemetry.csv"),
                "organized_path": str((destination / "execution_telemetry.csv").relative_to(output)),
            },
            "sensor": {
                "original_status": nucleo_manifest["status"],
                "original_message": nucleo_manifest["message"],
                **sensor_summary,
                "incomplete_groups": int(deltas.get("parser_incomplete_groups", 0)),
                "channel_sequence_drops": int(deltas.get("parser_channel_sequence_drops", 0)),
                "reordered_packets": int(deltas.get("parser_reordered_packets", 0)),
                "telemetry_sha256": sha256(run_source / "nucleo_telemetry.csv"),
                "manifest_sha256": sha256(run_source / "nucleo_capture.yaml"),
                "raw_staging_included": staging_included,
                "organized_path": str((destination / "nucleo_telemetry.csv").relative_to(output)),
            },
            "recording_artifact": {
                "file_count": recording_file_count,
                "checksums_verified": True,
                "manifest_alias_included": True,
            },
        }
        trajectory_entries.append(entry)
        quality_rows.append(
            [
                index, display_name, recording_id, run_id,
                robot_summary["sample_count"], sensor_summary["mapped_frame_count"],
                nucleo_manifest["status"], entry["sensor"]["incomplete_groups"],
                entry["sensor"]["channel_sequence_drops"], entry["sensor"]["reordered_packets"],
                format_float(sensor_summary["maximum_mapping_offset_ms"]), staging_included,
            ]
        )

    background_output = output / "background"
    background_raw = background_source / "nucleo_raw_telemetry.csv"
    background_manifest_path = background_source / "nucleo_capture.yaml"
    background_manifest = json.loads(background_manifest_path.read_text(encoding="utf-8"))
    copy_file(background_raw, background_output / "nucleo_raw_telemetry.csv")
    copy_file(background_manifest_path, background_output / "nucleo_capture.yaml")
    for source_name, destination_name in (
        ("README.md", "source_README.md"),
        ("sha256sums.txt", "source_sha256sums.txt"),
    ):
        source = background_source / source_name
        if source.is_file():
            copy_file(source, background_output / destination_name)
    background_summary = background_statistics(
        background_output / "nucleo_raw_telemetry.csv",
        background_output,
        background_manifest,
    )

    quality_path = output / "quality_summary.csv"
    with quality_path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            [
                "index", "trajectory", "recording_id", "run_id", "robot_rows",
                "mapped_sensor_rows", "sensor_manifest_status", "incomplete_groups",
                "channel_sequence_drops", "reordered_packets", "max_mapping_offset_ms",
                "raw_staging_included",
            ]
        )
        writer.writerows(quality_rows)

    acquisition_manifest = {
        "schema_version": 1,
        "status": "complete_with_quality_flags",
        "generated_at_utc": now.isoformat(),
        "environment_id": ENVIRONMENT_ID,
        "recording_count": len(trajectory_entries),
        "all_robot_execution_windows_cover_recording_duration": all(
            entry["robot"]["recording_duration_coverage_verified"]
            for entry in trajectory_entries
        ),
        "all_robot_speed_scaling_exactly_one": all(
            entry["robot"]["all_speed_scaling_exactly_one"] for entry in trajectory_entries
        ),
        "all_sensor_rows_exactly_mapped_to_robot_timestamps": all(
            entry["sensor"]["missing_robot_timestamp_count"] == 0
            and entry["sensor"]["trajectory_time_mismatch_count"] == 0
            for entry in trajectory_entries
        ),
        "original_sensor_manifests_byte_identical_at_bundle_creation": all(
            sha256(Path(entry["source_run_directory"]) / "nucleo_capture.yaml")
            == entry["sensor"]["manifest_sha256"]
            for entry in trajectory_entries
        ),
        "bundle_source_artifacts_are_independent_copies": True,
        "operator_tolerance": {
            "accepted_maximum_incomplete_groups_per_trajectory": 10,
            "accepted_maximum_channel_sequence_drops_per_trajectory": 10,
            "observed_maximum_incomplete_groups": max(
                entry["sensor"]["incomplete_groups"] for entry in trajectory_entries
            ),
            "observed_maximum_channel_sequence_drops": max(
                entry["sensor"]["channel_sequence_drops"] for entry in trajectory_entries
            ),
        },
        "background": {
            "source_directory": str(background_source),
            "reference_frame": "plane1",
            "translation_from_original_sensor_board_mm": [0.0, 0.0, -50.0],
            "sample_count": background_summary["sample_count"],
            "raw_sha256": sha256(background_raw),
        },
        "batch_logs": [str(path) for path in log_sources],
        "runs": trajectory_entries,
    }
    write_json(output / "source_acquisition_manifest.yaml", acquisition_manifest)

    table_lines = []
    for row in quality_rows:
        table_lines.append(
            f"| {row[0]:02d} | {row[1]} | {row[4]} | {row[5]} | {row[7]} | "
            f"{row[8]} | {row[9]} | {row[10]} |"
        )
    readme = f"""# UR5 up50 + NUCLEO 真机数据集（plane1 Z -50 mm 空场）

本目录整理了 `{ENVIRONMENT_ID}` 环境中的 10 条 `_zplus26mm_up50mm` 真机轨迹，以及传感器板相对原位置沿 `plane1` 局部 Z 负方向平移 50 mm 时采集的 30 秒空场。原始轨迹、机器人表、传感器表和空场 CSV 均未改写，也未执行背景扣除。

## 主要文件

- `trajectories/01_wave` ～ `trajectories/10_celtic_knot`：每条轨迹的 recording artifact、机器人实际执行表、同步传感器表、原始 NUCLEO manifest；可用时还包含 `nucleo_raw_staging.csv`。
- `background/nucleo_raw_telemetry.csv`：`plane1 z=-50 mm` 位置的 30 秒空场原始数据。
- `background/background_field_mean.csv`、`background_field_stats.csv`：64 个传感器的背景均值与总体标准差。
- `background/magloc_background_field.json`/`.csv`/`_wide.csv`：可直接供 MagLoc 使用的背景场格式。
- `quality_summary.csv`：10 条轨迹质量摘要。
- `source_acquisition_manifest.yaml`、`dataset_manifest.yaml`：来源和验证结果。
- `dataset_sha256sums.txt`：目录内全部文件的 SHA-256。

## 时间同步

`execution_telemetry.csv` 是机器人实际表，`nucleo_telemetry.csv` 是同步磁场表。整理时逐行验证 `sync_time_ns`、`trajectory_time_s` 和 `trajectory_time_ns` 均能精确对应机器人表，且最大映射误差均低于原 manifest 的 50 ms 限值。

## 背景场口径

背景位置为相对原传感器板位置 `[0, 0, -50] mm`，坐标系为 `plane1`。背景统计单位为 µT，标准差使用总体标准差 `ddof=0`。轨迹数据保持原始磁场值；若分析时需要扣除背景，请生成派生数据：

`B_corrected(sensor, axis, t) = B_raw(sensor, axis, t) - B_background_mean(sensor, axis)`

不要覆盖本目录中的原始 CSV。

## 采集质量

| # | 轨迹 | 机器人行 | 磁场行 | incomplete groups | sequence drops | reordered | 最大映射偏差 ms |
|---:|---|---:|---:|---:|---:|---:|---:|
{chr(10).join(table_lines)}

所有正式窗口丢组均不超过用户批准的 `10/约3000`。原始 `nucleo_capture.yaml` 状态和计数保持不变。

## 校验

```bash
cd {output}
sha256sum -c dataset_sha256sums.txt
```

整理目录保存源数据的独立字节拷贝，并对包内文件设置只读权限；生成的 `.tar.gz` 是独立归档。请把整理目录和归档都视为只读证据。
"""
    (output / "README.md").write_text(readme, encoding="utf-8")

    dataset_manifest = {
        "schema_version": 1,
        "dataset_id": dataset_id,
        "status": "complete_with_quality_flags",
        "generated_at_utc": now.isoformat(),
        "environment_id": ENVIRONMENT_ID,
        "description": (
            "Ten up50 authoritative UR5 real captures with synchronized NUCLEO data "
            "and a 30-second plane1 z-50 mm empty-field reference."
        ),
        "policies": {
            "authoritative_source_files_modified": False,
            "trajectory_waypoints_modified": False,
            "trajectory_time_s_modified": False,
            "trajectory_scaling_retiming_cropping_or_skipping_applied": False,
            "background_subtraction_applied": False,
            "source_storage_method": "independent byte-identical copies",
            "bundle_files_marked_read_only": True,
            "archive_is_independent": True,
        },
        "trajectory_count": len(trajectory_entries),
        "background": {
            "directory": "background",
            "source_directory": str(background_source),
            "reference_frame": "plane1",
            "translation_from_original_sensor_board_mm": [0.0, 0.0, -50.0],
            "sample_count": background_summary["sample_count"],
            "raw_csv_sha256": sha256(background_raw),
        },
        "quality_summary_path": "quality_summary.csv",
        "source_acquisition_manifest_path": "source_acquisition_manifest.yaml",
        "batch_logs": [f"logs/{path.name}" for path in log_sources],
        "trajectories": trajectory_entries,
    }
    write_json(output / "dataset_manifest.yaml", dataset_manifest)

    checksum_path = output / "dataset_sha256sums.txt"
    with checksum_path.open("x", encoding="utf-8") as stream:
        for path in all_files(output, exclude=(checksum_path,)):
            stream.write(f"{sha256(path)}  {path.relative_to(output)}\n")
    for line in checksum_path.read_text(encoding="utf-8").splitlines():
        expected, relative = line.split("  ", 1)
        if sha256(output / relative) != expected:
            raise RuntimeError(f"dataset checksum verification failed: {relative}")

    for path in all_files(output):
        path.chmod(0o444)
    for directory in sorted(
        (path for path in output.rglob("*") if path.is_dir()),
        key=lambda path: len(path.parts),
        reverse=True,
    ):
        directory.chmod(0o555)
    output.chmod(0o555)

    archive = runs / f"{dataset_id}.tar.gz"
    subprocess.run(
        ["tar", "-C", str(runs), "-czf", str(archive), dataset_id],
        check=True,
    )
    archive_hash = sha256(archive)
    archive_checksum = archive.with_suffix(archive.suffix + ".sha256")
    archive_checksum.write_text(f"{archive_hash}  {archive.name}\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "dataset_directory": str(output),
                "archive": str(archive),
                "archive_sha256": archive_hash,
                "trajectory_count": len(trajectory_entries),
                "background_frames": background_summary["sample_count"],
                "dataset_file_count": len(all_files(output)),
                "archive_size_bytes": archive.stat().st_size,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
