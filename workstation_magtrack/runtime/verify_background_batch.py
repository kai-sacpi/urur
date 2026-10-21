#!/usr/bin/env python3
"""Read-only verification of raw backgrounds; write only new final reports.

No serial port is opened. No capture files, manifests or quality thresholds are
modified. All acceptance metrics are recomputed independently of the collector.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "src/hx_waypoint_nucleo"))
from hx_waypoint_nucleo.capture import RAW_CAPTURE_COLUMNS  # noqa: E402

ZERO_COUNTERS = (
    "malformed_packets", "bytes_discarded", "source_sequence_drops",
    "reordered_packets", "duplicate_packets", "timeout_groups",
    "pending_evictions", "unexpected_channel_packets",
    "duplicate_or_reordered_groups",
)
SENSOR_COLUMNS = tuple(
    f"sensor_{sensor:02d}_{axis}_ut" for sensor in range(64) for axis in "xyz"
)
BEIJING = timezone(timedelta(hours=8))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def timestamp(ns, tz=timezone.utc):
    return datetime.fromtimestamp(ns / 1e9, tz).isoformat(timespec="microseconds")


def check_quality(count, coverage, counters):
    require(count >= 11940, f"only {count} complete groups")
    require(coverage >= 119.9, f"only {coverage} s actual sample coverage")
    incomplete = counters["incomplete_groups"]
    require(incomplete >= 0, "negative incomplete counter")
    ratio = incomplete / (count + incomplete)
    require(ratio <= 0.005, f"incomplete group ratio {ratio} exceeds 0.005")
    for name in ZERO_COUNTERS:
        require(counters[name] == 0, f"forbidden per-window counter {name}={counters[name]}")
    return ratio


def check_nonoverlap(intervals):
    ordered = sorted(intervals)
    for previous, current in zip(ordered, ordered[1:]):
        require(previous[1] <= current[0], f"overlapping windows: {previous[2]} and {current[2]}")
    return ordered


def scan_csv(path, window_start, window_end, baseline, end):
    """Inspect every row; equal receive timestamps in a read batch are valid."""
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        columns = next(reader)
        require(tuple(columns) == RAW_CAPTURE_COLUMNS, f"CSV schema mismatch: {path}")
        positions = {name: n for n, name in enumerate(columns)}
        sensor_indices = [positions[name] for name in SENSOR_COLUMNS]
        raw_stats = [(positions[name], name.removeprefix("parser_"))
                     for name in columns if name.startswith("parser_")
                     and name != "parser_host_monotonic_ns"]
        previous_stats = dict(baseline)
        first = last = None
        first_wall = last_wall = None
        last_logical = None
        count = 0
        for row in reader:
            count += 1
            label = f"{path}:{count + 1}"
            require(len(row) == len(columns), f"column count mismatch at {label}")
            receive = int(row[positions["sensor_rx_sync_time_ns"]])
            parser_time = int(row[positions["parser_host_monotonic_ns"]])
            wall = int(row[positions["sensor_rx_ros_time_ns"]])
            logical = int(row[positions["logical_seq"]])
            require(window_start <= receive < window_end, f"sample outside window at {label}")
            require(parser_time == receive, f"receive timestamp altered at {label}")
            require(last is None or receive >= last, f"receive time reversed at {label}")
            require(last_logical is None or logical == last_logical + 1, f"duplicate/missing output logical sequence at {label}")
            require(row[positions["channel_ids"]] == "1;2;3;4", f"missing channel at {label}")
            require(0 <= int(row[positions["src_seq"]]) < 16, f"invalid source sequence at {label}")
            require(all(math.isfinite(float(row[n])) for n in sensor_indices), f"nonfinite sensor data at {label}")
            for n, name in raw_stats:
                value = int(row[n])
                require(previous_stats[name] <= value <= end[name], f"invalid cumulative counter {name} at {label}")
                if name in ZERO_COUNTERS:
                    require(value == baseline[name], f"forbidden counter changed inside window: {name} at {label}")
                previous_stats[name] = value
            require(int(row[positions["parser_completed_groups"]]) == logical + 1, f"output count mismatch at {label}")
            if first is None:
                first, first_wall = receive, wall
            last, last_wall, last_logical = receive, wall, logical
        require(count > 0, f"empty CSV: {path}")
        for _, name in raw_stats:
            require(previous_stats[name] == end[name], f"CSV final counter differs from manifest: {name}, {path}")
    return {"frames": count, "first_sample_ns": first, "last_sample_ns": last,
            "first_wall_ns": first_wall, "last_wall_ns": last_wall,
            "actual_sample_coverage_s": (last - first) / 1e9}


def verify_window(run, root):
    raw = run / "nucleo_raw_telemetry.csv"
    manifest_path = run / "nucleo_capture.yaml"
    manifest = json.loads(manifest_path.read_text())
    capture = manifest["capture"]
    require(manifest["quality"]["accepted"] and not manifest["quality"]["reasons"], f"manifest not accepted: {run}")
    require(manifest["experiment"] == "empty_sensor_field", f"wrong experiment: {run}")
    require(not manifest["ur_control_started"] and not manifest["robot_motion_commanded"], f"robot motion declared: {run}")
    require(capture["capture_error"] is None, f"capture error: {run}")
    require(capture["channels"] == [1, 2, 3, 4], f"wrong channel set: {run}")
    require(capture["requested_duration_s"] == 120.0 and capture["capture_clock_duration_s"] == 120.0, f"wrong duration: {run}")
    require(capture["sync_attempts_in_whole_session"] in (0, 1), f"repeated SYNC: {run}")
    start, finish = capture["window_start_monotonic_ns"], capture["window_end_monotonic_ns"]
    require(finish - start == 120_000_000_000, f"window is not exactly 120 s: {run}")
    counters = manifest["counts"]["parser"]
    baseline, end = manifest["parser_counters"]["baseline"], manifest["parser_counters"]["end"]
    require(manifest["parser_counters"]["manifest_scope"] == "per_window_delta", f"unknown counter scope: {run}")
    for name, value in counters.items():
        require(value == end[name] - baseline[name] and value >= 0, f"counter delta mismatch: {name}, {run}")
    before = raw.stat()
    measurements = scan_csv(raw, start, finish, baseline, end)
    count, coverage = measurements["frames"], measurements["actual_sample_coverage_s"]
    require(count == manifest["counts"]["frames_written"] == counters["completed_groups"], f"row count mismatch: {run}")
    require(abs(coverage - capture["actual_sample_coverage_s"]) < 1e-9, f"coverage mismatch: {run}")
    require(measurements["first_wall_ns"] == capture["first_sample_wall_time_ns"] and measurements["last_wall_ns"] == capture["last_sample_wall_time_ns"], f"wall timestamp mismatch: {run}")
    ratio = check_quality(count, coverage, counters)
    raw_sha, manifest_sha = digest(raw), digest(manifest_path)
    require(raw_sha == manifest["artifacts"]["raw_sensor_csv"]["sha256"], f"raw SHA256 mismatch: {run}")
    require(before.st_size == manifest["artifacts"]["raw_sensor_csv"]["size_bytes"], f"raw file size mismatch: {run}")
    after = raw.stat()
    require((before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns), f"raw file changed during verification: {run}")
    stored_hashes = {}
    for line in (run / "sha256sums.txt").read_text().splitlines():
        value, name = line.split(maxsplit=1)
        stored_hashes[name.lstrip("*")] = value
    require(stored_hashes[raw.name] == raw_sha and stored_hashes[manifest_path.name] == manifest_sha, f"stored SHA256 mismatch: {run}")
    session = run.parent.parent
    return {"session": "." if session == root else str(session.relative_to(root)),
            "run": str(run.relative_to(root)), "raw_csv": str(raw), "manifest": str(manifest_path),
            "raw_sha256": raw_sha, "manifest_sha256": manifest_sha, "raw_size_bytes": before.st_size,
            "window_start_monotonic_ns": start, "window_end_monotonic_ns": finish,
            **measurements, "incomplete_groups": counters["incomplete_groups"],
            "incomplete_group_ratio": ratio,
            "first_sample_utc": timestamp(measurements["first_wall_ns"]),
            "last_sample_utc": timestamp(measurements["last_wall_ns"]),
            "first_sample_beijing": timestamp(measurements["first_wall_ns"], BEIJING),
            "last_sample_beijing": timestamp(measurements["last_wall_ns"], BEIJING),
            "sync_mode": capture["sync_mode"], "sync_attempts": capture["sync_attempts_in_whole_session"],
            "required_zero_counters_pass": True}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", required=True, type=Path)
    ap.add_argument("--expected-count", type=int, default=100)
    args = ap.parse_args()
    root = args.root.resolve()
    sessions = [root] + sorted(p for p in (root / "continuations").iterdir() if (p / "status.json").exists())
    runs, session_summary = [], []
    for session in sessions:
        status = json.loads((session / "status.json").read_text())
        require(status["state"] != "recording", f"session still recording: {session}")
        accepted = sorted((session / "accepted").glob("*/nucleo_capture.yaml"))
        rejected = list((session / "rejected").glob("*/nucleo_capture.yaml"))
        require(len(accepted) == status["accepted"] and len(rejected) == status["rejected"], f"directory/status count mismatch: {session}")
        session_summary.append({"session": "." if session == root else str(session.relative_to(root)),
                                "accepted": len(accepted), "rejected": len(rejected),
                                "attempts": status["attempts"], "state": status["state"],
                                "started_at_utc": status["started_at_utc"]})
        runs.extend(p.parent for p in accepted)
    require(len(runs) == args.expected_count, f"expected {args.expected_count} accepted files, found {len(runs)}")
    records = []
    for index, run in enumerate(runs, 1):
        records.append(verify_window(run, root))
        if index % 10 == 0:
            print(f"verified {index}/{len(runs)} raw CSVs, manifests and SHA256", flush=True)
    check_nonoverlap([(r["window_start_monotonic_ns"], r["window_end_monotonic_ns"], r["run"]) for r in records])
    check_nonoverlap([(r["first_wall_ns"], r["last_wall_ns"], r["run"]) for r in records])
    require(len({r["raw_sha256"] for r in records}) == len(records), "duplicate whole-file raw capture")
    records.sort(key=lambda r: r["window_start_monotonic_ns"])
    for index, row in enumerate(records, 1):
        row["record_id"] = f"background_{index:03d}"
    generated = datetime.now(timezone.utc)
    summary = {"schema_version": 1, "status": "verified_complete", "root": str(root),
               "generated_at_utc": generated.isoformat(), "generated_at_beijing": generated.astimezone(BEIJING).isoformat(),
               "accepted": len(records), "rejected_retained": sum(s["rejected"] for s in session_summary),
               "total_window_attempts": sum(s["attempts"] for s in session_summary),
               "total_frames": sum(r["frames"] for r in records), "total_window_duration_s": len(records) * 120,
               "min_frames": min(r["frames"] for r in records), "max_frames": max(r["frames"] for r in records),
               "min_actual_sample_coverage_s": min(r["actual_sample_coverage_s"] for r in records),
               "max_incomplete_group_ratio": max(r["incomplete_group_ratio"] for r in records),
               "csv_column_count": len(RAW_CAPTURE_COLUMNS), "sensor_count": 64, "axes_per_sensor": 3,
               "sessions": session_summary, "all_window_intervals_nonoverlapping": True,
               "all_sample_wall_intervals_nonoverlapping": True, "raw_and_manifest_sha256_verified": True,
               "finite_sensor_values_verified": True, "required_zero_counters": ZERO_COUNTERS,
               "required_zero_counters_all_pass": True, "raw_data_modified": False,
               "sample_data_copied_to_mac": False, "verifier_source_sha256": digest(Path(__file__)),
               "limitations": ["Host receive timestamps are not hardware sample-clock timestamps.",
                               "Grouping quality does not independently prove a physically magnet-free environment.",
                               "No long-duration hardware/firmware drift fix is claimed."]}
    index_path = root / "FINAL_INDEX.csv"
    with index_path.open("x", newline="") as stream:
        fields = ["record_id"] + [key for key in records[0] if key != "record_id"]
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(records)
    json_path = root / "FINAL_VERIFICATION.json"
    with json_path.open("x") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    report_path = root / "FINAL_REPORT.zh-CN.md"
    session_table = "\n".join(f"| `{s['session']}` | {s['accepted']} | {s['rejected']} | {s['attempts']} |" for s in session_summary)
    report = f"""# NUCLEO 100 条 120 秒背景场最终验收

验收时间：{summary['generated_at_beijing']}（北京时间）。

## 结论

独立读取并核验 **{len(records)} 条合格真实串口记录**，每条窗口严格 120 s，
互不重叠，总窗口时长 {len(records) * 120} s（200 分钟）。
共 {summary['total_window_attempts']} 次窗口尝试，{summary['rejected_retained']} 条拒收原件全部保留。
不是将多个记录重采样、拼接或改写来凑数；原始 CSV 和 manifest 未修改。

## 逐文件核验

- CSV 共 {summary['csv_column_count']} 列，包含四通道、64 个传感器的 XYZ；逐行检查列结构和所有磁场数值为有限数。
- 总完整组 {summary['total_frames']}；每条完整组范围 {summary['min_frames']}～{summary['max_frames']}，均不少于 11940。
- 最小实际首末样本覆盖 {summary['min_actual_sample_coverage_s']:.9f} s，要求至少 119.9 s。
- 最大不完整组比例 {summary['max_incomplete_group_ratio'] * 100:.6f}%，要求不超过 0.5%。
- 重新计算每个窗口的计数差，核对每行原始累计计数与 manifest baseline/end；所有要求为零的计数增量均为零。
- 原始 CSV 的行数、大小、SHA256 与 manifest 及 sha256sums.txt 一致，manifest 自身的 SHA256 也一致。
- 所有样本在所属半开区间 [start,end) 内；五次有效采集会话之间的 120 s 窗口不重叠，实际样本墙钟区间也不重叠。
- 接收时间戳同一次 read 中允许相等，不将其误判为重复采样；输出逻辑序号逐行连续，无重复的整份 CSV。

## 来源及筛选

| 会话（相对批次根目录） | 合格 | 拒收保留 | 尝试 |
|---|---:|---:|---:|
{session_table}

用户多次确认重启 NUCLEO 后，分别开启独立续采目录；每个启动周期最多一次初始 SYNC，
其后复用串口。此前缺通道的预检失败未生成 120 s 窗口，单列在会话表中，不计入窗口尝试。
原会话 status.json 作为历史故障记录保留；不要把它当作最终总状态，最终结论以本报告
及 FINAL_VERIFICATION.json 为准。

前两次有效会话在接收修复前，后续三次有效会话在接收修复后。
修复仅令串口立即读取已可用字节，空闲时等待一个字节，没有修改解析器的严格 40 ms
组帧期限、质量标准或原始样本。修复说明见 diagnostics/receive_fix__20261016T133243Z/。
旧的长时间运行仍存在跨通道时序漂移证据，见 STOP_AFTER_REBOOT_2240.md；本次完成
不代表已修复固件/采集源的长期漂移，不据全局乱序计数断言线上实际包乱序。

## 文件和时间

根目录：`{root}`

- FINAL_INDEX.csv：每条记录的真实路径、来源会话、样本数、窗口、首末样本时间、质量及哈希。
- FINAL_VERIFICATION.json：机器可读的验收汇总、指标和验证程序源码哈希。
- FINAL_SHA256SUMS.txt：报告、索引及全部 100 份原始 CSV/manifest 的哈希索引。
- 每条记录目录仍保留 nucleo_raw_telemetry.csv、nucleo_capture.yaml、sha256sums.txt。

索引同时提供 UTC 和北京时间（UTC+8）；目录名时间后缀使用 UTC。
传感器时间含原始源/NUCLEO 计数和主机接收时间；主机接收时间不等于硬件精确采样时钟。
本验收只证明既定收包完整性和时间窗口要求；空场为实验设置，未独立证明物理环境绝对无磁源。
未操作机械臂，未重启板子，所有采样数据仍仅存 Ubuntu24，未同步到 Mac。
"""
    with report_path.open("x") as stream:
        stream.write(report)
    with (root / "FINAL_SHA256SUMS.txt").open("x") as stream:
        for path in (index_path, json_path, report_path):
            stream.write(f"{digest(path)}  {path.relative_to(root)}\n")
        for row in records:
            stream.write(f"{row['raw_sha256']}  {Path(row['raw_csv']).relative_to(root)}\n")
            stream.write(f"{row['manifest_sha256']}  {Path(row['manifest']).relative_to(root)}\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
