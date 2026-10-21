"""No-hardware Linux PTY regression for the production capture process.

A separately spawned simulated device emits the real four-channel HUB wire
format at 100 Hz. The unchanged PersistentSensor default owns the PTY through
its own spawned process while the parent revalidates the frozen parking bundle.
This is a software scheduling regression, NOT physical sensor/robot acceptance.
No physical serial path can be supplied, no ROS is imported and no goal is sent.
All new evidence (including normal staging and locks) stays beneath --output.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import multiprocessing
from multiprocessing.connection import Connection
import os
from pathlib import Path
import pty
import select
import struct
import sys
import time
import tty
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from collection.drift import INTEGRITY_COUNTERS
from collection.prepared_parking import PreparedParkingBundle
from collection.sensor import PersistentSensor


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_new(path: Path, value: Any) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def hub_group(index: int) -> bytes:
    """One actual-format group; no fake in-memory parser/session is injected."""
    if isinstance(index, bool) or not isinstance(index, int) or index < 0:
        raise ValueError("group index must be a nonnegative integer")
    packets = []
    for channel in range(1, 5):
        # Counts deliberately vary by sensor and channel, not all-zero payloads.
        axes = tuple(channel * 100 + sensor * 3 + axis
                     for sensor in range(16) for axis in range(3))
        tick = (1000 + index * 10) & 0xFFFFFFFF
        payload = b"L5" + bytes((index % 16,)) + struct.pack(">48hI", *axes, tick)
        packets.append(b"\x55\xaa" + bytes((channel,)) + struct.pack(
            "<II", index & 0xFFFFFFFF, tick) + payload)
    return b"".join(packets)


def _producer(connection: Connection, stop: Any) -> None:
    """Own a new PTY only; never open a user-supplied serial device."""
    master: int | None = None
    slave: int | None = None
    count, control, max_lateness_ns = 0, bytearray(), 0
    started = time.monotonic_ns()
    error: str | None = None
    try:
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        port = os.ttyname(slave)
        if not port.startswith("/dev/pts/"):
            raise RuntimeError("probe requires a newly allocated Linux /dev/pts PTY")
        connection.send({"port": port, "producer_pid": os.getpid(),
                         "started_monotonic_ns": started, "start_method": "spawn"})
        # Keeping the slave open prevents transient EIO before pyserial opens it.
        # Atomic 456-byte group writes also avoid artificial partial startup
        # packets when the normal serial input-buffer reset occurs.
        deadline = time.monotonic_ns()
        while not stop.is_set():
            remaining = (deadline - time.monotonic_ns()) / 1e9
            if remaining > 0 and stop.wait(remaining):
                break
            max_lateness_ns = max(max_lateness_ns, time.monotonic_ns() - deadline)
            packet = memoryview(hub_group(count))
            write_deadline = time.monotonic() + .25
            while packet:
                try:
                    size = os.write(master, packet)
                    if size <= 0:
                        raise RuntimeError("PTY simulated-device write made no progress")
                    packet = packet[size:]
                except BlockingIOError:
                    if stop.is_set():
                        return
                    if time.monotonic() >= write_deadline:
                        raise RuntimeError("simulated PTY transport stalled for more than 0.25s")
                    select.select([], [master], [], .01)
            count += 1
            deadline += 10_000_000
            try:
                command = os.read(master, 4096)
                if command:
                    control.extend(command)
                    if len(control) > 4096:
                        raise RuntimeError("unexpected excess host control traffic to simulated PTY")
            except BlockingIOError:
                pass
    except BaseException as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        result = {"producer_pid": os.getpid(), "groups_emitted": count,
                  "expected_rate_hz": 100, "channel_count": 4,
                  "max_producer_lateness_ms": max_lateness_ns / 1e6,
                  "host_control_bytes_received": len(control),
                  "host_control_hex": control.hex(), "error": error,
                  "stopped_monotonic_ns": time.monotonic_ns()}
        try:
            connection.send(result)
        except (BrokenPipeError, EOFError, OSError):
            pass
        for descriptor in (master, slave):
            if descriptor is not None:
                os.close(descriptor)
        connection.close()


def _reserve_output(workspace: Path, output: Path) -> Path:
    workspace, output = workspace.resolve(), output.resolve()
    if not output.is_relative_to(workspace / "runs") or output == workspace / "runs":
        raise ValueError("--output must be a NEW directory beneath workspace/runs")
    output.mkdir(parents=True, exist_ok=False)
    return output


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values) - 1, math.ceil(fraction * len(values)) - 1)]


def receive_metrics(path: Path, intervals: list[dict[str, Any]]) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    times = [int(row["sensor_rx_sync_time_ns"]) for row in rows]

    def metrics(selected: list[int]) -> dict[str, Any]:
        gaps = [(b - a) / 1e6 for a, b in zip(selected, selected[1:])]
        return {"frames": len(selected), "max_receive_gap_ms": max(gaps, default=None),
                "p99_receive_gap_ms": _percentile(gaps, .99),
                "zero_interval_arrivals": sum(value == 0 for value in gaps),
                "gaps_over_20ms": sum(value > 20 for value in gaps),
                "receive_clock_regressions": sum(value < 0 for value in gaps)}

    return {"complete_raw": metrics(times), "verification_intervals": [
        {**interval, **metrics([stamp for stamp in times
                               if interval["start_monotonic_ns"] <= stamp < interval["end_monotonic_ns"]])}
        for interval in intervals],
        "gap_statistics_are_observational_not_replacement_for_existing_drift_guard": True}


def assess(workspace: Path, output: Path, bundle_path: Path, config_path: Path) -> dict[str, Any]:
    if sys.platform != "linux":
        raise RuntimeError("this no-hardware diagnostic requires Linux PTYs")
    output = _reserve_output(workspace, output)
    report: dict[str, Any] = {
        "schema_version": 1, "status": "failed", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "simulated_wire_transport_capture_process_cpu_isolation_regression_only",
        "physical_hardware_opened": False, "robot_commanded": False,
        "physical_sensor_acceptance": False, "ros_imported": False,
        "timestamps_modified": False, "validation_thresholds_changed": False,
        "parent_pid": os.getpid(), "output": str(output), "parking_bundle": str(bundle_path),
        "parking_bundle_sha256": _sha256(bundle_path / "bundle.json"),
        "source_sha256": {path.name: _sha256(path) for path in (
            Path(__file__), Path(__file__).parents[1] / "collection/sensor.py",
            Path(__file__).parents[1] / "collection/capture_process.py",
            Path(__file__).parents[1] / "collection/drift.py")},
        "verification_intervals": [], "errors": [],
    }
    sensor: PersistentSensor | None = None
    process: Any = None
    parent: Connection | None = None
    stop: Any = None
    try:
        # The constructor's unchanged full verification occurs before serial
        # starts; two MORE identical complete proofs occur after startup-ready.
        bundle = PreparedParkingBundle(workspace, bundle_path, config_path)
        context = multiprocessing.get_context("spawn")
        parent, child = context.Pipe(duplex=False)
        stop = context.Event()
        process = context.Process(target=_producer, args=(child, stop), name="simulated-nucleo-pty")
        process.start()
        child.close()
        if not parent.poll(10):
            raise RuntimeError("simulated PTY producer did not announce readiness within10s")
        hello = parent.recv()
        if not isinstance(hello, dict) or not str(hello.get("port", "")).startswith("/dev/pts/"):
            raise RuntimeError(f"simulated PTY producer failed: {hello}")
        report["simulated_device"] = hello
        # Isolate normal temporary staging/locks too, without injecting a fake
        # session or changing any of the production capture defaults.
        capture_workspace = output / "simulated_capture_workspace"
        (capture_workspace / "runs").mkdir(parents=True)
        sensor = PersistentSensor.start_session(
            capture_workspace / "runs/probe", "software_isolation_probe", hello["port"],
            workspace=capture_workspace, hardware_boot_id="simulated_pty_not_physical_hardware")
        report["sensor_root"] = str(sensor.root)
        report["startup_ready"] = sensor.check_ready()
        report["capture_identity"] = json.loads((sensor.root / "epoch.json").read_text())["capture_process"]
        identity = report["capture_identity"]
        if (identity.get("start_method") != "spawn" or
                identity.get("capture_pid") in (None, os.getpid(), hello["producer_pid"]) or
                identity.get("parent_pid") != os.getpid()):
            raise RuntimeError("production default did not establish a separate spawned capture owner")
        for index in range(2):
            interval = {"verification_index": index + 1, "start_monotonic_ns": time.monotonic_ns()}
            cpu_start = time.process_time()
            bundle.verify()
            interval.update(end_monotonic_ns=time.monotonic_ns(), cpu_s=time.process_time() - cpu_start)
            interval["wall_s"] = (interval["end_monotonic_ns"] - interval["start_monotonic_ns"]) / 1e9
            report["verification_intervals"].append(interval)
            report[f"ready_after_verification_{index + 1}"] = sensor.check_ready()
        time.sleep(2)
        report["final_ready"] = sensor.check_ready()
    except BaseException as exc:
        report["errors"].append(f"{type(exc).__name__}: {exc}")
    finally:
        if sensor is not None:
            try:
                report["session"] = sensor.close()
                report["final_drift"] = json.loads((sensor.root / "drift.json").read_text())
                raw = Path(report["session"]["raw_csv"])
                report["receive_metrics"] = receive_metrics(raw, report["verification_intervals"])
            except BaseException as exc:
                report["errors"].append(f"capture cleanup: {type(exc).__name__}: {exc}")
        if stop is not None:
            stop.set()
        if process is not None:
            process.join(5)
            if process.is_alive():
                # Only our synthetic producer; never terminate the capture owner
                # because an unconfirmed stop must retain its own serial lock.
                process.terminate()
                process.join(5)
                report["errors"].append("simulated PTY producer required termination")
            report["producer_exitcode"] = process.exitcode
        if parent is not None:
            try:
                if parent.poll(1):
                    report["producer_result"] = parent.recv()
                else:
                    report["errors"].append("simulated PTY producer final report missing")
            except (EOFError, OSError) as exc:
                report["errors"].append(f"producer result: {type(exc).__name__}: {exc}")
            parent.close()
        session, producer = report.get("session", {}), report.get("producer_result", {})
        if len(report["verification_intervals"]) != 2:
            report["errors"].append("both full parking verifications did not complete")
        if session.get("status") != "closed" or session.get("capture_error"):
            report["errors"].append("production capture session did not close cleanly")
        parser = session.get("parser", {})
        nonzero = {name: parser.get(name) for name in INTEGRITY_COUNTERS if parser.get(name) != 0}
        if nonzero:
            report["errors"].append(f"nonzero/missing parser integrity counters: {nonzero}")
        if producer.get("error") or producer.get("host_control_bytes_received") != 0:
            report["errors"].append("simulated producer failed or unexpected SYNC/control traffic was observed")
        if session.get("sync_mode") != "reused_existing_stream":
            report["errors"].append("streaming simulated PTY should require no handshake")
        if not report["errors"]:
            report["status"] = "passed_software_isolation_regression_only"
        report["ros_imported"] = any(name == "rclpy" or name.startswith("rclpy.") for name in sys.modules)
        if report["ros_imported"]:
            report["errors"].append("unexpected ROS import in no-hardware diagnostic")
            report["status"] = "failed"
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        _write_new(output / "isolation_report.json", report)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--parking-bundle", type=Path)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()
    bundle = (args.parking_bundle or workspace / "collection/prepared_parking/parking_20261017_v2").resolve()
    config = (args.config or workspace / "collection/config.yaml").resolve()
    report = assess(workspace, args.output, bundle, config)
    print(json.dumps({"status": report["status"], "report": str(args.output / "isolation_report.json"),
                      "errors": report["errors"]}, indent=2))
    raise SystemExit(0 if not report["errors"] else 1)


if __name__ == "__main__":
    main()
