#!/usr/bin/env python3
"""Run a shortest-route canary, then fixed-time compile/mock/inspect/play of 120.

Only the existing isolated simulation entry point is called. The supervisor
never starts the real driver, reads a sensor, changes source inputs or relaxes a
failed gate. Individual terminal failures remain failures while other routes
continue; infrastructure errors or interruption stop further dispatch.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Any

try:
    from .prepare import BATCH, ID_RE, encoded, immutable_outputs, protected, read, sha, validate_sources
    from .export_index import PHASES, export, source_identity
except ImportError:
    from prepare import BATCH, ID_RE, encoded, immutable_outputs, protected, read, sha, validate_sources
    from export_index import PHASES, export, source_identity


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_bytes(encoded(value))
    temporary.replace(path)


def phase_complete(status: dict[str, Any], names: list[str], phase: str) -> bool:
    records = status.get("trajectories", {})
    prerequisite = {"compile": None, "build": "compile", "inspect": "build", "play": "inspect"}[phase]
    eligible = names if prerequisite is None else [name for name in names if records.get(name, {}).get(prerequisite, {}).get("status") == "passed"]
    return all(records.get(name, {}).get(phase, {}).get("status") in {"passed", "failed"} for name in eligible)


def verify_state(status: dict[str, Any], index: dict[str, Any], root: Path) -> None:
    expected = {row["trajectory_id"]: row for row in index["trajectories"]}
    if status.get("workspace") != str(root) or status.get("environment_id") != "mla_short":
        raise ValueError("campaign state belongs to a different workspace/environment")
    for name, entry in status.get("trajectories", {}).items():
        if name not in expected or entry.get("source") != source_identity(expected[name]):
            raise ValueError(f"campaign state contains a nonauthority or different source: {name}")


def summary(status: dict[str, Any], index: dict[str, Any], state: str,
            attempts: list[dict[str, Any]], inventory: dict[str, Any] | None = None) -> dict[str, Any]:
    names = {row["trajectory_id"] for row in index["trajectories"]}
    entries = status.get("trajectories", {})
    complete = sum(all(entries.get(name, {}).get(p, {}).get("status") == "passed" for p in PHASES) for name in names)
    result = {"updated_utc": now(), "state": state, "source_batch_id": BATCH, "expected_trajectory_count": len(names),
              "observed_trajectory_count": len(entries), "complete_four_stage_passed": complete,
              "phase_counts": {p: dict(Counter(entries.get(name, {}).get(p, {}).get("status", "pending") for name in names)) for p in PHASES},
              "command_attempts": attempts, "ready_on_this_host": None if inventory is None else inventory["ready_on_this_host"],
              "robot_used": False, "sensor_used": False, "simulation_only": True, "real_execution_authorized": False,
              "workflow_passed": False}
    result["workflow_passed"] = bool(state == "complete" and len(names) == 120 and complete == len(names)
                                     and set(entries) == names and inventory is not None
                                     and inventory["ready_on_this_host"] == len(names)
                                     and attempts and all(item["exit_code"] == 0 for item in attempts))
    return result


class Campaign:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.root = args.workspace.resolve()
        self.directory = self.root / "runs" / args.batch
        self.directory.mkdir(parents=True, exist_ok=True)
        self.status_path = self.directory / "status.json"
        self.summary_path = self.directory / "campaign_summary.json"
        self.current: subprocess.Popen[Any] | None = None
        self.stop = threading.Event()
        self.attempts: list[dict[str, Any]] = []
        self.index = read(self.root / "continuous_augmented/replay/planning/execution_index.json")
        if self.root.name != "workstation_magtrack" or self.index != validate_sources(self.root):
            raise ValueError("frozen source index differs from validated 120-route source bank")
        snapshot = read(self.root / "continuous_augmented/replay/planning/protected_inputs.json")
        if snapshot.get("sha256") != protected(self.root):
            raise ValueError("protected old authorities/environment changed")

    def read_status(self) -> dict[str, Any]:
        if not self.status_path.exists():
            return {"workspace": str(self.root), "environment_id": "mla_short", "trajectories": {}}
        status = read(self.status_path)
        verify_state(status, self.index, self.root)
        return status

    def save_summary(self, state: str, inventory: dict[str, Any] | None = None, error: str = "") -> dict[str, Any]:
        status_error = ""
        try:
            status = self.read_status()
        except (OSError, ValueError, KeyError, TypeError) as invalid_status:
            status_error = str(invalid_status)
            status = {"trajectories": {}}
        result = summary(status, self.index, state, self.attempts, inventory)
        if status_error:
            result.update(workflow_passed=False, status_validation_error=status_error)
        if error:
            result["error"] = error
        atomic_json(self.summary_path, result)
        print(json.dumps(result), flush=True)
        return result

    def interrupted(self, signum: int, frame: Any) -> None:
        if self.stop.is_set():
            return
        self.stop.set()
        if self.current is not None and self.current.poll() is None:
            # The existing coordinator owns cleanup of its separately-sessioned
            # compiler and ROS groups. Signal only our child coordinator, never
            # tmux sessions, unrelated batches, ROS domains, or the real driver.
            try:
                self.current.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass

    def wait_existing(self) -> None:
        for batch in self.args.wait_batch:
            lock_path = self.root / "runs" / batch / ".coordinator.lock"
            if not lock_path.parent.exists():
                raise ValueError(f"requested wait batch does not exist: {batch}")
            with lock_path.open("a+") as handle:
                while not self.stop.is_set():
                    try:
                        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        fcntl.flock(handle, fcntl.LOCK_UN)
                        break
                    except BlockingIOError:
                        self.stop.wait(5)
        if self.stop.is_set():
            raise KeyboardInterrupt

    def run_phase(self, names: list[str], label: str, phase: str) -> None:
        if self.stop.is_set():
            raise KeyboardInterrupt
        subset = self.directory / f"{label}_ids.json"
        immutable_outputs({subset: encoded({"trajectories": names})})
        command = ["bash", str(self.root / "runtime/run_batch.sh"), "--ids-file", str(subset),
                   "--batch", self.args.batch, "--phase", phase, "--jobs", str(self.args.jobs),
                   "--compile-timeout", str(self.args.compile_timeout), "--adopt-existing-compiled"]
        log_path = self.directory / f"{label}_{phase}.log"
        started = now()
        with log_path.open("a") as output:
            output.write(json.dumps({"started_utc": started, "command": command}) + "\n")
            output.flush()
            self.current = subprocess.Popen(command, cwd=self.root, stdout=output, stderr=subprocess.STDOUT,
                                            start_new_session=True)
            # Signal may arrive during Popen before self.current is assigned.
            # In that race the handler can only set stop, so catch it here.
            if self.stop.is_set() and self.current.poll() is None:
                self.current.send_signal(signal.SIGINT)
            try:
                # Polling permits graceful own-coordinator cancellation. Its
                # own finally blocks then stop all compiler/mock process groups.
                while self.current.poll() is None:
                    time.sleep(.25)
                exit_code = self.current.returncode
            finally:
                if self.current.poll() is None:
                    self.current.send_signal(signal.SIGINT)
                    self.current.wait()
                self.current = None
        self.attempts.append({"label": label, "phase": phase, "started_utc": started, "finished_utc": now(),
                              "exit_code": exit_code, "log": log_path.relative_to(self.root).as_posix()})
        self.save_summary(f"{label}_{phase}_finished")
        if self.stop.is_set() or exit_code in (130, -signal.SIGINT, -signal.SIGTERM):
            raise KeyboardInterrupt
        if not phase_complete(self.read_status(), names, phase):
            raise RuntimeError(f"{label}/{phase} did not leave terminal results for all eligible IDs; infrastructure failure")
        # A nonzero child may represent one trajectory failing a strict gate.
        # Continue the remaining trajectories, but retain nonzero in the final
        # summary so no failed stage can be re-labelled successful.

    def run_locked(self) -> int:
        signal.signal(signal.SIGINT, self.interrupted)
        signal.signal(signal.SIGTERM, self.interrupted)
        self.wait_existing()
        all_rows = sorted(self.index["trajectories"], key=lambda row: (row["duration_s"], row["trajectory_id"]))
        if self.args.canary:
            canaries = self.args.canary
        else:
            targets = [[r for r in all_rows if r["group_id"] == "fixed_height" and r["profile"]["lower_m"] == .07],
                       [r for r in all_rows if r["group_id"] == "full_height_one_way" and r["profile"]["direction"] == "ascending_height"],
                       [r for r in all_rows if r["group_id"] == "full_height_one_way" and r["profile"]["direction"] == "descending_height"]]
            canaries = [rows[0]["trajectory_id"] for rows in targets]
        if (not 1 <= len(canaries) <= 3 or len(set(canaries)) != len(canaries)
                or any(name not in {r["trajectory_id"] for r in all_rows} for name in canaries)):
            raise ValueError("request one to three distinct canaries from exact120 authority")
        remaining = [row["trajectory_id"] for row in all_rows if row["trajectory_id"] not in canaries]
        plan = {"source_index_sha256": sha(self.root / "continuous_augmented/replay/planning/execution_index.json"),
                "batch": self.args.batch, "canary_ids": canaries, "remaining_ids": remaining, "expected_trajectory_count": 120}
        immutable_outputs({self.directory / "campaign_plan.json": encoded(plan)})
        self.save_summary("started")
        for label, names in (("canary", canaries), (f"remaining{len(remaining)}", remaining)):
            for phase in PHASES:
                self.run_phase(names, label, phase)
            inventory = export(self.root, [self.status_path])
            self.save_summary(f"{label}_inventory_exported", inventory)
        inventory = export(self.root, [self.status_path])
        result = self.save_summary("complete", inventory)
        return 0 if result["workflow_passed"] else 2

    def run(self) -> int:
        with (self.root / "continuous_augmented/replay/.campaign.lock").open("a+") as global_lock, (self.directory / ".campaign.lock").open("a+") as lock:
            # A second supervisor must not overwrite the active owner's summary.
            # The shared lock also prevents different-batch owners from racing
            # over the same reserved domains and replay inventory files.
            fcntl.flock(global_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                return self.run_locked()
            except KeyboardInterrupt:
                self.save_summary("interrupted")
                return 130
            except Exception as error:
                self.save_summary("failed", error=str(error))
                return 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--batch", default=BATCH)
    parser.add_argument("--jobs", type=int, choices=(1, 2), default=2)
    parser.add_argument("--compile-timeout", type=float, default=3600.)
    parser.add_argument("--canary", action="append", default=[])
    parser.add_argument("--wait-batch", action="append", default=[])
    args = parser.parse_args()
    if (not ID_RE.fullmatch(args.batch) or any(not ID_RE.fullmatch(name) for name in args.wait_batch)
            or not math.isfinite(args.compile_timeout) or args.compile_timeout <= 0):
        raise ValueError("unsafe batch names or invalid positive compile timeout")
    campaign = Campaign(args)
    try:
        return campaign.run()
    except BlockingIOError:
        print("Another supervisor already owns this campaign; no state was overwritten", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
