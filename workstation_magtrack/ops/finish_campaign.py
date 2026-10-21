#!/usr/bin/env python3
"""Continue the already-started simulation campaign after compilation exits.

Runs only the task's isolated mock entry point. This supervisor never launches
the real driver, changes authorities, retries with relaxed limits, or uses a
sensor. It leaves a truthful summary even when individual trajectories fail.
"""

from __future__ import annotations

import fcntl
import json
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
BATCH = "authority_v4"
DIRECTORY = ROOT / "runs" / BATCH
INDEX = ROOT / "planning/authority_index_v4.json"


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def summarize(state: str, phases: dict[str, int]) -> dict:
    status = json.loads((DIRECTORY / "status.json").read_text())
    records = status["trajectories"]
    result = {
        "updated_utc": now(), "state": state, "batch": BATCH,
        "authority_index": str(INDEX), "trajectory_count": len(records),
        "phase_counts": {phase: dict(Counter(r.get(phase, {}).get("status", "pending")
                                             for r in records.values()))
                         for phase in ("compile", "build", "inspect", "play")},
        "phase_exit_codes": phases,
        "complete_passed": sum(all(r.get(p, {}).get("status") == "passed"
                                   for p in ("compile", "build", "inspect", "play"))
                               for r in records.values()),
        "real_robot_used": False, "sensor_used": False,
    }
    result["workflow_passed"] = (
        {"build", "inspect", "play"}.issubset(phases)
        and all(code == 0 for code in phases.values())
        and result["complete_passed"] == result["trajectory_count"]
    )
    target = DIRECTORY / "campaign_summary.json"
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(target)
    print(json.dumps(result), flush=True)
    return result


def main() -> int:
    if ROOT.name != "workstation_magtrack":
        raise RuntimeError("unexpected workspace")
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    with (DIRECTORY / ".campaign.lock").open("a+") as supervisor_lock:
        fcntl.flock(supervisor_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # The compile coordinator owns its lock until all compiler children
        # finish. Do not infer completion from an unchanging log or GPU load.
        print(f"{now()} waiting for existing compile coordinator", flush=True)
        while True:
            with (DIRECTORY / ".coordinator.lock").open("a+") as coordinator_lock:
                try:
                    fcntl.flock(coordinator_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    time.sleep(5)
                    continue
                status_path = DIRECTORY / "status.json"
                if not status_path.exists():
                    raise RuntimeError("compile status is missing; refusing to infer success")
                records = json.loads(status_path.read_text())["trajectories"]
                expected = {r["trajectory_id"] for r in json.loads(INDEX.read_text())["trajectories"]}
                if set(records) != expected:
                    raise RuntimeError("compile batch does not contain the exact frozen active set")
                if any(r.get("compile", {}).get("status") not in {"passed", "failed"}
                       for r in records.values()):
                    raise RuntimeError("compile exited with interrupted/pending entries; explicit resume required")
                break
        exits: dict[str, int] = {}
        summarize("compile_finished", exits)
        for phase in ("build", "inspect", "play"):
            print(f"{now()} starting isolated mock {phase}", flush=True)
            with (DIRECTORY / f"{phase}_batch.log").open("a") as log:
                command = ["bash", str(ROOT / "runtime/run_batch.sh"), "--ids-file", str(INDEX),
                           "--batch", BATCH, "--phase", phase]
                result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                        check=False)
            exits[phase] = result.returncode
            summarize(f"{phase}_finished", exits)
        postchecks = {
            "inventory": [sys.executable, str(ROOT / "ops/export_replay_index.py"),
                          "--workspace", str(ROOT), "--require-all"],
            # The parent environment's system Matplotlib has an old NumPy ABI.
            # Refresh exact JSON/Markdown evidence here without changing any
            # dependency. Planned figures were already generated on the Mac.
            "independent_source_audit": [str(ROOT.parent / ".venv/bin/python"),
                "-m", "validation.audit", "--workspace", str(ROOT),
                "--authority-index", str(INDEX), "--runtime-index",
                str(DIRECTORY / "status.json"), "--no-plots"],
        }
        for name, command in postchecks.items():
            with (DIRECTORY / f"{name}.log").open("a") as log:
                result = subprocess.run(command, cwd=ROOT, stdout=log,
                                        stderr=subprocess.STDOUT, check=False)
            exits[name] = result.returncode
        result = summarize("finished", exits)
        return 0 if result["workflow_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
