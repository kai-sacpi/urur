"""Read-only replay of saved sensor clocks; never recertify a physical run.

Run on Ubuntu24 with the corrected collection code. JSON goes to stdout so
the caller can save a NEW evidence file, without touching the old session.
No ROS, serial transport, or robot modules are imported.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from collection import drift


def sha256(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def assess(directory: Path) -> dict:
    raw = directory / "nucleo_raw_telemetry.csv"
    original_hash = sha256(raw)
    original = json.loads((directory / "drift.json").read_text())
    tracker = drift.DeviceClockDrift(require_stable_start=True)
    with raw.open(newline="") as stream:
        for row in csv.DictReader(stream):
            tracker.update_row(row)
    report = tracker.report()
    if sha256(raw) != original_hash:
        raise RuntimeError(f"raw input changed while assessing: {raw}")
    windows = []
    for path in sorted((directory / "windows").glob("*/window.json")):
        value = json.loads(path.read_text())
        windows.append({"manifest": str(path), "sha256": sha256(path),
                        "original_status": value["status"],
                        "original_quality": value["quality"],
                        "parser_counter_deltas": value.get("parser_counter_deltas", {})})
    return {
        "session": str(directory), "raw_sha256": original_hash,
        "raw_unchanged": True, "original_drift_file": str(directory / "drift.json"),
        "original_drift_sha256": sha256(directory / "drift.json"),
        "original_drift_status": original["status"],
        "original_drift_error": original.get("error"),
        "corrected_policy_replay": report,
        "original_windows_not_reclassified": windows,
        "physical_recording_accepted": False,
        "scope": "offline_clock_policy_regression_not_sensor_integrity_or_real_trial_acceptance",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps({
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "offline_only": True, "hardware_opened": False, "robot_commanded": False,
        "source_sha256": {"drift.py": sha256(Path(drift.__file__)),
                          "assess_startup_baseline.py": sha256(Path(__file__))},
        "sessions": [assess(path.resolve()) for path in args.session],
    }, indent=2))


if __name__ == "__main__":
    main()
