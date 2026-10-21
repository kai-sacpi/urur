#!/usr/bin/env python3
"""Summarize a reachable-subset build index without changing its evidence."""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict
from pathlib import Path


def read(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-index", type=Path, required=True)
    parser.add_argument("--conditions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    conditions = {r["condition_id"]: r for r in read(args.conditions)}
    latest: dict[str, dict[str, str]] = {}
    for row in read(args.build_index):
        latest[row["condition_id"]] = row
    fields = (
        "condition_id", "location_id", "direction_id", "z_cm", "status",
        "recording_id", "recording_path", "recording_manifest_sha256", "message",
    )
    rows: list[dict[str, str]] = []
    for cid, condition in conditions.items():
        event = latest.get(cid, {})
        rows.append({
            "condition_id": cid,
            "location_id": condition["location_id"],
            "direction_id": condition["direction_id"],
            "z_cm": f"{100.0 * float(condition['p_A_z_m']):.3f}",
            "status": event.get("status", "missing"),
            "recording_id": event.get("recording_id", ""),
            "recording_path": event.get("recording_path", ""),
            "recording_manifest_sha256": event.get("recording_manifest_sha256", ""),
            "message": event.get("message", ""),
        })
    rows.sort(key=lambda r: (r["location_id"], r["direction_id"], float(r["z_cm"])))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    counts = Counter(r["status"] for r in rows)
    by_location: dict[str, Counter[str]] = defaultdict(Counter)
    for row in rows:
        by_location[row["location_id"]][row["status"]] += 1
    print("conditions", len(rows), "status", dict(counts))
    for location, status in sorted(by_location.items()):
        print(location, dict(status))
    print("wrote", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
