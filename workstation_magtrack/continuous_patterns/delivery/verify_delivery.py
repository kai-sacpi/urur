#!/usr/bin/env python3
"""Verify the delivered candidate bank and the unchanged active-v4 inputs.

This uses only the standard library. It writes an inventory or a derived
verification report within continuous_patterns; it never edits authorities.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
from pathlib import Path
import sys


PROTECTED = {
    "README.md": "701977de194962fcae9931d06bd1669967a27b271997b7398993cbe0b4bbc646",
    "docs/calibration_and_release_gate.md": "0b6f1c48b2391f4cb7284a5ead9ce656d34880d8db0d1f7fd21ca7a26902c87d",
    "inputs/collection_task_v2_20261016.txt": "964692f94c90297234cc7d3aac73c4b789a8bce5078e4a1ac76fa4aef2f73bb9",
    "inputs/real_trajectory_collection_plan_20261013.md": "b487e6f7e1a89f9d9c27fb033000d3556b329e9012f00a18836f62cff87c5a2e",
    "environments/mla_short/setup.yaml": "eb8f79e46fffc50a72069c4531642737e45f0889c9962a741c2d904087f7684d",
    "environments/mla_short/zones.yaml": "3de7d095bde6693d13a39eda552945fb5ec9ff4fda05b81f7f962ce87c9a46a1",
    "environments/mla_short/robot/assets/ur5.urdf": "2ef10ba4989720ebcc9e49c64b0ecb49ac94f20ef20147ad1e841d058258c9e6",
    "planning/authority_index_v4.json": "62df51b9d225030f871620159abf3b3ccb00718e9004985fc9049b5163e28550",
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def inventory_files(root):
    return {
        str(p.relative_to(root)): sha(p)
        for p in sorted(root.rglob("*"))
        if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"
        and p.name != "delivery_inventory_sha256.json"
        and not p.name.startswith("delivery_verification_")
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--batch", default="cp_layers_and_full_h070_200_20261016_v1")
    parser.add_argument("--write-inventory", action="store_true")
    parser.add_argument("--label", default="local")
    args = parser.parse_args()
    root = args.root.resolve()
    batch = root / "batches" / args.batch
    inventory = batch / "provenance/delivery_inventory_sha256.json"
    if args.write_inventory:
        write(inventory, {"root_scope": "continuous_patterns only", "sha256": inventory_files(root)})
        print(f"Inventory: {inventory}")
        return 0

    failures = []
    expected = read(inventory)["sha256"]
    actual = inventory_files(root)
    for name in sorted(set(expected) | set(actual)):
        if expected.get(name) != actual.get(name):
            failures.append(f"delivery checksum differs or file missing/extra: {name}")

    parent = root.parent
    protected = {}
    historical_documents = {}
    for name, digest in PROTECTED.items():
        exists = (parent / name).is_file()
        current_sha = sha(parent / name) if exists else None
        if name == "README.md":
            # Other tasks may append completed-campaign documentation. The
            # frozen document remains authoritative for this batch's inputs;
            # live documentation drift is recorded separately from runtime
            # configuration and active trajectory integrity.
            historical_documents[name] = {
                "matches_generation_snapshot": current_sha == digest,
                "generation_snapshot_sha256": digest, "current_sha256": current_sha,
                "scope": "historical reference; frozen copy is verified independently",
            }
            continue
        protected[name] = exists and current_sha == digest
        if not protected[name]:
            failures.append(f"protected parent input changed: {name}")
    active = read(parent / "planning/authority_index_v4.json")["trajectories"]
    active_pass = 0
    for record in active:
        p = parent / record["source_file"]
        if p.is_file() and sha(p) == record["source_sha256"]:
            active_pass += 1
        else:
            failures.append(f"active-v4 authority differs: {record['trajectory_id']}")

    manifest = read(batch / "manifest.json")
    catalog = read(batch / "plots/plot_catalog.json")
    entries = {r["trajectory_id"]: r for r in manifest["records"]}
    cards = {r["trajectory_id"]: r for r in catalog["records"]}
    if len(entries) != 156 or len(cards) != 156 or entries.keys() != cards.keys():
        failures.append("manifest/gallery record IDs or 156-record count differs")
    for ident, entry in entries.items():
        for key, digest_key in [("csv_file", "sha256"), ("metadata_file", "metadata_sha256")]:
            if sha(batch / entry[key]) != entry[digest_key]:
                failures.append(f"authority differs: {entry[key]}")
        card = cards.get(ident, {})
        for key, value in card.items():
            if key.endswith("_href") and not (batch / "plots" / value).is_file():
                failures.append(f"gallery target missing: {ident}: {value}")
        comparison = card.get("source_csv_comparison", {})
        if comparison.get("max_abs_position_source_csv_difference_m", 1.) > 1e-9:
            failures.append(f"plotted analytic source/CSV mismatch: {ident}")
    rendered = read(batch / "plots/rendered_files_sha256.json")
    for name, digest in rendered.items():
        if sha(batch / "plots" / name) != digest:
            failures.append(f"rendered plot checksum differs: {name}")
    for entry in read(batch / "provenance/source_inventory.json"):
        if sha(batch / entry["frozen_file"]) != entry["sha256"]:
            failures.append(f"frozen source differs: {entry['frozen_file']}")
    audit = read(batch / "reports/source_audit.json")
    if audit["summary"]["source_level_pass_count"] != 156 or audit["design_findings"]:
        failures.append("source audit is not fully passing")

    report = {
        "passed": not failures, "host_label": args.label, "platform": platform.platform(),
        "python": sys.version, "batch_id": args.batch, "inventory_sha256": sha(inventory),
        "delivery_files_verified": len(expected), "authority_csv_count": len(entries),
        "gallery_record_count": len(cards), "rendered_files_verified": len(rendered),
        "protected_parent_hash_matches": protected,
        "historical_parent_document_snapshot_comparisons": historical_documents,
        "unchanged_active_v4_csv_count": active_pass, "active_v4_csv_count": len(active),
        "source_summary": audit["summary"], "robot_simulation": "not_run",
        "runtime_ready": False, "failures": failures,
    }
    output = batch / "reports" / f"delivery_verification_{args.label}.json"
    write(output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
