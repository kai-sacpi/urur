"""Pure-file v3 prior-bank identity and seed-diversity admission tests."""
from __future__ import annotations

from pathlib import Path
import pytest

from workstation_magtrack.continuous_augmented.replay_v3 import prepare as prep


def write(path: Path, value: dict | str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(prep.encoded(value) if isinstance(value, dict) else value.encode())


def make_banks(root: Path) -> None:
    for bank_index, (label, batch) in enumerate({"v3": prep.BATCH, **prep.PRIOR_BANKS}.items()):
        directory = root / "continuous_augmented/batches" / batch
        rows = []
        for i in range(2):
            name = f"{label}_source_{i}"
            csv_file, metadata_file = f"csv/{name}.csv", f"metadata/{name}.json"
            write(directory / csv_file, f"unique source body {label}/{i}\n")
            seed = bank_index*100+i
            write(directory / metadata_file, {"trajectory_id": name, "seed": seed,
                "seed_streams": {key: seed*10+offset+10000 for offset, key in enumerate(("space", "speed", "orientation", "duration"))}})
            rows.append({"trajectory_id": name, "csv_file": csv_file, "metadata_file": metadata_file,
                         "sha256": prep.sha(directory / csv_file), "metadata_sha256": prep.sha(directory / metadata_file), "seed": seed})
        write(directory / "manifest.json", {"batch_id": batch, "records": rows})
    refresh_report(root)


def refresh_report(root: Path) -> None:
    names = {"v3": prep.BATCH, **prep.PRIOR_BANKS}
    write(root / "continuous_augmented/batches" / prep.BATCH / "reports/cross_bank_diversity.json", {
        "manifest_sha256": {label: prep.sha(root / "continuous_augmented/batches" / batch / "manifest.json") for label, batch in names.items()},
        "all_disjoint": True, "passed": True, "not_robot_pass": True,
        "integrity_errors": [], "collisions": {key: [] for key in ("trajectory_ids", "csv_sha256", "authority_seeds", "seed_streams")}})


def evidence(root: Path) -> dict:
    return prep.cross_bank_evidence(root, expected_counts={"v3": 2, "v2": 2, "preview": 2})


def test_disjoint_actual_banks_bind_report_to_three_manifests(tmp_path: Path) -> None:
    make_banks(tmp_path)
    result = evidence(tmp_path)
    assert set(result["cross_bank_manifest_sha256"]) == {"v3", "v2", "preview"}
    assert len(result["cross_bank_diversity_sha256"]) == 64


@pytest.mark.parametrize("kind", ["trajectory_ids", "csv_sha256", "authority_seeds", "seed_streams"])
def test_report_claim_cannot_hide_actual_cross_bank_intersection(tmp_path: Path, kind: str) -> None:
    make_banks(tmp_path)
    base = tmp_path / "continuous_augmented/batches"
    new_bank, old_bank = base / prep.BATCH, base / prep.PRIOR_BANKS["v2"]
    new_manifest, old_manifest = prep.read(new_bank / "manifest.json"), prep.read(old_bank / "manifest.json")
    row, old = new_manifest["records"][0], old_manifest["records"][0]
    metadata_path = new_bank / row["metadata_file"]
    metadata, old_metadata = prep.read(metadata_path), prep.read(old_bank / old["metadata_file"])
    if kind == "trajectory_ids":
        row["trajectory_id"] = metadata["trajectory_id"] = old["trajectory_id"]
    elif kind == "csv_sha256":
        write(new_bank / row["csv_file"], (old_bank / old["csv_file"]).read_text())
        row["sha256"] = prep.sha(new_bank / row["csv_file"])
    elif kind == "authority_seeds":
        row["seed"] = metadata["seed"] = old["seed"]
    else:
        # A reused value under a DIFFERENT role must still be rejected.
        metadata["seed_streams"]["space"] = old_metadata["seed_streams"]["orientation"]
    write(metadata_path, metadata)
    row["metadata_sha256"] = prep.sha(metadata_path)
    write(new_bank / "manifest.json", new_manifest)
    refresh_report(tmp_path)
    with pytest.raises(ValueError, match=kind):
        evidence(tmp_path)


def test_cross_report_stale_manifest_binding_is_rejected(tmp_path: Path) -> None:
    make_banks(tmp_path)
    report_path = tmp_path / "continuous_augmented/batches" / prep.BATCH / "reports/cross_bank_diversity.json"
    report = prep.read(report_path)
    report["manifest_sha256"]["v2"] = "0"*64
    write(report_path, report)
    with pytest.raises(ValueError, match="three actual manifests"):
        evidence(tmp_path)


def test_prior_csv_tamper_is_not_accepted_as_new_baseline(tmp_path: Path) -> None:
    make_banks(tmp_path)
    old_bank = tmp_path / "continuous_augmented/batches" / prep.PRIOR_BANKS["preview"]
    row = prep.read(old_bank / "manifest.json")["records"][0]
    write(old_bank / row["csv_file"], "changed old source\n")
    with pytest.raises(ValueError, match="actual CSV/metadata hash"):
        evidence(tmp_path)
