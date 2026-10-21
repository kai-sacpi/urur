"""Frozen provenance is verified as bytes, never imported or executed."""
from __future__ import annotations

import builtins
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

from workstation_magtrack.continuous_augmented.replay_v3 import prepare as prep


ACTUAL_FILES = ("__init__.py", "core.py", "generate.py", "generate_batch.py",
                "generate_extension.py", "common.py")
REFERENCE_FILES = ("trajectory_generator.py", "generate_new_board_esophageal_up50_trajectories.py")
REFERENCE_ROLE = "historical_open_route_reference_only_not_imported"


def put(path: Path, value: bytes | dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(prep.encoded(value) if isinstance(value, dict) else value)


@dataclass
class FrozenFixture:
    root: Path
    batch: Path
    parent: Path
    inventory: dict[str, Any]
    manifest: dict[str, Any]

    def save_inventory(self) -> None:
        self.manifest["provenance"] = deepcopy(self.inventory)
        put(self.batch / "provenance/source_inventory.json", self.inventory)


@pytest.fixture
def frozen(tmp_path: Path) -> FrozenFixture:
    root = tmp_path / "workstation_magtrack"
    batch = root / "continuous_augmented/batches" / prep.BATCH
    parent = root / "continuous_augmented/batches" / prep.PRIOR_BANKS["v2"]
    parent_sources = []
    for name in REFERENCE_FILES:
        relative = f"provenance/reference_sources/{name}"
        put(parent / relative, f"# frozen parent reference: {name}\n".encode())
        parent_sources.append({"frozen_file": relative, "sha256": prep.sha(parent / relative),
                               "origin_path": str(root / "historical_repository" / name)})
    parent_provenance = {"sources": parent_sources}
    put(parent / "provenance/source_inventory.json", parent_provenance)
    put(parent / "manifest.json", {"batch_id": prep.PRIOR_BANKS["v2"],
                                   "provenance": parent_provenance})
    binding = {"reference_parent_batch": prep.PRIOR_BANKS["v2"],
               "reference_parent_manifest_sha256": prep.sha(parent / "manifest.json"),
               "reference_parent_manifest_origin": str(parent / "manifest.json"),
               "reference_sources_are_execution_imports": False}
    sources = []
    for name in ACTUAL_FILES:
        relative = f"provenance/generator_sources/{name}"
        payload = f'raise RuntimeError("must never execute frozen {name}")\n'.encode()
        put(batch / relative, payload)
        origin = (root / "generation/common.py" if name == "common.py"
                  else root / "continuous_augmented" / name)
        sources.append({"frozen_file": relative, "sha256": prep.sha(batch / relative),
                        "origin_path": str(origin), "source_role": "actual_generation_source"})
    for original in parent_sources:
        relative = original["frozen_file"]
        put(batch / relative, (parent / relative).read_bytes())
        sources.append({"frozen_file": relative, "sha256": prep.sha(batch / relative),
                        "origin_path": str(parent / relative), "source_role": REFERENCE_ROLE,
                        "reference_parent_frozen_file": relative,
                        "reference_parent_source_sha256": original["sha256"],
                        "not_current_parent_repository_source": True, **binding})
    fixture = FrozenFixture(root, batch, parent, {"sources": sources, **binding},
                            {"batch_id": prep.BATCH})
    fixture.save_inventory()
    return fixture


def test_complete_inventory_has_portable_hash_binding_and_never_imports_generator(
    frozen: FrozenFixture, monkeypatch: pytest.MonkeyPatch,
) -> None:
    original_import = builtins.__import__

    def guarded_import(name: str, *args: Any, **kwargs: Any) -> Any:
        if any(name.endswith("." + filename[:-3]) for filename in ACTUAL_FILES if filename != "__init__.py"):
            raise AssertionError(f"provenance verifier imported a generator: {name}")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    result = prep.verify_provenance(frozen.root, frozen.manifest)
    assert result == {
        "provenance_inventory_file": (frozen.batch / "provenance/source_inventory.json").relative_to(frozen.root).as_posix(),
        "provenance_inventory_sha256": prep.sha(frozen.batch / "provenance/source_inventory.json"),
    }


def test_mutated_actual_frozen_source_is_rejected(frozen: FrozenFixture) -> None:
    put(frozen.batch / "provenance/generator_sources/core.py", b"# changed after generation\n")
    with pytest.raises(ValueError, match="frozen provenance source"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_disk_inventory_must_equal_manifest_provenance(frozen: FrozenFixture) -> None:
    altered = deepcopy(frozen.inventory)
    altered["unreviewed_extra_claim"] = True
    put(frozen.batch / "provenance/source_inventory.json", altered)
    with pytest.raises(ValueError, match="inventory differs from manifest"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_duplicate_frozen_inventory_entries_are_rejected(frozen: FrozenFixture) -> None:
    frozen.inventory["sources"].append(deepcopy(frozen.inventory["sources"][0]))
    frozen.save_inventory()
    with pytest.raises(ValueError, match="frozen provenance source"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_missing_extension_source_is_rejected_even_with_matching_inventory(frozen: FrozenFixture) -> None:
    frozen.inventory["sources"] = [entry for entry in frozen.inventory["sources"]
                                   if entry["frozen_file"] != "provenance/generator_sources/generate_extension.py"]
    frozen.save_inventory()
    with pytest.raises(ValueError, match="actual generation source inventory"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_actual_generation_source_set_cannot_be_silently_extended(frozen: FrozenFixture) -> None:
    relative = "provenance/generator_sources/unreviewed_extra.py"
    put(frozen.batch / relative, b"# unused but inaccurately claimed execution source\n")
    frozen.inventory["sources"].append({"frozen_file": relative, "sha256": prep.sha(frozen.batch / relative),
                                        "origin_path": str(frozen.root / "unreviewed_extra.py"),
                                        "source_role": "actual_generation_source"})
    frozen.save_inventory()
    with pytest.raises(ValueError, match="actual generation source inventory"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_historical_parent_sha_must_bind_actual_parent_manifest(frozen: FrozenFixture) -> None:
    frozen.inventory["reference_parent_manifest_sha256"] = "f" * 64
    for entry in frozen.inventory["sources"]:
        if entry["source_role"] == REFERENCE_ROLE:
            entry["reference_parent_manifest_sha256"] = "f" * 64
    frozen.save_inventory()
    with pytest.raises(ValueError, match="parent provenance binding"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_historical_origin_must_be_parent_frozen_file_not_live_repository(frozen: FrozenFixture) -> None:
    entry = next(entry for entry in frozen.inventory["sources"] if entry["source_role"] == REFERENCE_ROLE)
    entry["origin_path"] = str(frozen.root / "current_repository" / REFERENCE_FILES[0])
    frozen.save_inventory()
    with pytest.raises(ValueError, match="historical reference source provenance"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_rehashed_historical_copy_cannot_diverge_from_parent_frozen_source(frozen: FrozenFixture) -> None:
    entry = next(entry for entry in frozen.inventory["sources"] if entry["source_role"] == REFERENCE_ROLE)
    put(frozen.batch / entry["frozen_file"], b"# rehashed local source, but not the actual reviewed parent\n")
    entry["sha256"] = prep.sha(frozen.batch / entry["frozen_file"])
    entry["reference_parent_source_sha256"] = entry["sha256"]
    frozen.save_inventory()
    with pytest.raises(ValueError, match="historical reference source provenance"):
        prep.verify_provenance(frozen.root, frozen.manifest)


def test_mutated_parent_reference_bytes_are_not_hidden_by_matching_v3_inventory(frozen: FrozenFixture) -> None:
    put(frozen.parent / "provenance/reference_sources" / REFERENCE_FILES[0], b"# parent bytes mutated\n")
    with pytest.raises(ValueError, match="historical reference source provenance"):
        prep.verify_provenance(frozen.root, frozen.manifest)
