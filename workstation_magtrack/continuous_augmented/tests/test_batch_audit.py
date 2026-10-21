"""Bank quota, provenance and independence tests; no generator/robot imports."""
from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
SPEC = importlib.util.spec_from_file_location("augmented_bank_audit", ROOT/"batch_audit.py")
assert SPEC is not None and SPEC.loader is not None
bank = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bank)


def fixture_record(index: int, family: str, *, height: int | None = None,
                   variant: int = 1, direction: str | None = None) -> tuple[dict, dict]:
    seed = 100000+index
    children = np.random.SeedSequence(seed).spawn(4)
    streams = {name: int(child.generate_state(1, dtype=np.uint64)[0])
               for name, child in zip(bank.STREAM_NAMES, children, strict=True)}
    full = height is None
    identifier = f"bank_{index:03d}_{family}"
    group = "full_height_one_way" if full else "fixed_height"
    profile = ({"mode": group, "lower_m": .07, "upper_m": .20, "direction": direction} if full
               else {"mode": "fixed", "lower_m": height/1000., "upper_m": height/1000., "direction": "constant"})
    h = np.linspace(.07, .20, 8) if full else np.full(8, height/1000.)
    if direction == "descending_height":
        h = h[::-1]
    controls = np.c_[np.linspace(-.05, .05, 8), np.zeros(8), -h]
    record = {"trajectory_id": identifier, "group_id": group, "family_id": family,
              "variant_index": variant, "height_layer_mm": height, "seed": seed,
              "height_profile": copy.deepcopy(profile), "orientation_mode": "smooth_multi_axis", "draw_duration_s": 30.,
              "direction": direction if full else "constant"}
    meta = {"trajectory_id": identifier, "group_id": group, "family_id": family,
            "variant_index": variant, "height_layer_mm": height,
            "direction": direction if full else "constant",
            "seed": seed, "seed_streams": streams, "height_profile": profile,
            "draw_duration_s": 30.,
            "curve": {"position_control_points_m": controls.tolist()},
            "orientation": {"mode": "smooth_multi_axis", "euler_xyz_control_points_rad":
                            [[0., .01, .02], [.02, .03, -.01], [.01, -.02, .06]]},
            "time_warp": {"density_control_points_positive": [.7, 1.2, 1.0, .9, 1.1]}}
    return record, meta


def bank_fixture() -> tuple[list[dict], list[dict]]:
    pairs = []
    for height in bank.LAYERS_MM:
        for family in bank.FAMILIES:
            pairs.append(fixture_record(len(pairs), family, height=height))
    for family in bank.FAMILIES:
        for variant in range(1, 7):
            pairs.append(fixture_record(len(pairs), family, variant=variant,
                                        direction="ascending_height" if variant % 2 else "descending_height"))
    return [pair[0] for pair in pairs], [pair[1] for pair in pairs]


def provenance_fixture(directory: Path) -> tuple[Path, dict, list[dict], Path]:
    workspace = directory/"workstation_magtrack"
    environment = workspace/"environments/mla_short"
    environment.mkdir(parents=True)
    (environment/"setup.yaml").write_text("rod_length_m: 0.190\n")
    (environment/"zones.yaml").write_text("minimum_clearance_m: 0.005\n")
    (environment/"robot").mkdir()
    (environment/"robot/collision_spheres.yaml").write_text("spheres: frozen\n")
    (workspace/"curobo.lock").write_text("compiler: pinned\n")
    batch = workspace/"continuous_augmented/batches/test"
    batch.mkdir(parents=True)
    fingerprint = bank.environment_identity(environment)
    entries = []

    def freeze(name: str, payload: bytes) -> None:
        path = batch/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        entries.append({"origin_path": "/portable/source", "frozen_file": name,
                        "sha256": bank.audit.sha256(path)})

    for name in ("core.py", "generate.py", "generate_batch.py", "common.py"):
        freeze(f"provenance/generator_sources/{name}", f"# frozen {name}\n".encode())
    for name in fingerprint["files"]:
        freeze(f"provenance/environment_snapshot/{name}", (environment/name).read_bytes())
    freeze("provenance/curobo.lock", (workspace/"curobo.lock").read_bytes())
    inventory = batch/"provenance/source_inventory.json"
    inventory.write_text(json.dumps({"sources": entries, "environment_fingerprint": fingerprint})+"\n")
    meta = {"trajectory_id": "test", "provenance_inventory_file": "provenance/source_inventory.json",
            "provenance_inventory_sha256": bank.audit.sha256(inventory)}
    manifest = {"environment_fingerprint": fingerprint}
    return batch, manifest, [meta], environment


class BatchQuotaTests(unittest.TestCase):
    def test_valid_120_exact_quotas_and_independent_streams(self):
        records, metadata = bank_fixture()
        result = bank.quota_check(records, metadata, {"old_preview"})
        self.assertTrue(result["passed"], result["errors"])
        self.assertEqual(result["group_counts"], {"fixed_height": 84, "full_height_one_way": 36})
        self.assertEqual(result["independent_stream_count"], 480)

    def test_preview_collision_and_duplicate_ids_rejected(self):
        records, metadata = bank_fixture()
        result = bank.quota_check(records, metadata, {records[0]["trajectory_id"]})
        self.assertFalse(result["passed"])
        self.assertTrue(result["preview_ID_overlap"])
        records[-1]["trajectory_id"] = records[0]["trajectory_id"]
        self.assertFalse(bank.quota_check(records, metadata, set())["passed"])

    def test_missing_layer_family_rejected_not_trusted_from_count(self):
        records, metadata = bank_fixture()
        metadata[0]["height_profile"]["lower_m"] = .08
        metadata[0]["height_profile"]["upper_m"] = .08
        records[0]["height_layer_mm"] = 80
        result = bank.quota_check(records, metadata, set())
        self.assertTrue(any("14 layers" in error for error in result["errors"]))

    def test_manifest_height_pose_and_duration_labels_bound_to_metadata(self):
        records, metadata = bank_fixture()
        records[0]["height_profile"]["lower_m"] = .09
        records[1]["orientation_mode"] = "fixed"
        records[2]["draw_duration_s"] = 10.
        result = bank.quota_check(records, metadata, set())
        self.assertEqual(sum("labels disagree with metadata" in error for error in result["errors"]), 3)

    def test_full_height_direction_label_must_match_geometry(self):
        records, metadata = bank_fixture()
        metadata[-1]["height_profile"]["direction"] = "ascending_height"
        records[-1]["direction"] = "ascending_height"
        result = bank.quota_check(records, metadata, set())
        self.assertTrue(any("analytical endpoints" in error for error in result["errors"]))
        self.assertTrue(any("three ascending" in error for error in result["errors"]))

    def test_fixed_or_single_axis_orientation_rejected(self):
        records, metadata = bank_fixture()
        metadata[0]["orientation"]["mode"] = "fixed"
        metadata[1]["orientation"]["euler_xyz_control_points_rad"] = [[0., 0., 0.], [0., 0., .1]]
        result = bank.quota_check(records, metadata, set())
        self.assertTrue(any("all 120" in error for error in result["errors"]))
        self.assertTrue(any("at least two axes" in error for error in result["errors"]))

    def test_duplicated_or_nonreproducible_random_stream_rejected(self):
        records, metadata = bank_fixture()
        metadata[-1]["seed_streams"]["space"] = metadata[0]["seed_streams"]["space"]
        result = bank.quota_check(records, metadata, set())
        self.assertTrue(any("SeedSequence" in error for error in result["errors"]))
        self.assertTrue(any("globally unique" in error for error in result["errors"]))

    def test_nonpositive_or_constant_speed_density_rejected(self):
        records, metadata = bank_fixture()
        metadata[0]["time_warp"]["density_control_points_positive"] = [1., 1., 1.]
        metadata[1]["time_warp"]["density_control_points_positive"] = [1., -.1, 2.]
        result = bank.quota_check(records, metadata, set())
        self.assertEqual(sum("random speed field" in error for error in result["errors"]), 2)


class FrozenProvenanceTests(unittest.TestCase):
    def test_full_frozen_environment_and_code_identity_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            args = provenance_fixture(Path(directory))
            result = bank.provenance_check(*args)
            self.assertTrue(result["passed"], result["errors"])
            self.assertEqual(result["environment_file_count"], 3)

    def test_recordings_and_trajectories_excluded_but_robot_assets_are_not(self):
        with tempfile.TemporaryDirectory() as directory:
            batch, manifest, metadata, environment = provenance_fixture(Path(directory))
            for folder in ("trajectories", "recordings"):
                (environment/folder).mkdir()
                (environment/folder/"new.csv").write_text("new output, not environment\n")
            self.assertTrue(bank.provenance_check(batch, manifest, metadata, environment)["passed"])
            (environment/"robot/collision_spheres.yaml").write_text("changed spheres\n")
            self.assertFalse(bank.provenance_check(batch, manifest, metadata, environment)["passed"])

    def test_frozen_generator_tamper_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            batch, manifest, metadata, environment = provenance_fixture(Path(directory))
            (batch/"provenance/generator_sources/core.py").write_text("modified\n")
            result = bank.provenance_check(batch, manifest, metadata, environment)
            self.assertTrue(any("frozen provenance SHA256 mismatch" in error for error in result["errors"]))

    def test_curobo_lock_and_metadata_inventory_binding_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            batch, manifest, metadata, environment = provenance_fixture(Path(directory))
            (environment.parent.parent/"curobo.lock").write_text("different compiler\n")
            metadata[0]["provenance_inventory_sha256"] = "0"*64
            result = bank.provenance_check(batch, manifest, metadata, environment)
            self.assertTrue(any("binding mismatch" in error for error in result["errors"]))
            self.assertTrue(any("live mla_short" in error for error in result["errors"]))

    def test_provenance_path_escape_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, "escapes"):
                bank.safe_path(root, "../outside.py")


if __name__ == "__main__":
    unittest.main()
