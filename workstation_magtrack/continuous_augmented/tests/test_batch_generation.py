"""Small CPU tests for the new bank adapter; no old authority is edited."""
from __future__ import annotations

from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from workstation_magtrack.continuous_augmented import core, generate as preview, generate_batch as bank


class BatchDesignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.designs = bank.design_records()

    def test_exact_quota_and_unique_ids_seeds(self):
        self.assertEqual(len(self.designs), 120)
        self.assertEqual(len({m["trajectory_id"] for m in self.designs}), 120)
        self.assertEqual(len({m["seed"] for m in self.designs}), 120)
        fixed = [m for m in self.designs if m["group_id"] == "fixed_height"]
        self.assertEqual(Counter(m["height_layer_mm"] for m in fixed), Counter({h: 6 for h in bank.HEIGHT_LAYERS_MM}))
        for family in bank.FAMILIES:
            full = [m for m in self.designs if m["group_id"] == "full_height_one_way" and m["family_id"] == family]
            self.assertEqual({m["variant_index"] for m in full}, set(range(1, 7)))
            self.assertEqual(Counter(m["direction"] for m in full), Counter({"ascending_height": 3, "descending_height": 3}))

    def test_all_multi_axis_and_independent_seed_streams(self):
        for meta in self.designs:
            self.assertEqual(meta["orientation"]["mode"], "smooth_multi_axis")
            controls = np.array(meta["orientation"]["euler_xyz_control_points_rad"])
            self.assertTrue(np.all(np.ptp(controls, axis=0) > 1e-5))
            self.assertEqual(meta["seed_streams"], preview._seed_streams(meta["seed"]))
            self.assertEqual(len(set(meta["seed_streams"].values())), 4)

    def test_new_seed_changes_id_and_controls_without_touching_original(self):
        original = bank.design_batch_metadata(0, height_mm=70)
        before = copy.deepcopy(original)
        other = bank.design_batch_metadata(0, height_mm=70, master_seed=bank.MASTER_SEED+10000)
        self.assertNotEqual(original["trajectory_id"], other["trajectory_id"])
        self.assertNotEqual(original["curve"]["position_control_points_m"], other["curve"]["position_control_points_m"])
        self.assertEqual(original, before)

    def test_control_hulls_open_direction_and_height_contract(self):
        for meta in self.designs:
            proof = core.path_certificate(meta)
            self.assertTrue(proof["strictly_monotone_projection"])
            self.assertGreater(proof["endpoint_distance_m"], .10)
            self.assertLessEqual(proof["tilt_bound_deg"], 25.)
            cp = np.array(meta["curve"]["position_control_points_m"])
            base = np.array(meta["curve"]["baseline_control_points_m"])
            delta = np.array(meta["curve"]["deformation_control_points_m"])
            np.testing.assert_allclose(base+delta, cp, atol=1e-16)
            self.assertLessEqual(np.max(np.abs(cp[:, :2])), .130)
            h = -cp[:, 2]
            if meta["group_id"] == "fixed_height":
                np.testing.assert_array_equal(h, np.full(8, meta["height_layer_mm"]/1000.))
            else:
                self.assertAlmostEqual(float(h.min()), .070)
                self.assertAlmostEqual(float(h.max()), .200)
                signs = np.diff(h)
                self.assertTrue(np.all(signs > 0.) if meta["direction"] == "ascending_height" else np.all(signs < 0.))

    def test_reject_undeclared_or_contradictory_quota(self):
        for kwargs in ({"height_mm": 75}, {"height_mm": 70, "variant_index": 2},
                       {"variant_index": 7}, {"variant_index": 1, "direction": "descending_height"}):
            with self.assertRaises(ValueError):
                bank.design_batch_metadata(0, **kwargs)

    def test_full_variant_directions_not_bound_to_family(self):
        for family_index in range(6):
            for variant in range(1, 7):
                meta = bank.design_batch_metadata(family_index, variant_index=variant)
                self.assertEqual(meta["direction"], "ascending_height" if variant % 2 else "descending_height")

    def test_single_new_source_rewrite_identical_or_reject_changed_bytes(self):
        meta = preview.finalize_duration(bank.design_batch_metadata(0, height_mm=70), samples=1025)
        with tempfile.TemporaryDirectory() as directory:
            csv = Path(directory)/"source.csv"
            sampled = preview.sampled_authority(meta)
            first = preview.write_authority(csv, *sampled)
            payload = csv.read_bytes()
            again = preview.write_authority(csv, *sampled)
            self.assertEqual(first["source_sha256"], again["source_sha256"])
            self.assertEqual(payload, csv.read_bytes())
            t, p, q, phase = sampled
            p = p.copy()
            p[20, 0] += .001
            with self.assertRaises(FileExistsError):
                preview.write_authority(csv, t, p, q, phase)
            self.assertEqual(payload, csv.read_bytes())

    def test_environment_fingerprint_is_canonical_and_excludes_data(self):
        fingerprint, refs = bank._environment_snapshot()
        canonical = json.dumps(fingerprint["files"], sort_keys=True, separators=(",", ":")).encode("utf-8")
        self.assertEqual(fingerprint["sha256"], hashlib.sha256(canonical).hexdigest())
        self.assertTrue(all(name.split("/")[0] not in ("trajectories", "recordings") for name in fingerprint["files"]))
        self.assertEqual(len(refs), len(fingerprint["files"])+1)
        self.assertEqual(fingerprint["lock_sha256"], hashlib.sha256((bank.WORKSPACE/"curobo.lock").read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
