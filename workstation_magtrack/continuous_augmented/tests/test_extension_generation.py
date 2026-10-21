"""Extension-adapter tests; no new experiment artifacts are generated on Mac."""
from __future__ import annotations

import copy
from pathlib import Path
import sys
import unittest
from unittest import mock

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from workstation_magtrack.continuous_augmented import generate_extension as extension


class ExtensionGenerationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parent = extension.ROOT/"batches"/extension.REFERENCE_PARENT_BATCH
        cls.old = extension.prior_authorities(cls.parent)
        cls.designs = extension.bank.design_records(extension.MASTER_SEED)

    def test_120_new_ids_seeds_and_480_streams_are_disjoint(self):
        result = extension.assert_disjoint(self.designs, self.old)
        self.assertEqual(result["new_unique_seed_stream_count"], 480)
        self.assertEqual(result["trajectory_id_overlap"], 0)
        self.assertEqual(result["authority_seed_overlap"], 0)
        self.assertEqual(result["seed_stream_overlap"], 0)

    def test_seed_or_stream_collision_is_rejected_before_writes(self):
        for field in ("trajectory_id", "seed", "seed_streams"):
            broken = copy.deepcopy(self.designs)
            broken[0][field] = copy.deepcopy(self.old[0][field])
            with self.assertRaisesRegex(ValueError, "disjoint"):
                extension.assert_disjoint(broken, self.old)

    def test_historical_reference_origin_and_parent_manifest_are_truthful(self):
        context, refs = extension.checked_reference_sources(self.parent)
        self.assertEqual(context["reference_parent_manifest_sha256"], extension.REFERENCE_PARENT_MANIFEST_SHA256)
        self.assertFalse(context["reference_sources_are_execution_imports"])
        self.assertEqual(len(refs), 2)
        for source, target, attributes in refs:
            self.assertTrue(source.is_relative_to(self.parent.resolve()/"provenance/reference_sources"))
            self.assertTrue(target.startswith("reference_sources/"))
            self.assertTrue(attributes["not_current_parent_repository_source"])
            self.assertEqual(attributes["reference_parent_source_sha256"], extension.bank.preview.digest(source.read_bytes()))

    def test_corrupt_reference_bytes_fail_sha_check_without_editing_any_file(self):
        original = Path.read_bytes
        reference = self.parent/"provenance/reference_sources"/extension.REFERENCE_NAMES[0]
        def read_bytes(path):
            payload = original(path)
            return payload+b"\n# not authoritative\n" if path.resolve() == reference.resolve() else payload
        with mock.patch.object(Path, "read_bytes", read_bytes):
            with self.assertRaisesRegex(ValueError, "reference parent SHA256 mismatch"):
                extension.checked_reference_sources(self.parent)

    def test_parent_manifest_is_hash_pinned(self):
        original = Path.read_bytes
        manifest = self.parent/"manifest.json"
        def read_bytes(path):
            payload = original(path)
            return payload+b"\n" if path.resolve() == manifest.resolve() else payload
        with mock.patch.object(Path, "read_bytes", read_bytes):
            with self.assertRaisesRegex(ValueError, "hash-pinned"):
                extension.checked_reference_sources(self.parent)


if __name__ == "__main__":
    unittest.main()
