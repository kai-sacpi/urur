"""Synthetic fixtures only: none of these authorizations is physical evidence."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from .manifest import encoded, immutable_json, require_real_release, sha
from .release import (ACCEPTANCE_KIND, BIAS_MODE, REFERENCE_TYPE, SCOPE,
                      create_raw_acceptance, create_released_manifest, verify_release)
from . import test_commissioning as commissioning_fixture


class RawReleaseTests(unittest.TestCase):
    def setUp(self) -> None:
        # Reuse a nominal two-recording fixture, not its test cases.
        fixture = commissioning_fixture.CommissioningTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.root, self.parent = fixture.root, fixture.parent
        self.parent_value = copy.deepcopy(fixture.parent_value)
        proof = self.parent_value["parking_prepare_bundle"]
        for path_key, sha_key in ((None, "prepared_bundle_manifest_sha256"),
                                  ("gpu_validation_file", "gpu_validation_sha256"),
                                  ("independent_mock_evidence_file", "independent_mock_evidence_sha256")):
            path = self.root / (f"{proof['prepared_bundle']}/bundle.json" if path_key is None else proof[path_key])
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('"unit-test proof; strong verifier explicitly faked"\n')
            proof[sha_key] = sha(path)
        self.parent.write_bytes(encoded(self.parent_value))
        # The real verifier recomputes existing parking proofs.  Tiny synthetic
        # test assets cannot masquerade as that proof: fake only this boundary.
        self.parking = patch(f"{__package__}.release._verify_parking_payload").start()
        self.addCleanup(patch.stopall)
        self.acceptance_path = self.root / "collection/raw_authorizations/test/authorization.json"
        self.acceptance = create_raw_acceptance(self.root, self.parent, "test_installation",
            "Human fixture explicitly authorizes raw FK and sensor acquisition; bias is deferred.", "test_request")
        immutable_json(self.acceptance_path, self.acceptance)
        self.value = create_released_manifest(self.root, self.parent, self.acceptance_path)

    def save_acceptance(self, value: dict) -> dict:
        self.acceptance_path.write_bytes(encoded(value))
        updated = copy.deepcopy(self.value)
        updated["formal_release"]["acceptance"]["sha256"] = sha(self.acceptance_path)
        return updated

    def test_raw_scope_preserves_nominal_history_and_truthful_reference(self) -> None:
        before = {p: sha(p) for p in self.root.rglob("*") if p.is_file()}
        result = verify_release(self.root, self.value)
        self.assertEqual(result["scope"], SCOPE)
        self.assertFalse(result["formal_calibrated_release"])
        self.assertEqual(result["reference"]["type"], REFERENCE_TYPE)
        self.assertFalse(result["reference"]["calibrated_magnet_center"])
        self.assertFalse(result["reference"]["independent_absolute_validation"])
        self.assertEqual(result["bias_policy"]["mode"], BIAS_MODE)
        self.assertFalse(result["bias_policy"]["applied"])
        self.assertTrue(result["recordings_verified"])
        self.assertEqual({k: v for k, v in self.value.items() if k != "formal_release"}, self.parent_value)
        self.assertFalse(self.value["real_execution_authorized"])
        self.assertTrue(self.value["config"]["execution"]["simulation_only"])
        self.assertTrue(all(not row["real_execution_authorized"] for row in self.value["recordings"]))
        self.assertEqual(before, {p: sha(p) for p in self.root.rglob("*") if p.is_file()})
        self.assertEqual(result["operator_asserted_installation"]["rod_length_m"], .190)

    def test_original_nominal_and_old_boolean_release_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            verify_release(self.root, self.parent_value)
        # The original guard never interprets the new scope as booleans granting
        # calibration; root's integration explicitly dispatches the raw scope.
        with self.assertRaises(ValueError):
            require_real_release(self.parent_value)
        mutated = copy.deepcopy(self.value)
        mutated["real_execution_authorized"] = True
        with self.assertRaisesRegex(ValueError, "exact frozen nominal parent"):
            verify_release(self.root, mutated)

    def test_published_parking_operator_expectation_is_not_measured_acceptance(self) -> None:
        parent = copy.deepcopy(self.parent_value)
        parent["parking_prepare_bundle"]["magnetic_interference_status"] = (
            "operator_expected_low_interference_not_measured")
        self.parent.write_bytes(encoded(parent))
        acceptance = create_raw_acceptance(self.root, self.parent, "test_installation",
            "Current user authorizes raw collection, not measured magnetic acceptance.", "parking_metadata")
        path = self.root / "collection/raw_authorizations/parking_metadata.json"
        immutable_json(path, acceptance)
        value = create_released_manifest(self.root, self.parent, path)
        self.assertEqual(verify_release(self.root, value)["scope"], SCOPE)
        parent["parking_prepare_bundle"]["magnetic_interference_status"] = "measured_accepted"
        self.parent.write_bytes(encoded(parent))
        with self.assertRaisesRegex(ValueError, "must not be promoted"):
            create_raw_acceptance(self.root, self.parent, "test_installation", "User permission", "bad_claim")

    def test_scope_cannot_be_upgraded_or_broadened(self) -> None:
        patches = ({"scope": "installation_bound_formal_collection"},
                   {"formal_calibrated_release": True}, {"formal_calibrated_release": 0},
                   {"accepted": True}, {"release_id": "../other"}, {"schema_version": True})
        for changes in patches:
            value = copy.deepcopy(self.value)
            value["formal_release"].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                verify_release(self.root, value)

    def test_reference_bias_and_installation_contracts_are_exact(self) -> None:
        patches = (("reference", {"type": "magnet_ground_truth"}),
                   ("reference", {"calibrated_magnet_center": True}),
                   ("reference", {"independent_absolute_validation": True}),
                   ("reference", {"frame": "S"}),
                   ("bias_policy", {"applied": True}),
                   ("bias_policy", {"applied": 0}),
                   ("bias_policy", {"raw_data_preserved": False}),
                   ("installation", {"rod_length_m": .33365}),
                   ("installation", {"rod_diameter_m": .010}),
                   ("installation", {"robot_ip": "192.168.1.100"}),
                   ("installation", {"unchanged_frozen_setup": 1}))
        for field, changes in patches:
            acceptance = copy.deepcopy(self.acceptance)
            acceptance[field].update(changes)
            value = self.save_acceptance(acceptance)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                verify_release(self.root, value)

    def test_operator_statement_is_not_inferred_from_report_or_model(self) -> None:
        for changes in ({"statement": " "}, {"statement": "x" * 4097}, {"statement": None},
                        {"origin": "model_guess"}, {"authorization": "calibrated_ground_truth"},
                        {"request_id": "../request"}, {"accepted": True}):
            acceptance = copy.deepcopy(self.acceptance)
            acceptance["operator_authorization"].update(changes)
            value = self.save_acceptance(acceptance)
            with self.subTest(changes=str(changes)[:80]), self.assertRaises(ValueError):
                verify_release(self.root, value)

    def test_acceptance_kind_fields_installation_and_request_are_checked(self) -> None:
        for changes in ({"kind": "accepted_joint_calibration"}, {"schema_version": True},
                        {"installation_id": "../installation"}, {"calibration_accepted": True}):
            acceptance = copy.deepcopy(self.acceptance)
            acceptance.update(changes)
            value = self.save_acceptance(acceptance)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                verify_release(self.root, value)
        value = copy.deepcopy(self.value)
        value["formal_release"]["release_id"] = "some_other_request"
        self.save_acceptance(self.acceptance)
        with self.assertRaisesRegex(ValueError, "request identity mismatch"):
            verify_release(self.root, value)

    def test_all_frozen_input_bindings_must_match_exact_parent(self) -> None:
        for key in ("config", "environment_hashes", "recording_inventory_sha256", "parking_binding_sha256", "parent_manifest"):
            acceptance = copy.deepcopy(self.acceptance)
            acceptance["frozen_inputs"][key] = "nonempty_but_not_proof"
            value = self.save_acceptance(acceptance)
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "frozen_inputs"):
                verify_release(self.root, value)

    def test_manifest_row_and_historical_flags_cannot_be_changed(self) -> None:
        for target, key, item in (("root", "real_execution_authorized", True),
                                  ("row", "real_execution_authorized", True),
                                  ("row", "duration_s", 1),
                                  ("row", "unknown", True),
                                  ("execution", "simulation_only", False)):
            value = copy.deepcopy(self.value)
            nested = value if target == "root" else value["recordings"][0] if target == "row" else value["config"]["execution"]
            nested[key] = item
            with self.subTest(target=target, key=key), self.assertRaises(ValueError):
                verify_release(self.root, value)
        value = copy.deepcopy(self.value)
        value["recordings"].reverse()
        with self.assertRaises(ValueError):
            verify_release(self.root, value)

    def test_hash_format_and_current_bytes_are_required(self) -> None:
        for key in ("parent_manifest", "acceptance"):
            for digest in ("x", "F" * 64, "0" * 64, None):
                value = copy.deepcopy(self.value)
                value["formal_release"][key]["sha256"] = digest
                with self.subTest(key=key, digest=digest), self.assertRaises(ValueError):
                    verify_release(self.root, value)

    def test_traversal_absolute_noncanonical_and_symlink_bindings_fail(self) -> None:
        for key in ("parent_manifest", "acceptance"):
            for name in ("../outside.json", str(self.acceptance_path),
                         "collection/./raw_authorizations/test/authorization.json", ""):
                value = copy.deepcopy(self.value)
                value["formal_release"][key]["path"] = name
                with self.subTest(key=key, name=name), self.assertRaises(ValueError):
                    verify_release(self.root, value)
        alias = self.root / "authorization_alias"
        alias.symlink_to(self.acceptance_path.parent, target_is_directory=True)
        value = copy.deepcopy(self.value)
        value["formal_release"]["acceptance"]["path"] = "authorization_alias/authorization.json"
        with self.assertRaisesRegex(ValueError, "symlink"):
            verify_release(self.root, value)
        loop = self.root / "back_to_root"
        loop.symlink_to(self.root, target_is_directory=True)
        alias_path = loop / self.acceptance_path.relative_to(self.root)
        with self.assertRaisesRegex(ValueError, "symlink"):
            create_released_manifest(self.root, self.parent, alias_path)

    def test_duplicate_json_keys_and_nonfinite_values_rejected(self) -> None:
        for data in (b'{"schema_version":1,"schema_version":1}', b'{"schema_version":NaN}'):
            self.acceptance_path.write_bytes(data)
            value = copy.deepcopy(self.value)
            value["formal_release"]["acceptance"]["sha256"] = sha(self.acceptance_path)
            with self.subTest(data=data), self.assertRaises(ValueError):
                verify_release(self.root, value)

    def test_tampered_source_seal_environment_config_and_parking_fail_startup(self) -> None:
        paths = ["environments/mla_short/trajectories/route_0.csv",
                 "environments/mla_short/recordings/route_0__sealed/joint_trajectory.csv",
                 "environments/mla_short/environment.yaml", "collection/config.yaml",
                 self.parent_value["parking_prepare_bundle"]["gpu_validation_file"]]
        for name in paths:
            path = self.root / name
            before = path.read_bytes()
            path.write_bytes(before + b"tamper\n")
            try:
                with self.subTest(path=name), self.assertRaises(ValueError):
                    verify_release(self.root, self.value)
            finally:
                path.write_bytes(before)

    def test_added_unsealed_recording_file_rejected(self) -> None:
        path = self.root / "environments/mla_short/recordings/route_0__sealed/unlisted.csv"
        path.write_text("an unsealed payload\n")
        with self.assertRaisesRegex(ValueError, "protect every recording file"):
            verify_release(self.root, self.value)

    def test_metadata_only_check_is_explicitly_not_a_startup_seal_check(self) -> None:
        path = self.root / "environments/mla_short/recordings/route_0__sealed/joint_trajectory.csv"
        path.write_text("changed selected payload; dispatch gate must reject it\n")
        self.parking.reset_mock()
        result = verify_release(self.root, self.value, verify_recordings=False)
        self.assertFalse(result["recordings_verified"])
        self.parking.assert_not_called()
        with self.assertRaises(ValueError):
            verify_release(self.root, self.value)
        for item in (None, 0, 1, "false"):
            with self.subTest(item=item), self.assertRaises(ValueError):
                verify_release(self.root, self.value, verify_recordings=item)

    def test_parking_top_level_file_still_checked_in_metadata_only_mode(self) -> None:
        proof = self.parent_value["parking_prepare_bundle"]
        path = self.root / proof["independent_mock_evidence_file"]
        path.write_text("changed parking proof\n")
        with self.assertRaisesRegex(ValueError, "SHA256 changed"):
            verify_release(self.root, self.value, verify_recordings=False)

    def test_parent_and_acceptance_bytes_rechecked_before_return(self) -> None:
        from . import release
        original = release._acceptance
        def change_after_check(acceptance, parent, binding):
            original(acceptance, parent, binding)
            self.acceptance_path.write_bytes(self.acceptance_path.read_bytes() + b" ")
        with patch(f"{__package__}.release._acceptance", side_effect=change_after_check):
            with self.assertRaisesRegex(ValueError, "changed during verification"):
                verify_release(self.root, self.value)

    def test_constructors_reject_external_file_and_missing_current_authorization(self) -> None:
        for statement in ("", " ", None):
            with self.subTest(statement=statement), self.assertRaises(ValueError):
                create_raw_acceptance(self.root, self.parent, "test_installation", statement, "request")
        with self.assertRaises(ValueError):
            create_released_manifest(self.root, self.parent, self.root.parent / "outside.json")
        self.assertEqual(self.acceptance["kind"], ACCEPTANCE_KIND)

    def test_sensor_quality_policy_is_explicit_versioned_and_defaults_strict(self) -> None:
        self.assertEqual(verify_release(self.root, self.value)["sensor_quality_policy"], "strict_v1")
        acceptance = copy.deepcopy(self.acceptance)
        acceptance["sensor_quality_policy"] = "bounded_group_loss_10_per_3000_v1"
        value = self.save_acceptance(acceptance)
        self.assertEqual(verify_release(self.root, value)["sensor_quality_policy"],
                         "bounded_group_loss_10_per_3000_v1")
        for policy in (None, True, {"allowed": 10}, "unlimited", "bounded_group_loss_100_per_3000_v1"):
            acceptance["sensor_quality_policy"] = policy
            value = self.save_acceptance(acceptance)
            with self.subTest(policy=policy), self.assertRaisesRegex(ValueError, "sensor quality policy"):
                verify_release(self.root, value)


if __name__ == "__main__":
    unittest.main()
