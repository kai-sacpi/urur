from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest

from .commissioning import create_trial, require_trial
from .manifest import create, immutable_json, require_real_release, sha


class CommissioningTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "workstation_magtrack"
        (self.root / "collection").mkdir(parents=True)
        config = self.root / "collection/config.yaml"
        config.write_bytes(Path(__file__).with_name("config.yaml").read_bytes())
        (self.root / "curobo.lock").write_text("pinned test revision\n")
        env = self.root / "environments/mla_short"
        env.mkdir(parents=True)
        (env / "environment.yaml").write_text("nominal test environment\n")
        sources = env / "trajectories"
        sources.mkdir()
        rows = []
        for number in range(2):
            name = f"route_{number}"
            source = sources / f"{name}.csv"
            source.write_text(f"authority {number}\n")
            directory = env / "recordings" / f"{name}__sealed"
            directory.mkdir(parents=True)
            (directory / "source_tip_trajectory.csv").write_bytes(source.read_bytes())
            (directory / "joint_trajectory.csv").write_text(f"joints {number}\n")
            (directory / "manifest.yaml").write_text("schema_version: 2\n")
            names = ["source_tip_trajectory.csv", "joint_trajectory.csv", "manifest.yaml"]
            (directory / "sha256sums.txt").write_text("".join(
                f"{sha(directory / name)}  {name}\n" for name in names))
            rows.append({"trajectory_id": name, "recording_id": directory.name,
                         "source_sha256": sha(source), "duration_s": 60,
                         **{phase: "passed" for phase in ("compile", "build", "inspect", "play")},
                         "coherent_controller_witness": "passed",
                         "ready_for_simulation_replay_on_this_host": True,
                         "real_execution_authorized": False})
        inventory = self.root / "inventory.json"
        immutable_json(inventory, {"environment_id": "mla_short", "recordings": rows})
        self.parent = self.root / "reports/final_delivery/collection_manifest.json"
        self.parent_value = create(self.root, [inventory], config)
        self.parent_value["parking_prepare_bundle"] = {
            "asset_type": "collision_checked_prepare_joint_bundle", "tipdraw_recording": False,
            "independent_mock_status": "passed", "real_execution_authorized": False,
            "prepared_bundle": "collection/prepared_parking/test_bundle",
            "prepared_bundle_manifest_sha256": "1" * 64,
            "gpu_validation_file": "runs/parking_gpu/validation.json",
            "gpu_validation_sha256": "2" * 64,
            "independent_mock_evidence_file": "runs/parking_mock/evidence.json",
            "independent_mock_evidence_sha256": "3" * 64,
            "directions": {"home_to_park": {}, "park_to_home": {}},
        }
        immutable_json(self.parent, self.parent_value)

    def trial(self, **overrides) -> dict:
        kwargs = {"root": self.root, "parent_path": self.parent, "trajectory_id": "route_0",
                  "operator_statement": "已连接真机，现场确认无问题；开始一次试采集。",
                  "request_id": "operator_trial_20261017_001"}
        kwargs.update(overrides)
        return create_trial(**kwargs)

    def replace_parent(self, value: dict) -> None:
        self.parent.write_text(json.dumps(value))

    def test_single_nominal_row_and_frozen_provenance(self) -> None:
        original = {p: sha(p) for p in self.root.rglob("*") if p.is_file()}
        trial = self.trial()
        require_trial(self.root, trial)
        self.assertEqual(trial["recordings"], [self.parent_value["recordings"][0]])
        for key in ("config", "config_sha256", "environment_hashes", "inventory_provenance",
                    "parking_prepare_bundle"):
            self.assertEqual(trial[key], self.parent_value[key])
        self.assertFalse(trial["real_execution_authorized"])
        self.assertTrue(trial["config"]["execution"]["simulation_only"])
        self.assertFalse(trial["config"]["execution"]["real_execution_authorized"])
        self.assertFalse(trial["recordings"][0]["real_execution_authorized"])
        self.assertEqual(trial["commissioning"]["scope"], "single_trial_commissioning")
        self.assertFalse(trial["commissioning"]["calibration_tcp_acceptance"])
        self.assertFalse(trial["commissioning"]["formal_real_release"])
        self.assertEqual(original, {p: sha(p) for p in self.root.rglob("*") if p.is_file()})

    def test_saved_trial_is_immutable_and_verifies_from_disk(self) -> None:
        output = self.root / "runs/trial/commissioning.json"
        immutable_json(output, self.trial())
        require_trial(self.root, json.loads(output.read_text()))
        with self.assertRaises(FileExistsError):
            immutable_json(output, self.trial())

    def test_formal_release_gate_unchanged(self) -> None:
        with self.assertRaisesRegex(ValueError, "simulation_only"):
            require_real_release(self.trial())

    def test_relative_parent_accepted(self) -> None:
        require_trial(self.root, self.trial(parent_path=Path("reports/final_delivery/collection_manifest.json")))

    def test_unknown_source_and_unsafe_identifiers_rejected(self) -> None:
        for overrides in ({"trajectory_id": "unknown"}, {"trajectory_id": "../route_0"},
                          {"request_id": "../trial"}, {"request_id": ""}, {"trajectory_id": ["route_0"]}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.trial(**overrides)

    def test_missing_operator_statement_rejected(self) -> None:
        for statement in ("", " ", None, "a" * 4097):
            with self.subTest(statement=str(statement)[:10]), self.assertRaises(ValueError):
                self.trial(operator_statement=statement)

    def test_non_published_or_escaping_parent_rejected(self) -> None:
        for path in (self.root / "collection_manifest.json", self.root.parent / "collection_manifest.json"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.trial(parent_path=path)

    def test_parent_current_bytes_must_match(self) -> None:
        trial = self.trial()
        self.parent.write_text(self.parent.read_text() + " ")
        with self.assertRaisesRegex(ValueError, "parent manifest SHA256 changed"):
            require_trial(self.root, trial)

    def test_duplicate_parent_identity_rejected(self) -> None:
        parent = copy.deepcopy(self.parent_value)
        parent["recordings"].append(copy.deepcopy(parent["recordings"][0]))
        self.replace_parent(parent)
        with self.assertRaisesRegex(ValueError, "duplicate"):
            self.trial()

    def test_unknown_or_formal_parent_fields_rejected(self) -> None:
        for field in ("unexpected", "accepted_installation_release", "real_release"):
            parent = copy.deepcopy(self.parent_value)
            parent[field] = {"accepted": True}
            self.replace_parent(parent)
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.trial()

    def test_parent_formal_true_rejected(self) -> None:
        parent = copy.deepcopy(self.parent_value)
        parent["recordings"][1]["real_execution_authorized"] = True
        self.replace_parent(parent)
        with self.assertRaisesRegex(ValueError, "false formal"):
            self.trial()

    def test_unknown_selected_row_field_rejected(self) -> None:
        parent = copy.deepcopy(self.parent_value)
        parent["recordings"][0]["physical_calibration_accepted"] = True
        self.replace_parent(parent)
        with self.assertRaisesRegex(ValueError, "invalid parent recordings"):
            self.trial()

    def test_failed_witness_rejected(self) -> None:
        parent = copy.deepcopy(self.parent_value)
        parent["recordings"][0]["coherent_witness_provenance"]["coherent_controller_witness"] = "failed"
        self.replace_parent(parent)
        with self.assertRaisesRegex(ValueError, "controller witness"):
            self.trial()

    def test_changed_authority_environment_seal_or_config_rejected(self) -> None:
        paths = [self.root / "environments/mla_short/trajectories/route_0.csv",
                 self.root / "environments/mla_short/environment.yaml",
                 self.root / "environments/mla_short/recordings/route_0__sealed/joint_trajectory.csv",
                 self.root / "collection/config.yaml"]
        for path in paths:
            trial = self.trial()
            before = path.read_bytes()
            path.write_bytes(before + b"changed\n")
            try:
                with self.subTest(path=path), self.assertRaises(ValueError):
                    require_trial(self.root, trial)
            finally:
                path.write_bytes(before)

    def test_second_or_swapped_row_cannot_enter_trial(self) -> None:
        for rows in (self.parent_value["recordings"], [self.parent_value["recordings"][1]]):
            trial = self.trial()
            trial["recordings"] = copy.deepcopy(rows)
            with self.subTest(rows=len(rows)), self.assertRaisesRegex(ValueError, "exact parent selection"):
                require_trial(self.root, trial)

    def test_trial_fields_cannot_broaden_release_or_labels(self) -> None:
        patches = [({"real_execution_authorized": True}, None),
                   ({"accepted_installation_release": {"accepted": True}}, None),
                   ({"scope": "whole_campaign"}, "commissioning"),
                   ({"formal_real_release": True}, "commissioning"),
                   ({"calibration_tcp_acceptance": True}, "commissioning"),
                   ({"labels": "accepted_magnet_ground_truth"}, "commissioning"),
                   ({"unknown": True}, "commissioning")]
        for patch, nested in patches:
            trial = self.trial()
            target = trial if nested is None else trial[nested]
            target.update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                require_trial(self.root, trial)

    def test_attestation_scope_and_origin_fixed(self) -> None:
        for patch in ({"origin": "model_guess"}, {"authorization": "all_336_trajectories"},
                      {"extra": True}):
            trial = self.trial()
            trial["commissioning"]["operator_attestation"].update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                require_trial(self.root, trial)

    def test_parent_path_cannot_be_absolute_or_traversal(self) -> None:
        for name in (str(self.parent), "reports/final_delivery/../final_delivery/collection_manifest.json",
                     "../outside/collection_manifest.json"):
            trial = self.trial()
            trial["commissioning"]["parent_manifest"]["path"] = name
            with self.subTest(name=name), self.assertRaises(ValueError):
                require_trial(self.root, trial)

    def test_parking_binding_cannot_change(self) -> None:
        trial = self.trial()
        trial["parking_prepare_bundle"]["prepared_bundle_manifest_sha256"] = "f" * 64
        with self.assertRaisesRegex(ValueError, "exact parent selection"):
            require_trial(self.root, trial)


if __name__ == "__main__":
    unittest.main()
