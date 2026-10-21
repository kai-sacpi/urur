"""Read-only, installation-bound authorization for raw robot/sensor collection.

This scope is deliberately NOT a calibration acceptance.  Historical nominal
manifests/configuration/recordings remain unchanged and false-authorized.  A
separate operator authorization binds their exact bytes for raw acquisition;
actual-joint-derived nominal rod-tip FK is never relabelled magnetic truth.

The caller supplies the current human's authorization, not text discovered in
an untrusted file.  File hashes provide integrity/provenance, not a signature
or evidence that a physical calibration has been performed.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import re
from typing import Any

from .commissioning import PARENT_FIELDS, _parent_relative, _read_parent
from .manifest import ID, encoded, environment_hashes, recording_hashes


SCOPE = "operator_authorized_raw_collection"
ACCEPTANCE_KIND = "operator_authorized_raw_collection_acceptance"
REFERENCE_TYPE = "actual_joint_derived_nominal_rod_tip"
BIAS_MODE = "deferred_shared_bias_from_low_height_data"
DIGEST = re.compile(r"[0-9a-f]{64}\Z")
RELEASE_FIELDS = frozenset({"schema_version", "scope", "release_id", "parent_manifest",
                            "acceptance", "formal_calibrated_release"})
ACCEPTANCE_FIELDS = frozenset({"schema_version", "kind", "installation_id", "operator_authorization",
                               "installation", "reference", "bias_policy", "sensor_quality_policy", "frozen_inputs"})
SENSOR_QUALITY_POLICIES = frozenset({"strict_v1", "bounded_group_loss_10_per_3000_v1"})
REFERENCE = {"type": REFERENCE_TYPE, "frame": "plane1",
             "feedback": "actual_joint_states_and_frozen_fk", "calibrated_magnet_center": False,
             "independent_absolute_validation": False, "active_tcp": "unknown_not_used_for_joint_fk"}
BIAS_POLICY = {"mode": BIAS_MODE, "applied": False, "raw_data_preserved": True}
INSTALLATION = {"environment_id": "mla_short", "robot_type": "ur5", "robot_ip": "192.168.1.99",
                "tool_id": "rod_190mm_d5mm", "rod_length_m": .190, "rod_diameter_m": .005,
                "unchanged_frozen_setup": True}


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _identifier(value: Any, label: str) -> str:
    _require(isinstance(value, str) and bool(ID.fullmatch(value)), f"unsafe {label}")
    return value


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, item in pairs:
        _require(key not in result, f"duplicate JSON field: {key}")
        result[key] = item
    return result


def _json(data: bytes) -> dict[str, Any]:
    def nonfinite(value: str) -> Any:
        raise ValueError(f"nonfinite JSON value: {value}")
    value = json.loads(data, object_pairs_hook=_strict_object, parse_constant=nonfinite)
    _require(isinstance(value, dict), "release evidence must be a JSON object")
    return value


def _file(root: Path, relative: Any) -> Path:
    _require(isinstance(relative, str) and bool(relative), "missing workspace-relative file path")
    name = Path(relative)
    _require(not name.is_absolute() and ".." not in name.parts and
             name.as_posix() == relative and relative != ".", "noncanonical or escaping release path")
    path = root
    for part in name.parts:
        path = path / part
        _require(not path.is_symlink(), f"symlink forbidden in release path: {relative}")
    _require(path.is_file() and path.resolve().is_relative_to(root), f"missing or escaping release file: {relative}")
    return path


class _Reader:
    """One verification's read snapshots, rechecked before returning permission."""
    def __init__(self, root: Path):
        self.root = root
        self.files: dict[str, str] = {}

    def read(self, relative: str, expected: str | None = None) -> bytes:
        if expected is not None:
            _require(isinstance(expected, str) and bool(DIGEST.fullmatch(expected)), "invalid release SHA256")
        data = _file(self.root, relative).read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        _require(expected is None or digest == expected, f"release file SHA256 changed: {relative}")
        _require(relative not in self.files or self.files[relative] == digest,
                 f"release file changed during verification: {relative}")
        self.files[relative] = digest
        return data

    def binding(self, value: Any) -> tuple[dict[str, Any], str]:
        _require(isinstance(value, dict) and set(value) == {"path", "sha256"}, "invalid release file binding")
        _require(isinstance(value["sha256"], str) and bool(DIGEST.fullmatch(value["sha256"])),
                 "invalid release SHA256")
        return _json(self.read(value["path"], value["sha256"])), value["sha256"]

    def finish(self) -> None:
        for relative, digest in self.files.items():
            _require(hashlib.sha256(_file(self.root, relative).read_bytes()).hexdigest() == digest,
                     f"release file changed during verification: {relative}")


def _relative(root: Path, path: Path | str) -> str:
    path = Path(path)
    if path.is_absolute():
        # macOS /var -> /private/var is outside the workspace.  Permit that
        # root alias without resolving away a forbidden *in-workspace* symlink.
        # Use the outermost root spelling: a nested symlink pointing back to
        # root must remain in the relative suffix and be rejected by _file().
        anchor = next((p for p in reversed(path.parents) if p.resolve() == root), None)
        _require(anchor is not None, "release evidence escapes workspace")
        name = path.relative_to(anchor).as_posix()
    else:
        name = path.as_posix()
    _file(root, name)
    return name


def _parking(reader: _Reader, parent: dict[str, Any], *, full: bool) -> None:
    proof = parent["parking_prepare_bundle"]
    required = {"asset_type", "tipdraw_recording", "independent_mock_status", "real_execution_authorized",
                "prepared_bundle", "prepared_bundle_manifest_sha256", "gpu_validation_file",
                "gpu_validation_sha256", "independent_mock_evidence_file",
                "independent_mock_evidence_sha256", "directions"}
    optional = {"magnetic_interference_status"}
    _require(isinstance(proof, dict) and required <= set(proof) <= required | optional,
             "invalid frozen parking binding fields")
    if "magnetic_interference_status" in proof:
        _require(proof["magnetic_interference_status"] == "operator_expected_low_interference_not_measured",
                 "parking magnetic interference must not be promoted to measured acceptance")
    bundle = proof["prepared_bundle"]
    _require(isinstance(bundle, str), "invalid frozen parking directory")
    reader.read(f"{bundle}/bundle.json", proof["prepared_bundle_manifest_sha256"])
    reader.read(proof["gpu_validation_file"], proof["gpu_validation_sha256"])
    reader.read(proof["independent_mock_evidence_file"], proof["independent_mock_evidence_sha256"])
    if not full:
        # The runner/consumer separately verifies the full parking payload
        # immediately before parking use.  This is NOT an initial acceptance.
        return
    _verify_parking_payload(reader, proof, bundle)


def _verify_parking_payload(reader: _Reader, proof: dict[str, Any], bundle: str) -> None:
    """Recompute existing GPU + independent mock proofs; no hardware access."""
    from .prepared_parking import PreparedParkingBundle
    from .mock_parking import verify_mock_evidence
    prepared = PreparedParkingBundle(reader.root, reader.root / bundle, reader.root / "collection/config.yaml")
    prepared.verify()
    _require(prepared.manifest.get("source_validation", {}).get("sha256") == proof["gpu_validation_sha256"]
             and prepared.manifest.get("directions") == proof["directions"],
             "parking bundle differs from original frozen provenance")
    evidence = verify_mock_evidence(reader.root,
        _file(reader.root, proof["independent_mock_evidence_file"]), reader.root / bundle)
    _require(evidence.get("status") == "passed", "parking independent playback evidence failed")


def _parent(reader: _Reader, binding: dict[str, Any], *, verify_recordings: bool) -> dict[str, Any]:
    # Strict JSON is checked before using the existing nominal-only reader.
    reader.binding(binding)
    parent, digest = _read_parent(reader.root, binding["path"], binding["sha256"])
    _require(digest == binding["sha256"], "parent manifest changed while loading")
    reader.read("collection/config.yaml", parent["config_sha256"])
    current = environment_hashes(reader.root)
    _require(current == parent["environment_hashes"], "frozen raw-collection environment changed")
    for relative, digest in current.items():
        reader.read(relative, digest)
    if verify_recordings:
        for row in parent["recordings"]:
            _identifier(row["recording_id"], "recording ID")
            _require(row["source_file"] == f"environments/mla_short/trajectories/{row['trajectory_id']}.csv"
                     and row["recording_directory"] == f"environments/mla_short/recordings/{row['recording_id']}",
                     "raw-collection recording identity/path mismatch")
            reader.read(row["source_file"], row["source_sha256"])
            _require(isinstance(row["sealed_hashes"], dict) and row["sealed_hashes"], "empty sealed recording binding")
            for relative, digest in row["sealed_hashes"].items():
                reader.read(relative, digest)
            actual = recording_hashes(reader.root, reader.root / row["recording_directory"], row["source_sha256"])
            _require(actual == row["sealed_hashes"], "sealed recording file set changed")
    _parking(reader, parent, full=verify_recordings)
    return parent


def _frozen_inputs(parent: dict[str, Any], binding: dict[str, Any]) -> dict[str, Any]:
    return {"parent_manifest": copy.deepcopy(binding),
            "config": {"path": "collection/config.yaml", "sha256": parent["config_sha256"]},
            "environment_hashes": copy.deepcopy(parent["environment_hashes"]),
            "recording_inventory_sha256": hashlib.sha256(encoded(parent["recordings"])).hexdigest(),
            "parking_binding_sha256": hashlib.sha256(encoded(parent["parking_prepare_bundle"])).hexdigest()}


def _acceptance(value: Any, parent: dict[str, Any], binding: dict[str, Any]) -> None:
    _require(isinstance(value, dict) and set(value) == ACCEPTANCE_FIELDS, "invalid raw authorization fields")
    _require(value["schema_version"] == 1 and type(value["schema_version"]) is int
             and value["kind"] == ACCEPTANCE_KIND, "not an operator-authorized raw collection acceptance")
    _identifier(value["installation_id"], "installation ID")
    _require(isinstance(value["sensor_quality_policy"], str)
             and value["sensor_quality_policy"] in SENSOR_QUALITY_POLICIES,
             "unrecognized versioned sensor quality policy")
    authorization = value["operator_authorization"]
    _require(isinstance(authorization, dict) and set(authorization) ==
             {"request_id", "origin", "statement", "authorization"}, "invalid operator authorization fields")
    _identifier(authorization["request_id"], "request ID")
    _require(authorization["origin"] == "human_user_current_chat"
             and authorization["authorization"] == "raw_robot_sensor_collection",
             "raw collection requires a current human authorization")
    statement = authorization["statement"]
    _require(isinstance(statement, str) and bool(statement.strip()) and len(statement) <= 4096,
             "an explicit current human authorization statement is required")
    # Canonical JSON equality distinguishes true/1 and false/0 in nested gates.
    for field, expected in (("installation", INSTALLATION), ("reference", REFERENCE),
                            ("bias_policy", BIAS_POLICY), ("frozen_inputs", _frozen_inputs(parent, binding))):
        _require(encoded(value[field]) == encoded(expected), f"raw collection {field} binding differs")


def create_raw_acceptance(root: Path, parent_path: Path, installation_id: str,
                          operator_statement: str, request_id: str, *,
                          quality_policy: str = "strict_v1") -> dict[str, Any]:
    """Return an operator RAW authorization; never write or claim metrology.

    Call only from trusted current-user authorization.  The caller saves the
    returned object using an exclusive/immutable write, then passes that file
    to create_released_manifest().  No ROS/serial/controller access occurs.
    """
    root = Path(root).resolve()
    reader = _Reader(root)
    relative = _parent_relative(root, Path(parent_path))
    reader.read(relative)
    binding = {"path": relative, "sha256": reader.files[relative]}
    parent = _parent(reader, binding, verify_recordings=True)
    value = {"schema_version": 1, "kind": ACCEPTANCE_KIND, "installation_id": installation_id,
             "operator_authorization": {"request_id": request_id, "origin": "human_user_current_chat",
                 "statement": operator_statement, "authorization": "raw_robot_sensor_collection"},
             "installation": copy.deepcopy(INSTALLATION), "reference": copy.deepcopy(REFERENCE),
             "bias_policy": copy.deepcopy(BIAS_POLICY), "sensor_quality_policy": quality_policy,
             "frozen_inputs": _frozen_inputs(parent, binding)}
    _acceptance(value, parent, binding)
    reader.finish()
    return value


def create_released_manifest(root: Path, parent_path: Path, acceptance_path: Path, *,
                             verify_recordings: bool = True) -> dict[str, Any]:
    """Return a new scoped manifest, preserving every historical nominal field."""
    root = Path(root).resolve()
    reader = _Reader(root)
    parent_relative = _parent_relative(root, Path(parent_path))
    reader.read(parent_relative)
    parent_binding = {"path": parent_relative, "sha256": reader.files[parent_relative]}
    acceptance_relative = _relative(root, acceptance_path)
    acceptance = _json(reader.read(acceptance_relative))
    _require(type(verify_recordings) is bool, "verify_recordings must be boolean")
    parent = _parent(reader, parent_binding, verify_recordings=verify_recordings)
    _acceptance(acceptance, parent, parent_binding)
    value = copy.deepcopy(parent)
    value["formal_release"] = {"schema_version": 1, "scope": SCOPE,
        "release_id": acceptance["operator_authorization"]["request_id"],
        "parent_manifest": parent_binding,
        "acceptance": {"path": acceptance_relative, "sha256": reader.files[acceptance_relative]},
        "formal_calibrated_release": False}
    reader.finish()
    return value


def verify_release(root: Path, value: dict[str, Any], *, verify_recordings: bool = True) -> dict[str, Any]:
    """Verify a raw scope and return its truthful reference/identity binding.

    The default full verification is mandatory at startup.  False performs
    only an unchanged-metadata check, NOT initial acceptance: callers MUST
    reverify the selected source+seal before dispatch, and the unchanged
    parking consumer must independently verify its full bundle before use.
    """
    root = Path(root).resolve()
    _require(type(verify_recordings) is bool, "verify_recordings must be boolean")
    _require(isinstance(value, dict) and set(value) == PARENT_FIELDS | {"formal_release"},
             "raw release requires exact nominal parent fields plus its separate scope")
    release = value["formal_release"]
    _require(isinstance(release, dict) and set(release) == RELEASE_FIELDS, "invalid raw release scope fields")
    _require(type(release["schema_version"]) is int and release["schema_version"] == 1
             and release["scope"] == SCOPE and release["formal_calibrated_release"] is False,
             "raw collection scope cannot claim a formal calibrated release")
    _identifier(release["release_id"], "release ID")
    reader = _Reader(root)
    parent = _parent(reader, release["parent_manifest"], verify_recordings=verify_recordings)
    expected = {**parent, "formal_release": copy.deepcopy(release)}
    _require(encoded(value) == encoded(expected), "raw manifest differs from exact frozen nominal parent")
    acceptance, digest = reader.binding(release["acceptance"])
    _acceptance(acceptance, parent, release["parent_manifest"])
    _require(release["release_id"] == acceptance["operator_authorization"]["request_id"], "raw request identity mismatch")
    reader.finish()
    return {"scope": SCOPE, "installation_id": acceptance["installation_id"],
            "release_id": release["release_id"], "formal_calibrated_release": False,
            "reference": copy.deepcopy(REFERENCE), "bias_policy": copy.deepcopy(BIAS_POLICY),
            "sensor_quality_policy": acceptance["sensor_quality_policy"],
            "parent_manifest_sha256": release["parent_manifest"]["sha256"], "acceptance_sha256": digest,
            "recordings_verified": verify_recordings,
            "operator_asserted_installation": copy.deepcopy(INSTALLATION)}
