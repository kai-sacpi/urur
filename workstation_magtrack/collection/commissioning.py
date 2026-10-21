"""One operator-confirmed real trial, without asserting a formal installation release.

This opt-in manifest is a separate scope, not a way to turn nominal delivery
metadata into calibration or TCP acceptance.  Creating/verifying it opens no
ROS connection, serial device, or robot controller.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import re
from typing import Any

from .manifest import ID, config_at, contained, encoded, sha, verify


PARENT_FIELDS = frozenset({
    "schema_version", "environment_id", "config", "config_sha256",
    "environment_hashes", "inventory_provenance", "real_execution_authorized",
    "recordings", "parking_prepare_bundle",
})
ROW_FIELDS = frozenset({
    "trajectory_id", "recording_id", "duration_s", "source_file", "source_sha256",
    "recording_directory", "sealed_hashes", "coherent_witness_provenance",
    "real_execution_authorized",
})
COMMISSIONING_FIELDS = frozenset({
    "schema_version", "scope", "request_id", "parent_manifest",
    "selected_trajectory_id", "operator_attestation", "labels",
    "calibration_tcp_acceptance", "formal_real_release",
})
SCOPE = "single_trial_commissioning"
DIGEST = re.compile(r"[0-9a-f]{64}\Z")


def _identifier(value: Any, name: str) -> str:
    if not isinstance(value, str) or not ID.fullmatch(value):
        raise ValueError(f"unsafe {name}")
    return value


def _nominal(value: Any) -> None:
    """Do not admit a hidden/granted formal release at any nesting level."""
    if isinstance(value, dict):
        for key, item in value.items():
            if key in {"accepted_installation_release", "real_release"}:
                raise ValueError("commissioning cannot carry a formal release")
            if key in {"real_execution_authorized", "formal_real_release",
                       "calibration_tcp_acceptance"} and item is not False:
                raise ValueError("commissioning must retain false formal authorization/acceptance")
            _nominal(item)
    elif isinstance(value, list):
        for item in value:
            _nominal(item)


def _parent_relative(root: Path, parent_path: Path) -> str:
    parent = parent_path.resolve() if parent_path.is_absolute() else contained(root, str(parent_path))
    if not parent.is_relative_to(root):
        raise ValueError("parent manifest escapes workspace")
    relative = parent.relative_to(root)
    if (len(relative.parts) != 3 or relative.parts[0] != "reports" or
            relative.name != "collection_manifest.json"):
        raise ValueError("parent must be a published reports/*/collection_manifest.json")
    return relative.as_posix()


def _read_parent(root: Path, relative: str, expected: str | None = None) -> tuple[dict[str, Any], str]:
    if not isinstance(relative, str) or _parent_relative(root, Path(relative)) != relative:
        raise ValueError("parent manifest path must be canonical and workspace-relative")
    raw = contained(root, relative).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if expected is not None and (not isinstance(expected, str) or not DIGEST.fullmatch(expected)
                                 or expected != digest):
        raise ValueError("parent manifest SHA256 changed")
    parent = json.loads(raw)
    if not isinstance(parent, dict) or set(parent) != PARENT_FIELDS:
        raise ValueError("parent is not the nominal published collection manifest")
    if parent.get("schema_version") != 1 or parent.get("environment_id") != "mla_short":
        raise ValueError("commissioning requires mla_short collection schema 1")
    _nominal(parent)
    if (parent.get("real_execution_authorized") is not False or
            parent.get("config", {}).get("execution", {}).get("simulation_only") is not True or
            parent["config"]["execution"].get("real_execution_authorized") is not False):
        raise ValueError("parent must retain nominal simulation-only authorization")
    config = root / "collection/config.yaml"
    if sha(config) != parent["config_sha256"] or config_at(config) != parent["config"]:
        raise ValueError("frozen collection config changed")
    rows = parent["recordings"]
    if (not isinstance(rows, list) or not rows or any(not isinstance(row, dict) for row in rows) or
            any(set(row) != ROW_FIELDS for row in rows)):
        raise ValueError("invalid parent recordings")
    ids = [_identifier(row.get("trajectory_id"), "trajectory ID") for row in rows]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate parent trajectory ID")
    parking = parent["parking_prepare_bundle"]
    if (not isinstance(parking, dict) or
            parking.get("asset_type") != "collision_checked_prepare_joint_bundle" or
            parking.get("tipdraw_recording") is not False or
            parking.get("independent_mock_status") != "passed" or
            parking.get("real_execution_authorized") is not False):
        raise ValueError("parent lacks accepted nominal parking prepare binding")
    return parent, digest


def _trial(parent: dict[str, Any], relative: str, digest: str, trajectory_id: str,
           operator_statement: str, request_id: str) -> dict[str, Any]:
    _identifier(trajectory_id, "trajectory ID")
    _identifier(request_id, "request ID")
    if (not isinstance(operator_statement, str) or not operator_statement.strip() or
            len(operator_statement) > 4096):
        raise ValueError("an explicit current operator statement is required")
    rows = [row for row in parent["recordings"] if row["trajectory_id"] == trajectory_id]
    if len(rows) != 1:
        raise ValueError("trial must select exactly one known parent trajectory")
    selected = rows[0]
    rid = _identifier(selected.get("recording_id"), "recording ID")
    if (selected["source_file"] != f"environments/mla_short/trajectories/{trajectory_id}.csv" or
            selected["recording_directory"] != f"environments/mla_short/recordings/{rid}" or
            selected.get("real_execution_authorized") is not False):
        raise ValueError("invalid nominal selected row identity")
    witnesses = selected["coherent_witness_provenance"]
    if (not isinstance(witnesses, dict) or not witnesses or
            not set(witnesses).issubset({"coherent_playback_witness", "coherent_controller_witness"}) or
            any(state != "passed" for state in witnesses.values())):
        raise ValueError("selected row lacks passed controller witness")
    result = copy.deepcopy(parent)
    result["recordings"] = [copy.deepcopy(selected)]
    result["commissioning"] = {
        "schema_version": 1, "scope": SCOPE, "request_id": request_id,
        "parent_manifest": {"path": relative, "sha256": digest},
        "selected_trajectory_id": trajectory_id,
        "operator_attestation": {
            "statement": operator_statement, "origin": "human_user_current_chat",
            "authorization": "one_real_acquisition_trial",
        },
        "labels": "nominal_robot_tip_not_magnet_ground_truth",
        "calibration_tcp_acceptance": False, "formal_real_release": False,
    }
    return result


def create_trial(root: Path, parent_path: Path, trajectory_id: str,
                 operator_statement: str, request_id: str) -> dict[str, Any]:
    """Construct a single-row trial; caller must save with immutable_json()."""
    root = root.resolve()
    relative = _parent_relative(root, parent_path)
    parent, digest = _read_parent(root, relative)
    value = _trial(parent, relative, digest, trajectory_id, operator_statement, request_id)
    verify(root, value)
    return value


def require_trial(root: Path, value: dict[str, Any]) -> None:
    """Check the one-trial scope and unchanged parent/assets before hardware use."""
    root = root.resolve()
    if not isinstance(value, dict) or set(value) != PARENT_FIELDS | {"commissioning"}:
        raise ValueError("invalid single-trial manifest fields")
    commissioning = value["commissioning"]
    if (not isinstance(commissioning, dict) or set(commissioning) != COMMISSIONING_FIELDS or
            commissioning.get("schema_version") != 1 or commissioning.get("scope") != SCOPE):
        raise ValueError("invalid single-trial commissioning scope")
    binding = commissioning["parent_manifest"]
    attestation = commissioning["operator_attestation"]
    if (not isinstance(binding, dict) or set(binding) != {"path", "sha256"} or
            not isinstance(attestation, dict) or
            set(attestation) != {"statement", "origin", "authorization"}):
        raise ValueError("invalid commissioning parent/attestation binding")
    _nominal(value)
    parent, digest = _read_parent(root, binding["path"], binding["sha256"])
    expected = _trial(parent, binding["path"], digest,
                      commissioning["selected_trajectory_id"], attestation["statement"],
                      commissioning["request_id"])
    if encoded(value) != encoded(expected):
        raise ValueError("commissioning manifest differs from exact parent selection and attestation scope")
    verify(root, value)
