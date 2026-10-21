"""Explicit one-trial real motion, without changing the formal release gate.

Only the constructor and trial revalidation are specialized.  Actual-state,
planner/hash, full-path, exact FJT, settle, stationary, and cancellation behavior
remain the unchanged ``MotionController`` implementation.  Importing this
module, or rejecting a trial, never creates a ROS transport.
"""
from __future__ import annotations

import copy
import math
import os
from pathlib import Path
from typing import Any

import yaml

from .commissioning import require_trial
from .manifest import encoded
from .motion import MotionBackend, MotionController, sha256


class CommissioningMotionController(MotionController):
    """Operator-authorized, single-record commissioning on real domain zero.

    This is deliberately not ``super(mode='simulation')`` followed by a mode
    change, nor a fabricated installation release.  The original config and
    parking bundle remain byte-identical and simulation acceptance remains
    nominal evidence, not a claim of calibrated magnet-center ground truth.
    """

    def __init__(self, workspace: Path | str, config_path: Path | str,
                 manifest: dict[str, Any], evidence_root: Path | str,
                 prepared_parking: Path | str, *, backend: MotionBackend | None = None,
                 domain_id: int | None = None):
        self.workspace = Path(workspace).expanduser().resolve()
        # The authority guard must precede environment/backend/ROS construction.
        require_trial(self.workspace, manifest)
        self.commissioning_manifest = copy.deepcopy(manifest)
        self._commissioning_bytes = encoded(self.commissioning_manifest)
        self.config_path = Path(config_path).expanduser().resolve()
        expected_config = (self.workspace / "collection/config.yaml").resolve()
        if (self.config_path != expected_config or
                sha256(self.config_path) != manifest["config_sha256"]):
            raise ValueError("commissioning requires the unchanged parent-bound collection config")
        self.config = yaml.safe_load(self.config_path.read_text())
        if self.config != manifest["config"] or self.config.get("environment_id") != "mla_short":
            raise ValueError("commissioning config differs from the exact parent-bound mla_short policy")
        self.mode = "real"
        self.domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0")) if domain_id is None else domain_id
        if isinstance(self.domain_id, bool) or not isinstance(self.domain_id, int) or self.domain_id != 0:
            raise ValueError("real commissioning requires ROS_DOMAIN_ID=0, never an isolated simulation domain")

        # Use the existing full GPU + independent native-mock binding verifier,
        # not a path/boolean substitute.  Its imports are hardware-free.
        from .run import prepared_parking_path
        parking_path = prepared_parking_path(self.workspace, self.config_path, self.commissioning_manifest)
        if (prepared_parking is None or
                Path(prepared_parking).expanduser().resolve() != parking_path):
            raise ValueError("commissioning parking must be the exact parent-bound prepared bundle")

        from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
        from hx_waypoint_curobo.robot_kinematics import Pose, RobotKinematics
        from .prepared_parking import PreparedParkingBundle

        self.environment = EnvironmentBuilder().load(self.workspace / "environments/mla_short")
        self.environment_hashes = dict(self.environment.hashes)
        self.lock_sha256 = sha256(self.workspace / "curobo.lock")
        self.config_sha256 = sha256(self.config_path)
        self.revision = str(yaml.safe_load((self.workspace / "curobo.lock").read_text())["commit"])
        parking = self.config["parking"]
        if (parking.get("operator_confirmed") is not True or
                tuple(parking["joint_names"]) != self.environment.joint_names or
                len(parking["joint_positions_deg"]) != 6 or
                not all(math.isfinite(float(x)) for x in parking["joint_positions_deg"])):
            raise ValueError("parking requires six operator-confirmed joints in exact environment order")
        self.park = tuple(math.radians(float(x)) for x in parking["joint_positions_deg"])
        self.home = self.environment.home_joint_positions
        if (abs(float(self.environment.tcp["rod"]["length_m"])-.190) > 1e-12 or
                abs(float(self.environment.tcp["rod"]["radius_m"])-.0025) > 1e-12 or
                abs(float(self.environment.planner["validation"]["minimum_collision_distance_m"])-.005) > 1e-12):
            raise ValueError("commissioning requires unchanged rod190/d5 and hard 5mm clearance")
        self.options = {"planning_timeout_s": 120., "settle_timeout_s": 10.,
                        "max_state_age_s": .5, "settle_velocity_rad_s": .01,
                        "endpoint_tolerance_rad": .001}
        self.options.update(self.config.get("motion", {}))
        if any(isinstance(v, bool) or not math.isfinite(float(v)) or float(v) <= 0
               for v in self.options.values()):
            raise ValueError("all motion limits must be finite positive numbers")
        self.backend = backend
        self._last_state_source_ns = 0
        self._last_state_sequence = 0
        self.evidence_root = Path(evidence_root).expanduser().resolve()
        self.last_move: dict[str, Any] | None = None
        tool = self.environment.setup_config.tool
        transform = tool.parent_to_tip
        self.tool_pose = Pose(transform.translation_m, transform.quaternion_xyzw)
        self.model = RobotKinematics.from_urdf_bytes(
            (self.environment.root / self.environment.asset_provenance["generated_urdf"]["path"]).read_bytes(),
            base_link=self.environment.base_frame, joint_names=self.environment.joint_names)
        self.prepared_parking = PreparedParkingBundle(self.workspace, parking_path, self.config_path)

    def _unchanged(self) -> None:
        if encoded(self.commissioning_manifest) != self._commissioning_bytes:
            raise ValueError("single-trial commissioning authority changed after construction")
        require_trial(self.workspace, self.commissioning_manifest)
        super()._unchanged()
