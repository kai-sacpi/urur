"""Hash-bound raw-collection authority around the unchanged motion consumer.

The original nominal config, parking producer proof, and sealed motion consumer
remain unchanged.  A release is separate authority, not a relabeling of their
simulation acceptance or a claim of independently measured magnetic truth.
"""
from __future__ import annotations

import copy
import math
import os
from pathlib import Path
from typing import Any

import yaml

from .manifest import encoded
from .motion import MotionBackend, MotionController, sha256
from .release import verify_release
from .stationary_guard import RawStationaryMonitor


class ReleasedMotionController(MotionController):
    """Real-only raw acquisition; actual joint FK stays nominal and uncorrected.

    Motion uses joint FJT and joint-derived FK, not teach-pendant TCP inversion.
    Unknown active TCP is not invented or treated as calibrated evidence. Shared
    low-height bias estimation remains a separate, downstream analysis step.
    """

    def __init__(self, workspace: Path | str, config_path: Path | str,
                 manifest: dict[str, Any], evidence_root: Path | str,
                 prepared_parking: Path | str, *, backend: MotionBackend | None = None,
                 domain_id: int | None = None):
        self.workspace = Path(workspace).expanduser().resolve()
        # Reject absent/unaccepted authority before environment/backend construction.
        # Main performs the complete inventory verification; dispatch verifies
        # the selected source/seal again. Here recheck release/parent/config/
        # environment without hashing all 336 recordings at every prepare move.
        release = verify_release(self.workspace, manifest, verify_recordings=False)
        if (release.get("scope") != "operator_authorized_raw_collection" or
                release.get("formal_calibrated_release") is not False):
            raise ValueError("released motion requires explicit raw-only nominal-reference authority")
        self.released_manifest = copy.deepcopy(manifest)
        self._released_bytes = encoded(self.released_manifest)
        self.release = copy.deepcopy(release)
        self._release_bytes = encoded(self.release)
        self.config_path = Path(config_path).expanduser().resolve()
        if (self.config_path != (self.workspace / "collection/config.yaml").resolve() or
                sha256(self.config_path) != manifest["config_sha256"]):
            raise ValueError("released motion requires the unchanged parent-bound nominal config")
        self.config = yaml.safe_load(self.config_path.read_text())
        if self.config != manifest["config"] or self.config.get("environment_id") != "mla_short":
            raise ValueError("released motion config differs from the parent-bound mla_short policy")
        # No nominal permission flag is overwritten or treated as a real release.
        self.mode = "real"
        self.domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0")) if domain_id is None else domain_id
        if isinstance(self.domain_id, bool) or not isinstance(self.domain_id, int) or self.domain_id != 0:
            raise ValueError("released real motion requires ROS_DOMAIN_ID=0")
        from .run import prepared_parking_path
        parking_path = prepared_parking_path(self.workspace, self.config_path, self.released_manifest)
        if (prepared_parking is None or
                Path(prepared_parking).expanduser().resolve() != parking_path):
            raise ValueError("released parking must be the exact parent-bound prepared bundle")

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
            raise ValueError("released motion requires unchanged rod190/d5 and hard 5mm clearance")
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
        if (encoded(self.released_manifest) != self._released_bytes or
                encoded(self.release) != self._release_bytes):
            raise ValueError("installation release authority changed after construction")
        current = verify_release(self.workspace, self.released_manifest, verify_recordings=False)
        if encoded(current) != self._release_bytes:
            raise ValueError("verified installation release changed after construction")
        super()._unchanged()

    def verify_runtime_identity(self) -> dict[str, Any]:
        """Fail fast on an unavailable runtime, without retry or fallback.

        Identity is a bounded health request, not a collision-planning request;
        its timeout must not inherit the planner's 120-second budget.
        """
        self._unchanged()
        if self.mode != "real" or type(self.domain_id) is not int or self.domain_id != 0:
            raise ValueError("released real motion requires mode=real and ROS_DOMAIN_ID=0")
        timeout_s = min(10., float(self.options["planning_timeout_s"]))
        identity = dict(self._backend().runtime_identity(timeout_s))
        expected = {"execution_mode": "real",
                    "environments_root": str(self.workspace / "environments"),
                    "telemetry_root": str(self.workspace / "runs")}
        if any(identity.get(k) != v for k, v in expected.items()) or identity.get("use_sim_time") is not False:
            raise ValueError(f"runtime identity mismatch: {identity!r} != {expected!r}")
        return {**identity, "domain_id": self.domain_id, "verified": True}

    def stationary_monitor(self, output_path: Path | str,
                           duration_s: float = 30.) -> RawStationaryMonitor:
        return RawStationaryMonitor(self, Path(output_path), duration_s)

