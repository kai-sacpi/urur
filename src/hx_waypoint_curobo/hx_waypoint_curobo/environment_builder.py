from __future__ import annotations

import hashlib
from dataclasses import dataclass
from math import isfinite, tau
from pathlib import Path
from typing import Any, Mapping, Sequence
from xml.etree import ElementTree

import numpy as np
import yaml

from .errors import ConfigurationError
from .math3d import matrix_pose, pose_matrix
from .models import TimedPose
from .robot_kinematics import RobotKinematics
from .setup_config import RigidTransform, SetupConfig, parse_setup


ROOT_CONFIG_FILES = (
    "environment.yaml",
    "setup.yaml",
    "zones.yaml",
    "planner.yaml",
    "recording.yaml",
    "controllers.yaml",
)
ROBOT_CONFIG_FILES = (
    "robot/robot_model.yaml",
    "robot/calibrated_kinematics.yaml",
    "robot/asset_provenance.yaml",
    "robot/curobo_robot.yaml",
    "robot/collision_spheres.yaml",
)
ROBOT_DATA_FILES = ("robot/driver_kinematics.yaml",)
REPRODUCIBLE_TREE_DIRECTORIES = (
    "meshes",
    "robot/assets",
)

# Sensor-centered MLA scene variants share the same measured work frame and
# collision model. Their tool geometry is part of the variant contract.
MLA_SENSOR_CENTER_ENV_IDS = frozenset(("mla", "mla_short"))


def _read_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ConfigurationError(f"cannot load YAML {path}: {error}") from error
    if not isinstance(data, dict):
        raise ConfigurationError(f"{path} must contain a YAML mapping")
    if data.get("schema_version") != 1:
        raise ConfigurationError(f"{path}: schema_version must be 1")
    return data


def _require(
    mapping: Mapping[str, Any],
    dotted_path: str,
    expected_type: type | tuple[type, ...],
) -> Any:
    value: Any = mapping
    traversed: list[str] = []
    for key in dotted_path.split("."):
        traversed.append(key)
        if not isinstance(value, Mapping) or key not in value:
            raise ConfigurationError(f"missing configuration field {'.'.join(traversed)}")
        value = value[key]
    if not isinstance(value, expected_type):
        expected_name = (
            " or ".join(item.__name__ for item in expected_type)
            if isinstance(expected_type, tuple)
            else expected_type.__name__
        )
        raise ConfigurationError(
            f"configuration field {dotted_path} must be {expected_name}"
        )
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class JointPositionDomain:
    name: str
    lower: float
    upper: float
    period: float | None


@dataclass(frozen=True)
class EnvironmentBundle:
    root: Path
    environment: Mapping[str, Any]
    setup: Mapping[str, Any]
    setup_config: SetupConfig
    tcp: Mapping[str, Any]
    frames: Mapping[str, Any]
    zones: Mapping[str, Any]
    planner: Mapping[str, Any]
    recording: Mapping[str, Any]
    controllers: Mapping[str, Any]
    robot_model: Mapping[str, Any]
    calibrated_kinematics: Mapping[str, Any]
    asset_provenance: Mapping[str, Any]
    curobo_robot: Mapping[str, Any]
    collision_spheres: Mapping[str, Any]
    hashes: Mapping[str, str]

    @property
    def environment_id(self) -> str:
        return str(_require(self.environment, "environment_id", str))

    @property
    def robot_type(self) -> str:
        return str(_require(self.robot_model, "robot.type", str))

    @property
    def joint_names(self) -> tuple[str, ...]:
        names = _require(self.robot_model, "robot.joint_names", list)
        if (
            len(names) != 6
            or any(
                not isinstance(name, str) or not name or name.strip() != name
                for name in names
            )
            or len(set(names)) != len(names)
        ):
            raise ConfigurationError(
                "robot.joint_names must contain six unique non-empty strings"
            )
        return tuple(names)

    @property
    def base_frame(self) -> str:
        return str(_require(self.frames, "base_frame", str))

    @property
    def home_joint_positions(self) -> tuple[float, ...]:
        values = _require(self.robot_model, "robot.home_joint_positions", list)
        if len(values) != len(self.joint_names):
            raise ConfigurationError(
                "robot.home_joint_positions must match robot.joint_names"
            )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(float(value))
            for value in values
        ):
            raise ConfigurationError(
                "robot.home_joint_positions must contain finite numbers"
            )
        return tuple(float(value) for value in values)

    def canonical_joint_branch_mask(
        self,
        positions: Sequence[Sequence[float]],
    ) -> np.ndarray:
        values = np.asarray(positions, dtype=float)
        if values.ndim != 2 or values.shape[1] != len(self.joint_names):
            raise ConfigurationError(
                "canonical joint branch filtering requires an N x joint_count matrix"
            )
        configured = self.planner["ik"].get("canonical_joint_ranges_rad")
        if configured is None:
            return np.ones(len(values), dtype=bool)

        reference = np.asarray(self.home_joint_positions, dtype=float)
        domains = self.joint_position_domains
        joint_indices = {name: index for index, name in enumerate(self.joint_names)}
        mask = np.ones(len(values), dtype=bool)
        for name, bounds in configured.items():
            index = joint_indices[name]
            canonical = values[:, index]
            period = domains[index].period
            if period is not None:
                delta = np.remainder(
                    canonical - reference[index] + 0.5 * period,
                    period,
                ) - 0.5 * period
                canonical = reference[index] + delta
            lower, upper = (float(bound) for bound in bounds)
            mask &= canonical >= lower - 1.0e-9
            mask &= canonical <= upper + 1.0e-9
        return mask

    @property
    def joint_position_domains(self) -> tuple[JointPositionDomain, ...]:
        """Read controller joint domains from the hash-bound generated URDF."""

        relative = Path(
            _require(self.asset_provenance, "generated_urdf.path", str)
        )
        if relative.is_absolute() or ".." in relative.parts:
            raise ConfigurationError(
                "asset_provenance.generated_urdf.path must remain environment-relative"
            )
        root_path = self.root.resolve()
        path = (root_path / relative).resolve()
        try:
            path.relative_to(root_path)
        except ValueError as error:
            raise ConfigurationError(
                "asset_provenance.generated_urdf.path escapes the environment"
            ) from error
        try:
            root = ElementTree.parse(path).getroot()
        except (OSError, ElementTree.ParseError) as error:
            raise ConfigurationError(
                f"cannot parse generated robot URDF {path}: {error}"
            ) from error

        joint_elements = [
            element for element in root.findall("joint") if element.get("name")
        ]
        joint_element_names = [element.get("name") for element in joint_elements]
        duplicated = sorted(
            name
            for name in self.joint_names
            if joint_element_names.count(name) != 1
        )
        if duplicated:
            raise ConfigurationError(
                "generated robot URDF must define each configured joint exactly once: "
                + ", ".join(duplicated)
            )
        definitions = {element.get("name"): element for element in joint_elements}
        domains: list[JointPositionDomain] = []
        for name in self.joint_names:
            joint = definitions.get(name)
            if joint is None:
                raise ConfigurationError(
                    f"generated robot URDF is missing configured joint {name}"
                )
            joint_type = joint.get("type")
            if joint_type not in {"revolute", "prismatic"}:
                raise ConfigurationError(
                    f"configured joint {name} must have a bounded revolute or prismatic URDF type"
                )
            limit = joint.find("limit")
            if limit is None:
                raise ConfigurationError(
                    f"generated robot URDF joint {name} is missing position limits"
                )
            try:
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
            except (KeyError, TypeError, ValueError) as error:
                raise ConfigurationError(
                    f"generated robot URDF joint {name} has invalid position limits"
                ) from error
            if not isfinite(lower) or not isfinite(upper) or lower >= upper:
                raise ConfigurationError(
                    f"generated robot URDF joint {name} has invalid position limits"
                )
            domains.append(
                JointPositionDomain(
                    name=name,
                    lower=lower,
                    upper=upper,
                    period=tau if joint_type == "revolute" else None,
                )
            )
        return tuple(domains)

    def transform_pose_to_base(self, pose: TimedPose) -> TimedPose:
        source_to_base = self._frame_to_base(pose.frame_id)
        tip_in_source = pose_matrix(pose.position_m, pose.quaternion_xyzw)
        position, quaternion = matrix_pose(source_to_base @ tip_in_source)
        return TimedPose(
            time_s=pose.time_s,
            frame_id=self.base_frame,
            position_m=position,
            quaternion_xyzw=quaternion,
            phase=pose.phase,
        )

    def _frame_to_base(self, frame_id: str) -> np.ndarray:
        if frame_id == self.base_frame:
            return np.eye(4, dtype=float)
        definitions = _require(self.frames, "frames", dict)
        current = frame_id
        transform = np.eye(4, dtype=float)
        visited: set[str] = set()
        while current != self.base_frame:
            if current in visited:
                raise ConfigurationError(f"frame graph contains a cycle at {current}")
            visited.add(current)
            if current not in definitions or not isinstance(definitions[current], Mapping):
                raise ConfigurationError(f"frame {current!r} is not resolved by setup.yaml")
            definition = definitions[current]
            parent = _require(definition, "parent", str)
            translation = _require(definition, "translation_m", list)
            quaternion = _require(definition, "quaternion_xyzw", list)
            parent_to_current = pose_matrix(translation, quaternion)
            transform = parent_to_current @ transform
            current = parent
        return transform


class EnvironmentBuilder:
    """Load and fail-closed validate one self-contained deployment environment."""

    def load(self, root: str | Path) -> EnvironmentBundle:
        environment_root = Path(root).expanduser().resolve()
        if not environment_root.is_dir():
            raise ConfigurationError(f"environment directory does not exist: {environment_root}")
        config_paths = [
            environment_root / relative
            for relative in (*ROOT_CONFIG_FILES, *ROBOT_CONFIG_FILES)
        ]
        required_paths = [
            *config_paths,
            *(environment_root / relative for relative in ROBOT_DATA_FILES),
        ]
        missing = [
            str(path.relative_to(environment_root))
            for path in required_paths
            if not path.is_file()
        ]
        if missing:
            raise ConfigurationError(f"environment is missing required files: {', '.join(missing)}")

        loaded = {
            str(path.relative_to(environment_root)): _read_yaml(path)
            for path in config_paths
        }
        try:
            setup_config = parse_setup(loaded["setup.yaml"])
            tcp, frames = self._derive_setup_views(
                environment_root,
                setup_config,
                loaded["robot/robot_model.yaml"],
                loaded["robot/asset_provenance.yaml"],
            )
        except (OSError, TypeError, ValueError) as error:
            raise ConfigurationError(f"invalid setup.yaml: {error}") from error
        bundle = EnvironmentBundle(
            root=environment_root,
            environment=loaded["environment.yaml"],
            setup=loaded["setup.yaml"],
            setup_config=setup_config,
            tcp=tcp,
            frames=frames,
            zones=loaded["zones.yaml"],
            planner=loaded["planner.yaml"],
            recording=loaded["recording.yaml"],
            controllers=loaded["controllers.yaml"],
            robot_model=loaded["robot/robot_model.yaml"],
            calibrated_kinematics=loaded["robot/calibrated_kinematics.yaml"],
            asset_provenance=loaded["robot/asset_provenance.yaml"],
            curobo_robot=loaded["robot/curobo_robot.yaml"],
            collision_spheres=loaded["robot/collision_spheres.yaml"],
            hashes=self._hash_reproducible_inputs(environment_root),
        )
        self._validate(bundle)
        return bundle

    @staticmethod
    def _derive_setup_views(
        root: Path,
        setup: SetupConfig,
        robot_model: Mapping[str, Any],
        asset_provenance: Mapping[str, Any],
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        robot = _require(robot_model, "robot", dict)
        base_frame = _require(robot, "base_link", str)
        joint_names = _require(robot, "joint_names", list)
        home = _require(robot, "home_joint_positions", list)
        if len(joint_names) != len(home) or any(
            not isinstance(name, str) for name in joint_names
        ):
            raise ConfigurationError(
                "robot joint_names/home_joint_positions cannot derive setup frames"
            )

        urdf_relative = Path(_require(asset_provenance, "generated_urdf.path", str))
        if urdf_relative.is_absolute() or ".." in urdf_relative.parts:
            raise ConfigurationError(
                "asset_provenance.generated_urdf.path cannot derive setup frames safely"
            )
        urdf_path = (root / urdf_relative).resolve()
        try:
            urdf_path.relative_to(root.resolve())
        except ValueError as error:
            raise ConfigurationError("setup frame URDF escapes the environment") from error
        if urdf_path.is_symlink() or not urdf_path.is_file():
            raise ConfigurationError("setup frame URDF must be a regular non-symlink file")
        declared_urdf_hash = _require(asset_provenance, "generated_urdf.sha256", str)
        if _sha256(urdf_path) != declared_urdf_hash:
            raise ConfigurationError("generated URDF hash does not match asset provenance")

        kinematics = RobotKinematics.from_urdf_bytes(
            urdf_path.read_bytes(),
            base_link=base_frame,
            joint_names=joint_names,
        )
        parent_frame = setup.work_frame.parent_frame
        home_values = tuple(float(value) for value in home)
        base_to_parent_pose = kinematics.forward_kinematics(
            home_values,
            (parent_frame,),
        )[parent_frame]
        alternate_values = tuple(
            0.5 * (limit.lower_rad + limit.upper_rad)
            for limit in kinematics.position_limits
        )
        alternate_pose = kinematics.forward_kinematics(
            alternate_values,
            (parent_frame,),
        )[parent_frame]
        if not np.allclose(
            pose_matrix(base_to_parent_pose.position_m, base_to_parent_pose.quaternion_xyzw),
            pose_matrix(alternate_pose.position_m, alternate_pose.quaternion_xyzw),
            atol=1.0e-12,
            rtol=0.0,
        ):
            raise ConfigurationError(
                "setup work_frame.parent_frame must be fixed relative to robot base_link"
            )
        frames = setup.resolved_frames_mapping(
            base_frame=base_frame,
            base_to_parent=RigidTransform(
                translation_m=base_to_parent_pose.position_m,
                quaternion_xyzw=base_to_parent_pose.quaternion_xyzw,
            ),
        )
        return setup.tool.legacy_tcp_mapping(), frames

    def _validate(self, bundle: EnvironmentBundle) -> None:
        _ = bundle.environment_id
        _ = bundle.robot_type
        robot_joint_names = bundle.joint_names
        _ = bundle.base_frame
        self._validate_direct_feature_calibration(bundle)
        self._validate_asset_provenance(bundle)
        joint_domains = bundle.joint_position_domains
        home_positions = bundle.home_joint_positions
        for value, domain in zip(home_positions, joint_domains, strict=True):
            if (
                value < domain.lower
                or value > domain.upper
            ):
                raise ConfigurationError(
                    f"robot home position for {domain.name} is outside the generated URDF limits"
                )

        curobo_joint_names = _require(
            bundle.curobo_robot,
            "robot_cfg.kinematics.cspace.joint_names",
            list,
        )
        if (
            any(not isinstance(name, str) for name in curobo_joint_names)
            or tuple(curobo_joint_names) != robot_joint_names
        ):
            raise ConfigurationError(
                "cuRobo cspace joint_names must exactly match robot.joint_names order"
            )
        curobo_default_positions = _require(
            bundle.curobo_robot,
            "robot_cfg.kinematics.cspace.default_joint_position",
            list,
        )
        if (
            len(curobo_default_positions) != len(home_positions)
            or any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not isfinite(float(value))
                for value in curobo_default_positions
            )
            or not np.allclose(
                np.asarray(curobo_default_positions, dtype=float),
                np.asarray(home_positions, dtype=float),
                atol=1.0e-12,
                rtol=0.0,
            )
        ):
            raise ConfigurationError(
                "cuRobo default_joint_position must exactly match "
                "robot.home_joint_positions"
            )

        backend = _require(bundle.planner, "backend.type", str)
        if backend != "curobo_v2":
            raise ConfigurationError("planner backend.type must be exactly curobo_v2")
        revision_lock = _require(bundle.planner, "backend.revision_lock", str)
        if revision_lock != "curobo.lock":
            raise ConfigurationError("planner backend.revision_lock must be curobo.lock")

        preserve_timestamps = _require(
            bundle.planner,
            "timed_path.preserve_input_timestamps",
            bool,
        )
        if not preserve_timestamps:
            raise ConfigurationError(
                "planner timed_path.preserve_input_timestamps must be true"
            )

        use_previous_solution_seed = _require(
            bundle.planner,
            "ik.use_previous_solution_seed",
            bool,
        )
        if not use_previous_solution_seed:
            raise ConfigurationError(
                "planner ik.use_previous_solution_seed must be true"
            )
        candidates_per_pose = _require(
            bundle.planner,
            "ik.candidates_per_pose",
            int,
        )
        random_seed_reserve = _require(
            bundle.planner,
            "ik.random_seed_reserve",
            int,
        )
        if isinstance(candidates_per_pose, bool) or candidates_per_pose <= 0:
            raise ConfigurationError(
                "planner ik.candidates_per_pose must be a positive integer"
            )
        if (
            isinstance(random_seed_reserve, bool)
            or random_seed_reserve <= 0
            or random_seed_reserve >= candidates_per_pose
        ):
            raise ConfigurationError(
                "planner ik.random_seed_reserve must be a positive integer smaller "
                "than ik.candidates_per_pose"
            )
        ik_config = _require(bundle.planner, "ik", dict)
        canonical_joint_ranges = ik_config.get("canonical_joint_ranges_rad")
        if canonical_joint_ranges is not None:
            if not isinstance(canonical_joint_ranges, Mapping) or not canonical_joint_ranges:
                raise ConfigurationError(
                    "planner ik.canonical_joint_ranges_rad must be a non-empty mapping"
                )
            joint_indices = {
                name: index for index, name in enumerate(robot_joint_names)
            }
            for name, bounds in canonical_joint_ranges.items():
                if name not in joint_indices:
                    raise ConfigurationError(
                        "planner ik.canonical_joint_ranges_rad contains an unknown joint"
                    )
                if (
                    not isinstance(bounds, list)
                    or len(bounds) != 2
                    or any(
                        isinstance(value, bool)
                        or not isinstance(value, (int, float))
                        or not isfinite(float(value))
                        for value in bounds
                    )
                    or float(bounds[0]) >= float(bounds[1])
                ):
                    raise ConfigurationError(
                        "planner ik.canonical_joint_ranges_rad bounds must contain "
                        "two increasing finite numbers"
                    )
                reference = home_positions[joint_indices[name]]
                if reference < float(bounds[0]) or reference > float(bounds[1]):
                    raise ConfigurationError(
                        "robot.home_joint_positions must lie inside every configured "
                        "canonical IK joint range"
                    )

        internal_tolerance_guard_ratio = _require(
            bundle.planner,
            "validation.internal_tolerance_guard_ratio",
            (int, float),
        )
        if (
            isinstance(internal_tolerance_guard_ratio, bool)
            or not isfinite(float(internal_tolerance_guard_ratio))
            or not 0.0 < float(internal_tolerance_guard_ratio) < 1.0
        ):
            raise ConfigurationError(
                "planner validation.internal_tolerance_guard_ratio must be finite and "
                "strictly between 0 and 1"
            )

        on_violation = _require(bundle.planner, "limits.on_violation", str)
        allow_time_scale = _require(
            bundle.planner,
            "limits.allow_uniform_time_scale",
            bool,
        )
        maximum_time_scale = _require(
            bundle.planner,
            "limits.max_uniform_time_scale",
            (int, float),
        )
        if on_violation != "reject":
            raise ConfigurationError(
                "planner limits.on_violation must be reject; trajectory retiming is forbidden"
            )
        if allow_time_scale:
            raise ConfigurationError(
                "planner limits.allow_uniform_time_scale must be false; "
                "trajectory retiming is forbidden"
            )
        if (
            isinstance(maximum_time_scale, bool)
            or not isfinite(float(maximum_time_scale))
            or abs(float(maximum_time_scale) - 1.0) > 1.0e-12
        ):
            raise ConfigurationError(
                "planner limits.max_uniform_time_scale must be exactly 1.0"
            )

        simulation_type = _require(bundle.controllers, "simulation.controller_type", str)
        simulation_action = _require(bundle.controllers, "simulation.action_name", str)
        simulation_state_topic = _require(bundle.controllers, "simulation.state_topic", str)
        real_type = _require(bundle.controllers, "real.controller_type", str)
        real_action = _require(bundle.controllers, "real.action_name", str)
        expected_simulation_action = (
            "/joint_trajectory_controller/follow_joint_trajectory"
        )
        expected_simulation_state_topic = (
            "/joint_trajectory_controller/controller_state"
        )
        allowed_real_controllers = {
            "passthrough_trajectory_controller": (
                "/passthrough_trajectory_controller/follow_joint_trajectory"
            ),
            "joint_trajectory_controller": (
                "/joint_trajectory_controller/follow_joint_trajectory"
            ),
        }
        if (
            simulation_type != "joint_trajectory_controller"
            or simulation_action != expected_simulation_action
            or simulation_state_topic != expected_simulation_state_topic
        ):
            raise ConfigurationError(
                "simulation must use the exact joint_trajectory_controller action and state topic"
            )
        if allowed_real_controllers.get(real_type) != real_action:
            raise ConfigurationError(
                "real deployment must use an approved exact unscaled controller action"
            )
        if _require(bundle.controllers, "real.allow_fallback", bool):
            raise ConfigurationError("real controller fallback must remain disabled")

        _require(bundle.tcp, "tool.parent_link", str)
        _require(bundle.tcp, "tool.tip_frame", str)
        _require(bundle.tcp, "tool.translation_m", list)
        _require(bundle.tcp, "tool.quaternion_xyzw", list)
        _require(bundle.tcp, "rod.length_m", (int, float))
        _require(bundle.tcp, "rod.radius_m", (int, float))
        _require(bundle.zones, "contact_policy.allowed_contact_link", str)
        sample_dt_s = _require(bundle.recording, "recording.sample_dt_s", (int, float))
        if (
            isinstance(sample_dt_s, bool)
            or not isfinite(float(sample_dt_s))
            or float(sample_dt_s) <= 0.0
        ):
            raise ConfigurationError("recording.sample_dt_s must be finite and positive")
        if _require(bundle.recording, "recording.time_source", str) != "trajectory_time":
            raise ConfigurationError("recording.time_source must be trajectory_time")
        if (
            _require(bundle.recording, "recording.canonical_source", str)
            != "controller_action_goal"
        ):
            raise ConfigurationError(
                "recording.canonical_source must be controller_action_goal"
            )
        first_time = _require(bundle.recording, "recording.first_point.time_s", (int, float))
        if (
            isinstance(first_time, bool)
            or not isfinite(float(first_time))
            or float(first_time) != 0.0
        ):
            raise ConfigurationError("recording.first_point.time_s must be exactly zero")
        required_true_flags = (
            "recording.output_fields.positions",
            "recording.output_fields.velocities",
            "recording.output_fields.accelerations",
            "recording.first_point.require_zero_velocity",
            "recording.first_point.require_zero_acceleration",
            "recording.last_point.require_zero_velocity",
            "recording.last_point.require_zero_acceleration",
        )
        for field in required_true_flags:
            if not _require(bundle.recording, field, bool):
                raise ConfigurationError(f"{field} must be true")
        mismatch_policy = _require(
            bundle.recording,
            "start_state.mismatch_policy",
            str,
        )
        if mismatch_policy not in {"reject", "curobo_prepare"}:
            raise ConfigurationError(
                "start_state.mismatch_policy must be reject or curobo_prepare"
            )
        start_state = _require(bundle.recording, "start_state", dict)
        prepare_sequence = start_state.get(
            "prepare_sequence",
            "direct_to_recording_start",
        )
        if prepare_sequence not in {
            "direct_to_recording_start",
            "home_then_recording_start",
        }:
            raise ConfigurationError(
                "start_state.prepare_sequence must be direct_to_recording_start or "
                "home_then_recording_start"
            )
        if (
            prepare_sequence == "home_then_recording_start"
            and mismatch_policy != "curobo_prepare"
        ):
            raise ConfigurationError(
                "home_then_recording_start requires start_state.mismatch_policy=curobo_prepare"
            )

    @staticmethod
    def _validate_mla_sensor_center_frame(bundle: EnvironmentBundle) -> None:
        """Validate the sensor-centered ``new_board_mla`` environment.

        ``plane1`` is the verified sensor-board center: 53 mm along the local
        +Z axis of the raw UR Installation Feature ``new_board_mla``.  The
        orientation is unchanged.  The sensor column moves with that work
        frame, while the four ground slabs and every fixed boundary remain at
        their registered physical poses in ``base_link``.  The obsolete
        fixed-base support patch is intentionally absent.
        """

        if (
            bundle.environment_id not in MLA_SENSOR_CENTER_ENV_IDS
            or bundle.root.name != bundle.environment_id
        ):
            raise ConfigurationError(
                "MLA sensor-centered environment_id must be mla or mla_short and "
                "match its directory name"
            )

        expected_tool_length = (
            0.190 if bundle.environment_id == "mla_short" else 0.33365
        )
        if abs(bundle.setup_config.tool.length_m - expected_tool_length) > 1.0e-12:
            raise ConfigurationError(
                f"{bundle.environment_id} must use a {expected_tool_length:.5f} m TCP rod"
            )

        expected_raw_pose = (
            -0.08533,
            -0.48683,
            0.24284,
            1.16388,
            2.91348,
            0.00029,
        )
        expected_offset = (0.0, 0.0, 0.053)
        expected_plane_translation = (
            -0.08511768728206212,
            -0.4869042639951418,
            0.1898404772835752,
        )
        work_frame = bundle.setup_config.work_frame
        if work_frame.teach_pendant_pose_xyz_rotvec != expected_raw_pose:
            raise ConfigurationError(
                "mla setup.yaml must retain the measured new_board_mla Feature pose"
            )
        if work_frame.feature_to_work_frame_translation_m != expected_offset:
            raise ConfigurationError(
                "mla plane1 must remain exactly 53 mm along new_board_mla local +Z"
            )
        if work_frame.aliases:
            raise ConfigurationError(
                "mla must not expose the offset new_board_mla Feature as a plane1 alias"
            )
        if not np.allclose(
            np.asarray(work_frame.parent_to_child.translation_m, dtype=float),
            np.asarray(expected_plane_translation, dtype=float),
            atol=1.0e-12,
            rtol=0.0,
        ):
            raise ConfigurationError(
                "mla plane1 translation must resolve to the sensor-board center"
            )

        calibration = _require(bundle.environment, "calibration", dict)
        dashboard = _require(calibration, "dashboard", dict)
        expected_dashboard = {
            "robot_ip": "192.168.1.99",
            "variable": "new_board_mla",
            "source_type": "ur_installation_plane_feature",
            "pose_semantics": "base_to_feature",
            "raw_base_to_feature_pose_xyz_rotvec": list(expected_raw_pose),
        }
        if dashboard != expected_dashboard:
            raise ConfigurationError(
                "mla calibration.dashboard must remain the exact raw UR Feature authority"
            )

        binding = _require(calibration, "tf_binding", dict)
        expected_binding = {
            "policy": "feature_local_translation",
            "parent_frame": "base",
            "child_frame": "plane1",
            "source_feature": "new_board_mla",
            "translation_policy": "compose_feature_with_local_offset",
            "local_translation_frame": "feature",
            "local_translation_m": [0.0, 0.0, 0.053],
            "rotation_policy": "ur_rotation_vector_to_quaternion_xyzw",
            "orientation_correction": "none",
            "wall_used_as_pose_reference": False,
            "setup_pose_must_match_dashboard_pose": True,
            "setup_raw_pose_matches_dashboard_pose": True,
        }
        if binding != expected_binding:
            raise ConfigurationError(
                "mla calibration.tf_binding must place plane1 at the sensor center"
            )

        sensor = _require(calibration, "sensor_geometry", dict)
        keepout_max = 0.19096485659512854
        expected_sensor = {
            "sensor_center_local_z_m": 0.0,
            "actual_sensor_plane_local_z_m": 0.0,
            "protected_height_above_sensor_m": 0.060,
            "board_size_xy_m": [0.40, 0.40],
            "board_center_matches_feature_origin": False,
            "board_center_matches_work_frame_origin": True,
            "board_center_in_plane1_xy_m": [0.0, 0.0],
            "keepout_local_z_min_m": -0.060,
            "keepout_local_z_max_m": keepout_max,
            "tabletop_base_link_z_m": 0.0,
            "tabletop_reach_policy": "cover_full_board_footprint",
            "model_status": (
                "mla_new_board_mla_sensor_centered_plane1_to_tabletop"
            ),
        }
        if sensor != expected_sensor:
            raise ConfigurationError(
                "mla sensor geometry must remain the sensor-centered z=0 to tabletop contract"
            )

        base_from_plane = np.asarray(bundle._frame_to_base("plane1"), dtype=float)
        plane_rotation = base_from_plane[:3, :3]
        if plane_rotation[2, 2] >= 0.0:
            raise ConfigurationError("mla plane1 local +Z must point toward the tabletop")
        half_board_x = 0.5 * expected_sensor["board_size_xy_m"][0]
        half_board_y = 0.5 * expected_sensor["board_size_xy_m"][1]
        expected_tabletop_z = (
            base_from_plane[2, 3]
            + half_board_x * abs(plane_rotation[2, 0])
            + half_board_y * abs(plane_rotation[2, 1])
            - expected_sensor["tabletop_base_link_z_m"]
        ) / -plane_rotation[2, 2]
        if abs(keepout_max - expected_tabletop_z) > 1.0e-12:
            raise ConfigurationError(
                "mla sensor keepout +Z face must reach the base_link tabletop across "
                "the complete board footprint"
            )

        boundaries = _require(bundle.environment, "workspace_boundaries", dict)
        expected_boundaries = {
            "frame_id": "base_link",
            "representation": "fixed_base_link_collision_cuboids",
            "physical_pose_policy": "preserve_pre_recalibration_mla_scene",
            "remove_collision_object_ids": ["floor", "rear_keepout"],
            "ground": {
                "frame_id": "base_link",
                "size_xy_m": [3.0, 3.0],
                "top_z_m": 0.0,
                "thickness_m": 0.10,
                "ur5_nominal_reach_m": 0.85,
                "fixed_base_cutout_size_xy_m": [0.24, 0.24],
                "fixed_base_cutout_is_unfilled": True,
            },
            "fixed_collision_object_ids": [
                "x_positive_limit_wall",
                "y_positive_limit_wall",
                "top_negative_z_panel",
            ],
        }
        if boundaries != expected_boundaries:
            raise ConfigurationError(
                "mla workspace boundaries must remain fixed in base_link"
            )

        zones = bundle.zones
        expected_contact = {
            "allowed_contact_link": "rod_tip",
            "allowed_surface": "sensor_keepout_clearance_boundary",
            "maximum_tip_penetration_m": 0.0,
            "minimum_non_tip_clearance_m": 0.005,
        }
        if zones.get("contact_policy") != expected_contact:
            raise ConfigurationError("mla contact policy is not the fixed 5 mm contract")
        expected_cuboids = {
            "base_ground_x_negative": {
                "frame_id": "base_link",
                "dims": [1.38, 3.0, 0.10],
                "pose": [-0.81, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0],
            },
            "base_ground_x_positive": {
                "frame_id": "base_link",
                "dims": [1.38, 3.0, 0.10],
                "pose": [0.81, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0],
            },
            "base_ground_y_negative": {
                "frame_id": "base_link",
                "dims": [0.24, 1.38, 0.10],
                "pose": [0.0, -0.81, -0.05, 1.0, 0.0, 0.0, 0.0],
            },
            "base_ground_y_positive": {
                "frame_id": "base_link",
                "dims": [0.24, 1.38, 0.10],
                "pose": [0.0, 0.81, -0.05, 1.0, 0.0, 0.0, 0.0],
            },
            "sensor_board_keepout": {
                "frame_id": "plane1",
                "dims": [0.40, 0.40, 0.25096485659512857],
                "pose": [0.0, 0.0, 0.06548242829756427, 1.0, 0.0, 0.0, 0.0],
            },
            "x_positive_limit_wall": {
                "frame_id": "base_link",
                "dims": [0.01, 2.0, 2.0],
                "pose": [
                    0.5053236250594901,
                    0.05454297207665315,
                    0.179922086442941,
                    -0.00005100150018581122,
                    0.9293397736045651,
                    -0.36921898540765635,
                    -0.0022188758422972935,
                ],
            },
            "y_positive_limit_wall": {
                "frame_id": "base_link",
                "dims": [2.0, 0.01, 2.0],
                "pose": [
                    -0.4185333020801098,
                    -0.04305504688087691,
                    0.18352831324408936,
                    -0.00005100150018581122,
                    0.9293397736045651,
                    -0.36921898540765635,
                    -0.0022188758422972935,
                ],
            },
            "top_negative_z_panel": {
                "frame_id": "base_link",
                "dims": [0.165, 2.0, 0.01],
                "pose": [
                    0.9769621611413182,
                    1.1234196084095354,
                    0.9048545505968746,
                    -0.00005100150018581122,
                    0.9293397736045651,
                    -0.36921898540765635,
                    -0.0022188758422972935,
                ],
            },
        }
        curobo_scene = _require(zones, "curobo_scene", dict)
        if curobo_scene.get("cuboid") != expected_cuboids:
            raise ConfigurationError(
                "mla collision cuboids must keep only the sensor with plane1 and "
                "all other obstacles fixed in base_link"
            )
        expected_analysis = {
            "sensor_keepout_clearance_boundary": {
                "frame_id": "plane1",
                "normal_xyz": [0.0, 0.0, -1.0],
                "offset_m": 0.065,
                "x_range_m": [-0.20, 0.20],
                "y_range_m": [-0.20, 0.20],
            },
            "actual_sensor_plane": {
                "frame_id": "plane1",
                "normal_xyz": [0.0, 0.0, -1.0],
                "offset_m": 0.0,
            },
        }
        if zones.get("analysis_zones") != expected_analysis:
            raise ConfigurationError("mla analysis zones must match the new sensor geometry")

    @staticmethod
    def _validate_direct_feature_calibration(bundle: EnvironmentBundle) -> None:
        """Fail closed when an environment binds plane1 directly to a UR Feature."""

        direct_feature_contracts = {
            "new_board_4cm_rod33365": {
                "sensor_plane_local_z_m": 0.043,
                "keepout_local_z_min_m": -0.01,
                "keepout_local_z_max_m": 0.15,
                "clearance_boundary_offset_m": 0.015,
            },
            "new_board_4cm_rod33365_up50_icassp_phase2": {
                "sensor_plane_local_z_m": -0.007,
                "keepout_local_z_min_m": -0.067,
                "keepout_local_z_max_m": 0.10,
                "clearance_boundary_offset_m": 0.072,
                "original_sensor_plane_local_z_m": 0.043,
                "sensor_translation_local_z_m": -0.050,
                "protected_height_above_sensor_m": 0.060,
            },
        }
        requested_contract_ids = {
            identifier
            for identifier in (bundle.environment_id, bundle.root.name)
            if identifier in direct_feature_contracts
        }
        calibration_value = bundle.environment.get("calibration")
        binding_value = (
            calibration_value.get("tf_binding")
            if isinstance(calibration_value, Mapping)
            else None
        )
        declares_direct_feature_policy = (
            isinstance(binding_value, Mapping)
            and binding_value.get("policy") == "direct_ur_feature_pose"
        )
        declares_retired_mla_translation = (
            isinstance(binding_value, Mapping)
            and binding_value.get("policy") == "feature_local_translation"
        )
        if bundle.environment_id in MLA_SENSOR_CENTER_ENV_IDS:
            EnvironmentBuilder._validate_mla_sensor_center_frame(bundle)
            return
        if declares_retired_mla_translation:
            raise ConfigurationError(
                "feature_local_translation is reserved for the registered MLA "
                "sensor-center transform"
            )
        if not requested_contract_ids and not declares_direct_feature_policy:
            return
        if (
            bundle.environment_id not in direct_feature_contracts
            or len(requested_contract_ids) != 1
            or bundle.environment_id != bundle.root.name
        ):
            raise ConfigurationError(
                "direct-Feature environment_id must remain a registered authority and "
                "identical to its directory name"
            )
        target_environment_id = bundle.environment_id
        contract = direct_feature_contracts[target_environment_id]
        calibration = calibration_value
        if not isinstance(calibration, Mapping):
            raise ConfigurationError(
                f"{target_environment_id} requires calibration metadata"
            )
        binding = calibration.get("tf_binding")
        if not isinstance(binding, Mapping):
            raise ConfigurationError(
                f"{target_environment_id} requires calibration.tf_binding"
            )
        if binding.get("policy") != "direct_ur_feature_pose":
            raise ConfigurationError(
                "calibration.tf_binding.policy must be direct_ur_feature_pose"
            )
        expected_branch_ranges = {
            "shoulder_pan_joint": [-3.1415926536, 0.0],
        }
        if (
            bundle.planner.get("ik", {}).get("canonical_joint_ranges_rad")
            != expected_branch_ranges
        ):
            raise ConfigurationError(
                f"{target_environment_id} must lock the installed shoulder-pan IK branch"
            )
        if (
            bundle.recording.get("start_state", {}).get("prepare_sequence")
            != "home_then_recording_start"
        ):
            raise ConfigurationError(
                f"{target_environment_id} must prepare home_then_recording_start"
            )

        work_frame = bundle.setup_config.work_frame
        if _require(binding, "parent_frame", str) != work_frame.parent_frame:
            raise ConfigurationError(
                "calibration.tf_binding.parent_frame must match setup.yaml"
            )
        if _require(binding, "child_frame", str) != work_frame.child_frame:
            raise ConfigurationError(
                "calibration.tf_binding.child_frame must match setup.yaml"
            )
        required_binding_values = {
            "translation_policy": "unchanged",
            "rotation_policy": "ur_rotation_vector_to_quaternion_xyzw",
            "orientation_correction": "none",
        }
        for field, expected in required_binding_values.items():
            if _require(binding, field, str) != expected:
                raise ConfigurationError(
                    f"calibration.tf_binding.{field} must be {expected}"
                )
        if _require(binding, "wall_used_as_pose_reference", bool):
            raise ConfigurationError(
                "calibration.tf_binding.wall_used_as_pose_reference must be false"
            )
        if not _require(binding, "setup_pose_must_match_dashboard_pose", bool):
            raise ConfigurationError(
                "calibration.tf_binding.setup_pose_must_match_dashboard_pose must be true"
            )

        def finite_vector(value: Any, size: int, label: str) -> tuple[float, ...]:
            if not isinstance(value, list) or len(value) != size:
                raise ConfigurationError(f"{label} must contain exactly {size} numbers")
            result: list[float] = []
            for item in value:
                if (
                    isinstance(item, bool)
                    or not isinstance(item, (int, float))
                    or not isfinite(float(item))
                ):
                    raise ConfigurationError(f"{label} must contain finite numbers")
                result.append(float(item))
            return tuple(result)

        def exact_number(
            mapping: Mapping[str, Any],
            field: str,
            expected: float,
            label: str,
        ) -> float:
            value = _require(mapping, field, (int, float))
            if (
                isinstance(value, bool)
                or not isfinite(float(value))
                or abs(float(value) - expected) > 1.0e-12
            ):
                raise ConfigurationError(f"{label} must be exactly {expected}")
            return float(value)

        dashboard = _require(calibration, "dashboard", dict)
        if _require(dashboard, "variable", str) != "new_board_4cm":
            raise ConfigurationError(
                "calibration.dashboard.variable must remain new_board_4cm"
            )
        if (
            _require(dashboard, "source_type", str)
            != "ur_installation_plane_feature"
        ):
            raise ConfigurationError(
                "calibration.dashboard.source_type must be ur_installation_plane_feature"
            )
        if _require(dashboard, "pose_semantics", str) != "base_to_feature":
            raise ConfigurationError(
                "calibration.dashboard.pose_semantics must be base_to_feature"
            )
        raw_pose = finite_vector(
            dashboard.get("raw_base_to_feature_pose_xyz_rotvec"),
            6,
            "calibration.dashboard.raw_base_to_feature_pose_xyz_rotvec",
        )
        if raw_pose != work_frame.teach_pendant_pose_xyz_rotvec:
            raise ConfigurationError(
                "setup.yaml work-frame pose must exactly match the direct UR Feature pose"
            )
        if "new_board_4cm" not in work_frame.aliases:
            raise ConfigurationError(
                "setup.yaml work-frame aliases must retain new_board_4cm"
            )

        boundaries = _require(bundle.environment, "workspace_boundaries", dict)
        if _require(boundaries, "frame_id", str) != "new_board_4cm":
            raise ConfigurationError(
                "workspace_boundaries.frame_id must remain new_board_4cm"
            )
        if (
            _require(boundaries, "representation", str)
            != "finite_collision_slabs_on_prohibited_side"
        ):
            raise ConfigurationError(
                "workspace_boundaries must use finite collision slabs on the prohibited side"
            )
        exact_number(
            boundaries,
            "slab_thickness_m",
            0.01,
            "workspace_boundaries.slab_thickness_m",
        )
        exact_number(
            boundaries,
            "finite_span_m",
            2.0,
            "workspace_boundaries.finite_span_m",
        )
        remove_object_ids = boundaries.get("remove_collision_object_ids")
        if remove_object_ids != ["floor", "rear_keepout"]:
            raise ConfigurationError(
                "workspace_boundaries.remove_collision_object_ids must remove floor and "
                "rear_keepout"
            )
        ground = _require(boundaries, "ground", dict)
        if _require(ground, "frame_id", str) != "base_link":
            raise ConfigurationError(
                "workspace_boundaries.ground.frame_id must be base_link"
            )
        ground_size = finite_vector(
            ground.get("size_xy_m"),
            2,
            "workspace_boundaries.ground.size_xy_m",
        )
        if ground_size != (3.0, 3.0):
            raise ConfigurationError(
                "workspace_boundaries.ground.size_xy_m must be exactly [3.0, 3.0]"
            )
        exact_number(
            ground,
            "top_z_m",
            0.0,
            "workspace_boundaries.ground.top_z_m",
        )
        exact_number(
            ground,
            "thickness_m",
            0.10,
            "workspace_boundaries.ground.thickness_m",
        )
        nominal_robot_reach = exact_number(
            ground,
            "ur5_nominal_reach_m",
            0.85,
            "workspace_boundaries.ground.ur5_nominal_reach_m",
        )
        ground_cutout = finite_vector(
            ground.get("fixed_base_cutout_size_xy_m"),
            2,
            "workspace_boundaries.ground.fixed_base_cutout_size_xy_m",
        )
        if ground_cutout != (0.24, 0.24):
            raise ConfigurationError(
                "workspace_boundaries.ground.fixed_base_cutout_size_xy_m must be "
                "exactly [0.24, 0.24]"
            )
        exact_number(
            ground,
            "fixed_base_support_recess_m",
            0.0015,
            "workspace_boundaries.ground.fixed_base_support_recess_m",
        )
        exact_number(
            boundaries,
            "positive_x_limit_m",
            0.60,
            "workspace_boundaries.positive_x_limit_m",
        )
        exact_number(
            boundaries,
            "positive_y_limit_m",
            0.70,
            "workspace_boundaries.positive_y_limit_m",
        )
        top_panel = _require(boundaries, "top_panel", dict)
        exact_number(
            top_panel,
            "boundary_z_m",
            -0.72,
            "workspace_boundaries.top_panel.boundary_z_m",
        )
        top_x_range = finite_vector(
            top_panel.get("x_range_m"),
            2,
            "workspace_boundaries.top_panel.x_range_m",
        )
        if top_x_range != (0.129, 0.294):
            raise ConfigurationError(
                "workspace_boundaries.top_panel.x_range_m must be exactly [0.129, 0.294]"
            )
        exact_number(
            top_panel,
            "y_max_m",
            -0.10,
            "workspace_boundaries.top_panel.y_max_m",
        )
        exact_number(
            top_panel,
            "finite_y_min_m",
            -2.10,
            "workspace_boundaries.top_panel.finite_y_min_m",
        )

        sensor = _require(calibration, "sensor_geometry", dict)
        if not _require(sensor, "board_center_matches_feature_origin", bool):
            raise ConfigurationError(
                "calibration.sensor_geometry.board_center_matches_feature_origin must be true"
            )
        board_center = finite_vector(
            sensor.get("board_center_in_plane1_xy_m"),
            2,
            "calibration.sensor_geometry.board_center_in_plane1_xy_m",
        )
        if board_center != (0.0, 0.0):
            raise ConfigurationError(
                "confirmed Feature-origin board center must be exactly [0.0, 0.0]"
            )
        board_size = finite_vector(
            sensor.get("board_size_xy_m"),
            2,
            "calibration.sensor_geometry.board_size_xy_m",
        )
        if any(value <= 0.0 for value in board_size):
            raise ConfigurationError(
                "calibration.sensor_geometry.board_size_xy_m must be positive"
            )
        if board_size != (0.40, 0.40):
            raise ConfigurationError(
                f"{target_environment_id} board_size_xy_m must be exactly [0.40, 0.40]"
            )
        sensor_local_z = _require(
            sensor,
            "actual_sensor_plane_local_z_m",
            (int, float),
        )
        if (
            isinstance(sensor_local_z, bool)
            or not isfinite(float(sensor_local_z))
        ):
            raise ConfigurationError(
                "calibration.sensor_geometry.actual_sensor_plane_local_z_m must be finite"
            )
        expected_sensor_local_z = float(contract["sensor_plane_local_z_m"])
        if abs(float(sensor_local_z) - expected_sensor_local_z) > 1.0e-12:
            raise ConfigurationError(
                f"{target_environment_id} actual sensor plane must be exactly local "
                f"z={expected_sensor_local_z:+.3f} m"
            )
        if target_environment_id.endswith("_up50_icassp_phase2"):
            original_sensor_z = exact_number(
                sensor,
                "original_sensor_plane_local_z_m",
                float(contract["original_sensor_plane_local_z_m"]),
                "calibration.sensor_geometry.original_sensor_plane_local_z_m",
            )
            sensor_translation_z = exact_number(
                sensor,
                "sensor_translation_local_z_m",
                float(contract["sensor_translation_local_z_m"]),
                "calibration.sensor_geometry.sensor_translation_local_z_m",
            )
            protected_height = exact_number(
                sensor,
                "protected_height_above_sensor_m",
                float(contract["protected_height_above_sensor_m"]),
                "calibration.sensor_geometry.protected_height_above_sensor_m",
            )
            if abs(original_sensor_z + sensor_translation_z - float(sensor_local_z)) > 1.0e-12:
                raise ConfigurationError(
                    "up50 sensor plane must equal original plane plus the recorded translation"
                )
            if abs(float(sensor_local_z) - protected_height - float(
                contract["keepout_local_z_min_m"]
            )) > 1.0e-12:
                raise ConfigurationError(
                    "up50 keepout must extend exactly 60 mm above the current sensor plane"
                )
        keepout_min = _require(sensor, "keepout_local_z_min_m", (int, float))
        keepout_max = _require(sensor, "keepout_local_z_max_m", (int, float))
        if (
            isinstance(keepout_min, bool)
            or isinstance(keepout_max, bool)
            or not isfinite(float(keepout_min))
            or not isfinite(float(keepout_max))
            or float(keepout_min) >= float(keepout_max)
        ):
            raise ConfigurationError(
                "calibration.sensor_geometry keepout Z bounds must be finite and ordered"
            )
        expected_keepout_min = float(contract["keepout_local_z_min_m"])
        expected_keepout_max = float(contract["keepout_local_z_max_m"])
        if (
            abs(float(keepout_min) - expected_keepout_min) > 1.0e-12
            or abs(float(keepout_max) - expected_keepout_max) > 1.0e-12
        ):
            raise ConfigurationError(
                f"{target_environment_id} keepout Z bounds must be exactly "
                f"[{expected_keepout_min:+.3f}, {expected_keepout_max:+.3f}] m"
            )

        actual_surface = _require(
            bundle.zones,
            "analysis_zones.actual_sensor_plane",
            dict,
        )
        if _require(actual_surface, "frame_id", str) != work_frame.child_frame:
            raise ConfigurationError(
                "analysis_zones.actual_sensor_plane must use the direct Feature frame"
            )
        actual_normal = finite_vector(
            actual_surface.get("normal_xyz"),
            3,
            "analysis_zones.actual_sensor_plane.normal_xyz",
        )
        if actual_normal != (0.0, 0.0, -1.0):
            raise ConfigurationError(
                "analysis_zones.actual_sensor_plane.normal_xyz must be [0.0, 0.0, -1.0]"
            )
        actual_offset = _require(actual_surface, "offset_m", (int, float))
        if (
            isinstance(actual_offset, bool)
            or not isfinite(float(actual_offset))
            or abs(float(actual_offset) + float(sensor_local_z)) > 1.0e-12
        ):
            sensor_direction = "local +Z" if float(sensor_local_z) >= 0.0 else "local -Z"
            raise ConfigurationError(
                "analysis_zones.actual_sensor_plane must resolve to the configured "
                f"{sensor_direction}"
            )

        clearance_boundary = _require(
            bundle.zones,
            "analysis_zones.sensor_keepout_clearance_boundary",
            dict,
        )
        if _require(clearance_boundary, "frame_id", str) != work_frame.child_frame:
            raise ConfigurationError(
                "sensor_keepout_clearance_boundary must use the direct Feature frame"
            )
        clearance_normal = finite_vector(
            clearance_boundary.get("normal_xyz"),
            3,
            "analysis_zones.sensor_keepout_clearance_boundary.normal_xyz",
        )
        if clearance_normal != (0.0, 0.0, -1.0):
            raise ConfigurationError(
                "sensor_keepout_clearance_boundary.normal_xyz must be [0.0, 0.0, -1.0]"
            )
        expected_clearance_offset = float(contract["clearance_boundary_offset_m"])
        clearance_offset = _require(clearance_boundary, "offset_m", (int, float))
        if (
            isinstance(clearance_offset, bool)
            or not isfinite(float(clearance_offset))
            or abs(float(clearance_offset) - expected_clearance_offset) > 1.0e-12
        ):
            raise ConfigurationError(
                "sensor_keepout_clearance_boundary must remain exactly 5 mm outside "
                "the protected negative-Z face"
            )
        clearance_x_range = finite_vector(
            clearance_boundary.get("x_range_m"),
            2,
            "analysis_zones.sensor_keepout_clearance_boundary.x_range_m",
        )
        clearance_y_range = finite_vector(
            clearance_boundary.get("y_range_m"),
            2,
            "analysis_zones.sensor_keepout_clearance_boundary.y_range_m",
        )
        if clearance_x_range != (-0.20, 0.20) or clearance_y_range != (-0.20, 0.20):
            raise ConfigurationError(
                "sensor_keepout_clearance_boundary must cover exactly the 0.40 m square board"
            )
        if abs(
            expected_keepout_min + expected_clearance_offset - 0.005
        ) > 1.0e-12:
            raise ConfigurationError(
                "sensor keepout clearance boundary must remain exactly 5 mm outside "
                "the protected negative-Z face"
            )

        contact_policy = _require(bundle.zones, "contact_policy", dict)
        if (
            _require(contact_policy, "allowed_contact_link", str) != "rod_tip"
            or _require(contact_policy, "allowed_surface", str)
            != "sensor_keepout_clearance_boundary"
        ):
            raise ConfigurationError(
                "direct-Feature contact policy must bind rod_tip to the sensor boundary"
            )
        exact_number(
            contact_policy,
            "maximum_tip_penetration_m",
            0.0,
            "contact_policy.maximum_tip_penetration_m",
        )
        exact_number(
            contact_policy,
            "minimum_non_tip_clearance_m",
            0.005,
            "contact_policy.minimum_non_tip_clearance_m",
        )

        cuboids = _require(bundle.zones, "curobo_scene.cuboid", dict)
        expected_cuboid_names = {
            "base_ground_fixed_base_support",
            "base_ground_x_negative",
            "base_ground_x_positive",
            "base_ground_y_negative",
            "base_ground_y_positive",
            "sensor_board_keepout",
            "top_negative_z_panel",
            "x_positive_limit_wall",
            "y_positive_limit_wall",
        }
        if set(cuboids) != expected_cuboid_names:
            raise ConfigurationError(
                f"{target_environment_id} collision scene must contain exactly the five "
                "base-ground slabs, sensor keepout, and three workspace boundary slabs"
            )
        board_keepout = _require(
            cuboids,
            "sensor_board_keepout",
            dict,
        )
        if _require(board_keepout, "frame_id", str) != work_frame.child_frame:
            raise ConfigurationError(
                "sensor_board_keepout must use the direct Feature frame"
            )
        keepout_dims = finite_vector(
            board_keepout.get("dims"),
            3,
            "curobo_scene.cuboid.sensor_board_keepout.dims",
        )
        expected_dims = (
            board_size[0],
            board_size[1],
            float(keepout_max) - float(keepout_min),
        )
        if any(
            abs(actual - expected) > 1.0e-12
            for actual, expected in zip(keepout_dims, expected_dims, strict=True)
        ):
            raise ConfigurationError(
                "sensor_board_keepout dimensions must match sensor_geometry"
            )
        keepout_pose = finite_vector(
            board_keepout.get("pose"),
            7,
            "curobo_scene.cuboid.sensor_board_keepout.pose",
        )
        expected_pose = (
            0.0,
            0.0,
            0.5 * (float(keepout_min) + float(keepout_max)),
            1.0,
            0.0,
            0.0,
            0.0,
        )
        if any(
            abs(actual - expected) > 1.0e-12
            for actual, expected in zip(keepout_pose, expected_pose, strict=True)
        ):
            raise ConfigurationError(
                "sensor_board_keepout pose must be centered on the confirmed Feature origin"
            )

        expected_boundary_cuboids = {
            "base_ground_fixed_base_support": (
                "base_link",
                (0.24, 0.24, 0.0985),
                (0.0, 0.0, -0.05075, 1.0, 0.0, 0.0, 0.0),
            ),
            "base_ground_x_negative": (
                "base_link",
                (1.38, 3.0, 0.10),
                (-0.81, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0),
            ),
            "base_ground_x_positive": (
                "base_link",
                (1.38, 3.0, 0.10),
                (0.81, 0.0, -0.05, 1.0, 0.0, 0.0, 0.0),
            ),
            "base_ground_y_negative": (
                "base_link",
                (0.24, 1.38, 0.10),
                (0.0, -0.81, -0.05, 1.0, 0.0, 0.0, 0.0),
            ),
            "base_ground_y_positive": (
                "base_link",
                (0.24, 1.38, 0.10),
                (0.0, 0.81, -0.05, 1.0, 0.0, 0.0, 0.0),
            ),
            "x_positive_limit_wall": (
                "new_board_4cm",
                (0.01, 2.0, 2.0),
                (0.605, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            ),
            "y_positive_limit_wall": (
                "new_board_4cm",
                (2.0, 0.01, 2.0),
                (0.0, 0.705, 0.0, 1.0, 0.0, 0.0, 0.0),
            ),
            "top_negative_z_panel": (
                "new_board_4cm",
                (0.165, 2.0, 0.01),
                (0.2115, -1.10, -0.725, 1.0, 0.0, 0.0, 0.0),
            ),
        }
        for name, (
            expected_boundary_frame,
            expected_boundary_dims,
            expected_boundary_pose,
        ) in expected_boundary_cuboids.items():
            boundary = _require(cuboids, name, dict)
            if _require(boundary, "frame_id", str) != expected_boundary_frame:
                raise ConfigurationError(
                    f"curobo_scene.cuboid.{name}.frame_id must be "
                    f"{expected_boundary_frame}"
                )
            boundary_dims = finite_vector(
                boundary.get("dims"),
                3,
                f"curobo_scene.cuboid.{name}.dims",
            )
            boundary_pose = finite_vector(
                boundary.get("pose"),
                7,
                f"curobo_scene.cuboid.{name}.pose",
            )
            if any(
                abs(actual - expected) > 1.0e-12
                for actual, expected in zip(
                    boundary_dims,
                    expected_boundary_dims,
                    strict=True,
                )
            ):
                raise ConfigurationError(
                    f"curobo_scene.cuboid.{name}.dims do not match the workspace boundary"
                )
            if any(
                abs(actual - expected) > 1.0e-12
                for actual, expected in zip(
                    boundary_pose,
                    expected_boundary_pose,
                    strict=True,
                )
            ):
                raise ConfigurationError(
                    f"curobo_scene.cuboid.{name}.pose does not match the workspace boundary"
                )

        minimum_scene_clearance = _require(
            bundle.planner,
            "validation.minimum_collision_distance_m",
            (int, float),
        )
        if (
            isinstance(minimum_scene_clearance, bool)
            or not isfinite(float(minimum_scene_clearance))
            or float(minimum_scene_clearance) <= 0.0
        ):
            raise ConfigurationError(
                "validation.minimum_collision_distance_m must be finite and positive"
            )
        minimum_scene_clearance = float(minimum_scene_clearance)
        support_recess = float(ground["fixed_base_support_recess_m"])
        if support_recess >= minimum_scene_clearance:
            raise ConfigurationError(
                "fixed-base support recess must remain smaller than the scene clearance"
            )

        sphere_map = _require(bundle.collision_spheres, "collision_spheres", dict)
        configured_sphere_radii: list[float] = []
        for link_name, raw_spheres in sphere_map.items():
            if not isinstance(link_name, str) or not isinstance(raw_spheres, list):
                raise ConfigurationError(
                    "collision_spheres must map link names to sphere lists"
                )
            for raw_sphere in raw_spheres:
                if not isinstance(raw_sphere, Mapping):
                    raise ConfigurationError(
                        f"collision_spheres.{link_name} entries must be mappings"
                    )
                raw_radius = raw_sphere.get("radius")
                if (
                    isinstance(raw_radius, bool)
                    or not isinstance(raw_radius, (int, float))
                    or not isfinite(float(raw_radius))
                    or float(raw_radius) <= 0.0
                ):
                    raise ConfigurationError(
                        f"collision_spheres.{link_name} radius must be finite and positive"
                    )
                configured_sphere_radii.append(float(raw_radius))
        if not configured_sphere_radii:
            raise ConfigurationError("collision_spheres must contain at least one sphere")
        required_ground_half_span = (
            nominal_robot_reach
            + bundle.setup_config.tool.length_m
            + max(configured_sphere_radii)
            + minimum_scene_clearance
        )
        if 0.5 * min(ground_size) < required_ground_half_span:
            raise ConfigurationError(
                "base ground does not cover the conservative robot/tool reach envelope: "
                f"half-span {0.5 * min(ground_size):.6g} < "
                f"required {required_ground_half_span:.6g} m"
            )
        shoulder_spheres = _require(sphere_map, "shoulder_link", list)
        if len(shoulder_spheres) != 1 or not isinstance(shoulder_spheres[0], Mapping):
            raise ConfigurationError(
                f"{target_environment_id} requires exactly one shoulder_link sphere"
            )
        shoulder_center = finite_vector(
            shoulder_spheres[0].get("center"),
            3,
            "collision_spheres.shoulder_link[0].center",
        )
        shoulder_radius = _require(shoulder_spheres[0], "radius", (int, float))
        if (
            isinstance(shoulder_radius, bool)
            or not isfinite(float(shoulder_radius))
            or float(shoulder_radius) <= 0.0
        ):
            raise ConfigurationError(
                "collision_spheres.shoulder_link[0].radius must be finite and positive"
            )

        urdf_path = bundle.root / "robot" / "assets" / "ur5.urdf"
        try:
            urdf_root = ElementTree.parse(urdf_path).getroot()
        except (OSError, ElementTree.ParseError) as error:
            raise ConfigurationError(
                f"cannot parse generated URDF for ground-support validation: {urdf_path}"
            ) from error
        shoulder_joint = urdf_root.find("./joint[@name='shoulder_pan_joint']")
        shoulder_origin = (
            shoulder_joint.find("origin") if shoulder_joint is not None else None
        )
        if shoulder_joint is None or shoulder_origin is None:
            raise ConfigurationError(
                "generated URDF must define shoulder_pan_joint with an origin"
            )
        try:
            joint_xyz = tuple(
                float(value) for value in shoulder_origin.attrib["xyz"].split()
            )
            joint_rpy = tuple(
                float(value)
                for value in shoulder_origin.attrib.get("rpy", "0 0 0").split()
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ConfigurationError(
                "generated URDF shoulder_pan_joint origin must contain finite xyz/rpy"
            ) from error
        if (
            len(joint_xyz) != 3
            or len(joint_rpy) != 3
            or not all(isfinite(value) for value in (*joint_xyz, *joint_rpy))
            or any(
                abs(value) > 1.0e-12
                for value in (*joint_xyz[:2], *joint_rpy[:2])
            )
            or any(abs(value) > 1.0e-12 for value in shoulder_center)
        ):
            raise ConfigurationError(
                "fixed shoulder sphere must remain centered on the vertical base axis"
            )

        fixed_shoulder_support_clearance = (
            joint_xyz[2]
            + shoulder_center[2]
            - float(shoulder_radius)
            + support_recess
        )
        if fixed_shoulder_support_clearance < minimum_scene_clearance:
            raise ConfigurationError(
                "fixed shoulder sphere does not preserve the configured clearance from "
                "the recessed base support"
            )
        cutout_half_width = 0.5 * min(ground_cutout)
        fixed_shoulder_horizontal_requirement = (
            float(shoulder_radius) + minimum_scene_clearance
        )
        if cutout_half_width <= fixed_shoulder_horizontal_requirement:
            raise ConfigurationError(
                "fixed-base cutout does not preserve shoulder clearance from the z=0 slabs"
            )
        if abs(minimum_scene_clearance - 0.005) > 1.0e-12:
            raise ConfigurationError(
                "direct-Feature planner collision clearance must remain exactly 5 mm"
            )

    def _validate_asset_provenance(self, bundle: EnvironmentBundle) -> None:
        def require_sha256(value: object, label: str) -> str:
            if (
                not isinstance(value, str)
                or len(value) != 64
                or any(character not in "0123456789abcdef" for character in value)
            ):
                raise ConfigurationError(f"{label} must be a lowercase SHA-256 digest")
            return value

        def controlled_file(relative_value: object, expected: str, label: str) -> Path:
            if not isinstance(relative_value, str) or relative_value != expected:
                raise ConfigurationError(f"{label} must be exactly {expected}")
            relative = Path(relative_value)
            if relative.is_absolute() or ".." in relative.parts:
                raise ConfigurationError(f"{label} is not a safe environment-relative path")
            path = bundle.root / relative
            try:
                path.resolve(strict=True).relative_to(bundle.root)
            except (OSError, ValueError) as error:
                raise ConfigurationError(
                    f"{label} is missing or escapes the environment"
                ) from error
            if path.is_symlink() or not path.is_file():
                raise ConfigurationError(f"{label} must identify a regular non-symlink file")
            return path

        provenance_robot_type = _require(bundle.asset_provenance, "robot_type", str)
        calibration_robot_type = _require(
            bundle.calibrated_kinematics, "robot_type", str
        )
        if (
            provenance_robot_type != bundle.robot_type
            or calibration_robot_type != bundle.robot_type
        ):
            raise ConfigurationError(
                "asset provenance, calibration, and robot model types must match exactly"
            )
        _require(bundle.calibrated_kinematics, "deployment_allowed", bool)

        source_relative = ROBOT_DATA_FILES[0]
        source_path = controlled_file(
            _require(bundle.asset_provenance, "source_kinematics.path", str),
            source_relative,
            "asset_provenance.source_kinematics.path",
        )
        source_hash = _sha256(source_path)
        declared_source_hash = require_sha256(
            _require(bundle.asset_provenance, "source_kinematics.sha256", str),
            "asset_provenance.source_kinematics.sha256",
        )
        calibration_hash = require_sha256(
            _require(
                bundle.calibrated_kinematics,
                "driver_kinematics_sha256",
                str,
            ),
            "calibrated_kinematics.driver_kinematics_sha256",
        )
        if source_hash != declared_source_hash or source_hash != calibration_hash:
            raise ConfigurationError(
                "controlled driver kinematics hash does not match calibration/provenance"
            )

        asset_root = Path(
            _require(
                bundle.curobo_robot,
                "robot_cfg.kinematics.asset_root_path",
                str,
            )
        )
        urdf_relative = Path(
            _require(bundle.curobo_robot, "robot_cfg.kinematics.urdf_path", str)
        )
        if asset_root.is_absolute() or urdf_relative.is_absolute() or ".." in (
            *asset_root.parts,
            *urdf_relative.parts,
        ):
            raise ConfigurationError("cuRobo URDF paths must remain inside the environment")
        expected_urdf = (asset_root / urdf_relative).as_posix()
        urdf_path = controlled_file(
            _require(bundle.asset_provenance, "generated_urdf.path", str),
            expected_urdf,
            "asset_provenance.generated_urdf.path",
        )
        declared_urdf_hash = require_sha256(
            _require(bundle.asset_provenance, "generated_urdf.sha256", str),
            "asset_provenance.generated_urdf.sha256",
        )
        if _sha256(urdf_path) != declared_urdf_hash:
            raise ConfigurationError(
                "generated URDF hash does not match asset provenance"
            )

    def _hash_reproducible_inputs(self, root: Path) -> dict[str, str]:
        paths: list[Path] = []
        for relative in (*ROOT_CONFIG_FILES, *ROBOT_CONFIG_FILES, *ROBOT_DATA_FILES):
            paths.append(root / relative)
        for relative in REPRODUCIBLE_TREE_DIRECTORIES:
            directory = root / relative
            if directory.is_dir():
                paths.extend(path for path in directory.rglob("*") if path.is_file())
        return {
            str(path.relative_to(root)): _sha256(path)
            for path in sorted(paths, key=lambda item: str(item.relative_to(root)))
        }
