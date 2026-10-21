"""No GPU/ROS imports or remote/robot actions: nominal CPU screen regression."""
from __future__ import annotations

import copy
from dataclasses import replace
import json
from pathlib import Path
import unittest

import numpy as np

from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
from hx_waypoint_curobo.math3d import pose_matrix
from hx_waypoint_curobo.robot_kinematics import RobotKinematics

from workstation_magtrack.collection.geometry import (
    _self_pairs, _sphere_box_clearances, screen_home_parking, screen_joint_positions,
)


WORKSPACE = Path(__file__).resolve().parents[1]
PARKING = np.radians([-141.48, -84.74, -22.66, 19.73, 99.31, 42.18])


class GeometryScreenTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.env = EnvironmentBuilder().load(WORKSPACE / "environments/mla_short")
        cls.result = screen_home_parking(cls.env, PARKING)

    def test_known_parking_full_model_sampled_clearances(self) -> None:
        result = self.result
        self.assertEqual(result["status"], "CPU_sampled_passed")
        self.assertEqual(result["sample_count"], 201)
        self.assertEqual(result["required_minimum_clearance_m"], .005)
        self.assertEqual(result["robot_sphere_count"], 14)
        self.assertEqual(result["rod_sphere_count"], 39)
        self.assertEqual(result["world_cuboid_count"], 8)
        self.assertEqual(result["self_pair_count"], 470)
        self.assertAlmostEqual(result["minimum_world_clearance_m"], .031175137633868, places=12)
        self.assertAlmostEqual(result["minimum_self_clearance_m"], .029153568619374, places=12)
        self.assertEqual(result["world_minimum"]["obstacle_id"], "base_ground_y_negative")
        self.assertEqual(result["world_minimum"]["link_name"], "upper_arm_link")
        self.assertEqual(result["world_minimum"]["row_index"], 200)
        self.assertEqual(result["self_minimum"]["link_names"], ["forearm_link", "wrist_2_link"])
        self.assertTrue(all(row["joint_limits_passed"] and row["canonical_branch_passed"]
                            for row in result["rows"]))

    def test_exact_base_link_and_plane1_fk(self) -> None:
        np.testing.assert_allclose(self.result["end_tip_position_base_link_m"],
                                   [.02000572980357, -.06901496747779, 1.15153928283576], atol=1e-12)
        np.testing.assert_allclose(self.result["end_tip_position_plane1_m"],
                                   [.332120386501815, .449445928989515, -.962208271276683], atol=1e-12)
        # The UR controller's base has a fixed Rz(pi) from base_link; it is not
        # legitimate to attach its measured plane transform to base_link.
        kin = RobotKinematics.from_urdf_bytes(
            (self.env.root / self.env.asset_provenance["generated_urdf"]["path"]).read_bytes(),
            base_link=self.env.base_frame, joint_names=self.env.joint_names,
        )
        controller_base = kin.forward_kinematics(PARKING, ("base",))["base"]
        base_link_from_base = pose_matrix(controller_base.position_m, controller_base.quaternion_xyzw)
        self.assertFalse(np.allclose(base_link_from_base, np.eye(4)))

    def test_never_claims_planning_gpu_continuous_or_real_approval(self) -> None:
        result = self.result
        self.assertTrue(result["sampling_only"])
        self.assertTrue(result["reverse_direction_has_same_geometric_samples"])
        for flag in ("gpu_validated", "continuous_collision_checked", "dynamics_checked",
                     "real_execution_authorized", "motion_planned"):
            self.assertFalse(result[flag])
        self.assertIn("nominal", result["scope"])
        json.dumps(result, allow_nan=False)

    def test_home_and_parking_samples_are_not_modified(self) -> None:
        np.testing.assert_array_equal(self.result["rows"][0]["joint_positions_rad"], self.env.home_joint_positions)
        np.testing.assert_array_equal(self.result["rows"][-1]["joint_positions_rad"], PARKING)
        rows = np.linspace(self.env.home_joint_positions, PARKING, 11)
        before = rows.copy()
        forward = screen_joint_positions(self.env, rows)
        reverse = screen_joint_positions(self.env, rows[::-1])
        np.testing.assert_array_equal(rows, before)
        self.assertEqual(forward["minimum_world_clearance_m"], reverse["minimum_world_clearance_m"])
        self.assertEqual(forward["minimum_self_clearance_m"], reverse["minimum_self_clearance_m"])

    def test_path_environment_argument(self) -> None:
        result = screen_joint_positions(self.env.root, [PARKING])
        np.testing.assert_allclose(result["end_tip_position_plane1_m"], self.result["end_tip_position_plane1_m"])

    def test_signed_oriented_sphere_box_distance(self) -> None:
        # Identity unit-half-size box: center, face, edge/corner, and outside.
        centers = np.asarray([[0, 0, 0], [1, 0, 0], [2, 2, 0], [0, 0, 2]], dtype=float)
        radii = np.full(4, .2)
        expected = np.asarray([-1.2, -.2, np.sqrt(2) - .2, .8])
        np.testing.assert_allclose(_sphere_box_clearances(centers, radii, np.eye(4), np.full(3, 2.)), expected)
        angle = .63
        transform = pose_matrix([.3, -.4, .7], [0, 0, np.sin(angle / 2), np.cos(angle / 2)])
        transformed = centers @ transform[:3, :3].T + transform[:3, 3]
        np.testing.assert_allclose(_sphere_box_clearances(transformed, radii, transform, np.full(3, 2.)), expected)

    def test_rod_is_not_omitted_from_world_collision(self) -> None:
        env = self.env
        kin = RobotKinematics.from_urdf_bytes(
            (env.root / env.asset_provenance["generated_urdf"]["path"]).read_bytes(),
            base_link=env.base_frame, joint_names=env.joint_names,
        )
        tool = kin.forward_kinematics(PARKING, ("tool0",))["tool0"]
        transform = pose_matrix(tool.position_m, tool.quaternion_xyzw)
        center = (transform @ [.0, .0, .15, 1.])[:3]
        zones = copy.deepcopy(env.zones)
        zones["curobo_scene"]["cuboid"] = {"rod_only_obstacle": {
            "frame_id": env.base_frame, "dims": [.005, .005, .005],
            "pose": [*center.tolist(), 1., 0., 0., 0.],
        }}
        result = screen_joint_positions(replace(env, zones=zones), [PARKING])
        self.assertEqual(result["status"], "rejected")
        self.assertEqual(result["world_minimum"]["obstacle_id"], "rod_only_obstacle")
        self.assertIn("/rod_", result["world_minimum"]["sphere_name"])
        self.assertLess(result["minimum_world_clearance_m"], 0.)

    def test_self_ignore_is_symmetric_and_same_link_is_excluded(self) -> None:
        i, j = _self_pairs(["a", "b", "b", "c"], {"c": ["a"]})
        self.assertEqual(list(zip(i.tolist(), j.tolist())), [(0, 1), (0, 2), (1, 3), (2, 3)])

    def test_intermediate_sample_collision_is_not_hidden_by_free_endpoints(self) -> None:
        env = self.env
        midpoint = (np.asarray(env.home_joint_positions) + PARKING) / 2.
        kin = RobotKinematics.from_urdf_bytes(
            (env.root / env.asset_provenance["generated_urdf"]["path"]).read_bytes(),
            base_link=env.base_frame, joint_names=env.joint_names,
        )
        tool = kin.forward_kinematics(midpoint, ("tool0",))["tool0"]
        center = (pose_matrix(tool.position_m, tool.quaternion_xyzw) @ [0., 0., .15, 1.])[:3]
        zones = copy.deepcopy(env.zones)
        zones["curobo_scene"]["cuboid"] = {"midpoint_rod_obstacle": {
            "frame_id": env.base_frame, "dims": [.005] * 3,
            "pose": [*center.tolist(), 1., 0., 0., 0.],
        }}
        result = screen_joint_positions(replace(env, zones=zones),
                                        [env.home_joint_positions, midpoint, PARKING])
        self.assertEqual(result["status"], "rejected")
        self.assertEqual(result["world_minimum"]["row_index"], 1)
        self.assertGreater(result["rows"][0]["minimum_world_clearance_m"], .005)
        self.assertGreater(result["rows"][2]["minimum_world_clearance_m"], .005)
        self.assertLess(result["rows"][1]["minimum_world_clearance_m"], 0.)

    def test_self_buffer_is_applied_without_changing_world_geometry(self) -> None:
        config = copy.deepcopy(self.env.curobo_robot)
        config["robot_cfg"]["kinematics"]["self_collision_buffer"]["forearm_link"] = .2
        result = screen_joint_positions(replace(self.env, curobo_robot=config), [PARKING])
        self.assertEqual(result["status"], "rejected")
        self.assertLess(result["minimum_self_clearance_m"], 0.)
        self.assertAlmostEqual(result["minimum_world_clearance_m"], self.result["minimum_world_clearance_m"])

    def test_global_sphere_buffer_applies_to_world_and_both_self_radii(self) -> None:
        config = copy.deepcopy(self.env.curobo_robot)
        config["robot_cfg"]["kinematics"]["collision_sphere_buffer"] = .05
        result = screen_joint_positions(replace(self.env, curobo_robot=config), [PARKING])
        self.assertEqual(result["status"], "rejected")
        self.assertAlmostEqual(result["minimum_world_clearance_m"], self.result["minimum_world_clearance_m"] - .05)
        self.assertAlmostEqual(result["minimum_self_clearance_m"], self.result["minimum_self_clearance_m"] - .1)

    def test_mismatching_kinematics_base_is_rejected(self) -> None:
        config = copy.deepcopy(self.env.curobo_robot)
        config["robot_cfg"]["kinematics"]["base_link"] = "base"
        with self.assertRaises(ValueError):
            screen_joint_positions(replace(self.env, curobo_robot=config), [PARKING])

    def test_every_row_reports_joint_domain_and_branch_rejections(self) -> None:
        branch_bad = PARKING.copy()
        branch_bad[0] = .3
        limit_bad = PARKING.copy()
        limit_bad[3] = 7.
        result = screen_joint_positions(self.env, [PARKING, branch_bad, limit_bad])
        self.assertEqual(result["status"], "rejected")
        self.assertFalse(result["rows"][1]["canonical_branch_passed"])
        self.assertFalse(result["rows"][2]["joint_limits_passed"])
        self.assertIn("wrist_1_joint", result["rows"][2]["violating_joint_names"])

    def test_invalid_inputs_and_weakened_clearance_fail_closed(self) -> None:
        for positions in ([], [1, 2, 3], [[0.] * 5], [[np.nan] * 6], [[np.inf] * 6]):
            with self.subTest(positions=positions), self.assertRaises(ValueError):
                screen_joint_positions(self.env, positions)
        for count in (0, 1, True, 1.5):
            with self.subTest(count=count), self.assertRaises(ValueError):
                screen_home_parking(self.env, PARKING, sample_count=count)
        for target in ([0.] * 5, [np.nan] * 6):
            with self.assertRaises(ValueError):
                screen_home_parking(self.env, target)
        planner = copy.deepcopy(self.env.planner)
        planner["validation"]["minimum_collision_distance_m"] = .004
        with self.assertRaises(ValueError):
            screen_joint_positions(replace(self.env, planner=planner), [PARKING])


if __name__ == "__main__":
    unittest.main()
