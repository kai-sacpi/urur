import importlib.util
import csv
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).parents[1] / "runtime" / "playback_audit.py"
SPEC = importlib.util.spec_from_file_location("wm_play_audit", SOURCE)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def test_cached_20hz_progress_does_not_fake_incomplete_execution():
    # Real canary: 125 Hz actual states cover 32.9907243 s, while the runtime
    # correctly retains its most recent 20 Hz feedback value 32.944002369 s.
    rows = [{"sync_time_ns": str(round(t*1e9)), "trajectory_time_s": str(progress)}
            for t, progress in [(0, 0), (.008, 0), (.056, .048),
                                (32.944, 32.896), (32.982, 32.944002369),
                                (32.9907243, 32.944002369)]]
    assert audit.audit_timing(rows, 33, .01, .002, .05)[-1] == pytest.approx(32.9907243)


def test_actual_short_execution_still_rejected():
    rows = [{"sync_time_ns": "0", "trajectory_time_s": "0"},
            {"sync_time_ns": "32000000000", "trajectory_time_s": "31.99"}]
    with pytest.raises(ValueError, match="host duration"):
        audit.audit_timing(rows, 33, .01, .002, .05)


def test_terminal_hold_must_match_exact_sealed_goal_and_be_still():
    source = [SimpleNamespace(time_s=t, position_m=(0, 0, -.1), quaternion_xyzw=(0, 0, 0, 1))
              for t in (32.5, 33)]
    rows = [{"sync_time_ns": str(32800000000+i*10000000), "trajectory_time_s": "32.944",
             "joint_pos": "1", "joint_vel": "0"} for i in range(20)]
    result = audit.audit_terminal_hold(rows, source, [1], ["joint"], .02, .01)
    assert result["terminal_max_joint_error_rad"] == 0
    rows[-1]["joint_pos"] = "1.05"
    with pytest.raises(ValueError, match="not settled"):
        audit.audit_terminal_hold(rows, source, [1], ["joint"], .02, .01)


def test_controller_witness_selects_complete_goal_after_prepare_without_resampling():
    spec = importlib.util.spec_from_file_location("wm_controller_witness", SOURCE.with_name("controller_witness.py"))
    witness = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(witness)
    prepare = [{"reference_time_s": str(i*.008)} for i in range(80)]
    source = [{"reference_time_s": str(i*.008)} for i in range(126)]
    after = [{"reference_time_s": "0"}]
    selected, metrics = witness.select_episode(prepare + source + after, 1, .01, .002)
    assert selected == source
    assert metrics["raw_rows"] == 207
    assert metrics["selected_rows"] == 126
    assert metrics["native_median_interval_s"] == pytest.approx(.008)
    assert metrics["raw_goal_episodes"] == 3


@pytest.mark.parametrize("times,duration", [([10, 15], 20),
                                            ([0, .008, .016, .048, .056, .064], .064)])
def test_controller_witness_rejects_sparse_or_dropped_stream(times, duration):
    spec = importlib.util.spec_from_file_location("wm_controller_witness", SOURCE.with_name("controller_witness.py"))
    witness = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(witness)
    rows = [{"reference_time_s": str(t)} for t in times]
    with pytest.raises(ValueError, match="no complete"):
        witness.select_episode(rows, duration, .01, .002)


def test_perfect_w3_tracking_does_not_fail_on_cached_joint_delta(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[2] / "src" / "hx_waypoint_curobo"))
    from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
    from hx_waypoint_curobo.models import TimedPose
    from hx_waypoint_curobo.recording_store import RecordingInspector
    from hx_waypoint_curobo.robot_kinematics import Pose, RobotKinematics
    from hx_waypoint_curobo import trajectory_io
    omega = math.radians(20)  # W3, with a deliberately stale 80 ms cache.
    def quaternion(angle):
        return (0, 0, math.sin(angle/2), math.cos(angle/2))
    source = [TimedPose(t, "plane1", (0, 0, -.1), quaternion(omega*min(t, 1)), "draw")
              for t in (0, 1, 1.4)]
    tool = SimpleNamespace(parent_to_tip=SimpleNamespace(translation_m=(0, 0, 0),
                                                        quaternion_xyzw=(0, 0, 0, 1)),
                           parent_link="tool0", tip_frame="rod_tip")
    environment = SimpleNamespace(root=tmp_path, base_frame="base_link", joint_names=("joint",),
        asset_provenance={"generated_urdf": {"path": "robot.urdf"}}, setup_config=SimpleNamespace(tool=tool),
        transform_pose_to_base=lambda pose: pose,
        recording={"recording": {"sample_dt_s": .01, "validation": {
            "max_time_jitter_s": .002, "max_completion_overrun_s": .05,
            "max_frame_fk_position_error_m": .001, "max_joint_position_tracking_error_rad": .02}},
            "start_state": {"settle_velocity_rad_s": .01}},
        planner={"validation": {"tip_position_tolerance_m": .001, "tip_orientation_tolerance_deg": .5}})
    (tmp_path / "robot.urdf").write_text("unit-test model")
    def fk(positions, frames, **kwargs):
        return {"rod_tip": Pose(position_m=(0, 0, -.1), quaternion_xyzw=quaternion(positions[0]))}
    monkeypatch.setattr(EnvironmentBuilder, "load", lambda self, path: environment)
    monkeypatch.setattr(RecordingInspector, "inspect", lambda self, path: None)
    monkeypatch.setattr(trajectory_io, "read_timed_pose_csv", lambda path: source)
    monkeypatch.setattr(trajectory_io, "read_joint_trajectory_csv", lambda path, names:
                        SimpleNamespace(samples=[SimpleNamespace(positions=(omega,))]))
    monkeypatch.setattr(RobotKinematics, "from_urdf_bytes", classmethod(
        lambda cls, *args, **kwargs: SimpleNamespace(forward_kinematics=fk)))
    rows = []
    coherent_delta = 0.0
    for i in range(176):
        t = i*.008
        cached_t = math.floor((t+1e-12)/.08)*.08
        actual_q = omega*min(t, 1)
        reference_q = actual_q  # Perfect same-controller-time tracking.
        coherent_delta = max(coherent_delta, abs(actual_q-reference_q))
        rows.append({"sync_time_ns": round(t*1e9), "trajectory_time_s": cached_t,
                     "joint_pos": actual_q, "desired_joint_pos": omega*min(cached_t, 1),
                     "joint_vel": 0 if t >= 1 else omega, "tip_x": 0, "tip_y": 0,
                     "tip_z": -.1, "position_error_m": 0, "speed_scaling": 1})
    path = tmp_path / "telemetry.csv"
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    result = audit.audit_playback(path, tmp_path, tmp_path)
    assert result["status"] == "passed"
    assert coherent_delta == 0
    assert result["diagnostic_actual_vs_cached_desired_joint_delta_rad"] > .02
