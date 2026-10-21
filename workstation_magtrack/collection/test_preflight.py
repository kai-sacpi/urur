"""Read-only readiness contracts; no ROS, device, GPU or wall-clock waits."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Any, Callable
from unittest.mock import patch

import pytest
import yaml

from .preflight import JointFeedback, ReadinessError, check_readiness, require_readiness


NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


@dataclass
class Clock:
    now: float = 100.

    def monotonic(self) -> float:
        return self.now

    def wall_time_ns(self) -> int:
        return 1_700_000_000_000_000_000 + round(self.now * 1e9)

    def joint(self) -> JointFeedback:
        return JointFeedback(NAMES, (0.,) * 6, (0.,) * 6, self.wall_time_ns(), self.now)


class Backend:
    def __init__(self, root: Path, clock: Clock, change: Callable[[dict[str, Any]], None] | None = None):
        self.root, self.clock, self.change = root, clock, change
        self.polls = 0
        self.closed = False
        self.poll_budgets: list[float] = []

    def poll(self, timeout_s: float) -> dict[str, Any]:
        self.polls += 1
        self.poll_budgets.append(timeout_s)
        self.clock.now += timeout_s
        value = {
            "runtime": {"execution_mode": "real", "environments_root": str(self.root / "environments"),
                        "telemetry_root": str(self.root / "runs"), "use_sim_time": False},
            "controllers": [{"name": "joint_trajectory_controller", "state": "active",
                             "type": "joint_trajectory_controller/JointTrajectoryController"}],
            "action_ready": True, "joints": [self.clock.joint()], "program_running": None,
        }
        if self.change:
            self.change(value)
        return value

    def close(self) -> None:
        self.closed = True


@pytest.fixture
def rig(tmp_path: Path):
    environment = tmp_path / "environments/mla_short"
    (environment / "robot").mkdir(parents=True)
    (environment / "robot/robot_model.yaml").write_text(yaml.safe_dump({"robot": {"joint_names": list(NAMES)}}))
    (environment / "controllers.yaml").write_text(yaml.safe_dump({"real": {
        "controller_type": "joint_trajectory_controller",
        "action_name": "/joint_trajectory_controller/follow_joint_trajectory"}}))
    clock = Clock()
    backend = Backend(tmp_path, clock)

    def run(**kwargs: Any) -> dict[str, Any]:
        return check_readiness(tmp_path, backend=backend, monotonic=clock.monotonic,
                               wall_time_ns=clock.wall_time_ns, **kwargs)

    return tmp_path, clock, backend, run


def test_live_success_needs_two_advancing_samples_and_closes(rig):
    root, _, backend, run = rig
    report = run()
    assert report["passed"] is True
    assert backend.polls == 2
    assert backend.closed
    assert report["checks"]["actual_joint_state"]["live_samples"] == 2
    assert report["checks"]["runtime"]["telemetry_root"] == str(root / "runs")
    assert report["serial_opened"] is False
    assert report["no_motion_goals"] is True
    assert report["elapsed_s"] < .2


@pytest.mark.parametrize("key,value", [("execution_mode", "simulation"),
    ("environments_root", "/wrong/environments"), ("telemetry_root", "/wrong/runs"),
    ("use_sim_time", True), ("use_sim_time", 0)])
def test_wrong_runtime_fails_immediately_without_more_probes(rig, key, value):
    _, _, backend, run = rig
    backend.change = lambda v: v["runtime"].update({key: value})
    report = run()
    assert not report["passed"]
    assert "runtime identity mismatch" in report["errors"][0]
    assert backend.polls == 1
    assert backend.closed


@pytest.mark.parametrize("key", ["runtime", "controllers"])
def test_service_discovery_without_response_never_passes(rig, key):
    _, clock, backend, run = rig
    started = clock.now
    backend.change = lambda v: v.update({key: None})
    report = run(timeout_s=.21)
    assert not report["passed"]
    assert "deadline" in report["errors"][0]
    assert key.rstrip("s") in report["errors"][0]
    assert clock.now - started <= .210000001
    assert backend.poll_budgets[-1] < .05
    assert backend.closed


@pytest.mark.parametrize("controllers", [[],
    [{"name": "joint_trajectory_controller", "state": "inactive", "type": "joint_trajectory_controller/JointTrajectoryController"}],
    [{"name": "scaled_joint_trajectory_controller", "state": "active", "type": "joint_trajectory_controller/JointTrajectoryController"}],
    [{"name": "joint_trajectory_controller", "state": "active", "type": "wrong"}]])
def test_inactive_missing_wrong_or_scaled_controller_cannot_pass(rig, controllers):
    _, _, backend, run = rig
    backend.change = lambda v: v.update({"controllers": controllers})
    report = run()
    assert not report["passed"]
    assert "not uniquely active" in report["errors"][0]
    assert backend.polls == 1


def test_duplicate_controller_response_is_refused(rig):
    _, _, backend, run = rig
    backend.change = lambda v: v["controllers"].extend(v["controllers"][:])
    assert not run()["passed"]


def test_controller_action_requires_actual_server_readiness(rig):
    _, _, backend, run = rig
    backend.change = lambda v: v.update({"action_ready": False})
    report = run(timeout_s=.11)
    assert not report["passed"]
    assert "controller_action" in report["errors"][0]


def test_action_that_disappeared_before_runtime_reply_cannot_pass(rig):
    _, _, backend, run = rig
    def disappearing(value):
        value["action_ready"] = backend.polls == 1
        value["runtime"] = None if backend.polls == 1 else value["runtime"]
    backend.change = disappearing
    report = run(timeout_s=.11)
    assert not report["passed"]
    assert "controller_action" in report["errors"][0]


@pytest.mark.parametrize("mutation,message", [
    (lambda s: replace(s, names=s.names[:-1], positions=s.positions[:-1], velocities=s.velocities[:-1]), "full six-joint"),
    (lambda s: replace(s, velocities=()), "full six-joint"),
    (lambda s: replace(s, names=(s.names[0],) * 6), "full six-joint"),
    (lambda s: replace(s, names=(*s.names, "extra"), positions=(*s.positions, 0.),
                       velocities=(*s.velocities, 0.)), "full six-joint"),
    (lambda s: replace(s, positions=(math.nan,) * 6), "nonfinite"),
    (lambda s: replace(s, velocities=(math.inf,) * 6), "nonfinite"),
    (lambda s: replace(s, source_stamp_ns=s.source_stamp_ns - 600_000_000), "stale/future"),
    (lambda s: replace(s, receipt_monotonic_s=s.receipt_monotonic_s - .6), "stale/future"),
    (lambda s: replace(s, source_stamp_ns=s.source_stamp_ns + 60_000_000), "stale/future"),
    (lambda s: replace(s, source_stamp_ns=s.source_stamp_ns + 1), "stale/future"),
    (lambda s: replace(s, receipt_monotonic_s=s.receipt_monotonic_s + .01), "stale/future"),
    (lambda s: replace(s, source_stamp_ns=0), "stale/future"),
])
def test_actual_feedback_is_full_finite_and_fresh_at_both_clocks(rig, mutation, message):
    _, _, backend, run = rig
    backend.change = lambda v: v.update({"joints": [mutation(v["joints"][0])]})
    report = run()
    assert not report["passed"]
    assert message in report["errors"][0]
    assert backend.polls == 1


def test_joint_names_are_mapped_not_assumed_in_message_order(rig):
    _, _, backend, run = rig
    def reorder(value):
        sample = value["joints"][0]
        value["joints"] = [replace(sample, names=tuple(reversed(NAMES)), positions=(1., 2., 3., 4., 5., 6.))]
    backend.change = reorder
    assert run()["checks"]["actual_joint_state"]["positions_rad"] == [6., 5., 4., 3., 2., 1.]


@pytest.mark.parametrize("backward_ns", [0, 1_000_000])
def test_repeated_or_regressing_source_clock_is_not_live(rig, backward_ns):
    _, clock, backend, run = rig
    first_stamp = clock.wall_time_ns() + 50_000_000
    backend.change = lambda v: v.update({"joints": [replace(v["joints"][0], source_stamp_ns=first_stamp -
                                                        (backward_ns if backend.polls > 1 else 0))]})
    report = run()
    assert not report["passed"]
    assert "did not advance" in report["errors"][0]


def test_feedback_that_ages_while_waiting_for_service_cannot_pass(rig):
    _, clock, backend, run = rig
    first = clock.joint()
    def delayed(value):
        value["joints"] = [first] if backend.polls == 1 else []
        value["runtime"] = value["runtime"] if backend.polls > 12 else None
    backend.change = delayed
    report = run(timeout_s=.8)
    assert not report["passed"]
    assert "two advancing" in report["errors"][0]


def test_two_early_samples_must_still_be_fresh_at_final_decision(rig):
    _, _, backend, run = rig
    def delayed(value):
        value["joints"] = value["joints"] if backend.polls <= 2 else []
        value["runtime"] = value["runtime"] if backend.polls >= 14 else None
    backend.change = delayed
    report = run(timeout_s=.8)
    assert not report["passed"]
    assert "stale/future" in report["errors"][0]


def test_optional_program_running_false_is_actionable_but_unknown_is_allowed(rig):
    _, _, backend, run = rig
    backend.change = lambda v: v.update({"program_running": False})
    report = run()
    assert not report["passed"]
    assert "External Control" in report["errors"][0]


def test_dead_runtime_single_request_failure_does_not_retry(rig):
    _, _, backend, run = rig
    backend.change = lambda v: v.update({"runtime_error": "request failed"})
    report = run()
    assert not report["passed"]
    assert backend.polls == 1
    assert backend.closed


def test_one_shared_deadline_rejects_backend_overrun(rig):
    _, clock, backend, run = rig
    backend.change = lambda v: setattr(clock, "now", clock.now + 1.)
    report = run(timeout_s=.2)
    assert not report["passed"]
    assert "shared deadline" in report["errors"][0]


def test_require_raises_with_structured_report(rig):
    root, clock, backend, _ = rig
    backend.change = lambda v: v.update({"runtime_error": "runtime dead"})
    with pytest.raises(ReadinessError, match="runtime dead") as caught:
        require_readiness(root, backend=backend, monotonic=clock.monotonic, wall_time_ns=clock.wall_time_ns)
    assert caught.value.report["passed"] is False


@pytest.mark.parametrize("timeout", [True, False, 0., -1., 20.1, math.inf, math.nan])
def test_invalid_timeout_never_opens_a_backend(rig, timeout):
    root, _, _, _ = rig
    with patch("workstation_magtrack.collection.preflight.RosReadinessBackend") as ros:
        with pytest.raises(ValueError, match="timeout"):
            check_readiness(root, timeout_s=timeout)
        ros.assert_not_called()


@pytest.mark.parametrize("domain", [True, False, 0., 1, -1, "0"])
def test_real_domain_is_explicit_zero(rig, domain):
    root, _, _, _ = rig
    with patch("workstation_magtrack.collection.preflight.RosReadinessBackend") as ros:
        with pytest.raises(ValueError, match="ROS_DOMAIN_ID=0"):
            check_readiness(root, domain_id=domain)
        ros.assert_not_called()


def test_cleanup_failure_never_reports_success(rig):
    _, _, backend, run = rig
    def fail_close():
        raise RuntimeError("cleanup failed")
    backend.close = fail_close
    report = run()
    assert not report["passed"]
    assert "cleanup failed" in report["errors"][0]


def test_cleanup_cannot_turn_over_budget_probe_into_success(rig):
    _, clock, backend, run = rig
    backend.close = lambda: setattr(clock, "now", clock.now + 1.)
    report = run(timeout_s=.2)
    assert not report["passed"]
    assert "cleanup exceeded" in report["errors"][0]
