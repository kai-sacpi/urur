#!/usr/bin/env python3
"""Validate the new parking motion without launching ROS or commanding a robot.

Default: CPU geometry precheck only. --gpu explicitly opts into offline cuRobo
planning/validation after the shared batch-campaign GPU lock becomes available.
The optional analytic source is a new upstream design, never runtime repair.
No domain is occupied and no existing runtime, setup, zones, or recordings change.
"""
from __future__ import annotations

import argparse
from collections.abc import Sequence
from contextlib import contextmanager
from dataclasses import asdict
from decimal import Decimal
import fcntl
import hashlib
import json
import math
from pathlib import Path
import subprocess
import re
import sys
import time
from typing import Any, TYPE_CHECKING
from uuid import uuid4

if TYPE_CHECKING:
    from hx_waypoint_curobo.models import CompiledTrajectory

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from collection.motion import (MotionController, sha256, utc_now, validate_trajectory,
                                   write_joint_csv_exact, write_json_new)
    from collection.geometry import screen_home_parking, screen_joint_positions
else:
    from .motion import (MotionController, sha256, utc_now, validate_trajectory,
                         write_joint_csv_exact, write_json_new)
    from .geometry import screen_home_parking, screen_joint_positions


DEFAULT_SOURCE_KIND = "prepare_planner_v1"
MINIMUM_JERK_SOURCE_KIND = "minimum_jerk_joint_bridge_v1"
SOURCE_KINDS = (DEFAULT_SOURCE_KIND,MINIMUM_JERK_SOURCE_KIND)


def snapshot_producer_sources(output: Path) -> dict[str, str]:
    """Bind the report to the exact three producer byte snapshots on disk."""
    hashes = {}
    for name in ("motion.py","geometry.py","validate_parking.py"):
        data = Path(__file__).with_name(name).read_bytes()
        with (output / f"{Path(name).stem}_source_at_launch.py").open("xb") as stream:
            stream.write(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
    return hashes


def minimum_jerk_joint_bridge(joint_names: Sequence[str], start: Sequence[float],
                              target: Sequence[float], revision: str, *,
                              duration_s: float = 20., sample_dt_s: float = .01) -> CompiledTrajectory:
    """Design a new upstream source, never repair or retime an existing CSV.

    Production v1 uses the fixed defaults. Explicit alternative durations are
    for new upstream candidates/tests; unchanged dynamics admission may reject.
    """
    from hx_waypoint_curobo.models import CompiledTrajectory, JointSample
    values = (duration_s,sample_dt_s)
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or x <= 0 for x in values):
        raise ValueError("candidate duration and sample interval must be finite positive seconds")
    nanos = [Decimal(str(x))*1_000_000_000 for x in values]
    if any(x != x.to_integral_value() for x in nanos):
        raise ValueError("candidate duration and sample interval require exact integer nanoseconds")
    duration_ns, dt_ns = map(int,nanos)
    if duration_ns % dt_ns or duration_ns < dt_ns or duration_ns > 120_000_000_000:
        raise ValueError("candidate duration requires a whole number of sample intervals within 120 seconds")
    names, start, target = tuple(joint_names),tuple(start),tuple(target)
    if (len(names) != 6 or any(not isinstance(name,str) or not name.strip() for name in names)
            or len(set(names)) != 6 or len(start) != 6 or len(target) != 6
            or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x)
                   for x in (*start,*target))):
        raise ValueError("candidate requires six finite joints in one unique joint order")
    steps = duration_ns//dt_ns
    duration = duration_ns/1e9
    delta = tuple(b-a for a,b in zip(start,target,strict=True))
    samples = []
    zero = (0.,)*6
    for index in range(steps+1):
        t = index*dt_ns/1e9
        if index in (0,steps):
            samples.append(JointSample(t,start if index == 0 else target,zero,zero,"approach"))
            continue
        u = index/steps
        s = u*u*u*(10.+u*(-15.+6.*u))
        ds = 30.*u*u*(1.-u)*(1.-u)/duration
        dds = 60.*u*(1.-u)*(1.-2.*u)/(duration*duration)
        samples.append(JointSample(t,tuple(a+d*s for a,d in zip(start,delta,strict=True)),
            tuple(d*ds for d in delta),tuple(d*dds for d in delta),"approach"))
    return CompiledTrajectory(names,tuple(samples),metadata={"prepare":{
        "curobo_revision":revision,"uniform_time_scale":1.,"source_kind":MINIMUM_JERK_SOURCE_KIND,
        "duration_s":duration,"sample_dt_s":dt_ns/1e9,"duration_ns":duration_ns,"sample_dt_ns":dt_ns,
        "authority_contract":"exact_q_qd_qdd_time_v1","upstream_source_generated_before_validation":True}})


def write_minimum_jerk_candidate(directory: Path, joint_names: Sequence[str], start: Sequence[float],
                                  target: Sequence[float], revision: str, *,
                                  duration_s: float = 20., sample_dt_s: float = .01,
                                  generator_sha256: str | None = None) -> tuple[CompiledTrajectory,dict[str,Any],bytes]:
    """Seal a brand-new source before admission, then parse that same snapshot."""
    if __package__ in {None,""}:
        from collection.prepared_parking import _joint_bytes
    else:
        from .prepared_parking import _joint_bytes
    from hx_waypoint_curobo.models import CompiledTrajectory
    compiled = minimum_jerk_joint_bridge(joint_names,start,target,revision,
                                        duration_s=duration_s,sample_dt_s=sample_dt_s)
    csv = directory / "candidate_joint_trajectory.csv"
    write_joint_csv_exact(csv,compiled)
    data = csv.read_bytes()
    candidate_sha = hashlib.sha256(data).hexdigest()
    parameters = compiled.metadata["prepare"]
    manifest = {"schema_version":1,"kind":"immutable_upstream_parking_candidate",
        "source_kind":MINIMUM_JERK_SOURCE_KIND,"created_utc":utc_now(),
        "generator":"validate_parking.py::minimum_jerk_joint_bridge",
        "generator_code_sha256":generator_sha256 or sha256(Path(__file__)),
        "formula":"s=10*u^3-15*u^4+6*u^5; analytic q, qd, qdd; u=t/T",
        "joint_names":list(compiled.joint_names),"start_positions_rad":list(start),"target_positions_rad":list(target),
        "duration_s":parameters["duration_s"],"sample_dt_s":parameters["sample_dt_s"],
        "duration_ns":parameters["duration_ns"],"sample_dt_ns":parameters["sample_dt_ns"],
        "point_count":len(compiled.samples),"candidate_path":str(csv),"candidate_sha256":candidate_sha,
        "authority_contract":"exact_q_qd_qdd_time_v1","uniform_time_scale":1.,
        "candidate_written_before_validation":True,"runtime_modification_permitted":False,
        "simulation_only":True,"real_execution_authorized":False}
    manifest_path = directory / "candidate.json"
    write_json_new(manifest_path,manifest)
    restored = _joint_bytes(data,compiled.joint_names)
    if restored.samples != compiled.samples:
        raise ValueError("candidate source authority changed while serializing q/qd/qdd/time")
    proof = {**manifest,"manifest_path":str(manifest_path),"manifest_sha256":sha256(manifest_path)}
    metadata = {"prepare":{**parameters,"candidate_sha256":candidate_sha,
                           "candidate_manifest_sha256":proof["manifest_sha256"]}}
    return CompiledTrajectory(restored.joint_names,restored.samples,metadata),proof,data


@contextmanager
def campaign_gpu_lock(workspace: Path, wait: bool = False):
    """Exactly the same lock as the v2/v3 supervisors; never preempts them."""
    path = workspace / "continuous_augmented/replay/.campaign.lock"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+") as stream:
        last_report = 0.
        while True:
            try:
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if not wait:
                    raise RuntimeError("batch campaign is active; no GPU work started (use --wait-lock to queue)")
                if time.monotonic()-last_report >= 60:
                    print("Parking GPU validation queued behind existing replay campaigns; no ROS/robot access.", flush=True)
                    last_report = time.monotonic()
                time.sleep(1.)
        try:
            yield path
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)


def gpu_inventory() -> dict[str, object]:
    command = ["nvidia-smi", "--query-gpu=index,name,memory.used,memory.total,utilization.gpu", "--format=csv,noheader,nounits"]
    result = subprocess.run(command, check=True, capture_output=True, text=True, timeout=20.)
    if not result.stdout.strip():
        raise RuntimeError("nvidia-smi returned no GPU inventory")
    return {"command": command, "stdout": result.stdout.strip(), "checked_utc": utc_now()}


def wait_for_batches(workspace: Path, names: list[str]) -> dict[str, object]:
    """Do not compete with an already queued v3 owner when v2 releases its lock."""
    result: dict[str, object] = {}
    for name in names:
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*",name):
            raise ValueError("unsafe --after-batch name")
        path = workspace / "runs" / name / "campaign_summary.json"
        last_report = 0.
        while True:
            if path.is_file():
                summary = json.loads(path.read_text())
                state = summary.get("state")
                if state in {"failed","interrupted"} or (state == "complete" and summary.get("workflow_passed") is not True):
                    raise RuntimeError(f"prerequisite replay campaign {name} ended without full acceptance: {state}")
                if state == "complete" and summary.get("workflow_passed") is True:
                    result[name] = {"path":str(path),"sha256":sha256(path),"workflow_passed":True}
                    break
            if time.monotonic()-last_report >= 60:
                print(f"Parking validation queued after {name}; no GPU/ROS/robot access.",flush=True)
                last_report = time.monotonic()
            time.sleep(1.)
    return result


def validate(workspace: Path, config: Path, output: Path, *, gpu: bool = False,
             wait_lock: bool = False, after_batches: list[str] | None = None,
             source_kind: str = DEFAULT_SOURCE_KIND) -> dict[str, object]:
    if source_kind not in SOURCE_KINDS:
        raise ValueError("unknown parking source kind")
    motion = MotionController(workspace, config, mode="simulation", domain_id=221)
    output.mkdir(parents=True, exist_ok=False)
    code_hashes = snapshot_producer_sources(output)
    evidence: dict[str, object] = {
        "schema_version":1, "environment_id":"mla_short", "simulation_only":True,
        "real_execution_authorized":False, "robot_execution":"not_requested_not_run",
        "ros_runtime":"not_started", "started_utc":utc_now(), "status":"running",
        "parking_joint_positions_deg":motion.config["parking"]["joint_positions_deg"],
        "parking_joint_positions_rad":list(motion.park), "home_joint_positions_rad":list(motion.home),
        "joint_names":list(motion.environment.joint_names), "environment_hashes":motion.environment_hashes,
        "curobo_lock_sha256":motion.lock_sha256, "config_sha256":motion.config_sha256,
        "code_sha256":code_hashes,"source_kind":source_kind,
        "hard_world_self_clearance_m":.005, "rod_length_m":.190, "rod_diameter_m":.005,
        "uniform_time_scale":1., "authority_mutated":False,
        "gpu_full_path_validation":"not_run", "runtime_ready":False, "directions":{}}
    try:
        evidence["cpu_precheck"] = screen_home_parking(motion.environment, motion.park)
        evidence["cpu_precheck"]["admission_role"] = (
            "straight-line diagnostics only; analytic source must retain every exact joint/time value"
            if source_kind == MINIMUM_JERK_SOURCE_KIND else
            "straight-line diagnostics only; GPU may find a detour")
        evidence["cpu_endpoints"] = screen_joint_positions(motion.environment,[motion.home,motion.park])
        if evidence["cpu_endpoints"]["status"] == "rejected":
            raise ValueError("CPU nominal geometry rejected mandatory home/parking endpoint")
        if not gpu:
            evidence["status"] = "cpu_precheck_only"
            return evidence
        evidence["waited_for_batches"] = wait_for_batches(workspace,after_batches or [])
        with campaign_gpu_lock(workspace, wait_lock) as path:
            evidence["gpu_exclusive_lock"] = str(path)
            evidence["gpu_inventory_before"] = gpu_inventory()
            motion._unchanged()
            from hx_waypoint_curobo.curobo_adapter_v2 import CuroboAdapterV2
            from hx_waypoint_curobo.prepare_planner import PreparePlanner
            from hx_waypoint_curobo.trajectory_io import read_joint_trajectory_csv
            adapter = CuroboAdapterV2(motion.environment, workspace / "curobo.lock")
            planner = PreparePlanner(motion.environment, adapter) if source_kind == DEFAULT_SOURCE_KIND else None
            adapter.warmup()
            for direction, start, target in (("home_to_park",motion.home,motion.park),
                                             ("park_to_home",motion.park,motion.home)):
                item: dict[str, object] = {"status":"running", "start_positions_rad":list(start),
                    "target_positions_rad":list(target), "started_utc":utc_now()}
                evidence["directions"][direction] = item
                try:
                    motion._unchanged()
                    directory = output / direction
                    candidate = candidate_data = None
                    if source_kind == MINIMUM_JERK_SOURCE_KIND:
                        directory.mkdir()
                        compiled,candidate,candidate_data = write_minimum_jerk_candidate(
                            directory,motion.environment.joint_names,start,target,adapter.revision,
                            generator_sha256=code_hashes["validate_parking.py"])
                        item["upstream_candidate"] = candidate
                    else:
                        compiled = planner.plan(motion.environment.joint_names, start, target)
                    dynamics = validate_trajectory(motion.environment,compiled,start,target)
                    gpu_metrics = adapter.validate_joint_trajectory(compiled)
                    if source_kind == DEFAULT_SOURCE_KIND:
                        directory.mkdir()
                    csv_path = directory / "joint_trajectory.csv"
                    if candidate_data is None:
                        write_joint_csv_exact(csv_path,compiled)
                        restored = read_joint_trajectory_csv(csv_path,motion.environment.joint_names)
                    else:
                        with csv_path.open("xb") as stream:
                            stream.write(candidate_data)
                        if __package__ in {None,""}:
                            from collection.prepared_parking import _joint_bytes
                        else:
                            from .prepared_parking import _joint_bytes
                        restored = _joint_bytes(candidate_data,motion.environment.joint_names)
                    if restored.samples != compiled.samples:
                        raise ValueError("authoritative q/qd/qdd/t changed during CSV serialization")
                    # Independently validate the immutable replay bytes too.
                    validate_trajectory(motion.environment,restored,start,target)
                    serialized_metrics = adapter.validate_joint_trajectory(restored)
                    cpu = screen_joint_positions(motion.environment,[s.positions for s in restored.samples])
                    if cpu["status"] == "rejected":
                        raise ValueError("serialized emitted-point CPU geometry rejected GPU path")
                    if candidate is not None:
                        if (Path(candidate["candidate_path"]).read_bytes() != candidate_data
                                or csv_path.read_bytes() != candidate_data
                                or sha256(Path(candidate["manifest_path"])) != candidate["manifest_sha256"]):
                            raise ValueError("immutable upstream candidate or exact formal replay bytes changed")
                    motion._unchanged()
                    item.update(status="passed", finished_utc=utc_now(),
                        planner=("upstream analytic minimum-jerk joint bridge -> immutable authoritative CSV -> unchanged full-path GPU validator"
                                 if source_kind == MINIMUM_JERK_SOURCE_KIND else
                                 "PreparePlanner.plan -> cuRoboV2 plan_cspace -> fixed-clock optimizer -> full-path GPU validator"),
                        curobo_revision=adapter.revision, joint_trajectory_path=str(csv_path),
                        joint_trajectory_sha256=sha256(csv_path), dynamics=dynamics,
                        gpu_metrics=asdict(gpu_metrics), serialized_gpu_metrics=asdict(serialized_metrics),
                        cpu_sampled_geometry=cpu, prepare_metadata=dict(compiled.metadata))
                    write_json_new(directory / "evidence.json",item)
                except BaseException as error:
                    item.update(status="failed",finished_utc=utc_now(),error=f"{type(error).__name__}: {error}")
                    raise
            motion._unchanged()
            evidence.update(status="passed",gpu_full_path_validation="passed_both_directions",runtime_ready=True)
            return evidence
    except BaseException as error:
        evidence.update(status="failed",error=f"{type(error).__name__}: {error}")
        raise
    finally:
        evidence["finished_utc"] = utc_now()
        write_json_new(output / "validation.json",evidence)
        motion.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace",type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument("--config",type=Path)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--gpu",action="store_true",help="explicitly plan/validate on GPU; never commands a robot")
    parser.add_argument("--source-kind",choices=SOURCE_KINDS,default=DEFAULT_SOURCE_KIND,
                        help="explicit new upstream candidate source; never modifies any previous trajectory")
    parser.add_argument("--wait-lock",action="store_true",help="queue behind current replay supervisor")
    parser.add_argument("--after-batch",action="append",default=[],help="wait for this replay batch's full workflow acceptance before taking GPU lock")
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()
    config = (args.config or workspace / "collection/config.yaml").resolve()
    output = (args.output or workspace / "collection/parking_validation" / f"{time.time_ns()}_{uuid4().hex[:8]}").resolve()
    if args.after_batch and not args.gpu:
        parser.error("--after-batch only makes sense with explicit --gpu")
    result = validate(workspace,config,output,gpu=args.gpu,wait_lock=args.wait_lock,after_batches=args.after_batch,
                      source_kind=args.source_kind)
    print(f"{result['status']}: {output / 'validation.json'}",flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
