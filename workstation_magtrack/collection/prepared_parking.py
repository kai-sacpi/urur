#!/usr/bin/env python3
"""Immutable, portable prepare assets, not tipdraw recordings or real releases.

The original GPU producer certificate and matching producer source snapshots
remain byte-identical. A new bundle separately binds the versioned consumer;
it never relabels an old GPU run as having executed the new consumer code.
"""
from __future__ import annotations

import argparse
import csv
from decimal import Decimal, InvalidOperation
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Mapping

if __package__ in {None, ""}:
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from collection.motion import sha256, utc_now, validate_trajectory, write_json_new
else:
    from .motion import sha256, utc_now, validate_trajectory, write_json_new


DIRECTIONS = ("home_to_park","park_to_home")
PRODUCER_ALIASES = {"motion.py":"motion_source_at_launch.py",
                    "geometry.py":"geometry_source_at_launch.py",
                    "validate_parking.py":"validate_parking_source_at_launch.py"}
CONSUMER_FILES = ("motion.py","geometry.py","prepared_parking.py")
ASSET_TYPE = "collision_checked_prepare_joint_bundle"
AUTHORITY_CONTRACT = "exact_q_qd_qdd_time_v1"
DEFAULT_SOURCE_KIND = "prepare_planner_v1"
MINIMUM_JERK_SOURCE_KIND = "minimum_jerk_joint_bridge_v1"
CANDIDATE_FILES = ("candidate_joint_trajectory.csv","candidate.json")


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value,dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _file(root: Path, relative: str) -> Path:
    if not isinstance(relative,str) or not relative or Path(relative).is_absolute() or ".." in Path(relative).parts:
        raise ValueError("prepare bundle path must be a safe relative file")
    path = root / relative
    if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(root.resolve()):
        raise ValueError(f"prepare bundle has a missing, symlink, or escaping file: {relative}")
    return path


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _number(value: Any, label: str) -> float:
    _require(not isinstance(value,bool) and isinstance(value,(int,float)) and math.isfinite(value),
             f"nonfinite or nonnumeric {label}")
    return float(value)


def _positions(actual: Any, expected: Any, label: str, tolerance: float = 1e-12) -> None:
    _require(isinstance(actual,(list,tuple)) and len(actual)==6,f"missing six {label} positions")
    _require(max(abs(_number(a,label)-float(b)) for a,b in zip(actual,expected,strict=True)) <= tolerance,
             f"{label} differs from frozen home/confirmed parking")


def _environment(workspace: Path, config_path: Path):
    import yaml
    from hx_waypoint_curobo.environment_builder import EnvironmentBuilder
    env = EnvironmentBuilder().load(workspace / "environments/mla_short")
    config = yaml.safe_load(config_path.read_text())
    _require(config.get("environment_id")=="mla_short" and config["parking"].get("operator_confirmed") is True,
             "parking configuration is not confirmed mla_short")
    _require(tuple(config["parking"]["joint_names"])==env.joint_names,"parking joint order changed")
    park = tuple(math.radians(_number(x,"parking degrees")) for x in config["parking"]["joint_positions_deg"])
    _require(len(park)==6,"parking requires six joints")
    _require(abs(float(env.tcp["rod"]["length_m"])-.190)<1e-12 and
             abs(float(env.tcp["rod"]["radius_m"])-.0025)<1e-12 and
             abs(float(env.planner["validation"]["minimum_collision_distance_m"])-.005)<1e-12,
             "prepare bundle requires unchanged mla_short rod190/d5 and hard 5mm margin")
    revision = str(yaml.safe_load((workspace/"curobo.lock").read_text())["commit"])
    return env, config, park, revision


def _gpu_metrics(value: Mapping[str, Any], count: int, label: str) -> None:
    _require(value.get("checked_sample_count")==count,f"{label} did not check every emitted point")
    _require(_number(value.get("minimum_collision_distance_m"),label)>=.005,
             f"{label} lacks hard 5mm world/self clearance")
    for key in ("max_scene_collision_cost","max_self_collision_cost","max_joint_bound_cost"):
        _require(0<=_number(value.get(key),key)<=1e-7,f"{label} has nonzero collision/bound cost")
    for key in ("max_tip_position_error_m","max_tip_orientation_error_deg"):
        _require(_number(value.get(key),key)>=0,f"{label} has invalid metric")


def _read_joint(path: Path, names: tuple[str, ...]):
    return _joint_bytes(path.read_bytes(),names)


def _joint_bytes(data: bytes, names: tuple[str, ...]):
    from hx_waypoint_curobo.trajectory_io import joint_trajectory_columns
    from hx_waypoint_curobo.models import CompiledTrajectory, JointSample
    reader = csv.DictReader(io.StringIO(data.decode("utf-8"),newline=""))
    _require(tuple(reader.fieldnames or [])==joint_trajectory_columns(names),"prepare CSV header/order changed")
    samples = []
    for row in reader:
        _require(None not in row and all(v is not None for v in row.values()),"prepare CSV row width changed")
        try:
            ns = Decimal(row["time_s"])*1_000_000_000
            _require(ns.is_finite() and ns==ns.to_integral_value(),"prepare CSV time is not exact ROS nanoseconds")
            t = int(ns)/1e9
            _require(round(t*1e9)==int(ns),"prepare CSV time does not round-trip on ROS clock")
        except (InvalidOperation,OverflowError) as error:
            raise ValueError("invalid prepare CSV time") from error
        samples.append(JointSample(t,tuple(float(row[f"{j}_pos"]) for j in names),
            tuple(float(row[f"{j}_vel"]) for j in names),tuple(float(row[f"{j}_acc"]) for j in names),row["phase"]))
    return CompiledTrajectory(names,tuple(samples))


def _validate_candidate(report: dict[str, Any], item: dict[str, Any], directory: Path,
                        trajectory: Any, formal_bytes: bytes, start: Any, target: Any,
                        direction: str, *, bundled: bool) -> dict[str, bytes]:
    """Verify new upstream authority without reading old absolute paths in bundles."""
    proof = item.get("upstream_candidate")
    _require(isinstance(proof,dict),f"{direction} missing immutable upstream candidate provenance")
    manifest_path = _file(directory,"candidate.json")
    manifest_bytes = manifest_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    _require(isinstance(manifest,dict),f"{direction} candidate manifest is not an object")
    manifest_sha = hashlib.sha256(manifest_bytes).hexdigest()
    candidate_path = _file(directory,"candidate_joint_trajectory.csv")
    candidate_bytes = candidate_path.read_bytes()
    candidate_sha = hashlib.sha256(candidate_bytes).hexdigest()
    _require(proof=={**manifest,"manifest_path":proof.get("manifest_path"),"manifest_sha256":manifest_sha},
             f"{direction} upstream candidate proof differs from original manifest or SHA")
    _require(manifest.get("schema_version")==1 and
             manifest.get("kind")=="immutable_upstream_parking_candidate" and
             manifest.get("source_kind")==MINIMUM_JERK_SOURCE_KIND and
             manifest.get("authority_contract")==AUTHORITY_CONTRACT and
             manifest.get("uniform_time_scale")==1. and
             manifest.get("candidate_written_before_validation") is True and
             manifest.get("runtime_modification_permitted") is False and
             manifest.get("simulation_only") is True and manifest.get("real_execution_authorized") is False,
             f"{direction} candidate authority contract or generation identity changed")
    _require(manifest.get("generator")=="validate_parking.py::minimum_jerk_joint_bridge" and
             manifest.get("formula")=="s=10*u^3-15*u^4+6*u^5; analytic q, qd, qdd; u=t/T" and
             manifest.get("generator_code_sha256")==report.get("code_sha256",{}).get("validate_parking.py") and
             bool(re.fullmatch(r"[0-9a-f]{64}",str(manifest.get("generator_code_sha256","")))),
             f"{direction} candidate generator is not bound to original GPU producer")
    _require(manifest.get("candidate_sha256")==candidate_sha and candidate_bytes==formal_bytes,
             f"{direction} formal replay differs from immutable candidate bytes or SHA")
    _require(manifest.get("duration_s")==20. and manifest.get("sample_dt_s")==.01 and
             manifest.get("duration_ns")==20_000_000_000 and manifest.get("sample_dt_ns")==10_000_000 and
             manifest.get("point_count")==2001 and len(trajectory.samples)==2001 and trajectory.duration_s==20.,
             f"{direction} candidate is not fixed 20s/10ms/2001 points")
    _require(tuple(manifest.get("joint_names",[]))==trajectory.joint_names,
             f"{direction} candidate joint order changed")
    _positions(manifest.get("start_positions_rad"),start,direction+" candidate start",0.)
    _positions(manifest.get("target_positions_rad"),target,direction+" candidate target",0.)
    # Original absolute strings remain unchanged in the sealed report and JSON.
    # Their relationships are checked, but relocated bundles read only local copies.
    original_paths = []
    for value,name in ((manifest.get("candidate_path"),"candidate_joint_trajectory.csv"),
                       (proof.get("manifest_path"),"candidate.json"),
                       (item.get("joint_trajectory_path"),"joint_trajectory.csv")):
        _require(isinstance(value,str) and bool(value),f"{direction} missing original candidate path")
        path = Path(value)
        _require(path.is_absolute() and path.name==name and ".." not in path.parts,
                 f"{direction} original candidate path is inconsistent")
        if not bundled:
            _require(path.resolve()==_file(directory,name).resolve(),
                     f"{direction} original candidate path differs from source files")
        original_paths.append(path)
    _require(len({path.parent for path in original_paths})==1,
             f"{direction} candidate and formal source paths do not share the original directory")
    prepare = item.get("prepare_metadata",{}).get("prepare",{})
    required = {"source_kind":MINIMUM_JERK_SOURCE_KIND,"duration_s":20.,"sample_dt_s":.01,
                "duration_ns":20_000_000_000,"sample_dt_ns":10_000_000,
                "candidate_sha256":candidate_sha,"candidate_manifest_sha256":manifest_sha,
                "authority_contract":AUTHORITY_CONTRACT,"upstream_source_generated_before_validation":True}
    _require(all(prepare.get(key)==value for key,value in required.items()),
             f"{direction} prepare metadata differs from fixed upstream candidate")
    delta = tuple(b-a for a,b in zip(start,target,strict=True))
    zero = (0.,)*6
    for index,sample in enumerate(trajectory.samples):
        _require(round(sample.time_s*1e9)==index*10_000_000 and sample.phase=="approach",
                 f"{direction} candidate has an altered or near-duplicate sample clock/phase")
        if index in (0,2000):
            positions,velocities,accelerations = (tuple(start) if index==0 else tuple(target)),zero,zero
        else:
            u = index/2000
            s = u*u*u*(10.+u*(-15.+6.*u))
            ds = 30.*u*u*(1.-u)*(1.-u)/20.
            dds = 60.*u*(1.-u)*(1.-2.*u)/(20.*20.)
            positions = tuple(a+d*s for a,d in zip(start,delta,strict=True))
            velocities,accelerations = tuple(d*ds for d in delta),tuple(d*dds for d in delta)
        _require(sample.positions==positions and sample.velocities==velocities and sample.accelerations==accelerations,
                 f"{direction} candidate differs from analytic minimum-jerk v1 authority")
    return {"candidate_joint_trajectory.csv":candidate_bytes,"candidate.json":manifest_bytes}


def _validate_source(report: dict[str, Any], directory: Path, env: Any, config: dict[str, Any],
                     park: tuple[float, ...], revision: str, config_sha: str, lock_sha: str,
                     *, bundled: bool = False) -> dict[str, dict[str, Any]]:
    _require(report.get("schema_version")==1 and report.get("environment_id")=="mla_short" and
             report.get("status")=="passed" and report.get("gpu_full_path_validation")=="passed_both_directions" and
             report.get("runtime_ready") is True,"parking source is not a fully passed dual-direction GPU validation")
    _require(report.get("simulation_only") is True and report.get("real_execution_authorized") is False and
             report.get("robot_execution")=="not_requested_not_run" and report.get("ros_runtime")=="not_started",
             "source GPU evidence has unexpected hardware/runtime identity")
    _require(report.get("uniform_time_scale")==1. and report.get("authority_mutated") is False,
             "source prepare clock or authority was modified")
    _require(report.get("environment_hashes")==dict(env.hashes) and
             report.get("config_sha256")==config_sha and report.get("curobo_lock_sha256")==lock_sha,
             "GPU certificate environment/config/cuRobo lock differs")
    _require(tuple(report.get("joint_names",[]))==env.joint_names,"GPU certificate joint order changed")
    _require(report.get("hard_world_self_clearance_m")==.005 and report.get("rod_length_m")==.190 and
             report.get("rod_diameter_m")==.005,"GPU source used a different rod or clearance")
    _positions(report.get("home_joint_positions_rad"),env.home_joint_positions,"source home")
    _positions(report.get("parking_joint_positions_rad"),park,"source parking")
    _positions(report.get("parking_joint_positions_deg"),config["parking"]["joint_positions_deg"],"source parking degrees")
    _require(set(report.get("directions",{}))==set(DIRECTIONS),"GPU source must contain exactly both prepare directions")
    source_kind = report.get("source_kind",DEFAULT_SOURCE_KIND)
    _require(source_kind in {DEFAULT_SOURCE_KIND,MINIMUM_JERK_SOURCE_KIND},"unsupported parking source kind")
    results = {}
    for direction in DIRECTIONS:
        item = report["directions"][direction]
        local = directory / direction
        proof = _json(_file(local,"evidence.json"))
        _require(proof==item,f"{direction} evidence differs from top-level GPU report")
        csv_path = _file(local,"joint_trajectory.csv")
        csv_bytes = csv_path.read_bytes()
        _require(hashlib.sha256(csv_bytes).hexdigest()==item.get("joint_trajectory_sha256"),f"{direction} CSV SHA mismatch")
        _require(item.get("status")=="passed" and item.get("curobo_revision")==revision,
                 f"{direction} did not pass pinned cuRobo")
        start,target = (env.home_joint_positions,park) if direction==DIRECTIONS[0] else (park,env.home_joint_positions)
        _positions(item.get("start_positions_rad"),start,direction+" start")
        _positions(item.get("target_positions_rad"),target,direction+" target")
        trajectory = _joint_bytes(csv_bytes,env.joint_names)
        candidate_blobs = {}
        if source_kind==MINIMUM_JERK_SOURCE_KIND:
            candidate_blobs = _validate_candidate(report,item,local,trajectory,csv_bytes,start,target,direction,bundled=bundled)
        else:
            _require("upstream_candidate" not in item and
                     item.get("prepare_metadata",{}).get("prepare",{}).get("source_kind",DEFAULT_SOURCE_KIND)==DEFAULT_SOURCE_KIND,
                     f"{direction} legacy source has inconsistent candidate provenance")
        current = validate_trajectory(env,trajectory,start,target,endpoint_tolerance=1e-6)
        dynamics = item.get("dynamics",{})
        _require(dynamics.get("passed") is True and dynamics.get("uniform_time_scale")==1. and
                 dynamics.get("continuous_quintic_extrema_checked") is True and
                 dynamics.get("point_count")==len(trajectory.samples) and
                 dynamics.get("duration_s")==trajectory.duration_s and
                 dynamics.get("limits")==current["limits"],f"{direction} lacks exact-clock continuous dynamics proof")
        for key in ("max_joint_velocity_ratio","max_joint_acceleration_ratio","max_joint_jerk_ratio"):
            _require(0<=_number(dynamics.get(key),key)<=1.+1e-7,f"{direction} source dynamics failed")
        for label in ("gpu_metrics","serialized_gpu_metrics"):
            _gpu_metrics(item.get(label,{}),len(trajectory.samples),direction+" "+label)
        cpu = item.get("cpu_sampled_geometry",{})
        _require(cpu.get("status")=="CPU_sampled_passed" and cpu.get("sample_count")==len(trajectory.samples) and
                 cpu.get("environment_hashes")==dict(env.hashes),f"{direction} source CPU scene evidence differs")
        prepare = item.get("prepare_metadata",{}).get("prepare",{})
        _require(prepare.get("curobo_revision")==revision and prepare.get("uniform_time_scale")==1.,
                 f"{direction} missing fixed-clock prepare provenance")
        results[direction] = {"trajectory":trajectory,"proof":item,"current_dynamics":current,
                              "candidate_blobs":candidate_blobs}
    return results


def _producer_source(root: Path, name: str, expected: str) -> Path:
    _require(bool(re.fullmatch(r"[0-9a-f]{64}",expected)),"invalid producer source digest")
    for candidate in (name,PRODUCER_ALIASES[name]):
        path = root/candidate
        if path.is_file() and not path.is_symlink() and sha256(path)==expected:
            return path
    raise ValueError(f"no producer source snapshot matches original GPU certificate: {name}")


def create_bundle(workspace: Path | str, validation_path: Path | str, output: Path | str,
                  config_path: Path | str | None = None, producer_code_root: Path | str | None = None) -> dict[str, Any]:
    workspace, validation_path, output = Path(workspace).resolve(),Path(validation_path).resolve(),Path(output).resolve()
    config_path = Path(config_path or workspace/"collection/config.yaml").resolve()
    source_code = Path(producer_code_root or Path(__file__).parent).resolve()
    env,config,park,revision = _environment(workspace,config_path)
    config_sha,lock_sha = sha256(config_path),sha256(workspace/"curobo.lock")
    report = _json(validation_path)
    validated = _validate_source(report,validation_path.parent,env,config,park,revision,config_sha,lock_sha)
    _require(set(report.get("code_sha256",{}))==set(PRODUCER_ALIASES),"GPU report lacks exact producer code set")
    sources = {name:_producer_source(source_code,name,digest) for name,digest in report["code_sha256"].items()}
    blobs: dict[str,bytes] = {"source/validation.json":validation_path.read_bytes(),
                              "source/config.yaml":config_path.read_bytes(),
                              "source/curobo.lock":(workspace/"curobo.lock").read_bytes()}
    for name,path in sources.items():
        blobs[f"producer_code/{name}"] = path.read_bytes()
    for name in CONSUMER_FILES:
        blobs[f"consumer_code/{name}"] = Path(__file__).with_name(name).read_bytes()
    direction_rows = {}
    for direction,value in validated.items():
        source_dir = validation_path.parent/direction
        for name in ("evidence.json","joint_trajectory.csv"):
            blobs[f"source/{direction}/{name}"] = _file(source_dir,name).read_bytes()
        for name,data in value["candidate_blobs"].items():
            blobs[f"source/{direction}/{name}"] = data
        # Copy the exact q/qd/qdd/t bytes as the directly consumable asset.
        blobs[f"{direction}/joint_trajectory.csv"] = blobs[f"source/{direction}/joint_trajectory.csv"]
        t = value["trajectory"]
        direction_rows[direction] = {"joint_trajectory_file":f"{direction}/joint_trajectory.csv",
            "joint_trajectory_sha256":value["proof"]["joint_trajectory_sha256"],
            "source_evidence_file":f"source/{direction}/evidence.json",
            "duration_s":t.duration_s,"point_count":len(t.samples),
            "start_positions_rad":list(t.samples[0].positions),"target_positions_rad":list(t.samples[-1].positions),
            "gpu_validation_status":"passed", "fixed_clock":True,"uniform_time_scale":1.}
    manifest = {"schema_version":1,"asset_type":ASSET_TYPE,"authority_contract":AUTHORITY_CONTRACT,
        "created_utc":utc_now(),"environment_id":"mla_short","environment_hashes":dict(env.hashes),
        "config_sha256":config_sha,"curobo_lock_sha256":lock_sha,"curobo_revision":revision,
        "joint_names":list(env.joint_names),"home_joint_positions_rad":list(env.home_joint_positions),
        "parking_joint_positions_rad":list(park),"hard_world_self_clearance_m":.005,
        "rod_length_m":.190,"rod_diameter_m":.005,"tipdraw_recording":False,
        "simulation_only":True,"real_execution_authorized":False,"robot_execution":"not_run",
        "mock_execution":"not_certified_by_this_bundle", "directions":direction_rows,
        "source_kind":report.get("source_kind",DEFAULT_SOURCE_KIND),
        "source_validation":{"file":"source/validation.json","sha256":sha256(validation_path)},
        "producer_code_sha256":dict(report["code_sha256"]),
        "consumer_code_sha256":{name:sha256(Path(__file__).with_name(name)) for name in CONSUMER_FILES},
        "code_provenance":"original GPU producer snapshots are preserved; separately versioned consumer never claimed to have produced that run",
        "files":{name:hashlib.sha256(data).hexdigest() for name,data in blobs.items()}}
    output.mkdir(parents=True,exist_ok=False)
    for name,data in blobs.items():
        path = output/name
        path.parent.mkdir(parents=True,exist_ok=True)
        with path.open("xb") as stream:
            stream.write(data)
    write_json_new(output/"bundle.json",manifest)
    seal = {**manifest["files"],"bundle.json":sha256(output/"bundle.json")}
    with (output/"sha256sums.txt").open("x") as stream:
        stream.writelines(f"{digest}  {name}\n" for name,digest in sorted(seal.items()))
    PreparedParkingBundle(workspace,output,config_path).verify()
    return manifest


class PreparedParkingBundle:
    def __init__(self, workspace: Path | str, bundle_dir: Path | str,
                 config_path: Path | str | None = None):
        self.workspace,self.root = Path(workspace).resolve(),Path(bundle_dir).resolve()
        self.config_path = Path(config_path or self.workspace/"collection/config.yaml").resolve()
        self.manifest_path = _file(self.root,"bundle.json")
        self.manifest = _json(self.manifest_path)
        self.manifest_sha256 = sha256(self.manifest_path)
        self.seal_sha256 = sha256(_file(self.root,"sha256sums.txt"))
        self.verify()

    def verify(self) -> None:
        _require(sha256(_file(self.root,"bundle.json"))==self.manifest_sha256 and
                 sha256(_file(self.root,"sha256sums.txt"))==self.seal_sha256,"prepare manifest or seal changed after loading")
        m = _json(self.manifest_path)
        _require(m==self.manifest and m.get("schema_version")==1 and m.get("asset_type")==ASSET_TYPE and
                 m.get("authority_contract")==AUTHORITY_CONTRACT and m.get("environment_id")=="mla_short",
                 "unsupported or modified prepare bundle")
        _require(m.get("tipdraw_recording") is False and m.get("real_execution_authorized") is False and
                 m.get("simulation_only") is True and m.get("robot_execution")=="not_run",
                 "prepare assets must not impersonate sealed recordings or real release")
        _require(m.get("hard_world_self_clearance_m")==.005 and m.get("rod_length_m")==.190 and
                 m.get("rod_diameter_m")==.005,"bundle rod/clearance metadata changed")
        env,config,park,revision = _environment(self.workspace,self.config_path)
        _require(m.get("environment_hashes")==dict(env.hashes) and m.get("config_sha256")==sha256(self.config_path) and
                 m.get("curobo_lock_sha256")==sha256(self.workspace/"curobo.lock") and m.get("curobo_revision")==revision,
                 "prepare bundle environment/config/cuRobo lock changed")
        _positions(m.get("home_joint_positions_rad"),env.home_joint_positions,"bundle home")
        _positions(m.get("parking_joint_positions_rad"),park,"bundle parking")
        _require(tuple(m.get("joint_names",[]))==env.joint_names and set(m.get("directions",{}))==set(DIRECTIONS),
                 "prepare bundle joint order/direction set differs")
        _require(m.get("consumer_code_sha256")=={name:sha256(Path(__file__).with_name(name)) for name in CONSUMER_FILES},
                 "prepare consumer code version changed; create a new versioned bundle")
        files = m.get("files",{})
        _require(isinstance(files,dict) and files,"prepare bundle has no sealed file set")
        for name,digest in files.items():
            _require(sha256(_file(self.root,name))==digest,f"prepare bundle file SHA mismatch: {name}")
        _require(sha256(_file(self.root,"source/config.yaml"))==m["config_sha256"] and
                 sha256(_file(self.root,"source/curobo.lock"))==m["curobo_lock_sha256"],"source config/lock snapshot differs")
        seal = {}
        for line in _file(self.root,"sha256sums.txt").read_text().splitlines():
            digest,sep,name = line.partition("  ")
            _require(bool(sep) and bool(re.fullmatch(r"[0-9a-f]{64}",digest)) and name not in seal,
                     "invalid or duplicate prepare checksum row")
            seal[name] = digest
        _require(seal=={**files,"bundle.json":self.manifest_sha256},"prepare seal differs from exact manifest file set")
        report_path = _file(self.root,m["source_validation"]["file"])
        _require(sha256(report_path)==m["source_validation"]["sha256"],"original GPU report changed")
        report = _json(report_path)
        _require(m.get("source_kind",DEFAULT_SOURCE_KIND)==report.get("source_kind",DEFAULT_SOURCE_KIND),
                 "bundle source kind differs from original GPU report")
        _require(m.get("producer_code_sha256")==report.get("code_sha256") and
                 set(m.get("producer_code_sha256",{}))==set(PRODUCER_ALIASES),"original producer code provenance changed")
        for name,digest in m["producer_code_sha256"].items():
            _require(sha256(_file(self.root,f"producer_code/{name}"))==digest,"producer snapshot does not match GPU certificate")
        for name,digest in m["consumer_code_sha256"].items():
            _require(sha256(_file(self.root,f"consumer_code/{name}"))==digest,"consumer snapshot changed")
        validated = _validate_source(report,self.root/"source",env,config,park,revision,
                                     m["config_sha256"],m["curobo_lock_sha256"],bundled=True)
        for direction,row in m["directions"].items():
            t = validated[direction]["trajectory"]
            for name,data in validated[direction]["candidate_blobs"].items():
                _require(files.get(f"source/{direction}/{name}")==hashlib.sha256(data).hexdigest(),
                         "candidate provenance files are not part of the exact sealed bundle")
            expected_file = f"{direction}/joint_trajectory.csv"
            _require(row.get("joint_trajectory_file")==expected_file and
                     row.get("joint_trajectory_sha256")==validated[direction]["proof"]["joint_trajectory_sha256"] and
                     sha256(_file(self.root,expected_file))==row["joint_trajectory_sha256"] and
                     _file(self.root,expected_file).read_bytes()==_file(self.root,f"source/{direction}/joint_trajectory.csv").read_bytes(),
                     "directly playable prepare bytes differ from GPU source")
            _require(row.get("duration_s")==t.duration_s and row.get("point_count")==len(t.samples) and
                     row.get("fixed_clock") is True and row.get("uniform_time_scale")==1.,"prepare clock/count metadata changed")
            _positions(row.get("start_positions_rad"),t.samples[0].positions,direction+" asset start",0.)
            _positions(row.get("target_positions_rad"),t.samples[-1].positions,direction+" asset target",0.)

    def trajectory(self, direction: str):
        trajectory,evidence,_ = self.load_asset(direction)
        return trajectory,evidence

    def load_asset(self, direction: str):
        """Bind the parsed payload and copied evidence to one immutable byte snapshot."""
        _require(direction in DIRECTIONS,"unknown prepared parking direction")
        self.verify()
        row = self.manifest["directions"][direction]
        data = _file(self.root,row["joint_trajectory_file"]).read_bytes()
        _require(hashlib.sha256(data).hexdigest()==row["joint_trajectory_sha256"],"prepare bytes changed while loading")
        trajectory = _joint_bytes(data,tuple(self.manifest["joint_names"]))
        return trajectory,{**row,"asset_type":ASSET_TYPE,"bundle_path":str(self.root),
            "bundle_manifest_sha256":self.manifest_sha256,"source_validation_sha256":self.manifest["source_validation"]["sha256"],
            "curobo_revision":self.manifest["curobo_revision"],"gpu_validation_status":"passed",
            "producer_code_sha256":self.manifest["producer_code_sha256"],
            "consumer_code_sha256":self.manifest["consumer_code_sha256"]},data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command",required=True)
    create = sub.add_parser("create")
    create.add_argument("--workspace",type=Path,default=Path(__file__).resolve().parents[1])
    create.add_argument("--validation",type=Path,required=True)
    create.add_argument("--output",type=Path,required=True)
    create.add_argument("--config",type=Path)
    create.add_argument("--producer-code-root",type=Path,help="matching original GPU producer files, or *_source_at_launch.py snapshots")
    args = parser.parse_args()
    value = create_bundle(args.workspace,args.validation,args.output,args.config,args.producer_code_root)
    print(json.dumps({"asset_type":value["asset_type"],"bundle":str(args.output.resolve()),"directions":list(value["directions"]),
                      "tipdraw_recording":False,"real_execution_authorized":False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
