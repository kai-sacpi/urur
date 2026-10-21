#!/usr/bin/env python3
"""Build fixed-time replay evidence on isolated Ubuntu24 mock ROS domains.

Nothing here connects to a robot or sensor. Sources are immutable inputs. Each
phase's success is recorded only after its artifacts are hashed and checked.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone


PHASES = ("compile", "build", "inspect", "play")
ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
ACTIVE_COMMANDS = set()
ACTIVE_COMMANDS_LOCK = threading.Lock()
STOP_REQUESTED = threading.Event()


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def read_ids(path):
    path = Path(path)
    if path.suffix == ".json":
        value = json.loads(path.read_text())
        if isinstance(value, dict):
            value = value.get("trajectories", value.get("recordings", value.get("items",
                    value.get("active_records", value.get("records", [])))))
        if isinstance(value, dict):
            value = list(value.values())
        result = [x if isinstance(x, str) else x["trajectory_id"] for x in value]
    elif path.suffix == ".csv":
        with path.open(newline="") as stream:
            result = [row["trajectory_id"] for row in csv.DictReader(stream)]
    else:
        result = [line.strip() for line in path.read_text().splitlines()
                  if line.strip() and not line.lstrip().startswith("#")]
    if not result or any(not ID_RE.fullmatch(x) for x in result):
        raise ValueError("IDs must be nonempty path-safe trajectory names")
    if len(set(result)) != len(result):
        raise ValueError("Duplicate trajectory ID in batch")
    return result


def source_info(path):
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError(f"Source has fewer than two rows: {path}")
    times = [float(row["time_s"]) for row in rows]
    if times[0] != 0 or any(not math.isfinite(t) for t in times):
        raise ValueError(f"Source has invalid authoritative timestamps: {path}")
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError(f"Source timestamps are not strictly increasing: {path}")
    return {"sha256": digest(path), "waypoint_count": len(rows), "duration_s": times[-1]}


def tree_hashes(path):
    return {str(item): digest(item) for item in sorted(Path(path).rglob("*")) if item.is_file()}


def checked_domain(value):
    # Reserve the two domains specifically requested for this task; domain 0 is
    # never even probed and cannot be selected through a command-line option.
    if value not in (219, 220):
        raise ValueError("Only isolated simulation domains 219 and 220 are allowed")
    return value


def domain_environment(domain, log_path):
    checked_domain(domain)
    result = dict(os.environ)
    result.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
                  ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                  ROS_LOG_DIR=str(log_path), PYTHONUNBUFFERED="1")
    # Never inherit a discovery server or an explicit remote ROS peer list.
    result.pop("ROS_DISCOVERY_SERVER", None)
    result.pop("ROS_STATIC_PEERS", None)
    return result


def group_exists(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process(process, grace_seconds=(12, 5, 2)):
    if process is None:
        return
    # start_new_session=True makes this PID the owned PGID. The launch leader
    # can exit before a stubborn child, so its return code is not group liveness.
    pgid = process.pid
    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), grace_seconds):
        process.poll()  # Reap the leader independently of descendant cleanup.
        if not group_exists(pgid):
            return
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            if not group_exists(pgid):
                return
            time.sleep(0.025)
    process.poll()


def cancel_commands():
    STOP_REQUESTED.set()
    with ACTIVE_COMMANDS_LOCK:
        commands = list(ACTIVE_COMMANDS)
    # Escalate the owned groups concurrently so cancellation is bounded by one
    # grace period even when four GPU compilers are active.
    threads = [threading.Thread(target=stop_process, args=(process,)) for process in commands]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()


def compile_many(worker, names, jobs):
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=jobs)
    futures = []
    try:
        for name in names:
            if STOP_REQUESTED.is_set():
                raise KeyboardInterrupt
            futures.append(pool.submit(worker, name))
        for future in concurrent.futures.as_completed(futures):
            future.result()
    except BaseException:
        STOP_REQUESTED.set()
        for future in futures:
            future.cancel()
        cancel_commands()
        pool.shutdown(wait=True, cancel_futures=True)
        raise
    else:
        pool.shutdown(wait=True)


def run_command(command, log_path, env, timeout):
    with Path(log_path).open("w") as output:
        output.write("COMMAND " + json.dumps(command) + "\n")
        output.flush()
        with ACTIVE_COMMANDS_LOCK:
            if STOP_REQUESTED.is_set():
                raise KeyboardInterrupt
            process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                       env=env, start_new_session=True)
            ACTIVE_COMMANDS.add(process)
        try:
            code = process.wait(timeout=timeout)
        except BaseException:
            stop_process(process)
            raise
        finally:
            with ACTIVE_COMMANDS_LOCK:
                ACTIVE_COMMANDS.discard(process)
    if code:
        tail = "\n".join(Path(log_path).read_text(errors="replace").splitlines()[-12:])
        raise RuntimeError(f"Command exited {code}; log={log_path}\n{tail}")


class State:
    def __init__(self, root, batch, ids, environment):
        self.directory = root / "runs" / batch
        self.directory.mkdir(parents=True, exist_ok=True)
        self.process_lock = (self.directory / ".coordinator.lock").open("a+")
        try:
            fcntl.flock(self.process_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self.process_lock.close()
            raise RuntimeError(f"Another coordinator already owns batch {batch}") from error
        self.path = self.directory / "status.json"
        self.lock = threading.RLock()
        self.data = json.loads(self.path.read_text()) if self.path.exists() else {
            "schema_version": 1, "created_utc": utc_now(), "workspace": str(root),
            "environment_id": environment, "trajectories": {},
        }
        if self.data["workspace"] != str(root) or self.data["environment_id"] != environment:
            raise ValueError("Batch state belongs to a different workspace/environment")
        for name in ids:
            self.data["trajectories"].setdefault(name, {"attempts": {}})
        self.save()

    def save(self):
        with self.lock:
            self.data["updated_utc"] = utc_now()
            atomic_json(self.path, self.data)
            csv_path = self.directory / "status.csv"
            temporary = csv_path.with_suffix(".csv.tmp")
            fields = ["trajectory_id", "source_sha256", "duration_s", *PHASES,
                      "recording_id", "telemetry_path", "last_error"]
            with temporary.open("w", newline="") as output:
                writer = csv.DictWriter(output, fields)
                writer.writeheader()
                for name, entry in self.data["trajectories"].items():
                    row = {"trajectory_id": name, "source_sha256": entry.get("source", {}).get("sha256"),
                           "duration_s": entry.get("source", {}).get("duration_s")}
                    row.update({phase: entry.get(phase, {}).get("status", "pending") for phase in PHASES})
                    row["recording_id"] = entry.get("build", {}).get("recording_id", "")
                    row["telemetry_path"] = entry.get("play", {}).get("telemetry_path", "")
                    row["last_error"] = entry.get("last_error", "")
                    writer.writerow(row)
            temporary.replace(csv_path)

    def begin(self, name, phase):
        with self.lock:
            entry = self.data["trajectories"][name]
            for downstream in PHASES[PHASES.index(phase)+1:]:
                stale = entry.pop(downstream, None)
                if stale is not None:
                    stale = {**stale, "invalidated_by_phase": phase, "invalidated_utc": utc_now()}
                    entry["attempts"].setdefault(downstream, []).append(stale)
            previous = entry.get(phase)
            if previous:
                entry["attempts"].setdefault(phase, []).append(previous)
            log = self.directory / f"{name}__{phase}__{time.time_ns()}.log"
            entry[phase] = {"status": "running", "started_utc": utc_now(), "log": str(log)}
            self.save()
            return log

    def finish(self, name, phase, status, **details):
        with self.lock:
            entry = self.data["trajectories"][name]
            entry[phase].update(status=status, finished_utc=utc_now(), **details)
            if status == "failed":
                entry["last_error"] = details.get("error", "unknown error")
            elif phase == "play":
                entry.pop("last_error", None)
            self.save()
        print(json.dumps({"trajectory_id": name, "phase": phase, "status": status,
                          **details}), flush=True)


class RosSession:
    def __init__(self, workspace, environment, domain, logs, startup_timeout, external=False):
        self.workspace = workspace
        self.environment = environment
        self.domain = checked_domain(domain)
        self.logs = logs
        self.timeout = startup_timeout
        self.external = external
        self.process = None
        self.output = None

    def __enter__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rcl_interfaces.srv import GetParameters
        from hx_waypoint_interfaces.action import BuildRecording, PlayRecording
        from hx_waypoint_interfaces.srv import PlanJointPath
        self.rclpy = rclpy
        os.environ.pop("ROS_DISCOVERY_SERVER", None)
        os.environ.pop("ROS_STATIC_PEERS", None)
        os.environ.update(domain_environment(self.domain, self.logs / f"ros_domain_{self.domain}"))
        rclpy.init()
        self.node = rclpy.create_node(f"wm_batch_client_{self.domain}_{os.getpid()}")
        self.build_client = ActionClient(self.node, BuildRecording, "/hx_waypoint/build_recording")
        self.play_client = ActionClient(self.node, PlayRecording, "/hx_waypoint/play_recording")
        self.plan_client = self.node.create_client(PlanJointPath, "/hx_waypoint/curobo/plan_joint_path")
        self.parameter_client = self.node.create_client(GetParameters, "/hx_waypoint_runtime/get_parameters")
        self.BuildRecording = BuildRecording
        self.PlayRecording = PlayRecording
        self.GetParameters = GetParameters
        # Refuse to attach accidentally to another task's runtime on either
        # dedicated domain; this coordinator only stops its own process group.
        for _ in range(12):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        existing = self.node.get_node_names()
        if not self.external and any(name in existing for name in ("hx_waypoint_runtime", "controller_manager", "move_group")):
            self.close()
            raise RuntimeError(f"Domain {self.domain} already has a runtime; refusing to reuse it")
        path = self.logs / f"runtime_domain_{self.domain}__{time.time_ns()}.log"
        if not self.external:
            self.output = path.open("w")
            command = ["ros2", "launch", "hx_waypoint_bringup", "simulation.launch.py", "ur_type:=ur5",
                       f"environment_id:={self.environment}", f"workspace:={self.workspace}", "launch_rviz:=false"]
            self.output.write("COMMAND " + json.dumps(command) + "\n")
            self.output.flush()
            self.process = subprocess.Popen(command, stdout=self.output, stderr=subprocess.STDOUT,
                                            start_new_session=True)
        try:
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                self.check_alive()
                if (self.build_client.server_is_ready() and self.play_client.server_is_ready()
                        and self.plan_client.service_is_ready() and self.parameter_client.service_is_ready()):
                    break
                rclpy.spin_once(self.node, timeout_sec=0.25)
            else:
                raise TimeoutError(f"Mock runtime domain {self.domain} startup timed out; {path}")
            request = GetParameters.Request()
            request.names = ["execution_mode", "environments_root", "telemetry_root"]
            response = self.wait(self.parameter_client.call_async(request), 20)
            values = [value.string_value for value in response.values]
            expected = ["simulation", str(self.workspace / "environments"), str(self.workspace / "runs")]
            if values != expected:
                raise RuntimeError(f"Runtime identity mismatch: {values!r} != {expected!r}")
            self.identity = {"domain": self.domain, "execution_mode": values[0],
                             "environments_root": values[1], "telemetry_root": values[2],
                             "runtime_log": None if self.external else str(path),
                             "launch_pid": None if self.external else self.process.pid,
                             "externally_managed": self.external,
                             "started_utc": utc_now()}
            print(json.dumps({"mock_runtime_ready": self.identity}), flush=True)
            return self
        except BaseException:
            self.close()
            raise

    def check_alive(self):
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError(f"Managed simulation exited {self.process.returncode}")

    def wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.check_alive()
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
        if not future.done():
            raise TimeoutError(f"ROS operation timed out after {timeout:.1f} s")
        result = future.result()
        if result is None:
            raise RuntimeError("ROS operation returned no result")
        return result

    def action(self, phase, name, compiled, recording_id, timeout, log_path):
        client = self.build_client if phase == "build" else self.play_client
        action_type = self.BuildRecording if phase == "build" else self.PlayRecording
        goal = action_type.Goal()
        goal.environment_id = self.environment
        goal.environment_path = str(self.workspace / "environments" / self.environment)
        if phase == "build":
            goal.trajectory_id = name
            goal.source_trajectory_path = str(compiled / "source_tip_trajectory.csv")
            goal.compiled_trajectory_path = str(compiled / "joint_trajectory.csv")
            goal.launch_rviz = False
        else:
            goal.recording_id = recording_id
            goal.auto_prepare = True
            goal.externally_prepared_start = False
        handle = None
        witness = None
        if phase == "play":
            from controller_witness import ControllerWitness
            witness = ControllerWitness(self.node, str(log_path) + ".controller_state.csv")
        with Path(log_path).open("w") as output:
            output.write(json.dumps({"runtime": self.identity, "phase": phase, "trajectory_id": name}) + "\n")
            previous = [None, 0.0]
            def feedback(message):
                item = message.feedback
                # Record each state transition and one feedback every 5 s.
                if item.state != previous[0] or time.monotonic() - previous[1] >= 5:
                    output.write(json.dumps({"utc": utc_now(), "state": item.state,
                                             "progress": item.progress, "detail": item.detail}) + "\n")
                    output.flush()
                    previous[:] = [item.state, time.monotonic()]
            try:
                handle = self.wait(client.send_goal_async(goal, feedback_callback=feedback), 30)
                if not handle.accepted:
                    raise RuntimeError(f"{phase} action rejected goal")
                wrapped = self.wait(handle.get_result_async(), timeout)
                result = wrapped.result
                details = {"action_status": int(wrapped.status), "success": bool(result.success),
                           "message": result.message, "runtime": self.identity}
                for field in ("recording_id", "recording_path", "telemetry_path"):
                    if hasattr(result, field):
                        details[field] = getattr(result, field)
                if witness is not None:
                    details["controller_witness_path"] = str(witness.path)
                output.write(json.dumps({"result": details}) + "\n")
                if wrapped.status != 4 or not result.success:
                    raise RuntimeError(f"{phase} failed: {details}")
                return details
            except BaseException:
                if handle is not None and handle.accepted:
                    try:
                        self.wait(handle.cancel_goal_async(), 5)
                    except Exception:
                        pass
                raise
            finally:
                if witness is not None:
                    witness.close()

    def close(self):
        stop_process(self.process)
        if self.output is not None:
            self.output.close()
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
            self.node = None
        if getattr(self, "rclpy", None) is not None and self.rclpy.ok():
            self.rclpy.shutdown()

    def __exit__(self, *args):
        self.close()


class Coordinator:
    def __init__(self, args):
        self.args = args
        self.root = args.workspace.expanduser().resolve()
        self.environment = self.root / "environments" / args.env
        if args.env != "mla_short" or self.root.name != "workstation_magtrack":
            raise ValueError("This task requires workstation_magtrack/environments/mla_short")
        self.ids = read_ids(args.ids_file)
        self.offline_audit_failures = 0
        self.ids = [name for name in self.ids if name not in args.exclude]
        if not self.ids:
            raise ValueError("No requested IDs remain after exclusions")
        self.state = State(self.root, args.batch, self.ids, args.env)
        self.env = domain_environment(219, self.state.directory / "ros_compile")
        environment_hashes = {str(path): digest(path) for path in sorted(self.environment.rglob("*"))
                              if path.is_file() and path.relative_to(self.environment).parts[0]
                              not in ("trajectories", "recordings")}
        environment_hashes[str(self.root / "curobo.lock")] = digest(self.root / "curobo.lock")
        previous_environment = self.state.data.get("environment_hashes")
        if previous_environment is not None and previous_environment != environment_hashes:
            raise RuntimeError("Environment changed since this batch; start a NEW batch with fresh validation")
        self.state.data["environment_hashes"] = environment_hashes
        for name in self.ids:
            info = source_info(self.environment / "trajectories" / f"{name}.csv")
            entry = self.state.data["trajectories"][name]
            if entry.get("source") not in (None, info):
                raise RuntimeError(f"Authoritative input changed for {name}; use a NEW upstream trajectory ID")
            entry["source"] = info
        self.state.save()

    def verify_compiled(self, name, path):
        import yaml
        path = Path(path).resolve()
        if path.parent != self.root / "runs":
            raise RuntimeError("Compiler artifacts must be direct children of controlled runs root")
        report = yaml.safe_load((path / "compile_report.yaml").read_text())
        info = self.state.data["trajectories"][name]["source"]
        required = {"environment_id": self.args.env, "trajectory_id": name,
                    "source_trajectory_sha256": info["sha256"], "uniform_time_scale": 1.0,
                    "source_waypoint_count": info["waypoint_count"],
                    "matched_source_waypoint_count": info["waypoint_count"],
                    "source_duration_s": info["duration_s"], "duration_s": info["duration_s"],
                    "max_source_time_error_s": 0.0}
        for key, value in required.items():
            if report.get(key) != value:
                raise RuntimeError(f"Authoritative compile contract {key}: {report.get(key)!r} != {value!r}")
        if (report.get("schema_version") != 2 or
                report.get("compiler_contract_revision") != "fixed_time_waypoint_v2"):
            raise RuntimeError("Compile report schema must be 2 and contract fixed_time_waypoint_v2")
        if digest(path / "source_tip_trajectory.csv") != info["sha256"]:
            raise RuntimeError("Compiler source copy changed")
        if digest(path / "joint_trajectory.csv") != report["joint_trajectory_sha256"]:
            raise RuntimeError("Compiled joint hash mismatch")
        for key in ("max_joint_velocity_ratio", "max_joint_acceleration_ratio", "max_joint_jerk_ratio"):
            value = float(report[key])
            if not math.isfinite(value) or value > 1.0:
                raise RuntimeError(f"Dynamics contract rejected {key}={value}")
        return {"compiled_path": str(path), "report": report,
                "artifact_hashes": {str(path / f): digest(path / f) for f in
                                    ("source_tip_trajectory.csv", "joint_trajectory.csv", "compile_report.yaml")}}

    def valid_cached(self, name, phase):
        entry = self.state.data["trajectories"][name]
        item = entry.get(phase, {})
        if item.get("status") != "passed" or self.args.force:
            return False
        if phase == "play" and (
                not item.get("controller_witness_path") or
                item.get("playback_audit", {}).get("controller_witness", {}).get("status") != "passed"):
            # A legacy playback's historical success is not the current full
            # acceptance grade. A normal resume must perform a new witnessed
            # playback instead of silently treating that result as complete.
            return False
        hashes = item.get("artifact_hashes", {})
        if not hashes or any(not Path(path).is_file() or digest(path) != sha for path, sha in hashes.items()):
            raise RuntimeError(f"Previously passed {name}/{phase} artifacts changed; refusing silent reuse")
        if phase == "compile":
            self.verify_compiled(name, item["compiled_path"])
        return True

    def compile_one(self, name):
        if STOP_REQUESTED.is_set():
            return
        if self.valid_cached(name, "compile"):
            return
        log = self.state.begin(name, "compile")
        try:
            if self.args.adopt_existing_compiled:
                for path in sorted((self.root / "runs").glob(f"{name}__compiled__*"), reverse=True):
                    try:
                        details = self.verify_compiled(name, path)
                    except (KeyError, OSError, RuntimeError, ValueError):
                        continue
                    log.write_text(f"Adopted existing compile only after full fixed-time/source/hash audit: {path}\n")
                    self.state.finish(name, "compile", "passed", adopted_existing=True, **details)
                    return
            command = ["ros2", "run", "hx_waypoint_tools", "tipdraw", "--workspace", str(self.root),
                       "build-recording", "--env", self.args.env, "--trajectory", name, "--compile-only"]
            run_command(command, log, self.env, self.args.compile_timeout)
            paths = [Path(line.strip()) for line in log.read_text(errors="replace").splitlines()
                     if line.strip().startswith(str(self.root / "runs"))]
            if not paths:
                raise RuntimeError("Compiler returned success without an artifact path")
            self.state.finish(name, "compile", "passed", **self.verify_compiled(name, paths[-1]))
        except Exception as error:
            self.state.finish(name, "compile", "failed", error=str(error))

    def inspect_one(self, name, log):
        from hx_waypoint_curobo.recording_store import RecordingInspector
        entry = self.state.data["trajectories"][name]
        path = Path(entry["build"]["recording_path"])
        result = RecordingInspector().inspect(path)
        manifest = dict(result.manifest)
        log.write_text(json.dumps(manifest, indent=2, default=str) + "\n")
        return {"recording_id": path.name, "recording_path": str(path),
                "artifact_hashes": tree_hashes(path)}

    def audit_existing_play(self, name):
        entry = self.state.data["trajectories"][name]
        previous = entry.get("play", {})
        previous_log = Path(previous.get("log", ""))
        if not previous_log.is_file():
            raise RuntimeError("No prior independent playback action log exists")
        results = [json.loads(line)["result"] for line in previous_log.read_text().splitlines()
                   if line.startswith('{"result":')]
        if not results:
            raise RuntimeError("Prior playback log has no action result evidence")
        result = results[-1]
        if result.get("action_status") != 4 or result.get("success") is not True:
            raise RuntimeError("Prior action did not succeed; cannot replace it with an offline audit")
        identity = result.get("runtime", {})
        if (identity.get("domain") != 220 or identity.get("execution_mode") != "simulation" or
                identity.get("telemetry_root") != str(self.root / "runs")):
            raise RuntimeError("Prior playback log does not prove isolated workspace simulation")
        log = self.state.begin(name, "play")
        try:
            witness_path = result.get("controller_witness_path")
            if not witness_path or not Path(witness_path).is_file():
                raise RuntimeError("rerunplayrequired: previous playback lacks coherent controller-state witness")
            from playback_audit import audit_playback
            audit = audit_playback(result["telemetry_path"], entry["build"]["recording_path"], self.environment)
            audit_path = Path(str(log) + ".audit.json")
            from controller_witness import audit_controller_witness
            audit["controller_witness"] = audit_controller_witness(
                witness_path, entry["build"]["recording_path"], self.environment)
            atomic_json(audit_path, audit)
            log.write_text(json.dumps({"offline_reaudit_of_action_log": str(previous_log)}) + "\n" +
                           json.dumps({"result": result}) + "\n")
            result.update(playback_audit=audit, playback_audit_path=str(audit_path),
                          reaudited_existing=True, original_action_log=str(previous_log),
                          artifact_hashes={**entry["inspect"]["artifact_hashes"],
                                           result["telemetry_path"]: digest(result["telemetry_path"]),
                                           str(previous_log): digest(previous_log), str(audit_path): digest(audit_path)})
            result["artifact_hashes"][witness_path] = digest(witness_path)
            self.state.finish(name, "play", "passed", **result)
        except Exception as error:
            self.state.finish(name, "play", "failed", error=str(error))

    def run_phase(self, phase):
        if phase == "audit-play":
            for name in self.ids:
                try:
                    self.audit_existing_play(name)
                except Exception as error:
                    self.offline_audit_failures += 1
                    print(json.dumps({"trajectory_id": name, "phase": phase, "status": "failed",
                                      "error": str(error)}), flush=True)
            return
        if phase == "compile":
            compile_many(self.compile_one, self.ids, self.args.jobs)
            return
        pending = []
        prerequisite = {"build": "compile", "inspect": "build", "play": "inspect"}[phase]
        for name in self.ids:
            if self.valid_cached(name, phase):
                continue
            entry = self.state.data["trajectories"][name]
            if entry.get(prerequisite, {}).get("status") != "passed":
                print(json.dumps({"trajectory_id": name, "phase": phase,
                                  "status": "blocked", "prerequisite": prerequisite}), flush=True)
                continue
            self.valid_cached(name, prerequisite)
            pending.append(name)
        if not pending:
            return
        if phase == "inspect":
            for name in pending:
                log = self.state.begin(name, phase)
                try:
                    self.state.finish(name, phase, "passed", **self.inspect_one(name, log))
                except Exception as error:
                    self.state.finish(name, phase, "failed", error=str(error))
            return
        domain = 219 if phase == "build" else 220
        with RosSession(self.root, self.args.env, domain, self.state.directory, self.args.startup_timeout,
                        external=self.args.external_runtime) as session:
            for name in pending:
                entry = self.state.data["trajectories"][name]
                log = self.state.begin(name, phase)
                try:
                    compiled = Path(entry["compile"]["compiled_path"])
                    self.verify_compiled(name, compiled)
                    result = session.action(phase, name, compiled, entry.get("build", {}).get("recording_id"),
                                            entry["source"]["duration_s"] + self.args.action_overhead, log)
                    if phase == "build":
                        path = Path(result["recording_path"])
                        if path.parent.resolve() != (self.environment / "recordings").resolve():
                            raise RuntimeError("Recording path escaped selected environment")
                        if digest(path / "source_tip_trajectory.csv") != entry["source"]["sha256"]:
                            raise RuntimeError("Sealed recording source differs from authority")
                        result["artifact_hashes"] = tree_hashes(path)
                    else:
                        path = Path(result["telemetry_path"])
                        if self.root / "runs" not in path.resolve().parents:
                            raise RuntimeError("Independent playback telemetry escaped controlled runs")
                        if not path.is_file() or path.stat().st_size == 0:
                            raise RuntimeError("Independent playback returned no telemetry evidence")
                        from playback_audit import audit_playback
                        audit = audit_playback(path, entry["build"]["recording_path"], self.environment)
                        from controller_witness import audit_controller_witness
                        audit["controller_witness"] = audit_controller_witness(
                            result["controller_witness_path"], entry["build"]["recording_path"], self.environment)
                        audit_path = Path(str(log) + ".audit.json")
                        atomic_json(audit_path, audit)
                        result["playback_audit"] = audit
                        result["playback_audit_path"] = str(audit_path)
                        result["artifact_hashes"] = {
                            **entry["inspect"]["artifact_hashes"], str(path): digest(path),
                            result["controller_witness_path"]: digest(result["controller_witness_path"]),
                            str(audit_path): digest(audit_path)}
                    self.state.finish(name, phase, "passed", **result)
                except Exception as error:
                    self.state.finish(name, phase, "failed", error=str(error))
                    session.check_alive()

    def run(self):
        phases = PHASES if self.args.phase == "all" else (self.args.phase,)
        for phase in phases:
            self.run_phase(phase)
        final = "play" if phases[-1] == "audit-play" else phases[-1]
        passed = sum(self.state.data["trajectories"][name].get(final, {}).get("status") == "passed"
                     for name in self.ids)
        print(json.dumps({"phase": final, "passed": passed, "requested": len(self.ids),
                          "status_json": str(self.state.path)}), flush=True)
        return 0 if passed == len(self.ids) and not self.offline_audit_failures else 2


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--workspace", type=Path, required=True)
    p.add_argument("--ids-file", type=Path, required=True)
    p.add_argument("--env", default="mla_short")
    p.add_argument("--batch", default="recording_validation_v1")
    p.add_argument("--phase", choices=(*PHASES, "all", "audit-play"), default="all")
    p.add_argument("--jobs", type=int, choices=(1, 2, 3, 4), default=2)
    p.add_argument("--compile-timeout", type=float, default=1800)
    p.add_argument("--startup-timeout", type=float, default=240)
    p.add_argument("--action-overhead", type=float, default=300)
    p.add_argument("--force", action="store_true", help="Create a new attempt even after success")
    p.add_argument("--exclude", action="append", default=[], help="Skip one trajectory ID; repeatable")
    p.add_argument("--adopt-existing-compiled", action="store_true",
                   help="Audit and adopt an exact existing compile before launching a new one")
    p.add_argument("--external-runtime", action="store_true",
                   help="Use a centrally managed SIMULATION runtime on the phase's isolated domain")
    return p


def main():
    args = parser().parse_args()
    if not ID_RE.fullmatch(args.batch):
        raise ValueError("Batch name must be path-safe")
    if any(not math.isfinite(x) or x <= 0 for x in
           (args.compile_timeout, args.startup_timeout, args.action_overhead)):
        raise ValueError("Timeouts must be finite and positive")
    def terminate(signum, frame):
        STOP_REQUESTED.set()
        with ACTIVE_COMMANDS_LOCK:
            commands = list(ACTIVE_COMMANDS)
        for process in commands:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    try:
        return Coordinator(args).run()
    except KeyboardInterrupt:
        print("Batch interrupted; running mock processes were stopped", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"Batch stopped: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
