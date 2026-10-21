#!/usr/bin/env python3
"""Capture read-only Ubuntu24 runtime provenance without starting ROS nodes.

The only remote commands are shell setup, Python inspection, git, and readelf.
No remote file is created, changed, built, installed, or executed as a ROS node.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import shlex
import subprocess
from pathlib import Path


REMOTE = r'''
import base64, datetime, hashlib, importlib.util, json, os, pathlib, subprocess, sys
sys.dont_write_bytecode = True
parent = pathlib.Path("/home/huaxin/Codes/ur-waypoint")
workspace = parent / "workstation_magtrack"
os.environ["GIT_OPTIONAL_LOCKS"] = "0"
packages = ("hx_waypoint_curobo", "hx_waypoint_tools", "hx_waypoint_runtime", "hx_waypoint_interfaces", "hx_waypoint_bringup")
suffixes = {".py", ".cpp", ".hpp", ".h", ".action", ".srv", ".msg", ".yaml", ".yml", ".xml", ".cfg", ".cmake", ".lock"}
def command(args):
    p = subprocess.run(args, cwd=parent, capture_output=True, text=True, check=False)
    return {"argv": args, "returncode": p.returncode, "stdout": p.stdout, "stderr": p.stderr}
def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1048576), b""):
            h.update(block)
    return h.hexdigest()
files, payloads = [], {}
def capture(path, kind, copy=True, logical=None):
    if not path.is_file():
        files.append({"path": str(path), "kind": kind, "status": "unavailable"})
        return
    resolved = path.resolve()
    stat = resolved.stat()
    name = logical or str(path.relative_to(parent))
    row = {"path": str(path), "resolved_path": str(resolved), "logical_path": name,
           "kind": kind, "size_bytes": stat.st_size, "sha256": sha(resolved),
           "mtime_utc": datetime.datetime.fromtimestamp(stat.st_mtime, datetime.timezone.utc).isoformat(),
           "copied": bool(copy)}
    if copy:
        if stat.st_size > 8 * 1024 * 1024:
            raise RuntimeError("source capture size bound exceeded: " + str(path))
        row["snapshot_path"] = "runtime_sources/" + name
        payloads[row["snapshot_path"]] = base64.b64encode(resolved.read_bytes()).decode()
    files.append(row)
origins = {}
for name in ("hx_waypoint_curobo", "hx_waypoint_tools", "hx_waypoint_interfaces"):
    spec = importlib.util.find_spec(name)
    origins[name] = {"origin": None if spec is None else spec.origin,
                     "search_locations": [] if spec is None else list(spec.submodule_search_locations or [])}
    if spec is None or not spec.origin:
        continue
    package_dir = pathlib.Path(spec.origin).parent
    for path in sorted(package_dir.rglob("*")):
        if "__pycache__" in path.parts or not path.is_file():
            continue
        if path.suffix == ".py":
            capture(path, "actual_python_import_package")
        elif path.suffix == ".so":
            capture(path, "installed_generated_interface_extension", copy=False)
for name in packages:
    package_dir = parent / "src" / name
    for path in sorted(package_dir.rglob("*")):
        if "__pycache__" in path.parts or not path.is_file():
            continue
        if path.suffix in suffixes or path.name in ("CMakeLists.txt", "requirements.txt"):
            capture(path, "current_parent_source_tree_not_necessarily_installed")
installed_bringup = parent / "install" / "hx_waypoint_bringup" / "share" / "hx_waypoint_bringup"
for name in ("launch", "config"):
    for path in sorted((installed_bringup / name).rglob("*")):
        if path.is_file() and path.suffix in (".py", ".yaml"):
            capture(path, "installed_bringup_launch_or_config")
binary = parent / "install" / "hx_waypoint_runtime" / "lib" / "hx_waypoint_runtime" / "runtime_node"
capture(binary, "actual_installed_cpp_runtime_binary", copy=False)
for path in sorted((parent / "install" / "hx_waypoint_interfaces" / "lib").glob("*.so")):
    capture(path, "installed_generated_interface_library", copy=False)
for path in sorted((workspace / "runtime").glob("*")):
    if path.is_file() and path.suffix in (".py", ".sh"):
        capture(path, "actual_workspace_batch_orchestration")
for path in (workspace / "curobo.lock", workspace / "planning" / "frozen_nominal_snapshot_v4.json",
             pathlib.Path("/opt/ros/jazzy/setup.bash"), parent / "install" / "setup.bash", parent / ".venv" / "pyvenv.cfg"):
    capture(path, "runtime_selection_or_version_reference", copy=path.is_relative_to(workspace), logical=("external/" + str(path).lstrip("/")) if not path.is_relative_to(parent) else None)
dirty_paths = ["src/" + name for name in packages]
patch = command(["git", "diff", "--no-ext-diff", "--binary", "HEAD", "--", *dirty_paths])
status = command(["git", "status", "--porcelain=v1", "--untracked-files=all", "--", *dirty_paths])
comparisons = []
for row in files:
    if row.get("kind") != "actual_python_import_package":
        continue
    p = pathlib.Path(row["path"])
    package_name = next((name for name in ("hx_waypoint_curobo", "hx_waypoint_tools") if name in p.parts), None)
    if package_name is None:
        continue
    source = parent / "src" / package_name / package_name / p.name
    if source.is_file():
        comparisons.append({"installed_path": str(p), "source_path": str(source),
                            "installed_sha256": row["sha256"], "source_sha256": sha(source),
                            "identical": row["sha256"] == sha(source)})
dep_lock = workspace / "curobo.lock"
dep_commit = next((line.split(":", 1)[1].strip() for line in dep_lock.read_text().splitlines() if line.startswith("commit:")), None)
dep_path = parent / ".deps" / ("curobo-" + str(dep_commit))
dependency = {"expected_lock_commit": dep_commit, "source_path": str(dep_path), "copied": False,
              "git_head": command(["git", "-C", str(dep_path), "rev-parse", "HEAD"]),
              "dirty_tracked_status": command(["git", "-C", str(dep_path), "status", "--porcelain=v1", "--untracked-files=no"])}
dependency["locked_curobo_init_sha256"] = sha(dep_path / "curobo" / "__init__.py") if (dep_path / "curobo" / "__init__.py").is_file() else None
result = {"schema_version": 1,
          "captured_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
          "host_alias": "ubuntu24", "parent_workspace": str(parent),
          "python_executable": sys.executable, "python_version": sys.version,
          "git_head": command(["git", "rev-parse", "HEAD"]),
          "relevant_git_status": status, "relevant_tracked_diff": patch,
          "package_import_origins": origins, "files": files,
          "installed_vs_current_source_comparison": comparisons,
          "cpp_binary_build_id": command(["readelf", "-n", str(binary)]),
          "curobo_dependency": dependency,
          "remote_mutation_policy": "read_only_no_ROS_nodes_no_build_no_install_no_devices",
          "limitations": [
              "Package import paths identify the effective package roots, not a per-process historical import trace.",
              "Capture occurs during the batch; it is not proof that every file was identical before the batch started.",
              "The installed C++ binary is fingerprinted, not copied; its rebuild correspondence to current source is unproven.",
              "Source dirty diffs exclude unrelated packages and environment/configuration secrets.",
              "PyTorch, cuRobo dependencies, ROS and system shared libraries are not vendored; version/lock references remain necessary.",
              "Untracked relevant source files are copied but cannot appear in a git tracked diff."
          ]}
print(json.dumps({"manifest": result, "payloads": payloads}))
'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="ubuntu24")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    runtime_setup = (
        "set -e; source /opt/ros/jazzy/setup.bash; "
        "source /home/huaxin/Codes/ur-waypoint/.venv/bin/activate; "
        "source /home/huaxin/Codes/ur-waypoint/install/setup.bash; "
        "export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 GIT_OPTIONAL_LOCKS=0; exec python -B -"
    )
    command = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", args.host,
               "bash -lc " + shlex.quote(runtime_setup)]
    result = subprocess.run(command, input=REMOTE, text=True, capture_output=True, check=True, timeout=120)
    remote = json.loads(result.stdout)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for relative, payload in remote["payloads"].items():
        target = output / relative
        if not target.is_relative_to(output):
            raise ValueError("invalid source snapshot path")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(base64.b64decode(payload, validate=True))
    manifest = remote["manifest"]
    patch = manifest.pop("relevant_tracked_diff")
    manifest["tracked_diff_file"] = "parent_relevant_dirty.patch"
    patch_bytes = patch["stdout"].encode()
    manifest["tracked_diff_sha256"] = hashlib.sha256(patch_bytes).hexdigest()
    manifest["tracked_diff_command"] = {k: v for k, v in patch.items() if k != "stdout"}
    (output / "parent_relevant_dirty.patch").write_bytes(patch_bytes)
    (output / "runtime_fingerprint.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    rows = []
    for row in manifest["files"]:
        if row.get("copied"):
            snapshot_path = output / row["snapshot_path"]
            if hashlib.sha256(snapshot_path.read_bytes()).hexdigest() != row["sha256"]:
                raise RuntimeError("snapshot hash mismatch: " + str(snapshot_path))
            rows.append(f"{row['sha256']}  {row['snapshot_path']}")
    (output / "runtime_sources.sha256").write_text("\n".join(rows) + "\n")
    differences = [row for row in manifest["installed_vs_current_source_comparison"] if not row["identical"]]
    print(json.dumps({"captured_files": len(rows), "fingerprinted_files": len(manifest["files"]),
                      "installed_python_vs_source_differences": len(differences),
                      "parent_commit": manifest["git_head"]["stdout"].strip(),
                      "output": str(output)}, indent=2))


if __name__ == "__main__":
    main()
