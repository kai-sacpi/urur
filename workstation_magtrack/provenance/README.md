# Ubuntu24 runtime source provenance

`runtime_fingerprint.json` records the parent checkout commit, relevant dirty
status, actual Python import roots, installed C++ runtime binary SHA-256 and
ELF build ID, source/file hashes, and installed-versus-current-source comparisons.
`parent_relevant_dirty.patch` contains only the relevant five parent packages;
untracked source files are retained in `runtime_sources/` separately.

The snapshot deliberately distinguishes installed/build Python copies from the
current `src/` tree. A dirty checkout commit alone does not reproduce the runtime.
Captured source copies are checked against `runtime_sources.sha256` after copying.
Software/GPU versions and frozen environment hashes remain in the planning
snapshot, whose file hash is also captured here.

This is a read-only snapshot collected during offline batch validation, not a
historical import trace. The installed C++ executable is hashed rather than
vendored; correspondence between that executable and today's source tree is not
proved. Third-party dependencies and system shared libraries are not copied.
No claim of a complete self-contained rebuild is made.

To repeat the inspection without starting ROS nodes or accessing hardware:

```bash
cd /Users/huaxin/Codes/ur-waypoint
.venv/bin/python workstation_magtrack/provenance/capture_runtime_sources.py \
  --host ubuntu24 \
  --output workstation_magtrack/provenance
```

The collection script creates local evidence artifacts only. It reads remote
installed/source files and git metadata; it does not build, install, start a
runtime, contact the robot, or open a sensor port.
