# Isolated simulation and replay validation

Run on Ubuntu24 using the parent project's existing ROS Jazzy installation. All
new source CSVs, compiler outputs, recordings, telemetry, logs and state stay in
`workstation_magtrack/`. No sensor acquisition or real robot launch is included.

Compile first to avoid GPU/CPU load disturbing strict 100 Hz mock recording:

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
bash runtime/run_batch.sh --ids-file planning/calibration_manifest.json --batch calibration_v1 --phase compile --jobs 2
bash runtime/run_batch.sh --ids-file planning/calibration_manifest.json --batch calibration_v1 --phase build
bash runtime/run_batch.sh --ids-file planning/calibration_manifest.json --batch calibration_v1 --phase inspect
bash runtime/run_batch.sh --ids-file planning/calibration_manifest.json --batch calibration_v1 --phase play
```

For a complete list, `--phase all` performs those phases in order. An IDs file can
be a JSON array of objects containing `trajectory_id`, a JSON array of strings,
a CSV with a `trajectory_id` column, or one trajectory ID per text line. Use the
same batch name when resuming. Only passed steps whose artifact hashes still
match are skipped. Rejected attempts and complete logs remain in the batch
directory. Any failed or blocked requested result produces process status 2.
`--adopt-existing-compiled` can import an earlier canary compile only after its
source hash and full fixed-time report pass the same audit; `--exclude ID` skips
a canary that is still running. Both can be repeated safely across resumes.

Detached run:

```bash
mkdir -p runs
nohup bash runtime/run_batch.sh --ids-file planning/calibration_manifest.json --batch calibration_v1 --phase all --jobs 2 > runs/calibration_v1.batch.log 2>&1 < /dev/null &
```

State and evidence:

```text
runs/<batch>/status.json
runs/<batch>/status.csv
runs/<trajectory_id>__compiled__<timestamp>/
environments/mla_short/recordings/<recording_id>/
runs/<recording_id>__<timestamp>/simulation/execution_telemetry.csv
```

Build uses domain 219. Playback starts an independent mock runtime on domain 220
after the build runtime shuts down. Both are localhost-only; a pre-existing
runtime on either domain causes refusal instead of reuse. Only coordinator-owned
process groups are stopped. Runtime identity is checked to require simulation
mode and this workspace's environment and telemetry roots.

When a coordinator deliberately owns the launches outside this script, pass
`--external-runtime` to build/play. It still requires simulation mode and the
exact workspace roots on domain 219/220, but does not stop the external launch.
The external coordinator must launch a separate fresh runtime for playback.

No source timestamp or waypoint is changed. Compiler contract revision 2,
`uniform_time_scale == 1`, every authoritative source time, source SHA-256 and
dynamics ratios are checked. `RecordingInspector` independently checks each
sealed artifact before the playback phase. Runtime retains its own collision,
controller identity, startup/home and start-state gates without threshold edits.

Independent playback produces a per-attempt `.audit.json`: endpoint and feedback
coverage, unit speed scaling, every-row FK consistency, joint tracking, and actual
rod-tip source position/orientation errors must satisfy the existing environment
limits. It also reports asynchronous JointState delivery intervals and jitter;
these arrival timestamps are not misrepresented as canonical 100 Hz samples.
Canonical recorded 100 Hz streams remain subject to the original strict
sampling/jitter/FK checks in `RecordingInspector`.

The `execution_telemetry.csv` trajectory time is cached action feedback (normally
20 Hz), while actual JointState arrivals are normally 125 Hz. Independent audit
therefore validates host-duration coverage, actual FK self-consistency, desired
FK against its cached source time, and the
sealed terminal stationary hold. Actual FK versus a cached source time is
diagnostic only, as is actual q versus cached desired q. Configured tracking
limits are checked on the coherent controller-state witness below.
`--phase audit-play` rechecks a previous successful action's CSV
without launching or moving a mock; its earlier status/log remains in history.
Coherent controller-state witness is mandatory for this recheck: legacy actions
without that evidence become `failed` with `rerunplayrequired`, so they cannot be
upgraded to the full acceptance grade by an offline audit.

New playbacks additionally save every received
`JointTrajectoryControllerState` message to `.controller_state.csv`, including
prepare rows, original controller time and raw scaling field. Offline audit
selects the last complete reference-clock episode, verifies its reference q/qd
against the exact sealed quintic payload, and checks actual joint/FK against the
source at that same controller time. Native publication rate/jitter are reported
without resampling or invented endpoint rows. The standard unscaled controller
leaves its raw `speed_scaling_factor` at default 0; this is preserved as raw data
and interpreted through the already checked unscaled-controller identity.
