# ubuntu24 pre-commit verification evidence — 2026-07-17

This directory contains the compact, auditable evidence retained from the latest Ubuntu 24.04 verification run. The validated source identity is a **pre-commit worktree**, not a Git commit. The synchronized remote workspace intentionally has no `.git` directory, so this document does not invent or attribute a commit hash.

The verification covered ROS 2 Jazzy build/tests, five cold CUDA/cuRobo compiles, mock-hardware recording, sealed-artifact inspection, and independent mock playback. It did **not** validate a physical robot or authorize real-hardware execution. The validated environment explicitly remains:

```yaml
deployment_allowed: false
```

## Source identity

Remote worktree:

```text
ubuntu24:/home/huaxin/Codes/ur-waypoint
identity: pre-commit validated worktree
remote Git metadata: absent
```

`source/precommit-code-config.sha256` contains 115 per-file SHA-256 entries covering:

- root build/dependency metadata: `.gitignore`, `README.md`, `curobo.lock`, `pyproject.toml`, and `requirements-dev.txt`;
- all regular files under `src/`, `scripts/`, and `tests/`, excluding `.DS_Store`, `__pycache__`, and `*.pyc`;
- all files under `environments/lab_wall_a/`, excluding generated recordings and `.DS_Store`.

The manifest's own SHA-256 is:

```text
71c2d330a29f4d4e12a3b4bc5c8c8bcda7a45204fcd38c11b28047459b0a4c0b
```

This per-file manifest, together with the source and environment hashes embedded in each compile report, identifies the exact tested code/configuration bytes without claiming a nonexistent commit.

## Build and tests

The latest remote build/test run completed successfully for all five ROS packages:

```text
hx_waypoint_bringup
hx_waypoint_curobo
hx_waypoint_interfaces
hx_waypoint_runtime
hx_waypoint_tools
```

Results archived here:

- C++: **42/42 GTests passed**, 0 failures, 0 disabled, 0 errors; XML time `0.52 s`, timestamp `2026-07-17T20:47:49.651+08:00`.
- Python: **187/187 pytest tests passed**, 0 failures, 0 skipped, 0 errors; JUnit time `30.138 s`, timestamp `2026-07-17T21:01:34.299352+08:00`, host `ubuntu24`.

The fresh Python JUnit XML was generated from the same pre-commit remote worktree because the only pre-existing XML was an obsolete evidence copy:

```bash
cd /home/huaxin/Codes/ur-waypoint
set +u
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
set -u
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python -m pytest -p no:cacheprovider tests -q \
  --junitxml=/tmp/ur-waypoint-pytest-precommit-20260717.xml
```

## Five cold compiles

The five independent compile directories were:

```text
/home/huaxin/Codes/ur-waypoint/runs/circle_01__compiled__20260717_204903_825470
/home/huaxin/Codes/ur-waypoint/runs/circle_01__compiled__20260717_204953_477074
/home/huaxin/Codes/ur-waypoint/runs/circle_01__compiled__20260717_205520_303589
/home/huaxin/Codes/ur-waypoint/runs/circle_01__compiled__20260717_205607_946681
/home/huaxin/Codes/ur-waypoint/runs/circle_01__compiled__20260717_205642_427249
```

Each retained `compile_report.yaml` is byte-identical, with SHA-256:

```text
862a00179699602abc43c579745fc13fda903438a147295063042a7edc505604
```

The associated large canonical trajectories were verified remotely rather than copied. All five had identical hashes:

```text
joint_trajectory.csv:      20ac53e5de79d2ad00725ef787886796b15df12cd1eb4f76a8356eec023160c0
source_tip_trajectory.csv: 5aeff5f7e67343322748d3d998fd6b5126e196d3cf866f3b34b732e32cc2a713
```

The compile report retained in the final recording is also byte-identical to all five cold reports. It records:

- duration `20.925 s`, 2,252 canonical points, and 10/10 source waypoints with zero source-time error;
- `uniform_time_scale: 1.0`;
- maximum joint velocity/acceleration/jerk ratios `0.0618369181 / 0.4654587445 / 0.7378626288`;
- maximum tip position/orientation errors `0.0005913054 m / 0.0379329830 deg`;
- minimum checked collision clearance `0.0112554828 m`;
- zero scene-collision, self-collision, and joint-bound costs;
- cuRobo revision `a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9` and explicit whole-path joint-winding evidence.

## Validated recording

Remote recording:

```text
/home/huaxin/Codes/ur-waypoint/environments/lab_wall_a/recordings/circle_01__20260717_204957
```

The project inspector accepted it as `status: validated`:

```bash
ros2 run hx_waypoint_tools tipdraw inspect \
  --env lab_wall_a \
  --recording circle_01__20260717_204957
```

The complete remote recording was checked with:

```bash
cd /home/huaxin/Codes/ur-waypoint/environments/lab_wall_a/recordings/circle_01__20260717_204957
sha256sum -c sha256sums.txt
```

All 24 sealed entries returned `OK`. The validation report records successful controller completion, 2,619 actual-joint samples, 2,619 controller samples, eight TF/joint-frame streams with 20,952 rows, no estimated missing samples, and:

```text
max controller-reference position error: 0.000960211434 rad  (limit 0.02)
max controller-reference velocity error: 0.0106048059 rad/s (limit 0.10)
max joint position tracking error:        0.0000217082823 rad (limit 0.02)
max joint velocity tracking error:        0.00537522275 rad/s (limit 0.10)
```

## Independent playback telemetry

Independent mock playback completed for the validated recording. Its telemetry remains only on `ubuntu24` and was **not copied** into this bundle:

```text
/home/huaxin/Codes/ur-waypoint/runs/circle_01__20260717_204957__20260717_205234/simulation/execution_telemetry.csv
```

Remote read-only verification found:

```text
size:                       1,747,767 bytes
data rows:                  2,619
SHA-256:                    dcdc8730f27db9d58ba148e95b9c66c779ddb2ee85b7dd0edffd0a1f4aefc6a5
trajectory_time_s range:    0.000000000 .. 20.887999967
trajectory time monotonic:  true
ROS time monotonic:         true
ROS time span:              20.943048954 s
speed_scaling minimum:      1.0
speed_scaling maximum:      1.0
```

Playback used the validated artifact and the independent prepare workflow:

```bash
ros2 run hx_waypoint_tools tipdraw play \
  --env lab_wall_a \
  --recording circle_01__20260717_204957 \
  --auto-prepare
```

These are mock-controller results only. They do not override `deployment_allowed: false` and do not constitute real-robot calibration, safety, controller, TCP/payload, or hardware-in-the-loop acceptance.

## Bundle integrity and omissions

`SHA256SUMS` covers all 12 copied remote evidence files: five cold compile reports, four recording reports, two test XMLs, and the pre-commit source/config manifest. Verify locally with:

```bash
cd docs/evidence/ubuntu24-20260717
shasum -a 256 -c SHA256SUMS
```

The recording's archived `sha256sums.txt` is the original checksum list for the complete sealed remote artifact. Large joint trajectories, simulation samples, TF/frame streams, source copies, and playback telemetry were deliberately not copied. No credentials, host keys, environment dumps, robot IP addresses, or other secrets are included.
