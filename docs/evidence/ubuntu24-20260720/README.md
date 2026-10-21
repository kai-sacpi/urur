# ubuntu24 authoritative setup evidence — 2026-07-20

This bundle records the ROS 2 Jazzy/CUDA 12.8 verification performed on
`ubuntu24:/home/huaxin/Codes/ur-waypoint` after migrating the work frame and
rod geometry to the single authoritative `environments/<id>/setup.yaml`.
It covers build/tests, live TF and scene checks, three GPU compile → mock
recording → inspect chains, and three independently launched mock playbacks.
It does **not** authorize physical-robot execution:

```yaml
deployment_allowed: false
```

No source CSV, waypoint, phase or `time_s` was changed. No trajectory was
scaled, retimed, cropped, repaired or skipped by runtime.

## Build and runtime gates

- Five ROS packages built successfully on Ubuntu 24.04.
- The two C++ GTest executables passed `70 + 13 = 83` cases. The colcon result
  summary reports `85 tests, 0 errors, 0 failures, 0 skipped`, including the
  two CTest wrapper results.
- Repository pytest passed `314/314` tests after adding the prepare CUDA-graph
  regression contract.
- The isolated runtime used `ROS_DOMAIN_ID=77`; the user's existing default
  domain launch (PID `3399607`) was never stopped.
- Live transforms matched the setup-derived values: `base → plane1`
  translation `[-0.65, 0, 0.30]`, identity `plane1 → wall_draw_frame`, and
  `tool0 → rod_tip` translation `[0, 0, 0.264]`.
- MoveIt contained the same two converted cuboids as cuRobo: `floor` at
  approximately `[0, 0, -0.1]` with dimensions `[2, 2, 0.1]`, and
  `rear_keepout` at `[-0.3, 0, 0.5]` with dimensions `[0.1, 1.2, 1.2]`.
- A BuildRecording probe passed the live setup/simulation identity gates and
  then aborted on its deliberately missing controlled input file.
- A pre-migration recording was rejected because its manifest did not bind the
  newly required `setup.yaml` hash. Runtime did not compensate it using the
  new frame or rod geometry.

## Three complete authoritative chains

| Source | Validated recording | Duration / points | Source contract | Dynamics ratio v / a / jerk | Independent playback |
|---|---|---:|---|---|---|
| `horizontal_line_mj.csv` | `horizontal_line_mj__20260720_031210` | 33 s / 3,301 | 122/122, zero time error, SHA preserved, scale 1 | `0.007107 / 0.000380 / 0.000096` | 4,125 rows; scaling min=max=1 |
| `shallow_s_mj.csv` | `shallow_s_mj__20260720_033621` | 33 s / 3,301 | 122/122, zero time error, SHA preserved, scale 1 | `0.012932 / 0.002153 / 0.000141` | 4,126 rows; scaling min=max=1 |
| `circle_mj_v2.csv` | `circle_mj_v2__20260720_040223` | 63 s / 6,301 | 122/122, zero time error, SHA preserved, scale 1 | `0.013233 / 0.006077 / 0.003639` | 7,882 rows; scaling min=max=1 |

All three recording snapshots bind the same authoritative setup SHA-256:

```text
d904c67cc30c61321b1ea0ba0b4ebee3f637122e72261a363c1dd0aee33cb804
```

For each trajectory, the source endpoint at `duration - 3 s` and the final
`hold` waypoint have identical pose components. The canonical final 3 s
contains 301 samples: the boundary sample retains phase `draw`, the following
300 samples are `hold`, and all 301 have one identical joint position vector
with velocity and acceleration exactly zero.

Each playback used a newly launched mock runtime, separate from the runtime
that built its recording. Large telemetry CSVs remain on `ubuntu24`; their
paths, SHA-256 hashes, row counts, time ranges and speed-scaling extrema are in
`run/full_acceptance/<trajectory>/playback_evidence.txt`.

## Fail-closed attempts retained

- Reusing one long-lived runtime after completing `horizontal_line_mj` caused
  the next `shallow_s_mj` prepare call to report an upstream cuRobo
  `cudaErrorIllegalInstruction`. The action failed and no recording was sealed.
  An exact two-prepare GPU reproduction showed that the second
  graph-captured MotionPlanner call failed when the interpolation horizon
  changed. Resetting seeds did not repair it; eager execution did.
- The final adapter therefore fixes low-frequency safety prepare planning to
  `use_cuda_graph=False` and warms it with graph capture disabled. This affects
  only how cuRobo executes the independent prepare computation; it does not
  modify source or canonical trajectory data. The exact reproduction then
  passed, and one live ROS service instance returned two consecutive valid
  prepare trajectories of 491 and 211 points.
- The first `circle_mj_v2` recording attempt reached controller success, but
  `sim_actual_joint.csv` ended at `63.050847624 s`. This exceeded the fixed
  50 ms completion allowance by `0.847624 ms`, so the staging artifact was
  marked failed. A fresh-runtime retry of the identical CSV passed. No
  telemetry was cropped and no threshold was relaxed.

These failures demonstrate the intended behavior: infrastructure or telemetry
violations reject the whole attempt; runtime does not alter the authoritative
trajectory to force acceptance.

## Bundle layout

```text
run/
  build_and_tests.log
  final_code_build_and_tests.log
  sequential_prepare_exact_repro.log
  sequential_prepare_no_cuda_graph_test.log
  final_code_sequential_prepare.log
  final_code_sequential_service.log
  isolated_checks.log
  old_recording_rejection.log
  hold_and_hash_evidence.txt
  recording_checksum_verification.log
  full_acceptance/<trajectory>/*
recordings/<recording-id>/
  manifest.yaml
  compile_report.yaml
  validation_report.yaml
  sha256sums.txt
  environment_snapshot/setup.yaml
tests/
  test_runtime_core.gtest.xml
  test_simulation_contract.gtest.xml
SOURCE_SHA256SUMS
```

The copied per-recording `sha256sums.txt` files are the original checksum lists
for the complete remote artifacts; large referenced CSV files intentionally
remain remote. `SOURCE_SHA256SUMS` binds the final source/configuration
worktree while excluding evidence, run and recording payload directories.
Root `SHA256SUMS` covers every compact file retained here.
