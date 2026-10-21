# ubuntu24 strict waypoint evidence — 2026-07-18/19

This bundle records the ROS 2 Jazzy/CUDA 12.8 verification performed on
`ubuntu24:/home/huaxin/Codes/ur-waypoint`. It covers the imported legacy
trajectory matrix and three complete GPU compile → mock recording → inspect →
independent mock playback chains. It does **not** cover a physical robot and
does not authorize real-hardware execution. The verified environment remained:

```yaml
deployment_allowed: false
```

## Three complete mock chains

| Authoritative source | Validated recording | Duration / canonical points | Source evidence | Independent playback |
|---|---|---:|---|---|
| `horizontal_line_mj.csv` | `horizontal_line_mj__20260718_234606` | 33 s / 3,301 | 122/122, zero time error, source SHA preserved, scale 1 | 4,126 telemetry rows; speed scaling min=max=1 |
| `shallow_s_mj.csv` | `shallow_s_mj__20260719_001742` | 33 s / 3,301 | 122/122, zero time error, source SHA preserved, scale 1 | 4,132 telemetry rows; speed scaling min=max=1 |
| `circle_mj_v2.csv` | `circle_mj_v2__20260719_003343` | 63 s / 6,301 | 122/122, zero time error, source SHA preserved, scale 1 | 7,881 telemetry rows; speed scaling min=max=1 |

Each playback used a newly launched mock runtime, separate from the managed
runtime used to build the recording. The final hold is exactly 3 s and 301
canonical points for all three recordings; position change, velocity and
acceleration are all exactly zero throughout those hold points.

`full-acceptance/<trajectory>/evidence.txt` records source/artifact hashes,
point and waypoint counts, timing, telemetry rows and speed scaling. The
corresponding `recordings/<recording-id>/` directory retains the compact sealed
reports copied from the complete remote recording. Large telemetry CSVs remain
on `ubuntu24`; their SHA-256 values and row counts are retained in the evidence
text instead of duplicating them here.

## Rejected candidates were not modified

- `ellipse_mj.csv` completed GPU compilation and controller execution twice,
  but the mock source-pose validation measured maximum position errors of
  `1.1655 mm` and `1.0631 mm`, both above the fixed `1 mm` limit. Both attempts
  were rejected. No runtime tolerance, waypoint, pose or timestamp was changed.
- The first `shallow_s_mj.csv` build exceeded the fixed completion deadline by
  `38.774 µs`. The failed attempt was retained, and an independent retry of the
  same authoritative CSV succeeded. No threshold or trajectory was changed.
- `circle_mj_v2.csv` is a separately generated 60 s minimum-jerk draw plus 3 s
  hold and was already a distinct authoritative file before execution. It is
  not runtime retiming of a rejected circle.

## Legacy `localizaiton_ros2` matrix

The first 20-case matrix used four explicitly declared test profiles based on
p10/p15 plane candidates and 120/194/217 mm rods. It produced no strict passes:

| Result class | Count | Meaning |
|---|---:|---|
| Endpoint-rest necessary-condition rejection | 8 | The original first/last timed motion cannot satisfy the required zero endpoint velocity and acceleration |
| Fixed-time acceleration or jerk rejection | 9 | IK exists, but the authoritative time axis violates configured joint dynamics |
| Continuous IK frontier rejection | 1 | No clearance-valid candidate extends the recursively reachable joint branch |
| 15-minute budget exhaustion | 2 | This run did not establish feasibility or infeasibility; it is not reported as unreachable |

`legacy-matrix/compile_matrix.tsv` binds every tested CSV SHA to its declared
plane pose and rod length. `legacy-matrix/compile_logs/` retains the exact
per-case diagnostic. These are test-profile conclusions, not recovered
historical per-XLSX calibration bindings.

The later user-provided p16 pose has the opposite plane-normal convention and
is therefore registered separately; it is never averaged with or silently
substituted for p10/p15.

## Final-code regression and batching evidence

After the pose registry, provenance gates and batched singularity scoring were
synced, the final remote worktree passed:

```text
C++ GTest: 69 + 13 = 82 passed
Python pytest: 302 passed
matrix --validate-only: 20/20 rows plus header, exit 0
```

The final code then recompiled only the three short accepted trajectories; no
long snake was run. Each resulting `joint_trajectory.csv` was byte-identical to
the one already sealed in its successful recording:

```text
horizontal_line_mj: 32b58cc3...28dce == recording
shallow_s_mj:       16635bf7...d9575 == recording
circle_mj_v2:       1f1d7501...bd9c == recording
```

The fixed-candidate GPU microbenchmark likewise did not read or execute a
trajectory. On the RTX 3090, with chunk size 2048 and three-run medians:

| Candidates | Scalar FK/SVD | Batched FK/SVD | Speedup |
|---:|---:|---:|---:|
| 128 | 77.139 ms | 0.959 ms | 80.43× |
| 512 | 305.715 ms | 1.058 ms | 288.93× |
| 2,048 | 1.226834 s | 1.726 ms | 710.95× |

All three comparisons were exactly equal (`max_abs_error=0`,
`max_rel_error=0`). Missing, malformed or non-finite Jacobians now fail closed;
the optimization does not zero the singularity cost or change candidate order.

Evidence is under `final-code/`: the regression JUnit/GTest XML and source
manifest, the three short compile logs/hash comparison, and the reproducible
microbenchmark script/report/candidates/checksums.

## Bundle layout

```text
full-acceptance/
  summary.txt
  hold_evidence.tsv
  <trajectory>/{build,inspect,play,launch,evidence}.*
  ellipse_mj/*failed* evidence
legacy-matrix/
  compile_matrix.tsv
  compile_logs/*.log
recordings/
  <recording-id>/{manifest,compile_report,validation_report,sha256sums}.yaml|txt
final-code/
  final-regression/*
  post-batch-short-compiles/*
  ik-singularity-microbenchmark/*
```

The copied `sha256sums.txt` files are the original checksum lists for the full
remote recordings; most referenced large CSVs intentionally remain remote.
`SHA256SUMS` at this bundle root covers every compact file retained locally.
