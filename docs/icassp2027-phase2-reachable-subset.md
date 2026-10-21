# ICASSP 2027 Phase-2 reachable-subset validation

This bundle is an exploratory reachability validation, not a replacement for
the 94-condition formal protocol.  It keeps the eight spatial columns

```text
C, EX+, EX−, EY+, EY−, K++, K+−, K−−
```

and removes the entire `K_NP` column.  Within those eight columns it retains
only direction groups for which one fixed q-sign/free-roll realization passed
all required heights under the current environment, rod, TCP-floor,
collision, and canonical shoulder-branch constraints.

## Counts

```text
full nominal matrix:                 94 conditions
remove K_NP column:                  83 conditions
remove unresolved direction groups:  72 conditions
direction groups retained:           20
difficult direction stacks retained: 13 × 5 = 65 conditions
horizontal controls retained:        7 conditions (z=13 cm)
```

The 72 retained conditions are the exact rows in the generated
`conditions_reachable8.csv`.  The omitted rows are never rewritten at runtime:

```text
EX_N_DIF2: 5 heights
EY_P_DIF2: 5 heights
K_NN_CTRL: 1 condition
K_NP:      the complete 11-condition spatial column
```

The resulting bundle is labelled `exploratory` and `use_in_main_statistics=false`.
Every retained row still carries the original condition hash, registration
hash, environment hash, and the selected q-sign policy.  The authority CSVs
are therefore immutable inputs for the offline GPU compile/mock/inspect run.

## Offline verification result (2026-08-29)

The dedicated Ubuntu24 mock run completed all 72 rows.  The clean, immutable
batch index is the `build_evidence_reachable8_clean` bundle (the earlier
`build_evidence_reachable8` directory is retained as superseded run history):

```text
validated or hash-recovered: 70/72
failed:                       2/72
```

Condition-level status in the clean index is:

```text
C       6 recovered
EX_N    6 recovered
EX_P   10 recovered, 1 failed
EY_N   11 recovered
EY_P    6 recovered
K_NN   10 recovered
K_PN   10 recovered, 1 failed
K_PP   11 recovered
```

The two failed rows remain failed in the build index:

```text
MAIN_EX_P_CTRL_Z13
  final constrained-path FK position error = 1.08678 mm > 1.0 mm

MAIN_K_PN_DIF2_Z12
  inspect rejected because validation_report.max_tip_source_orientation_error_deg
  does not match the sealed telemetry
```

The first failure is a hard Cartesian-tolerance rejection.  The second has a
sealed directory produced by the runtime, but it is not counted as validated
because the independent inspector rejects the telemetry/report mismatch.
No tolerance, timestamp, waypoint, collision zone, or trajectory point was
changed to make either row pass.  The mock process group was stopped after the
run and the default real-robot ROS domain was left running and untouched.

The condition-level summary is
`artifacts/phase2_reachable8_clean_condition_summary.csv` in this checkout.
The corresponding local index and manifest copies are
`artifacts/phase2_reachable8_clean_build_index.csv` and
`artifacts/phase2_reachable8_clean_build_manifest.yaml`.

Evidence files are kept under the Ubuntu24 tree:

```text
/data/ur-waypoint/icassp2027_phase2/realizations_sign_policy/
/data/ur-waypoint/icassp2027_phase2/realizations_sign_policy/conditions_reachable8.csv
/data/ur-waypoint/icassp2027_phase2/authorities_reachable8/exploratory/
/data/ur-waypoint/icassp2027_phase2/build_evidence_reachable8_clean/exploratory/
```

The clean index SHA-256 is
`5a00186942b95ea6ca5e29c5d8eb73413484700fbbfdb6f6140ae98a31ff49b0`.

## Interpretation

This validation asks whether the reachable portion of the planned spatial
matrix can be executed and sealed without changing any Cartesian point,
timestamp, orientation, or hard collision/dynamics gate.  It does not claim
that the omitted directions are theoretically absent; they remain explicit
reachability failures and would require an upstream direction redesign or a
formal protocol revision.
