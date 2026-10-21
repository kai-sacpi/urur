# Phase-2 Z-stack + tilt validation evidence — 2026-08-28

This package indexes the two validated mock recordings for
`new_board_4cm_rod33365_up50_icassp_phase2`. No real-robot goal and no NUCLEO
capture were issued during this validation.

## Final v3 authority

- Sites: array center and `plane1 X=+50 mm`.
- Z-stack distances: `68,76,84,92,84,76,68 mm` from the current sensor plane.
- Tip Z: `-75,-83,-91,-99,-91,-83,-75 mm`.
- Deepest tilt set: normal, `±25 deg` about plane1 X and `±25 deg` about
  plane1 Y, returning to normal between tilts.
- Translation duration: 1.5 s minimum jerk.
- Tilt/return duration: 4.5 s minimum jerk.
- Each visit: 1 s settle command + immutable 3 s capture window.
- Duration: 105 s per trajectory; 15 windows; 931 source waypoints.
- Full-rank `d=b+Ro` design: rank 6, nominal condition number below 10.

Capture-window authority SHA-256:

```text
8e8f292aa6406a40fa5ecca5100b95baa422f09dd42127f90a7e44fac92f6959
```

See `recording_index.csv` for recording IDs, hashes and exact validation
metrics.

## Rejected candidates retained as evidence

The unversioned v1 used 30-degree tilt in 1.5 s. The authoritative dynamics
gate rejected it without retiming or scaling:

```text
velocity_ratio=0.714
acceleration_ratio=3.208
jerk_ratio=23.379
fixed-time fallback acceleration_ratio=1.407
fixed-time fallback jerk_ratio=2.623
```

The v2 candidate used 30-degree tilt in 4.5 s. After correcting the analytical
contact check so that plane alignment is required only near actual contact, v2
was rejected for a real robot-sphere clearance violation:

```text
sample=4146
sphere=4
measured=0.00483795838 m
required=0.005 m
```

The preserved v2 rejection log is:

```text
/data/ur-waypoint/icassp2027_phase2/z_stack_center_v2_compile_retry.log
SHA-256 178f11313a4306fbf7823e1a135a89e6f2d53131b51a86b2017aaef9044d3126
```

No failed candidate may be played.

## v3 validation result

Both v3 authorities completed:

```text
locked cuRobo GPU compile
mock build-recording
strict inspect
fresh independent mock playback
```

For both recordings:

```text
status=validated
duration_s=105
point_count=10501
source=931/931
source time error=0
uniform_time_scale=1
scene/self/bound collision costs=0
minimum collision distance=0.005781898275017738 m
```

The independent playback robot telemetry remains on Ubuntu24 and is mirrored
under `/data/ur-waypoint/icassp2027_phase2/evidence/`.

## Real UR5 + NUCLEO acquisition

Both validated v3 recordings were subsequently executed on the real UR5 with
the four-channel NUCLEO capture armed before controller submission. The mapped
telemetry preserves complete 192-channel groups inside the authoritative robot
playback window.
The center run performed the one permitted SYNC for the current NUCLEO boot;
the X+50 run reused that session without another handshake.

| Site | Robot rows | Complete mapped 192-channel groups | Incomplete | Sequence drops | Maximum mapping residual |
|---|---:|---:|---:|---:|---:|
| center | 13,132 | 10,505 | 0 | 0 | 6.279178 ms |
| X+50 mm | 13,126 | 10,500 | 6 | 31 | 5.406735 ms |

Both NUCLEO manifests report `status: complete`; both robot telemetry files
report speed scaling exactly 1.0 throughout. No packet was malformed. The X+50
quality flags are retained rather than hidden or interpolated; localization and
rigid-offset analysis must use only the mapped complete 192-channel groups.

The raw real-run bundle is stored at:

```text
/data/ur-waypoint/icassp2027_phase2/evidence/real_z_stack_tilt_20260828/
```

It contains both immutable run directories, logs, and `SHA256SUMS`. Exact run
IDs, quality counts and artifact hashes are recorded in `real_run_index.csv`.
