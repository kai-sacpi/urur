# Phase-2 Z-stack + tilt calibration

## Purpose

This calibration estimates the fixed board-frame offset `b` and the rigid
magnet/tool offset `o` in

```text
p_tracker - p_rod_tip = b + R_plane1_from_rod_tip * o
```

using actual UR5 FK orientation for every accepted sample. A fixed-orientation
Z-stack alone cannot separate `b_z` from `o_z`; it only measures their combined
projection. The additional non-collinear tilt poses make the six-parameter
design full rank.

## Environment and collision contract

The only permitted environment is:

```text
new_board_4cm_rod33365_up50_icassp_phase2
```

Its current sensor plane is `plane1 z=-0.007 m`. The physical keepout covers
`z=[-0.067,+0.100] m`, and the 5 mm contact/scene boundary is `z=-0.072 m`.
Consequently, the originally suggested 60 mm point is invalid. The
authoritative distances and tip coordinates are:

| Distance from sensor | `plane1` tip Z |
|---:|---:|
| 68 mm | -75 mm |
| 76 mm | -83 mm |
| 84 mm | -91 mm |
| 92 mm | -99 mm |

No collision object other than the shifted sensor keepout differs from the
legacy new-board environment.

## Two authoritative trajectories

The experiment runs once at each XY site:

```text
center: plane1 (x,y) = (0,0) m
x50:    plane1 (x,y) = (+0.050,0) m
```

At each site the fixed-axis Z-stack is:

```text
68 -> 76 -> 84 -> 92 -> 84 -> 76 -> 68 mm
```

At the deepest 92 mm point, before returning through the stack, the tool runs:

```text
normal
-> +25 deg about plane1 X -> normal
-> -25 deg about plane1 X -> normal
-> +25 deg about plane1 Y -> normal
-> -25 deg about plane1 Y -> normal
```

Every Z translation leg is a 1.5 s quintic minimum-jerk segment. The first GPU
candidate also used 1.5 s for 30-degree tilt, but was correctly rejected for
authoritative elbow acceleration/jerk violations. The `v2` authority used 4.5 s
tilt/return legs and passed dynamics, but its -30-degree X tilt left only
4.837958 mm robot-sphere clearance against the required 5 mm. The current `v3`
therefore uses symmetric 25-degree, 4.5 s minimum-jerk tilts. No runtime
retiming, scaling, clearance reduction or waypoint removal is permitted.
The rod-tip XYZ is exactly fixed during tilt legs. Every visit contains:

```text
1 s commanded hold for settling
3 s immutable capture window
```

Each `v3` trajectory is 105 s long and has 15 capture windows. The repeated normal
poses at 92 mm are drift checks. The tilt-fit design has rank 6; the nominal
condition number is below 10.

## Generate the authorities

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

python scripts/generate_phase2_z_stack_tilt_calibration.py
```

The directory retains rejected v1/v2 files for provenance:

```text
environments/new_board_4cm_rod33365_up50_icassp_phase2/trajectories/
  z_stack_tilt_calibration_center_up50_phase2.csv
  z_stack_tilt_calibration_x50_up50_phase2.csv
  z_stack_tilt_calibration_capture_windows.csv
  z_stack_tilt_calibration_center_up50_phase2_v2.csv
  z_stack_tilt_calibration_x50_up50_phase2_v2.csv
  z_stack_tilt_calibration_capture_windows_v2.csv
```

The command writes the current candidates without overwriting differing content:

```text
environments/new_board_4cm_rod33365_up50_icassp_phase2/trajectories/
  z_stack_tilt_calibration_center_up50_phase2_v3.csv
  z_stack_tilt_calibration_x50_up50_phase2_v3.csv
  z_stack_tilt_calibration_capture_windows_v3.csv
```

The unversioned and `_v2` files are retained rejected candidates and must never
be built or played. The three `_v3` files are the current candidates. The matching
capture-window CSV is authoritative. Runtime or analysis code may not move,
extend, shorten, crop, or replace a window.

The rejected center v1 compile reported maximum ratios of `0.714` velocity,
`3.208` acceleration and `23.379` jerk; its fixed-time fallback still reported
`1.407` acceleration and `2.623` jerk. This evidence is why v2 changes the tilt
duration upstream instead of modifying the trajectory at runtime.

## Acquisition

For each trajectory, capture the complete raw NUCLEO stream and actual robot
telemetry for the whole playback. Only rows whose host-monotonic timestamps lie
inside an accepted 3 s window enter the calibration fit. Each accepted NUCLEO
group must contain all 64 three-axis nodes (192 channels). FK position and
orientation come from actual joint states, not commanded poses.

The two trajectories must use the same physical rod, magnet, TCP, sensor-board
placement, NUCLEO boot/session and frozen environment hashes. A safety or
collision failure invalidates the run and may not be repaired by modifying the
authority.

## Offline fit

For every accepted tracker/FK pair, form

```text
d_i = p_tracker_i - p_rod_tip_i
A_i = [I_3, R_i]
```

and solve the weighted least-squares system

```text
d_i = A_i [b, o]^T.
```

The validated numerical core is
`hx_waypoint_tools.z_stack_calibration.fit_rigid_offsets`. It rejects non-proper
rotations and rank-deficient actual-FK orientation sets instead of reporting a
pseudoinverse guess.

The primary rigid-offset fit uses the nine deepest normal/tilt windows at each
site. Fit each site independently first, then perform a joint shared-`b,o` fit.
The outward/return Z-stack windows are diagnostics: regress the fitted Z
residual against sensor distance and compare outward versus return values. A
significant depth slope or center/x50 disagreement means the rigid-offset model
is insufficient; it must not be hidden by changing the trajectory or runtime.
