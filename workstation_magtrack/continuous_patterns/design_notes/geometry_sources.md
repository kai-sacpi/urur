# Continuous-pattern geometry v1

This document describes `src/patterns.py`, a geometry-only module for the new
`continuous_patterns/` batch. It does not define CSV timing, metric height,
orientation, rod length, collision thresholds, or robot release status.
Existing authorities were read and were not modified.

## Sources read

| Source | SHA256 at inspection |
|---|---|
| `/Users/huaxin/Codes/ur-waypoint/src/hx_waypoint_tools/hx_waypoint_tools/trajectory_generator.py` | `3e9c570224612edd8330915c75dc531a35a9bd964425523ca0bdc3dbca9f8225` |
| `/Users/huaxin/Codes/paper_test/BAFE-code-trajectory-perf/experiments/simulation_mechanism/motions.py` | `1b618ab100213727ec7abdc26c2697b1cb8a19d024f0c7cceb41fc2f1e983532` |
| `/Users/huaxin/Codes/ur-waypoint/scripts/generate_new_board_esophageal_up50_trajectories.py` | `fd28701ed21310ea6d414537a045d8e3275af520468d49a0006ce67d5ac00991` |

The third file is an entry point that delegates to
`generate_new_board_esophageal_up50_trajectories` in `trajectory_generator.py`.
The actual bent-tube formulas are therefore read from that generator, notably
`new_board_esophageal_centerline_position` (line 856),
`new_board_esophageal_geometry_position` (line 881), and
`_esophageal_startup_centerline_offsets` (line 937).

Reusable geometry in the first source includes:

- `_bafe_local_shape` (line 661): 1:4:5 wave, elliptic helical projection,
  third-harmonic rounded square, and phase-shifted radial breathing.
- `_candidate_raw_curve` (line 1117): trefoil, spirograph, 2:5 torus-knot,
  spherical latitude oscillation, harmonograph, Celtic weave, braided helix.
- `new_board_continuous_geometry_position` (line 1385): 2:3 Lissajous,
  perturbed figure eight, and multiple phase frequencies.

The simulation source uses `_sample_family_parameters` (line 1072) for seeded
size/aspect/phase/radial/depth variation and `_evaluate_family_shape` (line 1236)
for vector derivatives through order three. Its wave, rounded square, and
spiral product rules motivated explicit analytic derivatives and persisted
parameters here. Existing sampling bounds and hardware dimensions are not
copied into the new module.

Two legacy formulas would not alone establish geometric richness: the old
`superformula_flower` has `n2=n3=2`, so its radial term is exactly
`cos² + sin² = 1`; the XY part of `viviani` is a circle traversed twice. Neither
was promoted into the new XY catalog. The new names explicitly denote
projections where appropriate and do not claim a knot topology in 3D.

## API and units

```python
FAMILY_NAMES                          # frozen tuple of ten names
sample_parameters(family, seed, variant=0)  # JSON scalar dictionary
evaluate(family, u, parameters=None, derivative=0)  # N x 2
derivative_bounds(family, parameters=None, max_order=4)  # (order+1) x 2
carrier(family, u, parameters=None, derivative=0)  # optional (N,)
carrier_derivative_bounds(family, parameters=None, max_order=4)
catalog()                             # serializable dict keyed by family
```

All geometry is dimensionless. `u` is an unwrapped cycle parameter; one period
is `u ∈ [0,1]`, and every derivative is with respect to `u`. The caller must
apply metric scale and its time chain rule. The module never clips, retimes,
uses grid-based extrema to rescale, or chooses the physical height range.
Each XY coordinate and optional depth carrier has a proved absolute bound of
one. A rotated anisotropic metric placement must propagate derivative bounds
with the componentwise absolute transformation matrix, not only rotate the
bound vector as a signed vector.

All families are periodic and analytic; derivatives through order four match
at the period seam. This geometric property alone does not create stationary
start/end conditions. The caller supplies its smooth time law and hold rows.

## Formula catalog

Let `t = 2πu + phase_rad`. The following shows the unnormalized numerators;
normalizers are analytic coefficient-sum bounds, independent of samples.
The common `depth_phase_rad` is used only in the optional carrier.

| Family | XY formula and actual geometric feature | Normalization |
|---|---|---|
| `woven_lissajous` | `x=sin(2t+φx)+a sin(5t+φs)`, `y=sin(3t)+b sin(7t+φs+0.55)`; lobes and fine crossings at independently phased frequencies | `x/(1+a)`, `y/(1+b)` |
| `trefoil_projection` | `x=sin(t)+a sin(2t)`, `y=cos(t)-a cos(2t)`; unequal counter-turning first/second harmonics | both `/(1+a)` |
| `torus_knot_2_5` | `r=1+a cos(5t)`, `(x,y)=r(cos(2t),sin(2t))`; two azimuth turns with five radial lobes and multiple scales | both `/(1+a)` |
| `hypotrochoid_3_2` | `x=a cos(3t)+b cos(2t+φp)`, `y=a sin(3t)-b sin(2t+φp)`; counter-rotating rolling-circle pen projection | both `/(a+b)` |
| `braided_spiral` | `r=1+a cos(6t)+b cos(t+φs)`, `ψ=t+c sin(2t)`, `(x,y)=r(cos ψ,sin ψ)`; slow breathing, six local braids, nonuniform angular progression | both `/(1+a+b)` |
| `spherical_sweep` | `λ=a sin(4t+φλ)+b sin(t)`, `(x,y)=cos λ (cos 3t,sin 3t)`; three turns coupled to four latitude oscillations | no rescale; `|cos λ|≤1` |
| `compound_harmonograph` | `x=sin(2t+φx)+a sin(3t+φxs)`, `y=sin(3t)+b sin(5t+φys)`; coupled-pendulum multi-frequency loops | `x/(1+a)`, `y/(1+b)` |
| `bent_tube_braid` | `x=sin t`, `y=a sin(πx)+b sin(4t+φl)+c sin(7t+φs)`; smooth axial reversal on a bent centerline with local multi-scale winding | `y/(a+b+c)` |
| `rounded_square_shear` | `x=cos t-a cos 3t+b sin 2t`, `y=sin t+a sin 3t+0.7b sin(2t+φs)`; four smooth bends with asymmetric harmonic shear | `x/(1+a+b)`, `y/(1+a+0.7b)` |
| `celtic_weave` | `x=sin(2t+φx)+a sin 7t`, `y=sin 3t+b sin(6t+φ6)`; 2:3 crossings with distinct 7th/6th harmonic weave | `x/(1+a)`, `y/(1+b)` |

This catalog spans simple Fourier sums, products, and nested trigonometric
maps. It avoids corners, piecewise linear joins, and names that disguise an
unchanged line or circle. The fixed-height group is deliberately planar,
therefore its 3D torsion is zero. Nonzero torsion, vertical reversals and the
vertical separation of XY crossings must be measured on the actual full
height trajectories after the caller adds height. A family label alone is
not evidence of nonplanarity or a topological knot.

The optional depth carrier retains each source family's harmonic structure:
4+1 (woven), 3 (trefoil), 5 (torus), 3+5 (hypotrochoid), 6+1 (braid), nested
spherical latitude, 4+1 (harmonograph), 5 plus bent-centerline depth (tube),
2+5 (square), and 4+1 (Celtic). The physical 70–200 mm policy is entirely
outside the module, and callers may omit these carriers.

## Seed and parameter variation

The fixed v1 sampler uses explicit
`Generator(PCG64(SeedSequence([seed, family_index_1based, variant])))`.
It never uses process-global RNG state or Python's randomized hash.
Each return value persists `parameter_seed`, `parameter_variant`, the global
phase, the independent depth phase, and every coefficient used by its family.
The entire dictionary should be copied into the trajectory's metadata.

| Family | Sampled coefficient ranges; phases also sampled |
|---|---|
| woven | `a=0.13..0.22`, `b=0.12..0.20` |
| trefoil | second-harmonic weight `1.8..2.2` |
| torus | tube/radius ratio `0.22..0.36` |
| hypotrochoid | major weight `1.8..2.2`, pen `1.15..1.55` |
| braid | braid `0.16..0.24`, slow radius `0.04..0.09`, angle ripple `0.035..0.075 rad` |
| spherical | main latitude `0.48..0.72 rad`, slow latitude `0.02..0.05 rad` |
| harmonograph | secondary X `0.18..0.30`, secondary Y `0.15..0.25` |
| tube | centerline `0.45..0.65`, local winding `0.23..0.33`, fine winding `0.05..0.09` |
| square | rounding `0.08..0.13`, shear `0.12..0.20` |
| Celtic | X perturbation `0.075..0.13`, sixth harmonic `0.12..0.20` |

Metric scales, XY rotation, translation, number of periods, traversal
direction, height and time are caller metadata. Reversing traversal can be
implemented by changing `u(t)` and its chain rule; the module does not mutate
any completed CSV. Retain Python/NumPy versions and hashes of generating code.
The seed guarantees the sampling recipe in the recorded environment;
cross-platform transcendental functions need not give byte-identical floats.

## Derivatives and continuous bounds

`_Jet` stores Taylor coefficients `f_n=f^(n)/n!`, for `n=0..4`.
Products use the convolution `h_n=Σ f_k g_(n-k)`. The sin/cos composition
uses the exact coupled recurrences

```
sin(f)_n =  Σ(k f_k cos(f)_(n-k)) / n
cos(f)_n = -Σ(k f_k sin(f)_(n-k)) / n,  k=1..n.
```

There is no numerical differentiation in the public API. To create a global
bound, `_BoundJet` follows the same formula tree with coefficientwise absolute
values. Addition/subtraction use the triangle inequality; multiplication uses
the nonnegative Leibniz convolution. For either trigonometric composition,
the zeroth bound is 1 and

```
B_n = Σ(k A_k B_(n-k)) / n,  k=1..n,
```

where `A_k` bounds `|f^(k)|/k!`. This bounds both sin and cos by induction.
The affine phase has `A_1=2π`, `A_n=0` for `n≥2`; its unbounded zeroth value
is unused inside sin/cos. Multiplication by `n!` recovers derivative bounds.
The envelopes can be conservative, particularly for nested maps, but their
validity does not depend on the trajectory sampling rate. Machine rounding
still requires a small outward numeric allowance in a downstream certificate.

For the optional spherical carrier, `|λ|≤a+b<π/2`, so
`|sin λ|/sin(a+b)≤1` supplies the tighter zeroth bound. The API rejects latitude
amplitude sums outside this interval and all nonfinite geometry parameters.

## Local CPU verification

Command:

```bash
python3 workstation_magtrack/continuous_patterns/src/patterns.py --self-check
```

Observed on 2026-10-16 with NumPy 2.5.3: all ten families passed. The check
compared each analytic derivative 1..4 with a centered difference of the
preceding analytic derivative at 113 probe points; the largest normalized
discrepancy was `3.0816e-9`. It checked XY and optional depth carriers on 2001
points against all continuous global bounds, checked period-seam equality for
orders 0..4, and checked repeatability and variant separation for seed
20261016. This is source/geometry evidence. It is not a collision, IK,
robot-dynamics, calibration, hardware, or playback acceptance test.
