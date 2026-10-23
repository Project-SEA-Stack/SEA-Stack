# SEA-Stack v1.1 release notes

**Not released.** These notes cover work since the v1.0.0 tag (15 October
2026). None of it is in the v1.0.0 release. The v1.0.0 notes stay as
published:
[RELEASE_NOTES_v1.0.0.md](RELEASE_NOTES_v1.0.0.md).

## Changes since v1.0.0

- **Nonlinear hydrostatics in waves now warns about its validity limit.**
  Nonlinear hydrostatics computes buoyancy against the calm waterplane, while
  wave excitation stays linear and includes the linear Froude-Krylov force on
  the mean hull. When the relative motion approaches the draft or freeboard,
  the two no longer match and a body can be thrown clear of the water. The
  run now warns at start-up when nonlinear hydrostatics is combined with
  waves, and once per body if it fully emerges or fully submerges. New demos
  `iea_sphere/regular_waves_nl` and `regular_waves_lin` and the comparison
  test `test_compare_linear_vs_nonlinear_hs_sphere_reg` show the range of
  validity. Results for existing cases are unchanged.
- **Nonlinear Froude-Krylov forces (per body).** Nonlinear Froude-Krylov
  integrates the static and incident-wave pressure over the hull below the
  wave surface at each step and removes the linear Froude-Krylov part from
  the BEM excitation; radiation and diffraction stay linear. Turn it on with
  `wetted_surface: instantaneous` and a body-level `mesh_file`;
  `wave_pressure` selects Airy pressure with Wheeler stretching
  (`airy_wheeler`, default) or the long-wave approximation (`long_wave`).
  On the IEA OES Task 10 sphere at
  steepness H/(gT²) = 0.01, the heave RAO lies within 1% of the range of the
  MARIN and NREL weakly nonlinear codes at all 20 points (free and optimum
  PTO); at T = 11 s with PTO it gives 0.577 against 0.575 / 0.574, where
  linear gives 0.70. New demo `iea_sphere/regular_waves_nlfk`; theory, tests
  and limits in
  [NONLINEAR_FROUDE_KRYLOV.md](../theory/NONLINEAR_FROUDE_KRYLOV.md) and
  [EXAMPLE_WORKFLOWS.md](../usage/EXAMPLE_WORKFLOWS.md#large-waves-nonlinear-froude-krylov).
- **New body key `wetted_surface` for the hull pressure model.** It says
  which hull surface the static and incident-wave pressure act on: `mean`
  (default; linear hydrostatics and excitation), `calm_water` (mesh buoyancy
  below z = 0 with linear excitation, the v1.0.0 nonlinear hydrostatics) or
  `instantaneous` (nonlinear hydrostatics and Froude-Krylov on the hull
  below the wave). One key sets both parts, so they always use the same
  surface. v1.0.0 inputs are unchanged: `hydrostatics: nonlinear` (scalar,
  block or inline form) and the deprecated global `nonlinear_hydrostatics`
  flag both mean `calm_water`, and all v1.0.0 results are unchanged. A body
  can now give `mesh_file` once at body level; if `hydrostatics.mesh_file`
  is also given, it must match. Setting both `wetted_surface` and
  `hydrostatics` on one body is an error. The start-up log states the model
  chosen for each nonlinear body. The shipped nonlinear demos use
  `wetted_surface`.
- **Submerged-volume geometry fix (no effect on shipped cases).** The
  waterplane cap in the mesh submerged-volume calculation depended on the
  vertex order of each cut face. With the waterplane at z = 0, which is the
  only case nonlinear hydrostatics uses, the cap contributes nothing and
  results were correct; at any other waterplane height the volume could be
  wrong (13.9 m³ instead of 16.2 m³ for a test box). Fixed, with new unit
  tests (`test_submerged_volume`). All nonlinear-hydrostatics baselines are
  bit-identical.

## Known issues and limitations

- **Nonlinear Froude-Krylov is weakly nonlinear.** Radiation and diffraction
  stay linear about the mean position, the incident wave is not disturbed by
  the body, and slamming or green water is not modelled. Bodies should start
  at their BEM position (a warning is logged otherwise). Eta-table waves
  (`waves.method: irf_convolution`) are not supported. Cost scales with
  mesh vertices times wave components (8064-face IEA sphere: about 0.4 ms
  per step in regular waves, 1.6 ms in a 512-component irregular sea on a
  16-core desktop, using OpenMP).
- **Wave stretching now also feeds nonlinear Froude-Krylov.** The v1.0.0
  notes say stretching affects only the wave-kinematics query API, not
  hydrodynamic forces. That remains true for linear forces. The Airy +
  Wheeler pressure of nonlinear Froude-Krylov uses the same stretching, and
  it still cannot be turned off.
- **Not yet done** (recorded in the theory note, section 10): the Aalborg
  University sphere decay experiment, CCP-WSI Blind Test 3 (waiting on a
  licence check), and a WEC-Sim ellipsoid demonstration.

## Before this can be released

- Merge onto a release branch and bump the version strings listed in
  [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md). The v1.0.0 tag is unchanged.
