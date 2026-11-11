# SEA-Stack v1.1.0 release notes

**Release date:** 13 November 2026. Changes since v1.0.0 (15 October 2026).
The v1.0.0 notes are unchanged:
[RELEASE_NOTES_v1.0.0.md](RELEASE_NOTES_v1.0.0.md).

## Highlights

- Nonlinear Froude-Krylov forces for large waves, chosen per body with the
  new `wetted_surface` key.
- An external controller interface (preview): a controller in its own
  process (Python helper provided) reads and writes named model channels
  each control step. Demos: a sphere winch and a
  motion-compensated offshore crane.
- IRF excitation and wave elevation records now respect `waves.direction`.

## New features

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
  linear gives 0.70. New demos `iea_sphere/regular_waves_nlfk` and its
  linear twin `regular_waves_lin` (H = 6 m, T = 8 s, optimum PTO; heave RAO
  0.664 against 0.693). The sphere comparison test sweeps H = 0.5 to 6 m:
  the two agree within 0.1% below 1 m and differ by 4.2% at 6 m. Theory, tests
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
- **External controller interface (preview).** A `controller_file` in the
  setup YAML attaches a controller that runs in its own process (a Python
  helper is provided; any program that speaks the wire protocol works). Every
  `control_dt` (zero-order hold) it reads body, motor, TSDA and RSDA
  channels and writes motor set-points, TSDA rest lengths and RSDA rest
  angles. The YAML is checked strictly, with suggestions for misspelt keys,
  and the start-up log lists the channels. The Python helper
  (`ControllerModule`, `ControllerHarness`) uses the standard library only
  and lets a controller be tested without SEA-Stack. Requires
  `SEASTACK_ENABLE_EXTERNAL=ON`. The interface may change in later releases.
  Guide: [CONTROLLERS.md](../extending/CONTROLLERS.md).
- **Tension-only ropes (`ropes_file`).** A setup YAML can mark TSDAs as ropes
  with tension T = max(0, EA/L0 (L − L0) + c dL/dt): they pull but never
  push, so a rope can go slack. Works in single runs and campaigns. See
  [CONTROLLERS.md](../extending/CONTROLLERS.md#tension-only-ropes-ropes_file).
- **New demos for the controller interface.**
  In `iea_sphere/controller_winch` a winch hauls in 1 m of rope and lifts
  the sphere by 0.49987 m, against 0.49985 m from the series stiffness of
  rope and waterplane.
  `crane` (preview) is a 50 m workboat with a knuckle-boom crane that
  sets a 3 t payload onto a fixed platform in irregular waves. In the 3D
  motion-compensated case the mean crane-tip error is 0.08 m and touchdown
  is at 0.07 m/s, against 1.14 m and 2.0 m/s open loop. See the
  [crane README](../../data/demos/run_seastack/crane/README.md).
- **Initial body velocity documented and tested.** The model YAML body keys
  `initial_linear_velocity` (centre of mass, world axes) and
  `initial_angular_velocity` (body axes) already worked through Chrono's
  parser but were undocumented. They are now described in
  [EXAMPLE_WORKFLOWS.md](../usage/EXAMPLE_WORKFLOWS.md#initial-body-velocity)
  and covered by `test_initial_velocity` (IEA sphere decay from 0.5 m/s at
  equilibrium: same damped period as the 1 m offset decay, within 0.03%).
- **Python tools: an MHKiT/NDBC wave-data to SEA-Stack YAML writer and an
  HDF5 results plotter, in `demos/features` (contributed by Allie Riley,
  @alliemr123, [#18](https://github.com/Project-SEA-Stack/SEA-Stack/pull/18)).**
  `MHKiT_writer.py` downloads NDBC buoy data with MHKiT and writes the hydro
  YAML wave block (regular waves use the energy-equivalent height
  Hm0/√2). The HDF5 plotter `demos/h5outputsReader.py` is now
  `demos/features/h5_reader.py` and gains `--compare` to overlay several
  runs.
  Dependencies are in `requirements.txt`. In the release package the tools
  are in `demos/features/`; in the source tree they are in
  `data/demos/run_seastack/features/`.

## Fixes

- **IRF excitation now uses the sea's wave direction.** The H5 reader kept
  only the first BEM heading of the excitation impulse response, so
  `irf_convolution` ignored `waves.direction`: an OSWEC run at 30 deg was
  identical to 0 deg. The reader now keeps every heading, and the kernel is
  interpolated between BEM headings as in the frequency-domain path. A
  direction on a BEM heading uses that kernel unchanged, so 0 deg results
  are bit-identical. At 30 deg on the OSWEC, IRF and frequency-domain
  excitation agree within 0.4% (normalised RMS of flap pitch). Both paths
  warn when `waves.direction` differs from a single-heading H5 file, and
  forcing `irf_convolution` on a sea with several directions is now a
  start-up error.
- **Wave elevation records now take a direction.** `waves.direction` [deg]
  was ignored when the waves came from an `eta_file`: the DFT import used
  0 deg and `method: irf_convolution` used the first BEM heading. Both now
  use `direction`. Cases without `direction` are unchanged. The OSWEC
  heading comparison test now includes a 30 deg record and matches the same
  sea built from components. See
  [EXAMPLE_WORKFLOWS.md](../usage/EXAMPLE_WORKFLOWS.md#wave-elevation-records-eta_file).
- **A failed simulation step now gives a non-zero exit code.** A solver
  failure or an external-module error or timeout stopped the run but
  returned exit code 0, so scripts and campaigns treated a truncated run as
  a success. The run now returns 1 and records the error.
- **External-module error messages keep their line breaks.** The protocol
  decoder turned `\n` in a module's message (e.g. a Python traceback) into
  `n`. All JSON escapes are now decoded. The protocol also gains two
  optional fields (output labels and log lines); existing modules work
  unchanged.
- **Nonlinear hydrostatics in waves now warns about its validity limit.**
  Nonlinear hydrostatics computes buoyancy against the calm waterplane, while
  wave excitation stays linear and includes the linear Froude-Krylov force on
  the mean hull. When the relative motion approaches the draft or freeboard,
  the two no longer match and a body can be thrown clear of the water. The
  run now warns at start-up when nonlinear hydrostatics is combined with
  waves, and once per body if it fully emerges or fully submerges. New demo
  `iea_sphere/regular_waves_nl` and the comparison test
  `test_compare_linear_vs_nonlinear_hs_sphere_reg` show the range of
  validity. Results for existing cases are unchanged.
- **Submerged-volume geometry fix (no effect on shipped cases).** The
  waterplane cap in the mesh submerged-volume calculation depended on the
  vertex order of each cut face. With the waterplane at z = 0, which is the
  only case nonlinear hydrostatics uses, the cap contributes nothing and
  results were correct; at any other waterplane height the volume could be
  wrong (13.9 m³ instead of 16.2 m³ for a test box). Fixed, with new unit
  tests (`test_submerged_volume`). All nonlinear-hydrostatics baselines are
  bit-identical.

## Packaging

- **Packages contain only git-tracked demo files.** Before, any untracked
  folder under `data/demos/run_seastack/` (local outputs, scratch cases)
  went into the ZIP. Build from a git checkout; outside one, packaging falls
  back to the old rule with a warning.
- **The package contents are checked automatically.** `build.ps1 -Package`
  and `build.sh --package` run `scripts/check_package_contents.py` and fail
  on a mismatch (demo allowlist, `bin/run_seastack` as the only executable,
  nothing forbidden).

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
- **The external controller interface is a preview.** Channel names, YAML
  keys and the Python API may change. Limits are listed in
  [CONTROLLERS.md](../extending/CONTROLLERS.md#limits-v11-preview).
- **Not yet done** (recorded in the theory note, section 10): the Aalborg
  University sphere decay experiment, CCP-WSI Blind Test 3 (waiting on a
  licence check), and a WEC-Sim ellipsoid demonstration.
