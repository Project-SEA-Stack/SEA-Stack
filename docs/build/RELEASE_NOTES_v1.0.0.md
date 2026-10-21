# SEA-Stack v1.0.0 release notes

v1.0.0 is the v1.0.0-beta.4 feature set plus fixes that make the advertised
functionality trustworthy, installable, and defensible. It adds one opt-in
radiation option for regular waves, fixes excitation interpolation on
non-uniform BEM frequency grids, and fixes the state-space radiation fit for
finely sampled impulse responses. **State-space (`radiation.method:
state_space`) results change**, and so do the `rm3/mooring` demo results
(its wave input was truncated, see below). These fixes leave other
RIRF-convolution results on uniform BEM grids unchanged.

## Requirements

- **Chrono 10.0.0 exactly** (release tag `10.0.0`). Newer Chrono, including
  Chrono `main`, breaks the build. See [BUILD_CHRONO.md](BUILD_CHRONO.md).
- **Packages:** Windows (x64) and macOS. Linux is supported as a source build
  only (see [BUILD_MACOS.md](BUILD_MACOS.md), which covers macOS and Linux);
  no Linux package is published.
- **External PTO demos** need Python 3 on `PATH` (see `QUICKSTART.txt`).

## Changes since v1.0.0-beta.4

- **Constant-coefficient radiation for regular waves (opt-in).**
  `radiation.method: constant_coefficients` applies
  `F_rad = -A(w) xddot - B(w) xdot` with the full coupled BEM matrices at the
  wave frequency, read from `added_mass/all` and `radiation_damping/all`.
  A(w) replaces A_inf in the added-mass load (no double counting); B(w)
  replaces the RIRF convolution. Use it with coarse BEM grids, where the
  RIRF cannot reproduce A(w) and B(w). Only regular waves at a BEM grid period
  are accepted (relative tolerance `radiation.frequency_tolerance`, default
  `1e-4`); there is no interpolation or extrapolation, and other sea states,
  free decay and RIRF-only options stop the run with an explicit error. The
  default remains `rirf_convolution`. See
  [EXAMPLE_WORKFLOWS.md](../usage/EXAMPLE_WORKFLOWS.md#regular-waves-constant-coefficient-radiation)
  and the `rm3/regular_waves_cc` demo (source tree).
- **Excitation on non-uniform BEM frequency grids is now correct.**
  Frequency-domain excitation assumed evenly spaced frequencies. On grids
  that are not (for example uniform in period, T = 3..15 s), it read
  coefficients at the wrong frequency: on a 13-period grid, heave excitation
  was off by tens of percent between the end points (+68% magnitude and
  12 degrees phase at T = 8 s in one case). Such grids now use a bracket
  search. Grids evenly spaced in omega, including all shipped demo `.h5`
  files, keep the previous code path and give bit-identical results.
- **State-space radiation now fits the whole impulse response.** The fitter
  built its Hankel matrix from at most 2 x `max_hankel_size` - 1 = 399 samples
  of the RIRF, so for finely sampled kernels it saw only the start: 4 s of the
  20 s RM3 kernel (0.01 s sampling). RM3 heave was then off by up to 11% and
  PTO power by up to 22% in regular waves. Long kernels are now decimated
  uniformly so the fit spans their full duration at the same cost;
  `max_hankel_size` now sets the time resolution of the fit, not its length.
  RM3 state-space results now agree with the coupled frequency-domain
  solution within 0.1% (heave) and 0.2% (power). The bundled sphere and OSWEC
  state-space results move 4-12x closer to RIRF convolution (OSWEC decay: from
  45% to 3.6% maximum difference), and their regression baselines (and the
  two sphere state-space demo `expected/baseline.h5` files) were updated.
  RIRFs of at most 399 samples are fitted exactly as before.
- **New warnings for silently reduced inputs.** Eta-file imports now warn when
  the component cap (`waves.discretization.n_omega`, default 1000 Fourier
  bins) drops bins, and report the fraction of the measured variance retained;
  previously a record longer than about 1000 s lost most of its energy
  silently. Frequency-domain excitation now warns when wave components lie
  outside the BEM frequency range (their coefficients are clamped to the
  nearest grid end) and reports the share of wave energy affected.
- **Submerged-volume geometry fix (no effect on shipped cases).** The
  waterplane cap in the mesh submerged-volume calculation depended on the
  vertex order of each cut face. With the waterplane at z = 0, which is the
  only case nonlinear hydrostatics uses, the cap contributes nothing and
  results were correct; at any other waterplane height the volume could be
  wrong (13.9 m^3 instead of 16.2 m^3 for a test box). Fixed, with new unit
  tests (`test_submerged_volume`). All nonlinear-hydrostatics baselines are
  bit-identical.
- **RM3 mooring demo now uses its full wave record.** Its 3600 s eta file has
  3597 Fourier bins, but the demo capped the import at 1000, so every 4th bin
  was kept and only 24% of the measured wave variance reached the model (the
  reconstructed elevation correlated 0.41 with the record). The demo now sets
  `nfrequencies: 4000` (correlation 0.97), so its results change and its
  `expected/baseline.h5` was regenerated. The WEC-Sim/MoorDyn verification
  test and the RM3 mooring benchmark read the eta table directly and were not
  affected.
- **New comparison-report cases.** The comparison report now includes the
  sphere and RM3 in regular waves, comparing RIRF convolution, state space
  and constant coefficients against frequency-domain solutions (RM3: coupled
  two-body solve).
- **Unknown `radiation.method` values now warn.** Previously a typo fell back
  to `rirf_convolution` silently; it still falls back, with a warning.
- **Block-style hull damping in the hydro YAML is no longer dropped.**
  `linear_damping` / `quadratic_damping` written as a YAML block list (six
  `- value` lines under the key) were ignored without a message, so the run
  had no hull damping. Both the block and the inline `[a, b, c, d, e, f]` form
  now give the same six values [surge, sway, heave, roll, pitch, yaw]. A wrong
  value count, a non-numeric entry, an empty key, or an unterminated inline
  list now stops the run with the file, line and body name; previously these
  produced zeros or partial vectors. The applied damping is logged per body at
  startup (`Linear hull damping body1 = [...] (source: hydro YAML)`, or
  `= none`). All shipped demos use the inline form and are unchanged.
- **Other hydro YAML inputs that were silently ignored now work or stop the
  run.** `moordyn.bodies` and `waves.period.values` accept block lists, and
  `waves.period` accepts an inline list. The nested `waves.period:` block
  (`values`, `linspace`, `range`) was closed on its own line and never
  applied; it now works, and the first wave key after it is no longer lost.
  Inline `hydrostatics: { model: ..., mesh_file: ... }` is now read (it was
  ignored, although the `nonlinear_hydrostatics` deprecation warning
  recommends it). A
  `hydrostatics.model` other than `linear` / `nonlinear` is now an error
  instead of falling back to linear. Any other unexpected `- item` line, and
  a body entry that does not start with `- name:`, is now an error. Unknown
  body keys are logged as warnings. CRLF line endings are handled throughout.
  No shipped demo uses the affected forms.
- **Joint reactions are exported for every joint type.** Previously only
  `ChLinkLock` joints were recorded; others, including the `ChLinkUniversal`
  joints in the 5SA articulated WEC demos, were written as zeros.
  ([PR #7](https://github.com/Project-SEA-Stack/SEA-Stack/pull/7))
- **Joint reaction metadata is corrected.** Reaction forces and torques are
  stored in world axes, with torques about the origin of the joint's link
  frame 1 (or 2). The HDF5 attributes `frame1` / `frame2` now say `world`
  (they previously said `link1` / `link2`), and new `torque_point1` /
  `torque_point2` attributes state the torque reference point. The data
  values are unchanged.
- **5SA bimodal demo is stable to 600 s.** Its hydro YAML now uses the same
  empirical demo damping as the spreading case: linear damping on all six
  DOFs (surge added, sway reduced, heave/pitch/yaw added) plus quadratic
  damping on all six DOFs. Previously the heave mode ran away after about
  200 s. Bimodal results therefore differ from beta.4. The file also states
  `excitation: method: frequency_domain` explicitly; this is what the
  automatic choice already selects for multi-heading seas. (PR #7)
- **Linux source build works** against Chrono 10.0.0 (bundled yaml-cpp,
  HDF5 discovery, script permissions). (PR #7)
- **macOS/Linux build script keeps HydroIO on when `HDF5Dir` is unset.**
  `scripts/unix/build.sh` no longer silently turns HydroIO off; CMake looks
  for HDF5 itself. If HDF5 is not installed, configure now fails with a
  FindHDF5 error. Pass `--no-hydro-io` to build without HydroIO. (PR #7)
- **Chrono-free SDK consumers on MSVC no longer corrupt the heap.**
  `SEAStack::Core` now exports `/arch:AVX2`, so code linking
  `SEAStack::Hydro` or `SEAStack::HydroIO` through `find_package(SEAStack)`
  uses the same Eigen alignment (32 bytes) as the libraries. Consumers on
  MSVC are now compiled with AVX2. The MSVC STL macro
  `_ENABLE_EXTENDED_ALIGNED_STORAGE`, previously misspelled and therefore
  ignored, is now also defined and exported.
- **Linux/macOS Eigen ABI is pinned on `SEAStack::Core` (portable 16/16).**
  Non-MSVC builds export `EIGEN_MAX_ALIGN_BYTES=16` and
  `EIGEN_MAX_STATIC_ALIGN_BYTES=16` so Chrono-free and Chrono-facing
  translation units share one Eigen heap protocol. This requires Chrono 10
  built with `CH_USE_SIMD=OFF` and the same Eigen definitions (see
  [BUILD_CHRONO.md](BUILD_CHRONO.md)); do not mix this pin with a Chrono that
  exports `-march=native`. Configure warns if Chrono’s INTERFACE looks
  inconsistent. A two-TU allocator ABI test and the downstream consumer check
  assert the contract.
- **Radiation overlay scales consistently with body size.** The GUI
  overlay over-amplified small (lab-scale) bodies; a 1:100 model showed
  radiated waves about 7x larger, relative to its motion, than the same
  device at full scale. It is now scale-consistent. The empirical constant
  was re-referenced so a body of 10 m radius looks as before; other sizes
  change by a factor of sqrt(radius / 10 m). Roll and pitch contributions now
  use world-frame angular velocity, which changes the pattern for yawed
  bodies.
- **The GUI honours `enforce_realtime` again.** beta.4 always paced the GUI
  to real time and ignored the simulation YAML. Now
  `simulation: enforce_realtime: true` paces simulated time to wall-clock
  time, and `false` (the default, and the setting in all shipped demos) runs
  as fast as the machine allows while the window keeps redrawing. Headless
  (`--nogui`) runs always go as fast as possible.
- **Wigley demos are no longer shipped in the package.** They were meant to be
  source-only; the exclusion rule never matched. The trimaran demo covers
  Wigley-style hulls.
- **CMake warns about untested Chrono versions.** Configure prints a warning
  when Chrono reports a version other than 10.0.0. It does not stop the build.
- Documentation: version strings, package layout, and the Chrono requirement.

## Known issues and limitations

- **Constant-coefficient radiation is steady-state only.** Its start-up
  transient decays at a rate set by B(w) at the wave frequency, which can be
  slow when B(w) is small. Ramp the excitation, run long enough, and discard
  the transient.
- **Some radiation and wave options are narrower than their names suggest.**
  State-space radiation ignores `radiation.truncation_time` and kernel
  smoothing/tapering (it fits the full RIRF from the `.h5`).
  `state_space.r2_threshold` is accepted but not used: the fitter always keeps
  the best-R² order up to `max_order`. Savitzky-Golay smoothing (`sg`) always
  uses a 5-point window; `window_length` applies to `moving_average` only.
  Eta-file imports keep at most 1000 Fourier bins unless
  `waves.discretization.n_omega` is raised (a warning says when bins are
  dropped).
- **Radiation overlay is qualitative.** The GUI "Radiation (approx.)" overlay
  uses an empirical amplitude model (no Kochin functions). Use it to see where
  and when bodies radiate, not to read wave heights. It never affects physics
  or exported results.
- **Linux: Chrono must match the portable Eigen ABI.** Building against a
  Chrono install that enables SIMD / `-march=native` (the Chrono default on
  many machines) while SEA-Stack pins Eigen 16/16 can still abort with
  `double free` at teardown. Use the official portable Chrono recipe in
  [BUILD_CHRONO.md](BUILD_CHRONO.md).
- **Centre of mass and hydrodynamic reference point must agree.**
  SEA-Stack assumes the body centre of mass in the model YAML, the Chrono body
  position, the hydrodynamic reference point (`cg` in the BEM `.h5` file), and
  the MoorDyn coupling origin are the same point. Nothing checks this yet. The
  shipped demos are consistent; where a demo offsets a body from the `.h5`
  `cg`, it is a deliberate initial condition (for example a decay test). Check
  this when you author new cases.
- **Wave options are fixed.** Wave stretching is always on and cannot
  be configured; it affects only the wave-kinematics query API, not
  hydrodynamic forces. JONSWAP spectra are always scaled by the standard
  approximate factor (1 - 0.287 ln gamma), so the realised `Hs` is close to,
  but not exactly, the value given in the YAML.
- **External PTO replaces TSDA/RSDA stiffness and damping by default.**
  When `external_pto` drives a TSDA or RSDA, the external model supplies the
  force and the element's own spring, damping and preload from the model YAML
  are not applied. A warning is logged if they are non-zero. Set
  `combine_native: true` on the external PTO attach to apply them on top of
  the external force (see
  [EXTERNAL_FORCE_MODULES.md](../extending/EXTERNAL_FORCE_MODULES.md)).
- **Misleading HDF5 error hint on Windows.** Any HDF5 read error,
  including a missing dataset, is reported with advice about file locking by
  other applications. Read the `HDF5 error:` line for the actual cause.

## Deferred to a later release

- Startup check that the YAML centre of mass matches the `.h5` `cg`.
- Configurable wave stretching and JONSWAP normalisation.
- Clearer reporting (GUI and config) of which TSDA/RSDA parameters are active
  under an external PTO.
- Accurate HDF5 error messages.
- External PTO / Python packaging redesign, CI and pre-commit checks, GUI
  polish, Linux packaging.
- A hard Chrono version check in CMake.
