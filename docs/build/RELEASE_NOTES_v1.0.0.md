# SEA-Stack v1.0.0 release notes

v1.0.0 is the v1.0.0-beta.4 feature set plus fixes that make the advertised
functionality trustworthy, installable, and defensible. There are no new
features and no changes to the physics models.

## Requirements

- **Chrono 10.0.0 exactly** (release tag `10.0.0`). Newer Chrono, including
  Chrono `main`, breaks the build. See [BUILD_CHRONO.md](BUILD_CHRONO.md).
- **Packages:** Windows (x64) and macOS. Linux is supported as a source build
  only (see [BUILD_MACOS.md](BUILD_MACOS.md), which covers macOS and Linux);
  no Linux package is published.
- **External PTO demos** need Python 3 on `PATH` (see `QUICKSTART.txt`).

## Changes since v1.0.0-beta.4

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
- **Radiation overlay scales consistently with body size.** The GUI
  overlay over-amplified small (lab-scale) bodies; a 1:100 model showed
  radiated waves about 7x larger, relative to its motion, than the same
  device at full scale. It is now scale-consistent. The empirical constant
  was re-referenced so a body of 10 m radius looks as before; other sizes
  change by a factor of sqrt(radius / 10 m). Roll and pitch contributions now
  use world-frame angular velocity, which changes the pattern for yawed
  bodies.
- **Wigley demos are no longer shipped in the package.** They were meant to be
  source-only; the exclusion rule never matched. The trimaran demo covers
  Wigley-style hulls.
- Documentation: version strings, package layout, and the Chrono requirement.

## Known issues and limitations

- **Radiation overlay is qualitative.** The GUI "Radiation (approx.)" overlay
  uses an empirical amplitude model (no Kochin functions). Use it to see where
  and when bodies radiate, not to read wave heights. It never affects physics
  or exported results.
- **Linux: crash at exit.** On Linux, `run_seastack` can abort with
  `double free` (exit code 134) after the run has completed and results are
  written. Windows and macOS exit cleanly.
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
