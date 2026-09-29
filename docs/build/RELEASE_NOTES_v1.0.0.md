# SEA-Stack v1.0.0 release notes

v1.0.0 is the v1.0.0-beta.4 feature set plus fixes that make the advertised
functionality trustworthy, installable, and defensible. There are no new
features and no changes to the physics models.

## Requirements

- **Chrono 10.0.0 exactly** (release tag `10.0.0`). Newer Chrono, including
  Chrono `main`, breaks the build. See [BUILD_CHRONO.md](BUILD_CHRONO.md).
- **Packages:** Windows (x64) and macOS. Linux is supported as a source build
  only; no Linux package is published.
- **External PTO demos** need Python 3 on `PATH` (see `QUICKSTART.txt`).

## Changes since v1.0.0-beta.4

- **Joint reactions are exported for every joint type.** Previously only
  `ChLinkLock` joints were recorded; others, including the `ChLinkUniversal`
  joints in the 5SA articulated WEC demos, were written as zeros.
  ([PR #7](https://github.com/Project-SEA-Stack/SEA-Stack/pull/7))
- **5SA bimodal demo is stable to 600 s.** It gains heave/pitch/yaw damping
  (mirroring the spreading case) and frequency-domain excitation. Previously
  the heave mode ran away after about 200 s. (PR #7)
- **Linux source build works** against Chrono 10.0.0 (bundled yaml-cpp,
  HDF5 discovery, script permissions). (PR #7)
- **Chrono-free SDK consumers on MSVC no longer corrupt the heap.**
  `SEAStack::Core` now exports `/arch:AVX2` and
  `ENABLE_EXTENDED_ALIGNED_STORAGE`, so code linking `SEAStack::Hydro` or
  `SEAStack::HydroIO` through `find_package(SEAStack)` uses the same Eigen
  alignment as the libraries. Consumers on MSVC are now compiled with AVX2.
- **Radiation overlay scales consistently with body size.** The GUI
  overlay over-amplified small (lab-scale) bodies; a 1:100 model showed
  radiated waves about 7x larger, relative to its motion, than the same
  device at full scale. It is now scale-consistent. Full-scale bodies look the
  same as before.
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
  hydrodynamic forces. JONSWAP spectra are always normalised to the `Hs` given
  in the YAML.
- **External PTO overrides TSDA/RSDA stiffness and damping silently.**
  When `external_pto` drives a TSDA or RSDA, that element's spring
  and damping coefficients are set to zero and the external model supplies the
  force. Any stiffness or damping given for the element in the model YAML is
  ignored without a warning.
- **Misleading HDF5 error hint on Windows.** Any HDF5 read error,
  including a missing dataset, is reported with advice about file locking by
  other applications. Read the `HDF5 error:` line for the actual cause.

## Deferred to a later release

- Startup check that the YAML centre of mass matches the `.h5` `cg`.
- Configurable wave stretching and JONSWAP normalisation.
- Config validation for TSDA/RSDA parameters under an external PTO.
- Accurate HDF5 error messages.
- External PTO / Python packaging redesign, CI and pre-commit checks, GUI
  polish, Linux packaging.
- A hard Chrono version check in CMake.
