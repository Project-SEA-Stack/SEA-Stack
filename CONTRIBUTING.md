# Contributing to SEA-Stack

Thank you for your interest in contributing to SEA-Stack. This document
explains how to report issues, propose changes, and contribute code.

## Reporting bugs

Open a [GitHub Issue](https://github.com/Project-SEA-Stack/sea-stack/issues)
with:

- A clear description of the problem
- Steps to reproduce (YAML config, command line, or minimal code)
- Expected vs. actual behaviour
- Platform and compiler version (e.g. Windows 11, MSVC 2022)
- SEA-Stack version or commit hash

## Suggesting features

Open a GitHub Issue with the label **enhancement**. Describe the use case,
why existing functionality does not cover it, and (if applicable) a rough
idea of the approach.

## Module layout

SEA-Stack separates domain physics from the solver engine. The main
directories are:

- `libs/` — domain libraries (Hydro, PTO, Control, Infrastructure, HydroIO,
  Mooring). These are Chrono-free.
- `adapters/chrono/` — the Chrono adapter layer (the only code that imports
  Chrono).
- `apps/` — application executables (`run_seastack`).
- `examples/` — standalone usage examples (no Chrono required).
- `demos/` — Chrono-integrated demo executables.

See [TECHNICAL_OVERVIEW.md](TECHNICAL_OVERVIEW.md) for the full architecture.

## Extending SEA-Stack

SEA-Stack is designed for extension through well-defined interfaces:

- New force components — implement `IHydroForceComponent`
- New wave models — subclass `WaveBase`
- New PTO models — implement `IPTOModel`
- New controllers — implement `IController`
- New solver adapters — pattern from `adapters/chrono/`

See [docs/extending/EXTENDING.md](docs/extending/EXTENDING.md) for
step-by-step guides covering each extension point.

## Contributing code

We use a **fork-based workflow**. Work happens in your **GitHub fork** of the
repository: you create a **topic branch** there, push your commits, and open a
**pull request** to merge that branch into the upstream **`develop`** branch.
A **reviewer** (maintainer or delegate) checks the PR; after approval, it is
**merged into `develop`**. Promoting `develop` to release branches or `main` is
handled by the maintainers.

1. **Fork** the upstream repository on GitHub and **clone your fork** locally. If the project uses Git submodules (MoorDyn under `extern/MoorDyn`), use `git clone --recursive`, or after a plain clone run `git submodule update --init --recursive` from the **repository root** (the directory that contains `.gitmodules`).
2. **Update `develop`** from the upstream repository (fetch and merge or
   rebase so your starting point is current), then **create a branch from
   `develop`** with a short prefix that
   reflects the change, for example:
   - `feature/…` — new capability, demos, or larger additions
   - `fix/…` — bug fixes
   - `docs/…` — documentation-only changes  
   Use lowercase, hyphens for words, and keep the rest of the name specific
   (e.g. `feature/trimaran-mooring-demo`).
3. **Build and test** your changes locally — prefer
   `.\scripts\windows\run_regression_tests.ps1` (Windows) or
   `./scripts/unix/ctest_suite.sh regression` (macOS/Linux); use
   `.\scripts\windows\build.ps1 -Test` or raw `ctest` only for a quick gate.
   See [tests/README.md](tests/README.md); for CTest inventories and raw `ctest`, see
   [tests/TEST_SUITES_REFERENCE.md](tests/TEST_SUITES_REFERENCE.md).
4. **Push** the branch to **your fork** and **open a pull request** against
   **`develop`** with a clear description of the change and its motivation.
5. **Iterate with reviewers** (address feedback, update the branch as needed)
   until the PR is approved and merged.

### Contributing demos

New demos can take either of these forms (or both, if the same model is useful
in YAML and as a small C++ example):

- **YAML-driven cases** — add a model folder under
  [`data/demos/run_seastack/`](data/demos/run_seastack/) with shared assets
  (geometry, hydrodynamic databases, etc.) and one or more `*.setup.yaml`
  cases for [`run_seastack`](apps/seastack/). See
  [`data/demos/run_seastack/README.md`](data/demos/run_seastack/README.md)
  for layout and how to run cases.
- **C++ Chrono demos** — add a subdirectory under [`demos/`](demos/) with
  executable sources that call the Chrono adapter directly. See
  [`demos/README.md`](demos/README.md) for build flags and how runtime data
  is staged from `data/demos/run_seastack/*/assets/`.

Include a **model `README.md`** that states what the demo shows, key
parameters, how to build or run it, and any physical assumptions (frames,
units, hydro body ordering). Match the level of detail in existing demos.

**Reproducibility (recommended, not required):** If you can, add scripts or
documented commands that regenerate meshes and BEM / hydro exports (e.g.
panel models and `*.h5`) from source CAD or parametric inputs. That makes it
easier for others to verify and extend your case; checked-in assets alone are
still acceptable when a full pipeline is impractical.

### Code style

SEA-Stack follows the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
with these conventions:

- C++17 standard
- `PascalCase` for classes, `snake_case` for files and variables,
  `kPascalCase` for constants
- `include/` and `src/` layout within each library
- Physical quantities carry documented SI units in comments
- Prefer clarity over cleverness; avoid template metaprogramming
- Label your changes: `[BUG FIX]`, `[COMPATIBILITY]`, `[IMPROVEMENT]`,
  or `[INFRASTRUCTURE]`.

### Test expectations

- All existing regression tests must continue to pass.
- New functionality should include appropriate tests (unit tests for
  Chrono-free logic, regression tests for simulation behaviour).
- SEA-Stack reference data (`ss_ref_*.txt`) is **never regenerated**.
- Legacy and improved code paths remain **strictly separated**.
- Raw verification data under `data/verification/` is **never modified** after
  initial commit.
- Normalized verification data is **deterministically reproducible** from raw
  sources plus each case’s `normalize.py`.
- Every normalized dataset has a **`manifest.json`** with full provenance.
- Core regression stays **lean and fast**; broader checks belong in verification.
- Verification tolerances are **wider** than regression.
- Prefer **explicit, minimal** test infrastructure over unnecessary abstraction.

See [tests/README.md](tests/README.md) for suite overview and how to run tests.
For per-case tables and CI-style `ctest` usage, see
[tests/TEST_SUITES_REFERENCE.md](tests/TEST_SUITES_REFERENCE.md).


## License

By contributing, you agree that your contributions will be licensed under
the [MIT License](LICENSE), the same license that covers the project.
