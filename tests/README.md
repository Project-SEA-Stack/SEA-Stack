# SEA-Stack tests (user guide)

Use this page to **verify an install** and **run the main test suites** after a pre-built package or a source build. Maintainer reference: [TEST_SUITES_REFERENCE.md](TEST_SUITES_REFERENCE.md) (CTest inventories, raw `ctest`, runners), [REPORTING_AND_PLOTS.md](REPORTING_AND_PLOTS.md) (reports and plots), [CONTRIBUTING.md](../CONTRIBUTING.md) (constraints), [data/verification/README.md](../data/verification/README.md) (verification data layout and normalization).

## Verify your installation

**Pre-built package** — Python **3.10 or newer** required. From the package root:

- Windows: `tests\RUN-TESTS.ps1`
- macOS / Linux: `./tests/RUN-TESTS.sh`

This runs every demo case headless, compares output against expected baselines, and prints PASS/FAIL per case. Details and flags: [tests/regression/run_seastack/README.md](regression/run_seastack/README.md).

**Source build (Windows)** — After `scripts/windows/build.ps1` (or an equivalent CMake build), use the suite scripts in the table below. For a quick **CTest-only** pass without post-run report steps: `.\scripts\windows\build.ps1 -Test` or `.\scripts\windows\build.ps1 -Test -TestLabel unit`. Advanced: `ctest` from `build/` — see [TEST_SUITES_REFERENCE.md](TEST_SUITES_REFERENCE.md).

**macOS / Linux** — From the repo root after building:

```bash
./scripts/unix/ctest_suite.sh <suite>
```

Use `<suite>` = `regression`, `unit`, `verification`, `comparison`, `chrono-free`, or `benchmark`. Pass `--no-pdf` if Pandoc is on `PATH` but no LaTeX engine is installed.

## What the suites mean

Regression compares simulation output to frozen SEA-Stack baselines (`ss_ref_*.txt`) with tight tolerances; it is the usual **every-commit** gate. **Consistency** jobs are a subset of regression (extra label `consistency`) for internal checks such as spectrum vs eta round-trip.

Verification compares against **external or multi-code** reference data under `data/verification/` with **wider** tolerances. Comparison runs **two SEA-Stack configurations** against each other (no external reference). Benchmark measures **wall-clock timing** (JSON output; not a strict pass/fail gate like regression).

Unit tests exercise libraries with assertions. **Chrono-free** tests omit Chrono DLLs; useful when you only need the non-Chrono stack.

| Track / suite | Purpose | Typical CI | CTest label(s) |
|---------------|---------|------------|----------------|
| Unit | Library checks | Every commit | `unit` |
| Chrono-free | No Chrono | Every commit | `chrono-free` |
| Core regression | vs `ss_ref_*.txt` | Every commit (fast) | `regression` |
| Consistency | Internal self-validation | Every commit | `regression;consistency` |
| Verification | Cross-code / multi-code | Nightly / on-demand | `verification` |
| Comparison | Method vs method (internal) | On-demand | `comparison` |
| Benchmark | Performance timing | On-demand | `benchmark` |

## Run the main test suites

**Windows (PowerShell, from repo root)** — Prefer these wrappers; they align `ctest -C` with `-BuildType`, set parallelism, and run report steps where applicable. Pass `-j N` to cap parallelism (`0` = all CPUs on most scripts). See `-Help` on each script.

| Suite | Script |
|-------|--------|
| Unit | `.\scripts\windows\run_unit_tests.ps1` |
| Chrono-free | `.\scripts\windows\run_chrono_free_tests.ps1` |
| Regression | `.\scripts\windows\run_regression_tests.ps1` (`-Long` for long tests; `-NoPdf` to skip PDF) |
| Verification | `.\scripts\windows\run_verification_tests.ps1` |
| Comparison | `.\scripts\windows\run_comparison_tests.ps1` |
| Benchmark | `.\scripts\windows\run_benchmarks.ps1` (sequential `-j 1`) |

**Unix** — `./scripts/unix/ctest_suite.sh <suite>` as above.

Verbose ctest output, `-BuildType`/Release notes, and **raw `ctest` vs PowerShell** behavior for verification/comparison reports: [TEST_SUITES_REFERENCE.md](TEST_SUITES_REFERENCE.md).

## Reports (summary)

Regression, verification, and comparison can produce **Markdown** reports under `build/bin/<Config>/results/tests/...` (e.g. `.../regression/report/regression_test_report.md`). **PDF** needs Pandoc and a LaTeX engine (MiKTeX, MacTeX, TeX Live); Windows runners add PDF when `pandoc` is on `PATH` unless you pass `-NoPdf`. Markdown is still written if PDF conversion fails.

Configure-time vs runtime PDF behavior, `.status.json`, and shared generator code: [REPORTING_AND_PLOTS.md](REPORTING_AND_PLOTS.md).

## Further reading

- [TEST_SUITES_REFERENCE.md](TEST_SUITES_REFERENCE.md) — per-case tables, raw `ctest`, CI examples
- [data/verification/CONVENTIONS.md](../data/verification/CONVENTIONS.md) — SI units, frames, signs for verification data
- [data/verification/README.md](../data/verification/README.md) — directory layout, normalization, provenance
- [REPORTING_AND_PLOTS.md](REPORTING_AND_PLOTS.md) — report pipelines and plot standard
- [CONTRIBUTING.md](../CONTRIBUTING.md) — test expectations and maintainer constraints
- [regression/sphere/ETA_IMPORT.md](regression/sphere/ETA_IMPORT.md) — DFT eta-import pathway and `sphere_irreg_waves_eta`
