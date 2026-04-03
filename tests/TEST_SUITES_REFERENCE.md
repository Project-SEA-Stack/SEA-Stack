# Test suites reference (SEA-Stack)

Developer and CI reference: console behavior, raw `ctest`, PowerShell vs CMake report targets, CTest naming, and full per-suite inventories.

For a short user-facing guide (verify install, run suites, track overview), start at [`README.md`](README.md).

## Console output (PowerShell runners)

By default, ctest shows **live** completed-test lines (pass/fail and timings). Parallel **`Start N:`** lines are filtered out for readability; use **`-Verbose`** to show the full ctest stream including those lines (and `ctest -V`). Use **`-Quiet`** for minimal ctest output (`ctest -Q`). Report generators still default to **`--quiet`** unless **`-Verbose`** (regression / verification / comparison).

## `-BuildType` (beta)

Scripts accept `-BuildType` so `ctest -C` and report output under `build/bin/<BuildType>/...` stay aligned. **`Release` is the supported and validated mode** for report workflows. Non-Release may work only partially (Python/report discovery is still Release-centric in places)—do not assume full multi-config report support.

## Raw `ctest` vs PowerShell (verification and comparison)

These two models coexist on purpose:

1. **Raw `ctest -L verification` / `ctest -L comparison`:** Unchanged CMake behavior. Includes CTest targets `verification_report_generation` / `comparison_report_generation`. Whether `--pdf` is passed to Python is decided **at CMake configure time** (`find_program(pandoc)`).
2. **`run_verification_tests.ps1` / `run_comparison_tests.ps1`:** **Exclude** those report tests (`ctest -E`), run executables first, then invoke the Python report generators with a **runtime** pandoc check (same idea as `run_regression_tests.ps1`). Final exit code is still **ctest’s** exit code so failures are not masked.

Regression has **no** CTest report target; benchmarks already rerun the benchmark report script from `run_benchmarks.ps1` for visible output.

## Raw ctest examples (CI, Linux, advanced)

```bash
# Core regression only (CI gate):
ctest -L regression -LE "verification|comparison|benchmark"

# Regression report is not a CTest target; after tests:
#   .\scripts\windows\run_regression_tests.ps1
#   python tests/regression/utilities/generate_regression_report.py --build-dir build --config Release [--pdf]

ctest -L verification
ctest -L comparison
ctest -L benchmark
```

## Core Regression Tests

Fast regression tests against frozen SEA-Stack reference baselines (`ss_ref_*.txt`). Must complete in minutes. (Historically aligned with the predecessor HydroChrono project; current references are SEA-Stack-owned.)

**CTest identity:** `test_regression_<family>_<case_id>` for the simulation step and `..._<case_id>_reference` for the Python comparison. **CMake targets** remain `test_<legacy_name>` (e.g. `test_sphere_decay`). Regular-wave per-condition jobs use `test_regression_<family>_reg_waves_c<N>` / `..._reference`. Full-period sweeps: `test_regression_<family>_reg_waves_full` under the regression label; the verification suite uses `test_verification_<case>_regression_data` (same executable) so `ctest -L verification` does not pull in regression-labeled tests as CTest fixture prerequisites.

| Case | Model | CMake target | CTest run | Notes |
|------|-------|--------------|-----------|-------|
| `sphere_decay` | sphere | `test_sphere_decay` | `test_regression_sphere_decay` | Heave decay (convolution) |
| `sphere_decay_ss` | sphere | `test_sphere_decay_ss` | `test_regression_sphere_decay_ss` | Heave decay (state-space) |
| `sphere_reg_waves` | sphere | `test_sphere_reg_waves` | `test_regression_sphere_reg_waves` (+ per-`cN`) | Regular waves (2 core conditions when subset ON) |
| `sphere_irreg_waves` | sphere | `test_sphere_irreg_waves` | `test_regression_sphere_irreg_waves` | Irregular waves (spectrum) |
| `sphere_irreg_waves_ss` | sphere | `test_sphere_irreg_waves_ss` | `test_regression_sphere_irreg_waves_ss` | Irregular waves (state-space) |
| `sphere_irreg_waves_eta` | sphere | `test_sphere_irreg_waves_eta` | `test_regression_sphere_irreg_waves_eta` | DFT reconstruction; see [ETA import notes](regression/sphere/ETA_IMPORT.md) |
| `sphere_irreg_waves_eta_consistency` | sphere | `test_sphere_irreg_waves_eta_consistency` | `test_regression_sphere_irreg_waves_eta_consistency` | Consistency label |
| `f3of_decay_c1` / `c2` / `c3` | f3of | `test_f3of_decay_c*` | `test_regression_f3of_decay_c*` | Decay tests |
| `oswec_decay` | oswec | `test_oswec_decay` | `test_regression_oswec_decay` | Pitch decay (convolution) |
| `oswec_decay_ss` | oswec | `test_oswec_decay_ss` | `test_regression_oswec_decay_ss` | Pitch decay (state-space) |
| `oswec_reg_waves` | oswec | `test_oswec_reg_waves` | `test_regression_oswec_reg_waves` (+ per-`cN`) | Full sweep in verification |
| `oswec_irreg_waves` | oswec | `test_oswec_irreg_waves` | `test_regression_oswec_irreg_waves` | Irregular waves |
| `oswec_irreg_waves_ss` | oswec | `test_oswec_irreg_waves_ss` | `test_regression_oswec_irreg_waves_ss` | Irregular waves (state-space) |
| `rm3_decay` | rm3 | `test_rm3_decay` | `test_regression_rm3_decay` | Multi-body decay |
| `rm3_reg_waves` | rm3 | `test_rm3_reg_waves` | `test_regression_rm3_reg_waves` | Multi-body regular wave |

### Core Regression Subset

When `SEASTACK_CORE_REGRESSION_SUBSET=ON` (default), sphere and OSWEC regular wave tests
run only two representative conditions each:

- **Sphere**: conditions 3 (near resonance, ω≈1.43 rad/s) and 7 (off-resonance, ω≈0.79 rad/s)
- **OSWEC**: conditions 3 (T=8s, off-resonance) and 12 (T=19.5s, near resonance)

Set `SEASTACK_CORE_REGRESSION_SUBSET=OFF` to run the full sweep.

## Verification Tests

Broader code-to-code comparisons. Tolerances are wider than regression.

**CTest identity:** `test_verification_<family>_<case_id>` for the simulation step and `..._verification` for the cross-code comparison (RAO compare-only cases register only the latter). Data paths still use the verification directory slug (e.g. `sphere_decay_multicode/`).

| Test | Model | CMake target | CTest (run / compare) | External Source | Data Type |
|------|-------|--------------|----------------------|-----------------|-----------|
| `rm3_mooring` | rm3 | `test_rm3_mooring` | `test_verification_rm3_mooring` / `..._verification` | WEC-Sim/MoorDyn | Motions + fairleads |
| `sphere_decay_multicode` | sphere | `test_sphere_decay_multicode` | `test_verification_sphere_decay_multicode` / `..._verification` | ProteusDS, InWave, Marin, NREL CFD, WavEC | Heave decay |
| `sphere_rao_sweep` | sphere | `test_sphere_reg_waves_full` | `test_verification_sphere_rao_sweep_regression_data` then `test_verification_sphere_rao_sweep_verification` | SEA-Stack reference (10 conditions) | RAO |
| `oswec_decay_wecsim` | oswec | `test_oswec_decay_wecsim` | `test_verification_oswec_decay_wecsim` / `..._verification` | WEC-Sim | Pitch decay |
| `oswec_rao_sweep` | oswec | `test_oswec_reg_waves_full` | `test_verification_oswec_rao_sweep_regression_data` then `test_verification_oswec_rao_sweep_verification` | SEA-Stack reference (16 conditions) | RAO |
| `rm3_decay_wecsim` | rm3 | — | — | WEC-Sim | (not registered in current CMake) |
| `f3of_decay_multicode` | f3of | `test_f3of_decay_multicode` | `test_verification_f3of_decay_multicode` / `..._verification` | INW, WSM, WDN, PDS | 4 DOF decay |

## Internal Method Comparison Tests

Compare SEA-Stack methods against each other. No external reference data.

**CTest identity:** `test_comparison_<family>_<case_id>` and `..._comparison` for the Python analysis step. RM3 mesh check: `test_comparison_rm3_validate_rm3_mesh_topology` (execute only).

| CMake target | CTest run | What It Compares |
|--------------|-----------|------------------|
| `test_compare_excitation_irf_vs_fd_sphere_irreg` | `test_comparison_sphere_compare_excitation_irf_vs_fd_sphere_irreg` | kPolar vs kCartesian interpolation |
| `test_compare_eta_dft_vs_convolution_sphere_irreg` | `test_comparison_sphere_compare_eta_dft_vs_convolution_sphere_irreg` | DFT vs direct-eta convolution for eta-import |
| `test_compare_linear_vs_nonlinear_hs_sphere_decay` | `test_comparison_sphere_compare_linear_vs_nonlinear_hs_sphere_decay` | Linear vs nonlinear hydro |
| `test_compare_linear_vs_hydraulic_pto_rm3_irreg` | `test_comparison_rm3_compare_linear_vs_hydraulic_pto_rm3_irreg` | LinearPTO vs hydraulic PTO |
| `test_validate_rm3_mesh_topology` | `test_comparison_rm3_validate_rm3_mesh_topology` | Mesh topology validation |
| `compare_interp_polar_vs_cartesian_sphere_reg` | — | (planned; not in current CMake) |
| `compare_radiation_conv_vs_ss_sphere_decay` | — | (planned; not in current CMake) |

## Performance Benchmarks

Wall-clock timing tests. Output JSON with per-step costs.

**CTest identity:** `test_benchmark_<family>_<case_id>` (CMake targets stay `test_bench_*`). Example: target `test_bench_sphere_decay_conv` → CTest `test_benchmark_sphere_decay_conv`.

| CMake target (excerpt) | CTest | What It Measures |
|------------------------|-------|------------------|
| `test_bench_sphere_decay_conv` | `test_benchmark_sphere_decay_conv` | Sphere decay convolution timing |
| `test_bench_sphere_irreg_conv` | `test_benchmark_sphere_irreg_conv` | Sphere irregular convolution timing |
| `test_bench_oswec_reg_batch` | `test_benchmark_oswec_reg_batch` | OSWEC regular-wave batch |
| `test_bench_rm3_mooring_irreg` | `test_benchmark_rm3_mooring_irreg` | RM3 + MoorDyn irregular |
| `test_bench_f3of_decay_c3` | `test_benchmark_f3of_decay_c3` | F3OF decay timing |
| `test_bench_5sa_spreading` | `test_benchmark_5sa_spreading` | 5SA spreading (when mooring enabled) |
