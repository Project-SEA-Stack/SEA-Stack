# SEA-Stack Test Runner

Verify your installation by running the demo cases headless, comparing
simulation output to expected baselines, and generating comparison plots.

## Quick start (PowerShell)

From the package root:

```powershell
tests\RUN-TESTS.ps1
```

**Python 3.10+** is required for the test scripts.

The script will:

1. Offer to create a local Python virtual environment (`.venv`) and install
   the required packages (`numpy`, `h5py`, `PyYAML`, `matplotlib`).
2. Run the default set of demo cases with `run_seastack.exe --nogui`.
   This covers the core regression suite (IEA sphere, RM3, OSWEC, F3OF
   cases) aligned with the runtime ZIP (no `rm3/regular_waves`; F3OF ships
   `decay_dt3` only). It does not run every demo listed in the demo catalog.
3. Compare each result against its `expected/` baseline.
4. Save comparison plots to `demos/<model>/<case>/outputs/plots/`.
5. Print PASS / FAIL for each case.

### macOS / Linux (bash)

From the package root:

```bash
./tests/RUN-TESTS.sh
```

Optional: `--python /path/to/python3` or `--no-venv` (same semantics as the PowerShell script).

**Note:** File paths in this README assume the **packaged release** layout.
In a source checkout, the runner script is at
`tests\regression\run_seastack\run_tests.py` and demos are under
`data\demos\run_seastack\`.

## CTest (CMake build)

Reference compare steps and HTML/Markdown reports need the packages in
`tests/regression/run_seastack/requirements.txt` installed for the same Python
interpreter CMake selected (`Python3_EXECUTABLE`). From the repo root:

- **Unix:** `./scripts/unix/ctest_suite.sh regression` runs the full label set;
  add `--no-python` to run only C++ execute tests (no `reference`-labeled
  compares or report).
- **Windows:** after configuring the build, use
  `.\scripts\windows\run_regression_tests.ps1` (see `-Help`) or invoke `ctest`
  from the build directory with the same label conventions as your CMake setup.

## Manual usage

If you prefer to run the Python scripts directly:

```
pip install -r tests\requirements.txt
python tests\run_tests.py --all --exe bin\run_seastack.exe
```

### Per-test flags

| Flag | Demo case |
|------|-----------|
| `--sphere-decay` | IEA sphere, free decay |
| `--sphere-decay-ss` | IEA sphere, free decay (state-space) |
| `--sphere-irregular-ss` | IEA sphere, irregular waves (state-space) |
| `--sphere-decay-nl-1m` / `--sphere-decay-nl-5m` / `--sphere-decay-lin-5m` | IEA sphere decay variants (linear / nonlinear hydrostatics) |
| `--oswec-regular` | OSWEC regular waves |
| `--oswec-irregular` | OSWEC irregular (JONSWAP) |
| `--oswec-decay` | OSWEC still-water decay (minimal) |
| `--rm3-regular` | RM3 regular waves + PTO |
| `--rm3-irregular` | RM3 irregular (JONSWAP) + PTO |
| `--rm3-mooring` | RM3 mooring (MoorDyn) |
| `--rm3-decay` / `--rm3-decay-nl` | RM3 still-water decay (minimal / NL hydro) |
| `--f3of-decay-c1` | F3OF `decay_dt1` (source repo; omitted from release ZIP) |
| `--f3of-decay-c2` | F3OF `decay_dt2` (source repo; omitted from release ZIP) |
| `--f3of-decay-c3` | F3OF `decay_dt3` (alias of `--f3of-decay-dt3`) |
| `--f3of-decay-dt3` | F3OF `decay_dt3` (ships in release ZIP) |

Add `--show` to display plots interactively, or `--gui` to run simulations
with the visualisation window.

Use `--update-baseline` to copy the case’s latest `outputs/results.*.h5` into
`demos/<model>/<case>/expected/baseline.h5` after a successful run (refresh
pinned references when the solver or wave settings change).

## Requirements

- Python 3.10 or newer
- Packages listed in `tests\requirements.txt`:
  `numpy`, `h5py`, `PyYAML`, `matplotlib`

## Files

| File | Purpose |
|------|---------|
| `RUN-TESTS.ps1` | PowerShell wrapper (venv setup, PATH, env vars) |
| `run_tests.py` | Main test orchestrator |
| `run_simulation.py` | Runs a single simulation via `run_seastack.exe` |
| `compare_results.py` | Compares simulation output to a baseline |
| `compare_template.py` | Shared plotting helper |
| `generate_plots.py` | Regenerate comparison plots without re-simulating |
| `requirements.txt` | Python dependencies |
