# SEA-Stack release package layout (CPack ZIP)

This describes what ships in the **runtime** ZIP produced by `cmake --install` and CPack (`SEAStack-<version>-win64.zip`). It is **not** the same as runtime output folders created when users run simulations (see [RUNTIME_OUTPUT.md](RUNTIME_OUTPUT.md)).

## Top level

| Path | Purpose |
|------|---------|
| `bin/` | Executables (`run_seastack.exe`, optional `standalone_controller.exe`, optional `demo_sphere_decay.exe` when demos were enabled at build time) and third-party/runtime DLLs |
| `demos/` | YAML-driven `run_seastack` case data (geometry, hydro HDF5 inputs, configs), including RM3/OSWEC `external_pto*` Python PTO cases |
| `python/` | Thin IPC helper `seastack_external.py` for out-of-process force demos under `demos/` |
| `examples/external_pto/` | Comparison-plot scripts (`run_visual_verification.py`, `plot_verification.py`) for the external PTO demos |
| `data/chrono/` | Chrono runtime assets (optional) |
| `tests/` | Python/PowerShell regression helpers to validate an install |
| `LICENSE` | Project license |
| `THIRD_PARTY_NOTICES.txt` | Third-party notices (if present in source tree) |
| `QUICKSTART.txt` | Short getting-started notes (installed from `docs/build/QUICKSTART_RELEASE.txt`) |

## External PTO demos and comparison plots

Release packaging (`build.ps1 -Package` / `build.sh --package`) configures
`-DSEASTACK_ENABLE_EXTERNAL=ON` so `run_seastack` honours `external_pto_file`
and the plot scripts are installed.

From the unzipped package root (Python 3 + `h5py`/`numpy`/`matplotlib` from
`tests/requirements.txt`):

```text
# Windows
bin\run_seastack.exe --nogui demos\rm3\external_pto
python examples\external_pto\run_visual_verification.py
python examples\external_pto\run_visual_verification.py --platform oswec

# macOS / Linux
./bin/run_seastack --nogui demos/rm3/external_pto
python3 examples/external_pto/run_visual_verification.py
python3 examples/external_pto/run_visual_verification.py --platform oswec
```

The demos launch `command: ["python", ...]`; on macOS/Linux the host falls back
to `python3` when `python` is not on `PATH`.

`run_visual_verification.py` finds `bin/run_seastack` and `demos/<platform>/`
automatically when launched from the package tree.

## Policy: generated HDF5 results

Release ZIPs **do not** include user-generated simulation result `.h5` files. Users produce outputs by running `run_seastack` or packaged demos; see [RUNTIME_OUTPUT.md](RUNTIME_OUTPUT.md) for conventions.

## SDK / developer installs

Components marked `sdk` in CMake (headers, `SEAStackConfig.cmake`, static/import libs) are **not** part of the default CPack `runtime` ZIP. They are for `cmake --install` without CPack component filtering or for custom packaging.
