# SEA-Stack release package layout (CPack ZIP)

This describes what ships in the **runtime** ZIP produced by `cmake --install` and CPack (`SEAStack-<version>-win64.zip`). It is **not** the same as runtime output folders created when users run simulations (see [RUNTIME_OUTPUT.md](RUNTIME_OUTPUT.md)).

## Top level

| Path | Purpose |
|------|---------|
| `bin/` | Executables (`run_seastack.exe`, optional `standalone_controller.exe`, optional `demo_sphere_decay.exe` when demos were enabled at build time) and third-party/runtime DLLs |
| `demos/` | YAML-driven `run_seastack` case data (geometry, hydro HDF5 inputs, configs) |
| `data/chrono/` | Chrono runtime assets (optional) |
| `tests/` | Python/PowerShell regression helpers to validate an install |
| `LICENSE` | Project license |
| `THIRD_PARTY_NOTICES.txt` | Third-party notices (if present in source tree) |
| `QUICKSTART.txt` | Short getting-started notes (installed from `docs/build/QUICKSTART_RELEASE.txt`) |

## Policy: generated HDF5 results

Release ZIPs **do not** include user-generated simulation result `.h5` files. Users produce outputs by running `run_seastack` or packaged demos; see [RUNTIME_OUTPUT.md](RUNTIME_OUTPUT.md) for conventions.

## SDK / developer installs

Components marked `sdk` in CMake (headers, `SEAStackConfig.cmake`, static/import libs) are **not** part of the default CPack `runtime` ZIP. They are for `cmake --install` without CPack component filtering or for custom packaging.
