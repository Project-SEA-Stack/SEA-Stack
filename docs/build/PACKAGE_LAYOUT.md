# SEA-Stack release package layout (CPack ZIP)

This describes what ships in the **runtime** ZIP produced by `cmake --install` and CPack (`SEAStack-<version>-win64.zip`). It is **not** the same as runtime output folders created when users run simulations (see [RUNTIME_OUTPUT.md](RUNTIME_OUTPUT.md)).

## Top level

| Path | Purpose |
|------|---------|
| `bin/` | Executables (`run_seastack.exe`, optional `standalone_controller.exe`, optional `demo_sphere_decay.exe` when demos were enabled at build time) and third-party/runtime DLLs. When built with SPH support, this also includes the Chrono FSI SPH DLLs and the CUDA runtime DLLs they depend on (`nvrtc*`, `cublas*`, `cusparse*`). |
| `demos/` | YAML-driven `run_seastack` case data (geometry, hydro HDF5 inputs, configs), including RM3/OSWEC `external_pto*` Python PTO cases and, when built with SPH, the `objectdrop_sph` SPH case. (Wigley demos, including the coupled `wigley/sloshing` BEM+SPH tank case, stay in the source tree and are omitted from the ZIP.) |
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

## SPH (Chrono::FSI) support

When SEA-Stack is built with `-DSEASTACK_ENABLE_SPH=ON` (against a Chrono install built with `CH_ENABLE_MODULE_FSI_SPH=ON`), the package additionally bundles the SPH runtime: `Chrono_fsi.dll`, `Chrono_fsisph.dll`, `Chrono_fsisph_vsg.dll`, and the CUDA runtime DLLs `Chrono_fsisph.dll` links (`nvrtc*`, `nvrtc-builtins*`, `nvJitLink*`, `cublas*`, `cublasLt*`, `cusparse*`). The GPU **driver** (`nvcuda.dll`) is not redistributable and must be present on the target machine; SPH cases require an **NVIDIA GPU** at runtime. Run the `objectdrop_sph` demo to confirm SPH works:

```text
# Windows
bin\run_seastack.exe --nogui demos\objectdrop_sph\objectdrop_sph.setup.yaml
```

Coupled seakeeping + deck-tank sloshing (potential-flow exterior + SPH interior, setup with `hydro_file:` + `tank:`) is available from a source checkout as `data/demos/run_seastack/wigley/sloshing/` when the same SPH flags are enabled. It is GPU-intensive (SPH sub-steps ≪ 1 ms); keep demo durations short. See the demos README for flotation and cost notes.

## Policy: generated HDF5 results

Release ZIPs **do not** include user-generated simulation result `.h5` files. Users produce outputs by running `run_seastack` or packaged demos; see [RUNTIME_OUTPUT.md](RUNTIME_OUTPUT.md) for conventions.

## SDK / developer installs

Components marked `sdk` in CMake (headers, `SEAStackConfig.cmake`, static/import libs) are **not** part of the default CPack `runtime` ZIP. They are for `cmake --install` without CPack component filtering or for custom packaging.
