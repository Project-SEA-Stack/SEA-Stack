# Building SEA-Stack on Windows

Step-by-step guide for building SEA-Stack from source on Windows.

---

## Prerequisites

| Dependency | Version | Notes |
|------------|---------|-------|
| CMake | 3.21+ | [cmake.org](https://cmake.org/download/) or via Visual Studio installer |
| C++ compiler | MSVC 2019+ (C++17) | Visual Studio 2019/2022/2026 with "Desktop development with C++" workload |
| Eigen3 | 3.3+ | Auto-detected from Chrono if Chrono is enabled; otherwise set `EIGEN3_INCLUDE_DIR` |
| Project Chrono | v10+ | **Optional.** Required for `run_seastack` and time-domain simulation. Omit with `-NoChrono`. Build/install steps and required modules: [BUILD_CHRONO.md](BUILD_CHRONO.md) |
| HDF5 | 1.12+ | **Optional.** Required for HydroIO (BEMIO HDF5 import/export). Omit with `-NoHydroIO` |
| MoorDyn | Git submodule (`extern/MoorDyn`) | **Optional.** Enable with `-MoorDyn` (initialize submodules after clone; see below) |
| Vulkan Scene Graph | latest | **Optional.** 3D visualization. Enable with `-VSG` |

## 1. Clone the repository

```powershell
git clone --recursive https://github.com/Project-SEA-Stack/sea-stack.git
cd sea-stack
```

If you already cloned without `--recursive`, run this from the **repository root** (the folder that contains `.gitmodules`—for a standalone SEA-Stack clone, that is the same directory you `cd` into after `git clone`):

```powershell
git submodule update --init --recursive
```

## 2. Install HDF5 (optional, for HydroIO module)

If you need HDF5 file I/O (the default), install via vcpkg:

```powershell
vcpkg install hdf5[cpp]:x64-windows
```

If CMake reports **ZLIB** missing when configuring SEA-Stack, point CMake at the vcpkg **installed** tree for that triplet (example: set environment variable `CMAKE_PREFIX_PATH` to `C:\path\to\vcpkg\installed\x64-windows` in the same PowerShell session before `build.ps1`, or pass `-DCMAKE_PREFIX_PATH=...` if you configure CMake by hand).

If you don't need HDF5, skip this step and use `-NoHydroIO` when building.

## 3. Build and install Project Chrono (Chrono-enabled builds)

Skip this section if you use **`-NoChrono`**.

SEA-Stack expects an **installed** Chrono with the **Parsers** module enabled (and **VSG** only if you will pass **`-VSG`** to `build.ps1`). Use the same HDF5 stack as in §2 when you enable **`CH_ENABLE_HDF5`** in Chrono so HydroIO and Chrono stay aligned.

Concrete CMake options, `CMAKE_INSTALL_PREFIX`, and a Windows command-line example are in **[BUILD_CHRONO.md](BUILD_CHRONO.md)**. Set **`ChronoDir`** in the next step to the directory that contains **`ChronoConfig.cmake`** (typically `<install-prefix>/cmake`).

## 4. Configure `build-config.json`

Copy the example and edit paths to match your machine:

```powershell
copy build-config.example.json build-config.json
```

Edit `build-config.json`:

```json
{
    "ChronoDir": "C:/path/to/chrono-install/cmake",
    "PythonRoot": "C:/Users/you/miniforge3/envs/chrono-build",
    "HDF5Dir": "C:/path/to/vcpkg/installed/x64-windows/share/hdf5",
    "Generator": ""
}
```

- **`ChronoDir`** — path to the directory containing `ChronoConfig.cmake` or `chrono-config.cmake`. Leave empty for `-NoChrono` builds.
- **`HDF5Dir`** — path to vcpkg's HDF5 CMake config. Leave empty if HDF5 is not needed or will be auto-detected from Chrono.
- **`PythonRoot`** — Python environment used by Chrono. Leave empty if not applicable.
- **`Generator`** — CMake generator override (e.g. `"Ninja"`). Leave empty for the default (Visual Studio).

## 5. Build

### Basic build (Chrono + HydroIO)

```powershell
.\scripts\windows\build.ps1
```

### Full, clean build & package generation

```powershell
.\scripts\windows\build.ps1 -Clean -Verbose -Package -MoorDyn -VSG -Demos
```

### Domain libraries only (no Chrono)

```powershell
.\scripts\windows\build.ps1 -NoChrono
```

Builds Hydro, PTO, Control, Infrastructure, and HydroIO (if HDF5 is available). See [BUILD_MODULES.md](BUILD_MODULES.md) for more selective builds.

### Common build options

| Flag | Effect |
|------|--------|
| `-NoChrono` | Skip Chrono adapter and apps; build domain libraries only |
| `-NoHydroIO` | Disable HDF5 import/export |
| `-MoorDyn` | Enable MoorDyn mooring module |
| `-VSG` | Enable Vulkan Scene Graph visualization |
| `-Demos` | Build C++ demo executables |
| `-Package` | After build: `cmake --install` + CPack ZIP |
| `-Clean` | Remove build directory before configuring |
| `-Verbose` | Show full CMake and compiler output |
| `-Generator Ninja` | Use Ninja instead of Visual Studio |
| `-BuildType Debug` | Debug build (default: Release) |
| `-ConfigureOnly` | Configure without building |
| `-Doctor` | Run environment diagnostics |
| `-Help` | Show all options |

Run `.\scripts\windows\build.ps1 -Help` for the complete option list.

## 6. Verify the build

Run the CTest-labelled suites locally to confirm your build still matches
references and baselines. The labels below are what the `run_*_tests.ps1` scripts
and raw `ctest -L …` use. For packaged installs, pre-built demo checks, and
report/PDF behaviour, see [`tests/README.md`](../../tests/README.md) and
[`tests/TEST_SUITES_REFERENCE.md`](../../tests/TEST_SUITES_REFERENCE.md).

| CTest label | What it checks |
|-------------|----------------|
| `unit` | Library-level correctness (mostly fast; many tests are Chrono-free) |
| `regression` | Simulation output vs frozen SEA-Stack references (`ss_ref_*.txt`) |
| `verification` | Output vs external / multi-code data under `data/verification/` |
| `comparison` | Two SEA-Stack configurations compared to each other |
| `benchmark` | Wall-clock timing (performance; not a tight pass/fail correctness gate) |

### Run tests from a source build

Dedicated test scripts are included and can be run from a powershell:

```powershell
.\scripts\windows\run_regression_tests.ps1
```

Other suites: `run_unit_tests.ps1`, `run_chrono_free_tests.ps1`,
`run_verification_tests.ps1`, `run_comparison_tests.ps1`, `run_benchmarks.ps1`.
Run `-Help` on each for flags (`-Long`, `-j N`, `-NoPdf`, etc.).

Or invoke CTest directly yourself:

```powershell
cd build
ctest --test-dir . -C Release -L regression
```

### Optional: Pandoc and LaTeX for reports

If **pandoc** is on `PATH` and a LaTeX distribution is installed, the
regression, verification, and comparison scripts pass `--pdf` so report
generators also write a **PDF** alongside the Markdown report. Without
pandoc, reports are **Markdown-only** (figures still referenced or described
in the `.md`). Use `-NoPdf` to force Markdown-only even when pandoc is found
(e.g. LaTeX missing).

### Run a demo case

```powershell
build\bin\Release\run_seastack.exe data\demos\run_seastack\5sa\bimodal
```

Add `--nogui` to run without the 3D visualization window.

## 7. Package (optional)

`-Package` is already included in the **Full, clean build & package generation**
example in [section 5](#5-build). For a smaller change set, append it to whatever you
normally pass—for example `.\scripts\windows\build.ps1 -Package` after a basic
configure.

Output: `build\SEAStack-<version>-win64.zip`. See [PACKAGE_LAYOUT.md](PACKAGE_LAYOUT.md)
for the ZIP layout.

## 8. Troubleshooting

### Environment diagnostics

Run the built-in doctor to check your setup:

```powershell
.\scripts\windows\build.ps1 -Doctor
```

This checks CMake version, compiler availability, Chrono/HDF5 paths, and DLL accessibility.

### Common issues

| Problem | Solution |
|---------|----------|
| `Specify Chrono_DIR for Chrono adapter builds` | Set `ChronoDir` in `build-config.json` to point to Chrono's CMake config directory |
| HDF5 not found | Set `HDF5Dir` in `build-config.json`, or use `-NoHydroIO` |
| Eigen not found (no-Chrono build) | Set `-DEIGEN3_INCLUDE_DIR=C:/path/to/eigen` or install Eigen via vcpkg |
| Generator mismatch after changing options | Use `-Clean` to reconfigure from scratch |
| DLL not found at runtime | Ensure Chrono and HDF5 DLLs are on `PATH`, or run from the build output directory |

### Manual CMake (without build script)

```powershell
cmake -B build -DSEASTACK_ENABLE_CHRONO=ON -DChrono_DIR="C:/path/to/chrono/cmake"
cmake --build build --config Release
```

See [BUILD_MODULES.md](BUILD_MODULES.md) for the full set of CMake flags.
