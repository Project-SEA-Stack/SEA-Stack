# Building Individual SEA-Stack Modules

SEA-Stack is modular: you can build the full platform or only the libraries
you need. This guide explains the CMake flags that control which modules are
built and their dependency requirements.

---

## CMake options

| Flag | Default | Purpose |
|------|---------|---------|
| `SEASTACK_ENABLE_CHRONO` | ON | Chrono adapter, apps (`run_seastack`), and Chrono-dependent tests |
| `SEASTACK_ENABLE_HYDRO_IO` | ON | HDF5-based BEMIO import/export (`SEAStack::HydroIO`) |
| `SEASTACK_ENABLE_MOORING` | OFF | MoorDyn mooring module (`SEAStack::Mooring`) |
| `SEASTACK_ENABLE_EXTERNAL` | OFF | Out-of-process force modules (`SEAStack::External`) |
| `SEASTACK_ENABLE_VSG` | OFF | Vulkan Scene Graph 3D visualization |
| `SEASTACK_ENABLE_TESTS` | ON | Test executables |
| `SEASTACK_ENABLE_DEMOS` | OFF | C++ demo executables |
| `SEASTACK_ENABLE_APPS` | ON | Application executables (`run_seastack`) |

## Build profiles

### Full SEA-Stack (default)

Everything enabled, including Chrono adapter and `run_seastack`:

```
cmake -B build -DSEASTACK_ENABLE_CHRONO=ON -DChrono_DIR=/path/to/chrono
cmake --build build --config Release
```

With the build script:

```powershell
.\scripts\windows\build.ps1
```

**Dependencies:** CMake, C++17 compiler, Eigen3, Project Chrono (see [BUILD_CHRONO.md](BUILD_CHRONO.md) for required modules), HDF5 (for HydroIO).

### Hydro library only (no Chrono)

Build the hydrodynamic force libraries without any solver dependency:

```
cmake -B build -DSEASTACK_ENABLE_CHRONO=OFF
cmake --build build --config Release
```

With the build script:

```powershell
.\scripts\windows\build.ps1 -NoChrono
```

**What you get:** `SEAStack::Core`, `SEAStack::Hydro`, `SEAStack::HydroIO` (if HDF5 available), `SEAStack::PTO`, `SEAStack::Control`, `SEAStack::Infra`.

**Dependencies:** CMake, C++17 compiler, Eigen3, HDF5 (optional).

**Eigen note:** Without Chrono, Eigen is not auto-detected. Provide it via:
- System package manager (`apt install libeigen3-dev`, `brew install eigen`)
- `-DEIGEN3_INCLUDE_DIR=/path/to/eigen`

### Hydro without HDF5

If you do not need BEMIO HDF5 file I/O:

```
cmake -B build -DSEASTACK_ENABLE_CHRONO=OFF -DSEASTACK_ENABLE_HYDRO_IO=OFF
cmake --build build --config Release
```

With the build script:

```powershell
.\scripts\windows\build.ps1 -NoChrono -NoHydroIO
```

**What you get:** `SEAStack::Core`, `SEAStack::Hydro`, `SEAStack::PTO`, `SEAStack::Control`, `SEAStack::Infra`.

**Dependencies:** CMake, C++17 compiler, Eigen3 only.

### PTO and Control only

PTO and Control have no external dependencies beyond the C++ standard library.
A no-Chrono, no-HydroIO build includes them automatically:

```
cmake -B build -DSEASTACK_ENABLE_CHRONO=OFF -DSEASTACK_ENABLE_HYDRO_IO=OFF
cmake --build build --config Release
```

Link only the targets you need:

```cmake
find_package(SEAStack REQUIRED)
target_link_libraries(my_app PRIVATE SEAStack::PTO SEAStack::Control)
```

**Dependencies:** CMake, C++17 compiler. Eigen is still needed at configure
time for Core, but PTO and Control themselves do not use it.

### With mooring (MoorDyn)

MoorDyn is a **Git submodule** at `extern/MoorDyn` ([FloatingArrayDesign/MoorDyn](https://github.com/FloatingArrayDesign/MoorDyn)), pinned to a tested release tag. From the repository root, initialize it after clone:

```bash
git submodule update --init --recursive
```

(or `git clone --recursive` when cloning). Then configure with mooring enabled:

```
cmake -B build -DSEASTACK_ENABLE_MOORING=ON
cmake --build build --config Release
```

With the build script:

```powershell
.\scripts\windows\build.ps1 -MoorDyn
```

MoorDyn can be combined with any of the profiles above.

## Module dependency map

```
SEAStack::Core          (header-only)   → Eigen3
SEAStack::Infra         (static)        → C++ stdlib only
SEAStack::Hydro         (static)        → Core, Infra, OpenMP (optional)
SEAStack::HydroIO       (static)        → Hydro, Infra, HDF5
SEAStack::PTO           (static)        → C++ stdlib only
SEAStack::Control       (header-only)   → C++ stdlib only
SEAStack::External      (static, opt.)  → PTO, Core (TCP/JSON IPC for Python/MATLAB)
SEAStack::Mooring       (static)        → Core, Infra, MoorDyn
SEAStack::ChronoAdapter (static)        → Core, Hydro, PTO, Infra, Chrono
                                          optional: HydroIO, Mooring
```

## Tests by profile

| Profile | Available test labels |
|---------|----------------------|
| Full (Chrono ON) | `unit`, `regression`, `verification`, `comparison`, `benchmark` |
| No Chrono | `unit` (chrono-free subset only) |
| No HydroIO | `unit` (HDF5-dependent tests skipped) |

On Windows prefer `.\scripts\windows\run_unit_tests.ps1` (and
`run_regression_tests.ps1`, etc.); on macOS/Linux use
`./scripts/unix/ctest_suite.sh unit` (or `regression`, …). See
[`tests/README.md`](../../tests/README.md).

Raw CTest with a label filter:

```
ctest --test-dir build -C Release -L unit
```
