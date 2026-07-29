# Building Project Chrono for SEA-Stack

SEA-Stack does **not** vendor Project Chrono. For `run_seastack`, the Chrono adapter, and Chrono-backed tests, you must build and **install** Chrono yourself, then point `ChronoDir` in `build-config.json` at the installed CMake package directory (the folder that contains `ChronoConfig.cmake` or `chrono-config.cmake`).

This page describes the **Chrono modules and CMake options** that match what SEA-Stack actually consumes. Official upstream instructions remain authoritative for toolchains, optional third-party libraries, and troubleshooting; see [Project Chrono](https://projectchrono.org/) and the Chrono repository documentation.

---

## Version

- Use **Chrono v10** or newer (aligned with `CHRONO_VERSION` / `find_package(Chrono)` in SEA-Stack).
- Pin to a **release tag** (e.g. `10.0.0`) for reproducible CI and user support.

---

## What SEA-Stack asks CMake for

From the SEA-Stack root `CMakeLists.txt`:

| CMake usage | Meaning |
|-------------|---------|
| `find_package(Chrono … COMPONENTS Parsers)` | **Parsers** is **required** whenever Chrono is enabled. It supplies `Chrono::Chrono_parsers` (YAML-driven workflows in `run_seastack`). |
| `COMPONENTS VSG` | Added only when `SEASTACK_ENABLE_VSG` is ON (e.g. `-VSG` on `build.ps1`). Your Chrono install must have been built with the **VSG module** enabled, or configure will fail. |
| `COMPONENTS Vehicle` | Added only when `SEASTACK_ENABLE_VEHICLE` is ON (e.g. `-Vehicle` on `build.ps1`). Used by the drivable-vehicle demos (e.g. `demo_vlfp_vehicle`); your Chrono install must have been built with the **Vehicle module** enabled, or configure will fail. |
| `COMPONENTS FSI FSI_SPH` | Added only when `SEASTACK_ENABLE_SPH` is ON. Enables SPH (smoothed-particle hydrodynamics) simulations in `run_seastack` via Chrono's FSI YAML parser. Your Chrono install must have been built with **`CH_ENABLE_MODULE_FSI_SPH=ON`** (which needs **CUDA**), or configure fails with a clear error. |

The adapter and apps also link **`Chrono::Chrono_core`**, **OpenMP**, and **yaml-cpp** (often exposed as `Chrono::yaml-cpp` from Chrono’s package config).

---

## Recommended Chrono configuration (minimal for SEA-Stack)

Enable only what you need. For a typical **headless or default GUI-off** SEA-Stack build:

| CMake option | Recommended | Notes |
|--------------|-------------|--------|
| `CH_ENABLE_MODULE_PARSERS` | **ON** | **Required** for SEA-Stack with Chrono. |
| `CH_ENABLE_HDF5` | **ON** if you use SEA-Stack **HydroIO** | Keeps HDF5 usage consistent between Chrono and `SEAStack::HydroIO`. Use the same HDF5 install (e.g. same vcpkg triplet) for both Chrono and SEA-Stack when possible. |
| `CH_ENABLE_OPENMP` | **ON** | SEA-Stack requires **OpenMP (C++)** when Chrono is enabled. |
| `CH_ENABLE_MODULE_VSG` | **ON** only if you plan to use **`-VSG`** | Large dependency chain (Vulkan Scene Graph). Otherwise leave **OFF**. |
| `CH_ENABLE_MODULE_VEHICLE` | **ON** only if you plan to use **`-Vehicle`** | Needed by the drivable-vehicle demos (`demo_vlfp_vehicle`). Also ship Chrono's `data/vehicle/` tree with the install (SEA-Stack stages the HMMWV subset it needs). Otherwise leave **OFF**. |
| `CH_ENABLE_MODULE_FSI` | **ON** only if you plan to use **SPH** | The FSI coupling layer; prerequisite for the SPH fluid solver. |
| `CH_ENABLE_MODULE_FSI_SPH` | **ON** only if you plan to use **SPH** (`SEASTACK_ENABLE_SPH`) | GPU SPH fluid solver. **Requires CUDA** (NVIDIA GPU + toolkit); there is no CPU fallback. Enabling it also builds `Chrono_fsisph` / `Chrono_fsisph_vsg` and compiles the SPH YAML parser into `Chrono_parsers`. |
| `BUILD_SHARED_LIBS` | **ON** (typical on Windows) | Matches common DLL-based deployments; SEA-Stack copies Chrono DLLs next to executables on Windows. |
| `BUILD_TESTING` / `BUILD_DEMOS` | **OFF** | Optional; speeds up Chrono builds. |

You do **not** need Irrlicht, Sensor, etc., for the stock SEA-Stack stack unless you extend it yourself. **Vehicle** is only needed for the optional drivable-vehicle demos (`-Vehicle`).

The SEA-Stack Chrono adapter links to **`Chrono::Chrono_core`** and **`Chrono::Chrono_parsers`** (plus **`Chrono::Chrono_vsg`** if you use `-VSG`, and **`Chrono::Chrono_vehicle`** / **`Chrono::Chrono_vehicle_vsg`** if you use `-Vehicle`).

---

## Install layout and `ChronoDir`

After `cmake --install` (or the `INSTALL` target in Visual Studio):

- Set **`ChronoDir`** to the directory containing **`ChronoConfig.cmake`** (often `<install-prefix>/cmake` on Windows/Linux).

SEA-Stack’s `build-config.example.json` uses an **install** prefix path such as `…/chrono-install/cmake`; the same idea applies if your layout places the package config only in a build tree.

---

## Windows example (Visual Studio + vcpkg HDF5 / Eigen)

Prerequisites (see [BUILD_WINDOWS.md](BUILD_WINDOWS.md)): Visual Studio C++ workload, CMake, and vcpkg packages such as `hdf5[cpp]:x64-windows` and Eigen if not supplied elsewhere.

Configure Chrono (adjust paths):

```powershell
cmake -S C:/path/to/chrono-src -B C:/path/to/chrono-build `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_INSTALL_PREFIX=C:/path/to/chrono-install `
  -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DBUILD_SHARED_LIBS=ON `
  -DCH_ENABLE_MODULE_PARSERS=ON `
  -DCH_ENABLE_HDF5=ON `
  -DCH_ENABLE_OPENMP=ON `
  -DCH_ENABLE_MODULE_VSG=OFF `
  -DBUILD_TESTING=OFF `
  -DBUILD_DEMOS=OFF
```

Build and install:

```powershell
cmake --build C:/path/to/chrono-build --config Release
cmake --install C:/path/to/chrono-build --config Release
```

Then in `build-config.json`:

```json
"ChronoDir": "C:/path/to/chrono-install/cmake"
```

If you do **not** use the vcpkg toolchain when configuring Chrono, you must still ensure **HDF5**, **ZLIB**, and other transitive dependencies are discoverable (e.g. `CMAKE_PREFIX_PATH` pointing at `vcpkg/installed/x64-windows`). Using **`CMAKE_TOOLCHAIN_FILE`** is the most reproducible approach on Windows.

---

## macOS / Linux example

Same logical options; install Eigen (and HDF5 if `CH_ENABLE_HDF5=ON`) via your package manager or vcpkg, then:

```bash
cmake -S /path/to/chrono-src -B /path/to/chrono-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/path/to/chrono-install \
  -DBUILD_SHARED_LIBS=ON \
  -DCH_ENABLE_MODULE_PARSERS=ON \
  -DCH_ENABLE_HDF5=ON \
  -DCH_ENABLE_OPENMP=ON \
  -DCH_ENABLE_MODULE_VSG=OFF \
  -DBUILD_TESTING=OFF \
  -DBUILD_DEMOS=OFF
cmake --build /path/to/chrono-build --parallel
cmake --install /path/to/chrono-build
```

Use **`-DCH_ENABLE_MODULE_VSG=ON`** only if you will build SEA-Stack with **`--vsg` / `-VSG`** and have installed Chrono’s VSG prerequisites per upstream docs.

---

## SPH (Chrono::FSI) simulations

SEA-Stack can run fully-resolved fluid simulations with Chrono's smoothed-particle hydrodynamics (SPH) solver, driven from Chrono's FSI YAML files. This is an alternative fidelity to the default linear potential-flow path.

Requirements:

- Chrono built with **`CH_ENABLE_MODULE_FSI=ON`** and **`CH_ENABLE_MODULE_FSI_SPH=ON`**.
- A **CUDA** toolkit (the SPH solver is GPU-only; there is no CPU fallback). CUDA 12.6+/13.x additionally require Chrono to be compiled with the conformant MSVC preprocessor (`/Zc:preprocessor`) for the SPH sources on Windows.
- SEA-Stack configured with **`-DSEASTACK_ENABLE_SPH=ON`** (add `-VSG` for run-time SPH visualization).

Add to the Chrono configure command:

```powershell
  -DCH_ENABLE_MODULE_FSI=ON `
  -DCH_ENABLE_MODULE_FSI_SPH=ON
```

At **run** time an NVIDIA GPU + driver is required. Packaged SEA-Stack builds bundle the CUDA runtime DLLs (`nvrtc`, `cublas`, `cusparse`) that `Chrono_fsisph.dll` depends on, but not the GPU driver itself.

See `data/demos/run_seastack/objectdrop_sph/` for a pure SPH case. For a **coupled** potential-flow hull + SPH deck-sloshing tank (setup with both `hydro_file:` and a `tank:` block), see `data/demos/run_seastack/wigley/sloshing/` (source tree; requires the same SPH Chrono/SEA-Stack flags and GPU). Notes on flotation mass rebalancing and SPH cost are in that demo’s README and [../../data/demos/run_seastack/README.md](../../data/demos/run_seastack/README.md).

---

## Checklist before configuring SEA-Stack

1. `ChronoDir` points at the folder with **`ChronoConfig.cmake`**.
2. Chrono built with **`CH_ENABLE_MODULE_PARSERS=ON`**.
3. **OpenMP** available to the SEA-Stack configure (same compiler family as Chrono).
4. If **HydroIO** is ON: HDF5 found for SEA-Stack (and compatible with Chrono if Chrono was built with HDF5).
5. If **`-VSG`**: Chrono was built with **`CH_ENABLE_MODULE_VSG=ON`** and VSG dependencies are installed.
6. If **`-Vehicle`**: Chrono was built with **`CH_ENABLE_MODULE_VEHICLE=ON`** and its `data/vehicle/` directory is present in the install.
7. If **SPH** (`SEASTACK_ENABLE_SPH`): Chrono was built with **`CH_ENABLE_MODULE_FSI=ON`** and **`CH_ENABLE_MODULE_FSI_SPH=ON`**, a **CUDA** toolkit was found, and the `Chrono_fsisph` / `Chrono_fsisph_vsg` libraries are present. An NVIDIA GPU + driver is required at **run** time.

Run **`.\scripts\windows\build.ps1 -Doctor`** (or **`./scripts/unix/build.sh --doctor`**) to sanity-check paths before a full configure.
