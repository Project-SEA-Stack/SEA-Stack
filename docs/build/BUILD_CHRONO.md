# Building Project Chrono for SEA-Stack

SEA-Stack does **not** vendor Project Chrono. For `run_seastack`, the Chrono adapter, and Chrono-backed tests, you must build and **install** Chrono yourself, then point `ChronoDir` in `build-config.json` at the installed CMake package directory (the folder that contains `ChronoConfig.cmake` or `chrono-config.cmake`).

This page describes the **Chrono modules and CMake options** that match what SEA-Stack actually consumes. Official upstream instructions remain authoritative for toolchains, optional third-party libraries, and troubleshooting; see [Project Chrono](https://projectchrono.org/) and the Chrono repository documentation.

---

## Version

- SEA-Stack v1.0.0 is tested with and requires **Chrono 10.0.0** (release tag `10.0.0`).
- Newer Chrono versions, including Chrono `main`, are not supported: they already contain API changes that break the SEA-Stack build. CMake configure prints a warning if Chrono reports a version other than 10.0.0, but it does not stop the build, and a Chrono `main` build can still report 10.0.0. Pin the tag yourself.

---

## What SEA-Stack asks CMake for

From the SEA-Stack root `CMakeLists.txt`:

| CMake usage | Meaning |
|-------------|---------|
| `find_package(Chrono … COMPONENTS Parsers)` | **Parsers** is **required** whenever Chrono is enabled. It supplies `Chrono::Chrono_parsers` (YAML-driven workflows in `run_seastack`). |
| `COMPONENTS VSG` | Added only when `SEASTACK_ENABLE_VSG` is ON (e.g. `-VSG` on `build.ps1`). Your Chrono install must have been built with the **VSG module** enabled, or configure will fail. |

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
| `BUILD_SHARED_LIBS` | **ON** (typical on Windows) | Matches common DLL-based deployments; SEA-Stack copies Chrono DLLs next to executables on Windows. |
| `BUILD_TESTING` / `BUILD_DEMOS` | **OFF** | Optional; speeds up Chrono builds. |

You do **not** need Irrlicht, Vehicle, Sensor, etc., for the stock SEA-Stack stack unless you extend it yourself.

The SEA-Stack Chrono adapter links to **`Chrono::Chrono_core`** and **`Chrono::Chrono_parsers`** (and **`Chrono::Chrono_vsg`** if you use `-VSG`).

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

### macOS release packages

If you will build a redistributable macOS ZIP (`--package`), add these two options to the Chrono configure:

```bash
  -DCMAKE_DISABLE_FIND_PACKAGE_Python3=ON \
  -DCH_USE_EIGEN_OPENMP=OFF
```

- **`CMAKE_DISABLE_FIND_PACKAGE_Python3=ON`**: Chrono's Parsers module auto-detects Python and, if found, links `libChrono_parsers` against it (on the release build Mac it found the Command Line Tools Python 3.9). SEA-Stack only uses the YAML parser, but that link would make the package require the build machine's Python install. With this option `CHRONO_HAS_PYTHON` is undefined in `ChConfigParsers.h`, and no Chrono library links Python.
- **`CH_USE_EIGEN_OPENMP=OFF`**: the macOS v1.0.0 Chrono libraries were built with Eigen's internal OpenMP parallelism off (`EIGEN_DONT_PARALLELIZE`), as in the earlier release candidates. Keep it off so the package matches the tested build.

Use a separate Chrono build directory for this configuration and point `ChronoDir` at it. The SEA-Stack packaging step aborts if any bundled library still references a path outside the package.

---

## Checklist before configuring SEA-Stack

1. `ChronoDir` points at the folder with **`ChronoConfig.cmake`**.
2. Chrono built with **`CH_ENABLE_MODULE_PARSERS=ON`**.
3. **OpenMP** available to the SEA-Stack configure (same compiler family as Chrono).
4. If **HydroIO** is ON: HDF5 found for SEA-Stack (and compatible with Chrono if Chrono was built with HDF5).
5. If **`-VSG`**: Chrono was built with **`CH_ENABLE_MODULE_VSG=ON`** and VSG dependencies are installed.

Run **`.\scripts\windows\build.ps1 -Doctor`** (or **`./scripts/unix/build.sh --doctor`**) to sanity-check paths before a full configure.
