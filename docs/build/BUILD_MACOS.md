# Building SEA-Stack on macOS and Linux

Step-by-step guide for building SEA-Stack from source on macOS and Linux. The
same `build-config.json` schema and CMake options are shared with Windows; the
driver script is [`scripts/unix/build.sh`](../../scripts/unix/build.sh)
(counterpart to `scripts/windows/build.ps1`).

---

## Prerequisites

| Dependency | Version | Notes |
|------------|---------|-------|
| CMake | 3.21+ | macOS: `brew install cmake`. Linux: `sudo apt install cmake` or [cmake.org](https://cmake.org/download/) |
| C++ compiler | C++17 | macOS: Xcode Command Line Tools (`xcode-select --install`). Linux: GCC 9+ or Clang (`g++` / `clang++`) |
| Eigen3 | 3.3+ | macOS: `brew install eigen`. Linux: `sudo apt install libeigen3-dev`. Auto-detected from Chrono when Chrono is enabled; otherwise ensure system install or set `EIGEN3_INCLUDE_DIR` |
| Project Chrono | v10+ | **Optional.** Required for `run_seastack` and time-domain simulation. Omit with `--no-chrono`. Modules and CMake options: [BUILD_CHRONO.md](BUILD_CHRONO.md) |
| HDF5 | 1.12+ | **Optional.** Required for HydroIO (BEMIO HDF5 import/export). Omit with `--no-hydro-io` |
| MoorDyn | Git submodule (`extern/MoorDyn`) | **Optional.** Enable with `--moordyn` (initialize submodules after clone; see below) |
| Vulkan Scene Graph | latest | **Optional.** 3D visualization. Enable with `--vsg` |

## 1. Clone the repository

```bash
git clone --recursive https://github.com/Project-SEA-Stack/sea-stack.git
cd sea-stack
```

If you already cloned without `--recursive`, run this from the **repository root** (the folder that contains `.gitmodules`—for a standalone SEA-Stack clone, that is the same directory you `cd` into after `git clone`):

```bash
git submodule update --init --recursive
```

## 2. Install HDF5 (optional, for HydroIO module)

If you need HDF5 file I/O (the default), install a development package:

**macOS (Homebrew):**

```bash
brew install hdf5
```

**Ubuntu / Debian:**

```bash
sudo apt install libhdf5-dev
```

If you do not need HDF5, skip this step and use `--no-hydro-io` when building.

## 3. Build and install Project Chrono (Chrono-enabled builds)

Skip if you use **`--no-chrono`**.

You need an **installed** Chrono with the **Parsers** module (**VSG** only if you use **`--vsg`**). Match HDF5 to §2 when enabling **`CH_ENABLE_HDF5`** in Chrono. See **[BUILD_CHRONO.md](BUILD_CHRONO.md)** for CMake flags and install layout; set **`ChronoDir`** to the directory containing **`ChronoConfig.cmake`** (often `<install-prefix>/cmake`).

## 4. Configure `build-config.json`

Copy the example and edit paths to match your machine:

```bash
cp build-config.example.json build-config.json
```

Edit `build-config.json`:

```json
{
    "ChronoDir": "/path/to/chrono-install/cmake",
    "PythonRoot": "",
    "HDF5Dir": "",
    "Generator": ""
}
```

- **`ChronoDir`** — path to the directory containing `ChronoConfig.cmake` or `chrono-config.cmake`. Leave empty for `--no-chrono` builds.
- **`HDF5Dir`** — usually not needed; CMake finds HDF5 via `find_package` when installed from Homebrew or apt. If needed (see [Platform notes](#9-platform-notes)): e.g. `$(brew --prefix hdf5)/share/cmake/hdf5` on macOS.
- **`PythonRoot`** — Python environment used by Chrono. Leave empty if not applicable.
- **`Generator`** — CMake generator override (e.g. `"Ninja"`). Leave empty to use the script default (Ninja if on `PATH`).

See also `build-config.example.macos.json` in the repo root for a macOS-oriented template.

## 5. Build

### Basic build (Chrono + HydroIO)

```bash
./scripts/unix/build.sh
```

### Full, clean build & package generation

```bash
./scripts/unix/build.sh --clean --verbose --package --moordyn --vsg --demos
```

### Domain libraries only (no Chrono)

```bash
./scripts/unix/build.sh --no-chrono
```

Builds Hydro, PTO, Control, Infrastructure, and HydroIO (if HDF5 is available). See [BUILD_MODULES.md](BUILD_MODULES.md) for more selective builds.

### Common build options

| Flag | Effect |
|------|--------|
| `--no-chrono` | Skip Chrono adapter and apps; build domain libraries only |
| `--no-hydro-io` | Disable HDF5 import/export |
| `--moordyn` | Enable MoorDyn mooring module |
| `--vsg` | Enable Vulkan Scene Graph visualization |
| `--demos` | Build C++ demo executables |
| `--package` | After build: `cmake --install` + CPack ZIP |
| `--clean` | Remove build directory before configuring |
| `--verbose` | Show full CMake and compiler output |
| `--generator Ninja` | Use Ninja (or pass another CMake generator name) |
| `--build-type Debug` | Debug build (default: Release) |
| `--configure-only` | Configure without building |
| `--doctor` | Run environment diagnostics |
| `--help` | Show all options |

Run `./scripts/unix/build.sh --help` for the complete option list (`--test`, `--test-label`, `--reconfigure`, `--no-configure`, `--target`, etc.).

## 6. Verify the build

Run the CTest-labelled suites locally to confirm your build still matches
references and baselines. The labels below are what `ctest_suite.sh` and raw
`ctest -L …` use. For full detail (reports, PDFs, labels like `chrono-free`),
see [`tests/README.md`](../../tests/README.md) and
[`tests/TEST_SUITES_REFERENCE.md`](../../tests/TEST_SUITES_REFERENCE.md).

| CTest label | What it checks |
|-------------|----------------|
| `unit` | Library-level correctness (mostly fast; many tests are Chrono-free) |
| `regression` | Simulation output vs frozen SEA-Stack references (`ss_ref_*.txt`) |
| `verification` | Output vs external / multi-code data under `data/verification/` |
| `comparison` | Two SEA-Stack configurations compared to each other |
| `benchmark` | Wall-clock timing (performance; not a tight pass/fail correctness gate) |

### Run tests from a source build

Prefer **`ctest_suite.sh`**, which mirrors the Windows `run_*_tests.ps1`
scripts (correct `ctest -C`, parallelism, and post-suite report generation):

```bash
./scripts/unix/ctest_suite.sh regression
```

Other suites: `unit`, `chrono-free`, `verification`, `comparison`, `benchmark`.
Run `./scripts/unix/ctest_suite.sh --help` for options (`--no-pdf`, `-j N`,
`--long` for regression, etc.).

Quick build-and-test without the full report workflow:

```bash
./scripts/unix/build.sh --test
```

Or invoke CTest directly yourself:

```bash
cd build
ctest --test-dir . -C Release -L regression
```

### Optional: Pandoc and LaTeX for reports

If **pandoc** is on `PATH` and a LaTeX distribution is installed (TeX Live,
MacTeX, etc.), the suite scripts request **PDF** output alongside Markdown for
regression, verification, and comparison reports. Without pandoc, reports are
**Markdown-only** (figures still referenced or described in the `.md`). Use
`--no-pdf` to force Markdown-only even when pandoc is found (e.g. LaTeX
missing).

### Run a demo case

```bash
./build/bin/Release/run_seastack ./data/demos/run_seastack/5sa/bimodal
```

Add `--nogui` to run without the 3D visualization window.

## 7. Package (optional)

`--package` is already included in the **Full, clean build & package generation**
example in [section 5](#5-build). For a smaller change set, append it to whatever you
normally pass—for example `./scripts/unix/build.sh --package` after a basic
configure.

Output: `build/SEAStack-<version>-<platform>.zip` (CPack suffix is
`darwin-arm64`, `darwin-x64`, or `linux-x64` depending on OS and architecture).
See [PACKAGE_LAYOUT.md](PACKAGE_LAYOUT.md) for the archive layout.

For a full cut (version bump → package → tests → GitHub assets), see
[RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md).

## 8. Troubleshooting

### Environment diagnostics

Run the built-in doctor to check your setup:

```bash
./scripts/unix/build.sh --doctor
```

This checks CMake version, compiler availability, Chrono/HDF5 paths, and related configuration.

### Common issues

| Problem | Solution |
|---------|----------|
| `Specify Chrono_DIR` / Chrono not found | Set `ChronoDir` in `build-config.json` to Chrono's CMake config directory |
| HDF5 not found | Install HDF5 (Homebrew/apt), set `HDF5Dir` in `build-config.json` if needed, or use `--no-hydro-io` |
| Eigen not found (no-Chrono build) | Install Eigen (`brew` / `apt`) or pass `-DEIGEN3_INCLUDE_DIR=/path/to/eigen` |
| Generator mismatch after changing options | Use `--clean` to reconfigure from scratch |
| Shared library not found at runtime | Ensure Chrono/HDF5 libraries are discoverable (`DYLD_LIBRARY_PATH` / `LD_LIBRARY_PATH` as a last resort, or run from the build output tree) |

### Manual CMake (without build script)

```bash
cmake -B build \
    -DSEASTACK_ENABLE_CHRONO=ON \
    -DChrono_DIR=/path/to/chrono/cmake \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build
```

For a Chrono-free build:

```bash
cmake -B build \
    -DSEASTACK_ENABLE_CHRONO=OFF \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build
```

See [BUILD_MODULES.md](BUILD_MODULES.md) for the full set of CMake flags.

## 9. Platform notes

### macOS: OpenMP

Apple Clang does not ship OpenMP. SEA-Stack's CMake searches Homebrew paths
(`/opt/homebrew/opt/libomp`, `/usr/local/opt/libomp`). If OpenMP is not found,
the build proceeds without it (wave evaluation may be single-threaded).

```bash
brew install libomp
```

### macOS: HDF5 path hint

If CMake cannot find HDF5:

```bash
-DHDF5_DIR=$(brew --prefix hdf5)/share/cmake/hdf5
```

Or set `HDF5Dir` in `build-config.json` to the same path.

### macOS: Eigen via Homebrew

Eigen from `brew install eigen` is usually found automatically. For a custom
location:

```bash
-DEIGEN3_INCLUDE_DIR=/path/to/eigen
```

### Linux: Ubuntu / Debian

```bash
sudo apt install cmake g++ libeigen3-dev libhdf5-dev
```

CMake finds Eigen and HDF5 via system package paths.

### Linux: Eigen from Chrono

If building with Chrono, Eigen is often supplied via Chrono's includes. For
`--no-chrono` builds, install Eigen system-wide or pass `-DEIGEN3_INCLUDE_DIR`.
