# Using SEA-Stack from a Downstream Project

This guide shows how to consume SEA-Stack libraries from a separate CMake
project. SEA-Stack exports CMake package config so that `find_package` works
out of the box after an SDK install.

---

## Prerequisites

1. Build and install SEA-Stack (see [BUILD_WINDOWS.md](../build/BUILD_WINDOWS.md)
   or [BUILD_MACOS.md](../build/BUILD_MACOS.md)).
2. Run the install step:

```
cmake --install build --config Release
```

This places headers, static libraries, and `SEAStackConfig.cmake` under the
install prefix (default: system prefix, or set with `-DCMAKE_INSTALL_PREFIX`).

## Available targets

After `find_package(SEAStack REQUIRED)`, the following imported targets are
available:

| Target | Type | Dependencies |
|--------|------|-------------|
| `SEAStack::Core` | INTERFACE (header-only) | Eigen3 |
| `SEAStack::Infra` | STATIC | C++ stdlib |
| `SEAStack::Hydro` | STATIC | Core, Infra, OpenMP (optional) |
| `SEAStack::HydroIO` | STATIC | Hydro, HDF5 |
| `SEAStack::PTO` | STATIC | C++ stdlib |
| `SEAStack::Control` | INTERFACE (header-only) | C++ stdlib |
| `SEAStack::Mooring` | STATIC | Core, Infra, MoorDyn |
| `SEAStack::ChronoAdapter` | STATIC | Core, Hydro, PTO, Infra, Chrono |

HydroIO, Mooring, and ChronoAdapter are only available if they were enabled
when SEA-Stack was built.

## Minimal example: standalone hydro evaluation

### Directory layout

```
my_project/
  CMakeLists.txt
  main.cpp
```

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.21)
project(my_hydro_app LANGUAGES CXX)

find_package(SEAStack REQUIRED)

add_executable(my_hydro_app main.cpp)
target_link_libraries(my_hydro_app PRIVATE SEAStack::HydroIO)
target_compile_features(my_hydro_app PRIVATE cxx_std_17)
```

Linking `SEAStack::HydroIO` transitively pulls in `SEAStack::Hydro`,
`SEAStack::Core`, Eigen3, and HDF5.

### `main.cpp`

```cpp
#include <seastack/hydro_io/h5_reader.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/core/system_state.h>

#include <iostream>

int main() {
    using namespace seastack::hydro;

    // Load BEM coefficients
    HydroData data = seastack::hydro_io::H5FileInfo("device.h5", 1).ReadH5Data();

    // Define a regular wave
    SeaStateDefinition sea_state;
    sea_state.type = "regular";
    sea_state.amplitude = 0.5;               // H/2 [m]
    sea_state.omega = 2.0 * 3.14159 / 8.0;  // T = 8 s

    // Build the force model
    HydroModel model = HydroModelBuilder()
        .FromHydroData(std::move(data))
        .WithSeaState(sea_state)
        .EnableHydrostatics()
        .EnableRadiation()
        .EnableExcitation()
        .Build();

    // Evaluate at equilibrium
    SystemState state;
    state.bodies.resize(1);
    // ... fill positions/velocities from your solver ...

    BodyForces forces = model.Evaluate(state, 0.0);
    std::cout << "Fz = " << forces[0].force.z() << " N\n";

    return 0;
}
```

### Build

```bash
cmake -B build -DSEAStack_DIR=/path/to/seastack/install/lib/cmake/SEAStack
cmake --build build --config Release
```

## Minimal example: PTO and Control only

PTO and Control have no external dependencies beyond the C++ standard library.

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.21)
project(my_pto_app LANGUAGES CXX)

find_package(SEAStack REQUIRED)

add_executable(my_pto_app main.cpp)
target_link_libraries(my_pto_app PRIVATE SEAStack::PTO SEAStack::Control)
target_compile_features(my_pto_app PRIVATE cxx_std_17)
```

### `main.cpp`

```cpp
#include <seastack/pto/linear_pto.h>
#include <seastack/control/controller.h>
#include <iostream>

int main() {
    seastack::pto::LinearPTO pto(500.0, 50.0);  // k=500 N/m, c=50 N.s/m

    double force = pto.ComputeForce(/*displacement=*/0.1, /*velocity=*/0.5, /*time=*/0.0);
    std::cout << "PTO force = " << force << " N\n";

    return 0;
}
```

## Linking only Hydro (no HDF5)

If you load hydrodynamic data through your own mechanism (not BEMIO HDF5),
link `SEAStack::Hydro` directly and construct `HydroData` programmatically:

```cmake
target_link_libraries(my_app PRIVATE SEAStack::Hydro)
```

This avoids the HDF5 dependency entirely.

## Passing the install prefix

If SEA-Stack is installed to a non-system location, tell CMake where to
find it:

```bash
cmake -B build -DSEAStack_DIR=/path/to/seastack/lib/cmake/SEAStack
```

Or set `CMAKE_PREFIX_PATH`:

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/seastack
```

## SDK install vs runtime ZIP

- **Runtime ZIP** (CPack): contains executables, demos, and DLLs. No headers
  or CMake config. Intended for end users running `run_seastack`.
- **SDK install** (`cmake --install`): contains headers, static/import
  libraries, and `SEAStackConfig.cmake`. Intended for developers linking
  SEA-Stack into their own projects.

See [PACKAGE_LAYOUT.md](../build/PACKAGE_LAYOUT.md) for the runtime ZIP
structure.

## Further reading

- [`examples/standalone_hydro/`](../../examples/standalone_hydro/) — complete
  runnable standalone hydro example
- [`examples/standalone_controller/`](../../examples/standalone_controller/) —
  PTO + Control without Chrono or Eigen
- [TECHNICAL_OVERVIEW.md](../../TECHNICAL_OVERVIEW.md) — architecture and
  module details
- [BUILD_MODULES.md](../build/BUILD_MODULES.md) — CMake flags for selective
  builds
