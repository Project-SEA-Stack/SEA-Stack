# Sphere (IEA-style) C++ demos

Single floating body (**`body1`**) with heave-only dynamics and **still water**,
using the sphere mesh and **`sphere.h5`** from the IEA OES Task 10-style setup
(copied under `demos/sphere/` from `run_seastack/iea_sphere` assets).

## Key parameters

| Parameter | Value |
|-----------|--------|
| Mass | 261,800 kg |
| Initial heave z | −1.0 m |
| Hydro file | `demos/sphere/hydroData/sphere.h5` |
| Mesh | `demos/sphere/geometry/sphere.obj` |

## Available demos

| Executable | Radiation | Notes |
|------------|-----------|--------|
| `demo_sphere_decay` | Convolution (default / RIRF path in `HydroSystem`) | Optional Gnuplot output when enabled in CLI |
| `demo_sphere_decay_ss` | **State-space** (`RadiationMethod::kStateSpace`, fitted order ≤ 10, R² threshold 0.99) | Compare speed/stability vs convolution |

## What to tweak

- **`timestep`**, **`simulationDuration`**, linear solver (`SPARSE_QR` in both).
- State-space: `StateSpaceOptions` in `demo_sphere_decay_ss.cpp` (`max_order`,
  `r2_threshold`).

See also [`data/demos/run_seastack/iea_sphere/README.md`](../../data/demos/run_seastack/iea_sphere/README.md).
