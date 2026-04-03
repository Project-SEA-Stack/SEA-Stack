# Trimaran demo (SEA-Stack + Chrono)

Three-body trimaran (center hull + port and starboard outriggers) in irregular waves, with optional rigid or FEA cross-arms. This folder is a **learning progression**: start with hydro-only, then add mechanisms.

See also the [C++ demos index](../README.md).

**Packaged YAML demo** (same rigid-arm model as `demo_trimaran_rigid`): in the repo use
`data/demos/run_seastack/trimaran/` (`run_seastack …/demos/trimaran` in an install).

## Model (physical)

- **Hull form**: Wigley-style meshes (`center.obj`, `outrigger.obj`) with mass and inertia matched to the BEM hydrodynamic database in `trimaran.h5`.
- **Mechanism**: Cross-arms connect the center hull deck to each outrigger. The **rigid** demo uses a revolute at the center and a rigid weld at the tip. The **FEA** demo uses the **same topology** (revolute + tip weld) but replaces each rigid arm with an Euler beam (lumped root body + beam mesh + `ChLinkMateGeneric` clamps). There is no PTO/RSDA in the current FEA demo.

## Reference frame and units

- **World**: right-handed, **z up**, nominal waterline **z = 0**.
- **Wave / surge heading**: **+X** (forward along the ship).
- **Nautical lateral** (facing +X): **+Y = port (left)**, **−Y = starboard (right)**.
- **Units**: SI — meters, kilograms, seconds, newtons (forces), N·m (moments).

### Hull indexing (must match `trimaran.h5`)

| ChBody name | H5 group | CG y (world) | Role        |
|-------------|----------|--------------|-------------|
| `body1`     | body1    | 0            | Center hull |
| `body2`     | body2    | −Y           | Starboard   |
| `body3`     | body3    | +Y           | Port        |

`HydroSystem` expects `hydro_bodies` in order `{ body1, body2, body3 }`. Do not swap outrigger poses without regenerating the H5.

### Key geometry (see `trimaran_hulls.h`)

- Outrigger spacing: ±15 m in **y** (half-beam and deck heights are named constants in the header).

## Available demos (read in this order)

| Executable | What it shows |
|------------|----------------|
| `demo_trimaran_hydro` | Three free hulls, frequency-domain excitation + irregular sea; simplest integration path. |
| `demo_trimaran_rigid` | Rigid cross-arms (revolute + tip weld); linear + quadratic hydrodynamic damping. |
| `demo_trimaran_fea` | Same hydro + damping + dt as rigid; flexible Euler beams instead of rigid arm boxes (no RSDA). |

Each `.cpp` file is written to be read **top to bottom**: simulation parameters, solver notes, model setup, wave field, `HydroSystem`, main loop, text output.

## Build and run

After configuring the sea-stack build with demos enabled, targets are:

- `demo_trimaran_hydro`
- `demo_trimaran_rigid`
- `demo_trimaran_fea`

Runtime data (meshes, `trimaran.h5`) is copied under `<data>/demos/trimaran/` by CMake. Use the same CLI pattern as other SEA-Stack Chrono demos (e.g. `--data_dir`, visualization flags) via `GetCLIArguments` / `SetInitialEnvironment` in each source file.

## Sea state

All three demos share `MakeTrimaranDemoIrregularSea()` in `trimaran_sea_state.h` (JONSWAP-style irregular sea with directional spreading). The BEM file currently supports a **single wave heading**; spreading affects the free surface visualization more than multi-directional excitation until the H5 is regenerated with multiple headings.

## What to tweak

- **Sea state**: `trimaran_sea_state.h` — Hs, Tp, depth, seed, component counts.
- **Timestep**: top of each `demo_trimaran_*.cpp` (hydro uses a finer dt than rigid/FEA arms).
- **Damping**: quadratic and linear arrays at the top of each demo (body frame; see comments in code).
- **Arms**: rigid box geometry in `demo_trimaran_rigid.cpp`; beam section (E/G, OD, element count) in `demo_trimaran_fea.cpp` (current FEA demo uses ~200 MPa stiff-rubber–scale E/G, not steel).
- **Solver**: HHT and linear solver choices are inlined in each demo with short comments.

## Hydrodynamic data

- **Path**: `<data_dir>/demos/trimaran/hydroData/trimaran.h5`
- Hull **mass, inertia, and CoG z** in `trimaran_hulls.h` are aligned with the BEM export; change them together with the H5 if you alter the panel model.
