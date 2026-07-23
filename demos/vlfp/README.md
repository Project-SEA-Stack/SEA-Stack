# VLFP demos (SEA-Stack + Chrono)

Very Large Floating Platform: six identical box pontoons (29 m x 58 m x 5.2 m
each, 174 m overall) connected by pitch hinges, in regular waves. Model
developed with the University of Strathclyde. The second demo puts a drivable
Chrono::Vehicle HMMWV on the deck.

See also the [C++ demos index](../README.md).

**Packaged YAML demo** (same platform model as `demo_vlfp_reg_waves`): in the
repo use `data/demos/run_seastack/vlfp/regular_waves/` (`run_seastack
…/demos/vlfp/regular_waves` in an install).

## Model (physical)

- **Pontoons**: six identical boxes, CoG at z = +1.6 m, draft 1.0 m, deck top
  at z = +4.2 m. Mass and inertia matched to the coupled BEM database
  `vlfp.h5` (all six bodies with cross-coupling terms).
- **Mechanism**: five revolute pitch hinges (axis +Y) on the deck centreline at
  x = 29, 58, 87, 116, 145 m; a soft surge TSDA (k = 1000 N/m) on pontoon 3
  restrains slow drift (not a physical mooring).

## Reference frame and units

- **World**: right-handed, **z up**, still-water level **z = 0**.
- **Wave heading**: **+X** (along the platform's long axis).
- **Units**: SI — meters, kilograms, seconds, newtons.

### Body indexing (must match `vlfp.h5`)

`HydroSystem` expects `hydro_bodies` in order `{ body1 … body6 }` with pontoon
centres at x = 14.5, 43.5, …, 159.5 m. Do not reorder without a new H5.

## Available demos

| Executable | What it shows |
|------------|----------------|
| `demo_vlfp_reg_waves` | Six hinged pontoons in regular waves (H = 1 m, T = 10 s); C++ twin of the YAML case. |
| `demo_vlfp_vehicle`¹ | HMMWV (from Chrono's JSON specs, rigid tires) driving on the pontoon decks while the platform responds to waves. WASD driving, chase camera, vehicle telemetry panel in the SEA-Stack window. |

¹ Built only when **`SEASTACK_ENABLE_VEHICLE`** and **`SEASTACK_ENABLE_VSG`**
are ON and the Chrono install provides the Vehicle module
(`CH_ENABLE_MODULE_VEHICLE=ON`; see
[BUILD_CHRONO.md](../../docs/build/BUILD_CHRONO.md)).

## Vehicle demo notes

- The pontoons carry box **collision shapes** (SMC) in a shared collision
  family, so tires contact the decks but adjacent pontoons never collide at the
  hinge lines. The tire radius (~0.47 m) comfortably bridges the hinge gaps.
- Driving is headless-safe: with `--no_gui` the driver is locked with full
  braking (a parked-vehicle smoke test), since the settled platform has a
  slight deck slope on the end pontoons.
- **Numerics**: the demo defaults to `EULER_IMPLICIT_LINEARIZED` + `SPARSE_LU`
  with **state-space radiation** (fitted from the same H5 kernels), which runs
  near realtime at dt = 1e-3 s. HHT and direct RIRF convolution remain
  selectable in the source for accuracy comparisons but are several times
  slower at the vehicle timestep. A further (unimplemented) option is TMeasy
  tires plus a custom `ChTerrain` that reports deck height from pontoon poses —
  no contact solve at all, but the deck is then only approximated per wheel.

## What to tweak

- **Waves**: height/period/ramp constants at the top of each demo.
- **Damping**: per-body linear/quadratic arrays (viscous corrections on top of
  the BEM data; same values as the YAML case).
- **Vehicle**: spawn pose, JSON spec paths (engine/transmission/tire), chase
  camera parameters at the top of `demo_vlfp_vehicle.cpp`.

## Hydrodynamic data

- **Path**: `<data_dir>/demos/vlfp/hydroData/vlfp.h5`
- Provenance and conventions: see
  [`data/demos/run_seastack/vlfp/assets/README.md`](../../data/demos/run_seastack/vlfp/assets/README.md).
