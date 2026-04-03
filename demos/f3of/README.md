# F3OF C++ demo

**Three-body** platform: central **base** with **fore** and **aft** flaps on
**revolute** hinges. **Surge** is restrained with a **TSDA** spring-damper to
**ground** (station-keeping), not MoorDyn. Uses **`ChSystemSMC`**.

## Key parameters

| Body | Mass | Role |
|------|------|------|
| body1 (base) | 1,089,825 kg | Surge TSDA to ground: k = 10⁵ N/m, c = 10⁴ N·s/m |
| body2 / body3 (flaps) | 179,250 kg each | Pitch hinges at x = ±12.5 m |

| Sea / numerics | Value |
|----------------|--------|
| JONSWAP | Hs 2 m, Tp 8 s, γ 3.3, seed 42, 200 components |
| Ramp | 60 s |
| Timestep | 0.02 s |
| Hydro | `demos/f3of/hydroData/f3of.h5` |

## Available demos

| Executable | Description |
|------------|-------------|
| `demo_f3of_irreg_waves` | Irregular waves; logs base surge and flap pitches |

## What to tweak

- TSDA **stiffness/damping** for station-keeping strength.
- Sea parameters and **`simulationDuration`** in `demo_f3of_irreg_waves.cpp`.

See [`data/demos/run_seastack/f3of/README.md`](../../data/demos/run_seastack/f3of/README.md).
