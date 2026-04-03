# OSWEC C++ demo

Bottom-mounted **flap** (`body1`) and heavy **base** (`body2`) with the base
welded to ground. **Pitch** about a horizontal axis via **`ChLinkLockRevolute`**.
Irregular **JONSWAP** excitation through `ComponentSampler` and
`LinearDirectionalWaveField`.

## Key parameters

| Parameter | Value |
|-----------|--------|
| Flap mass | 127,000 kg |
| Base | ~fixed (mate lock to ground) |
| Wave ramp | 60 s |
| Sea state | Hs 2 m, Tp 8 s, γ 3.3, 200 frequencies, seed 42 |
| Timestep | 0.03 s |
| Hydro | `demos/oswec/hydroData/oswec.h5` |

## Available demos

| Executable | Description |
|------------|-------------|
| `demo_oswec_irreg_waves` | Full hydro + irregular waves; logs time series (e.g. flap pitch) to CSV under demo output dir |

## What to tweak

- Flap initial pose, **`simulationDuration`**, **`timestep`**, solver (**GMRES**).
- **`SeaStateDefinition`** at bottom of `demo_oswec_irreg_waves.cpp`.

Helper script: `compute_flap_pose.py` (pose utilities for this geometry).

See [`data/demos/run_seastack/oswec/README.md`](../../data/demos/run_seastack/oswec/README.md).
