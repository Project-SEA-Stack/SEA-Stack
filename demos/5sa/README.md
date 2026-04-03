# 5SA C++ demos

Five-segment attenuator built by **`SetupFiveSaModel`** in
[`five_sa_model_setup.h`](five_sa_model_setup.h): universal joints, segment
meshes, TSDA dashpots between segments, and **`5sa_directional.h5`** for
hydrodynamics. **MoorDyn** couples selected bodies to lines under
`demos/5sa/mooring/`.

## Key parameters

| Item | Value / note |
|------|----------------|
| Hydro | `5sa_directional.h5` (directional BEM dataset in `demos/5sa/hydroData/`) |
| Mooring | Spreading demo: bodies **0 and 2** coupled; bimodal: **0 and 4** (see each `.cpp`) |
| Timestep | 0.02 s |
| Depth | 50 m (sea state) |

## Available demos

| Executable | Waves | Mooring bodies |
|------------|-------|----------------|
| `demo_5sa_spreading` | JONSWAP Hs 3 m, Tp 10 s, **cos²s** spreading (s = 12), 64×21 freq×direction grid | `{0, 2}` |
| `demo_5sa_bimodal` | Two partitions: swell (0°) + wind sea (90°), each with cos²s spreading | `{0, 4}` |

Both require **`SEASTACK_ENABLE_MOORING`**.

## What to tweak

- **`SeaStateDefinition`** (Hs, Tp, spreading, partition headings).
- Articulation and TSDA layout — `five_sa_model_setup.h`.
- **`simulationDuration`**, **`timestep`**, solver.

See [`data/demos/run_seastack/5sa/README.md`](../../data/demos/run_seastack/5sa/README.md).
