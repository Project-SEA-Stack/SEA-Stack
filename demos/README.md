# SEA-Stack C++ demos

Small Chrono executables that exercise `HydroSystem`, waves, mooring, and PTO
adapters directly in code. They complement the YAML-driven
[`run_seastack`](../apps/seastack/) app and the packaged cases under
[`data/demos/run_seastack/`](../data/demos/run_seastack/).

## Build

Demos are built when **Chrono** and **`SEASTACK_ENABLE_DEMOS`** are **ON** (see
root `CMakeLists.txt`). Some targets also require **`SEASTACK_ENABLE_MOORING`**.

Runtime geometry and `*.h5` files are copied under
`<build-or-install>/data/demos/<model>/` via CMake `file(COPY …)` from
`data/demos/run_seastack/*/assets/`.

## CLI and output

Each demo uses `GetCLIArguments` / `SetInitialEnvironment` from
`adapters/chrono` (e.g. `--data_dir`, visualization toggles). Text/CSV output
usually goes under `GetDemoOutDir()` in a subfolder named after the demo group
(`RESULTS_DIR_NAME`).

## Index

| Folder | Executables | Topic |
|--------|-------------|--------|
| [rm3](rm3/README.md) | `demo_rm3_hydraulic_pto`, `demo_rm3_irreg_mooring`¹ | Hydraulic PTO + PI control vs linear damper + MoorDyn |
| [sphere](sphere/README.md) | `demo_sphere_decay`, `demo_sphere_decay_ss` | Heave decay, convolution vs state-space radiation |
| [oswec](oswec/README.md) | `demo_oswec_irreg_waves` | Hinged flap, irregular JONSWAP |
| [f3of](f3of/README.md) | `demo_f3of_irreg_waves` | Three-body flaps, surge TSDA restraint |
| [5sa](5sa/README.md) | `demo_5sa_spreading`, `demo_5sa_bimodal`¹ | Articulated attenuator, directional / bimodal sea + mooring |
| [trimaran](trimaran/README.md) | `demo_trimaran_hydro`, `demo_trimaran_rigid`, `demo_trimaran_fea` | Three-hull hydro → rigid arms → FEA arms |

¹ Built only when `SEASTACK_ENABLE_MOORING` is enabled.
