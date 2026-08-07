# SEA-Stack Demo Cases

This folder contains runnable YAML-based demo cases for SEA-Stack.
Each model has its own directory with shared assets (geometry, hydro data)
and one or more case configurations.

## Running a demo

From the **packaged release** root:

```
bin\run_seastack.exe demos\rm3\irregular_waves\rm3_irregular_waves.setup.yaml --nogui
```

If running from a **source build**, use the build-tree paths instead:

```
build\bin\Release\run_seastack.exe data\demos\run_seastack\rm3\irregular_waves\rm3_irregular_waves.setup.yaml --nogui
```

Output files are written to an `outputs/` subdirectory next to the setup YAML.

**Release ZIP note:** Some folders exist only in the full source repository (e.g. RM3 `regular_waves`, F3OF `decay_dt1` / `decay_dt2`). The downloadable runtime package omits those to keep the bundle focused; see each model’s `README.md`.

## Available demos

Each model directory has a `README.md` with a short description, **key
parameters**, and an **available cases** table (paths below are relative to
`run_seastack/`).

| Model | Case | Description | Setup YAML |
|-------|------|-------------|------------|
| 5sa | regular_waves | 5SA attenuator, regular waves | `5sa/regular_waves/5sa_regular.setup.yaml` |
| 5sa | irregular_waves | 5SA attenuator, irregular waves | `5sa/irregular_waves/5sa_irregular.setup.yaml` |
| 5sa | mooring | 5SA attenuator with MoorDyn mooring lines | `5sa/mooring/5sa_mooring.setup.yaml` |
| 5sa | bimodal | 5SA attenuator, bimodal sea state | `5sa/bimodal/5sa_bimodal.setup.yaml` |
| 5sa | spreading | 5SA attenuator, directional spreading | `5sa/spreading/5sa_spreading.setup.yaml` |
| 5sa | power_matrix | Batch / AEP-style inputs | `5sa/power_matrix/` (see `README.md` there) |
| rm3 | irregular_waves | RM3 WEC, JONSWAP irregular sea, linear PTO | `rm3/irregular_waves/rm3_irregular_waves.setup.yaml` |
| rm3 | external_pto* | Irregular waves + out-of-process Python PTO on TSDA (linear / adaptive / hydraulic) | `rm3/external_pto*/` |
| rm3 | mooring | RM3 WEC with MoorDyn mooring lines, irregular waves (eta import) | `rm3/mooring/rm3_mooring.setup.yaml` |
| rm3 | decay | RM3 two-body free decay in still water (minimal) | `rm3/decay/rm3_decay.setup.yaml` |
| rm3 | regular_waves | *(source repo only; omitted from release ZIP)* | `rm3/regular_waves/rm3_regular_waves.setup.yaml` |
| iea_sphere | decay | IEA sphere, free decay (RIRF convolution) | `iea_sphere/decay/iea_sphere_decay.setup.yaml` |
| iea_sphere | decay_ss | IEA sphere, free decay with state-space radiation | `iea_sphere/decay_ss/iea_sphere_decay_ss.setup.yaml` |
| iea_sphere | irregular_waves_ss | IEA sphere, irregular waves with state-space radiation | `iea_sphere/irregular_waves_ss/iea_sphere_irregular_waves_ss.setup.yaml` |
| iea_sphere | decay_nl_1m | IEA sphere, nonlinear hydrostatics, 1 m drop | `iea_sphere/decay_nl_1m/iea_sphere_decay_nl_1m.setup.yaml` |
| iea_sphere | decay_nl_5m | IEA sphere, nonlinear hydrostatics, 5 m drop | `iea_sphere/decay_nl_5m/iea_sphere_decay_nl_5m.setup.yaml` |
| iea_sphere | decay_lin_5m | IEA sphere, linear hydrostatics, 5 m drop | `iea_sphere/decay_lin_5m/iea_sphere_decay_lin_5m.setup.yaml` |
| oswec | regular_waves | OSWEC flap, regular waves | `oswec/regular_waves/oswec_regular_waves.setup.yaml` |
| oswec | irregular_waves | OSWEC flap, JONSWAP irregular sea | `oswec/irregular_waves/oswec_irregular_waves.setup.yaml` |
| oswec | external_pto* | Irregular waves + out-of-process Python PTO on RSDA (linear / adaptive / hydraulic) | `oswec/external_pto*/` |
| oswec | decay | OSWEC flap free decay in still water (minimal) | `oswec/decay/oswec_decay.setup.yaml` |
| f3of | irregular_waves | F3OF platform, JONSWAP irregular sea, surge restraint | `f3of/irregular_waves/f3of_irregular_waves.setup.yaml` |
| f3of | decay_dt3 | F3OF platform, DT3 flap pitch decay (base locked) | `f3of/decay_dt3/f3of_decay_dt3.setup.yaml` |
| f3of | decay_dt1, decay_dt2 | *(source repo only; omitted from release ZIP)* | `f3of/decay_dt1/…`, `f3of/decay_dt2/…` |
| trimaran | rigid | Three-hull trimaran with rigid cross-arms, irregular waves | `trimaran/model.setup.yaml` or `trimaran/rigid/trimaran_rigid.setup.yaml` |
| ship2ship | transfer_hulls | Two coupled Wigley hulls, beam seas (hydro-only precursor) | `ship2ship/transfer_hulls/ship2ship_transfer_hulls.setup.yaml` |
| ship2ship | transfer | Two coupled hulls, FEA linkspan bridge, HMMWV crossing in beam seas | `ship2ship/transfer/ship2ship_transfer.setup.yaml` |
| rov | crawler_drive | Support vessel + tracked crawler on lazy-wave MoorDyn umbilical | `rov/crawler_drive/crawler_drive.setup.yaml` |

**Note:** RM3 `irregular_waves` and OSWEC `irregular_waves` use **long-crested** seas with single-heading BEMIO data (`rm3.h5`, `oswec.h5`). For **bimodal** or **directional spreading** examples, see **5sa** `bimodal` / `spreading` and scripts under `5sa/assets/` and `wigley/assets/`.

## Directory layout

Each model follows this structure:

```
<model>/
  assets/           Shared geometry (.obj) and hydro data (.h5)
  <case>/           One directory per demo case
    <name>.setup.yaml
    <name>.model.yaml
    <name>.simulation.yaml
    <name>.hydro.yaml
    expected/       Reference baseline output (if available)
  signal_adapter.py Test signal extraction (used by test runner)
```

## Running the automated test suite

The `tests\` folder (at the package root) contains Python scripts that run
these same demos headless and compare the output against expected baselines.
From the package root (Python 3.10+):

```
tests\RUN-TESTS.ps1
```

On macOS / Linux use `./tests/RUN-TESTS.sh` instead.

See [`tests/README.md`](../../../tests/README.md) for verifying installs and
running test suites; per-demo flags for `RUN-TESTS.ps1` are documented on this page
and in [`tests/regression/run_seastack/README.md`](../../../tests/regression/run_seastack/README.md).
