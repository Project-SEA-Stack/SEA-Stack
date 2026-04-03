# RM3 (Reference Model 3)

Two-body point-absorber WEC: a float and a submerged reaction plate connected
by a prismatic (heave) joint with a linear spring-damper PTO. Based on the
reference RM3 geometry and mass distribution used in marine energy benchmarks.

## Key parameters

| Parameter | Value |
|-----------|-------|
| Float mass | 725,834 kg |
| Plate mass | 886,691 kg |
| PTO | Prismatic joint (heave); TSDA k = 0, c = 1.2×10⁶ N·s/m |
| Visualization | `float_cog.obj`, `plate_cog.obj` (see `assets/geometry/`) |

## Available cases

| Case | Description | Setup YAML |
|------|-------------|------------|
| irregular_waves | JONSWAP irregular sea, linear PTO (long-crested with single-heading `rm3.h5`) | `irregular_waves/rm3_irregular_waves.setup.yaml` |
| mooring | Irregular waves, MoorDyn lines, imported surface elevation | `mooring/rm3_mooring.setup.yaml` |
| decay | Two-body free decay in still water, no PTO damping | `decay/rm3_decay.setup.yaml` |
| decay_nl | Decay with nonlinear hydrostatics on the float | `decay_nl/rm3_decay_nl.setup.yaml` |
| regular_waves | Regular waves, linear PTO | `regular_waves/rm3_regular_waves.setup.yaml` |

**Release package:** `regular_waves` is usually omitted from the downloadable ZIP; use `irregular_waves` as the primary wave example there.

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
