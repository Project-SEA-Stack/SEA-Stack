# OSWEC (oscillating surge WEC)

Bottom-hinged flap on a fixed base: pitch/surge-type motion about a horizontal
axis, representative of a classical oscillating-surge converter geometry.

## Key parameters

| Parameter | Value |
|-----------|-------|
| Flap mass | 127,000 kg |
| Base | Fixed to ground (LOCK joint) |
| Flap–base joint | Revolute (pitch about y), hinge near seabed |
| Initial flap orientation | 10° (see model YAML) |
| Visualization | `flap.obj`, `base.obj` |

## Available cases

| Case | Description | Setup YAML |
|------|-------------|------------|
| regular_waves | Regular wave excitation | `regular_waves/oswec_regular_waves.setup.yaml` |
| irregular_waves | JONSWAP irregular sea (long-crested with single-heading `oswec.h5`) | `irregular_waves/oswec_irregular_waves.setup.yaml` |
| divergence_limits | Free flap + raised roll/pitch blow-up threshold (`divergence:` in hydro YAML) | `divergence_limits/oswec_divergence_limits.setup.yaml` |
| translucent_hull | Still-water free flap with mesh `opacity` so the base stays visible in VSG | `translucent_hull/oswec_translucent_hull.setup.yaml` |
| decay | Free decay in still water | `decay/oswec_decay.setup.yaml` |
| external_pto | Irregular waves + out-of-process linear damper on RSDA `PTO` | `external_pto/oswec_external_pto.setup.yaml` |
| external_pto_adaptive | Same sea / RSDA; PI-adapted `c(t)` + torque saturation | `external_pto_adaptive/oswec_external_pto_adaptive.setup.yaml` |
| external_pto_hydraulic | Same sea / RSDA; hydraulic accumulator module | `external_pto_hydraulic/oswec_external_pto_hydraulic.setup.yaml` |

External-PTO cases need `-DSEASTACK_ENABLE_EXTERNAL=ON` and Python 3. They reuse
the same `PtoModule` scripts as the RM3 TSDA demos; config is in rotational
units (`c = 12e6` N·m·s/rad). See each case `README.md`.

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
