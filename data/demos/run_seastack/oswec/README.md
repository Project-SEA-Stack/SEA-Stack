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
| decay | Free decay in still water | `decay/oswec_decay.setup.yaml` |

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
