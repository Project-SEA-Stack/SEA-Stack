# F3OF (floating three-body oscillating flap)

Three-body platform: central base with fore and aft flaps on pitch hinges.
Irregular-wave cases use a surge station-keeping TSDA to ground; decay cases
isolate flap modes with selective locking.

## Key parameters

| Parameter | Value |
|-----------|-------|
| Base mass | 1,089,825 kg |
| Each flap mass | 179,250 kg |
| Flap hinge spacing | ±12.5 m from base center (x) |
| Surge restraint (irregular_waves) | TSDA to ground, k = 10⁵ N/m, c = 10⁴ N·s/m |
| Irregular sea (irregular_waves) | JONSWAP Hs = 2 m, Tp = 8 s, γ = 3.3, seed 42, 60 s ramp |

## Available cases

| Case | Description | Setup YAML |
|------|-------------|------------|
| irregular_waves | JONSWAP irregular sea, surge station-keeping | `irregular_waves/f3of_irregular_waves.setup.yaml` |
| decay_dt3 | DT3: aft flap pitch decay, base locked | `decay_dt3/f3of_decay_dt3.setup.yaml` |
| decay_dt1 | DT1: surge decay, flaps locked *(source repo; omitted from release ZIP)* | `decay_dt1/f3of_decay_dt1.setup.yaml` |
| decay_dt2 | DT2: pitch decay, flaps locked *(source repo; omitted from release ZIP)* | `decay_dt2/f3of_decay_dt2.setup.yaml` |

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
