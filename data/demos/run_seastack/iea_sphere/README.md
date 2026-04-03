# IEA sphere

Floating hemisphere from the IEA OES Task 10 heave benchmark: a sphere
visualization (radius 5 m) on a heave-only prismatic constraint, used for
decay, irregular waves, and linear vs nonlinear hydrostatic comparisons.

## Key parameters

| Parameter | Value |
|-----------|-------|
| Sphere radius | 5.0 m |
| Body mass | 261,800 kg |
| Heave constraint | Prismatic joint to ground (z) |
| PTO / TSDA | k = 0, c = 0 (undamped for benchmark decay) |
| Initial heave / hydro setup | Set per case YAML |

## Available cases

| Case | Description | Setup YAML |
|------|-------------|------------|
| decay | Free decay, linear hydrostatics, 1 m drop | `decay/iea_sphere_decay.setup.yaml` |
| decay_ss | Free decay, state-space radiation | `decay_ss/iea_sphere_decay_ss.setup.yaml` |
| irregular_waves_ss | Irregular waves, state-space radiation | `irregular_waves_ss/iea_sphere_irregular_waves_ss.setup.yaml` |
| decay_nl_1m | Nonlinear hydrostatics (mesh-based buoyancy), 1 m drop | `decay_nl_1m/iea_sphere_decay_nl_1m.setup.yaml` |
| decay_nl_5m | Nonlinear hydrostatics, 5 m drop | `decay_nl_5m/iea_sphere_decay_nl_5m.setup.yaml` |
| decay_lin_5m | Linear hydrostatics, 5 m drop | `decay_lin_5m/iea_sphere_decay_lin_5m.setup.yaml` |

Nonlinear cases use instantaneous submerged volume from the OBJ mesh; 1 m and
5 m amplitudes illustrate linear vs large-displacement behavior.

## Optional analysis

Overlay linear vs nonlinear heave after running the decay cases:

```
python demos\iea_sphere\compare_lin_vs_nl.py demos\iea_sphere
```

Plots are written under `demos/iea_sphere/outputs/plots/`.

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
