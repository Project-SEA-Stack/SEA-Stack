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
| regular_waves_nl | Calm-water nonlinear hydrostatics (`wetted_surface: calm_water`), regular waves H = 2 m, T = 8 s. Not in the v1.0.0 package | `regular_waves_nl/iea_sphere_regular_waves_nl.setup.yaml` |
| regular_waves_lin | Linear hydrostatics, same waves (twin of `regular_waves_nl`). Not in the v1.0.0 package | `regular_waves_lin/iea_sphere_regular_waves_lin.setup.yaml` |
| regular_waves_nlfk | Nonlinear Froude-Krylov (`wetted_surface: instantaneous`), regular waves H = 6 m, T = 8 s, optimum PTO (633,980 N s/m). Not in the v1.0.0 package | `regular_waves_nlfk/iea_sphere_regular_waves_nlfk.setup.yaml` |

Nonlinear cases use instantaneous submerged volume from the OBJ mesh; 1 m and
5 m amplitudes illustrate linear vs large-displacement behavior.

## Calm-water nonlinear hydrostatics in waves

`wetted_surface: calm_water` (the same as v1.0.0 `hydrostatics: nonlinear`)
keeps buoyancy against the calm waterplane in waves. `regular_waves_nl` uses
it to show the limits of that model.

Calm-water nonlinear hydrostatics computes buoyancy against the waterplane z = 0.
Wave excitation stays linear and already includes the linear Froude-Krylov
force on the mean hull. The two are consistent only while heave and relative
motion stay well below the draft and freeboard (5 m for this sphere).

`regular_waves_nl` sits inside that range: its steady heave amplitude is about
2% above `regular_waves_lin`. The comparison test
`test_compare_linear_vs_nonlinear_hs_sphere_reg` shows the trend at T = 8 s:

| Wave amplitude | Nonlinear / linear heave amplitude |
|----------------|------------------------------------|
| 0.1 m | +0.3% |
| 1.0 m | +2.0% |
| 2.5 m | +16% |

With much larger waves (for example H = 10 m) the body is thrown clear of the
water and then driven deep below it. SEA-Stack warns at start-up when
calm-water nonlinear hydrostatics is used with waves, and once per body if it
fully emerges or fully submerges.

## Large waves: nonlinear Froude-Krylov

With `wetted_surface: instantaneous`, the static and incident-wave pressure
are integrated over the hull below the wave surface, and the linear Froude-Krylov part of the
excitation is removed. See
[EXAMPLE_WORKFLOWS.md](../../../../docs/usage/EXAMPLE_WORKFLOWS.md#large-waves-nonlinear-froude-krylov).

`regular_waves_nlfk` is close to the IEA OES Task 10 steep case (H = 6 m;
Task 10 uses 6.28 m at T = 8 s) with the optimum PTO damping. Its steady
heave RAO is 0.664. The Task 10 weakly nonlinear codes give 0.665 (MARIN) and
0.666 (NREL); linear models give 0.69. In the H = 10 m case above, nonlinear Froude-Krylov
keeps the heave within about 5.4 m.

## Optional analysis

Overlay linear vs nonlinear heave after running the decay cases:

```
python demos\iea_sphere\compare_lin_vs_nl.py demos\iea_sphere
```

Plots are written under `demos/iea_sphere/outputs/plots/`.

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.
