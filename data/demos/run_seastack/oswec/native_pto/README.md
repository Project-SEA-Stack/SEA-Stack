# OSWEC native PTO demo (RSDA, live parameters)

OSWEC flap with a native Chrono RSDA damper as the PTO. Same sea state and
damping as [`../external_pto/`](../external_pto/) (JONSWAP Hs = 2 m,
Tp = 8 s, seed = 42, 60 s ramp, 600 s; `c = 12e6` N·m·s/rad), but the
torque comes from the model YAML instead of a Python module, so it needs no
external build options.

Use it to try runtime parameter changes:

```bash
run_seastack data/demos/run_seastack/oswec/native_pto
```

In the Inspector's **PTO and connectors** section, drag the PTO damping
slider (or Ctrl+click it to type a value). The change applies from the next
time step and is logged under `/events` in the results file. When the window closes, the changes are also written to
`outputs/results.irregular.schedule.yaml`. Rerun them headless; results are
bit-identical to the GUI run:

```bash
run_seastack --nogui data/demos/run_seastack/oswec/native_pto \
    --schedule data/demos/run_seastack/oswec/native_pto/outputs/results.irregular.schedule.yaml
```

Sign convention (Chrono RSDA): torque = -k (angle - free_angle) - c ω, with
the angle about the RSDA axis (+Y here).

More on live changes and schedules: [docs/usage/GUI.md](../../../../../docs/usage/GUI.md).
