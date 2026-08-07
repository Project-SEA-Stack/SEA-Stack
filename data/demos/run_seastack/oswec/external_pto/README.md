# OSWEC external PTO demo (RSDA)

OSWEC flap case in a shared irregular sea state that replaces the Chrono RSDA
spring-damper with an out-of-process Python linear damper via
`external_pto_file` in the setup YAML (attach details live in
`*.external_pto.yaml`).

This is the rotational counterpart to the RM3 TSDA demos: same `PtoModule`
scripts, but `state.velocity` is [rad/s] and `force()` returns torque [N·m].

Sea state (same for all three OSWEC external-PTO demos): JONSWAP Hs = 2 m,
Tp = 8 s, seed = 42, 60 s excitation ramp, 600 s total. PTO damping
`c = 12e6` N·m·s/rad matches the native RSDA used in the OSWEC C++ regression
tests.

**Requirements**

- SEA-Stack built with `-DSEASTACK_ENABLE_EXTERNAL=ON`
- Python 3 on `PATH`: `python` on Windows, `python` or `python3` on
  macOS/Linux (the host falls back to `python3`). Or set `command` in the
  `*.external_pto.yaml` to your interpreter.

**Run**

```bash
run_seastack --nogui data/demos/run_seastack/oswec/external_pto
```

Uses `linear_damper_pto.py` (`τ = -c ω`). Model YAML spring/damping stay
at zero so the Python module alone owns the torque; set
`combine_native: true` on the attach YAML to layer model YAML `k`/`c` on
top (see [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md)).

Sibling cases (same sea state / RSDA wiring; only the PTO physics differ):

- [`../external_pto_adaptive/`](../external_pto_adaptive/) — adapting `c(t)`,
  torque saturation, and `reset()`.
- [`../external_pto_hydraulic/`](../external_pto_hydraulic/) — internal physics
  states (two accumulators) with `commit` / `rollback`.

Heave/TSDA ladder: [`../../rm3/external_pto/`](../../rm3/external_pto/).

Comparison plots (pitch / torque / energy, same style as RM3):

```bash
python examples/external_pto/run_visual_verification.py --platform oswec \
    --output-dir oswec_external_pto_verification
```

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
