# OSWEC hydraulic external PTO (RSDA)

Same irregular-wave OSWEC case as [`../external_pto/`](../external_pto/). This is
the third step in the demo ladder:

- linear: stateless `τ = -c ω`
- adaptive: adapting `c(t)`, saturation, `reset()`
- **this case:** internal physics states + `commit` / `rollback`

The module integrates two accumulator oil volumes each step and returns
`τ = -A_p (p_hi - p_lo) sign(ω)`. On an RSDA, `piston_area` is an effective
rotary displacement [m³/rad], not a literal cylinder bore. Energy balance
`E_abs = dE_gas + E_motor + E_relief` is its own oracle.

Sea state: JONSWAP Hs = 2 m, Tp = 8 s, seed = 42 (shared with the sibling demos).
Hydraulic sizes / `motor_conductance` in `*.external_pto.yaml` are tuned so
effective damping is on the same order as the linear RSDA damper
(`c_eff ~ A_p^2 / G_mot ≈ 12e6` N·m·s/rad).

**Run**

```bash
# Build with -DSEASTACK_ENABLE_EXTERNAL=ON
run_seastack --nogui data/demos/run_seastack/oswec/external_pto_hydraulic
```

| Case | What it adds |
|------|----------------|
| [`../external_pto/`](../external_pto/) | Stateless `τ = -c ω` |
| [`../external_pto_adaptive/`](../external_pto_adaptive/) | Adapting `c(t)`, saturation, `reset()` |
| **this case** | Internal physics states + `commit`/`rollback` |

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
