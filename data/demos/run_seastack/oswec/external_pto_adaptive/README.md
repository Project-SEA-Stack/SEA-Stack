# OSWEC adaptive-damping external PTO (RSDA)

Same irregular-wave OSWEC case as [`../external_pto/`](../external_pto/), but the
Python module is a small step up from the linear damper:

- same base law `τ = -c ω`
- `c` adapted online by a PI loop on `|ω|`
- torque clipped to `+/- force_max`
- `reset()` clears the controller state

Sea state: JONSWAP Hs = 2 m, Tp = 8 s, seed = 42 (shared with the sibling demos).
Controller gains / `c` band in `*.external_pto.yaml` are in rotational units
and tuned so absorbed power stays near the linear / hydraulic band.

**Run**

```bash
# Build with -DSEASTACK_ENABLE_EXTERNAL=ON
run_seastack --nogui data/demos/run_seastack/oswec/external_pto_adaptive
```

| Case | What it adds |
|------|----------------|
| [`../external_pto/`](../external_pto/) | Stateless `τ = -c ω` |
| **this case** | Adapting `c(t)`, saturation, `reset()` |
| [`../external_pto_hydraulic/`](../external_pto_hydraulic/) | Internal physics states + `commit`/`rollback` |

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
