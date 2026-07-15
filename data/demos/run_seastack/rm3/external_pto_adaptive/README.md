# RM3 adaptive-damping external PTO (stateful)

Same short RM3 decay case as [`../external_pto/`](../external_pto/), but the
Python module is a small step up from the linear damper:

- same base law `F = -c v`
- `c` adapted online by a PI loop on `|v|`
- force clipped to `+/- force_max`
- `reset()` clears the controller state

**Run**

```bash
# Build with -DSEASTACK_ENABLE_EXTERNAL=ON
run_seastack --nogui data/demos/run_seastack/rm3/external_pto_adaptive
```

| Case | What it adds |
|------|----------------|
| [`../external_pto/`](../external_pto/) | Stateless `F = -c v` |
| **this case** | Adapting `c(t)`, saturation, `reset()` |
| [`../external_pto_hydraulic/`](../external_pto_hydraulic/) | Internal physics states + `commit`/`rollback` |

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
