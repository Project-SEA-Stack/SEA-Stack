# RM3 hydraulic external PTO (dynamic subsystem)

Same short RM3 decay case as [`../external_pto/`](../external_pto/). This is
the third step in the demo ladder:

- linear: stateless `F = -c v`
- adaptive: adapting `c(t)`, saturation, `reset()`
- **this case:** internal physics states + `commit` / `rollback`

The module integrates two accumulator oil volumes each step and returns
`F = -A_p (p_hi - p_lo) sign(v)`. Energy balance
`E_abs = dE_gas + E_motor + E_relief` is its own oracle.

**Run**

```bash
# Build with -DSEASTACK_ENABLE_EXTERNAL=ON
run_seastack --nogui data/demos/run_seastack/rm3/external_pto_hydraulic
```

| Case | What it adds |
|------|----------------|
| [`../external_pto/`](../external_pto/) | Stateless `F = -c v` |
| [`../external_pto_adaptive/`](../external_pto_adaptive/) | Adapting `c(t)`, saturation, `reset()` |
| **this case** | Internal physics states + `commit`/`rollback` |

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
