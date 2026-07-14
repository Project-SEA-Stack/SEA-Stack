# RM3 adaptive-damping external PTO (stateful)

Same short RM3 decay case as [`../external_pto/`](../external_pto/), but driven
by a **stateful** Python force model instead of a constant damper.

`adaptive_damping_pto.py` implements:

- `F = clip(-k x - c(t) v - f_c sign(v), +/-F_max)`
- PI adaptation of the viscous coefficient `c(t)` to track a peak-|v| setpoint
  over a sliding window, with anti-windup (`n_states: 2`)
- `reset()` that clears the integrator, history buffer and damping state

This is a *variable-damping* controller: it never returns net energy to the
device, so it is deliberately **not** called "reactive control" (which has a
specific WEC meaning). Its verification property is *stateful, constrained
control* — see the prescribed-input golden test in
`examples/external_pto/verify_examples.py`.

**Run**

```bash
# Build with -DSEASTACK_ENABLE_EXTERNAL=ON
run_seastack --nogui data/demos/run_seastack/rm3/external_pto_adaptive
```

**Verification ladder position**

| Case | Property verified | Oracle |
|------|-------------------|--------|
| [`../external_pto/`](../external_pto/) (linear) | Transport + force correctness | Native `LinearPTO` |
| **this case** (adaptive) | Stateful controller + constraints | Prescribed-input replay vs independent recurrence |
| [`../external_pto_hydraulic/`](../external_pto_hydraulic/) | Dynamic external subsystem | Component equations + energy balance |

Canonical script (also copied next to `external_pto_example`):
`adaptive_damping_pto.py`.

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
