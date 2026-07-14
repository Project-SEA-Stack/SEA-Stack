# RM3 reduced hydraulic external PTO (dynamic subsystem)

Same short RM3 decay case as [`../external_pto/`](../external_pto/), driven by a
**dynamic hydraulic** Python force model with its own internal states.

`hydraulic_accumulator_pto.py` implements a reduced two-pressure-state circuit
inspired by the public WEC-Sim PTO-Sim RM3 hydraulic case:

- double-acting cylinder with ideal (smoothed) check-valve rectification,
- high- and low-pressure gas accumulators (polytropic), `n_states: 2`,
- pressure-dependent cylinder force `F = -A_p (p_hi - p_lo) sign(v)`,
- relief valve limiting `dp`,
- resistive hydraulic motor / load,
- forward-Euler sub-stepping inside the module,
- `reset()`, `commit()` and `rollback()` over the full internal state.

**Verification property:** a genuinely *dynamic external subsystem*. The module
tracks its own energy budget and satisfies, exactly per sub-step,

```
E_abs = dE_gas + E_motor + E_relief
```

so the energy balance is its own oracle (checked in
`examples/external_pto/verify_examples.py`). Cross-code comparison against a
pinned WEC-Sim PTO-Sim reference trace can be layered on later.

**Run**

```bash
# Build with -DSEASTACK_ENABLE_EXTERNAL=ON
run_seastack --nogui data/demos/run_seastack/rm3/external_pto_hydraulic
```

**Verification ladder position**

| Case | Property verified | Oracle |
|------|-------------------|--------|
| [`../external_pto/`](../external_pto/) (linear) | Transport + force correctness | Native `LinearPTO` |
| [`../external_pto_adaptive/`](../external_pto_adaptive/) | Stateful controller + constraints | Prescribed-input replay vs independent recurrence |
| **this case** (hydraulic) | Dynamic external subsystem | Component equations + energy balance |

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
