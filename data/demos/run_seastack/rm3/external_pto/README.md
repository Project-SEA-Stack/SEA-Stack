# RM3 external PTO demo

Short decay-style RM3 case that replaces the Chrono TSDA spring-damper with an
out-of-process Python linear damper via `external_pto:` in the setup YAML.

**Requirements**

- SEA-Stack built with `-DSEASTACK_ENABLE_EXTERNAL=ON`
- Python 3 on `PATH` (or set `command` to your interpreter)

**Run**

```bash
run_seastack --nogui data/demos/run_seastack/rm3/external_pto
```

Uses `linear_damper_pto.py` (`F = -c v`). This is the transport / force-
correctness baseline: the external result is checked against the native
SEA-Stack `LinearPTO` and the exact analytic law.

Sibling cases, chosen for different verification properties:

- [`../external_pto_adaptive/`](../external_pto_adaptive/) — stateful PI
  variable-damping controller (saturation, anti-windup, sliding window).
- [`../external_pto_hydraulic/`](../external_pto_hydraulic/) — reduced dynamic
  hydraulic PTO (two pressure states, accumulators, relief valve, motor).

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
