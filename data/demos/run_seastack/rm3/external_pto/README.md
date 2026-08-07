# RM3 external PTO demo

RM3 case in a shared irregular sea state that replaces the Chrono TSDA
spring-damper with an out-of-process Python linear damper via
`external_pto_file` in the setup YAML (attach details live in
`*.external_pto.yaml`).

Sea state (same for all three external-PTO demos): JONSWAP Hs = 2 m,
Tp = 8 s, seed = 42, 60 s excitation ramp, 600 s total. PTO damping
`c = 1.2e6` matches the native TSDA in `rm3/irregular_waves`.

**Requirements**

- SEA-Stack built with `-DSEASTACK_ENABLE_EXTERNAL=ON`
- Python 3 on `PATH`: `python` on Windows, `python` or `python3` on
  macOS/Linux (the host falls back to `python3`). Or set `command` in the
  `*.external_pto.yaml` to your interpreter.

**Run**

```bash
run_seastack --nogui data/demos/run_seastack/rm3/external_pto
```

Uses `linear_damper_pto.py` (`F = -c v`). This is the transport / force-
correctness baseline: the external result is checked against the native
SEA-Stack `LinearPTO` under the same irregular waves. Model YAML
`spring_coefficient` / `damping_coefficient` stay at zero so the Python
module alone owns the force. To layer a passive Chrono spring-damper on
top of an external module, set `combine_native: true` in the
`*.external_pto.yaml` and put non-zero `k`/`c` on the TSDA in the model
YAML (see [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md)).

**Authoring pattern**

All external-PTO demos use the same `PtoModule` shape: `setup` reads config,
`force(state)` returns the actuator force, and `run(...)` connects to SEA-Stack.

```python
from seastack_external import PtoModule, PtoState, run

class LinearDamperPTO(PtoModule):
    def setup(self, cfg):
        self.c = float(cfg.get("damping", 50.0))
        return {"name": "LinearDamperPTO", "version": "1.0", "n_states": 0}

    def force(self, state: PtoState) -> float:
        return -self.c * state.velocity

if __name__ == "__main__":
    run(LinearDamperPTO())
```

Sibling cases (same sea state; only the PTO physics differ):

- [`../external_pto_adaptive/`](../external_pto_adaptive/) — same `F = -c v`,
  plus adapting `c(t)`, force saturation, and `reset()`.
- [`../external_pto_hydraulic/`](../external_pto_hydraulic/) — internal physics
  states (two accumulators) with `commit` / `rollback`.

Comparison plots: `examples/external_pto/run_visual_verification.py`.

See [EXTERNAL_FORCE_MODULES.md](../../../../docs/extending/EXTERNAL_FORCE_MODULES.md).
