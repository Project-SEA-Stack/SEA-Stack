# External Force Modules

Language-agnostic coupling of user-written force / PTO / controller models
(Python, MATLAB, or later FMI) into a running SEA-Stack simulation.

## Design summary

SEA-Stack owns a C++ abstract interface `IExternalForceModel`
(`libs/external/include/seastack/external/external_force_model.h`).
Transport backends implement that interface. Higher-level bridges reuse the
existing PTO and hydro force paths:

| Bridge | Base | Role |
|--------|------|------|
| `ExternalPtoModel` | `IPTOModel` | 1-DOF link force via `PTOForceFunctor` + `ChLinkTSDA` |
| `ExternalForceComponent` | `IHydroForceComponent` | 6-DOF body forces in the hydro pipeline |

The first transport is **out-of-process IPC** over TCP loopback with
length-prefixed JSON (`IpcExternalForceModel`). Python and MATLAB speak the
same protocol; only thin client helpers differ.

## Why out-of-process IPC

- One C++ side serves every language (no embedded CPython / MATLAB Engine).
- Crash isolation: a bad user script does not corrupt the Chrono heap.
- No interpreter is bundled in the release package.
- Latency of one loopback round-trip per accepted step is negligible vs HHT.

## Solver re-evaluation (HHT) and time-caching

Chrono's HHT integrator re-queries forces during Newton iterations at the
same simulation time. SEA-Stack already freezes hydro/mooring/hydraulic-PTO
forces within a time level. External bridges follow the same rule:

- Round-trip to the module **only when** `time` advances.
- Repeated / backward evaluations return the cached output.

v1 therefore uses **explicit-within-step** coupling. Stiff controllers should
use a small `time_step`. The protocol also defines optional `commit` /
`rollback` operations for future FMI / true co-simulation.

## Wire protocol (v1)

**Transport:** TCP to `127.0.0.1` on an ephemeral port chosen by SEA-Stack.
The listen port is passed to the child as `--seastack-port <N>`.

**Framing:** each message is a 4-byte big-endian unsigned length followed by
that many UTF-8 JSON bytes (no newline required).

**Protocol version:** integer `kProtocolVersion = 1`, sent in `initialize`.

### Messages (SEA-Stack → module)

```json
{"op":"initialize","protocol":1,"kind":"pto","n_inputs":2,"n_outputs":1,"dt":0.01,"config":{"damping":1200000}}
{"op":"evaluate","t":1.23,"dt":0.01,"in":[0.05,-0.12]}
{"op":"reset"}
{"op":"commit"}
{"op":"rollback"}
{"op":"shutdown"}
```

### Replies (module → SEA-Stack)

```json
{"status":"ok","name":"MyPTO","version":"1.0","n_states":0}
{"status":"ok","out":[-144000.0]}
{"status":"ok"}
{"status":"error","message":"division by zero"}
```

- `initialize` reply must include `name`, `version`, and `n_states`.
- `evaluate` reply must include `out` (array of doubles, length = `n_outputs`).
- Any `status` other than `"ok"`, malformed JSON, wrong array length, or
  IPC timeout raises a C++ exception and aborts the run. The child process
  is terminated on failure.

### 1-DOF PTO I/O

| Index | Input | Unit | Convention |
|-------|-------|------|------------|
| 0 | displacement | m (or rad) | extension positive (`length - rest_length`) |
| 1 | velocity | m/s (or rad/s) | extending positive (Chrono TSDA) |

| Index | Output | Unit | Convention |
|-------|--------|------|------------|
| 0 | force | N (or N·m) | resistive / opposes motion (same as `IPTOModel`) |

### 6-DOF body-force I/O (ExternalForceComponent)

Inputs (per body, concatenated): position(3), orientation_rpy(3),
linear_velocity(3), angular_velocity(3) — world frame, matching `BodyState`.
Outputs (per body): force(3) [N], moment(3) [N·m] — world frame, matching
`GeneralizedForce`.

## Lifecycle

```
spawn child → accept TCP → initialize → evaluate* → shutdown → join
```

`Reset` clears module state without tearing down the connection.
`Commit` / `Rollback` are no-ops for the IPC v1 transport (commit is implied
by advancing time); FMI backends should implement them.

## User workflow (Python)

```python
from seastack_external import ExternalForceModule

class MyPTO(ExternalForceModule):
    def initialize(self, cfg):
        self.c = cfg.get("damping", 1.2e6)
        return {"name": "MyPTO", "version": "1.0", "n_states": 0}

    def evaluate(self, t, dt, inputs):
        disp, vel = inputs
        return [-self.c * vel]

    def shutdown(self):
        pass

if __name__ == "__main__":
    ExternalForceModule.run(MyPTO())
```

### Worked examples (by verification property)

Three Python examples ship as RM3 demos, chosen to cover different verification
properties rather than merely increasing complexity:

| Module | Property verified | Oracle | Demo |
|--------|-------------------|--------|------|
| `linear_damper_pto.py` | Transport + force correctness (`F = -c v`) | Native `LinearPTO` / analytic law | `demos/.../external_pto/` |
| `adaptive_damping_pto.py` | Stateful controller + constraints (PI variable damping, anti-windup, saturation, sliding window) | Prescribed-input replay vs independent recurrence | `demos/.../external_pto_adaptive/` |
| `hydraulic_accumulator_pto.py` | Dynamic external subsystem (two pressure states, rectifier, accumulators, relief valve, motor) | Component equations + exact energy balance | `demos/.../external_pto_hydraulic/` |

`adaptive_damping_pto.py` is a *variable-damping* controller: it never returns
net energy to the device, so it is deliberately not labelled "reactive control"
(which has a specific WEC meaning). `hydraulic_accumulator_pto.py` is a reduced
circuit inspired by the public WEC-Sim PTO-Sim RM3 hydraulic case; it integrates
its own pressure states each accepted step and implements `reset`/`commit`/
`rollback` over that state.

### Verifying a module without a full run

`examples/external_pto/replay_harness.py` replays a `(time, displacement,
velocity)` trace through any module **in-process** and reports force,
mechanical power `P_abs = -F v`, cumulative absorbed energy and saturation/limit
events. `examples/external_pto/verify_examples.py` runs the prescribed-input
golden checks for all three cases (ctest `test_external_pto_examples_golden`).
Use the harness to check a new module against your own reference before wiring
it into an RM3 run.

YAML (`*.setup.yaml`), when `SEASTACK_ENABLE_EXTERNAL` is on:

```yaml
external_pto:
  link: PTO
  command: ["python", "my_pto.py"]
  config:
    damping: 1200000
```

Release ZIPs install the IPC helper at `python/seastack_external.py`. Demo
scripts under `demos/` search parents for that folder (and, in a source
checkout, `libs/external/python/`). Keep interesting model code in the demo
script itself; do not put physics into `seastack_external.py`.

## Build option

```
-DSEASTACK_ENABLE_EXTERNAL=ON
```

Default is OFF. Enables `SEAStack::External`, unit tests, and YAML wiring
in `run_seastack`.

## Extending to MATLAB (not yet shipped as a worked example)

The protocol is language-agnostic, so the same C++ side works with a MATLAB
module — **no C++ changes are required**. Only the worked examples above are
Python today; a MATLAB port is a documented extension point, not a shipped
example.

To add one:

1. Reuse the reference IPC helper `libs/external/matlab/seastack_external.m`,
   which already implements the v1 framing (4-byte big-endian length + JSON)
   and the `initialize`/`evaluate`/`reset`/`commit`/`rollback`/`shutdown`
   dispatch loop against `127.0.0.1` on `--seastack-port`.
2. Write your model as `initialize`/`evaluate` handles (see the header comment
   in that file). Keep the physics in your model file; the helper stays generic.
3. Point the demo at it:

   ```yaml
   external_pto:
     link: PTO
     command: ["matlab", "-batch", "run_my_pto"]
   ```

The same displacement/velocity replay approach used by `replay_harness.py`
can validate a MATLAB module by feeding it a CSV trace and comparing forces
against the Python reference.

## Risks

- Explicit-within-step coupling is not fully implicit (see above).
- Parallel campaign cells each need their own child and ephemeral port.
- MATLAB startup cost can dominate short campaign cells.
- Stateful external models can break regression determinism; use `reset`
  and seed RNGs for reproducible cases.
