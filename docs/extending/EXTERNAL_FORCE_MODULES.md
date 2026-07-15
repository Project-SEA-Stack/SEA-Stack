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
| `ExternalPtoModel` | `IPTOModel` | 1-DOF link force/torque via `ChLinkTSDA` **or** `ChLinkRSDA` |
| `ExternalForceComponent` | `IHydroForceComponent` | 6-DOF body forces in the hydro pipeline |

Chrono functors:

| Functor | Link | Role |
|---------|------|------|
| `PTOForceFunctor` | TSDA | Lean `IPTOModel` (disp, vel) |
| `PTOTorqueFunctor` | RSDA | Lean `IPTOModel` (angle/omega as disp/vel) |
| `ExternalPtoForceFunctor` | TSDA | Rich kinematics → `ExternalPtoState` |
| `ExternalPtoTorqueFunctor` | RSDA | Rich kinematics → `ExternalPtoState` |

YAML `external_pto.link` may name either a `ChLinkTSDA` or a `ChLinkRSDA`;
attach discovers the type and registers the matching functor. Absorbed power
export uses `-(F·v)` / `-(T·ω)` when an IPTO / external functor is attached.

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
{"op":"initialize","protocol":1,"kind":"pto_tsda","n_inputs":17,"n_outputs":1,"dt":0.01,"in_names":["displacement","velocity","length", "..."],"config":{"damping":1200000}}
{"op":"evaluate","t":1.23,"dt":0.01,"in":[0.05,-0.12, ...]}
{"op":"reset"}
{"op":"commit"}
{"op":"rollback"}
{"op":"shutdown"}
```

The optional additive `in_names` array labels the positional `in` channels
(protocol v1, no version bump). Older modules may ignore it. Lean replay /
unit-test paths still use `kind:"pto"` with `n_inputs:2` and
`in_names:["displacement","velocity"]`. Chrono attach defaults to rich state
(`kind:"pto_tsda"` or `"pto_rsda"`, 17 channels); set `rich_state: false` in
YAML to keep the lean 2-channel contract.

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

Channels 0/1 are universal for both link types:

| Index | Input | TSDA unit | RSDA unit | Convention |
|-------|-------|-----------|-----------|------------|
| 0 | displacement | m | rad | extension positive (`length − rest_length` or `angle − rest_angle`) |
| 1 | velocity | m/s | rad/s | extending / opening positive |

| Index | Output | TSDA unit | RSDA unit | Convention |
|-------|--------|-----------|-----------|------------|
| 0 | force | N | N·m | resistive / opposes motion (same as `IPTOModel`) |

Rich Chrono attach also publishes (indexes 2–16):

| Names (TSDA) | Names (RSDA) | Notes |
|--------------|--------------|-------|
| `length`, `rest_length` | `angle`, `rest_angle` | Link-native absolute measure |
| `rel_accel` | `rel_accel` | Finite difference of relative velocity across accepted steps (0 on first step) |
| `body1_pos_{x,y,z}`, `body1_vel_{x,y,z}` | same | World-frame body 1 |
| `body2_pos_{x,y,z}`, `body2_vel_{x,y,z}` | same | World-frame body 2 |

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

The same script works on a TSDA or an RSDA. Units follow the link: the value
you return is force [N] on a TSDA or torque [N·m] on an RSDA.

### Preferred: `PtoModule.force(state)` + `run(...)`

Subclass `PtoModule`, implement `force(state)`, and call `run(...)`. Config
goes in optional `setup(cfg)`; stateful controllers also override `reset`
(and `commit` / `rollback` when needed). SEA-Stack's state arrives as a
`PtoState`; your return value is the actuator force (or torque).

```python
from seastack_external import PtoModule, PtoState, run

class MyPTO(PtoModule):
    def setup(self, cfg):
        self.c = float(cfg.get("damping", 1.2e6))
        return {"name": "MyPTO", "version": "1.0", "n_states": 0}

    def force(self, state: PtoState) -> float:
        return -self.c * state.velocity

    # optional lifecycle: reset(self) / commit(self) / rollback(self)

if __name__ == "__main__":
    run(MyPTO())
```

The same skeleton scales across the three demos:

1. **Linear** — only `force` (`F = -c v`).
2. **Adaptive** — same law, plus adapting `c(t)`, force saturation, and `reset`.
3. **Hydraulic** — internal physics states with `commit` / `rollback`.

`PtoState` always has `time`, `dt`, `displacement`, `velocity`. With rich state
enabled (default), it also carries link extras (`length`/`rest_length` or
`angle`/`rest_angle`), `rel_accel`, and body kinematics. Use `state.get(name)`
or `state.raw` for additional channels.

### Low-level positional API (still supported)

For non-PTO / multi-output modules, subclass `ExternalForceModule` and
implement `initialize` / `evaluate` with positional arrays:

```python
from seastack_external import ExternalForceModule, run

class MyPTO(ExternalForceModule):
    def initialize(self, cfg):
        self.c = cfg.get("damping", 1.2e6)
        return {"name": "MyPTO", "version": "1.0", "n_states": 0}

    def evaluate(self, t, dt, inputs):
        return [-self.c * inputs[1]]

if __name__ == "__main__":
    run(MyPTO())
```

YAML (`*.setup.yaml`), when `SEASTACK_ENABLE_EXTERNAL` is on. Prefer a dedicated
attach file (peer of `hydro_file`); this path is for **link actuators**
(`ChLinkTSDA` / `ChLinkRSDA`) only — not a custom 6-DOF body wrench:

```yaml
# setup.yaml
external_pto_file: my_case.external_pto.yaml
```

```yaml
# my_case.external_pto.yaml
link: PTO                 # name of a ChLinkTSDA or ChLinkRSDA
command: ["python", "my_pto.py"]
# rich_state: false       # optional; default true (17-channel kinematics)
timeout_ms: 20000
config:
  damping: 1200000
```

An inline `external_pto:` map in setup is still accepted for small one-offs;
do not set both `external_pto_file` and `external_pto:` together.

### Worked examples

| Module | What it shows | Demo |
|--------|---------------|------|
| `linear_damper_pto.py` | Stateless `F = -c v` | `demos/.../external_pto/` |
| `adaptive_damping_pto.py` | Adapting `c(t)` + saturation + `reset` | `demos/.../external_pto_adaptive/` |
| `hydraulic_accumulator_pto.py` | Internal states + `commit`/`rollback` | `demos/.../external_pto_hydraulic/` |

Start with the linear damper, then open the adaptive file — it is the same
skeleton with three small additions. The hydraulic demo is the advanced case.

### Verifying a module without a full run

`examples/external_pto/replay_harness.py` replays a `(time, displacement,
velocity)` trace through any module **in-process** and reports force,
mechanical power `P_abs = -F v`, cumulative absorbed energy and saturation/limit
events. `examples/external_pto/verify_examples.py` runs the prescribed-input
golden checks for all three cases (ctest `test_external_pto_examples_golden`).
Use the harness to check a new module against your own reference before wiring
it into an RM3 run.

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
   # setup.yaml
   external_pto_file: my_case.external_pto.yaml
   ```

   ```yaml
   # my_case.external_pto.yaml
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
