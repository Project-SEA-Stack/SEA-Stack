# MATLAB external force module (reference / future extension)

The worked examples that ship with SEA-Stack are Python
(`data/demos/run_seastack/rm3/external_pto*`). MATLAB is a **documented
extension point**, not a shipped example: the protocol is identical, so no C++
changes are needed to drive a MATLAB module.

`seastack_external.m` is a reference IPC helper that speaks the same v1 TCP/JSON
protocol as the Python helper (4-byte big-endian length + JSON, dispatching
`initialize`/`evaluate`/`reset`/`commit`/`rollback`/`shutdown`). SEA-Stack does
not bundle MATLAB; provide your own license and interpreter.

## Launch from setup YAML

```yaml
# setup.yaml
external_pto_file: my_case.external_pto.yaml
```

```yaml
# my_case.external_pto.yaml
link: PTO
command:
  - matlab
  - -batch
  - "addpath('path/to/libs/external/matlab'); seastack_external('linear_damper')"
config:
  damping: 1200000
```

Pass the listen port via environment variable `SEASTACK_PORT` if your MATLAB
invocation does not forward `--seastack-port`. The C++ host always appends
`--seastack-port <N>` to `command`; wrap a small `.bat`/shell script if needed
to set `SEASTACK_PORT` from that flag before calling `matlab -batch`.

## Writing your own model

`seastack_external('linear_damper')` implements a minimal `F = -c * v` reply as
a self-contained reference. For your own model, pass `initialize`/`evaluate`
function handles:

```matlab
seastack_external(@my_evaluate, @my_initialize)
```

Keep the physics in your model functions; the helper stays generic. To port one
of the Python examples (e.g. the PI variable-damping or hydraulic case),
re-implement its `evaluate` recurrence as a MATLAB handle and validate it by
replaying a `(time, displacement, velocity)` CSV and comparing forces against
the Python reference (see `examples/external_pto/replay_harness.py`).

See [EXTERNAL_FORCE_MODULES.md](../../docs/extending/EXTERNAL_FORCE_MODULES.md).
