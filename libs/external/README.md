# SEAStack::External

Out-of-process force / PTO / controller modules.

- Interface: `IExternalForceModel`
- Transport: `IpcExternalForceModel` (TCP loopback + length-prefixed JSON)
- Bridges: `ExternalPtoModel` (`IPTOModel`), `ExternalForceComponent` (`IHydroForceComponent`)
- Prototype: `FmiExternalForceModel` (throws until an FMI library is linked)
- Clients: `python/seastack_external.py` (IPC helper only); `matlab/seastack_external.m` (reference helper for a future MATLAB port)
- Demo physics (Python): `data/demos/run_seastack/rm3/external_pto*`
  — `linear_damper_pto.py`, `adaptive_damping_pto.py`, `hydraulic_accumulator_pto.py`
- Verification: `examples/external_pto/replay_harness.py` + `verify_examples.py` (prescribed-input golden tests)

Enable with `-DSEASTACK_ENABLE_EXTERNAL=ON`.

Release ZIPs install the Python helper at `python/seastack_external.py`.

Full design: [docs/extending/EXTERNAL_FORCE_MODULES.md](../../docs/extending/EXTERNAL_FORCE_MODULES.md).
