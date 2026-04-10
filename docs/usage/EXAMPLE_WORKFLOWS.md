# Example workflows

Pointers to runnable examples in the SEA-Stack tree: standalone libraries,
the full simulation app, and packaged demos. For a **separate** CMake project
that links against an installed SDK, see
[DOWNSTREAM_PROJECT.md](DOWNSTREAM_PROJECT.md).


## Standalone hydrodynamics

Evaluate hydrodynamic forces without Chrono — see
[`examples/standalone_hydro`](../../examples/standalone_hydro).

## Standalone PTO and control

Run PTO and control models without Chrono or Eigen — see
[`examples/standalone_controller`](../../examples/standalone_controller).

## Chrono-integrated simulation

Full time-domain multibody dynamics with `run_seastack` and YAML
configuration. After a build or package install, start from the
[Quick start](../../README.md#quick-start) section of the main README; case
YAML and assets live under [`data/demos/run_seastack/`](../../data/demos/run_seastack/).

## Demo-driven cases

Preconfigured models (RM3, 5SA, OSWEC, IEA sphere, F3OF, and others) with
documented command lines: [demo suite README](../../data/demos/run_seastack/README.md).
