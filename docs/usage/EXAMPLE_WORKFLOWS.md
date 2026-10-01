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

## Regular waves: constant-coefficient radiation

For a regular wave, `radiation.method: constant_coefficients` applies the
BEM coefficients at the wave frequency directly:

    F_rad = -A(w) xddot - B(w) xdot

A(w) and B(w) are the full coupled 6N x 6N matrices from the `.h5` file
(`added_mass/all`, `radiation_damping/all`). A(w) **replaces** the
infinite-frequency added mass A_inf in the Chrono added-mass load, so added
mass is never counted twice. B(w) is applied as the radiation force, in
place of the RIRF convolution or state-space model. Example:
[`rm3/regular_waves_cc`](../../data/demos/run_seastack/rm3/regular_waves_cc/).

```yaml
hydrodynamics:
  waves:
    type: regular
    height: 2.0
    period: 7.853984            # must be a BEM grid period
  radiation:
    method: constant_coefficients
    frequency_tolerance: 1.0e-4  # optional; relative |w_k - w| / w_k
```

Use it when the BEM frequency grid is coarse (for example design-screening
runs with a dozen periods). On such grids the RIRF is built from few
frequencies and can misrepresent A(w) and B(w) even at the grid points.
On dense grids both methods agree closely.

Rules (checked at startup; violations stop the run with an explicit error):

- **Regular waves only.** Irregular, bimodal, spread, `eta_file`, still
  water and free decay are rejected: a single frequency cannot represent
  them.
- **The period must be on the BEM grid.** No interpolation and no
  extrapolation. The match uses `frequency_tolerance` (default `1e-4`,
  relative). On failure the error lists the nearest grid period and all
  available periods.
- **RIRF-only options are rejected**: `smoothing`, `taper`, and radiation
  `truncation_time`.
- The `.h5` must contain `added_mass/all` and `radiation_damping/all`
  for every body (standard BEMIO output).

Startup log: the requested and matched period and frequency, the frequency
difference, the A(w), B(w) and A_inf diagonals per body, and a line
confirming that A(w) replaces A_inf.

Limitations:

- **Steady state only.** The start-up transient is wrong: it decays at the
  rate set by B(w) at the wave frequency, not by the true radiation damping
  at the body's natural frequency. If B(w) is small, the transient can take
  minutes of simulated time to decay. Use an excitation ramp, run long
  enough, and discard the transient before post-processing.
- Radiation damping is evaluated once per time step from the current
  velocity (as for the other radiation methods). The resulting error is first
  order in the time step: about 0.6% amplitude near heave resonance at
  `dt = 0.01 s` for the IEA sphere, halving at `dt = 0.005 s`.
