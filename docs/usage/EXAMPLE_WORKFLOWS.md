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

## Large waves: nonlinear Froude-Krylov

This option was added after v1.0.0 and is planned for v1.1. See
[RELEASE_NOTES_v1.1.md](../build/RELEASE_NOTES_v1.1.md).

Set `wetted_surface: instantaneous` and a `mesh_file` on a body. At each
time step SEA-Stack then integrates the static and incident-wave pressure
over the part of the hull that is below the wave surface. It subtracts the
linear Froude-Krylov force that the BEM excitation already contains, so only
diffraction and radiation stay linear. Use it when heave or relative motion
is a sizeable fraction of the draft or freeboard. Example:
[`iea_sphere/regular_waves_nlfk`](../../data/demos/run_seastack/iea_sphere/regular_waves_nlfk/).

```yaml
hydrodynamics:
  bodies:
    - name: body1
      h5_file: ../assets/hydroData/sphere.h5
      mesh_file: ../assets/geometry/sphere.obj   # closed hull, body frame
      wetted_surface: instantaneous              # mean (default) | calm_water | instantaneous
```

`wetted_surface` says which hull surface the static and incident-wave
pressure act on:

| Value | Static pressure (buoyancy) | Incident-wave pressure (Froude-Krylov) | Use |
|---|---|---|---|
| `mean` (default) | BEM stiffness on the mean hull | BEM excitation on the mean hull | Small motions; fastest |
| `calm_water` | Mesh below z = 0 | BEM excitation on the mean hull | v1.0.0 `hydrostatics: nonlinear`; eta-table waves; cross-checks |
| `instantaneous` | Mesh below the incident wave | Mesh below the incident wave | Large waves and motions |

`instantaneous` sets both parts together, so the hull never sees buoyancy
from one surface and wave pressure from another. `calm_water` is valid only
while the relative motion stays well below the draft and freeboard; in large
waves it can throw the body clear of the water, and the run logs a warning.
In still water the two nonlinear options give the same force, and
`instantaneous` uses the cheaper calm-water integrator. The start-up log
states the model chosen for each body.

`wave_pressure` (only with `instantaneous`) selects the wave pressure model:
`airy_wheeler` (default) is Airy pressure with Wheeler stretching above
z = 0; `long_wave` is `p = rho g (eta - z)`. Pick `long_wave` only for long
waves: it ignores the decay of the wave pressure with depth, so its error
grows with wavenumber times draft (about 4% heave for the IEA sphere at
T = 4.2 s).

v1.0.0 inputs still work: `hydrostatics: nonlinear` (scalar, block or inline
form) and the deprecated global `nonlinear_hydrostatics: true` both mean
`calm_water`.

Rules (checked at start-up; each stops the run with an explicit error):

- `calm_water` and `instantaneous` need a closed hull `mesh_file`. If both
  the body-level `mesh_file` and `hydrostatics.mesh_file` are given, they
  must be the same file.
- Set `wetted_surface` or `hydrostatics` on a body, not both.
- `instantaneous` needs `include_excitation: true`: the diffraction force
  still comes from the BEM excitation.
- `instantaneous` needs waves defined by discrete components (regular,
  spectral, or a DFT eta import). Eta-table imports (`waves.method:
  irf_convolution`) are rejected; use `calm_water` there.

The nonlinear Froude-Krylov force ramps in with the same ramp as the
excitation. A warning is logged if a body starts away from its BEM position
(the subtracted linear force is phased for that position) or if it leaves
the water completely.

Verification: unit, comparison and external tests are listed in
[docs/theory/NONLINEAR_FROUDE_KRYLOV.md](../theory/NONLINEAR_FROUDE_KRYLOV.md),
section 9 onwards. Against the IEA OES Task 10 weakly nonlinear codes
(MARIN, NREL) at steepness H/(gT²) = 0.01, the sphere heave RAO lies within
1% of the range spanned by the two codes at every period, free and with
optimum PTO.

Limits:

- Radiation and diffraction stay linear, about the mean position. The
  incident wave is not disturbed by the body. Slamming, breaking and green
  water are not modelled.
- Cost scales with mesh vertices times wave components. For the 8064-face
  IEA sphere mesh: about 0.4 ms per time step in a regular wave
  (`test_benchmark_sphere_reg_nlfk`), and about 1.6 ms in an irregular sea
  with 512 components (`test_benchmark_sphere_irreg_nlfk`; 16-core desktop,
  12 ms on one thread). The wave evaluation runs in parallel with OpenMP; set
  `OMP_NUM_THREADS` to limit it. Use the coarsest mesh that resolves the
  waterline.

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
