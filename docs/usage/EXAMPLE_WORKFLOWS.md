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

Set `hydrostatics: nonlinear` and a `mesh_file` on a body. In waves this
turns on nonlinear Froude-Krylov as well: SEA-Stack integrates the static
and incident-wave pressure over the part of the hull that is below the wave
surface at each time step. It then subtracts the linear Froude-Krylov force
that the BEM excitation already contains, so only diffraction and radiation
stay linear. Use it when heave or relative motion is a sizeable fraction of
the draft or freeboard. In that range, nonlinear hydrostatics alone (buoyancy
against the calm waterplane plus linear excitation) can throw the body clear
of the water. Example:
[`iea_sphere/regular_waves_nlfk`](../../data/demos/run_seastack/iea_sphere/regular_waves_nlfk/).

```yaml
hydrodynamics:
  bodies:
    - name: body1
      h5_file: ../assets/hydroData/sphere.h5
      mesh_file: ../assets/geometry/sphere.obj   # closed hull, body frame
      hydrostatics: nonlinear                    # in waves: nonlinear FK too
```

The start-up log states the model chosen for each body.

Advanced keys (most users do not need them):

| Key | Values | Meaning |
|---|---|---|
| `mesh_file` | path | Closed hull mesh, shared by nonlinear hydrostatics and nonlinear Froude-Krylov |
| `hydrostatics` | `linear` (default), `nonlinear` | Buoyancy from the BEM stiffness or from the mesh |
| `froude_krylov` | not set (default), `linear`, `nonlinear` | Not set: follows `hydrostatics` (see below). `linear`: incident-wave pressure on the mean hull, from the BEM excitation. `nonlinear`: on the instantaneous wetted hull |
| `froude_krylov_pressure` | `airy_wheeler` (default), `hydrostatic` | Airy pressure with Wheeler stretching, or the long-wave approximation `p = rho g (eta - z)` |

How `froude_krylov` is chosen when it is not set:

- `hydrostatics: linear`: linear.
- `hydrostatics: nonlinear` in still water: linear. In still water both
  models give the same force, and the calm-water model is cheaper.
- `hydrostatics: nonlinear` in regular or spectral waves: nonlinear.
- `hydrostatics: nonlinear` with an eta-table wave (`waves.method:
  irf_convolution`) or `include_excitation: false`: linear, with a warning.
  Nonlinear Froude-Krylov is not available in those cases.

`froude_krylov: nonlinear` on its own also sets `hydrostatics: nonlinear`.
Set `froude_krylov: linear` only if you want buoyancy against the calm
waterplane z = 0 with linear excitation in waves. That model is valid only
while the relative motion stays well below the draft and freeboard, and the
run logs a warning.

Pick `hydrostatic` pressure only for long waves: it ignores the decay of
the wave pressure with depth, so its error grows with wavenumber times draft
(about 4% heave for the IEA sphere at T = 4.2 s).

Rules for nonlinear Froude-Krylov (checked at start-up; each stops the run
with an explicit error):

- A `mesh_file` is required, and `hydrostatics` must not be set to `linear`.
- `include_excitation` must stay `true`: the diffraction force still comes
  from the BEM excitation.
- The wave must be defined by discrete components (regular, spectral, or a
  DFT eta import). Eta-table imports are rejected.
- If both the body-level `mesh_file` and `hydrostatics.mesh_file` are given,
  they must be the same file.

In still water the force equals nonlinear hydrostatics. The force ramps in
with the same ramp as the excitation. A warning is logged if a body starts
away from its BEM position (the subtracted linear force is phased for that
position) or if it leaves the water completely.

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
