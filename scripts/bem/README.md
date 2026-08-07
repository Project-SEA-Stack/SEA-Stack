# Shared BEM asset tooling

Reference implementation for turning a Capytaine solve into a BEMIO-convention
`.h5` that SEA-Stack (and WEC-Sim) can read. New demo BEM scripts should import
from here instead of writing their own HDF5 writer.

| File | Purpose |
|------|---------|
| `bemio_from_dataset.py` | Capytaine dataset -> normalised `hydro` dict -> BEMIO `.h5` |
| `check_bemio_h5.py` | Hull-agnostic sanity check of a written `.h5` |

Provenance: vendored from `C:\work\02_simulation\sphere_r0p12_bemio`, where it is
verified against the Hulme (1982) analytic floating-hemisphere solution.

## Usage

```python
from bemio_from_dataset import (
    BodyHydrostatics, build_hydro_from_dataset,
    radiation_irf, excitation_irf, write_bemio_h5,
)

hydro = build_hydro_from_dataset(dataset, [bh_body1, bh_body2])
radiation_irf(hydro, t_end=60.0, n_dt=1201, n_dw=1001)
excitation_irf(hydro, t_end=60.0, n_dt=2401, n_dw=1001)
write_bemio_h5(hydro, "model.h5")
```

Capytaine's NetCDF output carries no displaced volume, centre of buoyancy or
centre of gravity, and BEMIO requires all three, so per-body hydrostatics are
passed in explicitly as `BodyHydrostatics`.

```bash
python check_bemio_h5.py model.h5 --waterplane-area 400.0 --disp-vol 933.33
```

## Conventions

- Everything in the `.h5` is **normalised**: `C/(rho g)`, `A/rho`, `B/(rho w)`,
  forces `/(rho g)`, `K/rho`, `f/(rho g)`. The HDF5 `units` attributes describe
  the physical quantity, not the stored value - an inherited BEMIO quirk.
- Wave directions in **degrees**; DOF order surge, sway, heave, roll, pitch, yaw.
- Frequencies ascending; all-NaN frequencies dropped.
- `Ainf` from the Ogilvie relation; radiation state-space (`ss_*`) is not
  written, since SEA-Stack fits its own from the radiation IRF on request.
- BEM solves should pass a **lid mesh** over the waterplane
  (`mesh.generate_lid()`) to suppress irregular frequencies on surface-piercing
  bodies. This is a numerical device, unrelated to any geometric cap used to
  close a visualisation `.obj`.
- Meshes are written in the **BEM frame** (`z = 0` at the undisturbed free
  surface, `z` up), the same frame the coefficients refer to.

### The sign convention, and a bug it caught

Capytaine solves with `e^{-i w t}`; WAMIT and BEMIO use `e^{+i w t}`. The
imaginary part of the diffraction (scattering) **and** Froude-Krylov forces must
therefore be negated on export - `_forces_from_dataset` does this with
`im = -np.imag(z)`.

Skip it and every excitation phase is mirrored and the excitation IRF is
time-reversed. Heave hides the error because it is even in `t`; surge does not.
The check, per metre of wave amplitude at low frequency in head seas:

| DOF | Correct stored phase | Why |
|-----|----------------------|-----|
| Heave | ~0 deg | follows wave elevation |
| Surge | **+90 deg** | follows water-particle acceleration, which leads elevation |

The older per-demo writers in `data/demos/run_seastack/{wigley,trimaran}/assets/run_bem.py`
predate this module and omit the flip; their shipped `.h5` files store surge at
**-90 deg**. Regenerating them would shift existing demo and verification
results, so they are left alone deliberately - but do not copy those writers for
new work.
