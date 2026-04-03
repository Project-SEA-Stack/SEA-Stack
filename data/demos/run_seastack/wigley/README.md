# Wigley hull

Classical Wigley parabolic hull at ship scale: below the waterline the
half-beam follows the standard Wigley formula; above, vertical topsides and a
flat deck. Used for directional spreading and combined linear/quadratic damping
examples with a single rigid body.

## Key parameters

| Parameter | Value |
|-----------|-------|
| Waterline length (L) | 50.0 m |
| Beam (B) | 12.0 m |
| Draft (T) | 3.5 m |
| Freeboard (F) | 3.5 m |
| B/L | 0.24 |
| Equilibrium mass | 956,667 kg |
| CoG (below waterline) | −1.75 m |

Below the waterline the half-beam follows the Wigley formula  
`y(x,z) = (B/2) * (1 - (2x/L)^2) * (1 - (z/T)^2)` (see original Wigley hull definition).

## Available cases

| Case | Description | Setup YAML |
|------|-------------|------------|
| spreading | JONSWAP irregular (Hs = 3 m, Tp = 10 s), cos²s spreading (s = 12), 600 s | `spreading/wigley_spreading.setup.yaml` |
| damping | Same sea as spreading + linear damping (sway, roll) and quadratic damping (surge, sway, heave, roll, pitch) | `damping/wigley_damping.setup.yaml` |

The spreading case expects directional BEM data (`run_bem_directional.py`).

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.

### Regenerating assets

```text
cd assets
python generate_meshes.py
python run_bem_directional.py    # required for spreading
python run_bem.py                # optional single-heading H5
```

- `generate_meshes.py` — Nemoh and OBJ from parametric hull (stdlib only).
- `run_bem.py` — Single-heading Capytaine → `hydroData/wigley.h5` (needs capytaine, numpy, h5py, scipy).
- `run_bem_directional.py` — Multi-heading BEM → `hydroData/wigley_directional.h5`.
