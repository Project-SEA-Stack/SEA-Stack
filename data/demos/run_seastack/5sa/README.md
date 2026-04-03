# 5SA (five-segment attenuator)

Five cylindrical tube segments linked by universal joints (pitch and yaw),
with hydraulic-ram TSDAs between segments for power take-off. Demonstrates
articulated multi-body hydrodynamics, mooring, directional spreading, and
campaign-style power-matrix runs.

## Key parameters

| Parameter | Value |
|-----------|-------|
| Segment diameter | 4.0 m |
| Segment length | 36.0 m |
| Number of segments | 5 |
| Total length | 180.0 m |
| Draft | 1.8 m |
| Mass (per segment) | 438,293 kg |
| CoG offset (below waterline axis) | −0.2 m |
| TSDA damping coefficient | 500,000 N·s/m |
| TSDA free length | 2.0 m |
| TSDA moment arm | 1.5 m |

## Available cases

| Case | Description | Setup YAML |
|------|-------------|------------|
| regular_waves | Regular waves (H = 2 m, T = 10 s, 120 s) | `regular_waves/5sa_regular.setup.yaml` |
| irregular_waves | JONSWAP irregular sea (Hs = 3 m, Tp = 10 s, 600 s) | `irregular_waves/5sa_irregular.setup.yaml` |
| mooring | Irregular waves with MoorDyn spread mooring (50 m depth, 300 s) | `mooring/5sa_mooring.setup.yaml` |
| spreading | Irregular sea with directional spreading and mooring | `spreading/5sa_spreading.setup.yaml` |
| bimodal | Collinear bimodal sea with mooring | `bimodal/5sa_bimodal.setup.yaml` |
| power_matrix | Hs×Tp campaign / AEP-style batch (`run_seastack --campaign`) | See [`power_matrix/README.md`](power_matrix/README.md) |

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.

### Regenerating meshes and hydrodynamics

From the **package** root:

```powershell
cd demos\5sa\assets
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install capytaine numpy h5py matplotlib

python generate_meshes.py
python run_bem.py
# Optional directional BEM:
# python run_bem_directional.py

deactivate
```

- `generate_meshes.py` — Nemoh (`.nemoh`) and OBJ meshes from parametric profiles (stdlib only).
- `run_bem.py` — Capytaine BEM, IRFs, BEMIO HDF5 (`5sa.h5`).
- `run_bem_directional.py` — Directional BEM when required.
