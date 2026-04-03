# Trimaran

Three Wigley-derived hulls (center hull plus port/starboard outriggers) with
shared three-body BEM hydrodynamics. Packaged demo uses rigid cross-arms:
revolute at the center deck and a rigid link to each outrigger gunwale, with
linear and quadratic hydrodynamic damping in irregular JONSWAP seas.

## Key parameters

| Parameter | Center hull | Outrigger (each) |
|-----------|-------------|------------------|
| Waterline length | 50.0 m | 25.0 m |
| Beam | 12.0 m | 4.0 m |
| Draft | 3.5 m | 1.5 m |
| Freeboard | 3.5 m | 1.5 m |
| Mass | 956,667 kg | 68,333 kg |
| Lateral offset (y) | 0 | ±15 m (+X forward; H5 body indexing matches demo YAML) |

Rigid cross-arm boxes in the case YAML: 0.35×0.35 m section, 600 kg/m³,
geometry aligned with the C++ `demo_trimaran_rigid` example.

## Available cases

| Case | Description | Setup YAML |
|------|-------------|------------|
| rigid | Irregular JONSWAP, rigid cross-arm mechanism | `rigid/trimaran_rigid.setup.yaml` |

The directory `model.setup.yaml` at the trimaran folder root also resolves to
this rigid case (convenient package entry: `run_seastack demos\trimaran\`).

C++ developer demos (`demo_trimaran_hydro`, `demo_trimaran_rigid`,
`demo_trimaran_fea`) live under `demos/trimaran/` in the source repository.

## Assets

Shared geometry and BEMIO hydro data are in `assets/`.

### Regenerating meshes and hydrodynamics

From the **package** root:

```powershell
cd demos\trimaran\assets
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install capytaine h5py

python generate_meshes.py
python run_bem.py              # full (200 freq), or --quick for testing

deactivate
```

- `generate_meshes.py` — Nemoh and OBJ meshes for both hull sizes (stdlib only).
- `run_bem.py` — Coupled three-body Capytaine BEM → `trimaran.h5`.
