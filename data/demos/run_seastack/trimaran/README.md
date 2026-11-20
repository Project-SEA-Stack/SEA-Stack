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
| fea_arms | Same sea, flexible steel cross-arms (Chrono::FEA, preview) clamped between the hulls | `fea_arms/trimaran_fea_arms.setup.yaml` |

The directory `model.setup.yaml` at the trimaran folder root also resolves to
this rigid case (convenient package entry: `run_seastack demos\trimaran\`).

### `fea_arms` (FEA cross-arms)

Each cross-arm is a 7.3 m Euler beam (8 elements, steel box 800 × 1000 ×
25 mm, depth vertical, about 5.0 t) from `trimaran_fea_arms.structure.yaml`.
Both ends are clamped: to the center-hull deck edge (y = ±6 m, z = 3.5 m) and
to the outrigger's inboard gunwale (y = ±13 m, z = 1.5 m). This differs from
`rigid`, which has a revolute at the deck. The arms therefore carry the
relative wave loads between the hulls. The colour plot shows the vertical
bending moment Mz (±1 MN·m).

```powershell
.\build\bin\Release\run_seastack.exe data\demos\run_seastack\trimaran\fea_arms
```

100 s run at dt = 0.02 s (HHT): about 15 s of wall time headless.
`outputs/structure_diagnostics.csv` has the four mate reactions and per-arm
moment extremes at every step.

| Quantity (one arm) | Value |
|--------------------|-------|
| Arm weight | 49.0 kN |
| Mean hull reactions, first 10 s (root + tip) | 34.7 + 14.4 = 49.1 kN |
| Peak \|Mz\| after the 60 s ramp | ≈ 0.89 MN·m (≈ 34 MPa bending stress) |
| Peak torsion \|Mx\| | ≈ 0.49 MN·m |
| Peak arm bending (end-to-end change) | ≈ 6 mm |

Not checked here: the arms have no hydrodynamic load of their own, and the
hull BEM data are unchanged from `rigid`.

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
