# Radiated-wave visualisation spike

Python checks of the physics behind the planned radiated-wave overlay. They
use only the existing demo `.h5` files; no new BEM preprocessing is needed.
These are not production code.

Run `python run_all.py` (needs numpy, scipy, h5py and matplotlib; about 2 min).
Plots and JSON summaries go to `results/`.

| Script | Question |
|---|---|
| `s01_normalisation.py` | Units and normalisation of B, K and X; reconstructing B from K; the Haskind cross-check |
| `s02_psd.py` | Is the full coupled B(ω) symmetric and positive semidefinite? |
| `s03_patterns.py` | Assumed directional patterns vs Haskind (BEM) patterns; where they fail |
| `s04_coupling.py` | Damping implied by the assumed patterns vs BEM, for coupled modes and arrays |
| `s05_causality.py` | Phase choice and causality of the time-domain filters |
| `s06_harmonic_field.py` | Harmonic field map and energy flux vs radius |
| `s07_transient.py` | Free decay: dispersive waves and the time-domain energy balance |
| `s08_cost.py` | CPU, GPU and memory cost of the synthesis; accuracy of the spectral form |
| `s09_array_field.py` | 5SA array: superposed isolated sources vs the BEM array field |

## Conventions (same as `HydroData`)

- Time dependence is e^{+iωt}. Outgoing waves are H_m^(2)(kr).
- B = ρ·ω·B_file, K = ρ·K_file and X = ρ·g·X_file, per unit wave amplitude.
- DOFs are ordered surge, sway, heave, roll, pitch, yaw, about `properties/cg`.
- θ and the wave heading β are measured from +x, counter-clockwise.
- The far field is η = Re{A(θ)(kr)^(-1/2) e^{i(ωt−kr)}}, per unit body velocity.
- Haskind's relation: A_j(θ) = −e^{iπ/4}·k/(√(8π)·ρg·c_g)·X_j(θ+π). The phase
  constant was derived from a small-body point source and from an incident ×
  radiated flux balance. S5 checks it by causality (best within ±5–25°).

## Data issues found (since fixed)

The spike found two problems in the demo data. Both are fixed in the shipped
files; the scripts assume the fixed data.

- **Conjugated excitation phases.** The in-repo Capytaine scripts (`run_bem.py`)
  wrote `np.angle(F)` in Capytaine's e^{−iωt} convention, so the phases in
  `5sa*.h5`, `trimaran.h5` and `wigley_directional.h5` were conjugated relative
  to what SEA-Stack expects. The scripts now conjugate F before writing;
  `trimaran.h5` and `wigley_directional.h5` were converted in place with
  `scripts/fix_capytaine_excitation_phase.py`.
- **Aliased K(t) in the 5SA files.** Both were generated with `--quick`
  (30 frequencies), so the cosine sum for K(t) repeated within the 60 s RIRF
  (every 36.8 s for `5sa_directional.h5`). Transforming K directly gave about
  3× the true B. Both files were regenerated at full resolution (200 and 400
  frequencies). Files without stored B still go through `rwlib.b_from_k_fit`,
  which fits K on the stored BEM grid.
