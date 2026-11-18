# FEA cantilever

A 6 m aluminium box beam (300 x 300 x 10 mm) clamped to a fixed `ground`
body at one end and to a 322.66 kg `tip_mass` body at the other. Gravity is
on, so the beam carries its own weight plus the tip load. It is the smallest
complete `structure_file` case and a quick check that the FEA path works.

```text
run_seastack data/demos/run_seastack/fea/cantilever --nogui
```

## Files

| File | Role |
|------|------|
| `cantilever.setup.yaml` | Points at the files below; requests a `structure_static_load_path` check |
| `cantilever.model.yaml` | Chrono bodies: `ground` (fixed) and `tip_mass` |
| `cantilever.simulation.yaml` | SMC, implicit Euler, `SPARSE_LU`, 8 s at 1 ms |
| `cantilever.structure.yaml` | Material, section, beam, mates and diagnostics |

## Expected results

Section: A = 0.0116 m², I = 1.6279e-4 m⁴, EI = 1.1395e7 N·m².
Loads: tip P = 322.66 × 9.81 = 3165.3 N; self-weight w = ρAg = 307.2 N/m.

| Quantity | Formula | Analytic | Run (t = 8 s) |
|----------|---------|----------|---------------|
| Tip sag from the tip load | PL³ / 3EI | 20.00 mm | |
| Tip sag from self-weight | wL⁴ / 8EI | 4.37 mm | |
| Total tip sag | sum | 24.37 mm | 24.41 mm |
| Root moment | PL + wL²/2 | 24 522 N·m | 24 522 N·m |
| Root vertical reaction on `ground` | P + wL | 5008.8 N, downward | 5008.8 N |
| Load-path check at 4 s | mates along gravity vs beam weight | 1.8435 kN | 0.002 % error |

The 0.16 % difference in tip sag is the beam's self-weight being lumped at the
6 element nodes; it falls as 1/n² with more elements.

The load-path check sums the reactions on both support bodies along gravity.
The tip mass's own weight cancels (the root carries it, the tip mate holds it
up), so the check compares against the beam weight alone.

`rayleigh_beta` is set high (0.005 s) so the beam settles well before the
check at 4 s. With Chrono's Euler beam this gives a first-mode damping ratio of
about β·ω₁ ≈ 0.10 (see the damping note in docs/usage/STRUCTURE_YAML.md).

## Outputs

- `outputs/results.no_wave.h5`: `/inputs/structure/connector` (undeformed
  geometry) and `/results/structure/connector` (node positions, orientations
  and element forces for every recorded frame).
- `outputs/structure_diagnostics.csv`: world-frame mate reactions and the
  largest beam moments every 10 steps.
