# Ship to ship

Two identical 50 m Wigley hulls side by side, 20 m centre to centre (8 m clear
gap), in a regular beam sea. One coupled two-body BEM database
(`assets/hydroData/ship2ship.h5`, 12 DOF) represents the radiation and
diffraction interaction across the gap.

## Cases

| Case | What it runs | Setup YAML |
|------|--------------|------------|
| `transfer_hulls` | Hulls only: the two-body hydrodynamics | `transfer_hulls/ship2ship_transfer_hulls.setup.yaml` |
| `bridge` | Hulls plus a flexible aluminium linkspan (Chrono::FEA, preview) on four bearings | `bridge/ship2ship_bridge.setup.yaml` |
| `bridge_static` | The same bridge in still water, with a load-path check | `bridge_static/ship2ship_bridge_static.setup.yaml` |

`bridge/README.md` has the bridge layout, bearings, damping choice and
verification numbers.

## Common model

| Item | Value |
|------|-------|
| Hull | Wigley, L = 50 m, B = 12 m, T = 3.5 m, freeboard 3.5 m |
| Mass | 956.7 t per hull (933.3 m³), CoG z = −1.75 m |
| Bodies | `body1` at y = −10 m, `body2` at y = +10 m (H5 body order) |
| Station keeping | soft horizontal TSDA per hull to `ground` (k = c = 5e4), a DP proxy |
| Water depth | 50 m (BEM and wave field agree) |
| Sea state | regular, H = 2.0 m, T = 8 s, heading 90° (beam), 20 s cosine ramp |

In the beam sea the hulls heave and roll out of phase: peak relative heave is
1.21 m and roll is about ±6° (`body1`) and ±4° (`body2`).

**Gap resonance.** The BEM data show a sharp peak at ω = 1.12–1.19 rad/s
(T = 5.3–5.6 s), the piston mode of the water between the hulls. If you
change the sea state, stay away from those periods unless you add gap damping.

## Assets

- `assets/hydroData/ship2ship.h5`: Capytaine BEM, 24 headings (0–345° in 15°
  steps), so any heading runs without a new solve.
- `assets/geometry/hull.obj` (visual) and `assets/meshes/hull.nemoh` (BEM).
- `assets/generate_meshes.py` and `assets/run_bem.py` regenerate them. They
  need a source checkout (the hull shape comes from the Wigley demo's
  generator, and the H5 writer from `scripts/bem`) plus
  `pip install capytaine numpy scipy h5py xarray`.
