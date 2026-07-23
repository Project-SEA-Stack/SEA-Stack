# VLFP assets

Very Large Floating Platform (VLFP): six identical box pontoons connected by
pitch hinges. Model developed with the University of Strathclyde; validation
targets are the HYEL paper RAOs (see the `vlfp-seastack` project repository).

## Contents

- `geometry/pontoon_1.obj` ... `pontoon_6.obj` — box meshes from the Capytaine
  panel-mesh workflow (visualization; each is 29 m x 58 m x 5.2 m, local frame
  centred on the pontoon CoG).
- `hydroData/vlfp.h5` — multibody BEMIO database (all six bodies with
  cross-coupling), Capytaine -> BEMIO, with **body-frame** hydro reference
  points (`cg_local` variant required by SEA-Stack).

## Conventions

- Frame: z up, still-water level at z = 0.
- Pontoon CoG height: z = 1.6 m, so draft is 1.0 m and the deck top is at
  z = +4.2 m.
- Platform axis is +X; pontoon centres at x = 14.5, 43.5, ..., 159.5 m
  (29 m spacing), y = 29 m. Hinge lines at x = 29, 58, 87, 116, 145 m.

## Provenance

Copied from the Strathclyde VLFP tree (`vlfp-seastack` project):

- Meshes: `assets/designs/vlfp_6segment/meshes/pontoon_*.obj`
- Hydro: `assets/designs/vlfp_6segment/vlfp_bemio_cg_local.h5`
  (regenerated from `vlfp_bemio.h5` via `scripts/make_bemio_cg_local.py`)

Sea-water properties and the BEM frequency grid must match those used when the
H5 file was generated; document any regeneration in a study log.
