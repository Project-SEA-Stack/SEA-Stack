# 5SA Power Matrix Demo

Generates a WEC power matrix (absorbed power vs. sea state) for the
5-body articulated WEC using SEA-Stack's campaign mode.

## Quick start

```bash
# Minimal 3×4 grid (12 cells)
run_seastack --campaign 5sa_power_matrix.yaml

# Finer 7×8 grid with AEP and CSV output
run_seastack --campaign 5sa_power_matrix_with_aep.yaml
```

## Files

| File | Description |
|------|-------------|
| `5sa_power_matrix.yaml` | Minimal campaign — coarse Hs/Tp sweep |
| `5sa_power_matrix_with_aep.yaml` | Standard campaign — finer grid, scatter/AEP, CSV |
| `example_scatter.csv` | Synthetic scatter table (uniform weights, not a real site) |

## What it does

1. Reads the base case from `../spreading/` (model, simulation, hydro YAML).
2. For each (Hs, Tp) cell, overrides the wave parameters and runs a full
   simulation via `RunSingleCase`.
3. Cells exceeding the steepness limit are automatically skipped.
4. Results are written to `power_matrix_summary.h5` (and optionally `.csv`).
5. If a scatter table is provided, AEP (annual energy production) is computed
   as a probability-weighted sum.

## Outputs

- **`power_matrix_output/power_matrix_summary.h5`** — campaign summary:
  - **`/cells`** — one row per logical cell (sparse table): `hs`, `tp`,
    `mean_absorbed_power_W`, `total_absorbed_energy_J`, status, etc.
  - **`/matrix`** — when the campaign is a full **Hs×Tp** grid with a single
    heading and single seed, **2D datasets** `mean_absorbed_power_W` and
    `total_absorbed_energy_J` shaped `[n_hs, n_tp]` (row-major), plus 1D
    coordinate arrays `hs` and `tp` for HDFView heatmaps.
  - **`/axes`** — axis vectors from the campaign YAML.
  - **`/meta`** — provenance and **units / definitions** for power and energy
    (`mean_absorbed_power_W_units`, `mean_absorbed_power_W_definition`, …).
  - **`/timeseries`** — **on by default** (set **`output.summary_timeseries: false`**
    to disable and shrink the file). Each **successful** cell gets a group
    **`cell_<index>`** (five-digit index matching the row order in `/cells`)
    with datasets **`time_s`** and **`total_absorbed_power_W`** (sum over all
    exported TSDA/RSDA links at each **decimated** step; same physics semantics
    as `mean_absorbed_power_W`). See `/meta/total_pto_power_timeseries_definition`.
    **Requires serial execution** (`execution.max_workers: 1` or omitted); parallel
    campaign mode logs a warning and skips embedding.
- **`power_matrix_output/power_matrix_summary.csv`** — flat CSV (when `csv: true`).

### Metrics semantics (important)

- **`mean_absorbed_power_W`** is the **time average over the full simulated
  window** (including transients), **not** a crest or rated peak.
- Values are the **sum over all exported PTO links** (TSDA/RSDA elements
  discovered by the HDF5 exporter). Each link contributes **viscous damper
  power only**: **`c * v^2`** (TSDA, `v` = extension rate from Chrono) or
  **`c * omega^2`** (RSDA). **Spring and preload** terms are **not** included
  in this scalar (so it stays a dissipation-oriented PTO metric when linear
  damping applies).
- **TSDA** links driven by SEA-Stack **`PTOForceFunctor` / `IPTOModel`** use
  **time-mean `-(F*v)`** for metrics (not `c*v^2`); the CLI logs a one-time
  warning per link name. **MBS YAML** TSDAs use Chrono’s built-in damper
  functor but often leave **`GetDampingCoefficient()` at zero**; the exporter
  fills **`c` from `damping_coefficient` in the model YAML** so `c*v^2` is
  correct.
- **RSDA** metrics use **`c*omega^2`** with the same **`c` resolution** (link
  then YAML).
- For linear viscous damper power as above, values are **non-negative**. Scale
  follows the **model as defined in the YAML** (geometry, damping, sea
  state). Expect **much lower time-mean power than short-term peak** power
  for the same sea state.

### Per-cell HDF5 vs. metrics-only runs

- With **`per_cell_h5: false`** (default), each cell still runs an internal
  exporter so PTO metrics are available for the summary; that scratch HDF5
  may live under the system temp directory and is **removed after** metrics
  are read — **do not rely on it for post-processing**. The CLI does **not**
  log that path when a temp directory is used.
- With **`per_cell_h5: true`**, each cell writes a persistent
  `results.<wave_type>.h5` under a subfolder of `power_matrix_output` (see
  campaign YAML `output` section). Use this when you need **full time
  histories** (PTO forces, powers, kinematics, etc.).

### Time series in the summary file

- **Default:** the summary includes **`/timeseries/cell_*`** with decimated
  **`time_s`** and **`total_absorbed_power_W`** (sum over PTO links). Set
  **`output.summary_timeseries: false`** to omit them and reduce file size
  (~`n_cells × n_samples × 16` bytes order-of-magnitude when enabled).
- **Per-link histories** (forces, per-TSDA `absorbed_power`, etc.) still require
  **`per_cell_h5: true`** and the per-cell `results.*.h5` files.

## Notes

- The spreading case uses `end_time: 600.0` (10 minutes per cell). For quick
  validation, create a shorter simulation YAML override (e.g. `end_time: 30.0`).
- The minimal grid (3×4 = 12 cells) is deliberately coarse for fast iteration.
- The scatter table is synthetic and not representative of any real site.

## Exit codes

| Code | Meaning |
|------|---------|
| 0 | All cells completed successfully |
| 1 | Configuration or setup error |
| 3 | Campaign completed with one or more failed/diverged cells |
