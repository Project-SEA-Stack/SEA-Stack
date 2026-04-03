# SEA-Stack Verification Reference Data

This directory contains reference data for cross-code verification tests.

## Directory Structure

Each verification case follows this layout:

```
<model>_<case>/
    raw/                    # Original files from external tools (NEVER modified)
        <source_tool>/
    normalized/             # Converted to SEA-Stack standard format
        <source_tool>/
    manifest.json           # Provenance and metadata (schema v1.0)
    normalize.py            # Generates normalized/ from raw/
    README.md               # Human-readable description (optional)
```

## Cases

| Directory | Model | Sources | Status |
|-----------|-------|---------|--------|
| `rm3_mooring/` | RM3 | WEC-Sim/MoorDyn | Existing, manifest added |
| `sphere_decay_multicode/` | Sphere | ProteusDS, InWave, Marin, NREL CFD, WavEC, HydroChrono | Normalized |
| `sphere_rao_sweep/` | Sphere | InWave-HOTINT, InWave (Linear), Marin (Nonlinear), NREL (Nonlinear), ProteusDS (Linear), HydroChrono | Normalized |
| `oswec_decay_wecsim/` | OSWEC | WEC-Sim | Normalized |
| `oswec_rao_sweep/` | OSWEC | WEC-Sim | Normalized |
| `rm3_decay_wecsim/` | RM3 | WEC-Sim | Normalized (data only; not registered in CTest) |
| `f3of_decay_multicode/` | F3OF | INW, WSM, WDN, PDS, HydroChrono | Normalized |

## Canonical conventions

All normalized data conforms to [CONVENTIONS.md](CONVENTIONS.md):

- **Units**: SI throughout — m, rad, N, s. Angular RAOs in rad/m (not deg/m).
- **Reference frame**: Global, Z-up, origin at mean water level.
- **Displacements**: Relative to equilibrium (not absolute CG position).
- **Sign conventions**: Positive up (heave), positive bow-up (pitch).

## Normalization pipeline

Comparison scripts load reference data through `load_and_normalize()` from
`normalize_utils.py`. This function:

1. Parses the standard `#` metadata header
2. Validates units against the canonical convention
3. Automatically converts non-canonical units (e.g. `deg` → `rad`, `deg/m` → `rad/m`)
4. Returns `(time, data_columns, metadata)` with guaranteed SI units

To validate all normalized data against conventions:

```bash
python data/verification/check_conventions.py
```

## Multi-source overlay plots

Verification comparison scripts produce multi-source overlay plots using
`tests/verification/utilities/multi_source_plot.py`. Each plot shows
all available codes on a single figure with a clear legend, with SEA-Stack
drawn in black/bold. Optional difference subplots show deviation from a
chosen baseline.

## Standard file formats

**Time series** (`.txt`): `#`-prefixed header with `format: timeseries`, source, model, quantity,
units, columns. Space-delimited, SI units, time in column 0.

**RAO** (`.txt`): `#`-prefixed header with `format: rao`. Frequency in rad/s (column 0),
amplitude ratio, phase in radians.

## Provenance rules

Every normalized dataset records:

1. Source tool and version
2. Original file path within `raw/`
3. Extraction script name
4. Extraction date (ISO 8601)
5. Units (SI, per-column)
6. Sign conventions and coordinate frame
7. All transformations applied

## Shared Utilities

- `normalize_utils.py` — Helper functions for writing standard-format files and manifests

## Re-normalizing Data

To regenerate normalized files from raw sources:

```bash
python data/verification/<case>/normalize.py
```

Normalized files are deterministically reproducible from raw data + script.

## Manifest Schema (v1.0)

See `normalize_utils.py` for the manifest JSON structure. Key fields:
- `schema_version`: Always "1.0"
- `model`: Case identifier
- `sources`: Array of source tool info with raw file references
- `normalized_datasets`: Array with file paths, formats, units, transformations
- `simulation_parameters`: Optional simulation settings
