# Runtime output layout and HDF5 exports

This describes **outputs created when you run** SEA-Stack tools—not the CPack ZIP layout (see [PACKAGE_LAYOUT.md](PACKAGE_LAYOUT.md)).

## `run_seastack` (YAML-driven)

- Working directory is usually the case folder (e.g. `demos\5sa\bimodal`) or the repo/build output directory during development.
- Output directory and filenames come from the case YAML (simulation / export settings). Typical patterns include `results.<wave_type>.h5` under a per-run or per-case folder.
- For exact keys and defaults, see the case YAML files under `data/demos/run_seastack/` and application code in `apps/seastack/`.

## C++ demos (`demo_*`)

Demos use a compile-time `RESULTS_DIR_NAME` and write under a results subdirectory for that demo group; see `demos/CMakeLists.txt` and each `demo_*.cpp` for paths.

## Simulation export HDF5 schema (Chrono adapter)

Time-domain runs that export through the Chrono adapter use an HDF5 file with a **schema version** attribute (see `simulation_export.cpp` in `adapters/chrono`).

Top-level groups (high level):

- `/inputs` — model and simulation inputs (joints, PTO, environment, waves, etc.)
- `/results` — time histories (bodies, PTOs, joints, hydro components)
- `/meta` — run metadata (`system`, `config`, `run`)
- `/diagnostics` — optional diagnostics (e.g. radiation)
- `/visualization` — visual scene and per-sample display data for `run_seastack --replay`: body shapes and
  input pose, TSDA cable endpoints, and hydrodynamic loads by component (`forces`, float32). Visual only.
- `/events` — parameter changes applied while running (GUI or `--schedule`); see
  [docs/usage/GUI.md](../usage/GUI.md#changing-parameters-while-the-model-runs)

### `/meta/run` timing attributes

All times are wall-clock seconds.

| Attribute | Meaning |
| --- | --- |
| `wall_time_s` | Total elapsed time for the run, from start-up to the end of the time loop. Includes setup, GUI pauses and idle time after `end_time`. |
| `setup_wall_time_s` | Time before the time loop starts (model load, hydro set-up, viewer start-up). |
| `active_wall_time_s` | Time spent stepping. Use this for compute speed: simulated time / `active_wall_time_s`. |
| `paused_wall_time_s` | Time inside the time loop that was not stepping: GUI paused, waiting for Play, or idle after `end_time`. About zero for headless runs. |
| `mode` | `interactive` (GUI) or `headless` (`--nogui`). |

Headless `active_wall_time_s` includes HDF5 recording done during stepping. The console "Simulation speed" uses `active_wall_time_s`.

Units, sign conventions, and dataset names are defined in code; treat the exporter implementation as the source of truth when extending the schema.

## BEMIO / HydroIO inputs

Hydrodynamic **input** HDF5 files (BEMIO-style layout) are documented by the WEC-Sim/BEMIO ecosystem. SEA-Stack HydroIO reads that layout via `libs/hydro_io`; this is separate from the simulation **export** schema above.
