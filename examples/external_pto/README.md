# External PTO examples

Three external (out-of-process, Python) PTO modules, chosen to cover **different
verification properties** rather than just increasing complexity. Each is
verified against an independent oracle.

| Script | Property verified | Oracle |
|--------|-------------------|--------|
| `linear_damper_pto.py` | Transport + force correctness (stateless `F = -c v`) | Native SEA-Stack `LinearPTO` / exact analytic law |
| `adaptive_damping_pto.py` | Stateful step-up (PI-adapted `c(t)`, saturation, `reset`) | Prescribed-input replay vs an independent PI recurrence |
| `hydraulic_accumulator_pto.py` | Dynamic external subsystem (two pressure states, rectifier, accumulators, relief valve, motor) | Component equations + exact internal energy balance |

The canonical copies live in the RM3 demo directories (they ship in the release
ZIP under `demos/`) and are copied next to `external_pto_example` at build time:

- `data/demos/run_seastack/rm3/external_pto/linear_damper_pto.py`
- `data/demos/run_seastack/rm3/external_pto_adaptive/adaptive_damping_pto.py`
- `data/demos/run_seastack/rm3/external_pto_hydraulic/hydraulic_accumulator_pto.py`

The physics live in those module files; `seastack_external.py` is only the IPC
helper. All three demos use the same named-state API (`PtoModule.force(state)`
+ `run(...)`) — the same script attaches to a `ChLinkTSDA` or `ChLinkRSDA`.
Case wiring lives in `*.external_pto.yaml` (pointed at by `external_pto_file`
in setup); that path is for link actuators only, not 6-DOF body forces.

Minimal damper:

```python
from seastack_external import PtoModule, run

class MyPTO(PtoModule):
    def setup(self, cfg):
        self.c = float(cfg.get("damping", 50.0))

    def force(self, state):
        return -self.c * state.velocity

if __name__ == "__main__":
    run(MyPTO())
```

## Watch the simulations

GUI is the default for `run_seastack` (omit `--nogui`). From the repo root,
with a Release build that has Chrono + external modules enabled:

```bash
# Linear damper (transport / force-correctness baseline)
build/bin/Release/run_seastack.exe data/demos/run_seastack/rm3/external_pto

# Adaptive damping (stateful PI + saturation)
build/bin/Release/run_seastack.exe data/demos/run_seastack/rm3/external_pto_adaptive

# Hydraulic accumulator PTO (dynamic subsystem)
build/bin/Release/run_seastack.exe data/demos/run_seastack/rm3/external_pto_hydraulic
```

On Unix/macOS use `run_seastack` (no `.exe`) and ensure Chrono/HDF5 DLLs (or
shared libraries) are on the loader path.

The YAML runner sets a default camera suitable for RM3 float + spar motion
(eye ≈ `(0, -50, -10)`, look-at ≈ `(0, 0, -10)`, Z-up). The GUI animation is a
**qualitative physical sanity check** — confirm that the float and spar heave
plausibly, the PTO link moves, and nothing diverges.

**Frame / video capture:** the Chrono VSG visualization backend used by
`run_seastack` does not currently expose screenshot or video recording APIs.
Use an external screen recorder if you need a clip; quantitative evidence comes
from the plots below.

## Generate verification plots

The plots and reported metrics provide the **quantitative verification**. They
use the **same demo YAML configurations** as the automated regression tests
(`test_external_pto_rm3_regression`).

### One-command workflow

Runs the three demos headlessly, builds a native Chrono `LinearPTO` twin of the
linear case, then writes figures + a multipage PDF:

```bash
python examples/external_pto/run_visual_verification.py \
    --run-seastack build/bin/Release/run_seastack.exe \
    --output-dir external_pto_verification
```

Optional: `--open` to launch the PDF, `--skip-run` to replot existing outputs,
`--keep-twin` to retain the ephemeral native reference case directory.

### Plot-only (existing outputs)

```bash
python examples/external_pto/plot_verification.py --auto \
    --output-dir external_pto_verification

# Or pass paths explicitly:
python examples/external_pto/plot_verification.py \
    --linear data/demos/run_seastack/rm3/external_pto/outputs/results.still.h5 \
    --adaptive data/demos/run_seastack/rm3/external_pto_adaptive/outputs/results.still.h5 \
    --hydraulic data/demos/run_seastack/rm3/external_pto_hydraulic/outputs/results.still.h5 \
    --native <native-linpto-h5> \
    --output-dir external_pto_verification
```

Requires Python packages: `h5py`, `numpy`, `matplotlib`.

### Files created under `--output-dir`

| File | Contents |
|------|----------|
| `01_cross_case_overview.png` | Float heave, PTO force, absorbed energy (all three cases) |
| `02_linear_vs_native.png` | External vs native LinearPTO: heave, force, residuals |
| `03_adaptive_controller.png` | Force / saturation, adapted `c(t)`, absorbed energy |
| `04_hydraulic_energy_balance.png` | `E_abs = ΔE_gas + E_motor + E_relief` + residual |
| `external_pto_verification.pdf` | The four figures as a multipage PDF |
| `summary.csv`, `summary.txt` | Compact metrics table + verification status |

Figures use the shared SEA-Stack report style from
`tests/utilities/plot_helpers.py` (same palette / axis styling / DPI as
regression, comparison and verification plots).

Adaptive and hydraulic demos also write example-only controller/state CSVs to
`outputs/pto_diagnostics.csv` (overwritten each run) when configured in their
setup YAML. Those internal states are **not** in the main SEA-Stack HDF5 export.

Absorbed-power sign convention (HDF5 attribute): **positive = absorbing**,
formula `-force_mag * speed`.

## C++ / IPC example

```bash
# Build with -DSEASTACK_ENABLE_EXTERNAL=ON
external_pto_example --python linear_damper_pto.py   # compare vs in-process LinearPTO
external_pto_example --mock                           # transport-only, no Python
```

## Shared replay harness

`replay_harness.py` replays a `(time, displacement, velocity)` trace through any
module **in-process** (no IPC, no Chrono, no RM3 run) and reports force,
instantaneous mechanical power `P_abs = -F v`, cumulative absorbed energy, peak
force and saturation/limit events.

```bash
python replay_harness.py --module hydraulic_accumulator_pto.py \
    --class HydraulicAccumulatorPTO --amplitude 0.8 --period 6 --duration 12
python replay_harness.py --module adaptive_damping_pto.py \
    --class AdaptiveDampingPTO --input trace.csv --config '{"damping": 8e5}'
```

## Verification ladder

The three examples are checked at increasing levels of integration. All tests
are registered with CTest and run in a few seconds total.

| Level | Script | CTest name(s) | Needs |
|-------|--------|---------------|-------|
| In-process physics vs oracle | `verify_examples.py` (uses `replay_harness.py`) | `test_external_pto_examples_golden` | Python 3 |
| IPC transport equivalence | `compare_ipc_replay.py` | `test_external_pto_ipc_{linear,adaptive,hydraulic}` | Python 3 + `external_pto_example` |
| Full Chrono RM3 run | `verify_rm3_regression.py` | `test_external_pto_rm3_regression` | Chrono + `run_seastack` + `h5py` |
| Human-visible plots | `run_visual_verification.py` / `plot_verification.py` | (manual) | Chrono + `h5py` + `matplotlib` |

**Golden physics** — `verify_examples.py` runs the prescribed-input checks for all
three cases against their independent oracles, including hydraulic
component-level checks (no-motion equilibrium, constant flow, accumulator
compression/expansion, motor dissipation, relief opening, pressure bounds,
reversed motion):

```bash
python verify_examples.py
```

**IPC transport** — `compare_ipc_replay.py` runs the *same* prescribed trace
through the full IPC path (`ExternalPtoModel` + `IpcExternalForceModel` over TCP,
driven by `external_pto_example --replay`) and a direct in-process evaluation,
and asserts the force histories are bit-identical. This isolates framing / JSON /
config passing / C++ time-caching from the physics:

```bash
python compare_ipc_replay.py --exe ./external_pto_example \
    --module linear_damper_pto.py --class LinearDamperPTO
```

**Chrono regression** — `verify_rm3_regression.py` runs the three RM3 decay demos
through `run_seastack`. The linear case is an exact-equivalence check against a
native Chrono `LinearPTO` twin generated on the fly (heave matches tightly; the
instantaneous force matches within an explicit-within-step tolerance since the
external module is frozen per accepted step). The adaptive and hydraulic cases
use aggregate and cross-case ordering checks instead of committed time-history
baselines:

```bash
python verify_rm3_regression.py --exe /path/to/run_seastack
```

See also [docs/extending/EXTERNAL_FORCE_MODULES.md](../../docs/extending/EXTERNAL_FORCE_MODULES.md).
