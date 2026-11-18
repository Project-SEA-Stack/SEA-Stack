# The SEA-Stack viewer

`run_seastack` shows the model in its own 3D viewer while it runs. The viewer
only draws: a run you don't change from the GUI gives the same results as a
headless run (`--nogui`).

New in v1.1. See also
[RELEASE_NOTES_v1.1.md](../build/RELEASE_NOTES_v1.1.md#gui-and-visualization).

## Starting

```bash
run_seastack data/demos/run_seastack/rm3/irregular_waves     # run with the viewer
run_seastack --replay path/to/outputs/results.irregular.h5    # replay a recorded run
run_seastack data/demos/run_seastack/rm3/irregular_waves --nogui   # no window
```

- **Chrono viewer.** `--viewer=chrono` opens the older Chrono viewer. It also
  opens, with a warning, if the model contains something the SEA-Stack
  viewer cannot draw, or if the SEA-Stack viewer fails to start. It will be
  removed in a later release.
- **Platforms.** The SEA-Stack viewer has been tested on Windows only so far.

## Layout and controls

The window has a transport bar along the bottom, the 3D view, and an Inspector
on the right. The Inspector sections are Scene, Selected, Loads and motion,
Sea state, PTO and connectors, Changes and Display. Hover over a control for
a description.

| Input | Action |
|---|---|
| Left drag / middle drag / right drag or wheel | Rotate / pan / zoom |
| Click a body | Select it; click empty space to clear |
| Double-click a body | Frame it |
| `F` | Fit the whole model |
| `Space` | Play or pause |
| `[` / `]` / `\` | Half speed / double speed / real time |
| `,` / `.` | Step back / forward one recorded frame |
| `Home` / `End` | First frame / back to live (last frame in a replay) |
Shortcuts are ignored while you type in a text field.

**Speed.** The speed slider runs from 0.05x to 20x, and Max steps the live
run as fast as the machine allows. Playing recorded frames (timeline or
replay) at Max runs at 20x. The viewer always starts at real time (1x): the
simulation YAML key `enforce_realtime` is ignored by `run_seastack` (in v1.0,
`false` ran the GUI as fast as possible). Headless runs always run as fast as
possible. Speed only changes how many steps run per frame; it never changes
the time step or the results. If the model cannot keep up, the bar shows the
speed achieved in amber. The wall time in the status bar includes drawing and
pacing, so time a `--nogui` run to measure compute speed.

## Timeline and replay

The timeline holds the recorded frames of the run so far. Drag it or step
through it to look back; the simulation pauses while you do. Press Live (or
`End`) to return and carry on. When one time step lasts several frames, "Smooth slow
motion" (Display) draws poses interpolated between steps.

`--replay` plays a results file without running Chrono, at 0.05x to 20x. It
needs a file written by v1.1 or later (results schema 1.1 in `/meta`, which
adds `/visualization`); an older file gives a clear message. Long records are
thinned to fit about 512 MB. Force arrows and load plots work in a replay too:
every run records the hydrodynamic loads by component under
`/visualization/forces` (float32). Recording is always on. It adds about a
quarter to a third to the results file: RM3 irregular waves over 180 s grows
from 9.4 MB to 12.5 MB. The frame at t = 0 is the input pose, labelled as such,
because results start after the first step.

**Mooring lines** are recorded for replay under `/results/mooring`, following
`--output-level`. Peak tension per line is written at every recorded step at
every level. Node positions and tensions are written at 10 Hz in float32 at
`standard` (the default), at every recorded step at `detailed`, and not at all
at `compact`, in which case a replay draws no mooring lines. For RM3 with
moorings over 400 s, the file is 21.4 MB at `compact`, 33.4 MB at `standard`
and 113 MB at `detailed`, against 20.8 MB in v1.1.0 before mooring recording.

## Changing parameters while the model runs

**What can change.**

| Parameter | Name in schedules and `/events` | Units |
|---|---|---|
| TSDA spring, damping, rest length | `tsda.<link>.spring_coefficient`, `.damping_coefficient`, `.rest_length` | N/m, N s/m, m |
| RSDA spring, damping, rest angle | `rsda.<link>.spring_coefficient`, `.damping_coefficient`, `.rest_angle` | N m/rad, N m s/rad, rad |
| Wave height H (regular) or Hs (spectral) | `waves.height` | m |
| Height scale (several sea states or an elevation record) | `waves.height_scale` | - |
| Period T or Tp | `waves.period` | s |
| Direction the waves travel towards, counter-clockwise from +x (0 = along +x, 90 = along +y) | `waves.direction` | deg |

Use the sliders in "PTO and connectors" and "Sea state". A change applies
on slider release, at the next step boundary. Right-click a slider to change
its range; that changes only what the slider can reach, not the model.

**How wave changes apply.**

- A height change ramps in over two wave periods (half-cosine). It scales the
  excitation, its IRF convolution, nonlinear Froude-Krylov and the drawn sea
  together. Once the ramp (and, for IRF excitation, the kernel's memory) has
  passed, the forces are those of the new sea. During the ramp the two
  excitation paths differ slightly: frequency-domain excitation scales the
  force by the ramp value at each instant, while the IRF convolution scales
  each past elevation sample by the ramp value at its own time.
- A period or direction change cross-fades from the old sea to the new one
  over two dominant periods. Each new component starts in phase with the old
  sea at the origin. The two weights sum to one, so the wave height dips
  mid-fade: for unrelated seas the variance halves at the midpoint and Hs
  falls to about 0.7 of its value; it can fall further.
- A period or direction change is refused, with the reason shown, when:
  - more than 1% of the new wave energy lies outside the BEM frequencies or
    headings;
  - radiation uses constant coefficients (period changes);
  - one IRF excitation kernel covers several headings (direction changes);
  - a previous change is still fading in.

**Record of changes.** Every applied change is listed in the Changes section
and saved in the results file under `/events` (`step`, `time`, `parameter`,
`value`, `previous_value`, `origin`). `/inputs/simulation/waves` describes the
sea at t = 0 (attribute `changed_during_run`). Wave changes are also saved
under `/events/waves`, with the new wave components and the incident
elevation at the origin as run; a replay applies them, so it shows the
changed sea.

## Schedules: rerun the changes without the GUI

When the run ends, its changes are written to `<results>.schedule.yaml` next
to the HDF5 file ("Export schedule" in Changes writes it earlier). Rerun the
case headless with the same changes:

```bash
run_seastack data/demos/run_seastack/oswec/native_pto --nogui \
    --schedule data/demos/run_seastack/oswec/native_pto/outputs/results.irregular.schedule.yaml
```

A schedule rerun with the same time step applies each change before the same
step, so the results match the GUI run exactly. You can also write a
schedule by hand:

```yaml
schedule_version: 1
events:
  - {time: 100.0, parameter: "waves.height", value: 3.0}
  - {time: 200.0, parameter: "rsda.PTO.damping_coefficient", value: 8.0e6}
```

Each event needs `parameter`, `value`, and `step` or `time` [s]. A time is
rounded up to the next step boundary. `step` is used only when the file's
`time_step` [s] matches the run's; exported schedules write both. A change
that would be refused in the GUI stops a scheduled run with the reason.

## Analysis

- **Selected body.** Centre-of-mass position, rotation (Cardan angles,
  extrinsic X-Y-Z: about the fixed world axes, as `euler_xyz` in the HDF5),
  velocity and a breakdown of loads by source. Isolate hides the other
  bodies; Plot opens the plot window for it.
- **Scene table.** One row per body: show, label, axes, force arrows, trail.
  The "All bodies" row at the top sets a column for every body.
- **Force arrows.** One arrow per hydrodynamic component (and optionally the
  total and moments) at each centre of mass, in world axes. These are the
  solver's forces, copied for display. Hover over the legend to highlight a
  component on every body; click to toggle it.
- **Plots.** Position, rotation, velocity and loads by component for the
  selected body, from the recorded frames.

## Water and radiated waves

The Scene section sets how the water is drawn.

- **Style.** Elevation (default) colours the sea by height, with a legend in
  m. Shaded draws reflections of the sky and sun glint.
- **Detail.** Coarse, Normal or Fine sets grid points per wavelength, so the
  sea looks the same at model and full scale. The readout shows the cell size
  and points per wavelength (amber below 20).
- **Radiated waves.** Draws the waves each body radiates as it moves, added
  to the incident sea. Auto picks the best method the BEM data supports:
  Haskind (from the excitation force, needs enough headings), then
  Approximate (from the radiation damping B(ω)). Methods the data cannot
  support are greyed out, with the reason. This is a visual approximation:
  diffraction is not drawn, and nothing feeds back into the simulation.
  Approximate treats each body alone and ignores the coupling between
  bodies. Where their waves cancel, the drawn wave is too high: for RM3
  (where Auto picks Approximate), with float and spar heaving together at the
  same velocity, it is 2 to 3 times too high at 0.5 to 1 rad/s. While radiated
  waves are on (the default), the water is computed on the CPU instead of the
  GPU shader, which is slower on fine grids.

**Shadows** (Scene section, on by default) draws soft shadows from the main
light. Bodies shadow themselves and each other; they cast no shadow on the
water. Turn it off on a slow GPU.

## Model YAML options

Optional keys under `simulation.visualization` in the simulation YAML:

```yaml
simulation:
  visualization:
    theme: dark                       # dark | light; overrides the saved preference
    body_colors: model                # model (YAML colours, default) | palette
    mooring_tension_range: [0, 2.0e6] # [N]; fixed colour range for mooring tension
    camera:                           # optional; the viewer frames the model otherwise
      location: [40, -60, 20]         # [m], world frame
      target: [0, 0, 0]               # [m]
```

## Preferences

Theme, water style and detail, panel layout and force display settings are
saved about a second after a change and on exit, in
`%APPDATA%\SEA-Stack\viewer.ini` (Windows) or `~/.config/seastack/viewer.ini`
(macOS and Linux). Display > "Reset viewer preferences" restores the
defaults.
