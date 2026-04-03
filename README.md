# SEA-Stack

![License](https://img.shields.io/badge/license-MIT-blue)
![Version](https://img.shields.io/badge/version-1.0.0--beta-orange)
![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20macOS-lightgrey)

Open-source simulation software for offshore and marine energy systems—built for innovating, fast design exploration and engineering workflows.

---

SEA-Stack is a modular C++ simulation platform for offshore and marine energy systems. It is designed for fast engineering simulation and design exploration, with an architecture that can support higher-fidelity methods as the platform evolves.

Domain physics are separated from the solver engine, so the hydrodynamics, PTO, and control libraries can be used standalone or linked to alternative backends.

SEA-Stack helps engineers and researchers explore design spaces efficiently, reduce risk before expensive physical model testing and ocean trials, and **build confidence in new ocean technologies.**

## Example simulations
<table>
<tr>
<td align="center" valign="top" width="33%">
<strong>5SA</strong><br/>
<video src="docs/img/5sa.mp4" autoplay loop controls muted playsinline width="260"></video>
</td>
<td align="center" valign="top" width="33%">
<strong>RM3</strong><br/>
<video src="docs/img/RM3.mp4" autoplay loop controls muted playsinline width="260"></video>
</td>
<td align="center" valign="top" width="33%">
<strong>Trimaran FEA</strong><br/>
<video src="docs/img/trimaran_fea.mp4" autoplay loop controls muted playsinline width="260"></video>
</td>
</tr>
</table>

## Current capabilities

### System simulation and dynamics
- Coupled simulation of offshore and marine energy systems
- Multi-body system support with articulated joints and FEA, using
  [Project Chrono](https://projectchrono.org/) for multibody dynamics and time integration
- Dense infinite-frequency added mass from BEM included directly in the dynamics solver mass system via Chrono’s `ChLoadHydrodynamics`

### Hydrodynamics and wave loading
- Linear potential-flow hydrodynamics
- BEM hydrodynamic coefficients via BEMIO-format HDF5, including added mass, radiation IRFs, excitation, hydrostatic stiffness, and related data
- Nonlinear hydrostatics with mesh-based submerged volume calculation
- Radiation damping via convolution or state-space approximation
- Excitation via frequency-domain transfer functions or IRF convolution

### Waves and environmental inputs
- Regular, irregular, bimodal, and directional wave modelling with wave kinematics
- Precomputed elevation time-series import for IRF excitation workflows
- Linear and quadratic per-DOF damping
- Mooring with MoorDyn, including 3D visualization and tension-based colouring

### PTO and energy workflows
- Linear PTO and rectified hydraulic PTO, including check-valve bridge, accumulators, and motor-generator modelling
- Power matrix generation via **`run_seastack --campaign`**, including sea-state sweeps (for example Hs×Tp), per-cell time-domain runs, HDF5 summaries of mean absorbed power and energy, optional CSV export, and annual energy calculation from a site scatter table
  ([5SA demo](data/demos/run_seastack/5sa/power_matrix/README.md))

### Visualisation, configuration, and infrastructure
- 3D visualization with Vulkan Scene Graph and real-time GUI
- YAML-driven configuration with HDF5 results export
- Comprehensive test suites covering unit, regression, verification, comparison, and benchmark testing
- Modular architecture for research, extension, and engineering use

## Quick start

If you want to run the code quickly (within minutes) and try cases without touching C++, download a pre-built package from [Releases](https://github.com/Project-SEA-Stack/sea-stack/releases), extract it, and launch a demo—for example:

```
bin\run_seastack.exe demos\5sa\spreading
```

The packaged demos are all YAML-driven, so you can experiment with inputs and workflows from configuration files alone, without having to touch C++.

## Building from source

If you want to modify or extend the software, wire in a custom [Project Chrono](https://projectchrono.org/) build, or access more advanced capabilities, build from source as follows:

- **Clone & configure** — Clone on Windows, macOS, or Linux; use `git clone --recursive` only if you need optional submodules such as MoorDyn (see the walkthroughs below). Copy [`build-config.example.json`](build-config.example.json) to `build-config.json` and set paths to Chrono and, if you use full HydroIO, HDF5. Build/install Project Chrono for adapter builds using [`docs/build/BUILD_CHRONO.md`](docs/build/BUILD_CHRONO.md). You need a C++17 toolchain, CMake, and [Eigen3](https://eigen.tuxfamily.org/); the walkthroughs cover Chrono-free library builds versus the full `run_seastack` app and demos.
- **Build** — From the repo root, run [`scripts/windows/build.ps1`](scripts/windows/build.ps1) (Windows) or [`scripts/unix/build.sh`](scripts/unix/build.sh) (macOS/Linux); `-Help` / `--help` lists options. See [`scripts/README.md`](scripts/README.md) for how the drivers map to CMake.
- **Validate (recommended)** — Run the CTest suites (unit, regression, verification, comparison, benchmark) to check your build against references and baselines: §6 in the [Windows](docs/build/BUILD_WINDOWS.md#6-verify-the-build) and [macOS / Linux](docs/build/BUILD_MACOS.md#6-verify-the-build) guides; full detail in [`tests/README.md`](tests/README.md).

Detailed walkthroughs (toolchains, optional modules, packaging, troubleshooting, **§5** tests):

| Topic | Doc |
|-------|-----|
| Windows | [`docs/build/BUILD_WINDOWS.md`](docs/build/BUILD_WINDOWS.md) |
| macOS / Linux | [`docs/build/BUILD_MACOS.md`](docs/build/BUILD_MACOS.md) |
| Project Chrono (for SEA-Stack) | [`docs/build/BUILD_CHRONO.md`](docs/build/BUILD_CHRONO.md) |
| Selective CMake / modules | [`docs/build/BUILD_MODULES.md`](docs/build/BUILD_MODULES.md) |
| `build.ps1` / `build.sh` | [`scripts/README.md`](scripts/README.md) |
| CMake flags (overview §6.1) | [`TECHNICAL_OVERVIEW.md`](TECHNICAL_OVERVIEW.md) |
| Other build notes | [`docs/build/`](docs/build/) |

## Further reading

| If you want to… | Document | What it offers |
|------------------|----------|----------------|
| **See the big picture**—architecture, physics flow, module boundaries | [`TECHNICAL_OVERVIEW.md`](TECHNICAL_OVERVIEW.md) | End-to-end design, hydrodynamics and Chrono coupling, configuration, I/O and architectural diagrams.|
| **Add a new hydrodynamic force** (extra loads on bodies, custom physics in the hydro pipeline) | [`docs/extending/EXTENDING.md`](docs/extending/EXTENDING.md) — *Add a new hydrodynamic force component* | Implement **`IHydroForceComponent`**, place sources under `libs/hydro/`, register with the force model, add a unit test. |
| **Add a new PTO, controller, or solver adapter** | [`docs/extending/EXTENDING.md`](docs/extending/EXTENDING.md) (other sections) | Same cookbook style: interfaces (**`WaveBase`**, **`IPTOModel`**, **`IController`**, Chrono-style adapters), file layout, tests. |
| **Use SEA-Stack from another CMake project** | [`docs/usage/DOWNSTREAM_PROJECT.md`](docs/usage/DOWNSTREAM_PROJECT.md) | **`find_package(SEAStack)`**, imported targets, install layout, typical consumption patterns. |
| **Run worked examples** (standalone libraries, `run_seastack`, demo YAML cases) | [`docs/usage/EXAMPLE_WORKFLOWS.md`](docs/usage/EXAMPLE_WORKFLOWS.md) | Entry points into [`examples/`](examples/) and the [demo suite README](data/demos/run_seastack/README.md). |
| **Run tests** or dig into **labels, baselines, reports, and plots** | [`tests/README.md`](tests/README.md) · [`tests/TEST_SUITES_REFERENCE.md`](tests/TEST_SUITES_REFERENCE.md) · [`tests/REPORTING_AND_PLOTS.md`](tests/REPORTING_AND_PLOTS.md) | User-facing how-to; reference tables; Markdown/PDF reports and figures. |
| **Contribute** code or docs | [`CONTRIBUTING.md`](CONTRIBUTING.md) | Workflow, expectations, and repo conventions for contributors. |

## Project status

SEA-Stack is under active development. The current release (v1.0.0-beta) provides a robust modular core, validated offshore simulation workflows, and a suite of representative demos and test cases. Packages are currently available for Windows and macOS. Linux packaging and CI/CD are planned. Multi-fidelity capabilities and integration with higher-fidelity methods are planned as the platform evolves.

## License

Licensed under the MIT License. See [LICENSE](LICENSE).

Copyright (c) 2026 Simocean Ltd. and The National Laboratory of the
Rockies (NLR).
