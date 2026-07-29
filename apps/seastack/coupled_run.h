/*********************************************************************
 * @file  coupled_run.h
 * @brief Single-case runner for coupled seakeeping + SPH tank sloshing.
 *
 * A COUPLED case pairs a potential-flow (BEM) hull — exactly as the
 * potential-flow path (see single_run.h) — with a deck-mounted tank whose
 * interior fluid is modelled with Chrono::SPH. The two fluid domains act on one
 * hull rigid body and are two-way coupled through rigid-body dynamics: the
 * exterior ocean applies linear potential-flow loads (ChForce callbacks +
 * ChLoadHydrodynamics added mass), while the interior tank walls are BCE_RIGID
 * markers on the hull so the sloshing reaction feeds back into the hull.
 *
 * Dispatched from RunFromYAML when a setup carries BOTH a `hydro_file` (exterior
 * BEM) AND a `tank:` block. Requires an SPH-enabled build (SEASTACK_HAVE_SPH;
 * CUDA GPU required at runtime).
 *********************************************************************/

#ifndef SEASTACK_APP_COUPLED_RUN_H
#define SEASTACK_APP_COUPLED_RUN_H

#include <seastack/adapters/chrono/simulation_export.h>
#include <seastack/infra/config/yaml_discovery.h>

#include <string>

namespace seastack::app {

/// All inputs needed to execute one coupled (BEM hull + SPH tank) simulation.
struct CoupledRunConfig {
    std::string model_file;        ///< Resolved wigley-style .model.yaml
    std::string simulation_file;   ///< Resolved .simulation.yaml (contact, dt, end_time, integrator)
    std::string hydro_file;        ///< Resolved .hydro.yaml (exterior BEM + waves)
    std::string output_directory;  ///< Where to write results HDF5; empty disables persistent output.

    /// Deck-tank specification (dimensions, deck position, fill, SPH resolution).
    seastack::infra::SetupConfig::TankConfig tank;

    seastack::chrono::ExportConfig export_config;  ///< HDF5 dataset options.
    std::string cli_output_level;                  ///< Optional --output-level override.

    bool nogui = true;       ///< If false and built with VSG, open a Chrono VSG window (SPH particles + hull).
    bool debug_mode = false;
};

/// Structured result from a single coupled run (mirrors SingleRunResult / SphRunResult).
struct CoupledRunResult {
    int exit_code = 1;
    double sim_time_final = 0.0;
    double wall_time_s = 0.0;
    bool diverged = false;
    std::string error_message;         ///< Empty on success.
    std::string primary_artifact_path; ///< Resolved HDF5 path when results were written.
    std::string artifact_note;         ///< Explains missing HDF5 (e.g. no output directory).
};

/// Execute one coupled seakeeping + tank-sloshing simulation.
CoupledRunResult RunCoupledCase(const CoupledRunConfig& config);

}  // namespace seastack::app

#endif  // SEASTACK_APP_COUPLED_RUN_H
