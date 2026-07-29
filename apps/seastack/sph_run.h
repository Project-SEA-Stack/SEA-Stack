/*********************************************************************
 * @file  sph_run.h
 * @brief Single-case execution for Chrono::FSI (SPH) simulations.
 *
 * SEA-Stack normally runs Chrono multibody dynamics coupled with linear
 * potential-flow hydrodynamics (see single_run.h).  This module adds an
 * alternative fidelity path: fully-resolved fluid via Chrono's smoothed-particle
 * hydrodynamics (SPH) solver, driven entirely from Chrono's FSI YAML files
 * (a top-level FSI YAML that references an MBS model and an SPH fluid model).
 *
 * This path requires a Chrono build with the FSI and FSI_SPH modules enabled
 * (CUDA GPU required at runtime).  It is compiled only when SEA-Stack is built
 * with SEASTACK_ENABLE_SPH (compile-time SEASTACK_HAVE_SPH).
 *********************************************************************/

#ifndef SEASTACK_APP_SPH_RUN_H
#define SEASTACK_APP_SPH_RUN_H

#include <seastack/adapters/chrono/simulation_export.h>

#include <string>

namespace seastack::app {

/// All inputs needed to execute one SPH (Chrono::FSI) simulation.
struct SphRunConfig {
    /// Resolved path to the top-level Chrono FSI YAML file (type: FSI).
    std::string fsi_file;

    /// Where to write the SEA-Stack rigid-body results HDF5 and Chrono's native
    /// particle/body output.  Empty disables persistent output.
    std::string output_directory;

    /// HDF5 dataset options (level, decimation, compression, precision).
    seastack::chrono::ExportConfig export_config;
    /// Optional CLI --output-level override ("compact" | "standard" | "detailed").
    std::string cli_output_level;

    bool nogui       = true;   ///< Headless when true; VSG window when false.
    bool debug_mode  = false;
};

/// Structured result from a single SPH run.
struct SphRunResult {
    int exit_code = 1;
    double sim_time_final = 0.0;
    double wall_time_s = 0.0;
    std::string error_message;        ///< Empty on success.

    /// Resolved SEA-Stack HDF5 path when body-state results were written.
    std::string primary_artifact_path;
    /// When non-empty, explains missing HDF5 (e.g. no output directory).
    std::string artifact_note;
};

/// Execute a single SPH simulation described by a Chrono FSI YAML file.
///
/// Builds the coupled FSI system with Chrono's ChParserFsiYAML, runs the
/// co-simulation time loop, records rigid-body states to a SEA-Stack HDF5 file
/// (matching the potential-flow output layout) and optionally Chrono's native
/// particle output, and (when not headless) shows a VSG visualization.
SphRunResult RunSphCase(const SphRunConfig& config);

}  // namespace seastack::app

#endif  // SEASTACK_APP_SPH_RUN_H
