/*********************************************************************
 * @file  single_run.h
 * @brief Reusable single-case execution API for SEA-Stack.
 *
 * Provides SingleRunConfig / SingleRunResult types and RunSingleCase(),
 * the core function that executes one sea-state simulation and returns
 * structured metrics.  Used by both the CLI runner (RunFromYAML) and
 * the campaign/power-matrix workflow.
 *********************************************************************/

#ifndef SEASTACK_APP_SINGLE_RUN_H
#define SEASTACK_APP_SINGLE_RUN_H

#include <seastack/hydro/config/hydro_config.h>
#include <seastack/adapters/chrono/simulation_export.h>
#include <seastack/infra/config/yaml_discovery.h>

#include <string>
#include <variant>
#include <vector>

namespace seastack::app {

/// All inputs needed to execute one SEA-Stack simulation.
/// Paths must be resolved (absolute or relative-to-cwd) before calling
/// RunSingleCase.
struct SingleRunConfig {
    std::string model_file;
    std::string simulation_file;

    /// Either a resolved hydro file path, or a pre-patched YAMLHydroData
    /// (used by campaign mode to override wave parameters).
    /// An empty string means "run without hydrodynamics".
    std::variant<std::string, seastack::hydro::YAMLHydroData> hydro_source;

    /// Where to write per-run HDF5.  Empty string disables HDF5 export
    /// (metrics are still captured in-memory for the result struct).
    std::string output_directory;

    /// Directory for hydro diagnostics (e.g. RIRF summary CSVs).  Empty means
    /// fall back to output_directory, then hydro YAML parent path (see RunSingleCase).
    std::string diagnostics_output_directory;

    /// When hydro_source holds YAMLHydroData, path to the original hydro YAML
    /// (used for diagnostics output location when diagnostics_output_directory
    /// and output_directory are empty).
    std::string hydro_yaml_path;

    seastack::chrono::ExportConfig export_config;
    std::string cli_output_level;  ///< Optional CLI-level override for export level

    /// When true, RunSingleCase fills pto_total_* on success (decimated series from exporter).
    bool capture_pto_total_timeseries = false;

    std::string cell_label;   ///< Human-readable label for logging (e.g. "Hs=1.5_Tp=8")
    /// Optional log file path for completion summary (YAML runner only).
    std::string cli_log_file_path;
    /// When true (e.g. power-matrix cells), hide repetitive startup banners unless debug_mode.
    bool concise_cli    = false;
    bool nogui          = true;
    bool debug_mode     = false;
    bool trace_mode     = false;
    bool profile_mode   = false;

    /// Optional out-of-process PTO(s) (populated from setup YAML when enabled).
    /// `external_ptos` holds one entry per attached link; `external_pto` mirrors
    /// the first entry for backward compatibility.
    bool has_external_pto = false;
    seastack::infra::SetupConfig::ExternalPtoConfig external_pto;
    std::vector<seastack::infra::SetupConfig::ExternalPtoConfig> external_ptos;
};

/// Per-PTO viscous damper power metrics from a single run (see SimulationExporter::GetPtoSummary).
struct PtoMetrics {
    std::string name;
    double mean_absorbed_power_W   = 0.0;
    double final_absorbed_energy_J = 0.0;
};

/// Structured result from a single simulation run.
struct SingleRunResult {
    int exit_code         = 1;
    bool diverged         = false;
    double sim_time_final = 0.0;
    double wall_time_s    = 0.0;
    std::string error_message;          ///< Empty on success

    std::vector<PtoMetrics> pto_metrics;
    double total_mean_power_W = 0.0;    ///< Sum across all PTOs
    double total_energy_J     = 0.0;    ///< Sum across all PTOs

    /// Optional: decimated time [s] and total PTO power [W] (sum of links); same length.
    std::vector<double> pto_total_time_s;
    std::vector<double> pto_total_power_W;

    /// Resolved HDF5 path when results were written to a user output directory.
    std::string primary_artifact_path;
    /// When non-empty, explains missing HDF5 (e.g. metrics-only / temp directory).
    std::string artifact_note;
};

/// Execute a single sea-state simulation and return structured metrics.
///
/// Constructs a fresh ChSystem + HydroSystem, runs the time loop,
/// optionally writes HDF5, and returns.  All objects are destroyed on
/// return — safe to call repeatedly in a loop.
SingleRunResult RunSingleCase(const SingleRunConfig& config);

}  // namespace seastack::app

#endif  // SEASTACK_APP_SINGLE_RUN_H
