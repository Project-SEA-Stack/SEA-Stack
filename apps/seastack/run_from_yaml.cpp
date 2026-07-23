/*********************************************************************
 * @file  run_from_yaml.cpp
 * @brief CLI wrapper: parses arguments, resolves files, then delegates
 *        to RunSingleCase() for the actual simulation.
 *********************************************************************/

#include "run_from_yaml.h"
#include "single_run.h"
#include "app_init.h"

#include <seastack/config.h>
#include <seastack/version.h>
#include <seastack/infra/config/yaml_discovery.h>
#include <seastack/infra/logging.h>

#ifdef SEASTACK_HAVE_EXTERNAL
#include "external_pto_yaml.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

namespace seastack::app {

using seastack::infra::FindSetupFile;
using seastack::infra::ParseSetupFile;
using seastack::infra::SetupConfig;

// -----------------------------------------------------------------------------
// Utility: Find the first file matching a pattern in a directory.
// -----------------------------------------------------------------------------
static std::string FindFirstFile(const std::filesystem::path& directory, const std::string& pattern) {
    if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
        return "";
    }
    
    std::vector<std::filesystem::path> matches;
    
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().generic_string();
            if (filename.find(pattern) != std::string::npos) {
                matches.push_back(entry.path());
            }
        }
    }
    
    if (!matches.empty()) {
        std::sort(matches.begin(), matches.end());
        return matches.front().generic_string();
    }
    
    return "";
}

// -----------------------------------------------------------------------------
// Best-effort YAML probe (used for CLI display during startup).
// -----------------------------------------------------------------------------
static bool TryFindYamlDouble(const std::string& yaml_path, const std::string& key, double& out_value) {
    std::ifstream in(yaml_path);
    if (!in.is_open()) {
        return false;
    }
    auto ltrim = [](std::string& s) { s.erase(0, s.find_first_not_of(" \t\r\n")); };
    auto rtrim = [](std::string& s) { size_t p = s.find_last_not_of(" \t\r\n"); if (p == std::string::npos) s.clear(); else s.erase(p + 1); };
    std::string line;
    while (std::getline(in, line)) {
        ltrim(line); rtrim(line);
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find(':');
        if (pos == std::string::npos) continue;
        std::string k = line.substr(0, pos);
        std::string v = line.substr(pos + 1);
        ltrim(k); rtrim(k); ltrim(v); rtrim(v);
        if (k == key) {
            try {
                out_value = std::stod(v);
                return true;
            } catch (...) {
                return false;
            }
        }
    }
    return false;
}

// Helper: resolve model/sim/hydro paths from setup file + CLI overrides.
static bool ResolveInputFiles(const std::filesystem::path& input_dir, 
                      const std::string& model_file_arg, 
                      const std::string& sim_file_arg,
                      std::string& model_file, 
                      std::string& sim_file, 
                      SetupConfig& setup_config) {
    
    seastack::infra::debug::LogDebug("Checking for setup file...");
    auto setup_file_path = FindSetupFile(input_dir);
    bool using_setup_file = false;
    
    if (!setup_file_path.empty()) {
        seastack::infra::debug::LogDebug(std::string("Setup file found: ") + setup_file_path.generic_string());
        using_setup_file = true;
        setup_config = ParseSetupFile(setup_file_path);
        seastack::infra::debug::LogDebug("Setup file loaded");
#ifdef SEASTACK_HAVE_EXTERNAL
        try {
            LoadExternalPtoFromSetupYaml(setup_file_path, setup_config);
        } catch (const std::exception& e) {
            seastack::infra::cli::LogError(
                std::string("Failed to parse external_pto: ") + e.what());
            return false;
        }
#endif
        
        if (!model_file_arg.empty()) {
            model_file = model_file_arg;
            if (!std::filesystem::path(model_file).is_absolute()) {
                model_file = (input_dir / model_file).generic_string();
            }
        } else if (setup_config.has_model_file) {
            model_file = (input_dir / setup_config.model_file).generic_string();
            seastack::infra::debug::LogDebug(std::string("Model file from setup: ") + setup_config.model_file);
        }
        
        if (!sim_file_arg.empty()) {
            sim_file = sim_file_arg;
            if (!std::filesystem::path(sim_file).is_absolute()) {
                sim_file = (input_dir / sim_file).generic_string();
            }
        } else if (setup_config.has_simulation_file) {
            sim_file = (input_dir / setup_config.simulation_file).generic_string();
            seastack::infra::debug::LogDebug(std::string("Simulation file from setup: ") + setup_config.simulation_file);
        }
    } else {
        seastack::infra::debug::LogDebug("No setup file found, using command line arguments");
    }

    if (!using_setup_file || model_file.empty()) {
        if (!model_file_arg.empty()) {
            model_file = model_file_arg;
            if (!std::filesystem::path(model_file).is_absolute()) {
                model_file = (input_dir / model_file).generic_string();
            }
        } else {
            model_file = FindFirstFile(input_dir, ".model.yaml");
            if (model_file.empty()) {
                seastack::infra::cli::LogError("Could not find .model.yaml file");
                seastack::infra::cli::LogError(std::string("Directory: ") + input_dir.generic_string());
                return false;
            }
        }
    }

    if (!using_setup_file || sim_file.empty()) {
        if (!sim_file_arg.empty()) {
            sim_file = sim_file_arg;
            if (!std::filesystem::path(sim_file).is_absolute()) {
                sim_file = (input_dir / sim_file).generic_string();
            }
        } else {
            sim_file = FindFirstFile(input_dir, ".simulation.yaml");
            if (sim_file.empty()) {
                seastack::infra::cli::LogError("Could not find .simulation.yaml file");
                seastack::infra::cli::LogError(std::string("Directory: ") + input_dir.generic_string());
                return false;
            }
        }
    }

    seastack::infra::debug::LogDebug("Validating input files...");
    if (!std::filesystem::exists(model_file)) {
        seastack::infra::cli::LogError(std::string("Model file does not exist: ") + model_file);
        return false;
    }
    if (!std::filesystem::exists(sim_file)) {
        seastack::infra::cli::LogError(std::string("Simulation file does not exist: ") + sim_file);
        return false;
    }
    seastack::infra::debug::LogDebug("All input files validated successfully");
    
    return true;
}

// CLI display of simulation summary (before the run starts).
static void DisplaySimulationSummary(const std::string& input_directory,
                             const std::string& model_file,
                             const std::string& sim_file,
                             const SetupConfig& setup_config,
                             bool nogui,
                             const std::string& resolved_output_directory) {
    
    double timestep = 0.0;
    TryFindYamlDouble(sim_file, "time_step", timestep);
    if (timestep <= 0.0) timestep = 0.001;
    
    std::vector<std::string> summary_content;

    // Trailing separators make path::filename() empty (e.g. demos\case\).
    std::filesystem::path case_path(input_directory);
    while (!case_path.empty() &&
           (case_path.filename().empty() || case_path.filename() == "." ||
            case_path.filename() == "..")) {
        case_path = case_path.parent_path();
    }
    const std::string case_name = case_path.filename().string();
    
    summary_content.push_back(seastack::infra::cli::CreateAlignedLine(
        "🎯", "Case", case_name.empty() ? "(unnamed)" : case_name));
    summary_content.push_back(seastack::infra::cli::CreateAlignedLine(
        "📁", "Directory", seastack::infra::FormatCliPathForDisplay(input_directory)));
    summary_content.push_back(seastack::infra::cli::CreateAlignedLine("📄", "Model", std::filesystem::path(model_file).filename().string()));
    summary_content.push_back(seastack::infra::cli::CreateAlignedLine("⚙️", "Config", std::filesystem::path(sim_file).filename().string()));
    
    if (setup_config.has_hydro_file) {
        summary_content.push_back(seastack::infra::cli::CreateAlignedLine("🌊", "Hydro", setup_config.hydro_file));
    } else {
        summary_content.push_back(seastack::infra::cli::CreateAlignedLine("🌊", "Hydro", "None (no forces)"));
    }

    if (setup_config.has_external_pto) {
        if (setup_config.external_ptos.size() > 1) {
            summary_content.push_back(seastack::infra::cli::CreateAlignedLine(
                "🔌", "External PTO",
                std::to_string(setup_config.external_ptos.size()) +
                    " links (external_ptos)"));
        } else if (setup_config.has_external_pto_file) {
            summary_content.push_back(seastack::infra::cli::CreateAlignedLine(
                "🔌", "External PTO",
                std::filesystem::path(setup_config.external_pto_file)
                    .filename()
                    .string()));
        } else {
            summary_content.push_back(seastack::infra::cli::CreateAlignedLine(
                "🔌", "External PTO", "inline (setup YAML)"));
        }
    }
    
    summary_content.push_back("");
    
    double simulation_duration = 0.0;
    const bool has_end =
        TryFindYamlDouble(sim_file, "end_time", simulation_duration) && simulation_duration > 0.0;
    std::string time_value;
    if (has_end) {
        time_value = std::string("0–") + seastack::infra::FormatNumber(simulation_duration, 1) + " s, dt = " +
                     seastack::infra::FormatNumber(timestep, 3) + " s";
    } else {
        time_value = std::string("dt = ") + seastack::infra::FormatNumber(timestep, 3) + " s (no end_time in config)";
    }
    summary_content.push_back(seastack::infra::cli::CreateAlignedLine("⏱️", "Time", time_value));
    summary_content.push_back(seastack::infra::cli::CreateAlignedLine("🖥️", "Mode", nogui ? "Headless" : "GUI"));
    
    if (!resolved_output_directory.empty()) {
        summary_content.push_back(seastack::infra::cli::CreateAlignedLine(
            "📁", "Output", seastack::infra::FormatCliPathForDisplay(resolved_output_directory)));
    } else {
        summary_content.push_back(seastack::infra::cli::CreateAlignedLine(
            "📁", "Output", "(no output directory — HDF5 not saved under case)"));
    }
    
    seastack::infra::cli::ShowSectionBox("SEA-Stack Case", summary_content);
    seastack::infra::cli::ShowEmptyLine();
}

// =============================================================================
// RunFromYAML — thin CLI wrapper around RunSingleCase
// =============================================================================
int RunFromYAML(int argc, char* argv[]) {
    try {
#ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);
#endif

        // -----------------------------------------------------------------
        // 1. Parse CLI arguments
        // -----------------------------------------------------------------
        std::string model_file_arg;
        std::string sim_file_arg;
        std::string input_directory;
        std::string cli_output_level;
        bool nogui = false;
        bool quiet_mode = false;
        bool enable_logging = false;
        bool debug_mode = false;
        bool trace_mode = false;
        bool profile_mode = false;
        bool nobanner = false;

        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if ((arg == "--model" || arg == "--model_file") && i + 1 < argc) {
                model_file_arg = argv[++i];
            } else if ((arg == "--sim" || arg == "--sim_file") && i + 1 < argc) {
                sim_file_arg = argv[++i];
            } else if (arg == "--output-level" && i + 1 < argc) {
                cli_output_level = argv[++i];
            } else if (arg == "--nogui") {
                nogui = true;
            } else if (arg == "--log") {
                enable_logging = true;
            } else if (arg == "--no-log") {
                enable_logging = false;
            } else if (arg == "--debug") {
                debug_mode = true;
            } else if (arg == "--trace") {
                trace_mode = true;
                debug_mode = true;
            } else if (arg == "--profile") {
                profile_mode = true;
            } else if (arg == "--nobanner") {
                nobanner = true;
            } else if (arg == "--quiet") {
                quiet_mode = true;
            } else if (arg.substr(0, 1) != "-") {
                if (input_directory.empty()) {
                    input_directory = arg;
                }
            }
        }

        // -----------------------------------------------------------------
        // 2. Validate input directory
        // -----------------------------------------------------------------
        if (input_directory.empty()) {
            seastack::infra::cli::LogError("ERROR: No input directory provided to runner");
            return 1;
        }
        
        std::filesystem::path input_dir(input_directory);
        if (!std::filesystem::exists(input_dir)) {
            seastack::infra::cli::LogError(std::string("ERROR: Input directory does not exist: ") + input_directory);
            return 1;
        }

        // -----------------------------------------------------------------
        // 3. Setup logging
        // -----------------------------------------------------------------
        std::string log_file_path;
        if (enable_logging) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << "seastack_yaml_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".log";
            
            std::filesystem::path logs_dir = input_dir / "logs";
                if (!std::filesystem::exists(logs_dir)) {
                    std::filesystem::create_directories(logs_dir);
            }
            
            log_file_path = (logs_dir / ss.str()).generic_string();
        }
        
        seastack::infra::LoggingConfig log_cfg;
        log_cfg.log_file_path = log_file_path;
        log_cfg.enable_cli_output = !quiet_mode;
        log_cfg.enable_file_output = !log_file_path.empty();
        log_cfg.enable_debug_logging = debug_mode;
        log_cfg.console_level = debug_mode ? seastack::infra::LogLevel::Debug
                                           : seastack::infra::LogLevel::Info;
        log_cfg.file_level = log_cfg.enable_file_output
                                 ? seastack::infra::LogLevel::Debug
                                 : seastack::infra::LogLevel::Info;
        if (!seastack::infra::Initialize(log_cfg)) {
            std::cerr << "Warning: Failed to initialize logging system\n";
        }
        if (!quiet_mode && !nobanner) {
            seastack::infra::cli::ShowBannerCompact();
        }
        seastack::infra::debug::LogDebug(std::string("[startup] SEA-Stack v") + SEASTACK_VERSION + " starting");

        // -----------------------------------------------------------------
        // 4. Configure Chrono data path (shared with campaign/run-cell modes)
        // -----------------------------------------------------------------
        seastack::app::InitChronoEnvironment();

        // -----------------------------------------------------------------
        // 5. Resolve input files
        // -----------------------------------------------------------------
        std::string model_file;
        std::string sim_file;
        SetupConfig setup_config;
        
        if (!ResolveInputFiles(input_dir, model_file_arg, sim_file_arg, model_file, sim_file, setup_config)) {
            seastack::infra::cli::LogError("[startup] Input file resolution failed. Check that the input directory contains valid YAML files.");
            seastack::infra::Shutdown();
            return 1;
        }
        seastack::infra::debug::LogDebug("[startup] Input files resolved");

        // -----------------------------------------------------------------
        // 6. Display pre-run summary
        // -----------------------------------------------------------------
        std::string resolved_output_directory;
        if (setup_config.has_output_directory && !setup_config.output_directory.empty()) {
            resolved_output_directory =
                (std::filesystem::path(input_directory) / setup_config.output_directory).generic_string();
        }
        if (!quiet_mode) {
            DisplaySimulationSummary(input_directory, model_file, sim_file, setup_config, nogui,
                                     resolved_output_directory);
        }

        // -----------------------------------------------------------------
        // 7. Build SingleRunConfig and delegate to RunSingleCase
        // -----------------------------------------------------------------
        SingleRunConfig run_config;
        run_config.model_file      = model_file;
        run_config.simulation_file = sim_file;

        if (setup_config.has_hydro_file) {
            run_config.hydro_source = (std::filesystem::path(input_directory) / setup_config.hydro_file).generic_string();
        } else {
            run_config.hydro_source = std::string{};
        }

        if (setup_config.has_output_directory && !setup_config.output_directory.empty()) {
            run_config.output_directory = (std::filesystem::path(input_directory) / setup_config.output_directory).generic_string();
        }

        if (setup_config.has_output_config) {
            const auto& oc = setup_config.output_config;
            if (oc.level == "compact")       run_config.export_config.level = seastack::chrono::ExportLevel::kCompact;
            else if (oc.level == "detailed")  run_config.export_config.level = seastack::chrono::ExportLevel::kDetailed;
            else                              run_config.export_config.level = seastack::chrono::ExportLevel::kStandard;
            run_config.export_config.decimation  = oc.decimation;
            run_config.export_config.compression = oc.compression;
            run_config.export_config.use_float32 = (oc.precision == "float32");
        }

        run_config.cli_output_level = cli_output_level;
        run_config.cli_log_file_path = log_file_path;
        run_config.nogui        = nogui;
        run_config.debug_mode   = debug_mode;
        run_config.trace_mode   = trace_mode;
        run_config.profile_mode = profile_mode;
        run_config.has_external_pto = setup_config.has_external_pto;
        run_config.external_pto = setup_config.external_pto;
        run_config.external_ptos = setup_config.external_ptos;

        SingleRunResult result = RunSingleCase(run_config);

        // -----------------------------------------------------------------
        // 8. Post-run (quiet: one-line success + artifact hint)
        // -----------------------------------------------------------------
        if (quiet_mode && result.exit_code == 0) {
            std::cerr << "SEA-Stack: completed successfully";
            if (!result.primary_artifact_path.empty()) {
                std::cerr << "  Output: "
                          << seastack::infra::FormatCliPathForDisplay(result.primary_artifact_path);
            } else if (!result.artifact_note.empty()) {
                std::cerr << "  " << result.artifact_note;
            }
            std::cerr << std::endl;
        }
        seastack::infra::Shutdown();

        return result.exit_code;

    } catch (const std::exception& e) {
        seastack::infra::cli::LogError(std::string("Unhandled exception: ") + e.what());
        seastack::infra::cli::LogError("This may be a setup-phase failure (YAML loading, data path resolution, "
            "or dependency initialization) rather than a simulation-phase error.");
        seastack::infra::cli::LogError("  - Run with --doctor to check environment and package layout.");
        seastack::infra::cli::LogError("  - Run with --debug  for detailed diagnostic logs.");
        try { seastack::infra::Shutdown(); } catch (...) {}
        return 1;
    } catch (...) {
        seastack::infra::cli::LogError("Unknown fatal error during simulation startup or execution.");
        seastack::infra::cli::LogError("  - Run with --doctor to check environment and package layout.");
        seastack::infra::cli::LogError("  - Run with --debug  for detailed diagnostic logs.");
        try { seastack::infra::Shutdown(); } catch (...) {}
        return 1;
    }
}

}  // namespace seastack::app
