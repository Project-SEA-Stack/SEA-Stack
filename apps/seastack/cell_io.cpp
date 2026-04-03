/*********************************************************************
 * @file  cell_io.cpp
 * @brief YAML serialization for subprocess cell execution IPC.
 *********************************************************************/

#include "cell_io.h"
#include <seastack/hydro/config/yaml_parser.h>
#include <seastack/hydro/config/hydro_config.h>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <stdexcept>
#include <string>

namespace seastack::app {

void WriteCellConfigYAML(const std::string& path,
                         const SingleRunConfig& config,
                         double hs, double tp, double heading_deg, int seed) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "model_file"      << YAML::Value << config.model_file;
    out << YAML::Key << "simulation_file" << YAML::Value << config.simulation_file;

    // Write hydro file path (for subprocess to load and patch)
    if (auto* hp = std::get_if<std::string>(&config.hydro_source)) {
        out << YAML::Key << "hydro_file" << YAML::Value << *hp;
    }

    out << YAML::Key << "hs"          << YAML::Value << hs;
    out << YAML::Key << "tp"          << YAML::Value << tp;
    out << YAML::Key << "heading_deg" << YAML::Value << heading_deg;
    out << YAML::Key << "seed"        << YAML::Value << seed;

    out << YAML::Key << "output_directory" << YAML::Value << config.output_directory;
    if (!config.diagnostics_output_directory.empty()) {
        out << YAML::Key << "diagnostics_output_directory" << YAML::Value
            << config.diagnostics_output_directory;
    }
    out << YAML::Key << "cell_label"       << YAML::Value << config.cell_label;
    out << YAML::Key << "concise_cli"      << YAML::Value << config.concise_cli;
    out << YAML::Key << "nogui"            << YAML::Value << config.nogui;
    out << YAML::Key << "debug_mode"       << YAML::Value << config.debug_mode;
    out << YAML::Key << "profile_mode"     << YAML::Value << config.profile_mode;
    out << YAML::Key << "cli_output_level" << YAML::Value << config.cli_output_level;

    // Export config
    {
        std::string level_str = "standard";
        if (config.export_config.level == seastack::chrono::ExportLevel::kCompact)
            level_str = "compact";
        else if (config.export_config.level == seastack::chrono::ExportLevel::kDetailed)
            level_str = "detailed";
        out << YAML::Key << "export_level"       << YAML::Value << level_str;
        out << YAML::Key << "export_decimation"  << YAML::Value << config.export_config.decimation;
        out << YAML::Key << "export_compression" << YAML::Value << config.export_config.compression;
        out << YAML::Key << "export_float32"     << YAML::Value << config.export_config.use_float32;
    }

    out << YAML::EndMap;

    std::ofstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot write cell config: " + path);
    f << out.c_str();
}

SingleRunConfig ReadCellConfigYAML(const std::string& path) {
    auto node = YAML::LoadFile(path);
    SingleRunConfig cfg;
    cfg.model_file      = node["model_file"].as<std::string>();
    cfg.simulation_file = node["simulation_file"].as<std::string>();

    // Load hydro file, then patch wave overrides
    std::string hydro_file;
    if (node["hydro_file"]) {
        hydro_file = node["hydro_file"].as<std::string>();
    }

    if (!hydro_file.empty()) {
        cfg.hydro_yaml_path = hydro_file;
        auto hydro_data = seastack::hydro::ReadHydroYAML(hydro_file);
        hydro_data.waves.height    = node["hs"].as<double>();
        hydro_data.waves.period    = node["tp"].as<double>();
        hydro_data.waves.direction = node["heading_deg"].as<double>();
        hydro_data.waves.seed      = node["seed"].as<int>();
        cfg.hydro_source = hydro_data;
    } else {
        cfg.hydro_source = std::string{};
    }

    if (node["output_directory"]) cfg.output_directory = node["output_directory"].as<std::string>();
    if (node["diagnostics_output_directory"]) {
        cfg.diagnostics_output_directory =
            node["diagnostics_output_directory"].as<std::string>();
    }
    if (node["cell_label"])       cfg.cell_label       = node["cell_label"].as<std::string>();
    if (node["concise_cli"])      cfg.concise_cli      = node["concise_cli"].as<bool>();
    if (node["nogui"])            cfg.nogui            = node["nogui"].as<bool>();
    if (node["debug_mode"])       cfg.debug_mode       = node["debug_mode"].as<bool>();
    if (node["profile_mode"])     cfg.profile_mode     = node["profile_mode"].as<bool>();
    if (node["cli_output_level"]) cfg.cli_output_level = node["cli_output_level"].as<std::string>();

    // Export config
    if (node["export_level"]) {
        std::string lv = node["export_level"].as<std::string>();
        if (lv == "compact")       cfg.export_config.level = seastack::chrono::ExportLevel::kCompact;
        else if (lv == "detailed") cfg.export_config.level = seastack::chrono::ExportLevel::kDetailed;
        else                       cfg.export_config.level = seastack::chrono::ExportLevel::kStandard;
    }
    if (node["export_decimation"])  cfg.export_config.decimation   = node["export_decimation"].as<int>();
    if (node["export_compression"]) cfg.export_config.compression  = node["export_compression"].as<bool>();
    if (node["export_float32"])     cfg.export_config.use_float32  = node["export_float32"].as<bool>();

    return cfg;
}

void WriteCellResultYAML(const std::string& path,
                         const SingleRunResult& result) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "exit_code"           << YAML::Value << result.exit_code;
    out << YAML::Key << "diverged"            << YAML::Value << result.diverged;
    out << YAML::Key << "sim_time_final"      << YAML::Value << result.sim_time_final;
    out << YAML::Key << "wall_time_s"         << YAML::Value << result.wall_time_s;
    out << YAML::Key << "error_message"       << YAML::Value << result.error_message;
    out << YAML::Key << "total_mean_power_W"  << YAML::Value << result.total_mean_power_W;
    out << YAML::Key << "total_energy_J"      << YAML::Value << result.total_energy_J;
    out << YAML::EndMap;

    std::ofstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot write cell result: " + path);
    f << out.c_str();
}

SingleRunResult ReadCellResultYAML(const std::string& path) {
    auto node = YAML::LoadFile(path);
    SingleRunResult result;
    if (node["exit_code"])          result.exit_code         = node["exit_code"].as<int>();
    if (node["diverged"])           result.diverged          = node["diverged"].as<bool>();
    if (node["sim_time_final"])     result.sim_time_final    = node["sim_time_final"].as<double>();
    if (node["wall_time_s"])        result.wall_time_s       = node["wall_time_s"].as<double>();
    if (node["error_message"])      result.error_message     = node["error_message"].as<std::string>();
    if (node["total_mean_power_W"]) result.total_mean_power_W = node["total_mean_power_W"].as<double>();
    if (node["total_energy_J"])     result.total_energy_J    = node["total_energy_J"].as<double>();
    return result;
}

}  // namespace seastack::app
