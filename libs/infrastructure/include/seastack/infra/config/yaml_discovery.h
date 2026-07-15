/*********************************************************************
 * @file  yaml_discovery.h
 *
 * @brief Discovery and parsing of model.setup.yaml files.
 *
 * This module handles finding and parsing setup files that point to
 * other configuration files (model, simulation, hydro, output).
 *********************************************************************/

#ifndef SEASTACK_INFRA_CONFIG_YAML_DISCOVERY_H
#define SEASTACK_INFRA_CONFIG_YAML_DISCOVERY_H

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace seastack::infra {

/// Structure to hold parsed setup file configuration
struct SetupConfig {
    std::string model_file;
    std::string simulation_file;
    std::string hydro_file;
    std::string output_directory;
    
    bool has_model_file = false;
    bool has_simulation_file = false;
    bool has_hydro_file = false;
    bool has_output_directory = false;

    // Output configuration (from `output:` block in setup YAML)
    struct OutputConfig {
        std::string level = "standard";    // compact | standard | detailed
        int decimation = 1;
        std::string precision = "float64"; // float64 | float32
        bool compression = true;
    };
    OutputConfig output_config;
    bool has_output_config = false;

    /// Optional out-of-process PTO module (requires SEASTACK_ENABLE_EXTERNAL).
    struct ExternalPtoConfig {
        std::string link_name;                 ///< ChLinkTSDA or ChLinkRSDA name
        std::vector<std::string> command;      ///< argv to spawn
        std::string config_json = "{}";        ///< JSON object for module config
        std::string working_directory;         ///< optional cwd for child
        int timeout_ms = 10000;
        bool rich_state = true;                ///< publish full kinematics channel list
    };
    ExternalPtoConfig external_pto;
    bool has_external_pto = false;
};

/// Parse a model.setup.yaml file and return configuration
/// @param setup_path Path to the model.setup.yaml file
/// @return SetupConfig structure with parsed values
SetupConfig ParseSetupFile(const std::filesystem::path& setup_path);

/// Check if a model.setup.yaml file exists in the given directory
/// @param directory Directory to check for model.setup.yaml
/// @return Path to setup file if it exists, empty path otherwise
std::filesystem::path FindSetupFile(const std::filesystem::path& directory);

} // namespace seastack::infra

#endif  // SEASTACK_INFRA_CONFIG_YAML_DISCOVERY_H

