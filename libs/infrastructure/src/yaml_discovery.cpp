/*********************************************************************
 * @file  yaml_discovery.cpp
 *
 * @brief Implementation of setup file discovery and parsing.
 *********************************************************************/

#include <seastack/infra/config/yaml_discovery.h>
#include <seastack/infra/logging.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>

namespace seastack::infra {

std::filesystem::path FindSetupFile(const std::filesystem::path& directory) {
    // First try the traditional model.setup.yaml for backward compatibility
    auto setup_path = directory / "model.setup.yaml";
    if (std::filesystem::exists(setup_path) && std::filesystem::is_regular_file(setup_path)) {
        return setup_path;
    }
    
    // Then search for any *.setup.yaml files
    try {
        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (entry.is_regular_file()) {
                const auto& path = entry.path();
                const std::string filename = path.filename().string();
                const std::string suffix = ".setup.yaml";
                if (path.extension() == ".yaml" && 
                    filename.length() >= suffix.length() && 
                    filename.compare(filename.length() - suffix.length(), suffix.length(), suffix) == 0) {
                    return path;
                }
            }
        }
    } catch (const std::filesystem::filesystem_error&) {
        // Directory doesn't exist or can't be read
    }
    
    return {};
}

SetupConfig ParseSetupFile(const std::filesystem::path& setup_path) {
    SetupConfig config;
    
    std::ifstream file(setup_path);
    if (!file.is_open()) {
        seastack::infra::cli::LogWarning("Could not open setup file: " + setup_path.string());
        return config;
    }
    
    std::string line;
    bool in_output_block = false;
    while (std::getline(file, line)) {
        // Detect indentation for nested blocks
        size_t indent = line.find_first_not_of(" \t");
        std::string trimmed = line;
        trimmed.erase(0, indent);
        trimmed.erase(trimmed.find_last_not_of(" \t\r\n") + 1);

        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        // If indented and we're inside the output: block, parse sub-keys
        if (in_output_block && indent > 0) {
            size_t colon_pos = trimmed.find(':');
            if (colon_pos == std::string::npos) continue;
            std::string key = trimmed.substr(0, colon_pos);
            std::string value = trimmed.substr(colon_pos + 1);
            size_t cp = value.find('#');
            if (cp != std::string::npos) value = value.substr(0, cp);
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            if (key == "level") {
                config.output_config.level = value;
            } else if (key == "decimation") {
                try { config.output_config.decimation = std::stoi(value); } catch (...) {}
            } else if (key == "precision") {
                config.output_config.precision = value;
            } else if (key == "compression") {
                config.output_config.compression = (value == "true" || value == "1" || value == "yes");
            }
            config.has_output_config = true;
            continue;
        }

        // Top-level line resets the output block context
        if (indent == 0 || indent == std::string::npos) {
            in_output_block = false;
        }

        size_t colon_pos = trimmed.find(':');
        if (colon_pos == std::string::npos) continue;

        std::string key = trimmed.substr(0, colon_pos);
        std::string value = trimmed.substr(colon_pos + 1);
        size_t cp = value.find('#');
        if (cp != std::string::npos) value = value.substr(0, cp);
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

        if (key == "output" && value.empty()) {
            in_output_block = true;
        } else if (key == "model_file") {
            config.model_file = value;
            config.has_model_file = true;
        } else if (key == "simulation_file") {
            config.simulation_file = value;
            config.has_simulation_file = true;
        } else if (key == "hydro_file") {
            config.hydro_file = value;
            config.has_hydro_file = true;
            seastack::infra::debug::LogDebug(std::string("Hydrodynamics file: ") + value);
        } else if (key == "output_directory") {
            config.output_directory = value;
            config.has_output_directory = true;
        } else if (key == "vehicle_file") {
            config.vehicle_file = value;
            config.has_vehicle_file = true;
            seastack::infra::debug::LogDebug(std::string("Vehicle file: ") + value);
        } else if (key == "structure_file") {
            config.structure_file = value;
            config.has_structure_file = true;
            seastack::infra::debug::LogDebug(std::string("Structure file: ") + value);
        }
    }

    // Nested `checks:` is loaded by the app layer (yaml-cpp). Infra stays
    // stdlib-only; see LoadChecksFromSetupYaml in apps/seastack.
    
    return config;
}

} // namespace seastack::infra

