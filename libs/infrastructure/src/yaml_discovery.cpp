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

namespace {

// Trim leading/trailing whitespace in place.
void TrimInPlace(std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) { s.clear(); return; }
    s.erase(0, b);
    s.erase(s.find_last_not_of(" \t\r\n") + 1);
}

// Apply one `key: value` pair to a TankConfig. Shared by the inline setup
// `tank:` block and the dedicated `sph_file` parser so both accept identical
// keys.
void ApplyTankKey(SetupConfig::TankConfig& tank, const std::string& key, const std::string& value) {
    auto as_double = [&](double& out) { try { out = std::stod(value); } catch (...) {} };
    if (key == "length") as_double(tank.length);
    else if (key == "width") as_double(tank.width);
    else if (key == "height") as_double(tank.height);
    else if (key == "fill_depth") as_double(tank.fill_depth);
    else if (key == "deck_x") as_double(tank.deck_x);
    else if (key == "deck_y") as_double(tank.deck_y);
    else if (key == "deck_z") as_double(tank.deck_z);
    else if (key == "spacing") as_double(tank.spacing);
    else if (key == "fluid_density") as_double(tank.fluid_density);
    else if (key == "fluid_viscosity") as_double(tank.fluid_viscosity);
    else if (key == "cfd_step") as_double(tank.cfd_step);
    else if (key == "mbd_step") as_double(tank.mbd_step);
    else if (key == "hull_body") tank.hull_body = value;
    else if (key == "rebalance_mass")
        tank.rebalance_mass = (value == "true" || value == "1" || value == "yes");
    else if (key == "num_bce_layers") { try { tank.num_bce_layers = std::stoi(value); } catch (...) {} }
    else if (key == "artificial_viscosity") as_double(tank.artificial_viscosity);
    else if (key == "max_velocity") as_double(tank.max_velocity);
    else if (key == "roll_rate") as_double(tank.roll_rate);
}

// Parse a dedicated SPH fluid-definition file (`*.sph.yaml`) for its `tank:`
// block, applying keys into `tank`. Returns true if a `tank:` block was found.
// The file format mirrors the inline setup `tank:` block: a top-level `tank:`
// key followed by indented sub-keys.
bool ParseSphFileInto(const std::filesystem::path& sph_path, SetupConfig::TankConfig& tank) {
    std::ifstream file(sph_path);
    if (!file.is_open()) {
        seastack::infra::cli::LogWarning("Could not open sph_file: " + sph_path.string());
        return false;
    }

    bool in_tank_block = false;
    bool found_tank = false;
    std::string line;
    while (std::getline(file, line)) {
        size_t indent = line.find_first_not_of(" \t");
        std::string trimmed = line;
        trimmed.erase(0, indent);
        trimmed.erase(trimmed.find_last_not_of(" \t\r\n") + 1);
        if (trimmed.empty() || trimmed[0] == '#') continue;

        if (in_tank_block && indent != std::string::npos && indent > 0) {
            size_t colon_pos = trimmed.find(':');
            if (colon_pos == std::string::npos) continue;
            std::string key = trimmed.substr(0, colon_pos);
            std::string value = trimmed.substr(colon_pos + 1);
            size_t cp = value.find('#');
            if (cp != std::string::npos) value = value.substr(0, cp);
            TrimInPlace(key);
            TrimInPlace(value);
            ApplyTankKey(tank, key, value);
            continue;
        }

        // Top-level line ends any nested block.
        if (indent == 0 || indent == std::string::npos) {
            in_tank_block = false;
        }

        size_t colon_pos = trimmed.find(':');
        if (colon_pos == std::string::npos) continue;
        std::string key = trimmed.substr(0, colon_pos);
        std::string value = trimmed.substr(colon_pos + 1);
        TrimInPlace(key);
        TrimInPlace(value);
        if (key == "tank" && value.empty()) {
            in_tank_block = true;
            found_tank = true;
        }
    }
    return found_tank;
}

}  // namespace

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
    bool in_tank_block = false;
    while (std::getline(file, line)) {
        // Detect indentation for nested blocks
        size_t indent = line.find_first_not_of(" \t");
        std::string trimmed = line;
        trimmed.erase(0, indent);
        trimmed.erase(trimmed.find_last_not_of(" \t\r\n") + 1);

        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        // If indented and we're inside the tank: block, parse sub-keys
        if (in_tank_block && indent > 0) {
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

            ApplyTankKey(config.tank, key, value);
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

        // Top-level line resets any nested block context
        if (indent == 0 || indent == std::string::npos) {
            in_output_block = false;
            in_tank_block = false;
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
        } else if (key == "tank" && value.empty()) {
            in_tank_block = true;
            config.has_tank = true;
            seastack::infra::debug::LogDebug("SPH deck-sloshing tank block found");
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
        } else if (key == "fsi_file") {
            config.fsi_file = value;
            config.has_fsi_file = true;
            seastack::infra::debug::LogDebug(std::string("FSI (SPH) file: ") + value);
        } else if (key == "sph_file") {
            config.sph_file = value;
            config.has_sph_file = true;
            seastack::infra::debug::LogDebug(std::string("SPH file: ") + value);
        } else if (key == "output_directory") {
            config.output_directory = value;
            config.has_output_directory = true;
        }
    }

    // Resolve a dedicated SPH fluid-definition file (relative to the setup
    // file's directory) and parse its `tank:` block. This mirrors an inline
    // `tank:` block but keeps SPH fidelity in its own swappable file.
    if (config.has_sph_file && !config.sph_file.empty()) {
        std::filesystem::path sph_path = config.sph_file;
        if (!sph_path.is_absolute()) {
            sph_path = setup_path.parent_path() / sph_path;
        }
        if (ParseSphFileInto(sph_path, config.tank)) {
            config.has_tank = true;
            seastack::infra::debug::LogDebug("SPH deck-sloshing tank block found (from sph_file)");
        } else {
            seastack::infra::cli::LogWarning(
                "sph_file '" + config.sph_file + "' has no tank: block; ignoring.");
        }
    }

    return config;
}

} // namespace seastack::infra

