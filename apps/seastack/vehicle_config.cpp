/*********************************************************************
 * @file  vehicle_config.cpp
 * @brief yaml-cpp parser for the vehicle + terrain scenario schema.
 *********************************************************************/

#include "vehicle_config.h"

#include "yaml_read_helpers.h"

#include <chrono_vehicle/ChVehicleDataPath.h>
#include <chrono_vehicle/utils/ChUtilsJSON.h>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>
#include <string>

namespace seastack::app {

namespace {

using yaml_read::ReadArray;
using yaml_read::ReadBool;
using yaml_read::ReadDouble;
using yaml_read::ReadString;
using yaml_read::ToUpper;

void ParseVisualization(const YAML::Node& node, VehicleVisualizationConfig& viz) {
    if (!node) {
        return;
    }
    ReadString(node, "chassis", viz.chassis);
    ReadString(node, "sprocket", viz.sprocket);
    ReadString(node, "track_shoe", viz.track_shoe);
    ReadString(node, "suspension", viz.suspension);
    ReadString(node, "wheel", viz.wheel);
    ReadString(node, "tire", viz.tire);
}

void ParseDriver(const YAML::Node& node, DriverConfig& driver) {
    if (!node) {
        return;
    }

    std::string type = "PATH_FOLLOWER";
    ReadString(node, "type", type);
    driver.kind = (ToUpper(type) == "INTERACTIVE") ? DriverKind::Interactive
                                                   : DriverKind::PathFollower;

    if (node["path"] && node["path"].IsSequence()) {
        driver.path_xy.clear();
        for (const auto& pt : node["path"]) {
            if (pt.IsSequence() && pt.size() >= 2) {
                driver.path_xy.emplace_back(pt[0].as<double>(), pt[1].as<double>());
            }
        }
    }

    ReadDouble(node, "target_speed", driver.target_speed);
    ReadDouble(node, "start_time", driver.start_time);

    if (node["stop_at_x"]) {
        driver.has_stop_at_x = true;
        driver.stop_at_x = node["stop_at_x"].as<double>();
    }

    if (const YAML::Node steer = node["steering"]) {
        ReadDouble(steer, "lookahead", driver.steering.lookahead);
        ReadDouble(steer, "kp", driver.steering.kp);
        ReadDouble(steer, "ki", driver.steering.ki);
        ReadDouble(steer, "kd", driver.steering.kd);
    }
    if (const YAML::Node speed = node["speed"]) {
        ReadDouble(speed, "kp", driver.speed.kp);
        ReadDouble(speed, "ki", driver.speed.ki);
        ReadDouble(speed, "kd", driver.speed.kd);
    }
    if (const YAML::Node deltas = node["interactive_deltas"]) {
        ReadDouble(deltas, "steering", driver.interactive_deltas.steering);
        ReadDouble(deltas, "throttle", driver.interactive_deltas.throttle);
        ReadDouble(deltas, "braking", driver.interactive_deltas.braking);
    }
}

void ParseVehicle(const YAML::Node& node, VehicleConfig& vehicle) {
    ReadString(node, "vehicle_json", vehicle.vehicle_json);
    ReadString(node, "engine_json", vehicle.engine_json);
    ReadString(node, "transmission_json", vehicle.transmission_json);
    ReadString(node, "tire_json", vehicle.tire_json);

    if (node["initial_position"] && node["initial_position"].IsSequence() &&
        node["initial_position"].size() >= 2) {
        const YAML::Node pos = node["initial_position"];
        vehicle.initial_xy[0] = pos[0].as<double>();
        vehicle.initial_xy[1] = pos[1].as<double>();
        if (pos.size() >= 3) {
            // Third element is a number (world z) or the string "terrain".
            const std::string z_str = pos[2].as<std::string>();
            if (ToUpper(z_str) == "TERRAIN") {
                vehicle.spawn_on_terrain = true;
            } else {
                vehicle.spawn_on_terrain = false;
                vehicle.initial_z = pos[2].as<double>();
            }
        }
    }

    ReadDouble(node, "initial_yaw", vehicle.initial_yaw);
    ReadDouble(node, "ride_height", vehicle.ride_height);

    ParseVisualization(node["visualization"], vehicle.visualization);
    ParseDriver(node["driver"], vehicle.driver);

    if (const YAML::Node cam = node["chase_camera"]) {
        ReadArray(cam, "point", vehicle.chase_camera.point);
        ReadDouble(cam, "distance", vehicle.chase_camera.distance);
        ReadDouble(cam, "height", vehicle.chase_camera.height);
    }
}

void ParseTerrainMaterial(const YAML::Node& node, TerrainMaterialConfig& mat) {
    if (!node) {
        return;
    }
    ReadDouble(node, "friction", mat.friction);
    ReadDouble(node, "restitution", mat.restitution);
    ReadDouble(node, "Young_modulus", mat.young_modulus);
    ReadDouble(node, "Poisson_ratio", mat.poisson_ratio);
    ReadDouble(node, "normal_stiffness", mat.normal_stiffness);
    ReadDouble(node, "normal_damping", mat.normal_damping);
    ReadDouble(node, "tangential_stiffness", mat.tangential_stiffness);
    ReadDouble(node, "tangential_damping", mat.tangential_damping);
}

void ParseTerrain(const YAML::Node& node, TerrainConfig& terrain) {
    const YAML::Node patches = node["patches"];
    if (!patches || !patches.IsSequence()) {
        return;
    }
    for (const auto& p : patches) {
        TerrainPatchConfig patch;
        ReadString(p, "heightmap", patch.heightmap);
        ReadArray(p, "center", patch.center);
        ReadArray(p, "size", patch.size);
        ReadArray(p, "height_range", patch.height_range);
        ReadBool(p, "connected_mesh", patch.connected_mesh);
        ReadDouble(p, "sweep_sphere_radius", patch.sweep_sphere_radius);
        ParseTerrainMaterial(p["material"], patch.material);
        if (p["texture"]) {
            patch.has_texture = true;
            patch.texture = p["texture"].as<std::string>();
        }
        ReadArray(p, "color", patch.color);
        terrain.patches.push_back(std::move(patch));
    }
}

/// Infer wheeled vs tracked from the vehicle JSON's "Template" field, the same
/// way Chrono's own ChParserVehicleYAML::ReadVehicleType does.  The JSON is the
/// single source of truth: a `type:` key in the YAML is only cross-checked.
VehicleKind ReadVehicleKindFromJson(const std::string& vehicle_json) {
    const std::string resolved = ResolveVehicleDataFile(vehicle_json);
    rapidjson::Document doc;
    ::chrono::vehicle::ReadFileJSON(resolved, doc);
    if (doc.IsNull() || !doc.HasMember("Template") || !doc["Template"].IsString()) {
        throw std::runtime_error("Vehicle JSON '" + resolved +
                                 "' has no string 'Template' field, so the vehicle type "
                                 "(wheeled or tracked) cannot be determined.");
    }
    return std::string(doc["Template"].GetString()) == "WheeledVehicle" ? VehicleKind::Wheeled
                                                                        : VehicleKind::Tracked;
}

}  // namespace

std::string ResolveVehicleDataFile(const std::string& path) {
    if (std::filesystem::path(path).is_absolute() && std::filesystem::exists(path)) {
        return path;
    }
    return ::chrono::vehicle::GetVehicleDataFile(path);
}

VehicleScenarioConfig LoadVehicleConfigFromYaml(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        throw std::runtime_error("Could not load vehicle YAML '" + path + "': " + e.what());
    }

    const YAML::Node vehicle_node = root["vehicle"];
    if (!vehicle_node) {
        throw std::runtime_error("Vehicle YAML '" + path + "' has no top-level 'vehicle:' block.");
    }

    VehicleScenarioConfig config;
    ParseVehicle(vehicle_node, config.vehicle);

    if (config.vehicle.vehicle_json.empty()) {
        throw std::runtime_error("Vehicle YAML '" + path +
                                 "' is missing 'vehicle.vehicle_json'.");
    }

    config.vehicle.kind = ReadVehicleKindFromJson(config.vehicle.vehicle_json);

    // `type:` is optional and purely a readability aid; the JSON decides.
    if (vehicle_node["type"]) {
        const std::string declared = ToUpper(vehicle_node["type"].as<std::string>());
        const std::string actual =
            config.vehicle.kind == VehicleKind::Wheeled ? "WHEELED" : "TRACKED";
        if (declared != actual) {
            throw std::runtime_error("Vehicle YAML '" + path + "' declares type: " + declared +
                                     " but '" + config.vehicle.vehicle_json + "' is a " + actual +
                                     " vehicle. Remove the key or correct it.");
        }
    }

    if (const YAML::Node terrain_node = root["terrain"]) {
        config.has_terrain = true;
        ParseTerrain(terrain_node, config.terrain);
    }

    return config;
}

}  // namespace seastack::app
