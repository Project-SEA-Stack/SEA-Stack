/*********************************************************************
 * @file  vehicle_config.h
 * @brief Declarative Chrono::Vehicle + terrain scenario, parsed from YAML.
 *
 * Declarative vehicle + terrain parameters so a vehicle case can be described
 * in YAML instead of code.  Nothing here depends on Chrono; the
 * VehicleSubsystem (vehicle_subsystem.h) turns this config into live objects.
 *
 * WHY NOT chrono_parsers/yaml/ChParserVehicleYAML?
 *   Chrono ships a vehicle YAML parser that already covers the JSON specs,
 *   visualization types, spawn pose, chase camera and a RigidTerrain from a
 *   terrain JSON — roughly 60% of this schema.  We parse our own because:
 *     - it derives from ChParserMbsYAML and owns whole-simulation settings
 *       (solver, timestep, run duration) that the SEA-Stack runner already
 *       provides, and which already come from the hull's model YAML;
 *     - it has no drivers, so the repeatable path-follower runs (gains,
 *       start_time, stop_at_x, headless brake lock) would still need a schema;
 *     - it has no terrain-relative spawn height, chassis naming for MoorDyn
 *       coupling, or per-patch connected_mesh control.
 *   Adopting Chrono's parser for the overlap and keeping a thin SEA-Stack
 *   layer for the rest is a reasonable future move; it is a rewrite of working
 *   code, not a prerequisite.
 *
 * Frame / unit conventions (must stay explicit):
 *   - z up, undisturbed free surface at z = 0, lengths in metres.
 *   - initial_yaw is a rotation about +z in radians.
 *   - initial_position[2] may be a number (world z) or the string "terrain",
 *     meaning "query the terrain height under (x, y) and add ride_height".
 *   - JSON spec paths (vehicle/engine/transmission/tire) are relative to
 *     Chrono's vehicle data root, exactly like vehicle::GetVehicleDataFile().
 *   - Heightmap paths are relative to the same vehicle data root, or absolute.
 *********************************************************************/

#ifndef SEASTACK_APP_VEHICLE_CONFIG_H
#define SEASTACK_APP_VEHICLE_CONFIG_H

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace seastack::app {

/// Which Chrono::Vehicle template family the case uses.  Not a YAML input:
/// it is read from the vehicle JSON's "Template" field (see
/// LoadVehicleConfigFromYaml), so there is one source of truth.
enum class VehicleKind { Wheeled, Tracked };

/// Path-follower (repeatable, headless-friendly) vs interactive WASD driver.
enum class DriverKind { PathFollower, Interactive };

/// Visualization type strings as they appear in YAML (MESH | PRIMITIVES |
/// NONE | COLLISION).  Kept as strings here and mapped to Chrono's
/// VisualizationType in the subsystem, so this header stays Chrono-free.
struct VehicleVisualizationConfig {
    std::string chassis    = "MESH";
    std::string sprocket   = "MESH";        // tracked only
    std::string track_shoe = "MESH";        // tracked only
    std::string suspension = "PRIMITIVES";
    std::string wheel      = "MESH";
    std::string tire       = "MESH";        // wheeled only
};

/// PID gains for the speed controller (and, with lookahead, steering).
struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

struct SteeringGains {
    double lookahead = 5.0;  ///< m, path-follower look-ahead distance
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
};

/// Per-key increments for the interactive (WASD) driver.
struct InteractiveDeltas {
    double steering = 0.02;
    double throttle = 0.02;
    double braking  = 0.06;
};

struct DriverConfig {
    DriverKind kind = DriverKind::PathFollower;

    /// Path waypoints as (x, y); z is taken from the spawn height.  A straight
    /// survey line is just two points.
    std::vector<std::pair<double, double>> path_xy;
    double target_speed = 1.0;  ///< m/s

    SteeringGains steering;
    PidGains speed{2.0, 0.1, 0.0};

    /// Hold on the brakes until this simulated time (s); lets waves/mooring
    /// settle before the drive-off.
    double start_time = 0.0;

    /// Optional: once the chassis passes this world x, park on the brakes so a
    /// finite path does not make the tracker turn back on itself.
    bool has_stop_at_x = false;
    double stop_at_x = 0.0;

    InteractiveDeltas interactive_deltas;
};

/// Third-person chase camera (GUI only).
struct ChaseCameraConfig {
    std::array<double, 3> point{0.0, 0.0, 0.0};  ///< chase point on the chassis, vehicle frame
    double distance = 14.0;
    double height   = 2.0;
};

struct VehicleConfig {
    /// Filled from vehicle_json, not from YAML.
    VehicleKind kind = VehicleKind::Tracked;

    std::string vehicle_json;
    std::string engine_json;
    std::string transmission_json;
    std::string tire_json;  ///< wheeled only

    std::array<double, 2> initial_xy{0.0, 0.0};
    /// When true, spawn z = terrain height under (x, y) + ride_height.  When
    /// false, initial_z is the world z of the chassis reference frame.
    bool spawn_on_terrain = true;
    double initial_z   = 0.0;
    double initial_yaw = 0.0;  ///< rad, about +z
    double ride_height = 0.6;  ///< m, chassis reference above the terrain surface

    VehicleVisualizationConfig visualization;
    DriverConfig driver;
    ChaseCameraConfig chase_camera;

    /// Name assigned to the chassis body so MoorDyn coupling can find it.
    /// Left as a fixed default; not currently exposed in YAML.
    std::string chassis_name = "vehicle_chassis";
};

/// SMC contact material for a rigid terrain patch (defaults are silt/sand
/// seabed values).
struct TerrainMaterialConfig {
    double friction    = 0.8;
    double restitution = 0.01;
    double young_modulus = 1.0e7;
    double poisson_ratio = 0.3;
    double normal_stiffness     = 2.0e5;
    double normal_damping       = 40.0;
    double tangential_stiffness = 2.0e5;
    double tangential_damping   = 20.0;
};

struct TerrainPatchConfig {
    std::string heightmap;  ///< relative to Chrono vehicle data, or absolute
    std::array<double, 3> center{0.0, 0.0, 0.0};
    std::array<double, 2> size{100.0, 64.0};        ///< x, y extent [m]
    std::array<double, 2> height_range{0.0, 1.5};   ///< hMin, hMax [m]
    /// When false, the height map collapses to a single BVH triangle-mesh
    /// shape (large performance win for track shoes).
    bool connected_mesh = false;
    double sweep_sphere_radius = 0.01;
    TerrainMaterialConfig material;

    bool has_texture = false;
    std::string texture;  ///< relative to Chrono vehicle data, or absolute
    std::array<double, 3> color{0.42, 0.38, 0.30};
};

/// Terrain is always a Chrono::Vehicle RigidTerrain built from these patches;
/// deformable (SCM) terrain is not exposed yet.
struct TerrainConfig {
    std::vector<TerrainPatchConfig> patches;
};

/// The whole vehicle scenario: one vehicle and an optional terrain block.
struct VehicleScenarioConfig {
    VehicleConfig vehicle;
    bool has_terrain = false;
    TerrainConfig terrain;
};

/// Resolve a data file that is either an absolute path or relative to Chrono's
/// vehicle data root (vehicle::GetVehicleDataFile).
std::string ResolveVehicleDataFile(const std::string& path);

/// Parse a `<case>.vehicle.yaml` file.  Throws std::runtime_error on a missing
/// file or a malformed `vehicle:` block (an empty/only-terrain file is a hard
/// error because a vehicle scenario without a vehicle is meaningless), and on a
/// `type:` key that contradicts the vehicle JSON.
///
/// Requires Chrono's vehicle data path to be set (SetVehicleDataPath), because
/// the vehicle JSON is read here to determine wheeled vs tracked.
VehicleScenarioConfig LoadVehicleConfigFromYaml(const std::string& path);

}  // namespace seastack::app

#endif  // SEASTACK_APP_VEHICLE_CONFIG_H
