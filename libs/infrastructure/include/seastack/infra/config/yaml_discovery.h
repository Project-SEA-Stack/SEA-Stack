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

    /// Chrono FSI top-level YAML file for SPH simulations (references its own
    /// mbs.yaml + sph.yaml). When set, the case runs through the Chrono::FSI
    /// (SPH) path instead of the potential-flow path; model_file/simulation_file
    /// are then optional (the FSI YAML hierarchy carries the multibody model,
    /// solvers, and time step).
    std::string fsi_file;

    /// Dedicated SPH fluid-definition file (a `*.sph.yaml`). This is a
    /// role-based fidelity input, analogous to `hydro_file` for potential flow:
    /// it describes the SPH fluid acting on the model. Currently it carries the
    /// deck-sloshing `tank:` block (see TankConfig). Its `tank:` block is parsed
    /// into `tank` below (setting `has_tank`), exactly as an inline `tank:` block
    /// in the setup file would be.
    ///
    /// Intended workflow: a case can swap `hydro_file` <-> `sph_file` to change
    /// the fluid fidelity for the same model. When BOTH are present the case is
    /// COUPLED (potential-flow exterior + SPH interior). Path is relative to the
    /// setup file's directory.
    std::string sph_file;
    bool has_sph_file = false;

    /// Optional SPH deck-sloshing tank mounted on a potential-flow hull.
    /// Supplied either inline as a setup `tank:` block or (preferred) in a
    /// dedicated `sph_file` (see above). When both `hydro_file` and a tank are
    /// present, the case is a COUPLED seakeeping + tank-sloshing simulation: the
    /// exterior ocean uses linear potential flow (BEM) while the interior tank
    /// fluid uses Chrono::SPH, two-way coupled through the hull rigid body. All
    /// lengths are metres; the deck position is given in the hull reference frame
    /// (origin at the waterline, midship; z up). Requires an SPH-enabled build.
    struct TankConfig {
        double length = 4.0;            ///< tank interior size along ship x (m)
        double width = 4.0;             ///< tank interior size along ship y (m)
        double height = 2.0;            ///< tank wall height, open top (m)
        double fill_depth = 1.0;        ///< still-water fill depth (m)
        double deck_x = 0.0;            ///< tank floor centre x, hull-local (m)
        double deck_y = 0.0;            ///< tank floor centre y, hull-local (m)
        double deck_z = 3.5;            ///< tank floor height above waterline (m)
        double spacing = 0.10;          ///< SPH initial particle spacing (m)
        double fluid_density = 1000.0;  ///< tank fluid density (kg/m^3)
        double fluid_viscosity = 1.0e-3;///< tank fluid dynamic viscosity
        double cfd_step = 1.0e-4;       ///< SPH sub-step (s)
        double mbd_step = 1.0e-3;       ///< multibody sub-step / FSI coupling step (s)
        std::string hull_body = "body1";///< name of the hull body carrying the tank
        bool rebalance_mass = true;     ///< reduce hull mass by tank fluid mass (flotation)
        // SPH numerics (defaults tuned for water; expose so the look can be
        // tuned from the sph_file without recompiling). Keep num_bce_layers >= 3
        // or wall kernels lose support and particles stick/leak at boundaries.
        int num_bce_layers = 3;         ///< BCE wall layers (>=3 recommended)
        double artificial_viscosity = 0.05;  ///< lower = more mobile/splashy fluid
        double max_velocity = 4.0;      ///< expected max fluid speed; sets EOS sound speed
        double roll_rate = 0.0;         ///< initial hull roll rate about ship-x (rad/s); a
                                        ///< velocity-release excitation for still-water decay tests
    };
    TankConfig tank;
    bool has_tank = false;

    bool has_model_file = false;
    bool has_simulation_file = false;
    bool has_hydro_file = false;
    bool has_output_directory = false;
    bool has_fsi_file = false;

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
    /// Path as given in setup (`external_pto_file`), empty when inline `external_pto:`.
    std::string external_pto_file;
    bool has_external_pto_file = false;
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

