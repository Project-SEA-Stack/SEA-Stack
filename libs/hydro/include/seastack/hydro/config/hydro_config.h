/*********************************************************************
 * @file  hydro_config.h
 * @brief Public configuration types for parsed YAML hydro settings.
 *
 * MAIN TYPES:
 *   - HydroBody: Per-body hydrodynamic settings (H5 file, flags)
 *   - WaveSettings: Wave type, height, period, spectrum
 *   - YAMLHydroData: Top-level container for hydro.yaml data
 *********************************************************************/

#ifndef SEASTACK_HYDRO_CONFIG_H
#define SEASTACK_HYDRO_CONFIG_H

#include <seastack/hydro/radiation_types.h>
#include <string>
#include <vector>
#include <array>

namespace seastack::hydro {

/**
 * @brief Per-body hydrostatics model selection.
 *
 * Extensible: future models (analytical, wave-aware) add new enum values
 * and corresponding branches in HydroModelBuilder::Build().
 */
enum class HydrostaticsModel { kLinear, kNonlinear };

/**
 * @brief Per-body hydrostatics configuration.
 */
struct HydrostaticsSettings {
    HydrostaticsModel model = HydrostaticsModel::kLinear;
    std::string mesh_file;  ///< OBJ path, required when model == kNonlinear
};

/**
 * @brief Configuration for a hydrodynamic body.
 */
struct HydroBody {
    std::string name = "";
    std::string h5_file = "";
    std::string mesh_file = "";  ///< DEPRECATED: use hydrostatics.mesh_file instead
    HydrostaticsSettings hydrostatics;
    bool include_excitation = true;
    bool include_radiation = true;
    std::array<double, 6> linear_damping = {0, 0, 0, 0, 0, 0};     ///< Per-DOF linear [surge, sway, heave, roll, pitch, yaw]
    std::array<double, 6> quadratic_damping = {0, 0, 0, 0, 0, 0}; ///< Per-DOF quadratic (v*|v|) [surge, sway, heave, roll, pitch, yaw]
};

/**
 * @brief Configuration for directional spreading.
 */
struct WaveSpreadingSettings {
    std::string type = "none";        ///< "none" (long-crested) or "cos2s"
    double s = 12.0;                  ///< Spreading parameter for cos2s
};

/**
 * @brief Configuration for a spectral partition in a multi-modal sea state.
 */
struct WavePartitionSettings {
    std::string spectrum_type = "jonswap";
    double Hs    = 0.0;
    double Tp    = 0.0;
    double gamma = 3.3;
    double mean_direction_deg = 0.0;
    WaveSpreadingSettings spreading;
};

/**
 * @brief Discretization settings for the component sampler.
 */
struct WaveDiscretizationSettings {
    int n_omega = 0;  ///< Number of frequency bins (0 = use kDefaultWaveSpectralNOmega)
    int n_theta = 0;  ///< Number of directional bins (0 or 1 = long-crested)
};

/// Default `n_omega` for spectral irregular seas when YAML omits `waves.discretization.n_omega`
/// and `waves.nfrequencies` (see BuildSeaStateDefinition in the Chrono adapter).
inline constexpr int kDefaultWaveSpectralNOmega = 256;

/**
 * @brief Configuration for wave settings.
 */
struct WaveSettings {
    std::string type = "no_wave";  // "regular", "irregular", "no_wave"
    double height = 0.0;
    double period = 0.0;
    /// Wave propagation direction [deg], measured counter-clockwise from the positive x-axis in the horizontal plane (math convention, not compass).
    double direction = 0.0;
    double phase = 0.0;
    std::string spectrum = "pierson_moskowitz";  // "pierson_moskowitz", "jonswap", etc.
    int seed = -1; // optional irregular seed; -1 means unset
    // Sweep support (expanded values) for period; if empty, use 'period'
    std::vector<double> period_values;
    // Elevation import (if non-empty, overrides spectral generation)
    std::string eta_file;
    /// When `eta_file` is set: how to build the wave model. Empty or legacy
    /// tokens → DFT + LinearDirectionalWaveField; `irf_convolution` (or `irf`)
    /// → EtaTableWaveField (IRF excitation uses tabulated η). Ignored if no eta_file.
    std::string method;
    // Excitation ramp shape: "linear" (default) or "cosine" (WEC-Sim convention)
    std::string ramp_type    = "linear";
    double ramp_duration     = 0.0;   // Ramp duration [s]; 0 = no ramp (overrides function param if > 0)
    double frequency_min  = 0.0;   // [Hz]; 0 = auto (ComponentSampler derives from Tp)
    double frequency_max  = 0.0;   // [Hz]; 0 = auto
    int    nfrequencies   = 0;     // 0 = use default (kDefaultWaveSpectralNOmega for spectral, 1000 for eta import)

    // --- Directional wave extensions ---
    WaveSpreadingSettings spreading;           ///< Directional spreading (if type != "none")
    WaveDiscretizationSettings discretization; ///< Component sampling resolution
    std::vector<WavePartitionSettings> partitions; ///< For bimodal/multi-modal sea states
    double gamma = 3.3;  ///< JONSWAP peak enhancement factor (single-partition shorthand)
    double depth = 0.0;  ///< Water depth [m]; 0 = deep water (overridden by H5 if not set)
};

/**
 * @brief Top-level container for hydrodynamic configuration data from YAML.
 */
struct YAMLHydroData {
    std::vector<HydroBody> bodies;
    WaveSettings waves;
    
    // ─────────────────────────────────────────────────────────────────────────
    // Nonlinear hydrostatics (DEPRECATED system-level enable)
    // Prefer per-body hydrostatics: { model: nonlinear } in each HydroBody.
    // Kept for backward compatibility; ApplyLegacyHydrostaticsCompat() maps
    // this to per-body HydrostaticsSettings before downstream code sees it.
    // ─────────────────────────────────────────────────────────────────────────
    bool nonlinear_hydrostatics = false;
    
    // ─────────────────────────────────────────────────────────────────────────
    // Excitation method (optional; empty = kAuto in HydroSystem)
    // "auto" | "irf_convolution" | "frequency_domain"
    // ─────────────────────────────────────────────────────────────────────────
    std::string excitation_method = "";

    // ─────────────────────────────────────────────────────────────────────────
    // Convolution truncation (system-wide, wave-type-independent)
    // ─────────────────────────────────────────────────────────────────────────
    double excitation_truncation_time = 0.0;  ///< Excitation IRF truncation [s]; 0 = full
    double radiation_truncation_time = 0.0;   ///< Radiation RIRF truncation [s]; 0 = full
    
    // ─────────────────────────────────────────────────────────────────────────
    // Radiation method selection (system-wide)
    // ─────────────────────────────────────────────────────────────────────────
    std::string radiation_method = "rirf_convolution";  ///< "rirf_convolution" or "state_space"
    
    // ─────────────────────────────────────────────────────────────────────────
    // Radiation kernel processing (smoothing + tapering, composable)
    // ─────────────────────────────────────────────────────────────────────────
    RadiationKernelProcessing radiation_kernel_processing;
    
    // ─────────────────────────────────────────────────────────────────────────
    // State-space options (only used if radiation_method == "state_space")
    // ─────────────────────────────────────────────────────────────────────────
    int ss_max_order = 10;
    double ss_r2_threshold = 0.95;
    int ss_max_hankel_size = 200;
    int ss_r2_num_samples = 50;
    
    // ─────────────────────────────────────────────────────────────────────────
    // Diagnostics
    // ─────────────────────────────────────────────────────────────────────────
    bool output_kernel_fit = false;

    // ─────────────────────────────────────────────────────────────────────────
    // MoorDyn mooring coupling (optional)
    // ─────────────────────────────────────────────────────────────────────────
    bool moordyn_enabled = false;
    std::string moordyn_input_file;
    std::vector<std::string> moordyn_body_names;  // e.g. ["body1"]
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_CONFIG_H
