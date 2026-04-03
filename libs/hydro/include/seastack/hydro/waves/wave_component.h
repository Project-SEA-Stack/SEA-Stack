/*********************************************************************
 * @file  wave_component.h
 * @brief Core data types for the component-based wave architecture.
 *
 * Every sea state (regular, irregular long-crested, directional,
 * bimodal) is ultimately represented as a std::vector<WaveComponent>.
 * The types here are purely declarative -- they describe *what* a
 * sea state is, not *how* to evaluate it.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_WAVE_COMPONENT_H
#define SEASTACK_HYDRO_WAVES_WAVE_COMPONENT_H

#include <optional>
#include <string>
#include <vector>

namespace seastack::hydro {

/// A single linear wave component in the superposition model.
///
/// The free-surface elevation contributed by this component is:
///     eta_i(x,y,t) = amplitude * cos(k*(x*cos(direction) + y*sin(direction)) - omega*t + phase)
struct WaveComponent {
    double omega     = 0.0;  ///< Angular frequency [rad/s]
    double k         = 0.0;  ///< Wavenumber [rad/m] from dispersion relation
    double direction = 0.0;  ///< Propagation direction [rad], 0 = +X, pi/2 = +Y
    double amplitude = 0.0;  ///< Wave amplitude [m]
    double phase     = 0.0;  ///< Random phase [rad]
};

/// Frequency spectrum definition (declarative, no computation).
struct SpectrumDefinition {
    std::string type = "jonswap";  ///< "jonswap" or "pierson_moskowitz"
    double Hs    = 0.0;            ///< Significant wave height [m]
    double Tp    = 0.0;            ///< Peak period [s]
    double gamma = 3.3;            ///< JONSWAP peak enhancement factor (ignored for PM)
};

/// Directional spreading definition (declarative, no computation).
struct SpreadingDefinition {
    std::string type = "none";        ///< "none" (long-crested) or "cos2s"
    double mean_direction_deg = 0.0;  ///< Mean propagation direction [deg], 0 = +X
    double s = 12.0;                  ///< Spreading parameter for cos2s
};

/// A single spectral partition (one wave train in a potentially multi-modal sea state).
struct SeaStatePartition {
    SpectrumDefinition spectrum;
    SpreadingDefinition spreading;
};

/// Complete declarative description of a sea state.
///
/// This struct carries all inputs needed to sample wave components but performs
/// no computation itself.  Pass it to ComponentSampler::Build() to produce
/// a std::vector<WaveComponent>.
struct SeaStateDefinition {
    std::string type = "regular";  ///< "regular" or "irregular"
    double depth = 0.0;            ///< Water depth [m], 0 or inf = deep water
    double g     = 9.81;           ///< Gravitational acceleration [m/s^2]

    // --- Regular wave parameters (used when type == "regular") ---
    double amplitude     = 0.0;  ///< Wave amplitude [m]
    double omega         = 0.0;  ///< Angular frequency [rad/s]
    double direction_deg = 0.0;  ///< Propagation direction [deg]
    double phase_rad     = 0.0;  ///< Phase [rad]

    // --- Spectral partitions (used when type == "irregular") ---
    /// One or more spectral partitions.  For a unimodal sea state, provide
    /// exactly one partition.  For bimodal (swell + wind sea), provide two.
    std::vector<SeaStatePartition> partitions;

    // --- Elevation file import (alternative to spectral generation) ---
    /// If set, wave components are extracted from the eta time series via DFT
    /// instead of being sampled from a parametric spectrum.  The imported time
    /// series is inherently long-crested; direction defaults to 0 (or the value
    /// of direction_deg for type == "regular").
    std::string eta_file_path;

    // --- Discretization ---
    int n_omega     = 256;  ///< Number of frequency bins (matches kDefaultWaveSpectralNOmega for YAML defaults)
    int n_theta     = 1;    ///< Number of directional bins (1 = long-crested)
    double omega_min = 0.0; ///< Minimum angular frequency [rad/s] (0 = auto)
    double omega_max = 0.0; ///< Maximum angular frequency [rad/s] (0 = auto)

    int seed = 42;  ///< Random seed for phase generation
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_WAVE_COMPONENT_H
