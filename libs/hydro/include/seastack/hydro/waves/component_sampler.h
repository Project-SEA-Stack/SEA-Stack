/*********************************************************************
 * @file  component_sampler.h
 * @brief Converts a SeaStateDefinition into a list of WaveComponents.
 *
 * The sampler is the bridge between declarative sea-state descriptions
 * and the component list that drives LinearDirectionalWaveField.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_COMPONENT_SAMPLER_H
#define SEASTACK_HYDRO_WAVES_COMPONENT_SAMPLER_H

#include <seastack/hydro/waves/wave_component.h>
#include <random>
#include <string>
#include <vector>

namespace seastack::hydro {

class ComponentSampler {
  public:
    /// Build a list of wave components from a declarative sea-state definition.
    ///
    /// The returned vector is the canonical representation of the sea state
    /// and can be passed directly to LinearDirectionalWaveField.
    ///
    /// Handles all sea-state types:
    ///   - Regular:               1 component
    ///   - Long-crested irregular: n_omega components (n_theta == 1)
    ///   - Directional irregular:  n_omega * n_theta components per partition
    ///   - Bimodal:               union of components from all partitions
    ///   - Eta-import:            components extracted via DFT from a time series
    [[nodiscard]] static std::vector<WaveComponent> Build(const SeaStateDefinition& def);

    /// Build wave components by DFT of an imported elevation time series.
    ///
    /// Reads a "time:eta" text file, performs a DFT at the natural Fourier
    /// frequencies f_k = k / (N * dt) within [f_min_hz, f_max_hz], and
    /// returns one WaveComponent per retained frequency bin.  The result is
    /// inherently long-crested (single propagation direction).
    ///
    /// The number of bins is determined by the signal's frequency resolution
    /// (1 / T_window) and may be less than @p nf.  When more Fourier bins
    /// exist than @p nf, the grid is subsampled with uniform stride.
    ///
    /// @param path       Path to the eta file (lines of "time:elevation").
    /// @param depth      Water depth [m] (0 = deep water).
    /// @param g          Gravity [m/s^2].
    /// @param nf         Upper bound on the number of frequency bins.
    /// @param f_min_hz   Minimum frequency [Hz].
    /// @param f_max_hz   Maximum frequency [Hz].
    /// @param direction_rad  Propagation direction [rad] (default 0 = +X).
    [[nodiscard]] static std::vector<WaveComponent> BuildFromEtaFile(
        const std::string& path,
        double depth,
        double g,
        int nf = 1000,
        double f_min_hz = 0.001,
        double f_max_hz = 1.0,
        double direction_rad = 0.0);

  private:
    /// Sample components for a single spectral partition.
    static std::vector<WaveComponent> SamplePartition(
        const SeaStatePartition& partition,
        int n_omega,
        int n_theta,
        double omega_min,
        double omega_max,
        double depth,
        double g,
        std::mt19937& rng);

    /// Remove components whose amplitude is negligibly small.
    static void PruneComponents(std::vector<WaveComponent>& components,
                                double threshold_fraction = 1e-6);
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_COMPONENT_SAMPLER_H
