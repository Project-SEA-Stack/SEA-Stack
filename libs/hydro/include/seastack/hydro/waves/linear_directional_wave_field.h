/*********************************************************************
 * @file  linear_directional_wave_field.h
 * @brief Unified wave field: evaluates an arbitrary set of linear wave
 *        components via the standard WaveBase interface.
 *
 * This single class handles regular, long-crested irregular, directional
 * irregular, and bimodal sea states -- all as special cases of the same
 * component list.
 *
 * Excitation forces from BEM transfer functions are computed by
 * ExcitationComponent using InterpolateExcitationTransfer; this class
 * provides kinematics and environment (gravity, depth) only.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_LINEAR_DIRECTIONAL_WAVE_FIELD_H
#define SEASTACK_HYDRO_WAVES_LINEAR_DIRECTIONAL_WAVE_FIELD_H

#include <seastack/hydro/waves/wave_base.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/hydro_data.h>

#include <Eigen/Core>
#include <vector>

namespace seastack::hydro {

class LinearDirectionalWaveField : public WaveBase {
  public:
    /// Construct from a sampled component list.
    /// @param components  Sampled wave components (from ComponentSampler::Build)
    /// @param depth       Water depth [m] (0 or inf = deep water)
    explicit LinearDirectionalWaveField(std::vector<WaveComponent> components, double depth);

    void Initialize() override;
    WaveMode GetWaveMode() const override { return mode_; }

    double GetElevation(const Eigen::Vector3d& position, double time) const override;
    Eigen::Vector3d GetVelocity(const Eigen::Vector3d& position, double time, double elevation) const override;
    Eigen::Vector3d GetAcceleration(const Eigen::Vector3d& position, double time, double elevation) const override;

    /// Surface slope (d_eta/dx, d_eta/dy) at a given position and time.
    Eigen::Vector2d GetElevationGradientXY(const Eigen::Vector3d& position, double time) const override;

    /// Elevation using only the first max_components components (for faster visualization).
    double GetElevationForVisualization(const Eigen::Vector3d& position, double time, int max_components) const;

    /// Apply gravity and optional water depth from BEM simulation metadata, then
    /// recompute component wavenumbers if the environment changed.
    ///
    /// @param sim_data  Simulation block from HydroData (uses g; optionally water_depth).
    /// @param use_file_water_depth  If true, set water depth from sim_data; if false, keep
    ///                              the depth already stored on this field (e.g. YAML depth).
    void ApplySimulationEnvironment(const HydroData::SimulationParameters& sim_data,
                                    bool use_file_water_depth);

    /// Access the raw component list (for logging, plotting, diagnostics).
    [[nodiscard]] const std::vector<WaveComponent>& GetComponents() const { return components_; }

    // ── Overrides from WaveBase ──────────────────────────────────────

    const std::vector<WaveComponent>* GetWaveComponents() const override {
        return &components_;
    }

    void UpdateEnvironment(double gravity, double depth) override;

    double GetElevation(const Eigen::Vector3d& pos, double t, int max_components) const override;
    double GetCharacteristicPeriod() const override;
    void SetRampDuration(double seconds) override { ramp_duration_ = seconds; }
    double GetRampDuration() const override { return ramp_duration_; }

    /// Cosine envelope matching ExcitationComponent (frequency-domain excitation ramp).
    double GetExcitationRampForVisualization(double time) const override;

    // ── Spectral accessors (for diagnostics / HDF5 export) ──────────

    [[nodiscard]] std::vector<double> GetFrequenciesHz() const;
    [[nodiscard]] std::vector<double> GetSpectralDensityEstimate() const;

  private:
    WaveMode mode_ = WaveMode::kIrregular;
    double ramp_duration_ = 0.0;

    std::vector<WaveComponent> components_;

    // Pre-computed SIMD-friendly arrays (one element per component).
    Eigen::VectorXd amplitudes_;
    Eigen::VectorXd kx_;
    Eigen::VectorXd ky_;
    Eigen::VectorXd omegas_;
    Eigen::VectorXd phases_;
    Eigen::VectorXd cos_dirs_;
    Eigen::VectorXd sin_dirs_;
    Eigen::VectorXd wavenumbers_;

    void PrecomputeArrays();

    /// Cosine excitation ramp: 0.5*(1 - cos(pi*t/T_ramp)); 1 if no ramp or t >= T_ramp.
    double ExcitationRampFactor(double time) const;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_LINEAR_DIRECTIONAL_WAVE_FIELD_H
