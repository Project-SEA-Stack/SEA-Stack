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
#include <limits>
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
    double GetDynamicPressureHead(const Eigen::Vector3d& position, double time, double elevation) const override;

    /// Batched elevation and pressure head (see WaveBase). Each component's
    /// cosine is evaluated once per point and shared by both outputs; points
    /// are processed in parallel with OpenMP when the workload is large.
    /// Results do not depend on the thread count.
    void GetElevationAndPressureHead(const Eigen::Matrix3Xd& points,
                                     double time,
                                     double elevation_scale,
                                     Eigen::VectorXd& elevation,
                                     Eigen::VectorXd* pressure_head) const override;

    /// Surface slope (d_eta/dx, d_eta/dy) at a given position and time.
    Eigen::Vector2d GetElevationGradientXY(const Eigen::Vector3d& position, double time) const override;

    /// Elevation using only the max_components largest-amplitude components (for faster visualization).
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
    /// After a live period or direction change it holds every layer.
    [[nodiscard]] const std::vector<WaveComponent>& GetComponents() const { return components_; }

    /// Components of the newest layer: the sea in effect once any crossfade
    /// has finished (all components when there are no layers).
    [[nodiscard]] std::vector<WaveComponent> GetCurrentComponents() const;

    /// Live period or direction change: crossfade from the current components
    /// to `components` over [start_time, start_time + duration] (see WaveLayer).
    ///
    /// Wavenumbers are recomputed for this field's depth and gravity. When
    /// `components` has as many entries as the current set (same seed and
    /// discretisation), they are matched one to one and each phase is shifted
    /// so the new component has the phase of the current one at the origin at
    /// start_time:
    ///   phi_new = phi_old + (w_new - w_old) * start_time   (mod 2 pi)
    /// so a direction change leaves eta at the origin unchanged, and after a
    /// period change both sets start in step. With a different count (a
    /// period change in a fixed frequency band can change how many components
    /// survive pruning) the new components keep their own phases; eta stays
    /// continuous because the new set starts at weight 0.
    ///
    /// Evaluation cost grows with the number of changes: faded-out sets stay
    /// in the list, because IRF excitation convolves the elevation history.
    /// @return Time [s] at which the new set has full weight.
    /// @throws std::invalid_argument on an empty set or bad times;
    ///         std::logic_error if a crossfade is still running at start_time.
    double CrossfadeTo(std::vector<WaveComponent> components, double start_time, double duration);

    /// True while a crossfade started by CrossfadeTo is running at `time`.
    [[nodiscard]] bool IsCrossfading(double time) const { return time < crossfade_end_; }

    // ── Overrides from WaveBase ──────────────────────────────────────

    const std::vector<WaveComponent>* GetWaveComponents() const override {
        return &components_;
    }

    void UpdateEnvironment(double gravity, double depth) override;

    double GetElevation(const Eigen::Vector3d& pos, double t, int max_components) const override;
    double GetCharacteristicPeriod() const override;
    void SetRampDuration(double seconds) override { ramp_duration_ = seconds; }
    double GetRampDuration() const override { return ramp_duration_; }

    /// Cosine envelope matching ExcitationComponent (frequency-domain excitation ramp),
    /// times the amplitude envelope e(t).
    double GetExcitationRampForVisualization(double time) const override;

    // ── Spectral accessors (for diagnostics / HDF5 export) ──────────

    [[nodiscard]] std::vector<double> GetFrequenciesHz() const;
    [[nodiscard]] std::vector<double> GetSpectralDensityEstimate() const;

  private:
    WaveMode mode_ = WaveMode::kIrregular;
    double ramp_duration_ = 0.0;

    std::vector<WaveComponent> components_;
    double crossfade_end_ = -std::numeric_limits<double>::infinity();  ///< End of the last crossfade [s]

    // Pre-computed SIMD-friendly arrays (one element per component).
    Eigen::VectorXd amplitudes_;
    Eigen::VectorXd kx_;
    Eigen::VectorXd ky_;
    Eigen::VectorXd omegas_;
    Eigen::VectorXd phases_;
    Eigen::VectorXd cos_dirs_;
    Eigen::VectorXd sin_dirs_;
    Eigen::VectorXd wavenumbers_;

    // Same components sorted by decreasing amplitude within each layer, so a
    // visualization limit keeps the most energetic components of each set.
    Eigen::VectorXd viz_amplitudes_;
    Eigen::VectorXd viz_kx_;
    Eigen::VectorXd viz_ky_;
    Eigen::VectorXd viz_omegas_;
    Eigen::VectorXd viz_phases_;

    void PrecomputeArrays();

    /// Component amplitudes times the layer weights at `time` [m]. Returns
    /// amplitudes_ itself when there are no layers (bit-identical path);
    /// otherwise fills and returns `scratch`.
    const Eigen::VectorXd& AmplitudesAt(double time, Eigen::VectorXd& scratch) const;

    /// Cosine excitation ramp: 0.5*(1 - cos(pi*t/T_ramp)); 1 if no ramp or t >= T_ramp.
    double ExcitationRampFactor(double time) const;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_LINEAR_DIRECTIONAL_WAVE_FIELD_H
