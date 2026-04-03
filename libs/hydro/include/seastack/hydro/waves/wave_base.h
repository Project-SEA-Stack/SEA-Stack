/*********************************************************************
 * @file  wave_base.h
 * @brief Base classes shared by all wave models (kinematics only).
 *
 * WaveBase provides the kinematics interface: elevation, velocity,
 * acceleration. Excitation forces are computed by ExcitationComponent or
 * ExcitationIrfComponent, not on the wave object.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_WAVE_BASE_H
#define SEASTACK_HYDRO_WAVES_WAVE_BASE_H

#include <seastack/core/types.h>
#include <seastack/hydro/waves/wave_component.h>

#include <Eigen/Core>
#include <utility>
#include <vector>

namespace seastack::hydro {

enum class WaveMode {
    kNoWave   = 0,
    kRegular  = 1,
    kIrregular = 2
};

/**
 * @brief Abstract base class for all wave models (kinematics only).
 *
 * Coordinate conventions:
 *   - LinearDirectionalWaveField supports arbitrary propagation directions.
 *   - Z is vertical (positive upward), with z = mwl_ at mean water level.
 *   - Direction convention: 0 = +X, pi/2 = +Y (counter-clockwise from +X).
 *
 * Units:
 *   - Positions: meters [m]
 *   - Time: seconds [s]
 *   - Elevation η: meters [m]
 *   - Gradients ∂η/∂x, ∂η/∂y: dimensionless [m/m]
 *   - Velocities: meters per second [m/s]
 *   - Accelerations: meters per second squared [m/s²]
 */
class WaveBase {
  public:
    virtual ~WaveBase() = default;

    WaveBase(const WaveBase&)            = delete;
    WaveBase& operator=(const WaveBase&) = delete;

    virtual void Initialize()                                                                                     = 0;
    virtual WaveMode GetWaveMode() const                                                                          = 0;
    virtual double GetElevation(const Eigen::Vector3d& position, double time) const                               = 0;
    virtual Eigen::Vector3d GetVelocity(const Eigen::Vector3d& position, double time, double elevation) const     = 0;
    virtual Eigen::Vector3d GetAcceleration(const Eigen::Vector3d& position, double time, double elevation) const = 0;

    /// Surface slope (∂η/∂x, ∂η/∂y) at a given position and time.
    /// Default returns (0,0); subclasses with spatial variation override this.
    virtual Eigen::Vector2d GetElevationGradientXY(const Eigen::Vector3d& /*position*/, double /*time*/) const {
        return Eigen::Vector2d(0.0, 0.0);
    }

    Eigen::Vector3d GetVelocity(const Eigen::Vector3d& position, double time) const;
    Eigen::Vector3d GetAcceleration(const Eigen::Vector3d& position, double time) const;

    /// Elevation using at most max_components (for faster visualization).
    /// Default ignores the limit and calls GetElevation(pos, t).
    virtual double GetElevation(const Eigen::Vector3d& pos, double t, int /*max_components*/) const {
        return GetElevation(pos, t);
    }

    /// Dominant-component period [s]; 0 if unknown or no wave.
    virtual double GetCharacteristicPeriod() const { return 0.0; }

    /// Compute elevation at the origin over a uniform time grid.
    std::pair<std::vector<double>, std::vector<double>>
    ComputeElevationTimeSeries(double t_start, double t_end, double dt) const;

    void SetNumBodies(unsigned int n) { num_bodies_ = n; }
    unsigned int GetNumBodies() const { return num_bodies_; }

    /// Set excitation ramp duration [s]. 0 = no ramp.
    virtual void SetRampDuration(double /*seconds*/) {}

    /// Get the excitation ramp duration [s]. 0 = no ramp.
    virtual double GetRampDuration() const { return 0.0; }

    /// Time-dependent factor applied to incident free-surface elevation in
    /// visualization so the GUI matches the excitation envelope. Does not
    /// change kinematic GetElevation() (used by IRF and other physics).
    /// Default 1.0 (no scaling).
    virtual double GetExcitationRampForVisualization(double time) const;

    /// Set excitation IRF truncation time [s]. 0 = full IRF.
    virtual void SetExcitationTruncationTime(double /*seconds*/) {}

    double GetMWL() const { return mwl_; }
    double GetGravity() const { return g_; }
    double GetWaterDepth() const { return water_depth_; }
    bool GetWaveStretching() const { return wave_stretching_; }

    /// Discrete wave component list, or nullptr if this wave model does not
    /// decompose into a set of linear components (e.g. table-based waves).
    /// Used by the builder and excitation components to construct
    /// frequency-domain force computation without knowing the concrete type.
    virtual const std::vector<WaveComponent>* GetWaveComponents() const {
        return nullptr;
    }

    /// Update environment parameters (gravity, water depth) from H5 or
    /// simulation metadata. The base implementation stores the values;
    /// subclasses with cached derived quantities (e.g. wavenumbers) must
    /// override and recompute after calling the base version.
    virtual void UpdateEnvironment(double gravity, double depth) {
        g_ = gravity;
        water_depth_ = depth;
    }

  protected:
    WaveBase() = default;

    double mwl_           = 0.0;
    double g_             = 9.81;
    double water_depth_   = 0.0;
    bool wave_stretching_ = true;
    unsigned int num_bodies_ = 0;
};

class NoWave : public WaveBase {
  public:
    NoWave() = default;

    void Initialize() override {}
    WaveMode GetWaveMode() const override { return mode_; }
    double GetElevation(const Eigen::Vector3d&, double) const override { return 0.0; }
    Eigen::Vector3d GetVelocity(const Eigen::Vector3d&, double, double) const override { return Eigen::Vector3d(0.0, 0.0, 0.0); }
    Eigen::Vector3d GetAcceleration(const Eigen::Vector3d&, double, double) const override { return Eigen::Vector3d(0.0, 0.0, 0.0); }

  private:
    static constexpr WaveMode mode_ = WaveMode::kNoWave;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_WAVE_BASE_H
