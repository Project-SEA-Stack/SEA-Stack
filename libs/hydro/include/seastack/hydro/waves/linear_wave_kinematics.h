/*********************************************************************
 * @file  linear_wave_kinematics.h
 * @brief Per-component Airy wave kinematics with directional support.
 *
 * All methods are static and stateless. They evaluate one WaveComponent
 * at a given space-time point. The wave field sums results across all
 * components.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_LINEAR_WAVE_KINEMATICS_H
#define SEASTACK_HYDRO_WAVES_LINEAR_WAVE_KINEMATICS_H

#include <seastack/hydro/waves/wave_component.h>
#include <Eigen/Core>

namespace seastack::hydro {

class LinearWaveKinematics {
  public:
    /// Free-surface elevation for a single component.
    ///   eta = A * cos(kx*x + ky*y - omega*t + phi)
    [[nodiscard]] static double Elevation(const WaveComponent& c, double x, double y, double t);

    /// Surface slope (d_eta/dx, d_eta/dy) for a single component.
    [[nodiscard]] static Eigen::Vector2d ElevationGradient(const WaveComponent& c, double x, double y, double t);

    /// Particle velocity for a single component (Airy theory, finite or deep water).
    /// Position (x, y, z) is in the physical frame; z = 0 at mean water level.
    [[nodiscard]] static Eigen::Vector3d Velocity(const WaveComponent& c,
                                                   double x, double y, double z,
                                                   double t, double depth);

    /// Particle acceleration for a single component (Airy theory).
    [[nodiscard]] static Eigen::Vector3d Acceleration(const WaveComponent& c,
                                                       double x, double y, double z,
                                                       double t, double depth);

  private:
    /// Compute the phase argument: k*(x*cos(theta) + y*sin(theta)) - omega*t + phi
    static double PhaseArg(const WaveComponent& c, double x, double y, double t);

    /// True if the component is effectively in deep water at the given depth.
    static bool IsDeepWater(double wavenumber, double depth);
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_LINEAR_WAVE_KINEMATICS_H
