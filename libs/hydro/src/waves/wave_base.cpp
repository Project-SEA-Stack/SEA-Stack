/*********************************************************************
 * @file  wave_base.cpp
 * @brief Base wave model implementations.
 *********************************************************************/

#include <seastack/hydro/waves/wave_base.h>
#include <cmath>
#include <stdexcept>

namespace seastack::hydro {

double WaveBase::GetDynamicPressureHead(const Eigen::Vector3d& /*position*/, double /*time*/,
                                        double /*elevation*/) const {
    throw std::logic_error(
        "This wave model has no linear pressure field (GetDynamicPressureHead). "
        "Nonlinear Froude-Krylov needs a wave defined by discrete components.");
}

Eigen::Vector3d WaveBase::GetVelocity(const Eigen::Vector3d& position, double time) const {
    double elevation = wave_stretching_ ? GetElevation(position, time) : 0.0;
    return GetVelocity(position, time, elevation);
}

Eigen::Vector3d WaveBase::GetAcceleration(const Eigen::Vector3d& position, double time) const {
    double elevation = wave_stretching_ ? GetElevation(position, time) : 0.0;
    return GetAcceleration(position, time, elevation);
}

std::pair<std::vector<double>, std::vector<double>>
WaveBase::ComputeElevationTimeSeries(double t_start, double t_end, double dt) const {
    int n = static_cast<int>(std::ceil((t_end - t_start) / dt)) + 1;
    std::vector<double> times(n);
    std::vector<double> elevations(n);
    const Eigen::Vector3d origin(0.0, 0.0, 0.0);
    for (int i = 0; i < n; ++i) {
        double t = t_start + i * dt;
        times[i] = t;
        elevations[i] = GetElevation(origin, t);
    }
    return {times, elevations};
}

double WaveBase::GetExcitationRampForVisualization(double /*time*/) const {
    return 1.0;
}

}  // namespace seastack::hydro
