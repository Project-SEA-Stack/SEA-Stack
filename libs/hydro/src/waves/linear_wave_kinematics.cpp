/*********************************************************************
 * @file  linear_wave_kinematics.cpp
 * @brief Per-component Airy wave kinematics with directional support.
 *********************************************************************/

#include <seastack/hydro/waves/linear_wave_kinematics.h>
#include <seastack/core/math_constants.h>
#include "wave_constants.h"

#include <cmath>

using seastack::hydro::wave_detail::kDeepWaterKhThreshold;
using seastack::hydro::wave_detail::kEffectiveInfiniteDepth;
using seastack::hydro::wave_detail::kShallowWaterKhThreshold;

namespace {

struct DepthAttenuation {
    double h_cosh;
    double h_sinh;
};

/// Compute cosh/sinh depth attenuation factors for particle kinematics.
DepthAttenuation ComputeDepthAttenuation(double k_mag, double z,
                                          double depth, bool deep_water) {
    if (deep_water) {
        const double ekz = std::exp(k_mag * z);
        return {ekz, ekz};
    }
    const double kh = k_mag * depth;
    if (kh < kShallowWaterKhThreshold) {
        return {1.0 / kh, (z + depth) / depth};
    }
    const double denom = std::sinh(kh);
    return {std::cosh(k_mag * (z + depth)) / denom,
            std::sinh(k_mag * (z + depth)) / denom};
}

}  // namespace

namespace seastack::hydro {

bool LinearWaveKinematics::IsDeepWater(double wavenumber, double depth) {
    if (depth <= 0.0 || depth > kEffectiveInfiniteDepth || std::isinf(depth)) {
        return true;
    }
    return (std::abs(wavenumber) * depth > kDeepWaterKhThreshold);
}

double LinearWaveKinematics::PhaseArg(const WaveComponent& c, double x, double y, double t) {
    const double kx = c.k * std::cos(c.direction);
    const double ky = c.k * std::sin(c.direction);
    return kx * x + ky * y - c.omega * t + c.phase;
}

double LinearWaveKinematics::Elevation(const WaveComponent& c, double x, double y, double t) {
    return c.amplitude * std::cos(PhaseArg(c, x, y, t));
}

Eigen::Vector2d LinearWaveKinematics::ElevationGradient(const WaveComponent& c,
                                                         double x, double y, double t) {
    const double kx = c.k * std::cos(c.direction);
    const double ky = c.k * std::sin(c.direction);
    const double sin_phase = std::sin(PhaseArg(c, x, y, t));
    return Eigen::Vector2d(-c.amplitude * kx * sin_phase,
                           -c.amplitude * ky * sin_phase);
}

Eigen::Vector3d LinearWaveKinematics::Velocity(const WaveComponent& c,
                                                double x, double y, double z,
                                                double t, double depth) {
    const double k_mag = std::abs(c.k);
    if (k_mag == 0.0 || c.omega == 0.0 || c.amplitude == 0.0) {
        return Eigen::Vector3d::Zero();
    }

    const double cos_theta = std::cos(c.direction);
    const double sin_theta = std::sin(c.direction);
    const double phase = PhaseArg(c, x, y, t);
    const double cos_phase = std::cos(phase);
    const double sin_phase = std::sin(phase);

    const auto [H_cosh, H_sinh] = ComputeDepthAttenuation(
        k_mag, z, depth, IsDeepWater(c.k, depth));

    const double u_horiz = c.omega * c.amplitude * H_cosh * cos_phase;
    const double u_vert  = c.omega * c.amplitude * H_sinh * sin_phase;

    return Eigen::Vector3d(u_horiz * cos_theta,
                           u_horiz * sin_theta,
                           u_vert);
}

Eigen::Vector3d LinearWaveKinematics::Acceleration(const WaveComponent& c,
                                                    double x, double y, double z,
                                                    double t, double depth) {
    const double k_mag = std::abs(c.k);
    if (k_mag == 0.0 || c.omega == 0.0 || c.amplitude == 0.0) {
        return Eigen::Vector3d::Zero();
    }

    const double cos_theta = std::cos(c.direction);
    const double sin_theta = std::sin(c.direction);
    const double phase = PhaseArg(c, x, y, t);
    const double sin_phase = std::sin(phase);
    const double cos_phase = std::cos(phase);

    const auto [H_cosh, H_sinh] = ComputeDepthAttenuation(
        k_mag, z, depth, IsDeepWater(c.k, depth));

    const double w2 = c.omega * c.omega;
    const double a_horiz =  w2 * c.amplitude * H_cosh * sin_phase;
    const double a_vert  = -w2 * c.amplitude * H_sinh * cos_phase;

    return Eigen::Vector3d(a_horiz * cos_theta,
                           a_horiz * sin_theta,
                           a_vert);
}

}  // namespace seastack::hydro
