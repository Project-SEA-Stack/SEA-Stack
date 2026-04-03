/*********************************************************************
 * @file  linear_directional_wave_field.cpp
 * @brief Implementation of LinearDirectionalWaveField.
 *********************************************************************/

#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/linear_wave_kinematics.h>
#include <seastack/core/math_constants.h>
#include "wave_utilities.h"
#include "wave_constants.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

using seastack::hydro::wave_detail::kShallowWaterKhThreshold;

/// Compute the depth attenuation factors for horizontal and vertical
/// particle motion at elevation z in water of given depth.
///
/// Returns {H_cosh, H_sinh} where:
///   horizontal factor = omega * A * H_cosh * cos(phase)
///   vertical   factor = omega * A * H_sinh * sin(phase)
struct DepthAttenuation {
    double h_cosh;
    double h_sinh;
};

DepthAttenuation ComputeDepthAttenuation(double k_mag, double z, double water_depth) {
    if (seastack::hydro::is_in_deep_water(k_mag, water_depth)) {
        double ekz = std::exp(k_mag * z);
        return {ekz, ekz};
    }
    const double kh = k_mag * water_depth;
    if (kh < kShallowWaterKhThreshold) {
        return {1.0 / kh, (z + water_depth) / water_depth};
    }
    const double denom = std::sinh(kh);
    return {std::cosh(k_mag * (z + water_depth)) / denom,
            std::sinh(k_mag * (z + water_depth)) / denom};
}

}  // namespace

namespace seastack::hydro {

LinearDirectionalWaveField::LinearDirectionalWaveField(
    std::vector<WaveComponent> components,
    double depth)
    : components_(std::move(components)) {
    water_depth_ = depth;

    if (components_.size() == 1) {
        mode_ = WaveMode::kRegular;
    } else {
        mode_ = WaveMode::kIrregular;
    }

    PrecomputeArrays();
}

void LinearDirectionalWaveField::Initialize() {
    // Wavenumbers follow ComponentSampler + environment (ApplySimulationEnvironment /
    // UpdateEnvironment); no separate init required.
}

void LinearDirectionalWaveField::PrecomputeArrays() {
    const Eigen::Index n = static_cast<Eigen::Index>(components_.size());
    amplitudes_.resize(n);
    kx_.resize(n);
    ky_.resize(n);
    omegas_.resize(n);
    phases_.resize(n);
    cos_dirs_.resize(n);
    sin_dirs_.resize(n);
    wavenumbers_.resize(n);

    for (Eigen::Index i = 0; i < n; ++i) {
        const auto& c = components_[i];
        amplitudes_[i]  = c.amplitude;
        kx_[i]          = c.k * std::cos(c.direction);
        ky_[i]          = c.k * std::sin(c.direction);
        omegas_[i]      = c.omega;
        phases_[i]      = c.phase;
        cos_dirs_[i]    = std::cos(c.direction);
        sin_dirs_[i]    = std::sin(c.direction);
        wavenumbers_[i] = c.k;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Kinematics: elevation, velocity, acceleration
// ─────────────────────────────────────────────────────────────────────────────

double LinearDirectionalWaveField::GetElevation(const Eigen::Vector3d& position, double time) const {
    const double x = position.x();
    const double y = position.y();

    // Vectorized: phase_i = kx_i*x + ky_i*y - omega_i*t + phi_i
    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    return (amplitudes_.array() * phase_args.cos()).sum();
}

Eigen::Vector2d LinearDirectionalWaveField::GetElevationGradientXY(
    const Eigen::Vector3d& position, double time) const {
    const double x = position.x();
    const double y = position.y();

    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    const Eigen::ArrayXd neg_a_sin = -(amplitudes_.array() * phase_args.sin());

    double deta_dx = (neg_a_sin * kx_.array()).sum();
    double deta_dy = (neg_a_sin * ky_.array()).sum();

    return Eigen::Vector2d(deta_dx, deta_dy);
}

double LinearDirectionalWaveField::GetElevationForVisualization(
    const Eigen::Vector3d& position, double time, int max_components) const {
    const Eigen::Index n_total = amplitudes_.size();
    const Eigen::Index n = (max_components <= 0 || max_components >= n_total)
                           ? n_total
                           : static_cast<Eigen::Index>(max_components);

    const double x = position.x();
    const double y = position.y();

    const Eigen::ArrayXd phase_args = kx_.head(n).array() * x
                                    + ky_.head(n).array() * y
                                    - omegas_.head(n).array() * time
                                    + phases_.head(n).array();
    return (amplitudes_.head(n).array() * phase_args.cos()).sum();
}

double LinearDirectionalWaveField::GetElevation(
    const Eigen::Vector3d& pos, double t, int max_components) const {
    return GetElevationForVisualization(pos, t, max_components);
}

double LinearDirectionalWaveField::GetCharacteristicPeriod() const {
    if (components_.empty()) return 0.0;
    const auto& c = *std::max_element(
        components_.cbegin(), components_.cend(),
        [](const WaveComponent& a, const WaveComponent& b) {
            return a.amplitude < b.amplitude;
        });
    return (c.omega > 0.0) ? (2.0 * M_PI / c.omega) : 0.0;
}

std::vector<double> LinearDirectionalWaveField::GetFrequenciesHz() const {
    std::vector<double> freqs;
    freqs.reserve(components_.size());
    for (const auto& c : components_) {
        freqs.push_back(c.omega / (2.0 * M_PI));
    }
    return freqs;
}

std::vector<double> LinearDirectionalWaveField::GetSpectralDensityEstimate() const {
    // Estimate S(f) = A^2 / (2 * df) per component.
    // This is approximate -- it inverts the sampling formula A = sqrt(2*S*df).
    std::vector<double> S;
    S.reserve(components_.size());
    double df_approx = 0.0;
    if (components_.size() >= 2) {
        df_approx = std::abs(components_[1].omega - components_[0].omega) / (2.0 * M_PI);
    }
    if (df_approx <= 0.0) df_approx = 1.0;  // single component: return A^2/2

    for (const auto& c : components_) {
        S.push_back(c.amplitude * c.amplitude / (2.0 * df_approx));
    }
    return S;
}

Eigen::Vector3d LinearDirectionalWaveField::GetVelocity(
    const Eigen::Vector3d& position, double time, double elevation) const {
    auto pos = wave_stretching_
               ? GetWheelerStretchedPosition(position, elevation, water_depth_, mwl_)
               : position;

    const double x = pos.x();
    const double y = pos.y();
    const double z = pos.z() - mwl_;

    // Vectorized phase computation.
    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    const Eigen::ArrayXd cos_phase = phase_args.cos();
    const Eigen::ArrayXd sin_phase = phase_args.sin();

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    for (Eigen::Index i = 0; i < amplitudes_.size(); ++i) {
        const double k_mag = std::abs(wavenumbers_[i]);
        if (k_mag == 0.0 || omegas_[i] == 0.0 || amplitudes_[i] == 0.0) continue;

        const auto [H_cosh, H_sinh] = ComputeDepthAttenuation(k_mag, z, water_depth_);

        const double u_horiz = omegas_[i] * amplitudes_[i] * H_cosh * cos_phase[i];
        const double u_vert  = omegas_[i] * amplitudes_[i] * H_sinh * sin_phase[i];

        velocity[0] += u_horiz * cos_dirs_[i];
        velocity[1] += u_horiz * sin_dirs_[i];
        velocity[2] += u_vert;
    }

    return velocity;
}

Eigen::Vector3d LinearDirectionalWaveField::GetAcceleration(
    const Eigen::Vector3d& position, double time, double elevation) const {
    auto pos = wave_stretching_
               ? GetWheelerStretchedPosition(position, elevation, water_depth_, mwl_)
               : position;

    const double x = pos.x();
    const double y = pos.y();
    const double z = pos.z() - mwl_;

    const Eigen::ArrayXd phase_args = kx_.array() * x
                                    + ky_.array() * y
                                    - omegas_.array() * time
                                    + phases_.array();
    const Eigen::ArrayXd sin_phase = phase_args.sin();
    const Eigen::ArrayXd cos_phase = phase_args.cos();

    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    for (Eigen::Index i = 0; i < amplitudes_.size(); ++i) {
        const double k_mag = std::abs(wavenumbers_[i]);
        if (k_mag == 0.0 || omegas_[i] == 0.0 || amplitudes_[i] == 0.0) continue;

        const auto [H_cosh, H_sinh] = ComputeDepthAttenuation(k_mag, z, water_depth_);

        const double w2 = omegas_[i] * omegas_[i];
        const double a_horiz =  w2 * amplitudes_[i] * H_cosh * sin_phase[i];
        const double a_vert  = -w2 * amplitudes_[i] * H_sinh * cos_phase[i];

        acceleration[0] += a_horiz * cos_dirs_[i];
        acceleration[1] += a_horiz * sin_dirs_[i];
        acceleration[2] += a_vert;
    }

    return acceleration;
}

// ─────────────────────────────────────────────────────────────────────────────
// Environment update
// ─────────────────────────────────────────────────────────────────────────────

void LinearDirectionalWaveField::UpdateEnvironment(double gravity, double depth) {
    const double prev_depth = water_depth_;
    const double prev_g     = g_;

    WaveBase::UpdateEnvironment(gravity, depth);

    const bool env_changed = (std::abs(water_depth_ - prev_depth) > 1e-12)
                          || (std::abs(g_ - prev_g) > 1e-12);
    if (env_changed) {
        for (auto& c : components_) {
            c.k = ComputeWaveNumber(c.omega, water_depth_, g_);
        }
        PrecomputeArrays();
    }
}

void LinearDirectionalWaveField::ApplySimulationEnvironment(
    const HydroData::SimulationParameters& sim_data,
    bool use_file_water_depth) {
    const double prev_depth = water_depth_;
    const double prev_g     = g_;
    g_ = sim_data.g;
    if (use_file_water_depth) {
        water_depth_ = sim_data.water_depth;
    }

    const bool env_changed = (std::abs(water_depth_ - prev_depth) > 1e-12)
                            || (std::abs(g_ - prev_g) > 1e-12);
    if (env_changed) {
        for (auto& c : components_) {
            c.k = ComputeWaveNumber(c.omega, water_depth_, g_);
        }
        PrecomputeArrays();
    }
}

double LinearDirectionalWaveField::ExcitationRampFactor(double t) const {
    if (ramp_duration_ <= 0.0 || t >= ramp_duration_) {
        return 1.0;
    }
    if (t <= 0.0) {
        return 0.0;
    }
    return 0.5 * (1.0 - std::cos(M_PI * t / ramp_duration_));
}

double LinearDirectionalWaveField::GetExcitationRampForVisualization(double time) const {
    return ExcitationRampFactor(time);
}

}  // namespace seastack::hydro
