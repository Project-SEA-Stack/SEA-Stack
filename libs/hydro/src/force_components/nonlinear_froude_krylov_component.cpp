/*********************************************************************
 * @file  nonlinear_froude_krylov_component.cpp
 * @brief NonlinearFroudeKrylovComponent implementation.
 *
 * Per body and force evaluation:
 *   1. Transform the CG-frame mesh to the world frame (pose from BodyState).
 *   2. At each vertex: eta_v = r(t) * eta(x_v, y_v, t) and d_v = z_v - eta_v.
 *   3. Wet vertices: p_v = -rho g z_v + p_d (pressure model, section 5 of the
 *      theory doc). Clip points carry zero (they lie on the free surface).
 *   4. Integrate with WettedSurfacePressureIntegrator (moment about the CG).
 *   5. Subtract the mesh linear FK, precomputed once with the same quadrature.
 *********************************************************************/

#include <seastack/hydro/force_components/nonlinear_froude_krylov_component.h>

#include <seastack/core/math_constants.h>
#include <seastack/infra/logging.h>

#include "waves/wave_utilities.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace seastack::hydro {

namespace {

Eigen::Affine3d PoseFromState(const BodyState& body) {
    const Eigen::Vector3d& rpy = body.orientation_rpy;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translate(body.position);
    pose.rotate(Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()));
    return pose;
}

Eigen::Matrix<double, 6, 1> Stack(const Eigen::Vector3d& force, const Eigen::Vector3d& moment) {
    Eigen::Matrix<double, 6, 1> v;
    v << force, moment;
    return v;
}

}  // namespace

NonlinearFroudeKrylovComponent::NonlinearFroudeKrylovComponent(std::vector<BodySetup> bodies,
                                                               std::shared_ptr<WaveBase> wave,
                                                               double rho,
                                                               const Eigen::Vector3d& gravity,
                                                               double ramp_duration,
                                                               RampShape ramp_shape)
    : wave_(std::move(wave)),
      rho_g_(rho * gravity.norm()),
      ramp_duration_(ramp_duration),
      ramp_shape_(ramp_shape) {
    if (bodies.empty()) {
        throw std::invalid_argument("NonlinearFroudeKrylovComponent: at least one body required");
    }
    if (wave_ && wave_->GetWaveMode() == WaveMode::kNoWave) {
        wave_.reset();
    }

    if (wave_) {
        const auto* comps = wave_->GetWaveComponents();
        if (!comps) {
            throw std::runtime_error(
                "NonlinearFroudeKrylovComponent: the wave model has no discrete components "
                "(e.g. an eta-table import). Nonlinear Froude-Krylov needs a linear wave "
                "defined by components.");
        }
        const Eigen::Index n = static_cast<Eigen::Index>(comps->size());
        amplitudes_.resize(n);
        omegas_.resize(n);
        phases_.resize(n);
        kx_.resize(n);
        ky_.resize(n);
        double k_max = 0.0;
        for (Eigen::Index i = 0; i < n; ++i) {
            const auto& c = (*comps)[static_cast<std::size_t>(i)];
            amplitudes_[i] = c.amplitude;
            omegas_[i]     = c.omega;
            phases_[i]     = c.phase;
            kx_[i]         = c.k * std::cos(c.direction);
            ky_[i]         = c.k * std::sin(c.direction);
            if (c.amplitude != 0.0) k_max = std::max(k_max, std::abs(c.k));
        }
        shortest_wavelength_ = (k_max > 0.0) ? 2.0 * kPi / k_max : 0.0;
    }

    bodies_.reserve(bodies.size());
    for (auto& setup : bodies) {
        BodyData body{setup.body_index, setup.pressure_model, setup.bem_cg,
                      geometry::WettedSurfacePressureIntegrator(std::move(setup.mesh)),
                      Eigen::MatrixXd(), Eigen::MatrixXd()};
        ComputeLinearFroudeKrylovCoefficients(body);
        bodies_.push_back(std::move(body));
    }
}

double NonlinearFroudeKrylovComponent::RampFactor(double time) const {
    if (ramp_duration_ <= 0.0 || time >= ramp_duration_) return 1.0;
    if (time <= 0.0) return 0.0;
    if (ramp_shape_ == RampShape::kLinear) return time / ramp_duration_;
    return 0.5 * (1.0 - std::cos(kPi * time / ramp_duration_));
}

void NonlinearFroudeKrylovComponent::ComputeLinearFroudeKrylovCoefficients(BodyData& body) const {
    const Eigen::Index n_comp = amplitudes_.size();
    body.linear_fk_cos = Eigen::MatrixXd::Zero(6, n_comp);
    body.linear_fk_sin = Eigen::MatrixXd::Zero(6, n_comp);
    if (n_comp == 0) return;

    // Mean wetted surface: mesh at its BEM pose (BEM frame = world frame), clipped at z = 0.
    Eigen::Affine3d bem_pose = Eigen::Affine3d::Identity();
    bem_pose.translate(body.bem_cg);
    const Eigen::Matrix3Xd v = body.integrator.TransformVertices(bem_pose);
    const int nv = body.integrator.NumVertices();
    const Eigen::VectorXd height = v.row(2).transpose();
    const double depth = wave_->GetWaterDepth();

    Eigen::VectorXd p_cos(nv), p_sin(nv), q_cos(nv), q_sin(nv);
    for (Eigen::Index i = 0; i < n_comp; ++i) {
        const double k = std::hypot(kx_[i], ky_[i]);
        for (int j = 0; j < nv; ++j) {
            const double kx = kx_[i] * v(0, j) + ky_[i] * v(1, j);
            const double z_factor = (body.pressure_model == FroudeKrylovPressureModel::kHydrostatic)
                                        ? 1.0
                                        : AiryPressureDepthFactor(k, v(2, j), depth);
            // Wet vertices: linear dynamic pressure shape. Clip points: the
            // surface value rho g eta-shape, interpolated along the cut edge,
            // which is the first-order clip-point value of the nonlinear
            // integral (total pressure zero at z = eta).
            p_cos[j] = rho_g_ * z_factor * std::cos(kx);
            p_sin[j] = rho_g_ * z_factor * std::sin(kx);
            q_cos[j] = rho_g_ * std::cos(kx);
            q_sin[j] = rho_g_ * std::sin(kx);
        }
        // p_lin = r a [P_c cos(theta) - P_s sin(theta)], F = -int p n dS
        //   => F = r a [A cos(theta) + B sin(theta)], A = F[P_c], B = -F[P_s].
        const auto f_cos = body.integrator.Integrate(v, height, p_cos, q_cos, body.bem_cg);
        const auto f_sin = body.integrator.Integrate(v, height, p_sin, q_sin, body.bem_cg);
        body.linear_fk_cos.col(i) = Stack(f_cos.force, f_cos.moment);
        body.linear_fk_sin.col(i) = -Stack(f_sin.force, f_sin.moment);
    }
}

void NonlinearFroudeKrylovComponent::CheckInitialPosition(BodyData& body, const Eigen::Vector3d& position) {
    body.checked_position = true;
    if (shortest_wavelength_ <= 0.0) return;
    const double offset = (position.head<2>() - body.bem_cg.head<2>()).norm();
    if (offset > 0.01 * shortest_wavelength_) {
        LOG_WARNING("WARNING: body index " << body.body_index << " starts " << offset
                    << " m (horizontally) from its BEM CG position. Nonlinear Froude-Krylov, "
                    "like the BEM excitation, assumes the BEM frame is the world frame, so the "
                    "subtracted linear Froude-Krylov force is phased for the BEM position. "
                    "Place the body at its BEM position or regenerate the BEM data there.");
    }
}

void NonlinearFroudeKrylovComponent::Compute(const SystemState& state, double time, BodyForces& inout_forces) {
    const int num_system_bodies = static_cast<int>(state.bodies.size());
    if (static_cast<int>(last_wetted_area_.size()) != num_system_bodies) {
        last_wetted_area_.assign(num_system_bodies, 0.0);
        last_pressure_force_.assign(num_system_bodies, Eigen::Matrix<double, 6, 1>::Zero());
        last_linear_fk_.assign(num_system_bodies, Eigen::Matrix<double, 6, 1>::Zero());
    }

    const double ramp = wave_ ? RampFactor(time) : 0.0;
    const bool wavy = ramp > 0.0;

    for (auto& body : bodies_) {
        const int b = body.body_index;
        if (b < 0 || b >= num_system_bodies || b >= static_cast<int>(inout_forces.size())) {
            throw std::invalid_argument("NonlinearFroudeKrylovComponent::Compute: body index " +
                                        std::to_string(b) + " is out of range");
        }
        const auto& state_body = state.bodies[b];
        if (!body.checked_position) CheckInitialPosition(body, state_body.position);

        const Eigen::Matrix3Xd v = body.integrator.TransformVertices(PoseFromState(state_body));
        const int nv = body.integrator.NumVertices();
        height_buffer_.resize(nv);
        pressure_buffer_.resize(nv);
        zero_buffer_.setZero(nv);

        // Every vertex needs its true height above the surface: a dry vertex at
        // the end of a cut edge sets where the clip point falls.
        for (int j = 0; j < nv; ++j) {
            const Eigen::Vector3d x = v.col(j);
            if (!wavy) {
                height_buffer_[j] = x.z();
                pressure_buffer_[j] = -rho_g_ * x.z();
                continue;
            }
            const double eta = ramp * wave_->GetElevation(x, time);
            height_buffer_[j] = x.z() - eta;
            if (height_buffer_[j] > 0.0) {
                pressure_buffer_[j] = 0.0;  // dry vertex, value unused
            } else if (body.pressure_model == FroudeKrylovPressureModel::kHydrostatic) {
                pressure_buffer_[j] = rho_g_ * (eta - x.z());
            } else {
                pressure_buffer_[j] = rho_g_ * (ramp * wave_->GetDynamicPressureHead(x, time, eta) - x.z());
            }
        }

        const auto pressure = body.integrator.Integrate(v, height_buffer_, pressure_buffer_, zero_buffer_,
                                                        state_body.position);
        Eigen::Matrix<double, 6, 1> linear_fk = Eigen::Matrix<double, 6, 1>::Zero();
        if (wavy && amplitudes_.size() > 0) {
            const Eigen::ArrayXd theta = phases_.array() - omegas_.array() * time;
            const Eigen::VectorXd a_cos = (amplitudes_.array() * theta.cos()).matrix();
            const Eigen::VectorXd a_sin = (amplitudes_.array() * theta.sin()).matrix();
            linear_fk = ramp * (body.linear_fk_cos * a_cos + body.linear_fk_sin * a_sin);
        }

        inout_forces[b].force += pressure.force - linear_fk.head<3>();
        inout_forces[b].moment += pressure.moment - linear_fk.tail<3>();

        last_wetted_area_[b] = pressure.wetted_area;
        last_pressure_force_[b] = Stack(pressure.force, pressure.moment);
        last_linear_fk_[b] = linear_fk;

        if (pressure.wetted_area <= 0.0 && !body.warned_emerged) {
            body.warned_emerged = true;
            LOG_WARNING("WARNING: body index " << b << " fully emerged (no wetted surface) at t = " << time
                        << " s under nonlinear Froude-Krylov. Check the wave height and body draft; "
                        "slamming and water re-entry are not modelled.");
        }
    }
}

double NonlinearFroudeKrylovComponent::GetLastWettedArea(int body_index) const {
    if (body_index < 0 || body_index >= static_cast<int>(last_wetted_area_.size())) return 0.0;
    return last_wetted_area_[body_index];
}

Eigen::Matrix<double, 6, 1> NonlinearFroudeKrylovComponent::GetLastPressureForce(int body_index) const {
    if (body_index < 0 || body_index >= static_cast<int>(last_pressure_force_.size())) {
        return Eigen::Matrix<double, 6, 1>::Zero();
    }
    return last_pressure_force_[body_index];
}

Eigen::Matrix<double, 6, 1> NonlinearFroudeKrylovComponent::GetLastLinearFroudeKrylovForce(int body_index) const {
    if (body_index < 0 || body_index >= static_cast<int>(last_linear_fk_.size())) {
        return Eigen::Matrix<double, 6, 1>::Zero();
    }
    return last_linear_fk_[body_index];
}

}  // namespace seastack::hydro
