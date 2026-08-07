/*********************************************************************
 * @file  excitation_irf_component.cpp
 * @brief Excitation force via IRF convolution with elevation history.
 *
 * Implements the Cummins-equation excitation term:
 *   F_exc(t) = integral_0^T_max  K_exc(tau) * eta(t - tau) dtau
 * using trapezoidal quadrature, matching the removed IrregularWaves
 * class for backward compatibility with IRF-calibrated tests.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_irf_component.h>
#include <seastack/hydro/hydro_data.h>
#include <seastack/core/math_constants.h>

#include <cmath>
#include <stdexcept>

namespace seastack::hydro {

namespace {

Eigen::VectorXd ComputeWidthVector(const Eigen::VectorXd& t) {
    const Eigen::Index n = t.size();
    Eigen::VectorXd w(n);
    if (n == 0) return w;
    if (n == 1) {
        w[0] = 0.0;
        return w;
    }
    w[0] = 0.5 * (t[1] - t[0]);
    for (Eigen::Index i = 1; i < n - 1; ++i) {
        w[i] = 0.5 * (t[i + 1] - t[i - 1]);
    }
    w[n - 1] = 0.5 * (t[n - 1] - t[n - 2]);
    return w;
}

}  // namespace

ExcitationIrfComponent::ExcitationIrfComponent(
    const HydroData& data,
    std::shared_ptr<WaveBase> waves,
    int num_bodies,
    double truncation_time,
    double ramp_duration)
    : data_(data),
      waves_(std::move(waves)),
      num_bodies_(num_bodies),
      ramp_duration_(ramp_duration) {

    if (!waves_) {
        throw std::invalid_argument(
            "ExcitationIrfComponent: waves pointer must not be null");
    }
    if (num_bodies_ <= 0) {
        throw std::invalid_argument(
            "ExcitationIrfComponent: num_bodies must be > 0 (got " +
            std::to_string(num_bodies_) + ")");
    }
    if (static_cast<int>(data_.GetIrregularWaveInfos().size()) < num_bodies_) {
        throw std::invalid_argument(
            "ExcitationIrfComponent: irreg_wave_data has " +
            std::to_string(data_.GetIrregularWaveInfos().size()) +
            " entries but num_bodies is " + std::to_string(num_bodies_) +
            " (possible malformed H5 file)");
    }

    body_irfs_.resize(num_bodies_);
    for (int b = 0; b < num_bodies_; ++b) {
        const auto& irreg = data_.GetIrregularWaveInfos()[b];
        Eigen::VectorXd tvec = irreg.excitation_irf_time;
        Eigen::MatrixXd kmat = irreg.excitation_irf_matrix;  // 6 x Nt

        if (truncation_time > 0.0 && tvec.size() > 0) {
            Eigen::Index keep = tvec.size();
            for (Eigen::Index i = 0; i < tvec.size(); ++i) {
                if (tvec[i] > truncation_time) {
                    keep = i;
                    break;
                }
            }
            if (keep < tvec.size()) {
                tvec = tvec.head(keep).eval();
                kmat = kmat.leftCols(keep).eval();
            }
        }

        body_irfs_[b].time_vec  = tvec;
        body_irfs_[b].width_vec = ComputeWidthVector(tvec);
        body_irfs_[b].kernel    = kmat;
    }

    // Initialize eta_at_tau buffer to maximum size needed (max n_steps across all bodies)
    Eigen::Index max_n_steps = 0;
    for (const auto& birf : body_irfs_) {
        if (birf.time_vec.size() > max_n_steps) {
            max_n_steps = birf.time_vec.size();
        }
    }
    eta_at_tau_buffer_.resize(max_n_steps);

    TryBuildFastPath();
}

void ExcitationIrfComponent::TryBuildFastPath() {
    const auto* wave_comps = waves_->GetWaveComponents();
    if (!wave_comps || wave_comps->empty()) return;

    const auto& components = *wave_comps;

    const auto& birf0 = body_irfs_[0];
    const Eigen::Index n_irf = birf0.time_vec.size();
    const Eigen::Index n_comp = static_cast<Eigen::Index>(components.size());

    comp_amplitudes_.resize(n_comp);
    comp_omegas_.resize(n_comp);
    comp_phases_.resize(n_comp);
    for (Eigen::Index i = 0; i < n_comp; ++i) {
        comp_amplitudes_[i] = components[i].amplitude;
        comp_omegas_[i]     = components[i].omega;
        comp_phases_[i]     = components[i].phase;
    }

    // Precompute cos(omega_i * tau_j) and sin(omega_i * tau_j).
    irf_cos_wt_.resize(n_irf, n_comp);
    irf_sin_wt_.resize(n_irf, n_comp);
    for (Eigen::Index j = 0; j < n_irf; ++j) {
        const double tau_j = birf0.time_vec[j];
        for (Eigen::Index i = 0; i < n_comp; ++i) {
            const double arg = comp_omegas_[i] * tau_j;
            irf_cos_wt_(j, i) = std::cos(arg);
            irf_sin_wt_(j, i) = std::sin(arg);
        }
    }

    has_fast_path_ = true;
}

void ExcitationIrfComponent::ComputeFastPath(
    double time, Eigen::VectorXd& eta_at_tau) const {
    // theta_i = phi_i - omega_i * t
    const Eigen::ArrayXd theta = comp_phases_.array() - comp_omegas_.array() * time;
    const Eigen::VectorXd a_cos = (comp_amplitudes_.array() * theta.cos()).matrix();
    const Eigen::VectorXd a_sin = (comp_amplitudes_.array() * theta.sin()).matrix();

    // eta[j] = sum_i A_i*cos(phi_i - omega_i*t)*cos(omega_i*tau_j)
    //        - sum_i A_i*sin(phi_i - omega_i*t)*sin(omega_i*tau_j)
    eta_at_tau = irf_cos_wt_ * a_cos - irf_sin_wt_ * a_sin;

    if (ramp_duration_ > 0.0) {
        const auto& birf0 = body_irfs_[0];
        for (Eigen::Index s = 0; s < eta_at_tau.size(); ++s) {
            eta_at_tau[s] *= RampFactor(time - birf0.time_vec[s]);
        }
    }
}

void ExcitationIrfComponent::ComputeSlowPath(
    double time, Eigen::VectorXd& eta_at_tau) const {
    const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    const auto& birf0 = body_irfs_[0];
    for (Eigen::Index s = 0; s < eta_at_tau.size(); ++s) {
        const double t_eval = time - birf0.time_vec[s];
        double eta = waves_->GetElevation(origin, t_eval);
        if (ramp_duration_ > 0.0) {
            eta *= RampFactor(t_eval);
        }
        eta_at_tau[s] = eta;
    }
}

void ExcitationIrfComponent::Compute(
    const SystemState& /*state*/,
    double time,
    BodyForces& inout_forces) {

    // A larger buffer (auxiliary mooring-only bodies appended) is allowed; only
    // the first num_bodies_ are written.
    if (static_cast<int>(inout_forces.size()) < num_bodies_) {
        throw std::runtime_error(
            "ExcitationIrfComponent::Compute: inout_forces.size() too small");
    }

    const auto& birf0 = body_irfs_[0];
    const Eigen::Index n_steps = birf0.time_vec.size();
    if (n_steps == 0) return;

    // Use member buffer instead of allocating every timestep
    // Resize to exact size needed (buffer is pre-allocated to max size, so this is cheap)
    eta_at_tau_buffer_.resize(n_steps);
    Eigen::VectorXd& eta_at_tau = eta_at_tau_buffer_;
    
    if (has_fast_path_) {
        ComputeFastPath(time, eta_at_tau);
    } else {
        ComputeSlowPath(time, eta_at_tau);
    }

    for (int b = 0; b < num_bodies_; ++b) {
        const auto& birf = body_irfs_[b];

        for (int dof = 0; dof < kDofPerBody; ++dof) {
            double force_dof = 0.0;
            for (Eigen::Index s = 0; s < n_steps; ++s) {
                const double w = birf.width_vec[s];
                if (w == 0.0) continue;
                force_dof += birf.kernel(dof, s) * eta_at_tau[s] * w;
            }
            if (dof < 3) {
                inout_forces[b].force[dof] += force_dof;
            } else {
                inout_forces[b].moment[dof - 3] += force_dof;
            }
        }
    }
}

double ExcitationIrfComponent::RampFactor(double t) const {
    if (ramp_duration_ <= 0.0 || t >= ramp_duration_)
        return 1.0;
    if (t <= 0.0)
        return 0.0;
    return t / ramp_duration_;
}

}  // namespace seastack::hydro
