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
#include <seastack/hydro/heading_frame.h>
#include <seastack/hydro/hydro_data.h>
#include <seastack/core/math_constants.h>

#include <seastack/hydro/excitation_transfer.h>
#include <seastack/infra/logging.h>

#include <cmath>
#include <optional>
#include <stdexcept>

namespace seastack::hydro {

Eigen::MatrixXd SelectKernelForHeading(const std::vector<Eigen::MatrixXd>& kernels_by_heading,
                                       const Eigen::VectorXd& bem_headings,
                                       double direction_rad,
                                       const Eigen::MatrixXd& fallback) {
    const Eigen::Index n_dir = bem_headings.size();
    if (n_dir <= 1 || static_cast<Eigen::Index>(kernels_by_heading.size()) != n_dir) {
        if (n_dir > 1) {
            LOG_WARNING("Excitation IRF: H5 file has " << kernels_by_heading.size()
                        << " IRF headings but " << n_dir
                        << " wave_dir entries; using the first heading's IRF.");
        }
        return fallback;
    }

    // A direction on a BEM heading uses that kernel as stored (no blending).
    constexpr double kOnGridTolRad = 1e-9;
    for (Eigen::Index h = 0; h < n_dir; ++h) {
        double diff = std::fmod(direction_rad - bem_headings[h], kTwoPi);
        if (diff < 0.0) diff += kTwoPi;
        if (std::min(diff, kTwoPi - diff) <= kOnGridTolRad) {
            return kernels_by_heading[h];
        }
    }

    // Same bracket and linear weight as the frequency-domain path (Cartesian).
    const HeadingBracket bracket = FindHeadingBracket(bem_headings, direction_rad);
    const Eigen::MatrixXd& k_lo  = kernels_by_heading[bracket.lo];
    const Eigen::MatrixXd& k_hi  = kernels_by_heading[bracket.hi];
    if (k_lo.rows() != k_hi.rows() || k_lo.cols() != k_hi.cols()) {
        throw std::runtime_error("Excitation IRF: kernels for different headings have different sizes");
    }
    return k_lo + bracket.frac * (k_hi - k_lo);
}

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

    // The convolution uses the elevation at the origin, which carries no
    // direction, so the kernel must be the one for the sea's heading.
    std::optional<double> wave_direction;
    if (const auto* comps = waves_->GetWaveComponents(); comps && !comps->empty()) {
        wave_direction = (*comps)[0].direction;
        for (const auto& c : *comps) {
            if (c.direction != *wave_direction) {
                throw std::invalid_argument(
                    "ExcitationIrfComponent: excitation method 'irf_convolution' needs a "
                    "long-crested sea (one wave direction), but the wave components have "
                    "several directions. Use 'frequency_domain' or 'auto' for directional seas.");
            }
        }
    } else {
        wave_direction = waves_->GetEtaRecordDirection();
    }
    const Eigen::VectorXd& bem_headings = data_.GetSimulationInfo().wave_directions;
    if (wave_direction) {
        WarnIfSingleHeadingMismatch(bem_headings, *wave_direction, "irf_convolution");
    }

    body_irfs_.resize(num_bodies_);
    for (int b = 0; b < num_bodies_; ++b) {
        const auto& irreg = data_.GetIrregularWaveInfos()[b];
        Eigen::VectorXd tvec = irreg.excitation_irf_time;
        Eigen::MatrixXd kmat = irreg.excitation_irf_matrix;  // 6 x Nt, first BEM heading
        if (wave_direction) {
            kmat = SelectKernelForHeading(irreg.excitation_irf_by_heading, bem_headings, *wave_direction, kmat);
        }

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
    const SystemState& state,
    double time,
    BodyForces& inout_forces) {

    if (static_cast<int>(inout_forces.size()) != num_bodies_ ||
        static_cast<int>(state.bodies.size()) != num_bodies_) {
        throw std::runtime_error(
            "ExcitationIrfComponent::Compute: inout_forces or state.bodies size mismatch");
    }

    const auto& birf0 = body_irfs_[0];
    const Eigen::Index n_steps = birf0.time_vec.size();
    if (n_steps == 0) return;

    if (!waves_->GetWaveLayers().empty()) {
        ComputeLayered(state, time, inout_forces);
        return;
    }

    // Use member buffer instead of allocating every timestep
    // Resize to exact size needed (buffer is pre-allocated to max size, so this is cheap)
    eta_at_tau_buffer_.resize(n_steps);
    Eigen::VectorXd& eta_at_tau = eta_at_tau_buffer_;
    
    if (has_fast_path_) {
        ComputeFastPath(time, eta_at_tau);
    } else {
        ComputeSlowPath(time, eta_at_tau);
    }
    // Live wave-height changes: each past sample carries the envelope of its time.
    const AmplitudeEnvelope& envelope = *waves_->GetAmplitudeEnvelope();
    if (envelope.HasChanges()) {
        for (Eigen::Index s = 0; s < n_steps; ++s) {
            eta_at_tau[s] *= envelope.Value(time - birf0.time_vec[s]);
        }
    }

    // Excitation in each body's heading frame, rotated to world (heading_frame.h).
    for (int b = 0; b < num_bodies_; ++b) {
        const auto& birf = body_irfs_[b];

        Eigen::Matrix<double, kDofPerBody, 1> f_heading;
        for (int dof = 0; dof < kDofPerBody; ++dof) {
            double force_dof = 0.0;
            for (Eigen::Index s = 0; s < n_steps; ++s) {
                const double w = birf.width_vec[s];
                if (w == 0.0) continue;
                force_dof += birf.kernel(dof, s) * eta_at_tau[s] * w;
            }
            f_heading[dof] = force_dof;
        }
        const Eigen::Matrix3d heading_to_world = HeadingToWorld(state.bodies[b]);
        inout_forces[b].force  += heading_to_world * f_heading.head<3>();
        inout_forces[b].moment += heading_to_world * f_heading.tail<3>();
    }
}

void ExcitationIrfComponent::AddNewWaveComponents() {
    const auto& layers = waves_->GetWaveLayers();
    const auto* comps = waves_->GetWaveComponents();
    if (!has_fast_path_ || !comps) {
        throw std::logic_error("ExcitationIrfComponent: live period or direction changes need a wave defined "
                               "by components");
    }
    // Indices below are wave component indices; the arrays start at released_components_.
    const Eigen::Index n_old = released_components_ + comp_amplitudes_.size();
    const Eigen::Index n_new = static_cast<Eigen::Index>(comps->size());
    if (n_new > n_old) {
        const auto& birf0 = body_irfs_[0];
        const Eigen::Index n_irf = birf0.time_vec.size();
        const Eigen::Index n_kept = n_new - released_components_;
        comp_amplitudes_.conservativeResize(n_kept);
        comp_omegas_.conservativeResize(n_kept);
        comp_phases_.conservativeResize(n_kept);
        irf_cos_wt_.conservativeResize(Eigen::NoChange, n_kept);
        irf_sin_wt_.conservativeResize(Eigen::NoChange, n_kept);
        for (Eigen::Index i = n_old; i < n_new; ++i) {
            const auto& c = (*comps)[static_cast<std::size_t>(i)];
            const Eigen::Index col = i - released_components_;
            comp_amplitudes_[col] = c.amplitude;
            comp_omegas_[col] = c.omega;
            comp_phases_[col] = c.phase;
            for (Eigen::Index j = 0; j < n_irf; ++j) {
                const double arg = c.omega * birf0.time_vec[j];
                irf_cos_wt_(j, col) = std::cos(arg);
                irf_sin_wt_(j, col) = std::sin(arg);
            }
        }
    }
    // Kernels for layers 1, 2, ...: each layer is long-crested.
    const Eigen::VectorXd& bem_headings = data_.GetSimulationInfo().wave_directions;
    while (layer_kernels_.size() + 1 < layers.size()) {
        const WaveLayer& layer = layers[layer_kernels_.size() + 1];
        const double direction = (*comps)[layer.begin].direction;
        std::vector<Eigen::MatrixXd> kernels(static_cast<std::size_t>(num_bodies_));
        for (int b = 0; b < num_bodies_; ++b) {
            const auto& irreg = data_.GetIrregularWaveInfos()[b];
            const Eigen::MatrixXd k =
                SelectKernelForHeading(irreg.excitation_irf_by_heading, bem_headings, direction,
                                       irreg.excitation_irf_matrix);
            kernels[static_cast<std::size_t>(b)] = k.leftCols(body_irfs_[b].kernel.cols());
        }
        layer_kernels_.push_back(std::move(kernels));
    }
}

void ExcitationIrfComponent::ReleaseFinishedLayers(double time) {
    const auto& layers = waves_->GetWaveLayers();
    double kernel_span = 0.0;  // longest lag tau [s]
    for (const auto& birf : body_irfs_) {
        if (birf.time_vec.size() > 0) kernel_span = std::max(kernel_span, birf.time_vec.maxCoeff());
    }
    // A layer whose weight reached 0 at EndTime() is out of reach once
    // time - kernel_span >= EndTime(). The margin [s] covers later force
    // evaluations at slightly earlier times (integrator stages, a repeated
    // step); it assumes time steps well below 1 s.
    constexpr double margin = 1.0;
    std::size_t release = released_layers_;
    while (release + 1 < layers.size()) {  // the newest layer is never released
        const AmplitudeEnvelope& w = layers[release].weight;
        if (w.Target() != 0.0 || w.EndTime() > time - kernel_span - margin) break;
        ++release;
    }
    if (release == released_layers_) return;

    const Eigen::Index drop = static_cast<Eigen::Index>(layers[release].begin) - released_components_;
    const Eigen::Index keep = comp_amplitudes_.size() - drop;
    comp_amplitudes_ = comp_amplitudes_.tail(keep).eval();
    comp_omegas_ = comp_omegas_.tail(keep).eval();
    comp_phases_ = comp_phases_.tail(keep).eval();
    irf_cos_wt_ = irf_cos_wt_.rightCols(keep).eval();
    irf_sin_wt_ = irf_sin_wt_.rightCols(keep).eval();
    for (std::size_t l = std::max<std::size_t>(released_layers_, 1); l < release; ++l) {
        layer_kernels_[l - 1].clear();
        layer_kernels_[l - 1].shrink_to_fit();
    }
    released_layers_ = release;
    released_components_ += drop;
}

void ExcitationIrfComponent::ComputeLayered(const SystemState& state, double time, BodyForces& inout_forces) {
    AddNewWaveComponents();
    ReleaseFinishedLayers(time);
    const auto& layers = waves_->GetWaveLayers();
    const auto& birf0 = body_irfs_[0];
    const Eigen::Index n_steps = birf0.time_vec.size();

    const Eigen::ArrayXd theta = comp_phases_.array() - comp_omegas_.array() * time;
    const Eigen::VectorXd a_cos = (comp_amplitudes_.array() * theta.cos()).matrix();
    const Eigen::VectorXd a_sin = (comp_amplitudes_.array() * theta.sin()).matrix();
    const AmplitudeEnvelope& envelope = *waves_->GetAmplitudeEnvelope();

    std::vector<Eigen::Matrix<double, kDofPerBody, 1>> f_heading(
        static_cast<std::size_t>(num_bodies_), Eigen::Matrix<double, kDofPerBody, 1>::Zero());
    Eigen::VectorXd& eta = layer_eta_buffer_;
    Eigen::VectorXd& weight = layer_weight_buffer_;
    weight.resize(n_steps);
    for (std::size_t l = released_layers_; l < layers.size(); ++l) {
        // Weight of each past sample, with the ramp and e(t - tau).
        bool active = false;
        for (Eigen::Index s = 0; s < n_steps; ++s) {
            const double t_eval = time - birf0.time_vec[s];
            double w = layers[l].weight.Value(t_eval);
            if (w != 0.0) {
                active = true;
                if (ramp_duration_ > 0.0) w *= RampFactor(t_eval);
                if (envelope.HasChanges()) w *= envelope.Value(t_eval);
            }
            weight[s] = w;
        }
        if (!active) continue;  // faded out over the whole kernel length, or not started

        const Eigen::Index begin = static_cast<Eigen::Index>(layers[l].begin) - released_components_;
        const Eigen::Index size = static_cast<Eigen::Index>(layers[l].end - layers[l].begin);
        eta = irf_cos_wt_.middleCols(begin, size) * a_cos.segment(begin, size) -
              irf_sin_wt_.middleCols(begin, size) * a_sin.segment(begin, size);
        eta.array() *= weight.array();

        for (int b = 0; b < num_bodies_; ++b) {
            const auto& birf = body_irfs_[b];
            const Eigen::MatrixXd& kernel = (l == 0) ? birf.kernel : layer_kernels_[l - 1][static_cast<std::size_t>(b)];
            for (int dof = 0; dof < kDofPerBody; ++dof) {
                double force_dof = 0.0;
                for (Eigen::Index s = 0; s < n_steps; ++s) {
                    const double w = birf.width_vec[s];
                    if (w == 0.0) continue;
                    force_dof += kernel(dof, s) * eta[s] * w;
                }
                f_heading[static_cast<std::size_t>(b)][dof] += force_dof;
            }
        }
    }

    // Excitation in each body's heading frame, rotated to world (heading_frame.h).
    for (int b = 0; b < num_bodies_; ++b) {
        const Eigen::Matrix3d heading_to_world = HeadingToWorld(state.bodies[b]);
        inout_forces[b].force  += heading_to_world * f_heading[static_cast<std::size_t>(b)].head<3>();
        inout_forces[b].moment += heading_to_world * f_heading[static_cast<std::size_t>(b)].tail<3>();
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
