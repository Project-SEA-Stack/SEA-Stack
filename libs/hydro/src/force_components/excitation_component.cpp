/*********************************************************************
 * @file  excitation_component.cpp
 * @brief Implementation of wave excitation force component.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_component.h>
#include <seastack/hydro/heading_frame.h>

#include <cmath>
#include <stdexcept>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace seastack::hydro {

ExcitationComponent::ExcitationComponent(
    WaveComponentData wave_data,
    std::vector<Eigen::MatrixXd> excitation_re,
    std::vector<Eigen::MatrixXd> excitation_im,
    int num_bodies,
    double ramp_duration,
    std::shared_ptr<const AmplitudeEnvelope> envelope,
    std::shared_ptr<const WaveBase> wave,
    TransferFunction transfer)
    : wave_data_(std::move(wave_data)),
      excitation_re_(std::move(excitation_re)),
      excitation_im_(std::move(excitation_im)),
      num_bodies_(num_bodies),
      ramp_duration_(ramp_duration),
      envelope_(std::move(envelope)),
      wave_(std::move(wave)),
      transfer_(std::move(transfer)) {
    if (num_bodies_ <= 0) {
        throw std::invalid_argument(
            "ExcitationComponent: num_bodies must be > 0 (got " + std::to_string(num_bodies_) + ")");
    }
    if (static_cast<int>(excitation_re_.size()) != num_bodies_) {
        throw std::invalid_argument(
            "ExcitationComponent: excitation_re size mismatch");
    }
}

void ExcitationComponent::Compute(const SystemState& state,
                                  double time,
                                  BodyForces& inout_forces) {
    if (static_cast<int>(inout_forces.size()) != num_bodies_) {
        throw std::runtime_error(
            "ExcitationComponent::Compute: inout_forces.size() mismatch (expected " +
            std::to_string(num_bodies_) + ", got " + std::to_string(inout_forces.size()) + ")");
    }

    if (static_cast<int>(state.bodies.size()) != num_bodies_) {
        throw std::runtime_error(
            "ExcitationComponent::Compute: state.bodies.size() mismatch (expected " +
            std::to_string(num_bodies_) + ", got " + std::to_string(state.bodies.size()) + ")");
    }

    ComputeFromTFData(state, time, inout_forces);
}

void ExcitationComponent::AddNewWaveComponents() {
    const auto* comps = wave_->GetWaveComponents();
    const Eigen::Index n_old = wave_data_.amplitudes.size();
    const Eigen::Index n_new = comps ? static_cast<Eigen::Index>(comps->size()) : 0;
    if (n_new <= n_old) return;
    if (!transfer_) {
        throw std::logic_error("ExcitationComponent: the wave changed period or direction, but no excitation "
                               "transfer function was given for the new components");
    }
    const std::vector<WaveComponent> added(comps->begin() + n_old, comps->end());
    const ExcitationTFData tf = transfer_(added);
    if (static_cast<int>(tf.re.size()) != num_bodies_ || static_cast<int>(tf.im.size()) != num_bodies_) {
        throw std::runtime_error("ExcitationComponent: transfer function returned the wrong number of bodies");
    }
    wave_data_.amplitudes.conservativeResize(n_new);
    wave_data_.omegas.conservativeResize(n_new);
    wave_data_.phases.conservativeResize(n_new);
    for (Eigen::Index i = n_old; i < n_new; ++i) {
        const auto& c = (*comps)[static_cast<std::size_t>(i)];
        wave_data_.amplitudes[i] = c.amplitude;
        wave_data_.omegas[i] = c.omega;
        wave_data_.phases[i] = c.phase;
    }
    for (int b = 0; b < num_bodies_; ++b) {
        excitation_re_[b].conservativeResize(Eigen::NoChange, n_new);
        excitation_im_[b].conservativeResize(Eigen::NoChange, n_new);
        excitation_re_[b].rightCols(n_new - n_old) = tf.re[b];
        excitation_im_[b].rightCols(n_new - n_old) = tf.im[b];
    }
}

void ExcitationComponent::ComputeFromTFData(const SystemState& state, double time,
                                            BodyForces& inout_forces) {
    // Live period or direction change: the wave holds several weighted sets.
    const bool layered = wave_ && !wave_->GetWaveLayers().empty();
    if (layered) {
        AddNewWaveComponents();
    }
    const Eigen::Index n_comp = wave_data_.amplitudes.size();
    if (n_comp == 0) return;

    const Eigen::ArrayXd theta =
        wave_data_.phases.head(n_comp).array() -
        wave_data_.omegas.head(n_comp).array() * time;
    const Eigen::ArrayXd cos_theta = theta.cos();
    const Eigen::ArrayXd sin_theta = theta.sin();

    Eigen::ArrayXd weighted_cos =
        wave_data_.amplitudes.head(n_comp).array() * cos_theta;
    Eigen::ArrayXd weighted_sin =
        wave_data_.amplitudes.head(n_comp).array() * sin_theta;
    if (layered) {
        wave_->GetComponentWeights(time, weights_);
        weighted_cos *= weights_.array();
        weighted_sin *= weights_.array();
    }

    // Cosine ramp factor (scales only the excitation contribution, not the
    // entire accumulated force vector which already contains buoyancy, etc.)
    double ramp = 1.0;
    if (ramp_duration_ > 0.0 && time < ramp_duration_) {
        ramp = (time <= 0.0) ? 0.0
             : 0.5 * (1.0 - std::cos(M_PI * time / ramp_duration_));
    }
    if (envelope_ && envelope_->HasChanges()) {
        ramp *= envelope_->Value(time);
    }

    for (int b = 0; b < num_bodies_; ++b) {
        const Eigen::VectorXd f_re = excitation_re_[b] * weighted_cos.matrix();
        const Eigen::VectorXd f_im = excitation_im_[b] * weighted_sin.matrix();

        const Eigen::VectorXd f_heading = (f_re - f_im) * ramp;
        const Eigen::Matrix3d heading_to_world = HeadingToWorld(state.bodies[b]);
        inout_forces[b].force  += heading_to_world * f_heading.head<3>();
        inout_forces[b].moment += heading_to_world * f_heading.segment<3>(3);
    }
}

}  // namespace seastack::hydro
