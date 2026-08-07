/*********************************************************************
 * @file  excitation_component.cpp
 * @brief Implementation of wave excitation force component.
 *********************************************************************/

#include <seastack/hydro/force_components/excitation_component.h>

#include <cmath>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace seastack::hydro {

ExcitationComponent::ExcitationComponent(
    WaveComponentData wave_data,
    std::vector<Eigen::MatrixXd> excitation_re,
    std::vector<Eigen::MatrixXd> excitation_im,
    int num_bodies,
    double ramp_duration)
    : wave_data_(std::move(wave_data)),
      excitation_re_(std::move(excitation_re)),
      excitation_im_(std::move(excitation_im)),
      num_bodies_(num_bodies),
      ramp_duration_(ramp_duration) {
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
    // A larger buffer (auxiliary mooring-only bodies appended) is allowed; only
    // the first num_bodies_ are written.
    if (static_cast<int>(inout_forces.size()) < num_bodies_) {
        throw std::runtime_error(
            "ExcitationComponent::Compute: inout_forces.size() too small (need at least " +
            std::to_string(num_bodies_) + ", got " + std::to_string(inout_forces.size()) + ")");
    }

    ComputeFromTFData(time, inout_forces);
}

void ExcitationComponent::ComputeFromTFData(double time, BodyForces& inout_forces) {
    const Eigen::Index n_comp = wave_data_.amplitudes.size();
    if (n_comp == 0) return;

    const Eigen::ArrayXd theta =
        wave_data_.phases.head(n_comp).array() -
        wave_data_.omegas.head(n_comp).array() * time;
    const Eigen::ArrayXd cos_theta = theta.cos();
    const Eigen::ArrayXd sin_theta = theta.sin();

    const Eigen::ArrayXd weighted_cos =
        wave_data_.amplitudes.head(n_comp).array() * cos_theta;
    const Eigen::ArrayXd weighted_sin =
        wave_data_.amplitudes.head(n_comp).array() * sin_theta;

    // Cosine ramp factor (scales only the excitation contribution, not the
    // entire accumulated force vector which already contains buoyancy, etc.)
    double ramp = 1.0;
    if (ramp_duration_ > 0.0 && time < ramp_duration_) {
        ramp = (time <= 0.0) ? 0.0
             : 0.5 * (1.0 - std::cos(M_PI * time / ramp_duration_));
    }

    for (int b = 0; b < num_bodies_; ++b) {
        const Eigen::VectorXd f_re = excitation_re_[b] * weighted_cos.matrix();
        const Eigen::VectorXd f_im = excitation_im_[b] * weighted_sin.matrix();

        for (int dof = 0; dof < kDofPerBody; ++dof) {
            double val = (f_re[dof] - f_im[dof]) * ramp;
            if (dof < 3) {
                inout_forces[b].force[dof] += val;
            } else {
                inout_forces[b].moment[dof - 3] += val;
            }
        }
    }
}

}  // namespace seastack::hydro
