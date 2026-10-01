/*********************************************************************
 * @file  radiation_cc_component.cpp
 * @brief Implementation of the constant-coefficient radiation damping component.
 *********************************************************************/

#include <seastack/hydro/force_components/radiation_cc_component.h>
#include <seastack/core/types.h>

#include <stdexcept>
#include <string>

namespace seastack::hydro {

RadiationConstantCoefficientComponent::RadiationConstantCoefficientComponent(
    int num_bodies,
    const Eigen::MatrixXd& radiation_damping)
    : num_bodies_(num_bodies), radiation_damping_(radiation_damping) {
    if (num_bodies_ <= 0) {
        throw std::invalid_argument(
            "RadiationConstantCoefficientComponent: num_bodies must be > 0 (got " +
            std::to_string(num_bodies_) + ")");
    }
    const int n_dofs = kDofPerBody * num_bodies_;
    if (radiation_damping_.rows() != n_dofs || radiation_damping_.cols() != n_dofs) {
        throw std::invalid_argument(
            "RadiationConstantCoefficientComponent: radiation damping matrix is " +
            std::to_string(radiation_damping_.rows()) + "x" +
            std::to_string(radiation_damping_.cols()) + ", expected " +
            std::to_string(n_dofs) + "x" + std::to_string(n_dofs) + ".");
    }
    if (!radiation_damping_.allFinite()) {
        throw std::invalid_argument(
            "RadiationConstantCoefficientComponent: radiation damping matrix contains "
            "non-finite values.");
    }
    velocity_buffer_.setZero(n_dofs);
    force_buffer_.setZero(n_dofs);
}

void RadiationConstantCoefficientComponent::Compute(const SystemState& state,
                                                    double /*time*/,
                                                    BodyForces& inout_forces) {
    if (static_cast<int>(state.bodies.size()) != num_bodies_ ||
        static_cast<int>(inout_forces.size()) != num_bodies_) {
        throw std::runtime_error(
            "RadiationConstantCoefficientComponent::Compute: body count mismatch (expected " +
            std::to_string(num_bodies_) + ", state has " + std::to_string(state.bodies.size()) +
            ", forces have " + std::to_string(inout_forces.size()) + ")");
    }

    for (int b = 0; b < num_bodies_; ++b) {
        const auto& body_state = state.bodies[b];
        const int offset = b * kDofPerBody;
        for (int d = 0; d < 3; ++d) {
            velocity_buffer_(offset + d)     = body_state.linear_velocity[d];
            velocity_buffer_(offset + 3 + d) = body_state.angular_velocity[d];
        }
    }

    force_buffer_.noalias() = radiation_damping_ * velocity_buffer_;

    // F_rad,damping = -B(w) * xdot
    for (int b = 0; b < num_bodies_; ++b) {
        const int offset = b * kDofPerBody;
        for (int d = 0; d < 3; ++d) {
            inout_forces[b].force[d]  -= force_buffer_(offset + d);
            inout_forces[b].moment[d] -= force_buffer_(offset + 3 + d);
        }
    }
}

}  // namespace seastack::hydro
