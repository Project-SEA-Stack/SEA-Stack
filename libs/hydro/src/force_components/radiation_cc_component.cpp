/*********************************************************************
 * @file  radiation_cc_component.cpp
 * @brief Implementation of the constant-coefficient radiation damping component.
 *********************************************************************/

#include <seastack/hydro/force_components/radiation_cc_component.h>
#include <seastack/core/types.h>
#include <seastack/hydro/heading_frame.h>

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

    // Velocities and forces in each body's heading frame (heading_frame.h).
    for (int b = 0; b < num_bodies_; ++b) {
        const auto& body_state = state.bodies[b];
        const int offset = b * kDofPerBody;
        const Eigen::Matrix3d world_to_heading = HeadingToWorld(body_state).transpose();
        velocity_buffer_.segment<3>(offset)     = world_to_heading * body_state.linear_velocity;
        velocity_buffer_.segment<3>(offset + 3) = world_to_heading * body_state.angular_velocity;
    }

    force_buffer_.noalias() = radiation_damping_ * velocity_buffer_;

    // F_rad,damping = -B(w) * xdot
    for (int b = 0; b < num_bodies_; ++b) {
        const int offset = b * kDofPerBody;
        const Eigen::Matrix3d heading_to_world = HeadingToWorld(state.bodies[b]);
        inout_forces[b].force  -= heading_to_world * force_buffer_.segment<3>(offset);
        inout_forces[b].moment -= heading_to_world * force_buffer_.segment<3>(offset + 3);
    }
}

}  // namespace seastack::hydro
