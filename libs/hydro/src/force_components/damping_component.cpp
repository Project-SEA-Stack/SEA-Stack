/*********************************************************************
 * @file  damping_component.cpp
 * @brief Implementation of per-DOF body damping (linear + quadratic).
 *
 * Velocities are rotated from world frame to body frame using the
 * RPY orientation.  The damping wrench is computed in the body frame
 * and rotated back to the world frame for force accumulation.
 *********************************************************************/

#include <seastack/hydro/force_components/damping_component.h>

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace seastack::hydro {

namespace {

// Build the body-to-world rotation matrix from intrinsic ZYX Euler angles.
// orientation_rpy is [roll, pitch, yaw].
Eigen::Matrix3d BodyToWorldRotation(const Eigen::Vector3d& rpy) {
    const double roll  = rpy[0];
    const double pitch = rpy[1];
    const double yaw   = rpy[2];
    return (Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

}  // namespace

DampingComponent::DampingComponent(
    const std::vector<std::array<double, 6>>& linear_coeffs,
    const std::vector<std::array<double, 6>>& quadratic_coeffs)
    : linear_coeffs_(linear_coeffs),
      quadratic_coeffs_(quadratic_coeffs),
      num_bodies_(static_cast<int>(linear_coeffs.size())) {
    if (num_bodies_ == 0) {
        throw std::invalid_argument(
            "DampingComponent: linear_coeffs must not be empty");
    }
    if (!quadratic_coeffs_.empty() &&
        static_cast<int>(quadratic_coeffs_.size()) != num_bodies_) {
        throw std::invalid_argument(
            "DampingComponent: quadratic_coeffs size (" +
            std::to_string(quadratic_coeffs_.size()) +
            ") must match linear_coeffs size (" +
            std::to_string(num_bodies_) + ")");
    }
}

void DampingComponent::Compute(const SystemState& state,
                               double /*time*/,
                               BodyForces& inout_forces) {
    if (static_cast<int>(state.bodies.size()) < num_bodies_ ||
        static_cast<int>(inout_forces.size()) < num_bodies_) {
        throw std::invalid_argument(
            "DampingComponent::Compute: state.bodies size (" +
            std::to_string(state.bodies.size()) + ") or inout_forces size (" +
            std::to_string(inout_forces.size()) +
            ") is less than coefficients count (" +
            std::to_string(num_bodies_) + ")");
    }

    const bool has_quadratic = !quadratic_coeffs_.empty();

    for (int b = 0; b < num_bodies_; ++b) {
        const auto& body = state.bodies[b];

        // Rotation from body to world frame.
        const Eigen::Matrix3d R = BodyToWorldRotation(body.orientation_rpy);

        // Transform velocities from world frame to body frame.
        const Eigen::Vector3d v_lin_body  = R.transpose() * body.linear_velocity;
        const Eigen::Vector3d v_ang_body  = R.transpose() * body.angular_velocity;

        const auto& Bl = linear_coeffs_[b];

        // Damping wrench in body frame: F_i = -B_lin_i * v_i - B_quad_i * v_i * |v_i|
        Eigen::Vector3d f_body, m_body;
        f_body[0] = -Bl[0] * v_lin_body[0];
        f_body[1] = -Bl[1] * v_lin_body[1];
        f_body[2] = -Bl[2] * v_lin_body[2];
        m_body[0] = -Bl[3] * v_ang_body[0];
        m_body[1] = -Bl[4] * v_ang_body[1];
        m_body[2] = -Bl[5] * v_ang_body[2];

        if (has_quadratic) {
            const auto& Bq = quadratic_coeffs_[b];
            f_body[0] -= Bq[0] * v_lin_body[0] * std::abs(v_lin_body[0]);
            f_body[1] -= Bq[1] * v_lin_body[1] * std::abs(v_lin_body[1]);
            f_body[2] -= Bq[2] * v_lin_body[2] * std::abs(v_lin_body[2]);
            m_body[0] -= Bq[3] * v_ang_body[0] * std::abs(v_ang_body[0]);
            m_body[1] -= Bq[4] * v_ang_body[1] * std::abs(v_ang_body[1]);
            m_body[2] -= Bq[5] * v_ang_body[2] * std::abs(v_ang_body[2]);
        }

        // Rotate wrench back to world frame and accumulate.
        const Eigen::Vector3d f_world = R * f_body;
        const Eigen::Vector3d m_world = R * m_body;

        inout_forces[b].force  += f_world;
        inout_forces[b].moment += m_world;
    }
}

}  // namespace seastack::hydro
