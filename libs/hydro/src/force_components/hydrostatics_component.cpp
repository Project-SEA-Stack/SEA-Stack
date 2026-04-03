/*********************************************************************
 * @file  hydrostatics_component.cpp
 * @brief Implementation of hydrostatics force component.
 *
 * Linear hydrostatic restoring force:
 *   F_hs = -C * (x - x_eq)
 * where C is the 6x6 hydrostatic stiffness matrix and x_eq the equilibrium.
 * See: Faltinsen, O.M. (1990), "Sea Loads on Ships and Offshore Structures,"
 *      Cambridge University Press, Chapter 3.
 *********************************************************************/

#include <seastack/hydro/force_components/hydrostatics_component.h>
#include <seastack/hydro/hydro_data.h>

#include <Eigen/Dense>
#include <stdexcept>

namespace seastack::hydro {

HydrostaticsComponent::HydrostaticsComponent(
    const HydroData& file_info,
    int num_bodies,
    const std::vector<double>& equilibrium,
    const std::vector<double>& cb_minus_cg,
    const Eigen::Vector3d& gravitational_acceleration,
    const std::vector<int>& active_bodies)
    : file_info_(file_info),
      num_bodies_(num_bodies),
      equilibrium_(equilibrium),
      cb_minus_cg_(cb_minus_cg),
      gravitational_acceleration_(gravitational_acceleration),
      active_bodies_(active_bodies) {
    // If no active body subset specified, operate on all bodies.
    if (active_bodies_.empty()) {
        active_bodies_.resize(num_bodies);
        for (int i = 0; i < num_bodies; ++i) active_bodies_[i] = i;
    }

    // Require at least one body for hydrostatic force computation.
    if (num_bodies_ <= 0) {
        throw std::invalid_argument(
            "HydrostaticsComponent: num_bodies must be > 0 (got " + std::to_string(num_bodies_) + ")");
    }

    // Validate equilibrium array size: must have 6 DOF per body.
    const int expected_equilibrium_size = kDofPerBody * num_bodies_;
    if (static_cast<int>(equilibrium_.size()) != expected_equilibrium_size) {
        throw std::invalid_argument(
            "HydrostaticsComponent: equilibrium array size mismatch (expected " +
            std::to_string(expected_equilibrium_size) + ", got " +
            std::to_string(equilibrium_.size()) + ")");
    }

    // Validate cb_minus_cg array size: must have 3 components per body.
    const int expected_cb_cg_size = kDofLinOrRot * num_bodies_;
    if (static_cast<int>(cb_minus_cg_.size()) != expected_cb_cg_size) {
        throw std::invalid_argument(
            "HydrostaticsComponent: cb_minus_cg array size mismatch (expected " +
            std::to_string(expected_cb_cg_size) + ", got " +
            std::to_string(cb_minus_cg_.size()) + ")");
    }
    
    // Cache stiffness matrices, rho*g, and buoyancy forces (computed once)
    const double rho = file_info_.GetRhoVal();
    cached_rho_times_g_ = rho * gravitational_acceleration_.norm();
    
    cached_stiffness_matrices_.resize(num_bodies_);
    cached_buoyancy_forces_.resize(num_bodies_);
    
    for (int b = 0; b < num_bodies_; ++b) {
        // Cache stiffness matrix (6x6)
        cached_stiffness_matrices_[b] = file_info_.GetLinMatrix(b);
        
        // Cache buoyancy force at equilibrium
        const double displaced_volume = file_info_.GetDispVolVal(b);
        cached_buoyancy_forces_[b] = rho * (-gravitational_acceleration_) * displaced_volume;
    }
}

void HydrostaticsComponent::Compute(const SystemState& state,
                                double time,
                                BodyForces& inout_forces) {
    // Internal consistency check: state and forces must match expected body count.
    // This can fail if HydroSystem is misused (integration bug, not user config error).
    if (static_cast<int>(state.bodies.size()) != num_bodies_) {
        throw std::runtime_error(
            "HydrostaticsComponent::Compute: state.bodies.size() mismatch (expected " +
            std::to_string(num_bodies_) + ", got " + std::to_string(state.bodies.size()) + ")");
    }
    if (static_cast<int>(inout_forces.size()) != num_bodies_) {
        throw std::runtime_error(
            "HydrostaticsComponent::Compute: inout_forces.size() mismatch (expected " +
            std::to_string(num_bodies_) + ", got " + std::to_string(inout_forces.size()) + ")");
    }

    for (int b : active_bodies_) {
        const auto& body_state = state.bodies[b];
        const int body_offset = kDofPerBody * b;
        const double* const body_equilibrium = &equilibrium_[body_offset];

        // Current pose (from SystemState)
        const Eigen::Vector3d& position_world = body_state.position;
        const Eigen::Vector3d& rotation_rpy = body_state.orientation_rpy;

        // 6-DOF displacement from equilibrium (translation xyz, rotation rpy)
        Eigen::Matrix<double, kDofPerBody, 1> displacement_from_equilibrium;
        displacement_from_equilibrium[0] = position_world[0] - body_equilibrium[0];
        displacement_from_equilibrium[1] = position_world[1] - body_equilibrium[1];
        displacement_from_equilibrium[2] = position_world[2] - body_equilibrium[2];
        displacement_from_equilibrium[3] = rotation_rpy[0] - body_equilibrium[3];
        displacement_from_equilibrium[4] = rotation_rpy[1] - body_equilibrium[4];
        displacement_from_equilibrium[5] = rotation_rpy[2] - body_equilibrium[5];

        // Linear hydrostatic restoring force/torque (use cached stiffness matrix and rho*g)
        const Eigen::Matrix<double, kDofPerBody, 1> restoring_force_torque =
            -cached_rho_times_g_ * (cached_stiffness_matrices_[b] * displacement_from_equilibrium);
        inout_forces[b].force  += restoring_force_torque.head<3>();
        inout_forces[b].moment += restoring_force_torque.tail<3>();

        // Buoyancy force at equilibrium (use cached value)
        inout_forces[b].force += cached_buoyancy_forces_[b];

        // Buoyancy-induced moment about CG: (r_CB - r_CG) x buoyancy
        const int rotation_offset = kDofLinOrRot * b;
        const Eigen::Vector3d cg_to_cb(
            cb_minus_cg_[rotation_offset + 0],
            cb_minus_cg_[rotation_offset + 1],
            cb_minus_cg_[rotation_offset + 2]
        );
        const Eigen::Vector3d buoyancy_moment = cg_to_cb.cross(cached_buoyancy_forces_[b]);
        inout_forces[b].moment += buoyancy_moment;
    }
}

}  // namespace seastack::hydro

