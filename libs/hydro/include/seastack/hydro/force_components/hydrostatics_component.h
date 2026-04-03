/*********************************************************************
 * @file  hydrostatics_component.h
 * @brief Hydrostatics force component (restoring forces and buoyancy).
 *********************************************************************/

#ifndef SEASTACK_HYDRO_FORCE_COMPONENTS_HYDROSTATICS_COMPONENT_H
#define SEASTACK_HYDRO_FORCE_COMPONENTS_HYDROSTATICS_COMPONENT_H

#include <seastack/core/force_component.h>
#include <seastack/core/system_state.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace seastack::hydro {

class HydroData;

/**
 * @brief Hydrostatics force component (restoring stiffness + buoyancy).
 * 
 * Computes hydrostatic restoring forces from linear stiffness matrix
 * and buoyancy forces at equilibrium position.
 */
class HydrostaticsComponent : public IHydroForceComponent {
public:
    /**
     * @brief Constructor.
     * 
     * @param file_info Reference to HydroData containing stiffness matrices, equilibrium data
     * @param num_bodies Number of bodies in the system
     * @param equilibrium Equilibrium positions [6N] (xyz, rpy per body)
     * @param cb_minus_cg Center of buoyancy minus center of gravity [3N] (xyz per body)
     * @param gravitational_acceleration Gravitational acceleration vector (m/s^2)
     * @param active_bodies Body indices this component operates on (empty = all bodies)
     */
    HydrostaticsComponent(const HydroData& file_info,
                     int num_bodies,
                     const std::vector<double>& equilibrium,
                     const std::vector<double>& cb_minus_cg,
                     const Eigen::Vector3d& gravitational_acceleration,
                     const std::vector<int>& active_bodies = {});

    /**
     * @brief Get the component type.
     * @return HydroComponentType::kHydrostatics
     */
    HydroComponentType Type() const override { return HydroComponentType::kHydrostatics; }

    /**
     * @brief Compute hydrostatic force contribution.
     * 
     * Adds hydrostatic restoring forces and buoyancy forces to inout_forces.
     * Forces computed per body from displacement from equilibrium.
     * 
     * @param state Current system state (positions and orientations)
     * @param time Current simulation time (not used for hydrostatics)
     * @param inout_forces Force vector to add contribution to (one GeneralizedForce per body)
     */
    void Compute(const SystemState& state,
                double time,
                BodyForces& inout_forces) override;

private:
    static constexpr int kDofLinOrRot = 3;

    const HydroData& file_info_;
    int num_bodies_;
    std::vector<double> equilibrium_;
    std::vector<double> cb_minus_cg_;
    Eigen::Vector3d gravitational_acceleration_;
    std::vector<int> active_bodies_;  ///< Subset of bodies to operate on (empty = all)
    
    // Cached values computed once in constructor to avoid per-timestep lookups
    std::vector<Eigen::Matrix<double, kDofPerBody, kDofPerBody>> cached_stiffness_matrices_;  ///< Per-body stiffness matrices
    double cached_rho_times_g_;  ///< rho * |g| computed once
    std::vector<Eigen::Vector3d> cached_buoyancy_forces_;  ///< Per-body buoyancy force at equilibrium
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_FORCE_COMPONENTS_HYDROSTATICS_COMPONENT_H

