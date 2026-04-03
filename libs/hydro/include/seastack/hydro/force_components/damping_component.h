/*********************************************************************
 * @file  damping_component.h
 * @brief Per-DOF body damping: linear and optional quadratic terms.
 *
 * Computes a 6-DOF damping wrench in the body frame:
 *   F_i = -B_lin_i * v_i  -  B_quad_i * v_i * |v_i|
 *
 * Supplements potential-flow radiation damping with user-specified
 * viscous-like damping coefficients.  Particularly important for
 * DOFs where radiation damping is negligible (e.g. roll of slender
 * cylinders).
 *
 * World-frame velocities from BodyState are rotated into the body
 * frame before applying coefficients, and the resulting wrench is
 * rotated back to the world frame for accumulation.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_FORCE_COMPONENTS_DAMPING_COMPONENT_H
#define SEASTACK_HYDRO_FORCE_COMPONENTS_DAMPING_COMPONENT_H

#include <seastack/core/force_component.h>
#include <array>
#include <vector>

namespace seastack::hydro {

class DampingComponent : public IHydroForceComponent {
  public:
    /// @param linear_coeffs   Per-body linear damping [surge, sway, heave, roll, pitch, yaw].
    ///                        Units: N·s/m (translational) or N·m·s/rad (rotational).
    /// @param quadratic_coeffs Per-body quadratic damping (same DOF order).
    ///                        Units: N·s²/m² (translational) or N·m·s²/rad² (rotational).
    ///                        May be empty to use linear-only (backward compatible).
    DampingComponent(const std::vector<std::array<double, 6>>& linear_coeffs,
                     const std::vector<std::array<double, 6>>& quadratic_coeffs = {});

    HydroComponentType Type() const override { return HydroComponentType::kDamping; }

    void Compute(const SystemState& state,
                 double time,
                 BodyForces& inout_forces) override;

  private:
    std::vector<std::array<double, 6>> linear_coeffs_;
    std::vector<std::array<double, 6>> quadratic_coeffs_;
    int num_bodies_;
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_FORCE_COMPONENTS_DAMPING_COMPONENT_H
