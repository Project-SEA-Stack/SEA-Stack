/*********************************************************************
 * @file  hydraulic_cylinder.h
 * @brief Double-acting hydraulic cylinder model.
 *
 * Converts linear piston motion to volumetric flow and pressure
 * differential to force.  Stateless — pure geometry.
 *
 * Solver-agnostic: C++ stdlib only (no Eigen, no Chrono).
 *********************************************************************/
#ifndef SEASTACK_PTO_HYDRAULIC_CYLINDER_H
#define SEASTACK_PTO_HYDRAULIC_CYLINDER_H

namespace seastack {
namespace pto {

struct HydraulicCylinderParams {
    double piston_area;       // A_p [m^2]
    double dead_volume_a = 0; // V_dead_A [m^3] (reserved for v2 compressibility)
    double dead_volume_b = 0; // V_dead_B [m^3] (reserved for v2 compressibility)
    double bulk_modulus  = 0; // beta [Pa]   (reserved for v2 compressibility)
};

class HydraulicCylinder {
  public:
    explicit HydraulicCylinder(const HydraulicCylinderParams& params);

    /// Volumetric flow produced by piston motion: Q = A_p * v
    double ComputeFlow(double piston_velocity) const;

    /// Force from chamber pressure differential: F = (P_a - P_b) * A_p
    double ComputeForce(double pressure_a, double pressure_b) const;

    double piston_area() const { return params_.piston_area; }

  private:
    HydraulicCylinderParams params_;
};

}  // namespace pto
}  // namespace seastack

#endif  // SEASTACK_PTO_HYDRAULIC_CYLINDER_H
