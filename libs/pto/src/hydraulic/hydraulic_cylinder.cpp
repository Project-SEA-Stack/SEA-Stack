#include <seastack/pto/hydraulic/hydraulic_cylinder.h>

namespace seastack {
namespace pto {

HydraulicCylinder::HydraulicCylinder(const HydraulicCylinderParams& params)
    : params_(params) {}

double HydraulicCylinder::ComputeFlow(double piston_velocity) const {
    return params_.piston_area * piston_velocity;
}

double HydraulicCylinder::ComputeForce(double pressure_a,
                                       double pressure_b) const {
    return (pressure_a - pressure_b) * params_.piston_area;
}

}  // namespace pto
}  // namespace seastack
