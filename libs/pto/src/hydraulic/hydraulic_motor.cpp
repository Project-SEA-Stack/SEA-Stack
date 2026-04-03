#include <seastack/pto/hydraulic/hydraulic_motor.h>

namespace seastack {
namespace pto {

HydraulicMotor::HydraulicMotor(const HydraulicMotorParams& params)
    : params_(params) {}

double HydraulicMotor::ComputeFlow(double angular_velocity) const {
    return params_.displacement * angular_velocity * params_.vol_efficiency;
}

double HydraulicMotor::ComputeTorque(double pressure_diff) const {
    return params_.displacement * pressure_diff * params_.mech_efficiency;
}

}  // namespace pto
}  // namespace seastack
