#include <seastack/pto/linear_pto.h>

namespace seastack {
namespace pto {

LinearPTO::LinearPTO(double stiffness, double damping)
    : stiffness_(stiffness), damping_(damping) {}

double LinearPTO::ComputeForce(double displacement,
                                double velocity,
                                double /*time*/) {
    return -stiffness_ * displacement - damping_ * velocity;
}

}  // namespace pto
}  // namespace seastack
