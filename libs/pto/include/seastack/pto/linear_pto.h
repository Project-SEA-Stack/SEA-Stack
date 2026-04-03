/*********************************************************************
 * @file  linear_pto.h
 * @brief Linear PTO: constant stiffness + constant damping.
 *
 * F = -k * x  -  c * v
 *
 * This is the simplest possible PTO model and serves as a reference
 * implementation for the IPTOModel interface.
 *********************************************************************/
#ifndef SEASTACK_PTO_LINEAR_PTO_H
#define SEASTACK_PTO_LINEAR_PTO_H

#include <seastack/pto/pto_model.h>

namespace seastack {
namespace pto {

class LinearPTO : public IPTOModel {
  public:
    /// @param stiffness  Spring constant k [N/m or N·m/rad]
    /// @param damping    Damping coefficient c [N·s/m or N·m·s/rad]
    LinearPTO(double stiffness, double damping);

    double ComputeForce(double displacement,
                        double velocity,
                        double time) override;

    double stiffness() const { return stiffness_; }
    double damping() const { return damping_; }

  private:
    double stiffness_;
    double damping_;
};

}  // namespace pto
}  // namespace seastack

#endif  // SEASTACK_PTO_LINEAR_PTO_H
