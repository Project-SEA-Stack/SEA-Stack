/*********************************************************************
 * @file  main.cpp
 * @brief Standalone PTO + Control example — no Chrono dependency.
 *
 * Demonstrates that SEAStack::PTO and SEAStack::Control compile and
 * run independently of Chrono, HDF5, or any heavy simulation library.
 * This validates the "Raspberry Pi" use case.
 *
 * Simulates a damped mass-spring system with a linear PTO and a
 * simple proportional controller, using explicit Euler integration.
 *********************************************************************/

#include <seastack/pto/linear_pto.h>
#include <seastack/control/controller.h>

#include <cmath>
#include <iostream>
#include <iomanip>

namespace {

/// Simple proportional controller: output = gain * measurement
class ProportionalController : public seastack::control::IController {
  public:
    explicit ProportionalController(double gain) : gain_(gain) {}

    double Compute(double measurement, double /*time*/) override {
        return gain_ * measurement;
    }

  private:
    double gain_;
};

}  // namespace

int main() {
    // System parameters
    constexpr double mass = 100.0;       // [kg]
    constexpr double dt = 0.001;         // [s]
    constexpr double t_end = 10.0;       // [s]

    // PTO: spring + damper
    seastack::pto::LinearPTO pto(/*stiffness=*/500.0, /*damping=*/50.0);

    // Controller: adjusts damping command based on velocity
    ProportionalController controller(/*gain=*/80.0);

    // Initial conditions
    double x = 0.5;   // [m] initial displacement
    double v = 0.0;   // [m/s]
    double t = 0.0;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "SEA-Stack Standalone PTO + Control Example\n";
    std::cout << "==========================================\n";
    std::cout << "  mass = " << mass << " kg\n";
    std::cout << "  PTO: k=" << pto.stiffness() << " N/m, c=" << pto.damping() << " N·s/m\n";
    std::cout << "  Controller: proportional gain = 80.0\n\n";

    std::cout << std::setw(8) << "time" << std::setw(12) << "x"
              << std::setw(12) << "v" << std::setw(12) << "F_pto"
              << std::setw(12) << "F_ctrl" << "\n";

    int step = 0;
    while (t <= t_end) {
        double F_pto = pto.ComputeForce(x, v, t);
        double F_ctrl = controller.Compute(v, t);
        double F_total = F_pto - F_ctrl;
        double a = F_total / mass;

        if (step % 500 == 0) {
            std::cout << std::setw(8) << t << std::setw(12) << x
                      << std::setw(12) << v << std::setw(12) << F_pto
                      << std::setw(12) << F_ctrl << "\n";
        }

        v += a * dt;
        x += v * dt;
        t += dt;
        step++;
    }

    std::cout << "\nFinal state: x=" << x << " m, v=" << v << " m/s\n";
    bool converged = (std::abs(x) < 0.01 && std::abs(v) < 0.01);
    std::cout << "System " << (converged ? "converged" : "still oscillating") << ".\n";

    return converged ? 0 : 1;
}
