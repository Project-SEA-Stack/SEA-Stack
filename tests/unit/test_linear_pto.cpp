#include <seastack/pto/linear_pto.h>

#include "test_macros.h"

int main() {
    TestResults test_results;
    constexpr double tol = 1e-12;

    seastack::pto::LinearPTO pto(500.0, 50.0);

    // At rest: F = -k*0 - c*0 = 0
    TEST_NEAR(pto.ComputeForce(0.0, 0.0, 0.0), 0.0, tol, "Force at rest is zero");

    // Pure displacement: F = -k*x = -500 * 0.1 = -50
    TEST_NEAR(pto.ComputeForce(0.1, 0.0, 0.0), -50.0, tol, "Pure displacement force");

    // Pure velocity: F = -c*v = -50 * 2.0 = -100
    TEST_NEAR(pto.ComputeForce(0.0, 2.0, 0.0), -100.0, tol, "Pure velocity force");

    // Combined: F = -500*0.3 - 50*1.5 = -150 - 75 = -225
    TEST_NEAR(pto.ComputeForce(0.3, 1.5, 0.0), -225.0, tol, "Combined displacement + velocity force");

    // Negative displacement/velocity (restoring direction)
    TEST_NEAR(pto.ComputeForce(-0.2, -1.0, 0.0), 150.0, tol,
              "Negative displacement/velocity restoring force");

    // Getters
    TEST_NEAR(pto.stiffness(), 500.0, tol, "Stiffness getter");
    TEST_NEAR(pto.damping(), 50.0, tol, "Damping getter");

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
