#include <seastack/control/pi_controller.h>

#include "test_macros.h"

#include <iostream>

using namespace seastack::control;

static constexpr double kTol = 1e-10;

static TestResults test_results;

// ── Basic proportional response ────────────────────────────────────

static void test_proportional_only() {
    PIControllerParams p{};
    p.kp = 5.0;
    p.ki = 0.0;
    p.setpoint = 100.0;
    p.output_min = -1000.0;
    p.output_max = 1000.0;
    PIController ctrl(p);

    // measurement=110 -> error=+10 -> output = 5*10 = 50
    double out = ctrl.Compute(110.0, 0.0);
    TEST_NEAR(out, 50.0, kTol, "Proportional response to positive error");

    // measurement=90 -> error=-10 -> output = 5*(-10) = -50
    out = ctrl.Compute(90.0, 1.0);
    TEST_NEAR(out, -50.0, kTol, "Proportional response to negative error");
    std::cout << "  proportional_only: passed\n";
}

// ── Integral accumulation ──────────────────────────────────────────

static void test_integral_growth() {
    PIControllerParams p{};
    p.kp = 0.0;
    p.ki = 2.0;
    p.setpoint = 0.0;
    p.output_min = -1000.0;
    p.output_max = 1000.0;
    PIController ctrl(p);

    // First call at t=0: no dt yet, integral stays 0
    double out = ctrl.Compute(10.0, 0.0);
    TEST_NEAR(out, 0.0, kTol, "First call at t=0, output is zero");

    // Second call at t=1: dt=1, error=10 -> integral += 10*1 = 10
    // output = ki * integral = 2 * 10 = 20
    out = ctrl.Compute(10.0, 1.0);
    TEST_NEAR(out, 20.0, kTol, "Integral output after dt=1");
    TEST_NEAR(ctrl.integral(), 10.0, kTol, "Integral accumulation after dt=1");

    // Third call at t=2: dt=1, error=10 -> integral += 10 = 20
    // output = 2 * 20 = 40
    out = ctrl.Compute(10.0, 2.0);
    TEST_NEAR(out, 40.0, kTol, "Integral output after dt=2");
    std::cout << "  integral_growth: passed\n";
}

// ── Combined PI ────────────────────────────────────────────────────

static void test_pi_combined() {
    PIControllerParams p{};
    p.kp = 3.0;
    p.ki = 1.0;
    p.setpoint = 50.0;
    p.output_min = -1000.0;
    p.output_max = 1000.0;
    PIController ctrl(p);

    // t=0: error=10, integral=0  -> output = 3*10 + 1*0 = 30
    double out = ctrl.Compute(60.0, 0.0);
    TEST_NEAR(out, 30.0, kTol, "PI combined output at t=0");

    // t=0.5: dt=0.5, error=10 -> integral = 10*0.5 = 5
    // output = 3*10 + 1*5 = 35
    out = ctrl.Compute(60.0, 0.5);
    TEST_NEAR(out, 35.0, kTol, "PI combined output at t=0.5");
    std::cout << "  pi_combined: passed\n";
}

// ── Output saturation ──────────────────────────────────────────────

static void test_output_clamping() {
    PIControllerParams p{};
    p.kp = 100.0;
    p.ki = 0.0;
    p.setpoint = 0.0;
    p.output_min = -50.0;
    p.output_max = 200.0;
    PIController ctrl(p);

    // error=10 -> raw=1000, clamped to 200
    double out = ctrl.Compute(10.0, 0.0);
    TEST_NEAR(out, 200.0, kTol, "Output clamped to max");

    // error=-10 -> raw=-1000, clamped to -50
    out = ctrl.Compute(-10.0, 1.0);
    TEST_NEAR(out, -50.0, kTol, "Output clamped to min");
    std::cout << "  output_clamping: passed\n";
}

// ── Anti-windup: integral should not grow when saturated ───────────

static void test_anti_windup() {
    PIControllerParams p{};
    p.kp = 0.0;
    p.ki = 10.0;
    p.setpoint = 0.0;
    p.output_min = 0.0;
    p.output_max = 100.0;
    PIController ctrl(p);

    // Apply constant positive error (measurement=5) for many steps.
    // Output will saturate at 100.  Integral should stop growing once
    // saturated, because error is positive and output is above max.
    ctrl.Compute(5.0, 0.0);   // first call, no dt
    for (int i = 1; i <= 100; ++i) {
        ctrl.Compute(5.0, static_cast<double>(i));
    }
    double out = ctrl.Compute(5.0, 101.0);
    TEST_NEAR(out, 100.0, kTol, "Output saturated at max after windup");

    // Integral should be bounded around 10 (= 100/ki) rather than
    // having grown to ~500 (= 5*100 unclamped).
    TEST_ASSERT(ctrl.integral() < 15.0, "Integral bounded by anti-windup");
    std::cout << "  anti_windup: passed\n";
}

// ── Anti-windup: integral grows when error reduces saturation ──────

static void test_anti_windup_recovery() {
    PIControllerParams p{};
    p.kp = 0.0;
    p.ki = 10.0;
    p.setpoint = 0.0;
    p.output_min = 0.0;
    p.output_max = 100.0;
    PIController ctrl(p);

    // Saturate the controller with positive error
    ctrl.Compute(5.0, 0.0);
    for (int i = 1; i <= 20; ++i) {
        ctrl.Compute(5.0, static_cast<double>(i));
    }

    // Now apply negative error (should de-saturate).
    // Integral should be allowed to decrease (error is negative,
    // output is saturated high, so error reduces saturation).
    double integral_before = ctrl.integral();
    ctrl.Compute(-5.0, 21.0);
    TEST_ASSERT(ctrl.integral() < integral_before,
                "Integral decreases on error reversal");
    std::cout << "  anti_windup_recovery: passed\n";
}

// ── Reset ──────────────────────────────────────────────────────────

static void test_reset() {
    PIControllerParams p{};
    p.kp = 1.0;
    p.ki = 1.0;
    p.setpoint = 0.0;
    p.output_min = -1000.0;
    p.output_max = 1000.0;
    PIController ctrl(p);

    ctrl.Compute(10.0, 0.0);
    ctrl.Compute(10.0, 1.0);
    TEST_ASSERT(ctrl.integral() > 0.0, "Integral positive before reset");

    ctrl.Reset();
    TEST_NEAR(ctrl.integral(), 0.0, kTol, "Integral zero after reset");
    TEST_NEAR(ctrl.last_error(), 0.0, kTol, "Last error zero after reset");

    // After reset, first call should behave like fresh controller
    double out = ctrl.Compute(5.0, 0.0);
    TEST_NEAR(out, 5.0, kTol, "Output correct after reset");
    std::cout << "  reset: passed\n";
}

// ── Zero crossing ──────────────────────────────────────────────────

static void test_zero_crossing() {
    PIControllerParams p{};
    p.kp = 1.0;
    p.ki = 1.0;
    p.setpoint = 10.0;
    p.output_min = -1000.0;
    p.output_max = 1000.0;
    PIController ctrl(p);

    // Start above setpoint
    ctrl.Compute(15.0, 0.0);  // error = +5
    ctrl.Compute(15.0, 1.0);  // integral += 5

    // Cross below setpoint
    ctrl.Compute(5.0, 2.0);   // error = -5, integral += -5 -> integral = 0
    TEST_NEAR(ctrl.integral(), 0.0, kTol, "Integral zero after error sign change");
    std::cout << "  zero_crossing: passed\n";
}

// ── Time handling: same time should not accumulate ─────────────────

static void test_same_time() {
    PIControllerParams p{};
    p.kp = 0.0;
    p.ki = 1.0;
    p.setpoint = 0.0;
    p.output_min = -1000.0;
    p.output_max = 1000.0;
    PIController ctrl(p);

    ctrl.Compute(10.0, 0.0);
    ctrl.Compute(10.0, 1.0);
    double integral_at_1 = ctrl.integral();

    // Calling again at same time should not change integral
    ctrl.Compute(10.0, 1.0);
    TEST_NEAR(ctrl.integral(), integral_at_1, kTol,
              "Integral unchanged at same time");
    std::cout << "  same_time: passed\n";
}

// ── Main ───────────────────────────────────────────────────────────

int main() {
    std::cout << "test_pi_controller:\n";

    test_proportional_only();
    test_integral_growth();
    test_pi_combined();
    test_output_clamping();
    test_anti_windup();
    test_anti_windup_recovery();
    test_reset();
    test_zero_crossing();
    test_same_time();

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
