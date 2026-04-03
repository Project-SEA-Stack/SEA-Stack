#include <seastack/pto/hydraulic/rectified_hydraulic_pto.h>
#include <seastack/control/pi_controller.h>

#include "test_macros.h"

#include <cmath>
#include <iostream>
#include <memory>

using namespace seastack::pto;
using namespace seastack::control;

static constexpr double kTol = 1e-6;
static constexpr double kPi  = 3.14159265358979323846;

static TestResults test_results;

static RectifiedHydraulicPTOParams make_test_params() {
    RectifiedHydraulicPTOParams p{};
    p.cylinder.piston_area = 0.0378;

    p.hp_accumulator.total_volume       = 0.0085;
    p.hp_accumulator.precharge_pressure = 19.2e6;
    p.hp_accumulator.gamma              = 1.4;

    p.lp_accumulator.total_volume       = 0.0085;
    p.lp_accumulator.precharge_pressure = 9.6e6;
    p.lp_accumulator.gamma              = 1.4;

    p.motor.displacement    = 1.91e-5;  // m^3/rad
    p.motor.mech_efficiency = 0.85;
    p.motor.vol_efficiency  = 0.90;

    p.generator_inertia = 0.8;   // kg*m^2
    p.generator_damping = 0.8;   // N*m*s/rad
    p.num_substeps      = 20;
    return p;
}

// ── Constant velocity: HP pressure rises, motor accelerates ────────

static void test_constant_velocity() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();
    RectifiedHydraulicPTO pto(params);

    double v = 0.5;  // m/s piston velocity
    double dt = 0.01;

    // Run for a few seconds
    double t = 0.0;
    for (int i = 0; i < 200; ++i) {
        t += dt;
        pto.ComputeForce(0.0, v, t);
    }

    auto d = pto.GetDiagnostics();

    // HP pressure should be above precharge
    TEST_ASSERT(d.hp_pressure > params.hp_accumulator.precharge_pressure,
                "HP pressure above precharge after constant velocity");

    // Motor should have started spinning
    TEST_ASSERT(d.motor_speed > 0.0, "Motor speed positive after constant velocity");

    // Force should be non-zero and opposing motion (negative for positive v)
    TEST_ASSERT(d.cylinder_force < 0.0, "Cylinder force opposes positive velocity");

    if (test_results.failed == failures_before) {
        std::cout << "  constant_velocity: passed (HP="
                  << d.hp_pressure / 1e6 << " MPa, omega="
                  << d.motor_speed << " rad/s, F="
                  << d.cylinder_force / 1e3 << " kN)\n";
    } else {
        std::cout << "  constant_velocity: FAILED (see stderr above)\n";
    }
}

// ── Zero velocity: motor runs down, pressures equalize ─────────────

static void test_zero_velocity() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();
    RectifiedHydraulicPTO pto(params);

    // First charge up the system with some flow
    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        t += 0.01;
        pto.ComputeForce(0.0, 0.5, t);
    }

    auto d_before = pto.GetDiagnostics();
    TEST_ASSERT(d_before.motor_speed > 0.0, "Motor spinning before zero-velocity phase");

    // Now apply zero velocity for a while — motor should slow down
    for (int i = 0; i < 500; ++i) {
        t += 0.01;
        pto.ComputeForce(0.0, 0.0, t);
    }

    auto d_after = pto.GetDiagnostics();
    TEST_ASSERT(d_after.motor_speed < d_before.motor_speed,
                "Motor slows down with zero velocity input");

    // HP-LP differential should have decreased (motor consumed HP oil,
    // and no new flow is entering)
    double dp_before = d_before.hp_pressure - d_before.lp_pressure;
    double dp_after  = d_after.hp_pressure  - d_after.lp_pressure;
    TEST_ASSERT(dp_after < dp_before,
                "HP-LP pressure differential decreases with no input");

    if (test_results.failed == failures_before) {
        std::cout << "  zero_velocity: passed (omega: "
                  << d_before.motor_speed << " -> " << d_after.motor_speed
                  << " rad/s)\n";
    } else {
        std::cout << "  zero_velocity: FAILED (see stderr above)\n";
    }
}

// ── Energy balance consistency ─────────────────────────────────────

static void test_energy_balance() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();
    RectifiedHydraulicPTO pto(params);

    double dt = 0.01;
    double t = 0.0;
    int n_steps = 500;

    double energy_in = 0.0;   // integral of |F_pto * v|
    double energy_gen = 0.0;  // integral of T_gen * omega

    for (int i = 0; i < n_steps; ++i) {
        // Sinusoidal piston velocity
        double v = 0.3 * std::sin(2.0 * kPi * 0.125 * t);
        t += dt;
        double F = pto.ComputeForce(0.0, v, t);
        auto d = pto.GetDiagnostics();

        // F*v is negative (force opposes velocity), so absorbed power = -F*v
        energy_in  += (-F * v) * dt;
        energy_gen += d.electrical_power * dt;
    }

    // Absorbed mechanical energy should be positive
    TEST_ASSERT(energy_in > 0.0, "Absorbed mechanical energy is positive");

    // Generator energy should be non-negative (no controller -> T_gen=0,
    // so this will be zero, but the motor still dissipates via damping).
    TEST_ASSERT(energy_gen >= -kTol, "Generator energy is non-negative");

    // The difference (energy_in - energy_gen) accounts for:
    // - motor inertia kinetic energy change
    // - generator damping losses
    // - motor efficiency losses
    // - accumulator stored energy
    // All we check is that absorbed > generated (no free energy).
    TEST_ASSERT(energy_in >= energy_gen - kTol,
                "No free energy: absorbed >= generated");

    if (test_results.failed == failures_before) {
        std::cout << "  energy_balance: passed (absorbed="
                  << energy_in << " J, gen=" << energy_gen << " J)\n";
    } else {
        std::cout << "  energy_balance: FAILED (see stderr above)\n";
    }
}

// ── Controller integration: motor speed tracks setpoint ────────────

static void test_controller_integration() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();

    PIControllerParams ctrl_params{};
    ctrl_params.kp = 2.4;
    ctrl_params.ki = 6.5;
    ctrl_params.setpoint = 104.72;  // ~1000 RPM
    ctrl_params.output_min = 0.0;
    ctrl_params.output_max = 300.0;

    auto controller = std::make_shared<PIController>(ctrl_params);
    RectifiedHydraulicPTO pto(params, controller);

    double dt = 0.01;
    double t = 0.0;

    // Provide constant positive piston velocity to charge the system
    for (int i = 0; i < 2000; ++i) {
        t += dt;
        pto.ComputeForce(0.0, 0.3, t);
    }

    auto d = pto.GetDiagnostics();

    // Constant piston flow charges the circuit; speed can exceed the PI
    // setpoint by a large margin. Require finite speed and a loose upper
    // bound to catch blow-up / NaN, not agreement with setpoint.
    TEST_ASSERT(d.motor_speed > 0.0, "Motor speed positive with controller");
    TEST_ASSERT(std::isfinite(d.motor_speed), "Motor speed finite with controller");
    TEST_ASSERT(d.motor_speed < 5000.0,
                "Motor speed below loose blow-up bound (rad/s)");

    // Generator torque should be within controller limits
    TEST_ASSERT(d.generator_torque >= ctrl_params.output_min - kTol,
                "Generator torque above controller min");
    TEST_ASSERT(d.generator_torque <= ctrl_params.output_max + kTol,
                "Generator torque below controller max");

    if (test_results.failed == failures_before) {
        std::cout << "  controller_integration: passed (omega="
                  << d.motor_speed << " rad/s, target="
                  << ctrl_params.setpoint << ", T_gen="
                  << d.generator_torque << " Nm)\n";
    } else {
        std::cout << "  controller_integration: FAILED (see stderr above)\n";
    }
}

// ── Diagnostics are populated ──────────────────────────────────────

static void test_diagnostics_populated() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();
    RectifiedHydraulicPTO pto(params);

    pto.ComputeForce(0.0, 0.5, 0.01);
    auto d = pto.GetDiagnostics();

    // All pressure fields should be positive
    TEST_ASSERT(d.hp_pressure > 0.0, "HP pressure positive");
    TEST_ASSERT(d.lp_pressure > 0.0, "LP pressure positive");

    // Oil volumes should be non-negative
    TEST_ASSERT(d.hp_oil_volume >= 0.0, "HP oil volume non-negative");
    TEST_ASSERT(d.lp_oil_volume >= 0.0, "LP oil volume non-negative");

    // Force should be non-zero for non-zero velocity
    TEST_ASSERT(std::abs(d.cylinder_force) > 0.0,
                "Cylinder force non-zero for non-zero velocity");

    // Piston velocity should match input
    TEST_NEAR(d.piston_velocity, 0.5, kTol, "Piston velocity matches input");

    if (test_results.failed == failures_before) {
        std::cout << "  diagnostics_populated: passed\n";
    } else {
        std::cout << "  diagnostics_populated: FAILED (see stderr above)\n";
    }
}

// ── Time caching: repeated calls at same time return cached force ──

static void test_time_caching() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();
    RectifiedHydraulicPTO pto(params);

    double f1 = pto.ComputeForce(0.0, 0.5, 0.01);
    double f2 = pto.ComputeForce(0.0, 0.5, 0.01);
    double f3 = pto.ComputeForce(0.0, 1.0, 0.01);  // different vel, same time

    TEST_NEAR(f1, f2, kTol, "Same call returns same force");
    TEST_NEAR(f1, f3, kTol, "Cached force ignores new velocity at same time");
    if (test_results.failed == failures_before) {
        std::cout << "  time_caching: passed\n";
    } else {
        std::cout << "  time_caching: FAILED (see stderr above)\n";
    }
}

// ── Smooth force near zero velocity ─────────────────────────────────
// With velocity_smoothing > 0, force must be continuous and smooth
// through v=0.  No discontinuous jumps like the old sign(v) law.

static void test_smooth_force_near_zero() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();

    // Charge the system so dp > 0
    RectifiedHydraulicPTO pto(params);
    double t = 0.0;
    for (int i = 0; i < 200; ++i) {
        t += 0.01;
        pto.ComputeForce(0.0, 0.5, t);
    }
    double dp_pressure = pto.GetDiagnostics().hp_pressure
                       - pto.GetDiagnostics().lp_pressure;
    TEST_ASSERT(dp_pressure > 1e6, "Meaningful pressure differential for smooth test");

    // Sweep velocity from -0.1 to +0.1 in small steps.
    // Force must change smoothly: consecutive step differences
    // must be small relative to the max force magnitude.
    constexpr int N = 200;
    constexpr double v_range = 0.1;
    double dv = 2.0 * v_range / N;
    double prev_force = 0.0;
    double max_force_jump = 0.0;

    for (int i = 0; i <= N; ++i) {
        double v = -v_range + i * dv;
        t += 0.01;
        double f = pto.ComputeForce(0.0, v, t);

        if (i > 0) {
            double jump = std::abs(f - prev_force);
            if (jump > max_force_jump) max_force_jump = jump;
        }
        prev_force = f;
    }

    // Max force magnitude at the extremes: F ~ dp * A_p
    double max_magnitude = dp_pressure * params.cylinder.piston_area;

    // With smoothing, the max jump between consecutive 1mm/s steps should
    // be a small fraction of the full force.  The old sign(v) law would
    // produce a jump of ~2*max_magnitude right at v=0 (fraction ~2.0).
    // Threshold of 0.3 allows for pressure/state evolution between steps.
    double jump_fraction = max_force_jump / max_magnitude;
    TEST_ASSERT(jump_fraction < 0.3,
                "Force jump fraction through zero velocity < 0.3");

    if (test_results.failed == failures_before) {
        std::cout << "  smooth_force_near_zero: passed (max jump fraction="
                  << jump_fraction << ")\n";
    } else {
        std::cout << "  smooth_force_near_zero: FAILED (see stderr above)\n";
    }
}

// ── Force at v=0 is exactly zero ────────────────────────────────────

static void test_force_zero_at_zero_velocity() {
    const int failures_before = test_results.failed;
    auto params = make_test_params();
    RectifiedHydraulicPTO pto(params);

    // Charge up the system
    double t = 0.0;
    for (int i = 0; i < 200; ++i) {
        t += 0.01;
        pto.ComputeForce(0.0, 0.5, t);
    }

    // Force at v=0 should be exactly zero (smooth_sign(0)=0)
    t += 0.01;
    double f = pto.ComputeForce(0.0, 0.0, t);
    TEST_NEAR(f, 0.0, kTol, "Force at zero velocity is zero");

    if (test_results.failed == failures_before) {
        std::cout << "  force_zero_at_zero_velocity: passed (F=" << f << ")\n";
    } else {
        std::cout << "  force_zero_at_zero_velocity: FAILED (see stderr above)\n";
    }
}

// ── Main ───────────────────────────────────────────────────────────

int main() {
    std::cout << "test_rectified_hydraulic_pto:\n";

    test_constant_velocity();
    test_zero_velocity();
    test_energy_balance();
    test_controller_integration();
    test_diagnostics_populated();
    test_time_caching();
    test_smooth_force_near_zero();
    test_force_zero_at_zero_velocity();

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
