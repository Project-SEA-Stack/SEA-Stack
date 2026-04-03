#include <seastack/pto/hydraulic/hydraulic_cylinder.h>
#include <seastack/pto/hydraulic/hydraulic_accumulator.h>
#include <seastack/pto/hydraulic/hydraulic_motor.h>

#include "test_macros.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace seastack::pto;

static constexpr double kTol = 1e-10;

static TestResults test_results;

// ── HydraulicCylinder ──────────────────────────────────────────────

static void test_cylinder_flow() {
    HydraulicCylinderParams p{};
    p.piston_area = 0.05;  // m^2
    HydraulicCylinder cyl(p);

    TEST_NEAR(cyl.ComputeFlow(1.0), 0.05, kTol, "Cylinder flow at v=1");
    TEST_NEAR(cyl.ComputeFlow(-2.0), -0.10, kTol, "Cylinder flow at v=-2");
    TEST_NEAR(cyl.ComputeFlow(0.0), 0.0, kTol, "Cylinder flow at v=0");
    std::cout << "  cylinder_flow: passed\n";
}

static void test_cylinder_force() {
    HydraulicCylinderParams p{};
    p.piston_area = 0.05;
    HydraulicCylinder cyl(p);

    // F = (P_a - P_b) * A_p
    TEST_NEAR(cyl.ComputeForce(20e6, 10e6), 500000.0, kTol,
              "Cylinder force positive dp");
    TEST_NEAR(cyl.ComputeForce(10e6, 20e6), -500000.0, kTol,
              "Cylinder force negative dp");
    TEST_NEAR(cyl.ComputeForce(15e6, 15e6), 0.0, kTol,
              "Cylinder force zero dp");
    std::cout << "  cylinder_force: passed\n";
}

static void test_cylinder_accessor() {
    HydraulicCylinderParams p{};
    p.piston_area = 0.0378;
    HydraulicCylinder cyl(p);
    TEST_NEAR(cyl.piston_area(), 0.0378, kTol, "Piston area getter");
    std::cout << "  cylinder_accessor: passed\n";
}

// ── HydraulicAccumulator ───────────────────────────────────────────

static void test_accumulator_precharge() {
    AccumulatorParams p{};
    p.total_volume = 0.01;        // 10 L
    p.precharge_pressure = 10e6;  // 10 MPa
    p.gamma = 1.4;
    HydraulicAccumulator acc(p);

    // At precharge (oil_volume = 0), pressure = P_0
    TEST_NEAR(acc.OilVolume(), 0.0, kTol, "Oil volume at precharge");
    TEST_NEAR(acc.Pressure(), 10e6, kTol, "Pressure at precharge");
    std::cout << "  accumulator_precharge: passed\n";
}

static void test_accumulator_pressure_volume() {
    AccumulatorParams p{};
    p.total_volume = 0.01;
    p.precharge_pressure = 10e6;
    p.gamma = 1.4;
    HydraulicAccumulator acc(p);

    // Add some oil: V_gas shrinks, pressure rises
    // P = P_0 * (V_total / V_gas)^gamma
    double oil = 0.005;  // half the volume
    acc.SetOilVolume(oil);
    TEST_NEAR(acc.OilVolume(), oil, kTol, "Oil volume after set");

    double v_gas = p.total_volume - oil;
    double expected_p = p.precharge_pressure * std::pow(p.total_volume / v_gas, p.gamma);
    TEST_NEAR(acc.Pressure(), expected_p, 1.0, "Pressure after adding oil");

    // PressureForOilVolume should give the same result
    TEST_NEAR(acc.PressureForOilVolume(oil), expected_p, 1.0,
              "PressureForOilVolume matches Pressure");
    std::cout << "  accumulator_pressure_volume: passed\n";
}

static void test_accumulator_polytropic_invariant() {
    // Verify P * V_gas^gamma = const for several oil volumes
    AccumulatorParams p{};
    p.total_volume = 0.0085;
    p.precharge_pressure = 19.2e6;
    p.gamma = 1.4;
    HydraulicAccumulator acc(p);

    double pv_ref = p.precharge_pressure * std::pow(p.total_volume, p.gamma);
    double pv_tol = 1e-10 * pv_ref;

    double test_volumes[] = {0.0, 0.001, 0.003, 0.005, 0.007};
    for (double v_oil : test_volumes) {
        double pressure = acc.PressureForOilVolume(v_oil);
        double v_gas = p.total_volume - v_oil;
        double pv = pressure * std::pow(v_gas, p.gamma);
        TEST_NEAR(pv, pv_ref, pv_tol, "Polytropic invariant P*V^gamma");
    }
    std::cout << "  accumulator_polytropic_invariant: passed\n";
}

static void test_accumulator_clamping() {
    AccumulatorParams p{};
    p.total_volume = 0.01;
    p.precharge_pressure = 10e6;
    p.gamma = 1.4;
    HydraulicAccumulator acc(p);

    // Negative oil volume should be clamped to 0
    acc.SetOilVolume(-1.0);
    TEST_NEAR(acc.OilVolume(), 0.0, kTol, "Negative oil volume clamped to 0");

    // Excessive oil volume should be clamped to MaxOilVolume
    acc.SetOilVolume(100.0);
    TEST_NEAR(acc.OilVolume(), acc.MaxOilVolume(), kTol,
              "Excessive oil volume clamped to max");
    TEST_ASSERT(acc.OilVolume() < p.total_volume,
                "Clamped oil volume less than total volume");
    std::cout << "  accumulator_clamping: passed\n";
}

static void test_accumulator_invalid_params() {
    bool caught = false;
    try {
        AccumulatorParams p{};
        p.total_volume = -1.0;
        p.precharge_pressure = 10e6;
        p.gamma = 1.4;
        HydraulicAccumulator acc(p);
    } catch (const std::invalid_argument&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Invalid accumulator params throw");
    std::cout << "  accumulator_invalid_params: passed\n";
}

// ── HydraulicMotor ─────────────────────────────────────────────────

static void test_motor_flow() {
    HydraulicMotorParams p{};
    p.displacement = 2e-5;     // m^3/rad
    p.mech_efficiency = 0.85;
    p.vol_efficiency = 0.90;
    HydraulicMotor motor(p);

    // Q = D_m * omega * eta_vol
    double omega = 100.0;  // rad/s
    double expected = 2e-5 * 100.0 * 0.90;
    TEST_NEAR(motor.ComputeFlow(omega), expected, kTol, "Motor flow at omega=100");

    // Zero speed -> zero flow
    TEST_NEAR(motor.ComputeFlow(0.0), 0.0, kTol, "Motor flow at omega=0");
    std::cout << "  motor_flow: passed\n";
}

static void test_motor_torque() {
    HydraulicMotorParams p{};
    p.displacement = 2e-5;
    p.mech_efficiency = 0.85;
    p.vol_efficiency = 0.90;
    HydraulicMotor motor(p);

    // T = D_m * delta_P * eta_mech
    double dp = 10e6;  // 10 MPa
    double expected = 2e-5 * 10e6 * 0.85;
    TEST_NEAR(motor.ComputeTorque(dp), expected, kTol, "Motor torque at dp=10 MPa");

    // Zero pressure -> zero torque
    TEST_NEAR(motor.ComputeTorque(0.0), 0.0, kTol, "Motor torque at dp=0");
    std::cout << "  motor_torque: passed\n";
}

static void test_motor_accessor() {
    HydraulicMotorParams p{};
    p.displacement = 1.91e-5;
    p.mech_efficiency = 0.85;
    p.vol_efficiency = 0.90;
    HydraulicMotor motor(p);
    TEST_NEAR(motor.displacement(), 1.91e-5, kTol, "Motor displacement getter");
    std::cout << "  motor_accessor: passed\n";
}

// ── Main ───────────────────────────────────────────────────────────

int main() {
    std::cout << "test_hydraulic_components:\n";

    test_cylinder_flow();
    test_cylinder_force();
    test_cylinder_accessor();

    test_accumulator_precharge();
    test_accumulator_pressure_volume();
    test_accumulator_polytropic_invariant();
    test_accumulator_clamping();
    test_accumulator_invalid_params();

    test_motor_flow();
    test_motor_torque();
    test_motor_accessor();

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
