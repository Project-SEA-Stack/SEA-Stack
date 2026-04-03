/*********************************************************************
 * @file  test_directional_waves.cpp
 * @brief Unit tests for the component-based directional wave system.
 *
 * TEST SCENARIOS:
 *   1. ComponentSampler: regular wave produces 1 component
 *   2. ComponentSampler: long-crested irregular produces n_omega components
 *   3. ComponentSampler: directional irregular produces n_omega * n_theta components
 *   4. ComponentSampler: bimodal produces union of both partitions
 *   5. Cos2s spreading normalization integrates to 1
 *   6. LinearWaveKinematics: elevation matches analytic regular wave
 *   7. LinearDirectionalWaveField: elevation at origin matches analytic
 *   8. Directional spectrum variance conservation (Hs^2/16)
 *   9. Direction=0 produces identical results to +X propagation
 *  15. Force reconstruction: single-frequency, single-heading analytic match
 *  16. Phase sign verification: P=0 -> positive, P=pi -> negative
 *  16b. Transfer-function convention: H5/BEMIO vs ExcitationComponent, conjugation
 *  17. Two-frequency interpolation in Re/Im space
 *  18. Heading interpolation with force values
 *  19. Multi-body force independence
 *  20. n_theta=1 collapses to long-crested behavior
 *  21. 180-degree heading flip reverses surge force sign
 *  22. Old/new path equivalence: IRF-derived C/S vs freq-domain Re/Im
 *  23. BuildFromEtaFile: exact amplitude/phase recovery at Fourier frequencies
 *********************************************************************/

#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_wave_kinematics.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/excitation_transfer.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/force_components/excitation_component.h>
#include <seastack/hydro/hydro_data.h>
#include <seastack/hydro_io/h5_reader.h>
#include <seastack/core/math_constants.h>
#include <seastack/core/system_state.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

using namespace seastack::hydro;

// Forward declarations for internal wave_utilities functions used by tests.
// These are in seastack::hydro but defined in a private header.
namespace seastack::hydro {
double Cos2sSpreading(double theta, double theta_mean, double s);
double ComputeWaveNumber(double omega, double water_depth, double g,
                         double tolerance = 1e-6, int max_iterations = 100);
}  // namespace seastack::hydro

#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "FAILED: " << message << std::endl; \
            std::cerr << "  at " << __FILE__ << ":" << __LINE__ << std::endl; \
            return false; \
        } \
    } while (0)

#define TEST_NEAR(actual, expected, tol, message) \
    do { \
        double _a = (actual); double _e = (expected); double _t = (tol); \
        if (std::abs(_a - _e) > _t) { \
            std::cerr << "FAILED: " << message << std::endl; \
            std::cerr << "  expected=" << _e << " actual=" << _a \
                      << " diff=" << std::abs(_a - _e) << " tol=" << _t << std::endl; \
            return false; \
        } \
    } while (0)

// Forward declarations — definitions after MockH5Builder (uses HydroData types).
static double ExcitationForceDof(
    const std::vector<WaveComponent>& components,
    const std::vector<HydroData::RegularWaveInfo>& infos,
    const Eigen::VectorXd& h5_wave_directions,
    ExcitationInterpolation interp,
    int num_bodies,
    double ramp_duration,
    double time,
    int body_index,
    int dof_index);

static Eigen::VectorXd ExcitationForceVector(
    const std::vector<WaveComponent>& components,
    const std::vector<HydroData::RegularWaveInfo>& infos,
    const Eigen::VectorXd& h5_wave_directions,
    ExcitationInterpolation interp,
    int num_bodies,
    double ramp_duration,
    double time);

// ─────────────────────────────────────────────────────────────────────────────
// Test 1: Regular wave produces exactly 1 component
// ─────────────────────────────────────────────────────────────────────────────
static bool test_sampler_regular() {
    SeaStateDefinition def;
    def.type = "regular";
    def.amplitude = 1.5;
    def.omega = 2.0 * M_PI / 8.0;
    def.direction_deg = 30.0;
    def.phase_rad = 0.5;

    auto components = ComponentSampler::Build(def);
    TEST_ASSERT(components.size() == 1, "Regular wave should produce exactly 1 component");
    TEST_NEAR(components[0].amplitude, 1.5, 1e-12, "Regular amplitude");
    TEST_NEAR(components[0].omega, 2.0 * M_PI / 8.0, 1e-12, "Regular omega");
    TEST_NEAR(components[0].direction, 30.0 * M_PI / 180.0, 1e-12, "Regular direction");
    TEST_NEAR(components[0].phase, 0.5, 1e-12, "Regular phase");
    TEST_ASSERT(components[0].k > 0.0, "Wavenumber should be positive");

    std::cout << "  PASSED: test_sampler_regular" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2: Long-crested irregular produces n_omega components
// ─────────────────────────────────────────────────────────────────────────────
static bool test_sampler_long_crested() {
    SeaStateDefinition def;
    def.type = "irregular";
    def.n_omega = 50;
    def.n_theta = 1;
    def.seed = 42;

    SeaStatePartition p;
    p.spectrum.type = "jonswap";
    p.spectrum.Hs = 2.5;
    p.spectrum.Tp = 8.0;
    p.spreading.type = "none";
    p.spreading.mean_direction_deg = 0.0;
    def.partitions.push_back(p);

    auto components = ComponentSampler::Build(def);
    // May be slightly less than n_omega due to pruning, but should be close
    TEST_ASSERT(components.size() > 0, "Should produce components");
    TEST_ASSERT(components.size() <= 50, "Should not exceed n_omega");

    // All components should have the same direction (0 deg = 0 rad)
    for (const auto& c : components) {
        TEST_NEAR(c.direction, 0.0, 1e-12, "Long-crested: all directions should be 0");
    }

    std::cout << "  PASSED: test_sampler_long_crested (" << components.size() << " components)" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3: Directional irregular produces ~ n_omega * n_theta components
// ─────────────────────────────────────────────────────────────────────────────
static bool test_sampler_directional() {
    SeaStateDefinition def;
    def.type = "irregular";
    def.n_omega = 32;
    def.n_theta = 15;
    def.seed = 42;

    SeaStatePartition p;
    p.spectrum.type = "jonswap";
    p.spectrum.Hs = 2.5;
    p.spectrum.Tp = 8.0;
    p.spreading.type = "cos2s";
    p.spreading.mean_direction_deg = 210.0;
    p.spreading.s = 12.0;
    def.partitions.push_back(p);

    auto components = ComponentSampler::Build(def);
    TEST_ASSERT(components.size() > 0, "Should produce directional components");
    // After pruning, should be less than or equal to 32*15 = 480
    TEST_ASSERT(components.size() <= 32 * 15, "Should not exceed n_omega*n_theta");
    // But should be a reasonable fraction of the max
    TEST_ASSERT(components.size() > 100, "Should produce substantial number of components");

    std::cout << "  PASSED: test_sampler_directional (" << components.size() << " components)" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4: Bimodal produces union of both partitions
// ─────────────────────────────────────────────────────────────────────────────
static bool test_sampler_bimodal() {
    SeaStateDefinition def;
    def.type = "irregular";
    def.n_omega = 32;
    def.n_theta = 9;
    def.seed = 42;

    SeaStatePartition p1;
    p1.spectrum.type = "jonswap";
    p1.spectrum.Hs = 1.8;
    p1.spectrum.Tp = 9.5;
    p1.spreading.type = "cos2s";
    p1.spreading.mean_direction_deg = 220.0;
    p1.spreading.s = 20.0;
    def.partitions.push_back(p1);

    SeaStatePartition p2;
    p2.spectrum.type = "jonswap";
    p2.spectrum.Hs = 0.9;
    p2.spectrum.Tp = 5.0;
    p2.spreading.type = "cos2s";
    p2.spreading.mean_direction_deg = 190.0;
    p2.spreading.s = 8.0;
    def.partitions.push_back(p2);

    auto components = ComponentSampler::Build(def);
    TEST_ASSERT(components.size() > 0, "Bimodal should produce components");

    // Should have components from both partitions: different mean directions
    bool has_near_220 = false, has_near_190 = false;
    double dir_220 = 220.0 * M_PI / 180.0;
    double dir_190 = 190.0 * M_PI / 180.0;
    for (const auto& c : components) {
        if (std::abs(c.direction - dir_220) < 0.5) has_near_220 = true;
        if (std::abs(c.direction - dir_190) < 0.5) has_near_190 = true;
    }
    TEST_ASSERT(has_near_220, "Should have components near 220 deg");
    TEST_ASSERT(has_near_190, "Should have components near 190 deg");

    std::cout << "  PASSED: test_sampler_bimodal (" << components.size() << " components)" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5: Cos2s spreading normalization
// ─────────────────────────────────────────────────────────────────────────────
static bool test_cos2s_normalization() {
    // Numerical integration of D(theta) over [-pi, pi] should yield 1.
    for (double s : {2.0, 6.0, 12.0, 25.0, 50.0}) {
        int N = 10000;
        double d_theta = 2.0 * M_PI / N;
        double integral = 0.0;
        for (int i = 0; i < N; ++i) {
            double theta = -M_PI + (i + 0.5) * d_theta;
            integral += Cos2sSpreading(theta, 0.0, s) * d_theta;
        }
        TEST_NEAR(integral, 1.0, 0.005,
                  "Cos2s integral for s=" + std::to_string(s) + " should be ~1");
    }

    std::cout << "  PASSED: test_cos2s_normalization" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 6: LinearWaveKinematics elevation matches analytic
// ─────────────────────────────────────────────────────────────────────────────
static bool test_kinematics_elevation() {
    WaveComponent c;
    c.omega = 1.0;
    c.k = 0.102;  // deep water: k = omega^2/g
    c.direction = 0.0;
    c.amplitude = 2.0;
    c.phase = 0.3;

    double x = 5.0, y = 0.0, t = 2.0;
    double expected = c.amplitude * std::cos(c.k * x - c.omega * t + c.phase);
    double actual = LinearWaveKinematics::Elevation(c, x, y, t);

    TEST_NEAR(actual, expected, 1e-12, "Kinematics elevation for theta=0");

    // Now with direction
    c.direction = M_PI / 4.0;  // 45 degrees
    x = 3.0; y = 4.0;
    double kx = c.k * std::cos(c.direction);
    double ky = c.k * std::sin(c.direction);
    expected = c.amplitude * std::cos(kx * x + ky * y - c.omega * t + c.phase);
    actual = LinearWaveKinematics::Elevation(c, x, y, t);

    TEST_NEAR(actual, expected, 1e-12, "Kinematics elevation for theta=45deg");

    std::cout << "  PASSED: test_kinematics_elevation" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 7: LinearDirectionalWaveField elevation at origin
// ─────────────────────────────────────────────────────────────────────────────
static bool test_wave_field_elevation() {
    WaveComponent c;
    c.omega = 2.0 * M_PI / 8.0;
    c.k = 0.0631;
    c.direction = 0.0;
    c.amplitude = 1.0;
    c.phase = 0.0;

    auto field = LinearDirectionalWaveField({c}, 0.0);

    double t = 4.0;
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double eta = field.GetElevation(pos, t);
    double expected = c.amplitude * std::cos(-c.omega * t);

    TEST_NEAR(eta, expected, 1e-12, "Wave field elevation at origin");

    std::cout << "  PASSED: test_wave_field_elevation" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 8: Directional spectrum variance conservation
// ─────────────────────────────────────────────────────────────────────────────
static bool test_variance_conservation() {
    SeaStateDefinition def;
    def.type = "irregular";
    def.n_omega = 100;
    def.n_theta = 25;
    def.seed = 42;

    SeaStatePartition p;
    p.spectrum.type = "jonswap";
    p.spectrum.Hs = 3.0;
    p.spectrum.Tp = 10.0;
    p.spreading.type = "cos2s";
    p.spreading.mean_direction_deg = 0.0;
    p.spreading.s = 12.0;
    def.partitions.push_back(p);

    auto components = ComponentSampler::Build(def);

    // Variance = sum(a_i^2 / 2) should equal Hs^2/16 = m0
    double m0_expected = p.spectrum.Hs * p.spectrum.Hs / 16.0;
    double m0_actual = 0.0;
    for (const auto& c : components) {
        m0_actual += c.amplitude * c.amplitude / 2.0;
    }

    // Allow 15% tolerance due to discrete sampling and edge effects
    double rel_error = std::abs(m0_actual - m0_expected) / m0_expected;
    TEST_ASSERT(rel_error < 0.15,
                "Variance conservation: m0_expected=" + std::to_string(m0_expected) +
                " m0_actual=" + std::to_string(m0_actual) +
                " rel_error=" + std::to_string(rel_error));

    std::cout << "  PASSED: test_variance_conservation (rel_error=" 
              << std::fixed << std::setprecision(4) << rel_error << ")" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 9: Direction=0 matches +X propagation
// ─────────────────────────────────────────────────────────────────────────────
static bool test_direction_zero_matches_plus_x() {
    WaveComponent c;
    c.omega = 1.0;
    c.k = 0.102;
    c.direction = 0.0;  // +X
    c.amplitude = 1.5;
    c.phase = 0.7;

    double x = 10.0, y = 3.0, t = 5.0;

    // With direction=0, the y coordinate should not affect elevation
    double eta_y0 = LinearWaveKinematics::Elevation(c, x, 0.0, t);
    double eta_y3 = LinearWaveKinematics::Elevation(c, x, y, t);
    TEST_NEAR(eta_y0, eta_y3, 1e-12,
              "Direction=0: elevation should be independent of y");

    // Velocity y-component should be zero for theta=0
    auto vel = LinearWaveKinematics::Velocity(c, x, y, -1.0, t, 0.0);
    TEST_NEAR(vel.y(), 0.0, 1e-12, "Direction=0: v_y should be zero");

    std::cout << "  PASSED: test_direction_zero_matches_plus_x" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 10: Finite-depth wavenumber propagation
// ─────────────────────────────────────────────────────────────────────────────
static bool test_finite_depth_wavenumber() {
    const double depth = 50.0;
    const double g = 9.81;

    SeaStateDefinition def;
    def.type = "irregular";
    def.n_omega = 20;
    def.n_theta = 1;
    def.seed = 42;
    def.depth = depth;
    def.g = g;

    SeaStatePartition p;
    p.spectrum.type = "jonswap";
    p.spectrum.Hs = 2.0;
    p.spectrum.Tp = 10.0;
    p.spreading.type = "none";
    p.spreading.mean_direction_deg = 0.0;
    def.partitions.push_back(p);

    auto components = ComponentSampler::Build(def);
    TEST_ASSERT(!components.empty(), "Should produce components");

    for (const auto& c : components) {
        double k_deep = c.omega * c.omega / g;
        // For finite depth the wavenumber should differ from the deep-water value
        // (at least for the lower frequencies where kh is moderate).
        double kh = c.k * depth;
        if (kh < 3.0) {
            TEST_ASSERT(std::abs(c.k - k_deep) > 1e-6,
                        "Finite-depth k should differ from deep-water k for kh < 3");
        }
        // Verify dispersion relation: omega^2 = g*k*tanh(k*h)
        double lhs = c.omega * c.omega;
        double rhs = g * c.k * std::tanh(c.k * depth);
        TEST_NEAR(lhs, rhs, 1e-4, "Dispersion relation for finite depth");
    }

    std::cout << "  PASSED: test_finite_depth_wavenumber" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 11: Newton derivative correctness (dispersion relation accuracy)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_newton_dispersion_accuracy() {
    const double g = 9.81;
    struct TestCase { double omega; double depth; };
    TestCase cases[] = {
        {1.0, 5.0},    // shallow-intermediate
        {0.5, 10.0},   // intermediate
        {2.0, 20.0},   // deep-ish
        {0.8, 3.0},    // very shallow
    };

    for (const auto& tc : cases) {
        double k = ComputeWaveNumber(tc.omega, tc.depth, g);
        double lhs = tc.omega * tc.omega;
        double rhs = g * k * std::tanh(k * tc.depth);
        TEST_NEAR(lhs, rhs, 1e-8,
                  "Dispersion accuracy for omega=" + std::to_string(tc.omega) +
                  " depth=" + std::to_string(tc.depth));
    }

    std::cout << "  PASSED: test_newton_dispersion_accuracy" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 12: Heading wraparound in excitation interpolation
// ─────────────────────────────────────────────────────────────────────────────
static bool test_heading_wraparound() {
    // Build a simple wave component at 350 degrees (6.109 rad).
    WaveComponent c;
    c.omega = 1.0;
    c.k = 1.0 / 9.81;
    c.direction = 350.0 * M_PI / 180.0;
    c.amplitude = 1.0;
    c.phase = 0.0;

    auto field = LinearDirectionalWaveField({c}, 0.0);

    // Construct mock H5 data with headings every 45 degrees: 0, 45, ..., 315.
    const int n_headings = 8;
    Eigen::VectorXd headings(n_headings);
    for (int i = 0; i < n_headings; ++i) {
        headings[i] = static_cast<double>(i) * 45.0 * M_PI / 180.0;
    }

    // Build minimal RegularWaveInfo with 2 frequencies and n_headings headings.
    // Excitation magnitude = heading_index (so we can detect which headings were used).
    HydroData::RegularWaveInfo info;
    info.freq_list.resize(2);
    info.freq_list[0] = 0.5;
    info.freq_list[1] = 1.5;

    const int n_dof = 6;
    info.excitation_mag_matrix.resize(n_dof, n_headings, 2);
    info.excitation_phase_matrix.resize(n_dof, n_headings, 2);
    info.excitation_mag_matrix.setZero();
    info.excitation_phase_matrix.setZero();

    // Set mag for DOF 0 at each heading to the heading index value.
    for (int h = 0; h < n_headings; ++h) {
        for (int f = 0; f < 2; ++f) {
            info.excitation_mag_matrix(0, h, f) = static_cast<double>(h);
        }
    }

    HydroData::SimulationParameters sim;
    sim.water_depth = 0.0;
    sim.g = 9.81;

    field.SetNumBodies(1);

    // At 350 deg, the field should interpolate between heading 7 (315 deg)
    // and heading 0 (0/360 deg).  A purely clamped implementation would
    // use headings 6 and 7 (or just 7).
    // The interpolated mag for DOF 0 should be between 7.0 and 0.0,
    // weighted towards heading 0 (350 is 35 deg past 315, gap is 45 deg).
    double force_at_t0 = ExcitationForceDof(
        field.GetComponents(), {info}, headings, ExcitationInterpolation::kCartesian,
        1, 0.0, 0.0, 0, 0);

    // If heading 0 was used in the interpolation, the force should reflect
    // a blend between index 7 (=7.0) and index 0 (=0.0).
    // If clamped (no wrap), it would use indices 6 and 7 (6.0 and 7.0).
    // force magnitude should be < 6.5 if wrapping is correct.
    bool uses_wrap = (std::abs(force_at_t0) < 6.5 * c.amplitude) || (force_at_t0 == 0.0);
    TEST_ASSERT(uses_wrap,
                "Heading 350 deg should wrap between 315 and 0, not clamp to 270-315");

    std::cout << "  PASSED: test_heading_wraparound" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 13: Bimodal omega range covers both peaks
// ─────────────────────────────────────────────────────────────────────────────
static bool test_bimodal_omega_range() {
    SeaStateDefinition def;
    def.type = "irregular";
    def.n_omega = 64;
    def.n_theta = 1;
    def.seed = 42;

    // Swell partition with long period
    SeaStatePartition p1;
    p1.spectrum.type = "jonswap";
    p1.spectrum.Hs = 1.5;
    p1.spectrum.Tp = 14.0;
    p1.spreading.type = "none";
    p1.spreading.mean_direction_deg = 0.0;
    def.partitions.push_back(p1);

    // Wind sea partition with short period
    SeaStatePartition p2;
    p2.spectrum.type = "jonswap";
    p2.spectrum.Hs = 1.0;
    p2.spectrum.Tp = 5.0;
    p2.spreading.type = "none";
    p2.spreading.mean_direction_deg = 90.0;
    def.partitions.push_back(p2);

    auto components = ComponentSampler::Build(def);
    TEST_ASSERT(!components.empty(), "Should produce components");

    // The swell peak is at omega_p = 2*pi/14 ~ 0.449 rad/s.
    // omega_min should be <= 0.25 * omega_p_swell to capture the peak.
    double omega_p_swell = 2.0 * M_PI / 14.0;
    double min_omega = std::numeric_limits<double>::max();
    for (const auto& c : components) {
        min_omega = std::min(min_omega, c.omega);
    }

    TEST_ASSERT(min_omega < omega_p_swell,
                "omega_min should be below the swell peak frequency");

    std::cout << "  PASSED: test_bimodal_omega_range (min_omega="
              << min_omega << " omega_p_swell=" << omega_p_swell << ")" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 14: Invalid spreading type throws
// ─────────────────────────────────────────────────────────────────────────────
static bool test_invalid_spreading_throws() {
    SeaStateDefinition def;
    def.type = "irregular";
    def.n_omega = 16;
    def.n_theta = 9;
    def.seed = 42;

    SeaStatePartition p;
    p.spectrum.type = "jonswap";
    p.spectrum.Hs = 2.0;
    p.spectrum.Tp = 8.0;
    p.spreading.type = "cosine_bogus";
    p.spreading.mean_direction_deg = 0.0;
    p.spreading.s = 12.0;
    def.partitions.push_back(p);

    bool threw = false;
    try {
        (void)ComponentSampler::Build(def);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Invalid spreading type should throw std::invalid_argument");

    // Also test s <= 0 with valid cos2s type
    def.partitions[0].spreading.type = "cos2s";
    def.partitions[0].spreading.s = 0.0;
    threw = false;
    try {
        (void)ComponentSampler::Build(def);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    TEST_ASSERT(threw, "cos2s with s=0 should throw std::invalid_argument");

    std::cout << "  PASSED: test_invalid_spreading_throws" << std::endl;
    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// Helper: build minimal mock H5 data for excitation force tests
// ═════════════════════════════════════════════════════════════════════════════
struct MockH5Builder {
    int n_freq;
    int n_headings;
    int n_bodies;
    Eigen::VectorXd freq_list;
    Eigen::VectorXd headings_rad;
    std::vector<HydroData::RegularWaveInfo> infos;
    HydroData::SimulationParameters sim;

    MockH5Builder(int nf, int nh, int nb = 1)
        : n_freq(nf), n_headings(nh), n_bodies(nb) {
        sim.water_depth = 0.0;
        sim.g = 9.81;
        freq_list.resize(n_freq);
        headings_rad.resize(n_headings);
        infos.resize(n_bodies);
        for (int b = 0; b < n_bodies; ++b) {
            infos[b].freq_list = freq_list;
            infos[b].excitation_mag_matrix.resize(6, n_headings, n_freq);
            infos[b].excitation_phase_matrix.resize(6, n_headings, n_freq);
            infos[b].excitation_mag_matrix.setZero();
            infos[b].excitation_phase_matrix.setZero();
        }
    }

    void SetFreqs(const std::vector<double>& freqs) {
        for (int i = 0; i < n_freq; ++i) freq_list[i] = freqs[i];
        for (int b = 0; b < n_bodies; ++b) infos[b].freq_list = freq_list;
    }

    void SetHeadings(const std::vector<double>& hdeg) {
        for (int i = 0; i < n_headings; ++i)
            headings_rad[i] = hdeg[i] * M_PI / 180.0;
    }

    void SetExcitation(int body, int dof, int h_idx, int f_idx, double mag, double phase) {
        infos[body].excitation_mag_matrix(dof, h_idx, f_idx) = mag;
        infos[body].excitation_phase_matrix(dof, h_idx, f_idx) = phase;
    }
};

/// Frequency-domain excitation force on one DOF via ExcitationComponent (owned TF).
double ExcitationForceDof(
    const std::vector<WaveComponent>& components,
    const std::vector<HydroData::RegularWaveInfo>& infos,
    const Eigen::VectorXd& h5_wave_directions,
    ExcitationInterpolation interp,
    int num_bodies,
    double ramp_duration,
    double time,
    int body_index,
    int dof_index) {
    ExcitationTFData tf = InterpolateExcitationTransfer(
        components, infos, h5_wave_directions, interp);
    ExcitationComponent::WaveComponentData wcd;
    const Eigen::Index nc = static_cast<Eigen::Index>(components.size());
    wcd.amplitudes.resize(nc);
    wcd.omegas.resize(nc);
    wcd.phases.resize(nc);
    for (Eigen::Index i = 0; i < nc; ++i) {
        wcd.amplitudes[i] = components[i].amplitude;
        wcd.omegas[i]     = components[i].omega;
        wcd.phases[i]     = components[i].phase;
    }
    ExcitationComponent exc(std::move(wcd), std::move(tf.re), std::move(tf.im),
                            num_bodies, ramp_duration);
    SystemState state;
    state.bodies.resize(static_cast<std::size_t>(num_bodies));
    BodyForces bf(num_bodies);
    exc.Compute(state, time, bf);
    if (dof_index < 3) {
        return bf[body_index].force[dof_index];
    }
    return bf[body_index].moment[dof_index - 3];
}

/// Stacked 6*num_bodies vector (surge..yaw per body), same layout as legacy GetForceAtTime.
Eigen::VectorXd ExcitationForceVector(
    const std::vector<WaveComponent>& components,
    const std::vector<HydroData::RegularWaveInfo>& infos,
    const Eigen::VectorXd& h5_wave_directions,
    ExcitationInterpolation interp,
    int num_bodies,
    double ramp_duration,
    double time) {
    ExcitationTFData tf = InterpolateExcitationTransfer(
        components, infos, h5_wave_directions, interp);
    ExcitationComponent::WaveComponentData wcd;
    const Eigen::Index nc = static_cast<Eigen::Index>(components.size());
    wcd.amplitudes.resize(nc);
    wcd.omegas.resize(nc);
    wcd.phases.resize(nc);
    for (Eigen::Index i = 0; i < nc; ++i) {
        wcd.amplitudes[i] = components[i].amplitude;
        wcd.omegas[i]     = components[i].omega;
        wcd.phases[i]     = components[i].phase;
    }
    ExcitationComponent exc(std::move(wcd), std::move(tf.re), std::move(tf.im),
                            num_bodies, ramp_duration);
    SystemState state;
    state.bodies.resize(static_cast<std::size_t>(num_bodies));
    BodyForces bf(num_bodies);
    exc.Compute(state, time, bf);
    Eigen::VectorXd f(num_bodies * kDofPerBody);
    for (int b = 0; b < num_bodies; ++b) {
        for (int d = 0; d < 3; ++d) {
            f[b * kDofPerBody + d] = bf[b].force[d];
        }
        for (int d = 0; d < 3; ++d) {
            f[b * kDofPerBody + 3 + d] = bf[b].moment[d];
        }
    }
    return f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 15: Single-frequency force reconstruction (Test A)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_force_reconstruction() {
    const double omega = 1.0;
    const double A = 2.0;
    const double phi = 0.7;
    const double M = 5000.0;
    const double P = 0.4;

    WaveComponent c;
    c.omega = omega;
    c.k = omega * omega / 9.81;
    c.direction = 0.0;
    c.amplitude = A;
    c.phase = phi;

    auto field = LinearDirectionalWaveField({c}, 0.0);

    MockH5Builder h5(1, 1);
    h5.SetFreqs({omega});
    h5.SetHeadings({0.0});
    h5.SetExcitation(0, 0, 0, 0, M, P);
    h5.SetExcitation(0, 2, 0, 0, M * 0.5, -P);

    field.SetNumBodies(1);

    for (double t : {0.0, 1.0, 2.5, 5.0, 10.0}) {
        double theta = phi - omega * t;
        double expected_dof0 = A * M * std::cos(-P + theta);
        double expected_dof2 = A * (M * 0.5) * std::cos(P + theta);
        double actual_dof0 = ExcitationForceDof(
            field.GetComponents(), h5.infos, h5.headings_rad,
            ExcitationInterpolation::kCartesian, 1, 0.0, t, 0, 0);
        double actual_dof2 = ExcitationForceDof(
            field.GetComponents(), h5.infos, h5.headings_rad,
            ExcitationInterpolation::kCartesian, 1, 0.0, t, 0, 2);

        TEST_NEAR(actual_dof0, expected_dof0, 1e-6,
                  "Force DOF0 at t=" + std::to_string(t));
        TEST_NEAR(actual_dof2, expected_dof2, 1e-6,
                  "Force DOF2 at t=" + std::to_string(t));
    }

    std::cout << "  PASSED: test_force_reconstruction" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 16: Phase sign verification (Test B)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_force_phase_sign() {
    const double omega = 1.0;
    const double A = 1.0;
    const double M = 1000.0;

    WaveComponent c;
    c.omega = omega;
    c.k = omega * omega / 9.81;
    c.direction = 0.0;
    c.amplitude = A;
    c.phase = 0.0;

    // P=0: at t=0 with phi=0, F = A*M*cos(0) = A*M (positive)
    {
        auto field = LinearDirectionalWaveField({c}, 0.0);
        MockH5Builder h5(1, 1);
        h5.SetFreqs({omega});
        h5.SetHeadings({0.0});
        h5.SetExcitation(0, 0, 0, 0, M, 0.0);
        field.SetNumBodies(1);

        double f = ExcitationForceDof(
            field.GetComponents(), h5.infos, h5.headings_rad,
            ExcitationInterpolation::kCartesian, 1, 0.0, 0.0, 0, 0);
        TEST_NEAR(f, A * M, 1e-6, "Phase=0: force at t=0 should be +A*M");
        TEST_ASSERT(f > 0.0, "Phase=0: force should be positive");
    }

    // P=pi: at t=0 with phi=0, F = A*M*cos(pi) = -A*M (negative)
    {
        auto field = LinearDirectionalWaveField({c}, 0.0);
        MockH5Builder h5(1, 1);
        h5.SetFreqs({omega});
        h5.SetHeadings({0.0});
        h5.SetExcitation(0, 0, 0, 0, M, M_PI);
        field.SetNumBodies(1);

        double f = ExcitationForceDof(
            field.GetComponents(), h5.infos, h5.headings_rad,
            ExcitationInterpolation::kCartesian, 1, 0.0, 0.0, 0, 0);
        TEST_NEAR(f, -A * M, 1e-6, "Phase=pi: force at t=0 should be -A*M");
        TEST_ASSERT(f < 0.0, "Phase=pi: force should be negative");
    }

    // P=pi/2: at t=0 with phi=0, F = A*M*cos(pi/2) = 0
    {
        auto field = LinearDirectionalWaveField({c}, 0.0);
        MockH5Builder h5(1, 1);
        h5.SetFreqs({omega});
        h5.SetHeadings({0.0});
        h5.SetExcitation(0, 0, 0, 0, M, M_PI / 2.0);
        field.SetNumBodies(1);

        double f = ExcitationForceDof(
            field.GetComponents(), h5.infos, h5.headings_rad,
            ExcitationInterpolation::kCartesian, 1, 0.0, 0.0, 0, 0);
        TEST_NEAR(f, 0.0, 1e-6, "Phase=pi/2: force at t=0 should be ~0");
    }

    std::cout << "  PASSED: test_force_phase_sign" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 16b: Transfer-function convention mapping (H5/BEMIO vs ExcitationComponent)
//
// CONVENTIONS:
//   - H5/BEMIO (WEC-Sim): Stores H = mag*exp(i*phase) for exp(+i*omega*t).
//     Force = Re[H * A * exp(i*(phi + omega*t))].
//   - ExcitationComponent: theta = phi - omega*t; force uses conjugated Re/Im from
//     InterpolateExcitationTransfer so Re[H* e^{-i omega t}] matches WEC-Sim.
//   - Conjugation:        re = mag*cos(phase), im = -mag*sin(phase).
//
// This test uses phase=pi/4, phi=pi/4, t=0 => theta=pi/4. With conjugation:
//   F = A*M*cos(theta - phase) = A*M*cos(0) = A*M.
// Without conjugation (im = +mag*sin(phase)): F = A*M*cos(phase + theta) = 0.
// So this case would fail if conjugation were removed.
// ─────────────────────────────────────────────────────────────────────────────
static bool test_transfer_function_convention_conjugation() {
    const double omega = 1.0;
    const double A = 1.0;
    const double M = 1000.0;
    const double P = M_PI / 4.0;  // 45 deg: both Re and Im of H are non-zero

    WaveComponent c;
    c.omega = omega;
    c.k = omega * omega / 9.81;
    c.direction = 0.0;
    c.amplitude = A;
    c.phase = P;  // phi = P

    auto field = LinearDirectionalWaveField({c}, 0.0);
    MockH5Builder h5(1, 1);
    h5.SetFreqs({omega});
    h5.SetHeadings({0.0});
    h5.SetExcitation(0, 0, 0, 0, M, P);
    field.SetNumBodies(1);

    double t = 0.0;
    double theta = c.phase - omega * t;  // = P
    double expected = A * M * std::cos(theta - P);  // = A*M*cos(0) = A*M
    double actual = ExcitationForceDof(
        field.GetComponents(), h5.infos, h5.headings_rad,
        ExcitationInterpolation::kCartesian, 1, 0.0, t, 0, 0);

    TEST_NEAR(actual, expected, 1e-6,
              "Conjugation: F = A*M*cos(theta-phase) at phase=phi=pi/4, t=0");
    TEST_NEAR(actual, A * M, 1e-6, "Conjugation: force should be A*M (not 0)");

    std::cout << "  PASSED: test_transfer_function_convention_conjugation"
              << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 17: Two-frequency interpolation in Re/Im space (Test C)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_force_freq_interpolation() {
    const double omega1 = 0.5;
    const double omega2 = 1.5;
    const double omega_mid = 1.0;
    const double A = 1.0;

    const double M1 = 1000.0, P1 = 0.0;
    const double M2 = 3000.0, P2 = M_PI;

    // Stored as conjugate: re = mag*cos(P), im = -mag*sin(P).
    // Cartesian interpolation operates on the conjugated re/im.
    double Re1 = M1 * std::cos(P1), Im1_conj = -M1 * std::sin(P1);
    double Re2 = M2 * std::cos(P2), Im2_conj = -M2 * std::sin(P2);
    double Re_mid = 0.5 * (Re1 + Re2);
    double Im_mid_conj = 0.5 * (Im1_conj + Im2_conj);

    WaveComponent c;
    c.omega = omega_mid;
    c.k = omega_mid * omega_mid / 9.81;
    c.direction = 0.0;
    c.amplitude = A;
    c.phase = 0.0;

    auto field = LinearDirectionalWaveField({c}, 0.0);

    MockH5Builder h5(2, 1);
    h5.SetFreqs({omega1, omega2});
    h5.SetHeadings({0.0});
    h5.SetExcitation(0, 0, 0, 0, M1, P1);
    h5.SetExcitation(0, 0, 0, 1, M2, P2);
    field.SetNumBodies(1);

    double t = 0.0;
    double theta = c.phase - c.omega * t;
    double expected = A * (Re_mid * std::cos(theta) - Im_mid_conj * std::sin(theta));
    double actual = ExcitationForceDof(
        field.GetComponents(), h5.infos, h5.headings_rad,
        ExcitationInterpolation::kCartesian, 1, 0.0, t, 0, 0);

    TEST_NEAR(actual, expected, 1e-6,
              "Freq interpolation: Re/Im midpoint at omega_mid");

    // Verify this differs from naive mag/phase interpolation
    double M_naive = 0.5 * (M1 + M2);
    double P_naive = 0.5 * (P1 + P2);
    double naive_expected = A * M_naive * std::cos(-P_naive + theta);
    TEST_ASSERT(std::abs(expected - naive_expected) > 100.0,
                "Re/Im interpolation should differ from naive mag/phase near phase wrap");

    std::cout << "  PASSED: test_force_freq_interpolation" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 18: Heading interpolation with force values (Test D)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_force_heading_interpolation() {
    const double omega = 1.0;
    const double A = 1.0;

    WaveComponent c;
    c.omega = omega;
    c.k = omega * omega / 9.81;
    c.direction = 45.0 * M_PI / 180.0;
    c.amplitude = A;
    c.phase = 0.0;

    auto field = LinearDirectionalWaveField({c}, 0.0);

    MockH5Builder h5(1, 4);
    h5.SetFreqs({omega});
    h5.SetHeadings({0.0, 90.0, 180.0, 270.0});

    double M_at_0   = 2000.0;
    double M_at_90  = 1000.0;
    double M_at_180 = 2000.0;
    double M_at_270 = 1000.0;

    h5.SetExcitation(0, 0, 0, 0, M_at_0,   0.0);
    h5.SetExcitation(0, 0, 1, 0, M_at_90,  0.0);
    h5.SetExcitation(0, 0, 2, 0, M_at_180, 0.0);
    h5.SetExcitation(0, 0, 3, 0, M_at_270, 0.0);

    field.SetNumBodies(1);

    // At heading 45 deg (midway between 0 and 90), all phases=0:
    // Re = mag, Im = 0 for each heading. Linear interpolation in Re gives
    // Re_interp = 0.5*(Re_0 + Re_90) = 0.5*(2000+1000) = 1500.
    double t = 0.0;
    double theta = c.phase - c.omega * t;
    double expected = A * 1500.0 * std::cos(theta);
    double actual = ExcitationForceDof(
        field.GetComponents(), h5.infos, h5.headings_rad,
        ExcitationInterpolation::kCartesian, 1, 0.0, t, 0, 0);

    TEST_NEAR(actual, expected, 1e-6,
              "Heading interpolation at 45 deg should blend 0 and 90 deg values");

    std::cout << "  PASSED: test_force_heading_interpolation" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 19: Multi-body force independence (Test E)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_force_multibody() {
    const double omega = 1.0;
    const double A = 1.0;

    WaveComponent c;
    c.omega = omega;
    c.k = omega * omega / 9.81;
    c.direction = 0.0;
    c.amplitude = A;
    c.phase = 0.0;

    auto field = LinearDirectionalWaveField({c}, 0.0);

    MockH5Builder h5(1, 1, 2);
    h5.SetFreqs({omega});
    h5.SetHeadings({0.0});

    double M_body0 = 1000.0;
    double M_body1 = 5000.0;
    h5.SetExcitation(0, 0, 0, 0, M_body0, 0.0);
    h5.SetExcitation(1, 0, 0, 0, M_body1, 0.0);

    field.SetNumBodies(2);

    Eigen::VectorXd f = ExcitationForceVector(
        field.GetComponents(), h5.infos, h5.headings_rad,
        ExcitationInterpolation::kCartesian, 2, 0.0, 0.0);
    TEST_ASSERT(f.size() == 12, "2 bodies * 6 DOFs = 12 total DOFs");

    double f_body0_dof0 = f(0);
    double f_body1_dof0 = f(6);

    TEST_NEAR(f_body0_dof0, A * M_body0, 1e-6, "Body 0 DOF 0 force");
    TEST_NEAR(f_body1_dof0, A * M_body1, 1e-6, "Body 1 DOF 0 force");
    TEST_ASSERT(std::abs(f_body0_dof0 - f_body1_dof0) > 100.0,
                "Bodies should have different forces");

    std::cout << "  PASSED: test_force_multibody" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 20: n_theta=1 collapses to long-crested behavior (Test H)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_ntheta1_equals_long_crested() {
    const double Hs = 2.0, Tp = 8.0;
    const int n_omega = 32;
    const int seed = 123;

    // Path 1: long-crested (spreading=none, n_theta=1)
    SeaStateDefinition def1;
    def1.type = "irregular";
    def1.n_omega = n_omega;
    def1.n_theta = 1;
    def1.seed = seed;
    SeaStatePartition p1;
    p1.spectrum.type = "jonswap";
    p1.spectrum.Hs = Hs;
    p1.spectrum.Tp = Tp;
    p1.spreading.type = "none";
    p1.spreading.mean_direction_deg = 30.0;
    def1.partitions.push_back(p1);
    auto comp1 = ComponentSampler::Build(def1);

    // Path 2: directional with n_theta=1 (spreading=cos2s but only 1 bin)
    SeaStateDefinition def2;
    def2.type = "irregular";
    def2.n_omega = n_omega;
    def2.n_theta = 1;
    def2.seed = seed;
    SeaStatePartition p2;
    p2.spectrum.type = "jonswap";
    p2.spectrum.Hs = Hs;
    p2.spectrum.Tp = Tp;
    p2.spreading.type = "cos2s";
    p2.spreading.mean_direction_deg = 30.0;
    p2.spreading.s = 12.0;
    def2.partitions.push_back(p2);
    auto comp2 = ComponentSampler::Build(def2);

    TEST_ASSERT(comp1.size() == comp2.size(),
                "n_theta=1: both paths should produce same number of components");

    for (size_t i = 0; i < comp1.size(); ++i) {
        TEST_NEAR(comp1[i].omega, comp2[i].omega, 1e-12,
                  "n_theta=1: omega should match");
        TEST_NEAR(comp1[i].direction, comp2[i].direction, 1e-12,
                  "n_theta=1: direction should match");
        TEST_NEAR(comp1[i].amplitude, comp2[i].amplitude, 1e-12,
                  "n_theta=1: amplitude should match");
        TEST_NEAR(comp1[i].phase, comp2[i].phase, 1e-12,
                  "n_theta=1: phase should match");
    }

    // Also verify elevations match
    auto field1 = LinearDirectionalWaveField(comp1, 0.0);
    auto field2 = LinearDirectionalWaveField(comp2, 0.0);
    Eigen::Vector3d pos(10.0, 5.0, 0.0);
    for (double t : {0.0, 2.0, 5.0, 10.0}) {
        double eta1 = field1.GetElevation(pos, t);
        double eta2 = field2.GetElevation(pos, t);
        TEST_NEAR(eta1, eta2, 1e-12,
                  "n_theta=1: elevation should match at t=" + std::to_string(t));
    }

    std::cout << "  PASSED: test_ntheta1_equals_long_crested" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 21: 180-degree heading flip reverses surge force sign (Test G)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_heading_180_flip() {
    const double omega = 1.0;
    const double A = 1.0;
    const double M_surge = 2000.0;

    // Build H5 data with headings at 0 and 180 degrees.
    // Surge excitation: mag=M at both, phase=0 at heading 0, phase=pi at heading 180.
    // This models a symmetric body where the surge force reverses with wave direction.
    MockH5Builder h5(1, 2);
    h5.SetFreqs({omega});
    h5.SetHeadings({0.0, 180.0});
    h5.SetExcitation(0, 0, 0, 0, M_surge, 0.0);
    h5.SetExcitation(0, 0, 1, 0, M_surge, M_PI);

    // Component at heading 0
    WaveComponent c0;
    c0.omega = omega;
    c0.k = omega * omega / 9.81;
    c0.direction = 0.0;
    c0.amplitude = A;
    c0.phase = 0.0;

    auto field0 = LinearDirectionalWaveField({c0}, 0.0);
    field0.SetNumBodies(1);
    double f_0deg = ExcitationForceDof(
        field0.GetComponents(), h5.infos, h5.headings_rad,
        ExcitationInterpolation::kCartesian, 1, 0.0, 0.0, 0, 0);

    // Component at heading 180
    WaveComponent c180;
    c180.omega = omega;
    c180.k = omega * omega / 9.81;
    c180.direction = M_PI;
    c180.amplitude = A;
    c180.phase = 0.0;

    auto field180 = LinearDirectionalWaveField({c180}, 0.0);
    field180.SetNumBodies(1);
    double f_180deg = ExcitationForceDof(
        field180.GetComponents(), h5.infos, h5.headings_rad,
        ExcitationInterpolation::kCartesian, 1, 0.0, 0.0, 0, 0);

    TEST_NEAR(f_0deg, A * M_surge, 1e-6,
              "Heading 0: surge should be +A*M");
    TEST_NEAR(f_180deg, -A * M_surge, 1e-6,
              "Heading 180: surge should be -A*M");
    TEST_NEAR(f_0deg, -f_180deg, 1e-6,
              "Heading 0 and 180 should produce opposite surge forces");

    std::cout << "  PASSED: test_heading_180_flip" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 22: Old/new path equivalence (Test F)
//
// Without a real H5 file, we verify mathematical equivalence by constructing
// a synthetic excitation IRF K(τ) = A * exp(-α*τ), computing the legacy C/S
// coefficients analytically, then comparing against the new path fed with the
// corresponding analytical mag/phase.
//
// Analytical Fourier transform of K(τ) = A * exp(-α*τ) for τ ≥ 0:
//   H(ω) = A / (α + iω)
//   Re(H) = A*α / (α² + ω²)
//   Im(H) = -A*ω / (α² + ω²)
//   mag   = A / sqrt(α² + ω²)
//   phase = atan2(-ω, α)
// ─────────────────────────────────────────────────────────────────────────────
static bool test_old_new_equivalence() {
    const double alpha = 0.5;
    const double K_amp = 1000.0;

    // Frequencies must be uniformly spaced to match the grid-based interpolation.
    const int n_freq = 5;
    double freqs[] = {0.3, 0.6, 0.9, 1.2, 1.5};
    const double A = 1.0;

    std::vector<WaveComponent> comps(n_freq);
    for (int i = 0; i < n_freq; ++i) {
        comps[i].omega = freqs[i];
        comps[i].k = freqs[i] * freqs[i] / 9.81;
        comps[i].direction = 0.0;
        comps[i].amplitude = A;
        comps[i].phase = 0.3 * i;
    }

    // Compute analytical mag/phase for each frequency (DOF 0 only).
    MockH5Builder h5(n_freq, 1);
    std::vector<double> flist(freqs, freqs + n_freq);
    h5.SetFreqs(flist);
    h5.SetHeadings({0.0});

    // Also compute the legacy C/S coefficients analytically.
    std::vector<double> C_legacy(n_freq), S_legacy(n_freq);
    for (int i = 0; i < n_freq; ++i) {
        double w = freqs[i];
        double denom = alpha * alpha + w * w;
        double Re = K_amp * alpha / denom;
        double Im = -K_amp * w / denom;
        double mag = K_amp / std::sqrt(denom);
        double phase = std::atan2(Im, Re);

        h5.SetExcitation(0, 0, 0, i, mag, phase);

        C_legacy[i] = Re;
        S_legacy[i] = Im;
    }

    auto field = LinearDirectionalWaveField(comps, 0.0);
    field.SetNumBodies(1);

    // Compare forces at several times.
    // InterpolateExcitationTransfer stores conjugated H*; ExcitationComponent uses
    // the same time convention as the former wave-embedded path:
    //   F = sum A_i * (Re(H)*cos(θ) + Im(H)*sin(θ))
    // with θ = phi - omega*t, matching WEC-Sim Re[H * exp(+i*omega*t + i*phi)].
    for (double t : {0.0, 1.0, 3.0, 7.0, 15.0}) {
        double f_legacy = 0.0;
        for (int i = 0; i < n_freq; ++i) {
            double theta = comps[i].phase - comps[i].omega * t;
            f_legacy += comps[i].amplitude *
                (C_legacy[i] * std::cos(theta) + S_legacy[i] * std::sin(theta));
        }

        double f_new = ExcitationForceDof(
            field.GetComponents(), h5.infos, h5.headings_rad,
            ExcitationInterpolation::kCartesian, 1, 0.0, t, 0, 0);
        TEST_NEAR(f_new, f_legacy, 1e-6,
                  "Old/new equivalence at t=" + std::to_string(t));
    }

    std::cout << "  PASSED: test_old_new_equivalence" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 23: BuildFromEtaFile exact amplitude/phase recovery for well-separated
//          sinusoids at Fourier frequencies.
//
// Generates a synthetic eta(t) from 3 sinusoidal components whose frequencies
// lie exactly on the Fourier grid.  Verifies that BuildFromEtaFile recovers
// each component's amplitude and phase, and that the Parseval energy check
// passes (sum(A^2/2) ~ var(eta)).
// ─────────────────────────────────────────────────────────────────────────────
static bool test_build_from_eta_file_exact_recovery() {
    const double dt = 0.02;
    const int N = 10000;
    const double T_window = N * dt;       // 200 s
    const double df = 1.0 / T_window;     // 0.005 Hz

    struct TestComponent { int k; double amplitude; double phase; };
    TestComponent inputs[] = {
        {10,  1.5,  0.3},   // f = 0.05 Hz
        {40,  0.8,  2.1},   // f = 0.20 Hz
        {120, 0.4, -1.0},   // f = 0.60 Hz
    };
    const int n_inputs = 3;

    // Build the eta time series from the known components.
    std::vector<double> times(N);
    std::vector<double> etas(N, 0.0);
    for (int n = 0; n < N; ++n) {
        times[n] = n * dt;
        for (int c = 0; c < n_inputs; ++c) {
            double omega = 2.0 * M_PI * inputs[c].k * df;
            etas[n] += inputs[c].amplitude *
                       std::cos(inputs[c].phase - omega * times[n]);
        }
    }

    // Write to a temporary file in the system temp directory.
    const std::filesystem::path tmp_path =
        std::filesystem::temp_directory_path() / "test_eta_dft_roundtrip_tmp.txt";
    {
        std::ofstream f(tmp_path);
        TEST_ASSERT(f.is_open(), "Cannot create temp eta file");
        f << std::fixed << std::setprecision(12);
        for (int n = 0; n < N; ++n) {
            f << times[n] << ":" << etas[n] << "\n";
        }
    }

    // Extract via BuildFromEtaFile over a band that covers all 3 components.
    auto components = ComponentSampler::BuildFromEtaFile(
        tmp_path.string(), 0.0, 9.81, 5000, 0.01, 0.80);
    std::filesystem::remove(tmp_path);

    TEST_ASSERT(components.size() >= 3,
                "Should recover at least 3 components (got "
                + std::to_string(components.size()) + ")");

    // For each input component, find the matching extracted component.
    for (int c = 0; c < n_inputs; ++c) {
        double target_omega = 2.0 * M_PI * inputs[c].k * df;
        const WaveComponent* best = nullptr;
        double best_dist = 1e30;
        for (const auto& comp : components) {
            double dist = std::abs(comp.omega - target_omega);
            if (dist < best_dist) {
                best_dist = dist;
                best = &comp;
            }
        }
        TEST_ASSERT(best != nullptr, "Should find a component near target omega");
        TEST_NEAR(best->omega, target_omega, 1e-6,
                  "Omega match for input " + std::to_string(c));
        TEST_NEAR(best->amplitude, inputs[c].amplitude, 0.02,
                  "Amplitude recovery for input " + std::to_string(c));

        // Phase comparison: normalize both to [0, 2*pi).
        auto normalize_phase = [](double p) {
            p = std::fmod(p, 2.0 * M_PI);
            if (p < 0.0) p += 2.0 * M_PI;
            return p;
        };
        double phase_diff = normalize_phase(best->phase) -
                            normalize_phase(inputs[c].phase);
        if (phase_diff > M_PI) phase_diff -= 2.0 * M_PI;
        if (phase_diff < -M_PI) phase_diff += 2.0 * M_PI;
        TEST_NEAR(phase_diff, 0.0, 0.05,
                  "Phase recovery for input " + std::to_string(c));
    }

    // Parseval energy check: sum(A^2/2) should approximate var(eta).
    double mean_eta = 0.0;
    for (int n = 0; n < N; ++n) mean_eta += etas[n];
    mean_eta /= N;
    double var_eta = 0.0;
    for (int n = 0; n < N; ++n) {
        double d = etas[n] - mean_eta;
        var_eta += d * d;
    }
    var_eta /= N;

    double spectral_energy = 0.0;
    for (const auto& comp : components) {
        spectral_energy += 0.5 * comp.amplitude * comp.amplitude;
    }

    double parseval_ratio = spectral_energy / var_eta;
    TEST_NEAR(parseval_ratio, 1.0, 0.10,
              "Parseval ratio should be ~1.0 (got "
              + std::to_string(parseval_ratio) + ")");

    // Verify reconstructed eta matches original at several time points.
    auto field = LinearDirectionalWaveField(components, 0.0);
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    double max_err = 0.0;
    for (int n = N / 4; n < 3 * N / 4; ++n) {
        double eta_rec = field.GetElevation(origin, times[n]);
        double err = std::abs(eta_rec - etas[n]);
        if (err > max_err) max_err = err;
    }
    TEST_ASSERT(max_err < 0.05,
                "Reconstructed eta max error in middle 50% should be < 0.05 m (got "
                + std::to_string(max_err) + ")");

    std::cout << "  PASSED: test_build_from_eta_file_exact_recovery"
              << " (parseval=" << std::fixed << std::setprecision(4) << parseval_ratio
              << ", max_err=" << max_err << ")" << std::endl;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "=== Directional Waves Unit Tests ===" << std::endl;

    int passed = 0;
    int failed = 0;

    auto run = [&](bool (*test)()) {
        if (test()) ++passed; else ++failed;
    };

    run(test_sampler_regular);
    run(test_sampler_long_crested);
    run(test_sampler_directional);
    run(test_sampler_bimodal);
    run(test_cos2s_normalization);
    run(test_kinematics_elevation);
    run(test_wave_field_elevation);
    run(test_variance_conservation);
    run(test_direction_zero_matches_plus_x);
    run(test_finite_depth_wavenumber);
    run(test_newton_dispersion_accuracy);
    run(test_heading_wraparound);
    run(test_bimodal_omega_range);
    run(test_invalid_spreading_throws);
    run(test_force_reconstruction);
    run(test_force_phase_sign);
    run(test_transfer_function_convention_conjugation);
    run(test_force_freq_interpolation);
    run(test_force_heading_interpolation);
    run(test_force_multibody);
    run(test_ntheta1_equals_long_crested);
    run(test_heading_180_flip);
    run(test_old_new_equivalence);
    run(test_build_from_eta_file_exact_recovery);

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===" << std::endl;
    return (failed > 0) ? 1 : 0;
}
