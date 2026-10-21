/*********************************************************************
 * @file  test_wave_pressure.cpp
 * @brief Unit tests for WaveBase::GetDynamicPressureHead (Airy pressure
 *        with Wheeler stretching), used by nonlinear Froude-Krylov.
 *
 * Pass criteria from docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9:
 *   U1  head vs closed-form Airy, deep and finite depth: error <= 1e-12 * sum(a)
 *   U2  total pressure at z = eta (ramped and unramped): |p| <= 1e-9 rho g a
 *
 * Self-contained: no data files, no Chrono.
 *********************************************************************/

#include <seastack/hydro/waves/eta_table_wave_field.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_base.h>

#include "test_macros.h"

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

using seastack::hydro::EtaTableWaveField;
using seastack::hydro::LinearDirectionalWaveField;
using seastack::hydro::NoWave;
using seastack::hydro::WaveComponent;

namespace {

constexpr double kG   = 9.81;
constexpr double kRho = 1025.0;

/// Wavenumber from w^2 = g k tanh(k h) by Newton iteration (deep water if h <= 0).
double SolveWavenumber(double omega, double depth) {
    double k = omega * omega / kG;
    if (depth <= 0.0) return k;
    for (int it = 0; it < 100; ++it) {
        const double t  = std::tanh(k * depth);
        const double f  = omega * omega - kG * k * t;
        const double df = -kG * t - kG * k * depth * (1.0 - t * t);
        const double dk = f / df;
        k -= dk;
        if (std::abs(dk) < 1e-15 * k) break;
    }
    return k;
}

WaveComponent MakeComponent(double amplitude, double omega, double depth, double direction, double phase) {
    WaveComponent c;
    c.amplitude = amplitude;
    c.omega     = omega;
    c.k         = SolveWavenumber(omega, depth);
    c.direction = direction;
    c.phase     = phase;
    return c;
}

/// Closed-form Airy pressure head for one component, written independently of
/// the library: Wheeler-stretched z_s, then cosh k(z_s+h)/cosh kh (or e^{k z_s}).
double ClosedFormHead(const WaveComponent& c, double depth, const Eigen::Vector3d& p, double t,
                      double elevation) {
    const double phase = c.k * (p.x() * std::cos(c.direction) + p.y() * std::sin(c.direction))
                       - c.omega * t + c.phase;
    if (depth <= 0.0) {
        return c.amplitude * std::exp(c.k * (p.z() - elevation)) * std::cos(phase);
    }
    const double z_s = depth * (p.z() - elevation) / (depth + elevation);
    return c.amplitude * std::cosh(c.k * (z_s + depth)) / std::cosh(c.k * depth) * std::cos(phase);
}

std::vector<WaveComponent> MakeDirectionalSea(double depth) {
    return {MakeComponent(0.8, 0.6, depth, 0.0, 0.3),
            MakeComponent(0.5, 0.9, depth, 0.7, 2.1),
            MakeComponent(0.3, 1.4, depth, -1.2, 4.0),
            MakeComponent(0.1, 2.5, depth, 3.0, 1.0)};
}

}  // namespace

int main() {
    TestResults test_results;

    // Horizontal sample points (z ignored) and times.
    const std::vector<Eigen::Vector3d> points = {
        {0.0, 0.0, 0.0}, {12.3, -4.0, 0.0}, {-7.5, 20.0, 0.0}, {3.0, 3.0, 0.0}};
    const std::vector<double> times = {0.0, 1.7, 13.2};

    // U1: head vs closed form at several depths below the local surface.
    // Water depths: deep (0); 30 m; 6 m (kh ~ 0.6 for the lowest frequency);
    // 500 m, where the two highest frequencies use the kh > 89.4 deep branch.
    for (double depth : {0.0, 30.0, 6.0, 500.0}) {
        const auto comps = MakeDirectionalSea(depth);
        LinearDirectionalWaveField field(comps, depth);
        double sum_a = 0.0;
        for (const auto& c : comps) sum_a += c.amplitude;

        for (const auto& xy : points) {
            for (double t : times) {
                const double eta = field.GetElevation(xy, t);
                for (double below : {0.0, 0.5, 2.0, 4.0}) {
                    const Eigen::Vector3d p(xy.x(), xy.y(), eta - below);
                    if (depth > 0.0 && p.z() <= -depth) continue;
                    double expected = 0.0;
                    for (const auto& c : comps) expected += ClosedFormHead(c, depth, p, t, eta);
                    TEST_NEAR(field.GetDynamicPressureHead(p, t, eta), expected, 1e-12 * sum_a,
                              "U1 head vs closed-form Airy, depth " + std::to_string(depth));
                }
            }
        }
    }

    // U2: total pressure p = rho g (r * head - z) is zero at z = r * eta.
    for (double depth : {0.0, 30.0}) {
        const auto comps = MakeDirectionalSea(depth);
        LinearDirectionalWaveField field(comps, depth);
        for (double ramp : {1.0, 0.3}) {
            for (const auto& xy : points) {
                for (double t : times) {
                    const double eta = ramp * field.GetElevation(xy, t);
                    const Eigen::Vector3d p(xy.x(), xy.y(), eta);
                    const double head = field.GetDynamicPressureHead(p, t, eta);
                    const double pressure = kRho * kG * (ramp * head - p.z());
                    TEST_NEAR(pressure, 0.0, 1e-9 * kRho * kG * 0.8,
                              "U2 p = 0 at the free surface, depth " + std::to_string(depth) +
                                  ", ramp " + std::to_string(ramp));
                }
            }
        }
    }

    // Pressure decays monotonically with depth in deep water (single component, crest).
    {
        LinearDirectionalWaveField field({MakeComponent(1.0, 1.0, 0.0, 0.0, 0.0)}, 0.0);
        const double eta = field.GetElevation(Eigen::Vector3d::Zero(), 0.0);
        double prev = std::numeric_limits<double>::infinity();
        bool monotone = true;
        for (double z = eta; z > -30.0; z -= 1.0) {
            const double h = field.GetDynamicPressureHead(Eigen::Vector3d(0.0, 0.0, z), 0.0, eta);
            monotone = monotone && (h < prev);
            prev = h;
        }
        TEST_ASSERT(monotone, "deep-water head decays with depth under a crest");
    }

    // Still water and unsupported wave models.
    {
        NoWave still;
        TEST_NEAR(still.GetDynamicPressureHead(Eigen::Vector3d(1.0, 2.0, -3.0), 4.0, 0.0), 0.0, 0.0,
                  "NoWave head is zero");

        EtaTableWaveField table({0.0, 1.0, 2.0}, {0.0, 0.1, 0.0}, 0.0);
        bool threw = false;
        try {
            (void)table.GetDynamicPressureHead(Eigen::Vector3d::Zero(), 0.5, 0.0);
        } catch (const std::logic_error&) {
            threw = true;
        }
        TEST_ASSERT(threw, "eta-table wave without components rejects pressure queries");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
