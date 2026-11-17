// Unit tests for the radiated-wave overlay physics (viz_core; Chrono-free, VSG-free).

#include <seastack/viz_core/radiated_wave_synthesis.h>
#include <seastack/viz_core/radiated_waves.h>

#include "test_macros.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

using seastack::viz::BuildApproximateModel;
using seastack::viz::BuildHaskindModel;
using seastack::viz::CheckAvailability;
using seastack::viz::Complex;
using seastack::viz::GroupVelocity;
using seastack::viz::HankelSecondKind;
using seastack::viz::OutgoingRadialFactors;
using seastack::viz::PatternPower;
using seastack::viz::RadiatedWaveMethod;
using seastack::viz::RadiatedWaveModel;
using seastack::viz::RadiatedWaveSynthesizer;
using seastack::viz::RadiationBodySample;
using seastack::viz::RadiationVelocityHistory;
using seastack::viz::RadiationVizInputs;
using seastack::viz::ResolveMethod;
using seastack::viz::WaveNumber;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;
constexpr double kRho = 1025.0;

/// One body, damping B(w) from `b_of_w(w, i, j)`, deep water, no excitation.
template <typename F>
RadiationVizInputs OneBody(const std::vector<double>& omega, F b_of_w) {
    RadiationVizInputs in;
    in.rho = kRho;
    in.gravity = kG;
    in.depth = 0.0;
    in.omega = omega;
    in.bodies.push_back({"body", 0.0, 0.0, 100.0});
    for (double w : omega) {
        std::vector<double> b(36);
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                b[6 * i + j] = b_of_w(w, i, j);
            }
        }
        in.damping.push_back(b);
    }
    return in;
}

std::vector<double> Grid(double w0, double w1, int n) {
    std::vector<double> w(n);
    for (int i = 0; i < n; ++i) {
        w[i] = w0 + (w1 - w0) * i / (n - 1);
    }
    return w;
}

}  // namespace

int main() {
    TestResults test_results;

    // Hankel functions against SciPy (jv, yv).
    {
        struct Ref {
            int m;
            double x, j, y;
        };
        const Ref refs[] = {
            {0, 0.3, 0.97762624653829611, -0.80727357780451969},
            {0, 1.0, 0.76519768655796661, 0.088256964215676997},
            {1, 1.0, 0.44005058574493355, -0.78121282130028891},
            {0, 7.5, 0.2663396578803785, 0.11731328614820861},
            {1, 11.9, -0.22898324966192407, -0.03471149833403063},
            {0, 12.1, 0.069666773606807231, -0.21843838055092551},
            {1, 12.1, -0.21574897337692481, -0.078736931451395778},
            {0, 40.0, 0.0073668905842372906, 0.12593641705826095},
            {1, 40.0, 0.12603831803758497, -0.0057935058215496191},
            {2, 3.0, 0.48609126058589119, -0.16040039348492366},
            {5, 10.0, -0.2340615281867936, 0.13540304768936218},
            {8, 4.0, 0.004028667820819001, -11.471091976491051},
            {12, 30.0, 0.14825335109966001, 0.034143171346460215},
            {12, 250.0, -0.012709978683778976, -0.048865826225922518},
            {3, 0.8, 0.010246766330553604, -10.81464663357559},
        };
        for (const Ref& r : refs) {
            std::vector<Complex> h(std::max(r.m, 1) + 1);
            HankelSecondKind(std::max(r.m, 1), r.x, h.data());
            const double scale = std::abs(Complex(r.j, r.y));
            TEST_NEAR(h[r.m].real(), r.j, 1e-9 * scale + 1e-14, "J_m(x) matches SciPy");
            TEST_NEAR(-h[r.m].imag(), r.y, 1e-9 * scale + 1e-14, "Y_m(x) matches SciPy");
        }
        // Wronskian J_{m+1} Y_m - J_m Y_{m+1} = 2 / (pi x), all orders and both branches.
        double worst = 0.0;
        for (double x : {0.2, 2.0, 11.5, 12.5, 60.0, 300.0}) {
            std::vector<Complex> h(14);
            HankelSecondKind(13, x, h.data());
            for (int m = 0; m < 13; ++m) {
                const double w = h[m + 1].real() * -h[m].imag() - h[m].real() * -h[m + 1].imag();
                worst = std::max(worst, std::abs(w * kPi * x / 2.0 - 1.0));
            }
        }
        TEST_ASSERT(worst < 1e-9, "Hankel Wronskian holds for m <= 12");
        std::vector<Complex> f(3);
        OutgoingRadialFactors(2, 400.0, f.data());
        for (int m = 0; m <= 2; ++m) {
            const Complex far = std::polar(1.0 / std::sqrt(400.0), -400.0);
            const double next_term = std::abs(4.0 * m * m - 1.0) / (8.0 * 400.0);
            TEST_ASSERT(std::abs(f[m] - far) < 1.1 * next_term * std::abs(far), "F_m(x) -> x^-1/2 e^{-ix}");
        }
    }

    // Dispersion.
    {
        TEST_NEAR(WaveNumber(1.0, kG, 0.0), 1.0 / kG, 1e-15, "deep water k = w^2/g");
        const double k = WaveNumber(0.5, kG, 20.0);
        TEST_NEAR(kG * k * std::tanh(k * 20.0), 0.25, 1e-12, "finite depth satisfies w^2 = g k tanh(kh)");
        const double ks = WaveNumber(0.01, kG, 10.0);
        TEST_NEAR(GroupVelocity(0.01, ks, 10.0), std::sqrt(kG * 10.0), 1e-3, "shallow c_g = sqrt(g h)");
    }

    // Availability and Auto.
    {
        RadiationVizInputs in = OneBody(Grid(0.1, 2.0, 20), [](double, int i, int j) { return i == j ? 1e4 : 0.0; });
        TEST_ASSERT(CheckAvailability(RadiatedWaveMethod::kApproximate, in).available, "Approximate needs only B");
        TEST_ASSERT(!CheckAvailability(RadiatedWaveMethod::kHaskind, in).available, "Haskind needs excitation");
        TEST_ASSERT(!CheckAvailability(RadiatedWaveMethod::kKochin, in).reason.empty(), "Kochin has a reason");
        TEST_ASSERT(!CheckAvailability(RadiatedWaveMethod::kNearField, in).available, "near field not available");
        TEST_ASSERT(ResolveMethod(RadiatedWaveMethod::kAuto, in) == RadiatedWaveMethod::kApproximate,
                    "Auto falls back to Approximate");
        TEST_ASSERT(ResolveMethod(RadiatedWaveMethod::kHaskind, in) == RadiatedWaveMethod::kOff,
                    "an unavailable choice resolves to Off");

        in.headings = {0.0, kPi};
        in.excitation.assign(in.omega.size() * 2 * 6, Complex(1.0, 0.0));
        TEST_ASSERT(!CheckAvailability(RadiatedWaveMethod::kHaskind, in).available, "two headings are not enough");
        in.headings.clear();
        for (int d = 0; d <= 12; ++d) {
            in.headings.push_back(d * kPi / 6.0);  // 0..360 deg, 360 duplicates 0
        }
        in.excitation.assign(in.omega.size() * in.headings.size() * 6, Complex(1.0, 0.0));
        TEST_ASSERT(CheckAvailability(RadiatedWaveMethod::kHaskind, in).available, "30 deg headings all round");
        TEST_ASSERT(ResolveMethod(RadiatedWaveMethod::kAuto, in) == RadiatedWaveMethod::kHaskind,
                    "Auto prefers Haskind");
    }

    // Approximate: drawn power = 1/2 U^H B U for each pattern group's block
    // when the DOFs move in phase; terms between groups are not drawn.
    {
        auto b_of_w = [](double w, int i, int j) {
            const double d[6] = {4e4, 3e4, 9e4, 2e6, 5e6, 1e5};
            if (i == j) {
                return d[i] * (0.5 + w);
            }
            if ((i == 0 && j == 4) || (i == 4 && j == 0)) {
                return -0.6 * std::sqrt(d[0] * d[4]) * (0.5 + w);  // surge-pitch, not rank one
            }
            if ((i == 1 && j == 3) || (i == 3 && j == 1)) {
                return 0.9 * std::sqrt(d[1] * d[3]) * (0.5 + w);
            }
            if ((i == 0 && j == 2) || (i == 2 && j == 0)) {
                return 0.1 * std::sqrt(d[0] * d[2]);  // cross-group: not drawn
            }
            return 0.0;
        };
        RadiationVizInputs in = OneBody(Grid(0.2, 2.0, 10), b_of_w);
        const RadiatedWaveModel model = BuildApproximateModel(in);
        TEST_ASSERT(model.approximate && model.sources.size() == 1, "approximate model, one source");
        const Complex phase = std::polar(1.0, 0.7);
        std::vector<Complex> u;
        for (double v : {0.3, -0.2, 1.0, 0.02, -0.03, 0.01}) {
            u.push_back(v * phase);
        }
        static constexpr int kGroup[6] = {1, 2, 0, 2, 1, 3};
        double worst = 0.0;
        for (size_t k = 0; k < in.omega.size(); ++k) {
            double expected = 0.0;
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    if (kGroup[i] == kGroup[j]) {
                        expected += 0.5 * (std::conj(u[i]) * in.B(k, i, j) * u[j]).real();
                    }
                }
            }
            worst = std::max(worst, std::abs(PatternPower(model, kRho, k, u) / expected - 1.0));
        }
        TEST_ASSERT(worst < 1e-12, "Approximate reproduces each group's damping block exactly");
        TEST_ASSERT(!model.notes.empty(), "cross-group coupling reported");
    }

    // Haskind: excitation built from a known pattern about a body at p gives
    // back that pattern (constant, sign and phase-reference shift).
    {
        const std::vector<double> omega = Grid(0.3, 1.5, 5);
        const double px = 40.0;
        const double py = -25.0;
        auto pattern = [](double w, double theta, int j) -> Complex {
            // Heave: a0 + a2 e^{2i theta}; surge: cos theta; others zero.
            if (j == 2) {
                return Complex(0.4 * w, -0.1) + Complex(0.15, 0.05 * w) * std::polar(1.0, 2.0 * theta);
            }
            if (j == 0) {
                return Complex(0.0, 0.2 * w) * std::cos(theta);
            }
            return Complex(0.0, 0.0);
        };
        RadiationVizInputs in = OneBody(omega, [](double, int i, int j) { return i == j ? 1.0 : 0.0; });
        in.bodies[0].ref_x = px;
        in.bodies[0].ref_y = py;
        for (int d = 0; d < 36; ++d) {
            in.headings.push_back(d * kPi / 18.0);
        }
        in.excitation.resize(omega.size() * 36 * 6);
        for (size_t k = 0; k < omega.size(); ++k) {
            const double kw = WaveNumber(omega[k], kG, 0.0);
            const double cg = GroupVelocity(omega[k], kw, 0.0);
            const Complex c = -std::polar(1.0, 0.25 * kPi) * (kw / (std::sqrt(8.0 * kPi) * kRho * kG * cg));
            for (int d = 0; d < 36; ++d) {
                const double theta = in.headings[d] + kPi;
                const Complex unshift = std::polar(1.0, kw * (px * std::cos(theta) + py * std::sin(theta)));
                for (int j = 0; j < 6; ++j) {
                    in.excitation[(k * 36 + d) * 6 + j] = pattern(omega[k], theta, j) * unshift / c;
                }
            }
        }
        const RadiatedWaveModel model = BuildHaskindModel(in);
        TEST_ASSERT(!model.approximate, "Haskind model is not marked approximate");
        const auto& src = model.sources[0];
        TEST_ASSERT(src.max_order >= 2, "order covers the m = 2 heave harmonic");
        double worst = 0.0;
        for (size_t k = 0; k < omega.size(); ++k) {
            const double w = omega[k];
            worst = std::max(worst, std::abs(src.A(k, 0, 2) - Complex(0.4 * w, -0.1)));
            worst = std::max(worst, std::abs(src.A(k, 2, 2) - Complex(0.15, 0.05 * w)));
            worst = std::max(worst, std::abs(src.A(k, 1, 0) - Complex(0.0, 0.1 * w)));
            worst = std::max(worst, std::abs(src.A(k, -1, 0) - Complex(0.0, 0.1 * w)));
            worst = std::max(worst, std::abs(src.A(k, 1, 2)));
        }
        TEST_ASSERT(worst < 1e-10, "Haskind recovers the pattern harmonics about the body");
    }

    // Synthesis: steady heave at w0 gives eta = Re{alpha F_0(k0 r) U0 e^{i w0 t}}.
    // Surge is a cos(theta) dipole (no wave abeam).
    {
        const std::vector<double> omega = Grid(0.1, 3.0, 30);
        RadiationVizInputs in = OneBody(omega, [](double w, int i, int j) {
            return (i == j && (i == 2 || i == 0)) ? 2e5 * w : 0.0;
        });
        const RadiatedWaveModel model = BuildApproximateModel(in);
        RadiatedWaveSynthesizer synth;
        synth.Configure(model, 300.0, 2.0, 0.1);
        synth.SetSourceRadius(0, 5.0);
        TEST_ASSERT(synth.IsConfigured() && synth.NumFrequencies() > 20, "synthesiser configured");

        const double w0 = 1.0;
        const double u0 = 0.5;
        for (int dof : {2, 0}) {
            RadiationVelocityHistory history(0.1);
            for (int i = 0; i <= 6000; ++i) {
                RadiationBodySample s;
                s.velocity[dof] = u0 * std::cos(w0 * 0.1 * i);
                history.Add(0.1 * i, {s});
            }
            const double r = 150.0;
            const double k0 = WaveNumber(w0, kG, 0.0);
            std::vector<Complex> f(2);
            OutgoingRadialFactors(1, k0 * r, f.data());
            const double cg = GroupVelocity(w0, k0, 0.0);
            const int m = (dof == 2) ? 0 : 1;
            const double alpha = std::sqrt(k0 * 2e5 * w0 / (kRho * kG * cg * (m == 0 ? 2.0 * kPi : kPi)));
            const double expected = alpha * std::abs(f[m]) * u0;
            double peak = 0.0;
            double peak_abeam = 0.0;
            for (int i = 0; i < 64; ++i) {
                const double t = 500.0 + i * 2.0 * kPi / w0 / 64.0;
                synth.Update(history, t);
                peak = std::max(peak, std::abs(synth.Evaluate(r, 0.0)));
                peak_abeam = std::max(peak_abeam, std::abs(synth.Evaluate(0.0, r)));
            }
            TEST_NEAR(peak / expected, 1.0, 0.03, "steady radiated amplitude matches the far field");
            if (dof == 0) {
                TEST_ASSERT(peak_abeam < 0.02 * peak, "surge radiates nothing abeam");
            } else {
                TEST_NEAR(peak_abeam / peak, 1.0, 0.01, "heave radiates equally all round");
            }
        }
    }

    // Causality, and the same picture whatever the order of displayed times.
    {
        const std::vector<double> omega = Grid(0.1, 3.0, 30);
        RadiationVizInputs in = OneBody(omega, [](double w, int i, int j) { return (i == j) ? 2e5 * w : 0.0; });
        RadiatedWaveSynthesizer synth;
        synth.Configure(BuildApproximateModel(in), 300.0, 2.0, 0.1);
        RadiationVelocityHistory history(0.1);
        for (int i = 0; i <= 4000; ++i) {
            RadiationBodySample s;
            const double t = 0.1 * i;
            if (t > 18.0 && t < 22.0) {
                s.velocity[2] = 0.5 * (1.0 - std::cos(0.5 * kPi * (t - 18.0)));  // pulse 18..22 s
            }
            history.Add(t, {s});
        }
        synth.Update(history, 15.0);
        TEST_NEAR(synth.Evaluate(100.0, 0.0), 0.0, 1e-12, "nothing before the motion starts");
        // Deep water c_g = g / 2w: in the first 1.5 s only w < 0.025 rad/s (the
        // lowest synthesised frequencies, weight ~ w^0.5) can reach 200 m. The
        // smooth long-wave forerunner after that is physical.
        const double first_arrival = 18.0 + 1.5;
        double early = 0.0;
        double later = 0.0;
        for (double t = 18.0; t < 220.0; t += 0.5) {
            synth.Update(history, t);
            const double v = std::abs(synth.Evaluate(200.0, 0.0));
            if (t <= first_arrival) {
                early = std::max(early, v);
            } else {
                later = std::max(later, v);
            }
        }
        TEST_ASSERT(later > 0.0 && early < 0.005 * later, "waves reach 200 m no faster than c_g allows");

        synth.Update(history, 80.0);
        const double fresh = synth.Evaluate(120.0, 30.0);
        synth.Update(history, 60.0);  // seek back: recompute
        for (double t = 60.0; t <= 80.0 + 1e-9; t += 0.05) {
            synth.Update(history, t);  // play forward: sliding window
        }
        TEST_NEAR(synth.Evaluate(120.0, 30.0), fresh, 1e-9 + 1e-6 * std::abs(fresh),
                  "sliding update equals a fresh recompute (pause / seek safe)");
    }

    // Velocity history: resampling, same-time and rewind.
    {
        RadiationVelocityHistory h(0.1);
        RadiationBodySample a;
        RadiationBodySample b;
        b.velocity[2] = 2.5;
        b.x = 25.0;
        b.yaw = -3.0;
        a.yaw = 3.0;
        h.Add(0.0, {a});
        h.Add(0.25, {b});
        TEST_ASSERT(h.LastIndex() == 2, "grid samples at 0, 0.1, 0.2");
        TEST_NEAR((*h.Sample(1))[0].velocity[2], 1.0, 1e-12, "linear interpolation onto the grid");
        TEST_NEAR((*h.Sample(2))[0].x, 20.0, 1e-12, "position interpolated");
        TEST_NEAR(std::remainder((*h.Sample(1))[0].yaw - 3.0 - 0.4 * (2.0 * kPi - 6.0), 2.0 * kPi), 0.0, 1e-12,
                  "yaw interpolated the short way round");
        const auto gen = h.Generation();
        h.Add(0.25, {a});
        TEST_ASSERT(h.Generation() == gen && h.LastIndex() == 2, "same time is ignored");
        h.Add(0.1, {a});
        TEST_ASSERT(h.Generation() != gen && h.LastIndex() == 0, "rewind restarts the history");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
