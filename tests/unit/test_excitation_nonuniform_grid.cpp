/*********************************************************************
 * @file  test_excitation_nonuniform_grid.cpp
 * @brief InterpolateExcitationTransfer on uniform and non-uniform
 *        BEM frequency grids.
 *
 * Grids that are uniform in period (e.g. T = 3..15 s) are strongly
 * non-uniform in omega. The original linear index map read coefficients
 * at the wrong frequencies on such grids. This test checks:
 *   1. Non-uniform grid: exact values at every grid point and linear-in-
 *      omega interpolation between grid points (Cartesian and polar).
 *   2. Out-of-range omega clamps to the end values.
 *   3. Uniform grid (with WEC-Sim-like rounding noise): results are
 *      bit-identical to the original linear index map.
 *********************************************************************/

#include <seastack/hydro/excitation_transfer.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

using seastack::hydro::ExcitationInterpolation;
using seastack::hydro::HydroData;
using seastack::hydro::WaveComponent;

constexpr double kTwoPi = 6.283185307179586;

int g_failures = 0;

void Check(bool condition, const std::string& what) {
    std::cout << (condition ? "  PASS: " : "  FAIL: ") << what << "\n";
    if (!condition) ++g_failures;
}

/// Single-body, single-heading RegularWaveInfo with magnitude m_k and
/// phase p_k at grid point k for every DOF (DOF d scaled by d+1).
HydroData::RegularWaveInfo MakeInfo(const Eigen::VectorXd& omegas, const std::vector<double>& mag,
                                    const std::vector<double>& phase) {
    HydroData::RegularWaveInfo info;
    info.freq_list = omegas;
    const int n = static_cast<int>(omegas.size());
    info.excitation_mag_matrix.resize(6, 1, n);
    info.excitation_phase_matrix.resize(6, 1, n);
    for (int d = 0; d < 6; ++d) {
        for (int k = 0; k < n; ++k) {
            info.excitation_mag_matrix(d, 0, k) = (d + 1) * mag[k];
            info.excitation_phase_matrix(d, 0, k) = phase[k];
        }
    }
    return info;
}

std::vector<WaveComponent> Components(const std::vector<double>& omegas) {
    std::vector<WaveComponent> comps;
    for (double w : omegas) {
        WaveComponent c;
        c.omega = w;
        c.amplitude = 1.0;
        comps.push_back(c);
    }
    return comps;
}

/// The original (pre-fix) linear index map, reproduced for comparison.
void LegacyCartesian(const HydroData::RegularWaveInfo& info, double omega, int dof, double& re,
                     double& im) {
    const auto& w = info.freq_list;
    const Eigen::Index n = w.size();
    const double range = w[n - 1] - w[0];
    double freq_idx = (omega - w[0]) / range * static_cast<double>(n - 1);
    int f_lo = std::clamp(static_cast<int>(std::floor(freq_idx)), 0, static_cast<int>(n - 2));
    int f_hi = f_lo + 1;
    double f_frac = std::clamp(freq_idx - static_cast<double>(f_lo), 0.0, 1.0);
    const double m0 = info.excitation_mag_matrix(dof, 0, f_lo);
    const double p0 = info.excitation_phase_matrix(dof, 0, f_lo);
    const double m1 = info.excitation_mag_matrix(dof, 0, f_hi);
    const double p1 = info.excitation_phase_matrix(dof, 0, f_hi);
    const double re0 = m0 * std::cos(p0), im0 = -m0 * std::sin(p0);
    const double re1 = m1 * std::cos(p1), im1 = -m1 * std::sin(p1);
    re = re0 + f_frac * (re1 - re0);
    im = im0 + f_frac * (im1 - im0);
}

void TestNonUniformGrid() {
    std::cout << "\n[1] Non-uniform grid (uniform in period, T = 15..3 s)\n";
    const int n = 13;
    Eigen::VectorXd w(n);
    std::vector<double> mag(n), phase(n);
    for (int k = 0; k < n; ++k) {
        w[k] = kTwoPi / (15.0 - static_cast<double>(k));  // ascending omega
        mag[k] = 1.0 + 0.5 * k;
        phase[k] = 0.1 * k;
    }
    const std::vector<HydroData::RegularWaveInfo> data = {MakeInfo(w, mag, phase)};
    const Eigen::VectorXd one_heading = Eigen::VectorXd::Zero(1);

    std::vector<double> grid_omegas(w.data(), w.data() + n);
    for (auto method : {ExcitationInterpolation::kCartesian, ExcitationInterpolation::kPolar}) {
        const auto tf = seastack::hydro::InterpolateExcitationTransfer(Components(grid_omegas), data,
                                                                       one_heading, method);
        double err = 0.0;
        for (int k = 0; k < n; ++k) {
            for (int d = 0; d < 6; ++d) {
                const double m = (d + 1) * mag[k];
                err = std::max(err, std::abs(tf.re[0](d, k) - m * std::cos(phase[k])));
                err = std::max(err, std::abs(tf.im[0](d, k) + m * std::sin(phase[k])));
            }
        }
        Check(err < 1e-12, std::string("exact at all 13 grid points (") +
                               (method == ExcitationInterpolation::kPolar ? "polar" : "cartesian") + ")");
    }

    // Midpoint in omega between grid points 6 and 7 (T = 9 s and 8 s).
    const double w_mid = 0.5 * (w[6] + w[7]);
    const auto tf = seastack::hydro::InterpolateExcitationTransfer(
        Components({w_mid}), data, one_heading, ExcitationInterpolation::kPolar);
    const double mag_mid = 0.5 * (mag[6] + mag[7]);
    const double ph_mid = 0.5 * (phase[6] + phase[7]);
    Check(std::abs(tf.re[0](0, 0) - mag_mid * std::cos(ph_mid)) < 1e-12 &&
              std::abs(tf.im[0](0, 0) + mag_mid * std::sin(ph_mid)) < 1e-12,
          "linear in omega between bracketing grid points (polar midpoint)");

    // T = 8 s is grid point 7; the original index map read data near T = 6 s.
    const auto tf8 = seastack::hydro::InterpolateExcitationTransfer(
        Components({kTwoPi / 8.0}), data, one_heading, ExcitationInterpolation::kPolar);
    Check(std::abs(tf8.re[0](0, 0) - mag[7] * std::cos(phase[7])) < 1e-12,
          "T = 8 s returns the T = 8 s coefficient");

    const auto tf_out = seastack::hydro::InterpolateExcitationTransfer(
        Components({0.5 * w[0], 2.0 * w[n - 1]}), data, one_heading, ExcitationInterpolation::kPolar);
    Check(std::abs(tf_out.re[0](0, 0) - mag[0] * std::cos(phase[0])) < 1e-12 &&
              std::abs(tf_out.re[0](0, 1) - mag[n - 1] * std::cos(phase[n - 1])) < 1e-12,
          "out-of-range omega clamps to the end values");
}

void TestUniformGridBitIdentical() {
    std::cout << "\n[2] Uniform grid: bit-identical to the original index map\n";
    const int n = 240;
    Eigen::VectorXd w(n);
    std::vector<double> mag(n), phase(n);
    for (int k = 0; k < n; ++k) {
        // 0.05:0.05:12 rad/s with ~5e-7 relative rounding noise (as in sphere.h5).
        const double noise = 5e-7 * std::sin(1.3 * k + 0.2);
        w[k] = 0.05 * (k + 1) * (1.0 + noise);
        mag[k] = 1.0 + std::sin(0.07 * k);
        phase[k] = std::cos(0.05 * k);
    }
    const std::vector<HydroData::RegularWaveInfo> data = {MakeInfo(w, mag, phase)};
    std::vector<double> omegas;
    for (double x = 0.01; x < 12.5; x += 0.0137) omegas.push_back(x);
    const auto tf = seastack::hydro::InterpolateExcitationTransfer(
        Components(omegas), data, Eigen::VectorXd::Zero(1), ExcitationInterpolation::kCartesian);
    bool identical = true;
    for (std::size_t i = 0; i < omegas.size(); ++i) {
        for (int d = 0; d < 6; ++d) {
            double re = 0.0, im = 0.0;
            LegacyCartesian(data[0], omegas[i], d, re, im);
            identical = identical && tf.re[0](d, static_cast<Eigen::Index>(i)) == re &&
                        tf.im[0](d, static_cast<Eigen::Index>(i)) == im;
        }
    }
    Check(identical, "uniform grid with rounding noise: results bit-identical to legacy map");
}

}  // namespace

int main() {
    std::cout << "=== test_excitation_nonuniform_grid ===\n";
    TestNonUniformGrid();
    TestUniformGridBitIdentical();
    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED") << " (" << g_failures
              << " failure(s))\n";
    return g_failures == 0 ? 0 : 1;
}
