/*********************************************************************
 * @file  test_excitation_irf_heading.cpp
 * @brief Heading selection for excitation: FindHeadingBracket and the
 *        IRF kernel choice in SelectKernelForHeading.
 *
 * The IRF convolution uses the wave elevation at the origin, which does
 * not depend on direction, so the kernel must match the sea's heading.
 * Directions are radians, counter-clockwise from +X. This test checks:
 *   1. Bracket and weight between headings, on headings, and across the
 *      wrap from the last heading to the first heading + 2*pi.
 *   2. A direction on a BEM heading returns that kernel bit-for-bit.
 *   3. Between headings, kernels are blended linearly.
 *   4. Single-heading data and mismatched kernel counts fall back to the
 *      first-heading kernel.
 *********************************************************************/

#include <seastack/hydro/excitation_transfer.h>
#include <seastack/hydro/force_components/excitation_irf_component.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

using seastack::hydro::FindHeadingBracket;
using seastack::hydro::HeadingBracket;
using seastack::hydro::SelectKernelForHeading;

constexpr double kPi = 3.14159265358979323846;

int g_failures = 0;

void Check(bool condition, const std::string& what) {
    std::cout << (condition ? "  PASS: " : "  FAIL: ") << what << "\n";
    if (!condition) ++g_failures;
}

bool Near(double a, double b, double tol = 1e-12) { return std::abs(a - b) <= tol; }

bool BracketIs(const HeadingBracket& b, int lo, int hi, double frac) {
    return b.lo == lo && b.hi == hi && Near(b.frac, frac);
}

/// 6 x n kernel with K(d, s) = scale * (d + 1) * (s + 1).
Eigen::MatrixXd MakeKernel(double scale, int n) {
    Eigen::MatrixXd k(6, n);
    for (int d = 0; d < 6; ++d) {
        for (int s = 0; s < n; ++s) {
            k(d, s) = scale * (d + 1) * (s + 1);
        }
    }
    return k;
}

bool SameBits(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
    return a.rows() == b.rows() && a.cols() == b.cols() && (a.array() == b.array()).all();
}

void TestBracket() {
    std::cout << "\n--- FindHeadingBracket ---\n";
    Eigen::VectorXd h(3);
    h << 0.0, 0.5 * kPi, kPi;  // 0, 90, 180 deg

    Check(BracketIs(FindHeadingBracket(h, 0.25 * kPi), 0, 1, 0.5), "45 deg is halfway between 0 and 90 deg");
    Check(BracketIs(FindHeadingBracket(h, 0.75 * kPi), 1, 2, 0.5), "135 deg is halfway between 90 and 180 deg");
    Check(BracketIs(FindHeadingBracket(h, 0.5 * kPi), 0, 1, 1.0), "90 deg sits on the upper heading (frac = 1)");
    // Beyond 180 deg the bracket wraps: 180 deg -> 360 deg (= 0 deg), gap 180 deg.
    Check(BracketIs(FindHeadingBracket(h, 1.5 * kPi), 2, 0, 0.5), "270 deg wraps halfway from 180 to 360 deg");
    Check(BracketIs(FindHeadingBracket(h, -0.5 * kPi), 2, 0, 0.5), "-90 deg equals 270 deg");
    Check(BracketIs(FindHeadingBracket(h, 2.0 * kPi + 0.25 * kPi), 0, 1, 0.5), "405 deg equals 45 deg");

    Eigen::VectorXd one(1);
    one << 0.0;
    Check(BracketIs(FindHeadingBracket(one, 1.0), 0, 0, 0.0), "single heading always returns index 0");
    Check(BracketIs(FindHeadingBracket(Eigen::VectorXd(), 1.0), 0, 0, 0.0), "no headings returns index 0");
}

void TestKernelSelection() {
    std::cout << "\n--- SelectKernelForHeading ---\n";
    constexpr int kNt = 5;
    Eigen::VectorXd h(2);
    h << 0.0, 0.5 * kPi;
    const std::vector<Eigen::MatrixXd> kernels = {MakeKernel(1.0, kNt), MakeKernel(3.0, kNt)};
    const Eigen::MatrixXd fallback            = kernels[0];

    Check(SameBits(SelectKernelForHeading(kernels, h, 0.0, fallback), kernels[0]), "0 deg returns heading 0 exactly");
    Check(SameBits(SelectKernelForHeading(kernels, h, 0.5 * kPi, fallback), kernels[1]),
          "90 deg returns heading 1 exactly");
    Check(SameBits(SelectKernelForHeading(kernels, h, 2.0 * kPi, fallback), kernels[0]),
          "360 deg returns heading 0 exactly");

    // 30 deg: frac = 1/3, so K = K0 + (K1 - K0)/3 = (1 + 2/3) * base.
    const Eigen::MatrixXd k30 = SelectKernelForHeading(kernels, h, kPi / 6.0, fallback);
    Check((k30 - MakeKernel(1.0 + 2.0 / 3.0, kNt)).cwiseAbs().maxCoeff() < 1e-12, "30 deg blends 2/3 K0 + 1/3 K1");

    // 225 deg: wraps from 90 deg to 360 deg (gap 270 deg), frac = 135/270 = 0.5.
    const Eigen::MatrixXd k225 = SelectKernelForHeading(kernels, h, 1.25 * kPi, fallback);
    Check((k225 - MakeKernel(2.0, kNt)).cwiseAbs().maxCoeff() < 1e-12, "225 deg wraps halfway from K1 back to K0");

    Eigen::VectorXd one(1);
    one << 0.0;
    Check(SameBits(SelectKernelForHeading({kernels[0]}, one, 0.3, fallback), fallback),
          "single heading returns the fallback kernel");
    Check(SameBits(SelectKernelForHeading({kernels[0]}, h, 0.3, fallback), fallback),
          "kernel count != heading count returns the fallback kernel");
}

}  // namespace

int main() {
    std::cout << "=== test_excitation_irf_heading ===\n";
    TestBracket();
    TestKernelSelection();
    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED") << " (" << g_failures
              << " failure(s))\n";
    return g_failures == 0 ? 0 : 1;
}
