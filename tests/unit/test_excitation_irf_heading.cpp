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
 *   5. Headings that start below 0 (e.g. -30..30 deg) bracket correctly.
 *   6. Every direction the live-change check (CheckSeaStateChange) calls
 *      covered is reproduced by the interpolator's bracket, on several grids.
 *********************************************************************/

#include <seastack/hydro/excitation_transfer.h>
#include <seastack/hydro/force_components/excitation_irf_component.h>
#include <seastack/hydro/waves/sea_state_change.h>

#include <algorithm>
#include <cmath>
#include <utility>
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

void TestNegativeHeadings() {
    std::cout << "\n--- FindHeadingBracket, headings starting below 0 ---\n";
    const double deg = kPi / 180.0;
    Eigen::VectorXd h(2);
    h << -30.0 * deg, 30.0 * deg;
    Check(BracketIs(FindHeadingBracket(h, -15.0 * deg), 0, 1, 0.25), "-15 deg is a quarter from -30 to 30 deg");
    Check(BracketIs(FindHeadingBracket(h, 345.0 * deg), 0, 1, 0.25), "345 deg equals -15 deg");
    Check(BracketIs(FindHeadingBracket(h, 0.0), 0, 1, 0.5), "0 deg is halfway");
    // On the first heading exactly, the wrap bracket puts all weight on it
    // (as for 0 deg on a grid starting at 0).
    Check(BracketIs(FindHeadingBracket(h, -30.0 * deg), 1, 0, 1.0), "-30 deg sits on the first heading");
    // Outside the data: wrap from 30 deg to 330 deg (gap 300 deg).
    Check(BracketIs(FindHeadingBracket(h, 180.0 * deg), 1, 0, 0.5), "180 deg wraps halfway from 30 to 330 deg");

    Eigen::VectorXd sym(5);
    sym << -180.0 * deg, -90.0 * deg, 0.0, 90.0 * deg, 180.0 * deg;
    Check(BracketIs(FindHeadingBracket(sym, -45.0 * deg), 1, 2, 0.5), "-180..180 grid: -45 deg between -90 and 0");
    Check(BracketIs(FindHeadingBracket(sym, 135.0 * deg), 3, 4, 0.5), "-180..180 grid: 135 deg between 90 and 180");
}

/// Direction the bracket interpolates to [rad], in [0, 2 pi).
double BracketDirection(const Eigen::VectorXd& h, const HeadingBracket& b) {
    double lo = h[b.lo];
    double hi = h[b.hi];
    if (hi <= lo) hi += 2.0 * kPi;  // wrap from the last heading to the first + 2 pi
    double d = std::fmod(lo + b.frac * (hi - lo), 2.0 * kPi);
    return d < 0.0 ? d + 2.0 * kPi : d;
}

/// For every direction the live-change check calls covered, the
/// interpolator's bracket must reproduce that direction.
void TestCoverageAgreesWithBracket() {
    std::cout << "\n--- Heading coverage (CheckSeaStateChange) agrees with FindHeadingBracket ---\n";
    const double deg = kPi / 180.0;
    std::vector<std::pair<std::string, Eigen::VectorXd>> grids;
    grids.push_back({"0..90", Eigen::VectorXd::LinSpaced(4, 0.0, 90.0 * deg)});
    grids.push_back({"-30..30", Eigen::VectorXd::LinSpaced(3, -30.0 * deg, 30.0 * deg)});
    grids.push_back({"-180..180", Eigen::VectorXd::LinSpaced(9, -180.0 * deg, 180.0 * deg)});
    grids.push_back({"0..330", Eigen::VectorXd::LinSpaced(12, 0.0, 330.0 * deg)});
    grids.push_back({"-90..60", Eigen::VectorXd::LinSpaced(6, -90.0 * deg, 60.0 * deg)});
    seastack::hydro::SeaStateChangeLimits limits;
    for (const auto& [name, h] : grids) {
        limits.bem_headings = h;
        int covered = 0;
        double worst = 0.0;
        for (int i = -360; i < 720; ++i) {
            const double dir = (i + 0.37) * deg;
            seastack::hydro::WaveComponent c;
            c.amplitude = 1.0;
            c.omega = 1.0;
            c.direction = dir;
            if (!seastack::hydro::CheckSeaStateChange(seastack::hydro::SeaStateChangeKind::kDirection, {c}, limits)
                     .empty()) {
                continue;
            }
            ++covered;
            double want = std::fmod(dir, 2.0 * kPi);
            if (want < 0.0) want += 2.0 * kPi;
            double diff = std::abs(BracketDirection(h, FindHeadingBracket(h, dir)) - want);
            worst = std::max(worst, std::min(diff, 2.0 * kPi - diff));
        }
        Check(covered > 0 && worst < 1e-9, "grid " + name + " deg: " + std::to_string(covered) +
                                               " covered directions, worst bracket error " +
                                               std::to_string(worst / deg) + " deg");
    }
}

}  // namespace

int main() {
    std::cout << "=== test_excitation_irf_heading ===\n";
    TestBracket();
    TestKernelSelection();
    TestNegativeHeadings();
    TestCoverageAgreesWithBracket();
    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED") << " (" << g_failures
              << " failure(s))\n";
    return g_failures == 0 ? 0 : 1;
}
