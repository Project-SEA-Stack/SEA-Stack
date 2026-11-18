#pragma once

// Radiated-wave overlay: hydrodynamic inputs, methods and far-field patterns.
//
// Visual only; never affects the simulation. The physics was checked in the
// Python spike in scripts/spikes/radiated_waves (README.md there).
//
// Conventions (same as seastack::hydro::HydroData):
//  - Time dependence e^{+i w t}; outgoing waves ~ H_m^(2)(k r).
//  - DOFs per body: surge, sway, heave, roll, pitch, yaw, in the body's
//    heading frame (world frame rotated about +z by the body yaw), about the
//    BEM reference point (properties/cg).
//  - Angles theta (field direction) and beta (incident wave heading, the
//    direction the wave travels towards) are measured from +x, counter-clockwise.
//  - Far field per unit velocity U_j of DOF j [m per m/s or per rad/s]:
//        eta = Re{ A_j(theta) (k r)^(-1/2) e^{i(w t - k r)} },
//        A_j(theta) = sum_m a_jm e^{i m theta},
//    and the radiated power is
//        (rho g c_g / 2k) int |sum_j A_j U_j|^2 dtheta = 1/2 U^H B U.

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace seastack::viz {

using Complex = std::complex<double>;

namespace radiation_detail {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEulerGamma = 0.57721566490153286061;
}  // namespace radiation_detail

// ---------------------------------------------------------------------------
// Linear water waves
// ---------------------------------------------------------------------------

/// True if `depth` [m] means deep water (<= 0 or not finite).
inline bool IsDeepWater(double depth) { return !(depth > 0.0) || !std::isfinite(depth); }

/// Wavenumber k [rad/m] from w^2 = g k tanh(k h); deep water if IsDeepWater(depth).
inline double WaveNumber(double omega, double gravity, double depth) {
    if (!(omega > 0.0) || !(gravity > 0.0)) {
        return 0.0;
    }
    double k = omega * omega / gravity;
    if (IsDeepWater(depth) || k * depth > 20.0) {
        return k;  // tanh(k h) = 1 to double precision
    }
    if (k * depth < 1e-6) {
        k = omega / std::sqrt(gravity * depth);  // shallow-water start
    }
    for (int i = 0; i < 60; ++i) {
        const double th = std::tanh(k * depth);
        const double f = gravity * k * th - omega * omega;
        const double dfdk = gravity * th + gravity * k * depth * (1.0 - th * th);
        const double dk = f / std::max(dfdk, 1e-300);
        k = std::max(k - dk, 0.5 * k);
        if (std::abs(dk) < 1e-14 * k) {
            break;
        }
    }
    return k;
}

/// Group velocity c_g [m/s] for wavenumber k (from WaveNumber).
inline double GroupVelocity(double omega, double k, double depth) {
    if (!(k > 0.0)) {
        return 0.0;
    }
    const double cp = omega / k;
    if (IsDeepWater(depth) || k * depth > 20.0) {
        return 0.5 * cp;
    }
    const double kh2 = 2.0 * k * depth;
    return 0.5 * cp * (1.0 + kh2 / std::sinh(kh2));
}

/// Angular frequency [rad/s] of wavenumber k [rad/m].
inline double DispersionFrequency(double k, double gravity, double depth) {
    const double th = IsDeepWater(depth) ? 1.0 : std::tanh(k * depth);
    return std::sqrt(std::max(gravity * k * th, 0.0));
}

// ---------------------------------------------------------------------------
// Hankel functions of the second kind, integer order, real argument
// ---------------------------------------------------------------------------

/// H_m^(2)(x) = J_m(x) - i Y_m(x) for m = 0..m_max and x > 0, written to
/// out[0..m_max]. J_m by Miller's downward recurrence; Y_0 and Y_1 by their
/// power series (x <= 12) or the Hankel asymptotic expansion (x > 12), then
/// Y_m by upward recurrence. Relative accuracy about 1e-10.
inline void HankelSecondKind(int m_max, double x, Complex* out) {
    using radiation_detail::kEulerGamma;
    using radiation_detail::kPi;
    m_max = std::max(m_max, 1);
    std::vector<double> j_tmp;
    std::array<double, 2> y01{};

    // J_m: Miller's algorithm, normalised with J_0 + 2 sum_k J_2k = 1.
    const int top = std::max(m_max, static_cast<int>(x)) + 20 +
                    static_cast<int>(std::sqrt(40.0 * (std::max(m_max, static_cast<int>(x)) + 1)));
    const int n_start = top + (top % 2);
    j_tmp.assign(static_cast<size_t>(n_start) + 2, 0.0);
    j_tmp[n_start] = 1e-30;
    for (int n = n_start; n >= 1; --n) {
        j_tmp[n - 1] = (2.0 * n / x) * j_tmp[n] - j_tmp[n + 1];
        if (std::abs(j_tmp[n - 1]) > 1e200) {
            for (int i = n - 1; i <= n_start; ++i) {
                j_tmp[i] *= 1e-200;
            }
        }
    }
    double norm = j_tmp[0];
    for (int n = 2; n <= n_start; n += 2) {
        norm += 2.0 * j_tmp[n];
    }

    if (x <= 12.0) {
        // Abramowitz & Stegun 9.1.13 (Y_0) and 9.1.11 with n = 1 (Y_1).
        const double j0 = j_tmp[0] / norm;
        const double j1 = j_tmp[1] / norm;
        const double q = 0.25 * x * x;
        double term = 1.0;  // (x^2/4)^k / (k!)^2
        double harmonic = 0.0;
        double sum0 = 0.0;
        for (int k = 1; k < 200; ++k) {
            term *= q / (static_cast<double>(k) * k);
            harmonic += 1.0 / k;
            const double add = ((k % 2) ? 1.0 : -1.0) * harmonic * term;
            sum0 += add;
            if (std::abs(add) < 1e-17 * std::abs(sum0)) {
                break;
            }
        }
        y01[0] = (2.0 / kPi) * ((std::log(0.5 * x) + kEulerGamma) * j0 + sum0);

        double t1 = 0.5 * x;  // (x/2)^(2k+1) / (k! (k+1)!)
        double h_k = 0.0;     // harmonic number H_k
        double sum1 = 0.0;
        for (int k = 0; k < 200; ++k) {
            if (k > 0) {
                t1 *= q / (static_cast<double>(k) * (k + 1));
                h_k += 1.0 / k;
            }
            const double psi_sum = (-kEulerGamma + h_k) + (-kEulerGamma + h_k + 1.0 / (k + 1));
            const double add = ((k % 2) ? -1.0 : 1.0) * psi_sum * t1;
            sum1 += add;
            if (k > 2 && std::abs(add) < 1e-17 * std::abs(sum1)) {
                break;
            }
        }
        y01[1] = -2.0 / (kPi * x) + (2.0 / kPi) * std::log(0.5 * x) * j1 - sum1 / kPi;
    } else {
        // Hankel asymptotic expansion (A&S 9.2.5 - 9.2.10) for orders 0 and 1.
        for (int nu = 0; nu <= 1; ++nu) {
            const double mu = 4.0 * nu * nu;
            double p = 1.0;
            double qs = 0.0;
            double a = 1.0;
            double last = 1.0;
            for (int k = 1; k < 60; ++k) {
                a *= (mu - (2.0 * k - 1.0) * (2.0 * k - 1.0)) / (k * 8.0 * x);
                if (std::abs(a) > last) {
                    break;  // asymptotic series started to diverge
                }
                last = std::abs(a);
                const double sign = ((k / 2) % 2) ? -1.0 : 1.0;
                if (k % 2) {
                    qs += sign * a;
                } else {
                    p += sign * a;
                }
                if (last < 1e-17) {
                    break;
                }
            }
            const double chi = x - (0.5 * nu + 0.25) * kPi;
            y01[nu] = std::sqrt(2.0 / (kPi * x)) * (p * std::sin(chi) + qs * std::cos(chi));
        }
    }

    double y_prev = y01[0];
    double y_cur = y01[1];
    out[0] = Complex(j_tmp[0] / norm, -y01[0]);
    out[1] = Complex(j_tmp[1] / norm, -y01[1]);
    for (int m = 1; m < m_max; ++m) {
        const double y_next = (2.0 * m / x) * y_cur - y_prev;
        out[m + 1] = Complex(j_tmp[m + 1] / norm, -y_next);
        y_prev = y_cur;
        y_cur = y_next;
    }
}

/// Outgoing radial factor F_m(x) = sqrt(pi/2) e^{-i(pi/4 + m pi/2)} H_m^(2)(x),
/// m >= 0, for m = 0..m_max. Far field F_m -> x^(-1/2) e^{-i x}, so a field
/// sum_m a_m F_|m|(k r) e^{i m theta} has the far field A(theta) (kr)^(-1/2) e^{-ikr}
/// and is an exact outgoing solution.
inline void OutgoingRadialFactors(int m_max, double x, Complex* out) {
    using radiation_detail::kPi;
    HankelSecondKind(m_max, x, out);
    for (int m = 0; m <= m_max; ++m) {
        out[m] *= std::sqrt(0.5 * kPi) * std::polar(1.0, -(0.25 + 0.5 * m) * kPi);
    }
}

// ---------------------------------------------------------------------------
// Methods and inputs
// ---------------------------------------------------------------------------

/// How the radiated waves are computed. kAuto picks the best available one.
enum class RadiatedWaveMethod : int {
    kOff = 0,
    kAuto,
    kApproximate,  ///< Energy-matched point multipoles from B(w)
    kHaskind,      ///< Directional patterns from the BEM excitation (Haskind)
    kKochin,       ///< Kochin functions from the BEM solver (not in the H5 schema yet)
    kNearField,    ///< BEM near-field elevation (not implemented)
};
inline constexpr int kNumRadiatedWaveMethods = 6;

inline const char* RadiatedWaveMethodName(RadiatedWaveMethod method) {
    switch (method) {
        case RadiatedWaveMethod::kOff:
            return "Off";
        case RadiatedWaveMethod::kAuto:
            return "Auto";
        case RadiatedWaveMethod::kApproximate:
            return "Approximate";
        case RadiatedWaveMethod::kHaskind:
            return "Haskind";
        case RadiatedWaveMethod::kKochin:
            return "Kochin";
        case RadiatedWaveMethod::kNearField:
            return "BEM near field";
    }
    return "?";
}

/// One hydrodynamic body of the BEM data.
struct RadiationBodyInputs {
    std::string name;          ///< Body name (matches the scene body)
    double ref_x = 0.0;        ///< BEM reference point x (properties/cg) [m]
    double ref_y = 0.0;        ///< BEM reference point y [m]
    double disp_volume = 0.0;  ///< Displaced volume [m^3]
};

/// BEM data the overlay needs, in SI units and the conventions above. N is the
/// number of bodies; matrices are 6N x 6N with body b's DOFs at 6b..6b+5.
struct RadiationVizInputs {
    double rho = 1025.0;     ///< Water density [kg/m^3]
    double gravity = 9.81;   ///< [m/s^2]
    double depth = 0.0;      ///< Water depth [m]; <= 0 or infinite = deep
    std::vector<double> omega;  ///< BEM frequencies [rad/s], ascending
    std::vector<RadiationBodyInputs> bodies;
    /// Coupled radiation damping B(w_k) [N s/m, N s, N m s/rad], row-major:
    /// damping[k][6N * i + j].
    std::vector<std::vector<double>> damping;
    std::string damping_source;  ///< Where B came from, for the log
    std::vector<double> headings;  ///< Incident wave headings beta_d [rad]
    /// Excitation per unit wave amplitude (e^{+i w t}) [N/m, N m/m]:
    /// excitation[(k * headings.size() + d) * 6N + dof]. Empty if unknown.
    std::vector<Complex> excitation;

    size_t NumDof() const { return 6 * bodies.size(); }
    double B(size_t k, size_t i, size_t j) const { return damping[k][NumDof() * i + j]; }
    Complex X(size_t k, size_t d, size_t dof) const {
        return excitation[(k * headings.size() + d) * NumDof() + dof];
    }
    bool HasDamping() const {
        return !bodies.empty() && !omega.empty() && damping.size() == omega.size() &&
               damping.front().size() == NumDof() * NumDof();
    }
    bool HasExcitation() const {
        return !headings.empty() && excitation.size() == omega.size() * headings.size() * NumDof();
    }
};

/// Largest gap [rad] between the distinct headings around the circle; 2 pi
/// for fewer than two distinct headings.
inline double LargestHeadingGap(const std::vector<double>& headings) {
    using radiation_detail::kPi;
    std::vector<double> h;
    for (double b : headings) {
        double w = std::fmod(b, 2.0 * kPi);
        if (w < 0.0) {
            w += 2.0 * kPi;
        }
        h.push_back(w);
    }
    std::sort(h.begin(), h.end());
    h.erase(std::unique(h.begin(), h.end(), [](double a, double b) { return std::abs(a - b) < 1e-9; }), h.end());
    if (h.size() > 1 && std::abs(h.back() - h.front() - 2.0 * kPi) < 1e-9) {
        h.pop_back();  // 360 deg duplicates 0 deg
    }
    if (h.size() < 2) {
        return 2.0 * kPi;
    }
    double gap = h.front() + 2.0 * kPi - h.back();
    for (size_t i = 1; i < h.size(); ++i) {
        gap = std::max(gap, h[i] - h[i - 1]);
    }
    return gap;
}

/// Haskind patterns need headings around the whole circle, no gap above this.
inline constexpr double kMaxHaskindHeadingGapDeg = 40.0;

struct MethodAvailability {
    bool available = false;
    std::string reason;  ///< Why not (empty if available)
};

inline MethodAvailability CheckAvailability(RadiatedWaveMethod method, const RadiationVizInputs& in) {
    using radiation_detail::kPi;
    MethodAvailability a;
    switch (method) {
        case RadiatedWaveMethod::kOff:
            a.available = true;
            break;
        case RadiatedWaveMethod::kAuto:
        case RadiatedWaveMethod::kApproximate:
            a.available = in.HasDamping();
            if (!a.available) {
                a.reason = "needs radiation damping B(w) or the RIRF K(t) from the BEM file";
            }
            break;
        case RadiatedWaveMethod::kHaskind: {
            if (!in.HasDamping() || !in.HasExcitation()) {
                a.reason = "needs radiation damping and excitation from the BEM file";
                break;
            }
            const double gap_deg = LargestHeadingGap(in.headings) * 180.0 / kPi;
            a.available = gap_deg <= kMaxHaskindHeadingGapDeg;
            if (!a.available) {
                char buf[160];
                std::snprintf(buf, sizeof(buf),
                              "needs excitation for headings all round the body (gaps <= %.0f deg); "
                              "the file has %zu heading(s)",
                              kMaxHaskindHeadingGapDeg, in.headings.size());
                a.reason = buf;
            }
            break;
        }
        case RadiatedWaveMethod::kKochin:
            a.reason = "needs Kochin functions, which the BEMIO H5 file does not store";
            break;
        case RadiatedWaveMethod::kNearField:
            a.reason = "needs the BEM free-surface field; not implemented";
            break;
    }
    return a;
}

/// The method actually used for `requested`: kAuto becomes Haskind if
/// available, else Approximate; an unavailable method becomes kOff.
inline RadiatedWaveMethod ResolveMethod(RadiatedWaveMethod requested, const RadiationVizInputs& in) {
    if (requested == RadiatedWaveMethod::kAuto) {
        if (CheckAvailability(RadiatedWaveMethod::kHaskind, in).available) {
            return RadiatedWaveMethod::kHaskind;
        }
        requested = RadiatedWaveMethod::kApproximate;
    }
    return CheckAvailability(requested, in).available ? requested : RadiatedWaveMethod::kOff;
}

// ---------------------------------------------------------------------------
// Far-field model
// ---------------------------------------------------------------------------

/// Far-field harmonics of one radiating body: a_jm(w_k) for |m| <= max_order.
struct RadiatedWaveSource {
    std::string name;
    double ref_x = 0.0;  ///< BEM reference point the harmonics are about [m]
    double ref_y = 0.0;
    int max_order = 0;   ///< M
    /// a_jm(w_k) [m per m/s or per rad/s] at coefficients[((k * (2M+1)) + m + M) * 6 + j].
    std::vector<Complex> coefficients;

    size_t Index(size_t k, int m, int j) const {
        return (k * (2 * max_order + 1) + static_cast<size_t>(m + max_order)) * 6 + static_cast<size_t>(j);
    }
    Complex A(size_t k, int m, int j) const { return coefficients[Index(k, m, j)]; }
};

struct RadiatedWaveModel {
    RadiatedWaveMethod method = RadiatedWaveMethod::kOff;
    double gravity = 9.81;
    double depth = 0.0;
    std::vector<double> omega;  ///< Frequencies of the coefficients [rad/s]
    std::vector<RadiatedWaveSource> sources;
    bool approximate = true;  ///< Patterns assumed (not from BEM)
    std::string label;        ///< One line for the GUI
    std::vector<std::string> notes;  ///< Diagnostics for the log

    bool Empty() const { return sources.empty() || omega.empty(); }
};

namespace radiation_detail {

inline std::string Format(const char* fmt, double a, double b = 0.0, double c = 0.0) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, a, b, c);
    return buf;
}

/// Power scaling of DOF j of a body with displaced volume V: rotations are
/// compared at the surface speed of a point at L = V^(1/3).
inline double PowerScale(int j, double disp_volume) {
    return (j < 3) ? 1.0 : 1.0 / std::cbrt(std::max(disp_volume, 1e-6));
}

/// int_0^{2 pi} f(theta)^2 dtheta for the assumed patterns: 2 pi for m = 0, pi otherwise.
inline double PatternNorm(int m) { return (m == 0) ? 2.0 * kPi : kPi; }

/// Fraction of the power-scaled self-block (Frobenius, summed over frequency)
/// held by terms between the pattern groups {heave}, {surge, pitch},
/// {sway, roll}, {yaw}; the Approximate method does not draw them.
inline double CrossGroupFraction(const RadiationVizInputs& in, size_t b) {
    static constexpr int kGroup[6] = {1, 2, 0, 2, 1, 3};
    double cross = 0.0;
    double total = 0.0;
    for (size_t k = 0; k < in.omega.size(); ++k) {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                const double s = PowerScale(i, in.bodies[b].disp_volume) * PowerScale(j, in.bodies[b].disp_volume);
                const double v = in.B(k, 6 * b + i, 6 * b + j) * s;
                total += v * v;
                if (kGroup[i] != kGroup[j]) {
                    cross += v * v;
                }
            }
        }
    }
    return total > 0.0 ? std::sqrt(cross / total) : 0.0;
}

/// Fraction of the power-scaled full matrix (Frobenius, summed over frequency)
/// in blocks between different bodies.
inline double BodyCouplingFraction(const RadiationVizInputs& in) {
    const size_t n = in.NumDof();
    double cross = 0.0;
    double total = 0.0;
    for (size_t k = 0; k < in.omega.size(); ++k) {
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                const double s = PowerScale(static_cast<int>(i % 6), in.bodies[i / 6].disp_volume) *
                                 PowerScale(static_cast<int>(j % 6), in.bodies[j / 6].disp_volume);
                const double v = in.B(k, i, j) * s;
                total += v * v;
                if (i / 6 != j / 6) {
                    cross += v * v;
                }
            }
        }
    }
    return total > 0.0 ? std::sqrt(cross / total) : 0.0;
}

/// Quadrature weights [rad] for distinct headings around the circle (each
/// heading gets half of the gap to each neighbour). Duplicates (e.g. 0 and
/// 360 deg) get weight 0.
inline std::vector<double> PeriodicHeadingWeights(const std::vector<double>& headings) {
    const size_t n = headings.size();
    std::vector<double> wrapped(n);
    for (size_t d = 0; d < n; ++d) {
        double w = std::fmod(headings[d], 2.0 * kPi);
        wrapped[d] = (w < 0.0) ? w + 2.0 * kPi : w;
        if (std::abs(wrapped[d] - 2.0 * kPi) < 1e-9) {
            wrapped[d] = 0.0;
        }
    }
    std::vector<size_t> order;
    for (size_t d = 0; d < n; ++d) {
        bool duplicate = false;
        for (size_t e : order) {
            duplicate = duplicate || std::abs(wrapped[e] - wrapped[d]) < 1e-9;
        }
        if (!duplicate) {
            order.push_back(d);
        }
    }
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return wrapped[a] < wrapped[b]; });
    std::vector<double> weights(n, 0.0);
    const size_t m = order.size();
    for (size_t i = 0; i < m; ++i) {
        const double prev = wrapped[order[(i + m - 1) % m]] - (i == 0 ? 2.0 * kPi : 0.0);
        const double next = wrapped[order[(i + 1) % m]] + (i + 1 == m ? 2.0 * kPi : 0.0);
        weights[order[i]] = 0.5 * (next - prev);
    }
    return weights;
}

}  // namespace radiation_detail

/// Approximate method, per body from its own 6x6 damping block:
///  - Patterns (heading frame): heave 1 (m = 0); surge and pitch cos(theta),
///    sway and roll sin(theta) (m = 1); yaw sin(2 theta) (m = 2).
///  - |alpha_j| = sqrt(k B_jj / (rho g c_g int f^2)): the drawn wave carries
///    the power the damping takes from that DOF.
///  - Within the (surge, pitch) and (sway, roll) groups the second DOF gets
///    the relative phase arccos(B_ij / sqrt(B_ii B_jj)). The real part of the
///    drawn 2x2 power matrix then equals the damping block, so the power is
///    exact when the two DOFs move in phase or in antiphase (one shared
///    pattern cannot do better). Terms between groups and between bodies are
///    not drawn (reported in the notes).
///  - Phase: point multipole with real strength, sign(c) e^{i(pi/4 + m pi/2)};
///    heave sign -1, surge and sway +1 (universal at low frequency); yaw +1
///    (no reference: its power is right, its phase is a choice).
inline RadiatedWaveModel BuildApproximateModel(const RadiationVizInputs& in) {
    using namespace radiation_detail;
    RadiatedWaveModel model;
    model.method = RadiatedWaveMethod::kApproximate;
    model.gravity = in.gravity;
    model.depth = in.depth;
    model.omega = in.omega;
    model.approximate = true;
    model.label = "Approximate: point multipoles matched to B(w)";
    const size_t nk = in.omega.size();
    const Complex half_over_i(0.0, -0.5);
    for (size_t b = 0; b < in.bodies.size(); ++b) {
        RadiatedWaveSource src;
        src.name = in.bodies[b].name;
        src.ref_x = in.bodies[b].ref_x;
        src.ref_y = in.bodies[b].ref_y;
        src.max_order = 2;
        src.coefficients.assign(nk * 5 * 6, Complex(0.0, 0.0));
        std::array<double, 6> peak{};
        for (size_t k = 0; k < nk; ++k) {
            for (int j = 0; j < 6; ++j) {
                peak[j] = std::max(peak[j], in.B(k, 6 * b + j, 6 * b + j));
            }
        }
        int negative = 0;  // below -1% of the DOF's peak; smaller ones are numerical noise
        for (size_t k = 0; k < nk; ++k) {
            const double w = in.omega[k];
            const double kw = WaveNumber(w, in.gravity, in.depth);
            const double cg = GroupVelocity(w, kw, in.depth);
            if (!(kw > 0.0) || !(cg > 0.0)) {
                continue;
            }
            auto bjj = [&](int j) { return in.B(k, 6 * b + j, 6 * b + j); };
            auto amp = [&](int j, int m) {
                const double v = bjj(j);
                if (v < -0.01 * peak[j]) {
                    ++negative;
                }
                return std::sqrt(std::max(kw * v, 0.0) / (in.rho * in.gravity * cg * PatternNorm(m)));
            };
            auto rel_phase = [&](int lead, int other) {
                const double d = bjj(lead) * bjj(other);
                if (!(d > 0.0)) {
                    return 0.0;
                }
                const double c = in.B(k, 6 * b + lead, 6 * b + other) / std::sqrt(d);
                return std::acos(std::clamp(c, -1.0, 1.0));
            };
            auto multipole = [&](int m, double sign) { return sign * std::polar(1.0, (0.25 + 0.5 * m) * kPi); };

            const Complex heave = amp(2, 0) * multipole(0, -1.0);
            const Complex surge = amp(0, 1) * multipole(1, 1.0);
            const Complex pitch = amp(4, 1) * multipole(1, 1.0) * std::polar(1.0, rel_phase(0, 4));
            const Complex sway = amp(1, 1) * multipole(1, 1.0);
            const Complex roll = amp(3, 1) * multipole(1, 1.0) * std::polar(1.0, rel_phase(1, 3));
            const Complex yaw = amp(5, 2) * multipole(2, 1.0);

            src.coefficients[src.Index(k, 0, 2)] = heave;
            for (int s : {-1, 1}) {
                src.coefficients[src.Index(k, s, 0)] = 0.5 * surge;
                src.coefficients[src.Index(k, s, 4)] = 0.5 * pitch;
                // sin(t) = (e^{it} - e^{-it}) / 2i, and 1 / 2i = -i / 2; sin(2t) likewise.
                src.coefficients[src.Index(k, s, 1)] = static_cast<double>(s) * sway * half_over_i;
                src.coefficients[src.Index(k, s, 3)] = static_cast<double>(s) * roll * half_over_i;
                src.coefficients[src.Index(k, 2 * s, 5)] = static_cast<double>(s) * yaw * half_over_i;
            }
        }
        model.notes.push_back(src.name + ": coupling between pattern groups (not drawn) " +
                              Format("%.1f%% of the damping", 100.0 * CrossGroupFraction(in, b)));
        if (negative > 0) {
            model.notes.push_back(src.name + Format(": %.0f diagonal B(w) values below -1%% of their peak "
                                                    "(drawn as zero)",
                                                    static_cast<double>(negative)));
        }
        model.sources.push_back(std::move(src));
    }
    if (in.bodies.size() > 1) {
        model.notes.push_back(Format("%.0f bodies superposed, each from its own damping block; "
                                     "interaction between them is not modelled (%.0f%% of the damping "
                                     "is between bodies)",
                                     static_cast<double>(in.bodies.size()), 100.0 * BodyCouplingFraction(in)));
    }
    return model;
}

/// Haskind method: A_j(theta) = -e^{i pi/4} k / (sqrt(8 pi) rho g c_g) X_j(beta = theta + pi),
/// with the phase reference moved from the BEM origin to the body's reference
/// point p by e^{-i k p.e(theta)}, then expanded in harmonics |m| <= M. M is
/// the smallest order holding 98% of the pattern power, at most
/// kMaxHaskindOrder and at most (distinct headings) / 2 - 1: elongated hulls
/// at high k L need high orders, and the heading spacing limits what can be
/// resolved. Includes the interaction with the other bodies (the excitation
/// does). Requires CheckAvailability(kHaskind).
inline constexpr int kMaxHaskindOrder = 24;
inline RadiatedWaveModel BuildHaskindModel(const RadiationVizInputs& in) {
    using namespace radiation_detail;
    RadiatedWaveModel model;
    model.method = RadiatedWaveMethod::kHaskind;
    model.gravity = in.gravity;
    model.depth = in.depth;
    model.omega = in.omega;
    model.approximate = false;
    model.label = "Haskind: directional patterns from the BEM excitation";
    const size_t nk = in.omega.size();
    const size_t nd = in.headings.size();
    const std::vector<double> wq = PeriodicHeadingWeights(in.headings);
    size_t distinct = 0;
    for (double w : wq) {
        distinct += (w > 0.0) ? 1 : 0;
    }
    const int m_cap = std::max(1, std::min<int>(kMaxHaskindOrder, static_cast<int>(distinct) / 2 - 1));
    const Complex haskind_phase = -std::polar(1.0, 0.25 * kPi);
    std::vector<Complex> harmonic(2 * m_cap + 1);  // w_d e^{-i m theta_d} / 2 pi

    for (size_t b = 0; b < in.bodies.size(); ++b) {
        const RadiationBodyInputs& body = in.bodies[b];
        // Harmonics up to m_cap first; then keep the order that holds 98%.
        std::vector<Complex> full(nk * (2 * m_cap + 1) * 6, Complex(0.0, 0.0));
        std::vector<double> power_by_order(m_cap + 1, 0.0);
        auto full_index = [&](size_t k, int m, int j) {
            return (k * (2 * m_cap + 1) + static_cast<size_t>(m + m_cap)) * 6 + static_cast<size_t>(j);
        };
        for (size_t k = 0; k < nk; ++k) {
            const double w = in.omega[k];
            const double kw = WaveNumber(w, in.gravity, in.depth);
            const double cg = GroupVelocity(w, kw, in.depth);
            if (!(kw > 0.0) || !(cg > 0.0)) {
                continue;
            }
            const Complex c = haskind_phase * (kw / (std::sqrt(8.0 * kPi) * in.rho * in.gravity * cg));
            for (size_t d = 0; d < nd; ++d) {
                if (!(wq[d] > 0.0)) {
                    continue;
                }
                const double theta = in.headings[d] + kPi;
                const Complex shift =
                    std::polar(1.0, -kw * (body.ref_x * std::cos(theta) + body.ref_y * std::sin(theta)));
                for (int m = -m_cap; m <= m_cap; ++m) {
                    harmonic[m + m_cap] = std::polar(wq[d] / (2.0 * kPi), -m * theta);
                }
                for (int j = 0; j < 6; ++j) {
                    const Complex a_theta = c * in.X(k, d, 6 * b + j) * shift;
                    for (int m = -m_cap; m <= m_cap; ++m) {
                        full[full_index(k, m, j)] += a_theta * harmonic[m + m_cap];
                    }
                }
            }
            for (int j = 0; j < 6; ++j) {
                const double s = PowerScale(j, body.disp_volume);
                for (int m = -m_cap; m <= m_cap; ++m) {
                    power_by_order[std::abs(m)] += std::norm(full[full_index(k, m, j)]) * s * s;
                }
            }
        }
        double total = 0.0;
        for (double p : power_by_order) {
            total += p;
        }
        int order = m_cap;
        double held = 0.0;
        for (int m = 0; m <= m_cap; ++m) {
            held += power_by_order[m];
            if (total > 0.0 && held >= 0.98 * total) {
                order = std::max(m, 1);
                break;
            }
        }

        RadiatedWaveSource src;
        src.name = body.name;
        src.ref_x = body.ref_x;
        src.ref_y = body.ref_y;
        src.max_order = order;
        src.coefficients.assign(nk * (2 * order + 1) * 6, Complex(0.0, 0.0));
        std::array<double, 6> b_patterns{};
        std::array<double, 6> b_input{};
        std::array<double, 6> b_peak{};
        std::vector<double> implied(nk * 6, 0.0);  // B_jj implied by the drawn pattern at each w_k
        for (size_t k = 0; k < nk; ++k) {
            const double kw = WaveNumber(in.omega[k], in.gravity, in.depth);
            const double cg = GroupVelocity(in.omega[k], kw, in.depth);
            for (int j = 0; j < 6; ++j) {
                double sum = 0.0;
                for (int m = -order; m <= order; ++m) {
                    src.coefficients[src.Index(k, m, j)] = full[full_index(k, m, j)];
                    sum += std::norm(full[full_index(k, m, j)]);
                }
                if (kw > 0.0) {
                    // (rho g c_g / k) int |A_j|^2 dtheta.
                    implied[k * 6 + j] = in.rho * in.gravity * cg / kw * 2.0 * kPi * sum;
                    b_patterns[j] += implied[k * 6 + j];
                }
                const double bjj = std::max(in.B(k, 6 * b + j, 6 * b + j), 0.0);
                b_input[j] += bjj;
                b_peak[j] = std::max(b_peak[j], bjj);
            }
        }
        // Lowest frequency where a significant B_jj (> 10% of its peak) is off by more than 10%.
        double first_mismatch = 0.0;
        for (size_t k = 0; k < nk && first_mismatch == 0.0; ++k) {
            for (int j = 0; j < 6; ++j) {
                const double bjj = in.B(k, 6 * b + j, 6 * b + j);
                if (bjj > 0.1 * b_peak[j] && std::abs(implied[k * 6 + j] / bjj - 1.0) > 0.1) {
                    first_mismatch = in.omega[k];
                    break;
                }
            }
        }
        static constexpr const char* kDof[6] = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
        std::string ratios;
        for (int j = 0; j < 6; ++j) {
            if (b_input[j] > 0.0) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), " %s %.2f", kDof[j], b_patterns[j] / b_input[j]);
                ratios += buf;
            }
        }
        model.notes.push_back(src.name + Format(": M = %.0f harmonics; damping implied by the patterns / B(w):",
                                                static_cast<double>(order)) +
                              ratios);
        if (first_mismatch > 0.0) {
            model.notes.push_back(src.name + Format(": patterns match B(w) within 10%% below %.2f rad/s; above "
                                                    "that, the heading spacing or irregular frequencies limit them",
                                                    first_mismatch));
        }
        model.sources.push_back(std::move(src));
    }
    if (in.bodies.size() > 1) {
        model.notes.push_back("patterns include the interaction between bodies; each body's field is expanded "
                              "about its own reference point, so it is least accurate close to the array");
    }
    return model;
}

/// Model for `requested` (resolved with ResolveMethod); empty for kOff.
inline RadiatedWaveModel BuildRadiatedWaveModel(const RadiationVizInputs& in, RadiatedWaveMethod requested) {
    switch (ResolveMethod(requested, in)) {
        case RadiatedWaveMethod::kApproximate:
            return BuildApproximateModel(in);
        case RadiatedWaveMethod::kHaskind:
            return BuildHaskindModel(in);
        default:
            return RadiatedWaveModel{};
    }
}

/// Radiated power [W] the model's patterns carry for complex velocity
/// amplitudes u (6 per source, heading frame) at frequency index k:
/// (rho g c_g / 2k) int |sum_j A_j u_j|^2 dtheta, summed over sources
/// (no interference between sources). For checks and diagnostics.
inline double PatternPower(const RadiatedWaveModel& model, double rho, size_t k, const std::vector<Complex>& u) {
    using radiation_detail::kPi;
    const double w = model.omega[k];
    const double kw = WaveNumber(w, model.gravity, model.depth);
    const double cg = GroupVelocity(w, kw, model.depth);
    if (!(kw > 0.0)) {
        return 0.0;
    }
    double power = 0.0;
    for (size_t s = 0; s < model.sources.size(); ++s) {
        const RadiatedWaveSource& src = model.sources[s];
        for (int m = -src.max_order; m <= src.max_order; ++m) {
            Complex a(0.0, 0.0);
            for (int j = 0; j < 6; ++j) {
                a += src.A(k, m, j) * u[6 * s + j];
            }
            power += 2.0 * kPi * std::norm(a);  // int |sum_m a_m e^{im t}|^2 dt = 2 pi sum |a_m|^2
        }
    }
    return rho * model.gravity * cg / (2.0 * kw) * power;
}

}  // namespace seastack::viz
