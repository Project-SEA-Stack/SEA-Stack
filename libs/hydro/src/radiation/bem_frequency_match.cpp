/*********************************************************************
 * @file  bem_frequency_match.cpp
 * @brief Implementation of MatchBemFrequency.
 *********************************************************************/

#include <seastack/hydro/radiation/bem_frequency_match.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace seastack::hydro {

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

double PeriodOf(double omega) { return kTwoPi / omega; }

/// Appends "T = ... s (w = ... rad/s)" to `oss`.
void AppendPeriodAndFrequency(std::ostringstream& oss, double omega) {
    oss << "T = " << std::setprecision(10) << PeriodOf(omega) << " s (w = "
        << std::setprecision(10) << omega << " rad/s)";
}

/// Appends the available BEM periods, ascending in period.
void AppendAvailablePeriods(std::ostringstream& oss, const Eigen::VectorXd& bem_omegas) {
    std::vector<double> periods;
    periods.reserve(static_cast<size_t>(bem_omegas.size()));
    for (Eigen::Index k = 0; k < bem_omegas.size(); ++k) {
        if (bem_omegas[k] > 0.0) {
            periods.push_back(PeriodOf(bem_omegas[k]));
        }
    }
    std::sort(periods.begin(), periods.end());
    oss << "Available periods (" << periods.size() << "): ";
    constexpr size_t kMaxListed = 40;
    for (size_t i = 0; i < periods.size(); ++i) {
        if (periods.size() > kMaxListed && i == kMaxListed / 2) {
            oss << "..., ";
            i = periods.size() - kMaxListed / 2;
        }
        oss << std::setprecision(7) << periods[i] << (i + 1 < periods.size() ? ", " : " s");
    }
}

}  // namespace

BemFrequencyMatch MatchBemFrequency(const Eigen::VectorXd& bem_omegas,
                                    double omega_requested,
                                    double relative_tolerance) {
    if (bem_omegas.size() == 0) {
        throw std::runtime_error(
            "Constant-coefficient radiation: the BEM frequency list "
            "(simulation_parameters/w) is empty.");
    }
    if (!(omega_requested > 0.0) || !std::isfinite(omega_requested)) {
        throw std::runtime_error(
            "Constant-coefficient radiation: the regular-wave frequency must be finite "
            "and > 0 (got w = " + std::to_string(omega_requested) + " rad/s).");
    }
    if (!(relative_tolerance > 0.0) || !std::isfinite(relative_tolerance)) {
        throw std::runtime_error(
            "Constant-coefficient radiation: frequency_tolerance must be finite and > 0 "
            "(got " + std::to_string(relative_tolerance) + ").");
    }

    const Eigen::Index n = bem_omegas.size();
    const double omega_min = bem_omegas.minCoeff();
    const double omega_max = bem_omegas.maxCoeff();

    int nearest = 0;
    std::vector<int> matches;
    for (Eigen::Index k = 0; k < n; ++k) {
        const double diff = std::abs(bem_omegas[k] - omega_requested);
        if (diff < std::abs(bem_omegas[nearest] - omega_requested)) {
            nearest = static_cast<int>(k);
        }
        if (bem_omegas[k] > 0.0 && diff <= relative_tolerance * bem_omegas[k]) {
            matches.push_back(static_cast<int>(k));
        }
    }

    if (matches.size() == 1) {
        BemFrequencyMatch m;
        m.index = matches[0];
        m.omega_requested = omega_requested;
        m.omega_matched = bem_omegas[m.index];
        m.relative_difference = std::abs(m.omega_matched - omega_requested) / m.omega_matched;
        return m;
    }

    std::ostringstream oss;
    oss << "Regular-wave period ";
    AppendPeriodAndFrequency(oss, omega_requested);
    if (matches.size() > 1) {
        oss << " matches " << matches.size()
            << " BEM grid frequencies within the matching tolerance; the tolerance is "
               "too large for this grid.\n";
    } else if (omega_requested < omega_min || omega_requested > omega_max) {
        oss << " is outside the BEM frequency range [" << std::setprecision(10) << omega_min
            << ", " << omega_max << "] rad/s (periods " << std::setprecision(7)
            << PeriodOf(omega_max) << "-" << PeriodOf(omega_min)
            << " s). Constant-coefficient radiation does not extrapolate.\n";
    } else {
        oss << " has no matching hydrodynamic coefficient in the BEM grid. "
               "Constant-coefficient radiation does not interpolate.\n";
    }
    oss << "Nearest available: ";
    AppendPeriodAndFrequency(oss, bem_omegas[nearest]);
    oss << ", relative difference " << std::setprecision(3)
        << std::abs(bem_omegas[nearest] - omega_requested) / bem_omegas[nearest] << "\n";
    oss << "Matching tolerance (relative): " << std::setprecision(3) << relative_tolerance << "\n";
    AppendAvailablePeriods(oss, bem_omegas);
    throw std::runtime_error(oss.str());
}

}  // namespace seastack::hydro
