/*********************************************************************
 * @file  sea_state_change.cpp
 * @brief Checks for live wave period and direction changes.
 *********************************************************************/

#include <seastack/hydro/waves/sea_state_change.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace seastack::hydro {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHeadingTolRad = 1e-6;
constexpr double kBandRelTol = 1e-9;  // as InterpolateExcitationTransfer

double Deg(double rad) { return rad * 180.0 / kPi; }

/// Smallest angle between two directions [rad], in [0, pi].
double AngleBetween(double a, double b) {
    double d = std::fmod(a - b, kTwoPi);
    if (d < 0.0) d += kTwoPi;
    return std::min(d, kTwoPi - d);
}

bool HeadingCovered(double direction, const Eigen::VectorXd& headings) {
    const Eigen::Index n = headings.size();
    const double first = (n > 0) ? headings[0] : 0.0;
    if (n <= 1) {
        return AngleBetween(direction, first) <= kHeadingTolRad;
    }
    const double last = headings[n - 1];
    double max_step = 0.0;
    for (Eigen::Index h = 1; h < n; ++h) max_step = std::max(max_step, headings[h] - headings[h - 1]);
    // Direction relative to the first heading, in [0, 2 pi).
    double rel = std::fmod(direction - first, kTwoPi);
    if (rel < 0.0) rel += kTwoPi;
    if (rel <= last - first + kHeadingTolRad || rel >= kTwoPi - kHeadingTolRad) {
        return true;
    }
    const double wrap_gap = first + kTwoPi - last;
    return wrap_gap <= max_step + kHeadingTolRad;
}

std::string Percent(double fraction) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3g%%", 100.0 * fraction);
    return buf;
}

std::string Number(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.4g", v);
    return buf;
}

/// Sum of A_i^2 over `proposed`; 1 if zero, so shares stay finite.
double EnergyTotal(const std::vector<WaveComponent>& proposed) {
    double energy_total = 0.0;
    for (const auto& c : proposed) energy_total += c.amplitude * c.amplitude;
    return (energy_total > 0.0) ? energy_total : 1.0;
}

std::string CheckPeriodChange(const std::vector<WaveComponent>& proposed, const SeaStateChangeLimits& limits) {
    const double energy_total = EnergyTotal(proposed);
    if (limits.constant_coefficient_radiation) {
        return "constant-coefficient radiation uses A(w) and B(w) at the starting wave frequency, so the "
               "period cannot change during the run (restart, or use rirf_convolution or state_space "
               "radiation)";
    }

    const Eigen::Index n_freq = limits.bem_frequencies.size();
    if (n_freq > 1) {
        const double w_min = limits.bem_frequencies[0];
        const double w_max = limits.bem_frequencies[n_freq - 1];
        double outside = 0.0;
        for (const auto& c : proposed) {
            if (c.omega < w_min * (1.0 - kBandRelTol) || c.omega > w_max * (1.0 + kBandRelTol)) {
                outside += c.amplitude * c.amplitude;
            }
        }
        if (outside / energy_total > limits.max_energy_outside) {
            return Percent(outside / energy_total) + " of the wave energy would lie outside the BEM frequency "
                   "range " + Number(w_min) + "-" + Number(w_max) + " rad/s (periods " + Number(kTwoPi / w_max) +
                   "-" + Number(kTwoPi / w_min) + " s); the limit is " + Percent(limits.max_energy_outside);
        }
    }
    return "";
}

std::string CheckDirectionChange(const std::vector<WaveComponent>& proposed, const SeaStateChangeLimits& limits) {
    const double energy_total = EnergyTotal(proposed);
    const Eigen::VectorXd& headings = limits.bem_headings;
    if (!limits.heading_kernels && headings.size() > 1) {
        return "IRF excitation: the H5 file has one excitation kernel for " + std::to_string(headings.size()) +
               " BEM headings, so the direction cannot change during the run";
    }
    double outside = 0.0;
    for (const auto& c : proposed) {
        if (!HeadingCovered(c.direction, headings)) outside += c.amplitude * c.amplitude;
    }
    if (outside / energy_total > limits.max_energy_outside) {
        std::string have;
        if (headings.size() <= 1) {
            have = "one BEM heading (" + Number(Deg(headings.size() ? headings[0] : 0.0)) + " deg)";
        } else {
            have = "BEM headings " + Number(Deg(headings[0])) + " to " +
                   Number(Deg(headings[headings.size() - 1])) + " deg";
        }
        return Percent(outside / energy_total) + " of the wave energy would come from directions outside the " +
               have + "; the limit is " + Percent(limits.max_energy_outside);
    }
    return "";
}

}  // namespace

std::string CheckSeaStateChange(SeaStateChangeKind kind, const std::vector<WaveComponent>& proposed,
                                const SeaStateChangeLimits& limits) {
    if (proposed.empty()) {
        return "the new sea state has no wave components";
    }
    switch (kind) {
        case SeaStateChangeKind::kPeriod:
            return CheckPeriodChange(proposed, limits);
        case SeaStateChangeKind::kDirection:
            return CheckDirectionChange(proposed, limits);
    }
    return "";
}

std::string CheckSeaStateChange(const std::vector<WaveComponent>& current,
                                const std::vector<WaveComponent>& proposed,
                                const SeaStateChangeLimits& limits) {
    if (current.size() != proposed.size() || proposed.empty()) {
        return "the new sea state has a different number of wave components (" +
               std::to_string(proposed.size()) + " instead of " + std::to_string(current.size()) +
               "); only period and direction can change";
    }
    bool period_changed = false;
    bool direction_changed = false;
    for (std::size_t i = 0; i < proposed.size(); ++i) {
        period_changed = period_changed || std::abs(proposed[i].omega - current[i].omega) >
                                               1e-12 * std::abs(current[i].omega);
        direction_changed = direction_changed ||
                            AngleBetween(proposed[i].direction, current[i].direction) > 1e-12;
    }
    if (period_changed) {
        const std::string why = CheckPeriodChange(proposed, limits);
        if (!why.empty()) return why;
    }
    if (direction_changed) {
        return CheckDirectionChange(proposed, limits);
    }
    return "";
}

}  // namespace seastack::hydro
