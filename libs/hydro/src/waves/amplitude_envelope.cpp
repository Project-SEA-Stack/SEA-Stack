/*********************************************************************
 * @file  amplitude_envelope.cpp
 * @brief Runtime wave amplitude envelope e(t).
 *********************************************************************/

#include <seastack/hydro/waves/amplitude_envelope.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace seastack::hydro {

namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double AmplitudeEnvelope::Value(double time) const {
    if (segments_.empty() || time < segments_.front().t0) {
        return 1.0;
    }
    // Last segment that started at or before `time`.
    auto it = std::upper_bound(segments_.begin(), segments_.end(), time,
                               [](double t, const Segment& s) { return t < s.t0; });
    const Segment& s = *std::prev(it);
    if (time >= s.t1) {
        return s.v1;
    }
    const double u = (time - s.t0) / (s.t1 - s.t0);
    return s.v0 + (s.v1 - s.v0) * 0.5 * (1.0 - std::cos(kPi * u));
}

double AmplitudeEnvelope::RampTo(double start_time, double target, double duration) {
    if (!std::isfinite(start_time) || !std::isfinite(target) || !std::isfinite(duration)) {
        throw std::invalid_argument("AmplitudeEnvelope::RampTo: arguments must be finite");
    }
    if (target < 0.0) {
        throw std::invalid_argument("AmplitudeEnvelope::RampTo: target must be >= 0");
    }
    if (duration < 0.0) {
        throw std::invalid_argument("AmplitudeEnvelope::RampTo: duration must be >= 0");
    }
    if (!segments_.empty() && start_time < segments_.back().t0) {
        throw std::invalid_argument("AmplitudeEnvelope::RampTo: start_time is before the last change");
    }
    // A change at the same time as the previous one replaces it.
    if (!segments_.empty() && segments_.back().t0 == start_time) {
        segments_.pop_back();
    }
    Segment s;
    s.t0 = start_time;
    s.t1 = start_time + duration;
    s.v0 = Value(start_time);
    s.v1 = target;
    segments_.push_back(s);
    return s.t1;
}

}  // namespace seastack::hydro
