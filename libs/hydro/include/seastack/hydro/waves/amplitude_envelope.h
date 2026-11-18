/*********************************************************************
 * @file  amplitude_envelope.h
 * @brief Runtime wave amplitude envelope e(t) for live wave-height changes.
 *
 * e(t) [-] multiplies every incident-wave component amplitude in the force
 * calculations and in the drawn sea, on top of the start-up excitation ramp:
 *   eta(x, t) = e(t) * sum_i A_i cos(k_i.x - w_i t + phi_i)
 * so H (regular) and Hs (spectral) both scale by e. In linear theory the
 * excitation scales by e as well, which is exact once e is constant.
 *
 * e(t) = 1 until the first change, so unchanged runs are bit-identical to
 * runs without an envelope. A change at time t0 to target v1 ramps from the
 * current value with a half cosine over the transition duration T:
 *   e(t) = v0 + (v1 - v0) * 0.5 * (1 - cos(pi (t - t0) / T)),  t0 <= t <= t0 + T
 * Past segments are kept so that IRF excitation, which convolves the
 * elevation history, sees the envelope that was in effect at each past time.
 *
 * The same ramp serves as the weight of a component set in a live period or
 * direction crossfade (WaveLayer in wave_base.h); a set that fades in starts
 * at 0.
 *********************************************************************/

#ifndef SEASTACK_HYDRO_WAVES_AMPLITUDE_ENVELOPE_H
#define SEASTACK_HYDRO_WAVES_AMPLITUDE_ENVELOPE_H

#include <vector>

namespace seastack::hydro {

class AmplitudeEnvelope {
  public:
    /// @param initial_value e before the first change [-], >= 0.
    explicit AmplitudeEnvelope(double initial_value = 1.0);

    /// e(t) [-]. The initial value (1 by default) before the first change.
    double Value(double time) const;

    /// True once any change was made (callers may skip work while false).
    bool HasChanges() const { return !segments_.empty(); }

    /// Ramp from the current value e(start_time) to `target` [-] over
    /// `duration` [s] (0 = step). Later segments replace any segment still in
    /// progress from start_time on. Returns the time at which e reaches target.
    /// @throws std::invalid_argument if target < 0, duration < 0, either is not
    ///         finite, or start_time is before the last change.
    double RampTo(double start_time, double target, double duration);

    /// Value e reaches once the last change has finished [-].
    double Target() const { return segments_.empty() ? initial_value_ : segments_.back().v1; }

    /// Time [s] at which the last change ends; -inf before the first change.
    double EndTime() const;

  private:
    double initial_value_ = 1.0;
    struct Segment {
        double t0 = 0.0;  ///< Start [s]
        double t1 = 0.0;  ///< End [s]; t1 >= t0
        double v0 = 1.0;  ///< e at t0 [-]
        double v1 = 1.0;  ///< e from t1 on [-]
    };
    std::vector<Segment> segments_;  ///< Sorted by t0
};

}  // namespace seastack::hydro

#endif  // SEASTACK_HYDRO_WAVES_AMPLITUDE_ENVELOPE_H
