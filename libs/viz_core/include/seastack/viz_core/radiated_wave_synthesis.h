#pragma once

// Radiated-wave overlay: velocity history and time-domain synthesis.
//
// The radiated field is a causal linear filter of each body's velocity
// history. Per body s and angular harmonic m (radiated_waves.h):
//
//   eta(r, theta, t) = Re sum_m e^{i m theta} q_m(r, t),
//   q_m(r, t) = (1/pi) sum_n [sum_j a_jm(w_n) U_j(w_n)] F_|m|(k_n r) e^{i w_n t} dw,
//   U_j(w_n) = sum over the last T_w seconds of u_j(t') e^{-i w_n t'} dt',
//
// with w_n = n dw, dw = pi / T_w, and T_w long enough for the slowest drawn
// waves to cross the patch. It depends only on the history and t, so pause,
// speed changes and seeking give the same picture. The spike checked it
// against exact convolution (0.05-0.24% RMS, s08_cost.py).

#include <seastack/viz_core/radiated_waves.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace seastack::viz {

/// State of one radiating body at one instant.
struct RadiationBodySample {
    double x = 0.0;    ///< Centre of mass, world x [m]
    double y = 0.0;    ///< Centre of mass, world y [m]
    double yaw = 0.0;  ///< Heading-frame yaw [rad]
    /// Heading frame: surge, sway, heave [m/s], then roll, pitch, yaw rates [rad/s].
    std::array<double, 6> velocity{};
};

/// Body states on a fixed time grid t_i = t_0 + i dt, linearly interpolated
/// from samples added at any spacing (e.g. every solver step). A sample at an
/// earlier time than the last one restarts the history.
class RadiationVelocityHistory {
  public:
    /// dt: grid step [s]; max_duration: oldest samples are dropped beyond this [s].
    explicit RadiationVelocityHistory(double dt = 0.1, double max_duration = 3600.0)
        : dt_(dt), capacity_(static_cast<size_t>(std::ceil(max_duration / dt)) + 1) {}

    void Clear() {
        samples_.clear();
        has_last_ = false;
        first_index_ = 0;
        ++generation_;
    }

    void Add(double t, const std::vector<RadiationBodySample>& bodies) {
        constexpr double kSameTime = 1e-9;  // [s]
        if (has_last_ && (t < last_time_ - kSameTime || bodies.size() != last_.size())) {
            Clear();
        }
        if (!has_last_) {
            t0_ = t;
            first_index_ = 0;
            samples_.push_back(bodies);
            last_time_ = t;
            last_ = bodies;
            has_last_ = true;
            return;
        }
        if (t <= last_time_ + kSameTime) {
            return;
        }
        for (int64_t i = LastIndex() + 1; TimeOf(i) <= t + kSameTime; ++i) {
            const double a = std::clamp((TimeOf(i) - last_time_) / (t - last_time_), 0.0, 1.0);
            std::vector<RadiationBodySample> s(bodies.size());
            for (size_t b = 0; b < bodies.size(); ++b) {
                const RadiationBodySample& p = last_[b];
                const RadiationBodySample& q = bodies[b];
                s[b].x = p.x + a * (q.x - p.x);
                s[b].y = p.y + a * (q.y - p.y);
                s[b].yaw = p.yaw + a * std::remainder(q.yaw - p.yaw, 2.0 * radiation_detail::kPi);
                for (int j = 0; j < 6; ++j) {
                    s[b].velocity[j] = p.velocity[j] + a * (q.velocity[j] - p.velocity[j]);
                }
            }
            samples_.push_back(std::move(s));
            if (samples_.size() > capacity_) {
                samples_.pop_front();
                ++first_index_;
            }
        }
        last_time_ = t;
        last_ = bodies;
    }

    bool Empty() const { return samples_.empty(); }
    size_t NumBodies() const { return has_last_ ? last_.size() : 0; }
    double Dt() const { return dt_; }
    int64_t FirstIndex() const { return first_index_; }
    int64_t LastIndex() const { return first_index_ + static_cast<int64_t>(samples_.size()) - 1; }
    double TimeOf(int64_t i) const { return t0_ + static_cast<double>(i) * dt_; }
    /// Index of the last grid sample at or before t (may be outside the stored range).
    int64_t IndexAtOrBefore(double t) const { return static_cast<int64_t>(std::floor((t - t0_) / dt_ + 1e-9)); }
    /// Body states at grid index i, or nullptr if not stored.
    const std::vector<RadiationBodySample>* Sample(int64_t i) const {
        if (samples_.empty() || i < FirstIndex() || i > LastIndex()) {
            return nullptr;
        }
        return &samples_[static_cast<size_t>(i - first_index_)];
    }
    /// Changes whenever the history restarts.
    uint64_t Generation() const { return generation_; }

  private:
    double dt_;
    size_t capacity_;
    double t0_ = 0.0;
    int64_t first_index_ = 0;
    std::deque<std::vector<RadiationBodySample>> samples_;
    bool has_last_ = false;
    double last_time_ = 0.0;
    std::vector<RadiationBodySample> last_;
    uint64_t generation_ = 0;
};

/// Radial profiles of the radiated field, recomputed from the velocity history
/// for each displayed time, and the elevation they give at any point.
class RadiatedWaveSynthesizer {
  public:
    /// Precompute the frequency grid, propagation table and coefficients.
    /// r_max: field drawn out to this distance from each body [m];
    /// omega_cap: highest frequency drawn [rad/s] (also limited by the BEM
    /// range; the top 20% is tapered to zero); history_dt: [s].
    void Configure(const RadiatedWaveModel& model, double r_max, double omega_cap, double history_dt) {
        *this = RadiatedWaveSynthesizer{};
        if (model.Empty() || !(r_max > 0.0) || !(history_dt > 0.0)) {
            return;
        }
        gravity_ = model.gravity;
        depth_ = model.depth;
        omega_max_ = std::min(omega_cap, model.omega.back());
        if (!(omega_max_ > 0.0)) {
            return;
        }
        r_max_ = r_max;
        history_dt_ = history_dt;
        const double k_max = WaveNumber(omega_max_, gravity_, depth_);
        dr_ = 2.0 * radiation_detail::kPi / k_max / 8.0;
        n_r_ = static_cast<size_t>(std::ceil(r_max_ / dr_)) + 2;
        window_ = r_max_ / GroupVelocity(omega_max_, k_max, depth_) + 20.0;
        dw_ = radiation_detail::kPi / window_;
        n_w_ = static_cast<size_t>(std::ceil(omega_max_ / dw_));
        window_samples_ = static_cast<int64_t>(std::ceil(window_ / history_dt_));
        omega_.resize(n_w_);
        for (size_t n = 0; n < n_w_; ++n) {
            omega_[n] = static_cast<double>(n + 1) * dw_;
        }

        max_order_ = 0;
        for (const RadiatedWaveSource& src : model.sources) {
            max_order_ = std::max(max_order_, src.max_order);
        }
        BuildTable();
        BuildCoefficients(model);
        configured_ = true;
    }

    /// Fade the field in from r_body (zero inside) over max(r_body/2, 2 dr), per source.
    void SetSourceRadius(size_t s, double r_body) {
        if (s < sources_.size()) {
            sources_[s].radius = std::max(r_body, 0.0);
        }
    }

    size_t NumSources() const { return sources_.size(); }
    const std::string& SourceName(size_t s) const { return sources_[s].name; }
    bool IsConfigured() const { return configured_; }
    bool HasField() const { return has_field_; }
    double OmegaMax() const { return omega_max_; }
    double Window() const { return window_; }
    size_t NumFrequencies() const { return n_w_; }
    size_t NumRadii() const { return n_r_; }
    double RadialStep() const { return dr_; }
    double MaxRadius() const { return r_max_; }

    /// Recompute the profiles for display time t [s]. Sources are the
    /// history's bodies in order. Returns false (no field) if they do not match.
    bool Update(const RadiationVelocityHistory& history, double t) {
        has_field_ = false;
        if (!configured_ || history.Empty() || history.NumBodies() != sources_.size() ||
            std::abs(history.Dt() - history_dt_) > 1e-12) {
            return false;
        }
        int64_t end = std::min(history.IndexAtOrBefore(t), history.LastIndex());
        if (end < history.FirstIndex()) {
            return false;
        }
        const int64_t kRefreshEvery = 20000;  // full recompute against round-off drift
        const bool fresh = history.Generation() != generation_ || end_index_ == kNoIndex;
        if (fresh || end < end_index_ || end - end_index_ > window_samples_ / 2 || slides_ > kRefreshEvery) {
            Recompute(history, end);
            generation_ = history.Generation();
        } else {
            for (int64_t i = end_index_ + 1; i <= end; ++i) {
                Accumulate(history, i, 1.0);
                Accumulate(history, i - window_samples_, -1.0);
                ++slides_;
            }
        }
        end_index_ = end;

        const std::vector<RadiationBodySample>& now = *history.Sample(end);
        std::vector<Complex> phase_t(n_w_);
        for (size_t n = 0; n < n_w_; ++n) {
            phase_t[n] = std::polar(1.0, omega_[n] * t);
        }
        std::vector<Complex> spectrum(n_w_);
        std::vector<Complex> q(n_r_);
        for (size_t s = 0; s < sources_.size(); ++s) {
            Source& src = sources_[s];
            src.x = now[s].x;
            src.y = now[s].y;
            src.yaw = now[s].yaw;
            const int order = src.max_order;
            std::fill(src.cos_profile.begin(), src.cos_profile.end(), 0.0f);
            std::fill(src.sin_profile.begin(), src.sin_profile.end(), 0.0f);
            for (int m = -order; m <= order; ++m) {
                bool any = false;
                for (size_t n = 0; n < n_w_; ++n) {
                    Complex sum(0.0, 0.0);
                    for (int j = 0; j < 6; ++j) {
                        sum += src.coefficients[CoefIndex(order, n, m, j)] * src.spectra[j * n_w_ + n];
                    }
                    spectrum[n] = sum * phase_t[n];
                    any = any || sum != Complex(0.0, 0.0);
                }
                if (!any) {
                    continue;
                }
                std::fill(q.begin(), q.end(), Complex(0.0, 0.0));
                const std::vector<std::complex<float>>& table = tables_[static_cast<size_t>(std::abs(m))];
                for (size_t n = 0; n < n_w_; ++n) {
                    const Complex sn = spectrum[n];
                    const std::complex<float>* row = &table[n * n_r_];
                    for (size_t i = 0; i < n_r_; ++i) {
                        q[i] += sn * Complex(row[i].real(), row[i].imag());
                    }
                }
                // Re{q_m e^{im t} + q_-m e^{-im t}} = Re(q_m + q_-m) cos mt + Im(q_-m - q_m) sin mt.
                const size_t base = static_cast<size_t>(std::abs(m)) * n_r_;
                for (size_t i = 0; i < n_r_; ++i) {
                    src.cos_profile[base + i] += static_cast<float>(q[i].real());
                    if (m != 0) {
                        src.sin_profile[base + i] += static_cast<float>((m < 0 ? 1.0 : -1.0) * q[i].imag());
                    }
                }
            }
        }
        has_field_ = true;
        return true;
    }

    /// Radiated elevation [m] at world point (x, y), summed over sources.
    double Evaluate(double x, double y) const {
        if (!has_field_) {
            return 0.0;
        }
        double eta = 0.0;
        for (const Source& src : sources_) {
            eta += EvaluateSource(src, x, y);
        }
        return eta;
    }

  private:
    struct Source {
        std::string name;
        int max_order = 0;
        double radius = 0.0;  ///< Near-body fade starts here [m]
        /// a_jm(w_n) * taper * dw / pi at CoefIndex(max_order, n, m, j).
        std::vector<Complex> coefficients;
        /// U_j(w_n) [m or rad] at j * n_w + n.
        std::vector<Complex> spectra;
        /// Profiles C_m(r_i), S_m(r_i) [m] at m * n_r + i, m = 0..max_order.
        std::vector<float> cos_profile;
        std::vector<float> sin_profile;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    static size_t CoefIndex(int order, size_t n, int m, int j) {
        return (n * (2 * order + 1) + static_cast<size_t>(m + order)) * 6 + static_cast<size_t>(j);
    }

    /// P_m(k_n r_i) = F_m(k_n r_i), faded in over x = k r in [x0, x0 + 1] with
    /// x0 = max(m, 0.5): inside it the expansion is not meaningful (|H_m| grows
    /// like x^-m).
    void BuildTable() {
        tables_.assign(static_cast<size_t>(max_order_) + 1, std::vector<std::complex<float>>(n_w_ * n_r_));
        std::vector<Complex> f(static_cast<size_t>(max_order_) + 2);
        for (size_t n = 0; n < n_w_; ++n) {
            const double k = WaveNumber(omega_[n], gravity_, depth_);
            for (size_t i = 1; i < n_r_; ++i) {
                const double x = k * static_cast<double>(i) * dr_;
                if (!(x > 0.0)) {
                    continue;
                }
                OutgoingRadialFactors(std::max(max_order_, 1), x, f.data());
                for (int m = 0; m <= max_order_; ++m) {
                    const double x0 = std::max(static_cast<double>(m), 0.5);
                    const double s = std::clamp(x - x0, 0.0, 1.0);
                    const double fade = s * s * (3.0 - 2.0 * s);
                    if (fade > 0.0) {
                        const Complex v = f[m] * fade;
                        tables_[m][n * n_r_ + i] = std::complex<float>(static_cast<float>(v.real()),
                                                                       static_cast<float>(v.imag()));
                    }
                }
            }
        }
    }

    void BuildCoefficients(const RadiatedWaveModel& model) {
        const std::vector<double>& wb = model.omega;
        const double taper_start = 0.8 * omega_max_;
        for (const RadiatedWaveSource& in : model.sources) {
            Source src;
            src.name = in.name;
            src.max_order = in.max_order;
            const int order = in.max_order;
            src.coefficients.assign(n_w_ * (2 * order + 1) * 6, Complex(0.0, 0.0));
            for (size_t n = 0; n < n_w_; ++n) {
                const double w = omega_[n];
                if (w > wb.back()) {
                    continue;
                }
                double taper = 1.0;
                if (w > taper_start) {
                    taper = 0.5 * (1.0 + std::cos(radiation_detail::kPi * (w - taper_start) / (omega_max_ - taper_start)));
                }
                const double scale = taper * dw_ / radiation_detail::kPi;
                // Linear in w between BEM points; ramp to zero below the first.
                size_t k1 = static_cast<size_t>(std::lower_bound(wb.begin(), wb.end(), w) - wb.begin());
                for (int m = -order; m <= order; ++m) {
                    for (int j = 0; j < 6; ++j) {
                        Complex a;
                        if (k1 == 0) {
                            a = in.A(0, m, j) * (w / wb[0]);
                        } else {
                            const double f = (w - wb[k1 - 1]) / (wb[k1] - wb[k1 - 1]);
                            a = in.A(k1 - 1, m, j) * (1.0 - f) + in.A(k1, m, j) * f;
                        }
                        src.coefficients[CoefIndex(order, n, m, j)] = a * scale;
                    }
                }
            }
            src.spectra.assign(6 * n_w_, Complex(0.0, 0.0));
            src.cos_profile.assign((static_cast<size_t>(order) + 1) * n_r_, 0.0f);
            src.sin_profile.assign((static_cast<size_t>(order) + 1) * n_r_, 0.0f);
            sources_.push_back(std::move(src));
        }
    }

    /// Add (sign = 1) or remove (sign = -1) grid sample i from the window spectra.
    void Accumulate(const RadiationVelocityHistory& history, int64_t i, double sign) {
        const std::vector<RadiationBodySample>* sample = history.Sample(i);
        if (sample == nullptr) {
            return;  // before the history started: the bodies were at rest
        }
        const double ti = history.TimeOf(i);
        for (size_t n = 0; n < n_w_; ++n) {
            // std::polar is undefined for a negative magnitude; libc++ returns NaN.
            // sign is +1 to add a sample and -1 to drop one from the sliding window.
            const Complex ph = std::polar(history_dt_, -omega_[n] * ti) * sign;
            for (size_t s = 0; s < sources_.size(); ++s) {
                const std::array<double, 6>& u = (*sample)[s].velocity;
                Complex* spectra = sources_[s].spectra.data();
                for (int j = 0; j < 6; ++j) {
                    spectra[j * n_w_ + n] += u[j] * ph;
                }
            }
        }
    }

    void Recompute(const RadiationVelocityHistory& history, int64_t end) {
        for (Source& src : sources_) {
            std::fill(src.spectra.begin(), src.spectra.end(), Complex(0.0, 0.0));
        }
        for (int64_t i = std::max(end - window_samples_ + 1, history.FirstIndex()); i <= end; ++i) {
            Accumulate(history, i, 1.0);
        }
        slides_ = 0;
    }

    double EvaluateSource(const Source& src, double x, double y) const {
        const double dx = x - src.x;
        const double dy = y - src.y;
        const double r = std::sqrt(dx * dx + dy * dy);
        if (r <= src.radius || r >= r_max_) {
            return 0.0;
        }
        const double width = std::max(0.5 * src.radius, 2.0 * dr_);
        const double s = std::clamp((r - src.radius) / width, 0.0, 1.0);
        const double fade = s * s * (3.0 - 2.0 * s);
        const double pos = r / dr_;
        const size_t i0 = std::min(static_cast<size_t>(pos), n_r_ - 2);
        const double f = pos - static_cast<double>(i0);
        const double theta = std::atan2(dy, dx) - src.yaw;
        const double c1 = std::cos(theta);
        const double s1 = std::sin(theta);
        double cm = 1.0;  // cos(m theta), sin(m theta) by recurrence
        double sm = 0.0;
        double sum = 0.0;
        for (int m = 0; m <= src.max_order; ++m) {
            const size_t base = static_cast<size_t>(m) * n_r_ + i0;
            const double c = src.cos_profile[base] * (1.0 - f) + src.cos_profile[base + 1] * f;
            const double sn = src.sin_profile[base] * (1.0 - f) + src.sin_profile[base + 1] * f;
            sum += c * cm + sn * sm;
            const double c_next = cm * c1 - sm * s1;
            sm = sm * c1 + cm * s1;
            cm = c_next;
        }
        return fade * sum;
    }

    bool configured_ = false;
    bool has_field_ = false;
    double gravity_ = 9.81;
    double depth_ = 0.0;
    double omega_max_ = 0.0;
    double r_max_ = 0.0;
    double dr_ = 1.0;
    size_t n_r_ = 0;
    double window_ = 0.0;
    double dw_ = 0.0;
    size_t n_w_ = 0;
    double history_dt_ = 0.1;
    int64_t window_samples_ = 0;
    int max_order_ = 0;
    std::vector<double> omega_;
    std::vector<std::vector<std::complex<float>>> tables_;  ///< Per |m|: P_m at n * n_r + i
    std::vector<Source> sources_;
    static constexpr int64_t kNoIndex = std::numeric_limits<int64_t>::min();
    uint64_t generation_ = std::numeric_limits<uint64_t>::max();
    int64_t end_index_ = kNoIndex;
    int64_t slides_ = 0;
};

}  // namespace seastack::viz
