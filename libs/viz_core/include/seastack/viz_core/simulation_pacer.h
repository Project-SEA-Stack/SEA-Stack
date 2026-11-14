#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace seastack::viz {

/// Decides how many fixed-size physics steps the interactive loop should take
/// per rendered frame so that simulated time follows wall-clock time at a
/// requested playback speed.
///
/// The pacer never changes the integration step dt; it only chooses how many
/// steps of that fixed size to take before the next frame. Slowing down or
/// speeding up playback therefore cannot alter the computed results.
///
/// Speed is dimensionless: simulated seconds per wall-clock second.
/// A speed <= 0 means "unlimited" (step as fast as the hardware allows).
///
/// Time arguments are plain doubles in seconds so the class can be tested
/// with a fake clock. Pacing is anchored: the target simulated time is
/// anchor_sim + speed * (wall - anchor_wall). Call Reset() whenever stepping
/// (re)starts or the speed changes, and after falling behind, so that missed
/// time is dropped rather than replayed as a burst.
class SimulationPacer {
  public:
    static constexpr double kUnlimited = 0.0;

    /// Set the requested playback speed [-]. Values <= 0 select unlimited.
    void SetSpeed(double speed) { speed_ = (speed > 0.0 && std::isfinite(speed)) ? speed : kUnlimited; }
    double Speed() const { return speed_; }
    bool IsUnlimited() const { return speed_ <= 0.0; }

    /// Re-anchor pacing at the current simulated time and wall time [s].
    void Reset(double sim_time, double wall_time) {
        anchor_sim_ = sim_time;
        anchor_wall_ = wall_time;
        anchored_ = true;
    }
    bool IsAnchored() const { return anchored_; }

    /// Simulated time [s] the simulation should have reached at wall_time.
    double TargetSimTime(double wall_time) const {
        if (!anchored_ || IsUnlimited()) {
            return std::numeric_limits<double>::infinity();
        }
        return anchor_sim_ + speed_ * (wall_time - anchor_wall_);
    }

    /// Number of steps of size dt [s] needed to bring sim_time up to the target
    /// at wall_time. Returns a very large number when unlimited (the caller
    /// limits work by its own wall-clock frame budget) and 0 if not anchored.
    int StepsDue(double sim_time, double wall_time, double dt) const {
        if (!anchored_ || !(dt > 0.0)) {
            return 0;
        }
        if (IsUnlimited()) {
            return std::numeric_limits<int>::max();
        }
        const double lag = TargetSimTime(wall_time) - sim_time;
        if (lag <= 0.0) {
            return 0;
        }
        // Small tolerance so that accumulated floating-point time does not
        // drop a step that is due exactly on a frame boundary.
        constexpr double kStepTolerance = 1e-9;
        const double steps = std::floor(lag / dt + kStepTolerance);
        constexpr double kMaxSteps = static_cast<double>(std::numeric_limits<int>::max() / 2);
        return static_cast<int>(std::min(steps, kMaxSteps));
    }

  private:
    double speed_ = 1.0;
    double anchor_sim_ = 0.0;
    double anchor_wall_ = 0.0;
    bool anchored_ = false;
};

/// Accumulates wall-clock time spent actively stepping (excluding pauses,
/// setup and idle time) and measures the achieved playback speed over a
/// short window. Times are seconds.
class RunClock {
  public:
    /// Minimum active wall time [s] between achieved-speed updates.
    static constexpr double kSpeedWindow = 0.5;

    /// Record that the loop was actively stepping from wall_begin to wall_end
    /// and advanced simulated time by sim_advance.
    void AddActiveInterval(double wall_begin, double wall_end, double sim_advance) {
        const double dw = std::max(0.0, wall_end - wall_begin);
        active_wall_ += dw;
        window_wall_ += dw;
        window_sim_ += sim_advance;
        if (window_wall_ >= kSpeedWindow) {
            achieved_speed_ = window_sim_ / window_wall_;
            has_speed_ = true;
            window_wall_ = 0.0;
            window_sim_ = 0.0;
        }
    }

    /// Discard the partial speed window (e.g. after pause or a speed change).
    void ResetSpeedWindow() {
        window_wall_ = 0.0;
        window_sim_ = 0.0;
        has_speed_ = false;
    }

    /// Total active stepping time [s].
    double ActiveWallTime() const { return active_wall_; }
    /// Achieved simulated-time / wall-time ratio over the last window [-].
    double AchievedSpeed() const { return achieved_speed_; }
    bool HasAchievedSpeed() const { return has_speed_; }

  private:
    double active_wall_ = 0.0;
    double window_wall_ = 0.0;
    double window_sim_ = 0.0;
    double achieved_speed_ = 0.0;
    bool has_speed_ = false;
};

/// Snapshot of run progress written by the simulation loop each frame and
/// shown in the GUI status bar. All times in seconds.
struct RunStatus {
    double sim_time = 0.0;          ///< Current simulated time [s]
    double end_time = 0.0;          ///< Configured end time [s]; <= 0 if none
    double time_step = 0.0;         ///< Integration step dt [s] (never changed by pacing)
    double active_wall_time = 0.0;  ///< Wall time spent stepping, pauses excluded [s]
    double achieved_speed = 0.0;    ///< Measured simulated/wall time ratio [-]
    bool has_achieved_speed = false;
    int steps_last_frame = 0;       ///< Physics steps taken in the last frame
    bool falling_behind = false;    ///< True if the requested speed cannot be sustained
    bool finished = false;          ///< True once end_time has been reached
    /// Slow motion only: simulated time the view should show now, between the
    /// last two steps (for visual interpolation) [s]; NaN otherwise.
    double display_time = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace seastack::viz
