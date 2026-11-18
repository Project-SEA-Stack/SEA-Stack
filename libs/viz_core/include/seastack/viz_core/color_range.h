#pragma once

#include <algorithm>
#include <cmath>

namespace seastack::viz {

/// How the colour range of a scalar overlay (e.g. mooring tension [N]) is chosen.
enum class ColorRangeMode {
    kAuto,    ///< Track the data: snap up to new peaks, hold, then decay slowly
    kLocked,  ///< Freeze the range at its current values
    kFixed,   ///< User-set min/max (from YAML or the GUI)
};

inline const char* ColorRangeModeName(ColorRangeMode mode) {
    switch (mode) {
        case ColorRangeMode::kLocked:
            return "locked";
        case ColorRangeMode::kFixed:
            return "fixed";
        case ColorRangeMode::kAuto:
        default:
            return "auto";
    }
}

/// Colour range for a non-negative scalar such as tension magnitude.
///
/// Auto mode keeps min at 0 and uses a high-water mark for max: it snaps up
/// immediately to a new peak (plus 10% padding), holds for kHoldFrames
/// updates, then decays very slowly towards the current padded peak so the
/// colour scale stays stable while watching. Units are those of the data.
class ColorRange {
  public:
    static constexpr float kPadding = 0.10f;
    static constexpr int kHoldFrames = 180;
    static constexpr float kDecayAlpha = 0.00008f;
    static constexpr float kMinSpan = 1e-6f;

    /// Switch mode. Locking freezes the range currently shown, whichever
    /// mode it came from.
    void SetMode(ColorRangeMode mode) {
        if (mode == ColorRangeMode::kLocked && mode_ != ColorRangeMode::kLocked) {
            locked_min_ = Min();
            locked_max_ = Max();
        }
        mode_ = mode;
    }
    ColorRangeMode Mode() const { return mode_; }

    /// Set the fixed range used in kFixed mode. Invalid input (non-finite,
    /// or max <= min) is rejected and returns false.
    bool SetFixedRange(float min_value, float max_value) {
        if (!std::isfinite(min_value) || !std::isfinite(max_value) || !(max_value > min_value)) {
            return false;
        }
        fixed_min_ = min_value;
        fixed_max_ = max_value;
        has_fixed_ = true;
        return true;
    }
    bool HasFixedRange() const { return has_fixed_; }
    float FixedMin() const { return fixed_min_; }
    float FixedMax() const { return fixed_max_; }

    /// Feed the largest value seen this frame. Auto tracking runs in every
    /// mode so that switching back to Auto (or locking) is immediately sensible.
    void Update(float frame_max) {
        last_peak_ = std::max(frame_max, 0.0f);
        const float padded_max = std::max(frame_max, 0.0f) * (1.0f + kPadding);
        if (!initialized_) {
            auto_max_ = std::max(padded_max, 1.0f);
            initialized_ = true;
            hold_frames_remaining_ = kHoldFrames;
        } else if (padded_max > auto_max_) {
            auto_max_ = padded_max;
            hold_frames_remaining_ = kHoldFrames;
        } else if (hold_frames_remaining_ > 0) {
            --hold_frames_remaining_;
        } else {
            auto_max_ += kDecayAlpha * (padded_max - auto_max_);
        }
        if (auto_max_ < kMinSpan) {
            auto_max_ = 1.0f;
        }
    }

    /// Restart auto tracking from the next Update().
    void ResetAuto() { initialized_ = false; }

    /// Largest value passed to the last Update().
    float LastPeak() const { return last_peak_; }

    float Min() const {
        switch (mode_) {
            case ColorRangeMode::kFixed:
                return fixed_min_;
            case ColorRangeMode::kLocked:
                return locked_min_;
            case ColorRangeMode::kAuto:
            default:
                return 0.0f;
        }
    }
    float Max() const {
        switch (mode_) {
            case ColorRangeMode::kFixed:
                return fixed_max_;
            case ColorRangeMode::kLocked:
                return locked_max_;
            case ColorRangeMode::kAuto:
            default:
                return auto_max_;
        }
    }

    /// Map value to [0, 1] in the current range (clamped).
    float Normalize(float value) const {
        const float span = Max() - Min();
        if (!(span > kMinSpan)) {
            return 0.0f;
        }
        return std::clamp((value - Min()) / span, 0.0f, 1.0f);
    }

  private:
    ColorRangeMode mode_ = ColorRangeMode::kAuto;
    float auto_max_ = 1.0f;
    float last_peak_ = 0.0f;
    float locked_min_ = 0.0f;
    float locked_max_ = 1.0f;
    float fixed_min_ = 0.0f;
    float fixed_max_ = 1.0f;
    bool has_fixed_ = false;
    bool initialized_ = false;
    int hold_frames_remaining_ = 0;
};

}  // namespace seastack::viz
