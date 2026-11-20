#pragma once

#include <seastack/viz_core/frame_interpolation.h>
#include <seastack/viz_core/scene_model.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace seastack::viz {

/// Approximate heap + object size of one recorded frame [bytes].
inline size_t EstimateFrameBytes(const VizFrame& frame) {
    size_t bytes = sizeof(VizFrame) + frame.bodies.size() * sizeof(BodyState) +
                   frame.cable_lines.size() * sizeof(CableLineState) + frame.body_wrenches.size() * sizeof(Wrench);
    for (const MooringLineState& line : frame.mooring_lines) {
        bytes += sizeof(MooringLineState) + line.node_positions.size() * sizeof(Vec3) +
                 line.node_tensions.size() * sizeof(double);
    }
    for (const BeamLineState& beam : frame.beam_lines) {
        bytes += sizeof(BeamLineState) + beam.sections.size() * sizeof(BeamSectionState);
    }
    return bytes;
}

/// Number of frames of `frame_bytes` each that fit in `budget_bytes`,
/// between 2 and max_frames.
inline size_t FrameCapacityForBudget(size_t frame_bytes, size_t budget_bytes, size_t max_frames) {
    const size_t fit = budget_bytes / std::max<size_t>(frame_bytes, 1);
    return std::clamp<size_t>(fit, 2, std::max<size_t>(max_frames, 2));
}

/// Bounded ring buffer of recent VizFrames for scrubbing and replay.
///
/// Holds at most Capacity() frames; pushing to a full buffer drops the oldest.
/// Frames are expected in increasing time order (as produced by stepping);
/// a frame whose time is not after the newest one restarts the history, so a
/// reset or rewind of the simulation never mixes two timelines.
class FrameHistory {
  public:
    explicit FrameHistory(size_t capacity) : capacity_(capacity > 0 ? capacity : 1) { frames_.reserve(capacity_); }

    size_t Capacity() const { return capacity_; }
    size_t Size() const { return frames_.size(); }
    bool Empty() const { return frames_.empty(); }

    void Clear() {
        frames_.clear();
        head_ = 0;
    }

    void Push(VizFrame frame) {
        if (!Empty() && !(frame.time > Newest().time)) {
            Clear();
        }
        if (frames_.size() < capacity_) {
            frames_.push_back(std::move(frame));
        } else {
            frames_[head_] = std::move(frame);
            head_ = (head_ + 1) % capacity_;
        }
    }

    /// Frame i in time order: 0 = oldest, Size() - 1 = newest. Requires i < Size().
    const VizFrame& At(size_t i) const { return frames_[(head_ + i) % frames_.size()]; }
    const VizFrame& Oldest() const { return At(0); }
    const VizFrame& Newest() const { return At(Size() - 1); }

    /// Index of the frame closest in time to t (ties go to the earlier frame).
    /// Requires !Empty().
    size_t NearestIndex(double t) const {
        size_t lo = 0;
        size_t hi = Size() - 1;
        if (t <= At(lo).time) {
            return lo;
        }
        if (t >= At(hi).time) {
            return hi;
        }
        while (hi - lo > 1) {  // binary search: At(lo).time <= t < At(hi).time
            const size_t mid = lo + (hi - lo) / 2;
            if (At(mid).time <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return (t - At(lo).time <= At(hi).time - t) ? lo : hi;
    }

    /// State at time t [s], clamped to the recorded range. Between two
    /// recorded frames the result is interpolated (visual only); returns true
    /// in that case and false when t falls on a recorded frame. Requires !Empty().
    bool Sample(double t, VizFrame* out) const {
        constexpr double kSameTime = 1e-9;  // [s]
        const size_t i = NearestIndex(t);
        const double ti = At(i).time;
        if (std::abs(t - ti) <= kSameTime || t <= Oldest().time || t >= Newest().time) {
            *out = At(i);
            return false;
        }
        const size_t lo = (t > ti) ? i : i - 1;
        *out = InterpolateFrames(At(lo), At(lo + 1), t);
        return true;
    }

  private:
    size_t capacity_;
    std::vector<VizFrame> frames_;
    size_t head_ = 0;  ///< Index of the oldest frame once the buffer is full
};

}  // namespace seastack::viz
