#pragma once

#include <seastack/viz_core/frame_history.h>
#include <seastack/viz_core/scene_model.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace seastack::viz {

/// Time series read from the recorded frames, for plots, trajectories and
/// force-arrow scaling. Visual only.

/// Indices (oldest first) of the frames with time in [t_begin, t_end], at
/// most max_points of them, evenly strided. The last frame in the range is
/// always included, so a live plot ends at the newest state.
inline std::vector<size_t> SampleIndices(const FrameHistory& history, double t_begin, double t_end,
                                         size_t max_points) {
    std::vector<size_t> out;
    if (history.Empty() || t_end < t_begin || max_points == 0) {
        return out;
    }
    size_t first = history.NearestIndex(t_begin);
    if (history.At(first).time < t_begin && first + 1 < history.Size()) {
        ++first;
    }
    size_t last = history.NearestIndex(t_end);
    if (history.At(last).time > t_end && last > 0) {
        --last;
    }
    if (last < first || history.At(first).time > t_end || history.At(last).time < t_begin) {
        return out;
    }
    const size_t count = last - first + 1;
    const size_t stride = (count + max_points - 1) / max_points;
    // Stride back from the last frame so it is always kept.
    for (size_t k = 0; k * stride < count; ++k) {
        out.push_back(last - k * stride);
    }
    std::reverse(out.begin(), out.end());
    return out;
}

/// Roll, pitch, yaw [rad] of unit quaternion q (w, x, y, z): Cardan angles
/// X-Y'-Z'' (intrinsic), the same formula as Chrono's GetCardanAnglesXYZ and
/// the euler_xyz datasets of the HDF5 output.
inline Vec3 CardanAnglesXYZ(const Quat& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double sin_pitch = std::clamp(2.0 * (x * z - w * y), -1.0, 1.0);
    return {std::atan2(2.0 * (y * z + w * x), z * z - y * y - x * x + w * w), -std::asin(sin_pitch),
            std::atan2(2.0 * (x * y + z * w), x * x + w * w - z * z - y * y)};
}

/// Load of source `source` on body `body`; zero if the frame has no force data.
inline Wrench BodyWrench(const VizFrame& frame, size_t source, size_t body) {
    const size_t i = source * frame.bodies.size() + body;
    return i < frame.body_wrenches.size() ? frame.body_wrenches[i] : Wrench{};
}

/// Sum over all sources on body `body` (the total load the solver applied
/// from these sources).
inline Wrench TotalWrench(const VizFrame& frame, size_t body) {
    Wrench total;
    const size_t n_bodies = frame.bodies.size();
    if (n_bodies == 0) {
        return total;
    }
    for (size_t i = body; i < frame.body_wrenches.size(); i += n_bodies) {
        for (int k = 0; k < 3; ++k) {
            total.force[k] += frame.body_wrenches[i].force[k];
            total.moment[k] += frame.body_wrenches[i].moment[k];
        }
    }
    return total;
}

inline double Magnitude(const Vec3& v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

}  // namespace seastack::viz
