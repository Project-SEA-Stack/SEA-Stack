#pragma once

#include <seastack/viz_core/scene_model.h>

#include <algorithm>
#include <cmath>

namespace seastack::viz {

/// Visual-only blending of recorded frames, used to draw smooth slow motion
/// and timeline playback between stored snapshots. The result is never fed
/// back into the simulation and must be labelled as interpolated in the GUI.

/// Spherical linear interpolation between unit quaternions (w, x, y, z) along
/// the shorter arc; s in [0, 1]. Falls back to normalised linear blending
/// when the rotations are almost equal.
inline Quat Slerp(const Quat& a, const Quat& b_in, double s) {
    Quat b = b_in;
    double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (dot < 0.0) {  // q and -q are the same rotation; take the shorter arc
        dot = -dot;
        for (double& c : b) {
            c = -c;
        }
    }
    double wa = 1.0 - s;
    double wb = s;
    constexpr double kLinearThreshold = 0.9995;
    if (dot < kLinearThreshold) {
        const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
        const double sin_theta = std::sin(theta);
        wa = std::sin((1.0 - s) * theta) / sin_theta;
        wb = std::sin(s * theta) / sin_theta;
    }
    Quat q{wa * a[0] + wb * b[0], wa * a[1] + wb * b[1], wa * a[2] + wb * b[2], wa * a[3] + wb * b[3]};
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0.0) {
        for (double& c : q) {
            c /= norm;
        }
    }
    return q;
}

inline Vec3 Lerp(const Vec3& a, const Vec3& b, double s) {
    return {a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2])};
}

/// State at time t between frames a and b of the same scene (a.time <= t <=
/// b.time; t is clamped to that range). Positions, velocities, mooring nodes,
/// tensions and body forces are blended linearly, rotations by slerp. If the frames do
/// not have the same layout (e.g. mooring data appeared between them), the
/// frame nearer in time is returned unchanged.
inline VizFrame InterpolateFrames(const VizFrame& a, const VizFrame& b, double t) {
    const double span = b.time - a.time;
    if (!(span > 0.0)) {
        return t - a.time <= b.time - t ? a : b;
    }
    const double s = std::clamp((t - a.time) / span, 0.0, 1.0);
    const VizFrame& nearer = s <= 0.5 ? a : b;
    bool same_layout = a.bodies.size() == b.bodies.size() && a.mooring_lines.size() == b.mooring_lines.size() &&
                       a.cable_lines.size() == b.cable_lines.size() &&
                       a.body_wrenches.size() == b.body_wrenches.size();
    for (size_t i = 0; same_layout && i < a.mooring_lines.size(); ++i) {
        same_layout = a.mooring_lines[i].node_positions.size() == b.mooring_lines[i].node_positions.size() &&
                      a.mooring_lines[i].node_tensions.size() == b.mooring_lines[i].node_tensions.size();
    }
    if (!same_layout) {
        return nearer;
    }

    VizFrame out = a;
    out.time = a.time + s * span;
    out.step = nearer.step;
    for (size_t i = 0; i < out.bodies.size(); ++i) {
        const BodyState& ba = a.bodies[i];
        const BodyState& bb = b.bodies[i];
        BodyState& bo = out.bodies[i];
        bo.pose.position = Lerp(ba.pose.position, bb.pose.position, s);
        bo.pose.rotation = Slerp(ba.pose.rotation, bb.pose.rotation, s);
        bo.com_position = Lerp(ba.com_position, bb.com_position, s);
        bo.linear_velocity = Lerp(ba.linear_velocity, bb.linear_velocity, s);
        bo.angular_velocity = Lerp(ba.angular_velocity, bb.angular_velocity, s);
    }
    for (size_t i = 0; i < out.mooring_lines.size(); ++i) {
        MooringLineState& lo = out.mooring_lines[i];
        const MooringLineState& lb = b.mooring_lines[i];
        for (size_t k = 0; k < lo.node_positions.size(); ++k) {
            lo.node_positions[k] = Lerp(lo.node_positions[k], lb.node_positions[k], s);
        }
        for (size_t k = 0; k < lo.node_tensions.size(); ++k) {
            lo.node_tensions[k] += s * (lb.node_tensions[k] - lo.node_tensions[k]);
        }
    }
    for (size_t i = 0; i < out.cable_lines.size(); ++i) {
        CableLineState& co = out.cable_lines[i];
        const CableLineState& cb = b.cable_lines[i];
        co.point1 = Lerp(co.point1, cb.point1, s);
        co.point2 = Lerp(co.point2, cb.point2, s);
        co.tension += s * (cb.tension - co.tension);
    }
    for (size_t i = 0; i < out.body_wrenches.size(); ++i) {
        out.body_wrenches[i].force = Lerp(a.body_wrenches[i].force, b.body_wrenches[i].force, s);
        out.body_wrenches[i].moment = Lerp(a.body_wrenches[i].moment, b.body_wrenches[i].moment, s);
    }
    return out;
}

}  // namespace seastack::viz
