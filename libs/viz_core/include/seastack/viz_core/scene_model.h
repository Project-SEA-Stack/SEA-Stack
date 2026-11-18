#pragma once

#include <seastack/viz_core/scene_framing.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace seastack::viz {

/// Renderer-independent description of what the 3D view shows.
///
/// SceneDescription is built once (bodies, their visual shapes, mooring line
/// topology); VizFrame holds the state that changes every step (poses,
/// velocities, mooring node positions and tensions). Renderers and recorded
/// playback read these instead of the live physics system, so the same frame
/// can be drawn live, scrubbed back or replayed from file.
///
/// Conventions (all members):
///  - World frame, Z up, SI units: m, s, rad, N.
///  - Quaternions are unit, scalar-first (w, x, y, z), matching HDF5 output.
///  - A Pose maps local coordinates to the parent frame: p_parent = R p_local + t.
///  - Angular velocity is expressed in the world frame.

using Quat = std::array<double, 4>;  ///< (w, x, y, z)
/// Linear-light colour, alpha = opacity [0, 1]. Model colours (Chrono ChColor,
/// as given in the model YAML) are copied unchanged and treated as linear, as
/// Chrono's own VSG viewer does; no sRGB decoding is applied.
using Rgba = std::array<float, 4>;

inline constexpr Quat kIdentityQuat{1.0, 0.0, 0.0, 0.0};

struct Pose {
    Vec3 position{0.0, 0.0, 0.0};  ///< [m]
    Quat rotation = kIdentityQuat; ///< (w, x, y, z)
};

/// Rotate v by unit quaternion q.
inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    // t = 2 * cross(q.xyz, v); v' = v + w t + cross(q.xyz, t)
    const Vec3 t{2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])};
    return {v[0] + w * t[0] + (y * t[2] - z * t[1]), v[1] + w * t[1] + (z * t[0] - x * t[2]),
            v[2] + w * t[2] + (x * t[1] - y * t[0])};
}

/// Map a point from local to parent coordinates.
inline Vec3 Transform(const Pose& pose, const Vec3& p_local) {
    const Vec3 r = Rotate(pose.rotation, p_local);
    return {r[0] + pose.position[0], r[1] + pose.position[1], r[2] + pose.position[2]};
}

/// Compose poses: result maps child-local to grandparent (parent * child).
inline Pose Compose(const Pose& parent, const Pose& child) {
    const Quat& a = parent.rotation;
    const Quat& b = child.rotation;
    Pose out;
    out.rotation = {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    out.position = Transform(parent, child.position);
    return out;
}

/// Inverse pose: maps parent coordinates back to local (Compose(Inverse(p), p) = identity).
inline Pose Inverse(const Pose& pose) {
    const Quat& q = pose.rotation;
    Pose out;
    out.rotation = {q[0], -q[1], -q[2], -q[3]};
    const Vec3 r = Rotate(out.rotation, pose.position);
    out.position = {-r[0], -r[1], -r[2]};
    return out;
}

/// Triangle mesh in shape-local coordinates [m]. Shared between shapes and
/// frames; never modified after the scene is built.
struct TriangleMesh {
    std::vector<Vec3> vertices;
    std::vector<std::array<int, 3>> faces;  ///< Vertex indices, counter-clockwise = outward
    /// Optional per-corner normals (3 per face, index 3 f + k); empty = renderer decides.
    std::vector<Vec3> corner_normals;
};

enum class ShapeKind {
    kBox,       ///< size = full lengths along local x, y, z [m]
    kSphere,    ///< size[0] = radius [m]
    kCylinder,  ///< size[0] = radius, size[1] = length along local z [m]
    kMesh,      ///< mesh holds the geometry; size unused
};

struct VisualShape {
    ShapeKind kind = ShapeKind::kBox;
    Pose local;                       ///< Shape frame relative to the body frame
    Vec3 size{1.0, 1.0, 1.0};         ///< See ShapeKind
    std::shared_ptr<const TriangleMesh> mesh;  ///< kMesh only
    Rgba color{0.8f, 0.8f, 0.8f, 1.0f};
    float roughness = 0.55f;
    float metallic = 0.0f;
};

/// Extend bounds (in the body frame) by one shape placed at shape.local [m].
inline void ExtendBounds(const VisualShape& shape, Bounds3* bounds) {
    std::vector<Vec3> corners;
    switch (shape.kind) {
        case ShapeKind::kBox: {
            const Vec3 h{0.5 * shape.size[0], 0.5 * shape.size[1], 0.5 * shape.size[2]};
            for (int i = 0; i < 8; ++i) {
                corners.push_back({(i & 1) ? h[0] : -h[0], (i & 2) ? h[1] : -h[1], (i & 4) ? h[2] : -h[2]});
            }
            break;
        }
        case ShapeKind::kSphere: {
            const double r = shape.size[0];
            for (int i = 0; i < 8; ++i) {
                corners.push_back({(i & 1) ? r : -r, (i & 2) ? r : -r, (i & 4) ? r : -r});
            }
            break;
        }
        case ShapeKind::kCylinder: {
            const double r = shape.size[0];
            const double h = 0.5 * shape.size[1];
            for (int i = 0; i < 8; ++i) {
                corners.push_back({(i & 1) ? r : -r, (i & 2) ? r : -r, (i & 4) ? h : -h});
            }
            break;
        }
        case ShapeKind::kMesh:
            if (shape.mesh) {
                corners = shape.mesh->vertices;
            }
            break;
    }
    for (const auto& p : corners) {
        bounds->Extend(Transform(shape.local, p));
    }
}

struct BodyDescription {
    std::string name;
    bool fixed = false;
    std::vector<VisualShape> shapes;
    Bounds3 local_bounds;  ///< Bounds of all shapes in the body frame [m]
};

/// MoorDyn endpoint classification (same values as the MoorDyn C API).
enum class MooringEndType : int { kCoupled = -1, kFree = 0, kFixed = 1 };

struct MooringLineDescription {
    int num_nodes = 0;
    MooringEndType start_type = MooringEndType::kFree;
    MooringEndType end_type = MooringEndType::kFree;
};

/// Chrono TSDA point-point visual (YAML `visualization: { type: SEGMENT | SPRING }`).
enum class CableKind { kSegment, kSpring };

struct CableLineDescription {
    CableKind kind = CableKind::kSegment;
    /// SPRING coil radius [m]; unused for SEGMENT.
    double spring_radius = 0.05;
    int spring_resolution = 65;   ///< Helix sample count for SPRING
    double spring_turns = 5.0;    ///< Helix turns for SPRING
};

struct SceneDescription {
    std::vector<BodyDescription> bodies;
    std::vector<MooringLineDescription> mooring_lines;
    /// Hoist ropes / PTO springs (Chrono TSDA visuals), then distance
    /// constraints; separate from MoorDyn.
    std::vector<CableLineDescription> cable_lines;
    /// Names of the force sources in VizFrame::body_wrenches, e.g. "Excitation";
    /// empty until the first forces are known.
    std::vector<std::string> force_sources;
    Bounds3 model_bounds;     ///< World bounds of all bodies at build time [m]
    double gravity = 9.81;    ///< [m/s^2]
    double water_depth = 0.0; ///< [m]; <= 0 = deep / unknown
};

struct BodyState {
    Pose pose;                               ///< Frame the shapes are attached to, in the world
    Vec3 com_position{0.0, 0.0, 0.0};        ///< Centre of mass, world [m]
    Vec3 linear_velocity{0.0, 0.0, 0.0};     ///< Of the centre of mass, world [m/s]
    Vec3 angular_velocity{0.0, 0.0, 0.0};    ///< World frame [rad/s]
};

struct MooringLineState {
    std::vector<Vec3> node_positions;    ///< World [m]
    std::vector<double> node_tensions;   ///< Tension magnitude per node [N]
};

struct CableLineState {
    Vec3 point1{0.0, 0.0, 0.0};  ///< World endpoint 1 [m]
    Vec3 point2{0.0, 0.0, 0.0};  ///< World endpoint 2 [m]
    double tension = 0.0;        ///< |force| [N]; optional colouring
};

/// Load on one body from one source: force [N] and moment about the body's
/// centre of mass [N m], both in the world frame (as applied to Chrono).
struct Wrench {
    Vec3 force{0.0, 0.0, 0.0};
    Vec3 moment{0.0, 0.0, 0.0};
};

/// State at one simulated instant. Index i in bodies / mooring_lines /
/// cable_lines matches the same index in the SceneDescription.
struct VizFrame {
    double time = 0.0;      ///< Simulated time [s]
    int64_t step = 0;       ///< Physics step count at this frame
    std::vector<BodyState> bodies;
    std::vector<MooringLineState> mooring_lines;
    std::vector<CableLineState> cable_lines;
    /// Solver output: body_wrenches[s * bodies.size() + b] is force source s
    /// (SceneDescription::force_sources) on body b. Empty = no force data.
    std::vector<Wrench> body_wrenches;
};

/// True if the frame's arrays match the description's sizes.
inline bool IsConsistent(const SceneDescription& scene, const VizFrame& frame) {
    if (frame.bodies.size() != scene.bodies.size() ||
        frame.mooring_lines.size() != scene.mooring_lines.size() ||
        frame.cable_lines.size() != scene.cable_lines.size()) {
        return false;
    }
    if (!frame.body_wrenches.empty() &&
        frame.body_wrenches.size() != scene.force_sources.size() * scene.bodies.size()) {
        return false;
    }
    for (size_t i = 0; i < frame.mooring_lines.size(); ++i) {
        const auto& line = frame.mooring_lines[i];
        if (line.node_positions.size() != static_cast<size_t>(scene.mooring_lines[i].num_nodes) ||
            (!line.node_tensions.empty() && line.node_tensions.size() != line.node_positions.size())) {
            return false;
        }
    }
    return true;
}

}  // namespace seastack::viz
