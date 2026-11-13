#include <seastack/adapters/chrono/viz_extractor.h>

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeCylinder.h>
#include <chrono/assets/ChVisualShapePointPoint.h>
#include <chrono/assets/ChVisualShapeSphere.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystem.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <typeinfo>
#include <utility>

namespace seastack::chrono {

namespace viz = seastack::viz;

namespace {

viz::Vec3 ToVec3(const ::chrono::ChVector3d& v) { return {v.x(), v.y(), v.z()}; }

viz::Pose ToPose(const ::chrono::ChFrame<>& frame) {
    const ::chrono::ChQuaterniond& q = frame.GetRot();
    viz::Pose pose;
    pose.position = ToVec3(frame.GetPos());
    pose.rotation = {q.e0(), q.e1(), q.e2(), q.e3()};
    return pose;
}

/// Colour, opacity and PBR settings of a shape (first material, else shape colour).
/// Non-const only because Chrono's GetMaterials() is non-const; nothing is modified.
void CopyAppearance(::chrono::ChVisualShape& shape, viz::VisualShape* out) {
    const ::chrono::ChColor c = shape.GetColor();
    out->color = {c.R, c.G, c.B, shape.GetOpacity()};
    const auto& materials = shape.GetMaterials();
    if (!materials.empty() && materials.front()) {
        out->roughness = materials.front()->GetRoughness();
        out->metallic = materials.front()->GetMetallic();
    }
}

/// Non-const only because Chrono's GetMesh() is non-const; nothing is modified.
std::shared_ptr<const viz::TriangleMesh> ConvertMesh(::chrono::ChVisualShapeTriangleMesh& shape) {
    auto src = shape.GetMesh();
    if (!src) {
        return nullptr;
    }
    const ::chrono::ChVector3d scale = shape.GetScale();
    auto mesh = std::make_shared<viz::TriangleMesh>();
    mesh->vertices.reserve(src->GetCoordsVertices().size());
    for (const auto& v : src->GetCoordsVertices()) {
        mesh->vertices.push_back({v.x() * scale.x(), v.y() * scale.y(), v.z() * scale.z()});
    }
    const auto& faces = src->GetIndicesVertexes();
    mesh->faces.reserve(faces.size());
    for (const auto& f : faces) {
        mesh->faces.push_back({f.x(), f.y(), f.z()});
    }
    // Per-corner normals only when every face has normal indices.
    const auto& normal_idx = src->GetIndicesNormals();
    const auto& normals = src->GetCoordsNormals();
    if (normal_idx.size() == faces.size() && !normals.empty()) {
        mesh->corner_normals.reserve(3 * faces.size());
        for (const auto& fi : normal_idx) {
            for (int k = 0; k < 3; ++k) {
                const int i = fi[k];
                mesh->corner_normals.push_back(
                    (i >= 0 && static_cast<size_t>(i) < normals.size()) ? ToVec3(normals[i]) : viz::Vec3{0.0, 0.0, 1.0});
            }
        }
    }
    return mesh;
}

/// Local bounds of one shape in the body frame [m].
void ExtendBounds(const viz::VisualShape& shape, viz::Bounds3* bounds) {
    std::vector<viz::Vec3> corners;
    switch (shape.kind) {
        case viz::ShapeKind::kBox: {
            const viz::Vec3 h{0.5 * shape.size[0], 0.5 * shape.size[1], 0.5 * shape.size[2]};
            for (int i = 0; i < 8; ++i) {
                corners.push_back({(i & 1) ? h[0] : -h[0], (i & 2) ? h[1] : -h[1], (i & 4) ? h[2] : -h[2]});
            }
            break;
        }
        case viz::ShapeKind::kSphere: {
            const double r = shape.size[0];
            for (int i = 0; i < 8; ++i) {
                corners.push_back({(i & 1) ? r : -r, (i & 2) ? r : -r, (i & 4) ? r : -r});
            }
            break;
        }
        case viz::ShapeKind::kCylinder: {
            const double r = shape.size[0];
            const double h = 0.5 * shape.size[1];
            for (int i = 0; i < 8; ++i) {
                corners.push_back({(i & 1) ? r : -r, (i & 2) ? r : -r, (i & 4) ? h : -h});
            }
            break;
        }
        case viz::ShapeKind::kMesh:
            if (shape.mesh) {
                corners = shape.mesh->vertices;
            }
            break;
    }
    for (const auto& p : corners) {
        bounds->Extend(viz::Transform(shape.local, p));
    }
}

viz::BodyDescription DescribeBody(const ::chrono::ChBody& body) {
    viz::BodyDescription desc;
    desc.name = body.GetName();
    desc.fixed = body.IsFixed();
    auto model = body.GetVisualModel();
    if (!model) {
        return desc;
    }
    for (const auto& instance : model->GetShapeInstances()) {
        if (!instance.shape) {
            continue;
        }
        viz::VisualShape shape;
        shape.local = ToPose(instance.frame);
        if (auto box = std::dynamic_pointer_cast<::chrono::ChVisualShapeBox>(instance.shape)) {
            shape.kind = viz::ShapeKind::kBox;
            shape.size = ToVec3(box->GetLengths());
        } else if (auto sphere = std::dynamic_pointer_cast<::chrono::ChVisualShapeSphere>(instance.shape)) {
            shape.kind = viz::ShapeKind::kSphere;
            shape.size = {sphere->GetRadius(), 0.0, 0.0};
        } else if (auto cyl = std::dynamic_pointer_cast<::chrono::ChVisualShapeCylinder>(instance.shape)) {
            shape.kind = viz::ShapeKind::kCylinder;
            shape.size = {cyl->GetRadius(), cyl->GetHeight(), 0.0};
        } else if (auto tri = std::dynamic_pointer_cast<::chrono::ChVisualShapeTriangleMesh>(instance.shape)) {
            shape.kind = viz::ShapeKind::kMesh;
            shape.mesh = ConvertMesh(*tri);
            if (!shape.mesh) {
                continue;
            }
        } else {
            continue;  // unsupported shape type (e.g. model file, line, point cloud)
        }
        CopyAppearance(*instance.shape, &shape);
        ExtendBounds(shape, &desc.local_bounds);
        desc.shapes.push_back(std::move(shape));
    }
    return desc;
}

/// First Chrono SEGMENT/SPRING on the TSDA, if any; fills out->kind and spring params.
bool DescribeCableVisual(const ::chrono::ChLinkTSDA& tsda, viz::CableLineDescription* out) {
    auto model = tsda.GetVisualModel();
    if (!model || !out) {
        return false;
    }
    for (const auto& instance : model->GetShapeInstances()) {
        if (!instance.shape) {
            continue;
        }
        if (auto spring = std::dynamic_pointer_cast<::chrono::ChVisualShapeSpring>(instance.shape)) {
            out->kind = viz::CableKind::kSpring;
            out->spring_radius = spring->GetRadius();
            out->spring_resolution = spring->GetResolution();
            out->spring_turns = spring->GetTurns();
            return true;
        }
        if (std::dynamic_pointer_cast<::chrono::ChVisualShapeSegment>(instance.shape) ||
            std::dynamic_pointer_cast<::chrono::ChVisualShapePointPoint>(instance.shape)) {
            out->kind = viz::CableKind::kSegment;
            return true;
        }
    }
    return false;
}

}  // namespace

bool IsWaterSurfaceBody(const ::chrono::ChBody& body) {
    return body.GetName() == "water_surface" || body.GetName() == "animated_water_surface";
}

namespace {

bool IsDrawableBodyShape(const std::shared_ptr<::chrono::ChVisualShape>& shape) {
    return std::dynamic_pointer_cast<::chrono::ChVisualShapeBox>(shape) ||
           std::dynamic_pointer_cast<::chrono::ChVisualShapeSphere>(shape) ||
           std::dynamic_pointer_cast<::chrono::ChVisualShapeCylinder>(shape) ||
           std::dynamic_pointer_cast<::chrono::ChVisualShapeTriangleMesh>(shape);
}

bool HasVisualShapes(const ::chrono::ChPhysicsItem& item) {
    const auto model = item.GetVisualModel();
    return model && model->GetNumShapes() > 0;
}

std::string ItemName(const ::chrono::ChObj& item, const char* kind) {
    return item.GetName().empty() ? std::string("unnamed ") + kind
                                  : std::string(kind) + " '" + item.GetName() + "'";
}

}  // namespace

std::vector<std::string> FindUndrawableItems(const ::chrono::ChSystem& system) {
    std::vector<std::string> items;
    for (const auto& body : system.GetBodies()) {
        if (!body || IsWaterSurfaceBody(*body) || !body->GetVisualModel()) {
            continue;
        }
        for (const auto& instance : body->GetVisualModel()->GetShapeInstances()) {
            if (instance.shape && !IsDrawableBodyShape(instance.shape)) {
                const ::chrono::ChVisualShape& shape = *instance.shape;
                items.push_back(ItemName(*body, "body") + ": " + typeid(shape).name());
            }
        }
    }
    for (const auto& link : system.GetLinks()) {
        if (!link || !HasVisualShapes(*link)) {
            continue;
        }
        auto tsda = std::dynamic_pointer_cast<::chrono::ChLinkTSDA>(link);
        viz::CableLineDescription unused;
        if (!tsda || !DescribeCableVisual(*tsda, &unused)) {
            items.push_back(ItemName(*link, "link") + " visual");
        }
    }
    if (!system.GetMeshes().empty()) {
        items.push_back(std::to_string(system.GetMeshes().size()) + " FEA mesh(es)");
    }
    for (const auto& item : system.GetOtherPhysicsItems()) {
        if (item && HasVisualShapes(*item)) {
            items.push_back(ItemName(*item, "physics item") + " visual");
        }
    }
    return items;
}

VizExtractor::VizExtractor(::chrono::ChSystem* system, viz::MooringVizProvider mooring)
    : system_(system), mooring_(std::move(mooring)) {
    if (system_) {
        scene_.gravity = system_->GetGravitationalAcceleration().Length();
        for (const auto& body : system_->GetBodies()) {
            if (!body || IsWaterSurfaceBody(*body)) {
                continue;
            }
            bodies_.push_back(body);
            scene_.bodies.push_back(DescribeBody(*body));
        }
        // Hoist ropes / winch cables / PTO springs. Prefer YAML SEGMENT/SPRING
        // visuals; otherwise draw a straight cable when the endpoints are a
        // real distance apart (skips zero-length station-keeping TSDAs).
        for (const auto& link : system_->GetLinks()) {
            auto tsda = std::dynamic_pointer_cast<::chrono::ChLinkTSDA>(link);
            if (!tsda) {
                continue;
            }
            viz::CableLineDescription desc;
            if (!DescribeCableVisual(*tsda, &desc)) {
                const ::chrono::ChVector3d d = tsda->GetPoint2Abs() - tsda->GetPoint1Abs();
                if (d.Length() < 0.05) {
                    continue;  // station-keeping / coincident anchors
                }
                desc.kind = viz::CableKind::kSegment;
            }
            tsda_cables_.push_back(std::move(tsda));
            scene_.cable_lines.push_back(desc);
        }
    }
    // Model bounds from the first frame, so they match what is drawn.
    const viz::VizFrame first = Capture(0);
    for (size_t i = 0; i < bodies_.size(); ++i) {
        const auto& body = scene_.bodies[i];
        const auto& state = first.bodies[i];
        scene_.model_bounds.Extend(state.com_position);
        if (body.local_bounds.IsValid()) {
            const viz::Bounds3& b = body.local_bounds;
            for (int c = 0; c < 8; ++c) {
                scene_.model_bounds.Extend(viz::Transform(
                    state.pose, {(c & 1) ? b.max[0] : b.min[0], (c & 2) ? b.max[1] : b.min[1],
                                 (c & 4) ? b.max[2] : b.min[2]}));
            }
        }
    }
}

void VizExtractor::SetMooringProvider(viz::MooringVizProvider mooring) {
    mooring_ = std::move(mooring);
    scene_.mooring_lines.clear();
}

viz::VizFrame VizExtractor::Capture(int64_t step) {
    std::vector<viz::MooringLineVizData> provider_lines;
    if (mooring_) {
        provider_lines = mooring_();
        if (scene_.mooring_lines.empty() && !provider_lines.empty()) {
            for (const auto& line : provider_lines) {
                viz::MooringLineDescription desc;
                desc.num_nodes = static_cast<int>(line.node_positions.size());
                desc.start_type = static_cast<viz::MooringEndType>(static_cast<int>(line.start_point_type));
                desc.end_type = static_cast<viz::MooringEndType>(static_cast<int>(line.end_point_type));
                scene_.mooring_lines.push_back(desc);
            }
        }
    }

    viz::VizFrame frame;
    frame.step = step;
    frame.time = system_ ? system_->GetChTime() : 0.0;
    frame.bodies.reserve(bodies_.size());
    for (const auto& body : bodies_) {
        viz::BodyState state;
        state.pose = ToPose(body->GetVisualModelFrame());
        state.com_position = ToVec3(body->GetPos());
        state.linear_velocity = ToVec3(body->GetPosDt());
        state.angular_velocity = ToVec3(body->GetAngVelParent());
        frame.bodies.push_back(state);
    }
    if (!scene_.mooring_lines.empty()) {
        const size_t n = std::min(provider_lines.size(), scene_.mooring_lines.size());
        frame.mooring_lines.resize(scene_.mooring_lines.size());
        for (size_t i = 0; i < n; ++i) {
            auto& out = frame.mooring_lines[i];
            out.node_positions.reserve(provider_lines[i].node_positions.size());
            for (const auto& p : provider_lines[i].node_positions) {
                out.node_positions.push_back({p[0], p[1], p[2]});
            }
            out.node_tensions = provider_lines[i].node_tensions;
        }
    }
    frame.cable_lines.resize(tsda_cables_.size());
    for (size_t i = 0; i < tsda_cables_.size(); ++i) {
        const auto& tsda = tsda_cables_[i];
        auto& out = frame.cable_lines[i];
        out.point1 = ToVec3(tsda->GetPoint1Abs());
        out.point2 = ToVec3(tsda->GetPoint2Abs());
        out.tension = std::abs(tsda->GetForce());
    }
    return frame;
}

}  // namespace seastack::chrono
