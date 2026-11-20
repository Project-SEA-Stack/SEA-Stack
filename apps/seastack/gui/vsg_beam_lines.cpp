/// @file vsg_beam_lines.cpp
/// @brief Implementation of BeamLinesViz.
#include "vsg_beam_lines.h"

#include "vsg_mooring_lines.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace seastack::viz {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kCircleSides = 16;
const vsg::vec4 kSteelColor{0.55f, 0.57f, 0.60f, 1.0f};

/// First vec3 (N = 0 positions, 1 normals) or vec4 (N = 3 colours) array of
/// the VertexIndexDraw under a node.
template <int N, typename Array>
class FindArray : public vsg::Visitor {
  public:
    void apply(vsg::Object& object) override { object.traverse(*this); }
    void apply(vsg::VertexIndexDraw& vid) override {
        if (static_cast<int>(vid.arrays.size()) > N) vid.arrays[N]->data->accept(*this);
    }
    void apply(Array& array) override {
        if (!result) result = &array;
    }
    vsg::ref_ptr<Array> result;
};

vsg::dvec3 ToDvec3(const Vec3& v) { return {v[0], v[1], v[2]}; }

vsg::dvec3 Normalized(const vsg::dvec3& v) {
    const double len = vsg::length(v);
    return len > 1e-12 ? v / len : v;
}

}  // namespace

std::vector<BeamLinesViz::ProfileEdge> BeamLinesViz::MakeProfile(const BeamLineDescription& desc) {
    std::vector<ProfileEdge> edges;
    if (desc.circular) {
        const double r = desc.y_max;
        for (int j = 0; j < kCircleSides; ++j) {
            const double a0 = 2.0 * kPi * j / kCircleSides;
            const double a1 = 2.0 * kPi * (j + 1) / kCircleSides;
            edges.push_back({r * std::cos(a0), r * std::sin(a0), std::cos(a0), std::sin(a0), r * std::cos(a1),
                             r * std::sin(a1), std::cos(a1), std::sin(a1)});
        }
        return edges;
    }
    // Counter-clockwise seen from +x (along the beam): bottom, +y side, top, -y side.
    const double y0 = desc.y_min, y1 = desc.y_max, z0 = desc.z_min, z1 = desc.z_max;
    edges.push_back({y0, z0, 0.0, -1.0, y1, z0, 0.0, -1.0});
    edges.push_back({y1, z0, 1.0, 0.0, y1, z1, 1.0, 0.0});
    edges.push_back({y1, z1, 0.0, 1.0, y0, z1, 0.0, 1.0});
    edges.push_back({y0, z1, -1.0, 0.0, y0, z0, -1.0, 0.0});
    return edges;
}

void BeamLinesViz::Initialize(OverlayBuilder* builder, vsg::ref_ptr<vsg::Group> parent_group,
                              const SceneDescription& scene, const VizFrame& frame) {
    if (!builder || !parent_group || frame.beam_lines.empty() || frame.beam_lines.size() != scene.beam_lines.size()) {
        return;
    }
    builder_ = builder;
    meshes_.clear();
    meshes_.resize(scene.beam_lines.size());
    for (size_t i = 0; i < scene.beam_lines.size(); ++i) {
        const auto& sections = frame.beam_lines[i].sections;
        if (sections.size() < 2) {
            continue;
        }
        BeamMesh& bm = meshes_[i];
        bm.profile = MakeProfile(scene.beam_lines[i]);
        bm.num_rings = sections.size();
        const int edges = static_cast<int>(bm.profile.size());
        const int ring = 2 * edges;
        const int rings = static_cast<int>(bm.num_rings);
        const int cap0 = rings * ring;   // start cap: edges outline points, then the centre
        const int cap1 = cap0 + edges + 1;

        TriangleMesh mesh;
        mesh.vertices.assign(static_cast<size_t>(cap1 + edges + 1), Vec3{0.0, 0.0, 0.0});
        for (int r = 0; r + 1 < rings; ++r) {
            for (int e = 0; e < edges; ++e) {
                const int a = r * ring + 2 * e;
                const int b = a + 1;
                const int c = b + ring;
                const int d = a + ring;
                mesh.faces.push_back({a, b, c});
                mesh.faces.push_back({a, c, d});
            }
        }
        for (int e = 0; e < edges; ++e) {
            const int en = (e + 1) % edges;
            mesh.faces.push_back({cap0 + edges, cap0 + en, cap0 + e});  // faces -x
            mesh.faces.push_back({cap1 + edges, cap1 + e, cap1 + en});  // faces +x
        }

        auto node = builder->CreateIndexedOverlayMesh(mesh, {kSteelColor.r, kSteelColor.g, kSteelColor.b, 1.0f});
        if (!node) {
            continue;
        }
        bm.vertices = vsg::visit<FindArray<0, vsg::vec3Array>>(node).result;
        bm.normals = vsg::visit<FindArray<1, vsg::vec3Array>>(node).result;
        bm.colors = vsg::visit<FindArray<3, vsg::vec4Array>>(node).result;
        if (!bm.vertices || !bm.normals || !bm.colors || bm.vertices->size() != mesh.vertices.size()) {
            bm.vertices = {};
            continue;
        }
        bm.vertices->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
        bm.normals->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
        bm.colors->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
        WriteMesh(sections, std::vector<vsg::vec4>(sections.size(), kSteelColor), &bm);
        parent_group->addChild(node);
    }
    initialized_ = true;
    std::cout << "[BeamViz] Initialized " << scene.beam_lines.size() << " FEA beam line(s)\n";
}

void BeamLinesViz::Update(const SceneDescription& scene, const VizFrame& frame, BeamField field, ColorRange* range) {
    if (!initialized_ || frame.beam_lines.size() != meshes_.size() || frame.beam_lines.size() != scene.beam_lines.size()) {
        return;
    }
    const bool colored = field != BeamField::kNone && range;
    if (colored) {
        double peak = 0.0;
        for (const auto& beam : frame.beam_lines) {
            for (const auto& s : beam.sections) {
                peak = std::max(peak, std::abs(BeamFieldValue(s, field)));
            }
        }
        range->Update(static_cast<float>(peak));
    }
    std::vector<vsg::vec4> colors;
    for (size_t i = 0; i < meshes_.size(); ++i) {
        const auto& sections = frame.beam_lines[i].sections;
        if (!meshes_[i].vertices || sections.size() != meshes_[i].num_rings) {
            continue;
        }
        colors.assign(sections.size(), kSteelColor);
        if (colored) {
            for (size_t k = 0; k < sections.size(); ++k) {
                colors[k] = MooringLinesViz::TurboColormap(
                    range->Normalize(static_cast<float>(std::abs(BeamFieldValue(sections[k], field)))));
            }
        }
        WriteMesh(sections, colors, &meshes_[i]);
    }
}

void BeamLinesViz::WriteMesh(const std::vector<BeamSectionState>& sections, const std::vector<vsg::vec4>& colors,
                             BeamMesh* mesh) {
    const auto& profile = mesh->profile;
    const size_t edges = profile.size();
    const size_t ring = 2 * edges;
    const size_t rings = sections.size();
    auto& verts = *mesh->vertices;
    auto& norms = *mesh->normals;
    auto& cols = *mesh->colors;

    auto world_point = [](const vsg::dvec3& p, const vsg::dvec3& y, const vsg::dvec3& z, double py, double pz) {
        return vsg::vec3(p + y * py + z * pz);
    };
    for (size_t r = 0; r < rings; ++r) {
        const vsg::dvec3 p = ToDvec3(sections[r].position);
        const vsg::dvec3 y = Normalized(ToDvec3(sections[r].y_axis));
        const vsg::dvec3 z = Normalized(ToDvec3(sections[r].z_axis));
        for (size_t e = 0; e < edges; ++e) {
            const ProfileEdge& pe = profile[e];
            const size_t a = r * ring + 2 * e;
            verts[a] = world_point(p, y, z, pe.y0, pe.z0);
            verts[a + 1] = world_point(p, y, z, pe.y1, pe.z1);
            norms[a] = vsg::vec3(y * pe.ny0 + z * pe.nz0);
            norms[a + 1] = vsg::vec3(y * pe.ny1 + z * pe.nz1);
            cols[a] = colors[r];
            cols[a + 1] = colors[r];
        }
    }
    // End caps: outline points then the centre, normal along -x / +x of the section.
    for (int end = 0; end < 2; ++end) {
        const BeamSectionState& s = end == 0 ? sections.front() : sections.back();
        const vsg::dvec3 p = ToDvec3(s.position);
        const vsg::dvec3 y = Normalized(ToDvec3(s.y_axis));
        const vsg::dvec3 z = Normalized(ToDvec3(s.z_axis));
        const vsg::dvec3 x = Normalized(vsg::cross(y, z));
        const vsg::vec3 n(end == 0 ? -x : x);
        const vsg::vec4 c = end == 0 ? colors.front() : colors.back();
        const size_t base = rings * ring + static_cast<size_t>(end) * (edges + 1);
        for (size_t e = 0; e < edges; ++e) {
            verts[base + e] = world_point(p, y, z, profile[e].y0, profile[e].z0);
            norms[base + e] = n;
            cols[base + e] = c;
        }
        verts[base + edges] = vsg::vec3(p);
        norms[base + edges] = n;
        cols[base + edges] = c;
    }
    mesh->vertices->dirty();
    mesh->normals->dirty();
    mesh->colors->dirty();
}

const char* BeamFieldName(BeamField field) {
    switch (field) {
        case BeamField::kAxial: return "Axial force Fx";
        case BeamField::kTorsion: return "Torsion Mx";
        case BeamField::kMy: return "Bending My";
        case BeamField::kMz: return "Bending Mz";
        case BeamField::kNone: break;
    }
    return "None";
}

const char* BeamFieldUnits(BeamField field) { return field == BeamField::kAxial ? "N" : "N m"; }

}  // namespace seastack::viz
