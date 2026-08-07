/*********************************************************************
 * @file  structure_subsystem.cpp
 * @brief Implementation of StructureSubsystem.
 *
 * Builds an Euler-beam frame, mates, and rigid deck planks from a
 * StructureConfig (geometry, sections, mates, and attachments).
 *********************************************************************/

#include "structure_subsystem.h"

#include <seastack/infra/logging.h>

#include <chrono/assets/ChColor.h>
#include <chrono/assets/ChVisualMaterial.h>
#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/assets/ChVisualShapeFEA.h>
#include <chrono/collision/ChCollisionShapeBox.h>
#include <chrono/core/ChFrame.h>
#include <chrono/core/ChVector3.h>
#include <chrono/fea/ChBeamSectionEuler.h>
#include <chrono/fea/ChBuilderBeam.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChContactMaterialSMC.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChSystem.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace seastack::app {

namespace {

namespace fea = ::chrono::fea;
using ::chrono::ChColor;
using ::chrono::ChFramed;
using ::chrono::ChVector3d;

constexpr double kGravity = 9.81;  // m/s^2

/// Second moments of a thin-wall rectangular hollow (box) section about its
/// two principal axes: outer minus inner I = (b h^3 - b_i h_i^3) / 12.
struct BoxSection {
    double area;      // m^2
    double i_strong;  // m^4, depth cubed: resists bending in the deep direction
    double i_weak;    // m^4
    double j_tors;    // m^4
};

BoxSection ComputeBoxSection(double width, double depth, double wall) {
    const double wi = width - 2.0 * wall;
    const double di = depth - 2.0 * wall;
    BoxSection s;
    s.area = width * depth - wi * di;
    s.i_strong = (width * depth * depth * depth - wi * di * di * di) / 12.0;
    s.i_weak = (depth * width * width * width - di * wi * wi * wi) / 12.0;
    // Thin-walled box torsion approximated as the polar sum (adequate for a
    // grillage with stiff cross-beams; torsion is a secondary load path).
    s.j_tors = s.i_strong + s.i_weak;
    return s;
}

/// Chrono Euler section from a box, given the section's material.  With the
/// beam's Ydir = local y, the strong-axis (depth-cubed) moment must be Izz,
/// not Iyy.
std::shared_ptr<fea::ChBeamSectionEulerAdvanced> MakeBoxBeamSection(
        const BoxSection& box, double width, double depth,
        const StructureMaterialConfig& mat) {
    auto section = chrono_types::make_shared<fea::ChBeamSectionEulerAdvanced>();
    section->SetYoungModulus(mat.youngs_modulus);
    section->SetShearModulus(mat.shear_modulus);
    section->SetDensity(mat.density);
    section->SetArea(box.area);
    section->SetIzz(box.i_strong);  // local y is vertical: resists wheel loads
    section->SetIyy(box.i_weak);
    section->SetJ(box.j_tors);
    // Draw the actual rectangular section (local y, local z) = (depth, width).
    section->SetDrawThickness(depth, width);
    return section;
}

std::shared_ptr<::chrono::ChBody> FindBodyByName(::chrono::ChSystem& system,
                                                 const std::string& name) {
    for (const auto& b : system.GetBodies()) {
        if (b && b->GetName() == name) {
            return b;
        }
    }
    return nullptr;
}

::chrono::ChVisualShapeFEA::DataType MapDataType(const std::string& s) {
    using DT = ::chrono::ChVisualShapeFEA::DataType;
    if (s == "ELEM_BEAM_MY") return DT::ELEM_BEAM_MY;
    if (s == "ELEM_BEAM_MZ") return DT::ELEM_BEAM_MZ;
    if (s == "ELEM_BEAM_TX") return DT::ELEM_BEAM_TX;
    if (s == "NODE_SPEED_NORM") return DT::NODE_SPEED_NORM;
    if (s == "NONE") return DT::NONE;
    return DT::ELEM_BEAM_MZ;
}

::chrono::ChColormap::Type MapColormap(const std::string& s) {
    using CM = ::chrono::ChColormap::Type;
    if (s == "JET") return CM::JET;
    if (s == "INFERNO") return CM::INFERNO;
    if (s == "VIRIDIS") return CM::VIRIDIS;
    if (s == "PLASMA") return CM::PLASMA;
    return CM::JET;
}

}  // namespace

StructureSubsystem::StructureSubsystem(StructureConfig config, double settle_time_s)
    : config_(std::move(config)), settle_time_s_(settle_time_s) {}

StructureSubsystem::~StructureSubsystem() = default;

StructureSubsystem::NodePtr StructureSubsystem::ResolveNode(const std::string& ref) const {
    const auto dot = ref.find('.');
    if (dot == std::string::npos) {
        throw std::runtime_error("structure: node reference '" + ref +
                                 "' must be of the form beam.first|last|mid|<index>.");
    }
    const std::string beam = ref.substr(0, dot);
    const std::string qual = ref.substr(dot + 1);

    auto it = beam_nodes_.find(beam);
    if (it == beam_nodes_.end() || it->second.empty()) {
        throw std::runtime_error("structure: node reference '" + ref +
                                 "' names unknown or empty beam '" + beam + "'.");
    }
    const auto& nodes = it->second;

    if (qual == "first") {
        return nodes.front();
    }
    if (qual == "last") {
        return nodes.back();
    }
    if (qual == "mid") {
        return nodes[nodes.size() / 2];
    }
    // Otherwise a 0-based integer index.
    try {
        const std::size_t idx = static_cast<std::size_t>(std::stoul(qual));
        if (idx >= nodes.size()) {
            throw std::out_of_range("index");
        }
        return nodes[idx];
    } catch (const std::exception&) {
        throw std::runtime_error("structure: node reference '" + ref +
                                 "' has an invalid qualifier '" + qual + "'.");
    }
}

void StructureSubsystem::Attach(::chrono::ChSystem& system) {
    using namespace ::chrono;

    mesh_ = chrono_types::make_shared<fea::ChMesh>();

    // --- Materials and sections ------------------------------------------
    std::unordered_map<std::string, StructureMaterialConfig> materials;
    for (const auto& m : config_.materials) {
        materials.emplace(m.name, m);
    }

    struct SectionData {
        std::shared_ptr<fea::ChBeamSectionEulerAdvanced> section;
        double area = 0.0;
        double density = 0.0;
    };
    std::unordered_map<std::string, SectionData> sections;
    for (const auto& s : config_.sections) {
        if (s.type != "BOX") {
            throw std::runtime_error("structure: section '" + s.name + "' has unsupported type '" +
                                     s.type + "' (only BOX is supported).");
        }
        auto mit = materials.find(s.material);
        if (mit == materials.end()) {
            throw std::runtime_error("structure: section '" + s.name + "' references unknown "
                                     "material '" + s.material + "'.");
        }
        const BoxSection box = ComputeBoxSection(s.width, s.depth, s.wall);
        SectionData data;
        data.section = MakeBoxBeamSection(box, s.width, s.depth, mit->second);
        data.area = box.area;
        data.density = mit->second.density;
        sections.emplace(s.name, std::move(data));
    }

    auto section_by_name = [&](const std::string& name) -> SectionData& {
        auto it = sections.find(name);
        if (it == sections.end()) {
            throw std::runtime_error("structure: unknown section '" + name + "'.");
        }
        return it->second;
    };

    fea::ChBuilderBeamEuler builder;

    // --- Explicit beams --------------------------------------------------
    for (const auto& b : config_.beams) {
        SectionData& sd = section_by_name(b.section);
        const ChVector3d start(b.start[0], b.start[1], b.start[2]);
        const ChVector3d end(b.end[0], b.end[1], b.end[2]);
        const ChVector3d ydir(b.y_direction[0], b.y_direction[1], b.y_direction[2]);
        builder.BuildBeam(mesh_, sd.section, std::max(1, b.elements), start, end, ydir);
        beam_nodes_[b.name] = builder.GetLastBeamNodes();

        const double length = (end - start).Length();
        total_structure_weight_N_ += sd.area * sd.density * length * kGravity;
    }

    // --- Cross-beams (convenience ladder) --------------------------------
    if (config_.cross_beams) {
        const auto& cb = *config_.cross_beams;
        auto ait = beam_nodes_.find(cb.between[0]);
        auto bit = beam_nodes_.find(cb.between[1]);
        if (ait == beam_nodes_.end() || bit == beam_nodes_.end()) {
            throw std::runtime_error("structure: cross_beams.between references unknown beams '" +
                                     cb.between[0] + "' / '" + cb.between[1] + "'.");
        }
        if (ait->second.size() != bit->second.size()) {
            throw std::runtime_error(
                "structure: cross_beams rails '" + cb.between[0] + "' and '" + cb.between[1] +
                "' have different node counts; they must have equal element counts.");
        }
        SectionData& sd = section_by_name(cb.section);
        const ChVector3d ydir(cb.y_direction[0], cb.y_direction[1], cb.y_direction[2]);
        auto& centre_set = node_sets_[cb.node_set];

        const auto& rail_a = ait->second;
        const auto& rail_b = bit->second;
        for (std::size_t i = 0; i < rail_a.size(); ++i) {
            builder.BuildBeam(mesh_, sd.section, std::max(2, cb.elements), rail_a[i], rail_b[i],
                              ydir);
            const auto& cross_nodes = builder.GetLastBeamNodes();
            centre_set.push_back(cross_nodes[cross_nodes.size() / 2]);

            const double length =
                (rail_b[i]->GetPos() - rail_a[i]->GetPos()).Length();
            total_structure_weight_N_ += sd.area * sd.density * length * kGravity;
        }

        // Remember rail A for the settled midspan-sag check.
        sag_rail_ = rail_a;
    }

    // Self-weight of the frame is a genuine part of the load case.
    mesh_->SetAutomaticGravity(config_.automatic_gravity);
    system.Add(mesh_);

    // --- Mates (bearings) ------------------------------------------------
    for (const auto& m : config_.mates) {
        NodePtr node = ResolveNode(m.node);
        auto body = FindBodyByName(system, m.body);
        if (!body) {
            throw std::runtime_error("structure: mate '" + m.name + "' references unknown body '" +
                                     m.body + "'.");
        }
        auto mate = chrono_types::make_shared<ChLinkMateGeneric>();
        mate->SetName(m.name);
        const ChFramed frame(node->GetPos());
        mate->Initialize(node, body, false, frame, frame);
        mate->SetConstrainedCoords(m.constrain[0], m.constrain[1], m.constrain[2], m.constrain[3],
                                   m.constrain[4], m.constrain[5]);
        system.Add(mate);
        bearings_.push_back(mate);
    }

    // --- Rigid attachments (deck planks) ---------------------------------
    for (const auto& att : config_.rigid_attachments) {
        auto nit = node_sets_.find(att.node_set);
        if (nit == node_sets_.end()) {
            throw std::runtime_error("structure: rigid_attachment '" + att.name_prefix +
                                     "' references unknown node set '" + att.node_set + "'.");
        }

        // SMC contact material for the plank (shared properties with the deck).
        auto material = chrono_types::make_shared<ChContactMaterialSMC>();
        material->SetFriction(static_cast<float>(att.material.friction));
        material->SetRestitution(static_cast<float>(att.material.restitution));
        material->SetYoungModulus(static_cast<float>(att.material.youngs_modulus));
        material->SetPoissonRatio(static_cast<float>(att.material.poisson_ratio));
        material->SetKn(static_cast<float>(att.material.normal_stiffness));
        material->SetGn(static_cast<float>(att.material.normal_damping));
        material->SetKt(static_cast<float>(att.material.tangential_stiffness));
        material->SetGt(static_cast<float>(att.material.tangential_damping));

        const double lx = att.box[0];
        const double ly = att.box[1];
        const double lz = att.box[2];
        // Thin uniform box inertia.
        const double ixx = att.mass * (ly * ly + lz * lz) / 12.0;
        const double iyy = att.mass * (lx * lx + lz * lz) / 12.0;
        const double izz = att.mass * (lx * lx + ly * ly) / 12.0;
        const ChVector3d offset(att.offset[0], att.offset[1], att.offset[2]);

        int index = 1;
        for (const auto& node : nit->second) {
            const ChVector3d pos = node->GetPos() + offset;

            auto body = chrono_types::make_shared<ChBody>();
            body->SetName(att.name_prefix + "_" + std::to_string(index));
            body->SetPos(pos);
            body->SetMass(att.mass);
            body->SetInertiaXX(ChVector3d(ixx, iyy, izz));

            auto box = chrono_types::make_shared<ChCollisionShapeBox>(material, lx, ly, lz);
            body->AddCollisionShape(box, ChFrame<>());
            // Planks share the decks' collision family with self-collisions off,
            // so plank/deck overlap generates no contact but tires contact both.
            body->GetCollisionModel()->SetFamily(att.collision_family);
            body->GetCollisionModel()->DisallowCollisionsWith(att.collision_family);
            body->EnableCollision(true);

            auto viz = chrono_types::make_shared<ChVisualShapeBox>(lx, ly, lz);
            auto vmat = chrono_types::make_shared<ChVisualMaterial>();
            vmat->SetDiffuseColor(ChColor(static_cast<float>(att.color[0]),
                                          static_cast<float>(att.color[1]),
                                          static_cast<float>(att.color[2])));
            vmat->SetOpacity(static_cast<float>(att.opacity));
            viz->AddMaterial(vmat);
            body->AddVisualShape(viz, ChFrame<>());

            system.Add(body);
            total_structure_weight_N_ += att.mass * kGravity;

            auto clamp = chrono_types::make_shared<ChLinkMateGeneric>();
            clamp->SetName(att.name_prefix + "_clamp_" + std::to_string(index));
            clamp->Initialize(body, node, false, ChFramed(pos), ChFramed(node->GetPos()));
            clamp->SetConstrainedCoords(true, true, true, true, true, true);
            system.Add(clamp);

            ++index;
        }
    }

    // --- Visualisation ---------------------------------------------------
    {
        const auto& v = config_.visualization;
        auto vis_beam = chrono_types::make_shared<ChVisualShapeFEA>();
        vis_beam->SetFEMdataType(MapDataType(v.data_type));
        vis_beam->SetColormap(MapColormap(v.colormap));
        vis_beam->SetColormapRange(v.range[0], v.range[1]);
        vis_beam->SetSmoothFaces(true);
        vis_beam->SetWireframe(false);
        vis_beam->SetBeamResolution(v.beam_resolution);
        vis_beam->SetBeamResolutionSection(v.beam_resolution_section);
        mesh_->AddVisualShapeFEA(vis_beam);
    }
}

void StructureSubsystem::OnAfterStep(double time, double /*dt*/) {
    // One-off static load-path check once the model has settled but while the
    // wave ramp still leaves the sea essentially calm: the bearings should be
    // carrying only the structure's own weight, and rail-A midspan sag should
    // be close to the analytic dead-load deflection.
    if (static_check_reported_ || time < settle_time_s_) {
        return;
    }
    static_check_reported_ = true;

    double reaction = 0.0;
    for (const auto& bearing : bearings_) {
        if (bearing) {
            reaction += std::abs(bearing->GetReaction2().force.z());
        }
    }

    // Populate the check result struct for programmatic access
    if (total_structure_weight_N_ <= 0.0) {
        static_check_result_.invalid_reason =
            "the structure has no weight, so there is no load path to check";
    } else {
        static_check_result_.valid = true;
        static_check_result_.weight_kN = total_structure_weight_N_ / 1000.0;
        static_check_result_.reactions_kN = reaction / 1000.0;
        static_check_result_.error_percent =
            100.0 * (reaction - total_structure_weight_N_) / total_structure_weight_N_;
    }

    seastack::infra::cli::LogInfo("--- Structure static check at t = " +
                                  std::to_string(settle_time_s_) + " s ---");
    if (total_structure_weight_N_ > 0.0) {
        seastack::infra::cli::LogInfo(
            "  Structure weight:         " +
            std::to_string(total_structure_weight_N_ / 1000.0) + " kN");
        seastack::infra::cli::LogInfo(
            "  Sum of bearing reactions: " + std::to_string(reaction / 1000.0) + " kN");
    }
    if (sag_rail_.size() >= 3) {
        const auto& mid = sag_rail_[sag_rail_.size() / 2];
        const auto& root = sag_rail_.front();
        const auto& tip = sag_rail_.back();
        const double sag = mid->GetPos().z() -
                           0.5 * (root->GetPos().z() + tip->GetPos().z());
        seastack::infra::cli::LogInfo(
            "  Mid-span sag, measured:   " + std::to_string(1000.0 * sag) + " mm");
    }
}

}  // namespace seastack::app
