/*********************************************************************
 * @file  structure_subsystem.cpp
 * @brief Implementation of StructureSubsystem.
 *
 * Builds an Euler-beam frame, mates, and rigid deck planks from a
 * StructureConfig (geometry, sections, mates, and attachments).
 *********************************************************************/

#include "structure_subsystem.h"

#include "yaml_read_helpers.h"

#include <seastack/adapters/chrono/simulation_export.h>
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
#include <chrono/fea/ChElementBeamEuler.h>
#include <chrono/fea/ChMesh.h>
#include <chrono/fea/ChNodeFEAxyzrot.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChContactMaterialSMC.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChSystem.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <unordered_map>

namespace seastack::app {

namespace {

namespace fea = ::chrono::fea;
using ::chrono::ChColor;
using ::chrono::ChFramed;
using ::chrono::ChVector3d;

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
        const StructureMaterialConfig& mat, double rayleigh_beta, double rayleigh_alpha,
        const std::optional<double>& torsion_constant) {
    auto section = chrono_types::make_shared<fea::ChBeamSectionEulerAdvanced>();
    section->SetYoungModulus(mat.youngs_modulus);
    section->SetShearModulus(mat.shear_modulus);
    section->SetDensity(mat.density);
    section->SetArea(box.area);
    section->SetIzz(box.i_strong);  // local y is vertical: resists wheel loads
    section->SetIyy(box.i_weak);
    section->SetJ(torsion_constant.has_value() ? *torsion_constant : box.j_tors);
    // Always set: Chrono's ChBeamSectionEuler defaults to beta = 0.01 s, so an
    // omitted (or zero) rayleigh_beta would otherwise still damp the beam.
    section->SetRayleighDamping(rayleigh_beta, rayleigh_alpha);
    section->SetDrawThickness(depth, width);
    return section;
}

/// Look up a model body by name; the error lists the bodies that do exist.
std::shared_ptr<::chrono::ChBody> FindBodyByName(::chrono::ChSystem& system,
                                                 const std::string& name,
                                                 const std::string& where) {
    std::vector<std::string> names;
    for (const auto& b : system.GetBodies()) {
        if (b && b->GetName() == name) {
            return b;
        }
        if (b) {
            names.push_back(b->GetName());
        }
    }
    throw std::runtime_error(where + ": no body named '" + name + "' in the model." +
                             yaml_read::DidYouMean(name, names) +
                             " Bodies in the model: " + yaml_read::JoinNames(names) + ".");
}

// Values are validated by LoadStructureConfigFromYaml; anything else is a bug.
::chrono::ChVisualShapeFEA::DataType MapDataType(const std::string& s) {
    using DT = ::chrono::ChVisualShapeFEA::DataType;
    if (s == "ELEM_BEAM_MY") return DT::ELEM_BEAM_MY;
    if (s == "ELEM_BEAM_MZ") return DT::ELEM_BEAM_MZ;
    if (s == "ELEM_BEAM_TX") return DT::ELEM_BEAM_TX;
    if (s == "NODE_SPEED_NORM") return DT::NODE_SPEED_NORM;
    if (s == "NONE") return DT::NONE;
    throw std::logic_error("structure: unsupported visualization data_type '" + s + "'");
}

::chrono::ChColormap::Type MapColormap(const std::string& s) {
    using CM = ::chrono::ChColormap::Type;
    if (s == "JET") return CM::JET;
    if (s == "INFERNO") return CM::INFERNO;
    if (s == "VIRIDIS") return CM::VIRIDIS;
    if (s == "PLASMA") return CM::PLASMA;
    throw std::logic_error("structure: unsupported visualization colormap '" + s + "'");
}

/// Reaction wrench a mate applies to its 2nd object (the support body),
/// rotated from link frame 2 into world axes: {Fx, Fy, Fz, Mx, My, Mz}.
std::array<double, 6> ReactionOnBodyWorld(const ::chrono::ChLinkMateGeneric& mate) {
    const auto wrench = mate.GetReaction2();
    const ChFramed frame2 = mate.GetFrame2Abs();
    const ChVector3d f = frame2.TransformDirectionLocalToParent(wrench.force);
    const ChVector3d m = frame2.TransformDirectionLocalToParent(wrench.torque);
    return {f.x(), f.y(), f.z(), m.x(), m.y(), m.z()};
}

}  // namespace

StructureSubsystem::StructureSubsystem(StructureConfig config, double settle_time_s)
    : config_(std::move(config)), settle_time_s_(settle_time_s) {}

StructureSubsystem::~StructureSubsystem() = default;

StructureSubsystem::NodePtr StructureSubsystem::ResolveNode(const std::string& ref) const {
    const StructureNodeRef parsed = ParseStructureNodeRef(ref);
    auto it = beam_nodes_.find(parsed.beam);
    if (it == beam_nodes_.end() || it->second.empty()) {
        throw std::runtime_error("structure: node reference '" + ref +
                                 "' names unknown or empty beam '" + parsed.beam + "'.");
    }
    const auto& nodes = it->second;
    switch (parsed.kind) {
        case StructureNodeRef::Kind::kFirst:
            return nodes.front();
        case StructureNodeRef::Kind::kLast:
            return nodes.back();
        case StructureNodeRef::Kind::kMid:
            // For an odd element count there is no centre node; this is the
            // node just past the centre.
            return nodes[nodes.size() / 2];
        case StructureNodeRef::Kind::kIndex:
            if (parsed.index >= nodes.size()) {
                throw std::runtime_error("structure: node reference '" + ref + "' is out of range (beam '" +
                                         parsed.beam + "' has " + std::to_string(nodes.size()) +
                                         " nodes).");
            }
            return nodes[parsed.index];
    }
    throw std::logic_error("structure: unhandled node reference kind");
}

std::array<double, 6> StructureSubsystem::GetMateReactionWorld(const std::string& mate_name) const {
    for (const auto& mate : mates_) {
        if (mate && mate->GetName() == mate_name) {
            return ReactionOnBodyWorld(*mate);
        }
    }
    throw std::runtime_error("structure: unknown mate '" + mate_name + "'.");
}

void StructureSubsystem::Attach(::chrono::ChSystem& system) {
    using namespace ::chrono;

    mesh_ = chrono_types::make_shared<fea::ChMesh>();

    // Weight bookkeeping uses the system's gravity, the same G that
    // ChMesh::SetAutomaticGravity applies to the beam elements.
    const ChVector3d g_acc = system.GetGravitationalAcceleration();
    gravity_ = {g_acc.x(), g_acc.y(), g_acc.z()};
    const double g = g_acc.Length();
    // Beam self-weight is only a load when the mesh applies gravity.
    const double beam_g = config_.automatic_gravity ? g : 0.0;

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
        data.section = MakeBoxBeamSection(box, s.width, s.depth, mit->second, s.rayleigh_beta,
                                          s.rayleigh_alpha, s.torsion_constant);
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

    // Section dimensions by name, for the static export of beam geometry.
    std::unordered_map<std::string, std::pair<double, double>> section_dims;  // name -> (width, depth)
    for (const auto& s : config_.sections) {
        section_dims.emplace(s.name, std::make_pair(s.width, s.depth));
    }

    // --- Explicit beams --------------------------------------------------
    for (const auto& b : config_.beams) {
        SectionData& sd = section_by_name(b.section);
        const ChVector3d start(b.start[0], b.start[1], b.start[2]);
        const ChVector3d end(b.end[0], b.end[1], b.end[2]);
        const ChVector3d ydir(b.y_direction[0], b.y_direction[1], b.y_direction[2]);
        builder.BuildBeam(mesh_, sd.section, std::max(1, b.elements), start, end, ydir);
        const auto& nodes = builder.GetLastBeamNodes();
        beam_nodes_[b.name] = nodes;
        beam_elements_[b.name] = builder.GetLastBeamElements();

        // Undeformed node positions, captured before any dynamics run.
        std::vector<std::array<double, 3>> undeformed;
        undeformed.reserve(nodes.size());
        for (const auto& n : nodes) {
            const auto& p = n->GetPos();
            undeformed.push_back({p.x(), p.y(), p.z()});
        }
        undeformed_positions_[b.name] = std::move(undeformed);

        BeamSectionInfo info;
        info.name = b.name;
        info.section_name = b.section;
        if (auto dit = section_dims.find(b.section); dit != section_dims.end()) {
            info.width = dit->second.first;
            info.depth = dit->second.second;
        }
        beam_infos_.push_back(std::move(info));

        const double length = (end - start).Length();
        total_structure_weight_N_ += sd.area * sd.density * length * beam_g;
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
            const ChVector3d axis = rail_b[i]->GetPos() - rail_a[i]->GetPos();
            if (axis.Length() <= 0.0 ||
                Vcross(axis, ydir).Length() / (axis.Length() * ydir.Length()) < 1.0e-6) {
                throw std::runtime_error(
                    "structure: cross_beams station " + std::to_string(i) +
                    " is zero-length or parallel to cross_beams.y_direction; pick a y_direction "
                    "perpendicular to the cross-beams.");
            }
            builder.BuildBeam(mesh_, sd.section, std::max(2, cb.elements), rail_a[i], rail_b[i],
                              ydir);
            const auto& cross_nodes = builder.GetLastBeamNodes();
            centre_set.push_back(cross_nodes[cross_nodes.size() / 2]);

            total_structure_weight_N_ += sd.area * sd.density * axis.Length() * beam_g;
        }

        // Remember rail A for the settled midspan-sag check.
        sag_rail_ = rail_a;
    }

    // --- Explicit node sets (resolved from node references) --------------
    // Resolved after all beams are built so references like `jib.last` see
    // their target beam.  Appends to any set of the same name created by
    // cross_beams, so the two mechanisms can coexist.
    for (const auto& ns : config_.node_sets) {
        auto& target = node_sets_[ns.name];
        for (const auto& ref : ns.nodes) {
            target.push_back(ResolveNode(ref));
        }
    }

    // Self-weight of the frame is a genuine part of the load case.
    mesh_->SetAutomaticGravity(config_.automatic_gravity);
    system.Add(mesh_);

    // --- Mates (bearings) ------------------------------------------------
    for (const auto& m : config_.mates) {
        NodePtr node = ResolveNode(m.node);
        auto body = FindBodyByName(system, m.body, "structure: mate '" + m.name + "'");
        auto mate = chrono_types::make_shared<ChLinkMateGeneric>();
        mate->SetName(m.name);
        const ChFramed frame(node->GetPos());
        mate->Initialize(node, body, false, frame, frame);
        mate->SetConstrainedCoords(m.constrain[0], m.constrain[1], m.constrain[2], m.constrain[3],
                                   m.constrain[4], m.constrain[5]);
        system.Add(mate);
        mates_.push_back(mate);
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

            // When collide is false the plank is visual + clamped only: skip the
            // collision shape entirely and disable collision, so it rides the
            // frame without generating contacts.
            if (att.collide) {
                auto box = chrono_types::make_shared<ChCollisionShapeBox>(material, lx, ly, lz);
                body->AddCollisionShape(box, ChFrame<>());
                // Planks share the decks' collision family with self-collisions off,
                // so plank/deck overlap generates no contact but tires contact both.
                body->GetCollisionModel()->SetFamily(att.collision_family);
                body->GetCollisionModel()->DisallowCollisionsWith(att.collision_family);
                body->EnableCollision(true);
            } else {
                body->EnableCollision(false);
            }

            auto viz = chrono_types::make_shared<ChVisualShapeBox>(lx, ly, lz);
            auto vmat = chrono_types::make_shared<ChVisualMaterial>();
            vmat->SetDiffuseColor(ChColor(static_cast<float>(att.color[0]),
                                          static_cast<float>(att.color[1]),
                                          static_cast<float>(att.color[2])));
            vmat->SetOpacity(static_cast<float>(att.opacity));
            viz->AddMaterial(vmat);
            body->AddVisualShape(viz, ChFrame<>());

            system.Add(body);
            total_structure_weight_N_ += att.mass * g;

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

    // Optional high-rate structural diagnostics CSV.
    if (!config_.diagnostics.file.empty()) {
        std::filesystem::path csv_path(config_.diagnostics.file);
        if (!csv_path.is_absolute() && !diagnostics_output_directory_.empty()) {
            csv_path = std::filesystem::path(diagnostics_output_directory_) / csv_path;
        }
        if (csv_path.has_parent_path()) {
            std::error_code ec;
            std::filesystem::create_directories(csv_path.parent_path(), ec);
        }
        diagnostics_csv_ = std::make_unique<std::ofstream>(csv_path.string());
        if (*diagnostics_csv_) {
            *diagnostics_csv_
                << "time_s";
            for (const auto& mate : config_.mates) {
                *diagnostics_csv_ << "," << mate.name << "_Fx_N," << mate.name << "_Fy_N,"
                                  << mate.name << "_Fz_N," << mate.name << "_Mx_Nm,"
                                  << mate.name << "_My_Nm," << mate.name << "_Mz_Nm";
            }
            for (const auto& info : beam_infos_) {
                *diagnostics_csv_ << "," << info.name << "_max_abs_Mx_Nm," << info.name
                                  << "_max_abs_My_Nm," << info.name << "_max_abs_Mz_Nm,"
                                  << info.name << "_rel_dx_m," << info.name << "_rel_dy_m,"
                                  << info.name << "_rel_dz_m";
            }
            *diagnostics_csv_ << "\n";
            seastack::infra::cli::LogInfo("structure: writing diagnostics to " +
                                          csv_path.generic_string());
        } else {
            seastack::infra::cli::LogWarning(
                "structure: could not open diagnostics CSV '" + csv_path.generic_string() + "'");
            diagnostics_csv_.reset();
        }
    }
}

void StructureSubsystem::OnAfterStep(double time, double /*dt*/) {
    // High-rate structural diagnostics (mate reactions + beam moment extrema).
    if (diagnostics_csv_) {
        ++diagnostics_step_;
        const int every = std::max(1, config_.diagnostics.every_n_steps);
        if (diagnostics_step_ % every == 0) {
            *diagnostics_csv_ << time;
            for (const auto& mate : mates_) {
                const auto r = ReactionOnBodyWorld(*mate);
                *diagnostics_csv_ << ',' << r[0] << ',' << r[1] << ',' << r[2] << ',' << r[3]
                                  << ',' << r[4] << ',' << r[5];
            }
            for (const auto& info : beam_infos_) {
                double max_mx = 0.0;
                double max_my = 0.0;
                double max_mz = 0.0;
                const auto eit = beam_elements_.find(info.name);
                if (eit != beam_elements_.end()) {
                    for (const auto& elem : eit->second) {
                        for (const double eta : {-1.0, 0.0, 1.0}) {
                            ::chrono::ChVector3d force;
                            ::chrono::ChVector3d torque;
                            elem->EvaluateSectionForceTorque(eta, force, torque);
                            max_mx = std::max(max_mx, std::abs(torque.x()));
                            max_my = std::max(max_my, std::abs(torque.y()));
                            max_mz = std::max(max_mz, std::abs(torque.z()));
                        }
                    }
                }
                double rel_dx = 0.0;
                double rel_dy = 0.0;
                double rel_dz = 0.0;
                const auto nit = beam_nodes_.find(info.name);
                const auto uit = undeformed_positions_.find(info.name);
                if (nit != beam_nodes_.end() && uit != undeformed_positions_.end() &&
                    nit->second.size() >= 2 && uit->second.size() >= 2) {
                    const auto& n0 = nit->second.front()->GetPos();
                    const auto& n1 = nit->second.back()->GetPos();
                    const auto& u0 = uit->second.front();
                    const auto& u1 = uit->second.back();
                    const double cur_dx = n1.x() - n0.x();
                    const double cur_dy = n1.y() - n0.y();
                    const double cur_dz = n1.z() - n0.z();
                    const double und_dx = u1[0] - u0[0];
                    const double und_dy = u1[1] - u0[1];
                    const double und_dz = u1[2] - u0[2];
                    rel_dx = cur_dx - und_dx;
                    rel_dy = cur_dy - und_dy;
                    rel_dz = cur_dz - und_dz;
                }
                *diagnostics_csv_ << ',' << max_mx << ',' << max_my << ',' << max_mz << ','
                                  << rel_dx << ',' << rel_dy << ',' << rel_dz;
            }
            *diagnostics_csv_ << '\n';
        }
    }

    // One-off static load-path check once the model has settled but while the
    // sea is still essentially calm: the mates should be carrying only the
    // structure's own weight.  Reactions on the support bodies are summed as
    // signed components along gravity, so supports pulling in opposite
    // directions cancel instead of inflating the total.
    if (static_check_reported_ || time < settle_time_s_) {
        return;
    }
    static_check_reported_ = true;

    const double g = std::sqrt(gravity_[0] * gravity_[0] + gravity_[1] * gravity_[1] +
                               gravity_[2] * gravity_[2]);
    double reaction = 0.0;  // N, along the gravity direction
    if (g > 0.0) {
        for (const auto& mate : mates_) {
            const auto r = ReactionOnBodyWorld(*mate);
            reaction += (r[0] * gravity_[0] + r[1] * gravity_[1] + r[2] * gravity_[2]) / g;
        }
    }

    if (total_structure_weight_N_ <= 0.0) {
        static_check_result_.invalid_reason =
            "the structure has no weight (gravity off or zero), so there is no load path to check";
    } else if (mates_.empty()) {
        static_check_result_.invalid_reason = "the structure has no mates to carry its weight";
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
            "  Structure weight:          " +
            std::to_string(total_structure_weight_N_ / 1000.0) + " kN");
        seastack::infra::cli::LogInfo(
            "  Mate reactions (gravity):  " + std::to_string(reaction / 1000.0) + " kN");
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

StructureSubsystem::StructureFrameSample
StructureSubsystem::SampleBeam(const std::string& beam_name) const {
    StructureFrameSample sample;
    sample.beam_name = beam_name;

    const auto nit = beam_nodes_.find(beam_name);
    if (nit == beam_nodes_.end()) {
        throw std::runtime_error("structure: SampleBeam references unknown beam '" + beam_name +
                                 "'.");
    }
    sample.node_positions.reserve(nit->second.size());
    sample.node_orientations_wxyz.reserve(nit->second.size());
    for (const auto& n : nit->second) {
        const auto& p = n->GetPos();
        const auto q = n->GetRot();
        sample.node_positions.push_back({p.x(), p.y(), p.z()});
        sample.node_orientations_wxyz.push_back({q.e0(), q.e1(), q.e2(), q.e3()});
    }

    const auto eit = beam_elements_.find(beam_name);
    if (eit != beam_elements_.end()) {
        sample.element_forces.reserve(eit->second.size());
        for (const auto& elem : eit->second) {
            // Section force/torque at the element mid-point (normalized eta = 0):
            // F = [Fx, Fy, Fz], M = [Mx, My, Mz] in the element local (section)
            // frame (Fx=axial, Fy/Fz=shear, Mx=torsion, My/Mz=bending).
            ::chrono::ChVector3d force;
            ::chrono::ChVector3d torque;
            elem->EvaluateSectionForceTorque(0.0, force, torque);
            sample.element_forces.push_back(
                {force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z()});
        }
    }
    return sample;
}

void StructureSubsystem::OnRecord(seastack::chrono::SimulationExporter& exporter) {
    // On the first call, publish the static undeformed beam geometry. Callers
    // invoke OnRecord after BeginResults, so /inputs is already open.
    if (!structure_static_written_) {
        std::vector<seastack::chrono::SimulationExporter::StructureExportStatic> statics;
        statics.reserve(beam_infos_.size());
        for (const auto& info : beam_infos_) {
            seastack::chrono::SimulationExporter::StructureExportStatic s;
            s.beam_name = info.name;
            s.section_width = info.width;
            s.section_depth = info.depth;
            if (const auto uit = undeformed_positions_.find(info.name);
                uit != undeformed_positions_.end()) {
                s.undeformed_node_positions = uit->second;
            }
            const int num_nodes = static_cast<int>(s.undeformed_node_positions.size());
            for (int i = 0; i + 1 < num_nodes; ++i) {
                s.connectivity.push_back(i);
                s.connectivity.push_back(i + 1);
            }
            statics.push_back(std::move(s));
        }
        exporter.WriteStructureStatic(statics);
        structure_static_written_ = true;
    }

    // Deformed frames only on recorded (post-decimation) steps, so the
    // structure geometry stays aligned with the body/time datasets.
    if (!exporter.LastStepWasRecorded()) {
        return;
    }
    std::vector<seastack::chrono::SimulationExporter::StructureExportFrame> frames;
    frames.reserve(beam_infos_.size());
    for (const auto& info : beam_infos_) {
        StructureFrameSample sample = SampleBeam(info.name);
        seastack::chrono::SimulationExporter::StructureExportFrame f;
        f.beam_name = std::move(sample.beam_name);
        f.node_positions = std::move(sample.node_positions);
        f.node_orientations_wxyz = std::move(sample.node_orientations_wxyz);
        f.element_forces = std::move(sample.element_forces);
        frames.push_back(std::move(f));
    }
    exporter.RecordStructure(frames);
}

}  // namespace seastack::app
