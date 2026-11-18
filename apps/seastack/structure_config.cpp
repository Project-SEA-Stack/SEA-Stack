/*********************************************************************
 * @file  structure_config.cpp
 * @brief yaml-cpp parser and validator for the FEA structure schema.
 *********************************************************************/

#include "structure_config.h"

#include "yaml_read_helpers.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>

namespace seastack::app {

namespace {

namespace yr = yaml_read;

/// Accepted `visualization.data_type` and `visualization.colormap` values.
const std::vector<std::string> kDataTypes = {"ELEM_BEAM_MY", "ELEM_BEAM_MZ", "ELEM_BEAM_TX",
                                             "NODE_SPEED_NORM", "NONE"};
const std::vector<std::string> kColormaps = {"JET", "INFERNO", "VIRIDIS", "PLASMA"};

std::string Item(const std::string& where, const char* list, std::size_t i) {
    return where + ": " + list + "[" + std::to_string(i) + "]";
}

void RequirePositive(double value, const std::string& what) {
    if (!(value > 0.0)) {
        throw std::runtime_error(what + " must be > 0 (got " + std::to_string(value) + ").");
    }
}

void RequireNonNegative(double value, const std::string& what) {
    if (value < 0.0) {
        throw std::runtime_error(what + " must be >= 0 (got " + std::to_string(value) + ").");
    }
}

void RequireUniqueName(const std::string& name, std::set<std::string>& seen,
                       const std::string& where, const char* kind) {
    if (name.empty()) {
        throw std::runtime_error(where + ".name must not be empty.");
    }
    if (!seen.insert(name).second) {
        throw std::runtime_error(where + ": " + kind + " name '" + name + "' is used twice.");
    }
}

void RequireKnown(const std::string& name, const std::set<std::string>& known,
                  const std::string& what, const char* kind) {
    if (known.count(name) == 0) {
        const std::vector<std::string> names(known.begin(), known.end());
        throw std::runtime_error(what + " references unknown " + kind + " '" + name + "'." +
                                 yr::DidYouMean(name, names) + " Defined " + kind +
                                 "s: " + yr::JoinNames(names) + ".");
    }
}

double Norm(const std::array<double, 3>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/// Throw unless y_direction is usable as the section's local y for a beam
/// along `axis`: non-zero and not parallel to the beam.
void RequireValidYDirection(const std::array<double, 3>& axis, const std::array<double, 3>& ydir,
                            const std::string& what) {
    const double ny = Norm(ydir);
    if (!(ny > 0.0)) {
        throw std::runtime_error(what + ".y_direction must be a non-zero vector.");
    }
    const double na = Norm(axis);
    const std::array<double, 3> c = {axis[1] * ydir[2] - axis[2] * ydir[1],
                                     axis[2] * ydir[0] - axis[0] * ydir[2],
                                     axis[0] * ydir[1] - axis[1] * ydir[0]};
    // sin(angle) between the beam axis and y_direction.
    if (Norm(c) / (na * ny) < 1.0e-6) {
        throw std::runtime_error(what + ".y_direction is parallel to the beam axis, so the "
                                        "section orientation is undefined. Pick a direction "
                                        "perpendicular to the beam (it sets the local y axis).");
    }
}

void ReadConstrain(const YAML::Node& node, std::array<bool, 6>& out, const std::string& where) {
    const YAML::Node seq = node["constrain"];
    if (!seq.IsSequence() || seq.size() != 6) {
        throw std::runtime_error(where + ".constrain must be a list of 6 booleans "
                                         "[x, y, z, rot_x, rot_y, rot_z] in world axes.");
    }
    for (std::size_t i = 0; i < 6; ++i) {
        try {
            out[i] = seq[i].as<bool>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(where + ".constrain[" + std::to_string(i) +
                                     "] must be true or false.");
        }
    }
}

void ParseMaterials(const YAML::Node& node, StructureConfig& cfg, const std::string& where) {
    yr::RequireSequence(node, where + ": materials");
    std::set<std::string> names;
    for (std::size_t i = 0; i < node.size(); ++i) {
        const YAML::Node m = node[i];
        const std::string w = Item(where, "materials", i);
        yr::CheckKeys(m, {"name", "density", "youngs_modulus", "shear_modulus"}, w);
        for (const char* key : {"name", "density", "youngs_modulus", "shear_modulus"}) {
            yr::RequireKey(m, key, w);
        }
        StructureMaterialConfig mat;
        yr::ReadString(m, "name", mat.name, w);
        yr::ReadDouble(m, "density", mat.density, w);
        yr::ReadDouble(m, "youngs_modulus", mat.youngs_modulus, w);
        yr::ReadDouble(m, "shear_modulus", mat.shear_modulus, w);
        RequireUniqueName(mat.name, names, w, "material");
        RequirePositive(mat.density, w + ".density [kg/m^3]");
        RequirePositive(mat.youngs_modulus, w + ".youngs_modulus [Pa]");
        RequirePositive(mat.shear_modulus, w + ".shear_modulus [Pa]");
        cfg.materials.push_back(std::move(mat));
    }
}

void ParseSections(const YAML::Node& node, StructureConfig& cfg, const std::string& where) {
    yr::RequireSequence(node, where + ": sections");
    std::set<std::string> names;
    for (std::size_t i = 0; i < node.size(); ++i) {
        const YAML::Node s = node[i];
        const std::string w = Item(where, "sections", i);
        yr::CheckKeys(s, {"name", "type", "material", "width", "depth", "wall", "rayleigh_beta",
                          "rayleigh_alpha", "torsion_constant"},
                      w);
        for (const char* key : {"name", "material", "width", "depth", "wall"}) {
            yr::RequireKey(s, key, w);
        }
        StructureSectionConfig sec;
        yr::ReadString(s, "name", sec.name, w);
        yr::ReadString(s, "type", sec.type, w);
        sec.type = yr::ToUpper(sec.type);
        yr::ReadString(s, "material", sec.material, w);
        yr::ReadDouble(s, "width", sec.width, w);
        yr::ReadDouble(s, "depth", sec.depth, w);
        yr::ReadDouble(s, "wall", sec.wall, w);
        yr::ReadDouble(s, "rayleigh_beta", sec.rayleigh_beta, w);
        yr::ReadDouble(s, "rayleigh_alpha", sec.rayleigh_alpha, w);
        if (s["torsion_constant"]) {
            double j = 0.0;
            yr::ReadDouble(s, "torsion_constant", j, w);
            RequirePositive(j, w + ".torsion_constant [m^4]");
            sec.torsion_constant = j;
        }

        RequireUniqueName(sec.name, names, w, "section");
        if (sec.type != "BOX") {
            throw std::runtime_error(w + ".type '" + sec.type +
                                     "' is not supported (only BOX: a rectangular hollow section).");
        }
        RequirePositive(sec.width, w + ".width [m]");
        RequirePositive(sec.depth, w + ".depth [m]");
        RequirePositive(sec.wall, w + ".wall [m]");
        if (!(2.0 * sec.wall < sec.width) || !(2.0 * sec.wall < sec.depth)) {
            throw std::runtime_error(w + ".wall must be less than half of both width and depth "
                                         "(a BOX is hollow).");
        }
        RequireNonNegative(sec.rayleigh_beta, w + ".rayleigh_beta [s]");
        RequireNonNegative(sec.rayleigh_alpha, w + ".rayleigh_alpha [1/s]");
        cfg.sections.push_back(std::move(sec));
    }
}

void ParseBeams(const YAML::Node& node, StructureConfig& cfg, const std::string& where) {
    yr::RequireSequence(node, where + ": beams");
    std::set<std::string> names;
    for (std::size_t i = 0; i < node.size(); ++i) {
        const YAML::Node b = node[i];
        const std::string w = Item(where, "beams", i);
        yr::CheckKeys(b, {"name", "section", "start", "end", "elements", "y_direction"}, w);
        for (const char* key : {"name", "section", "start", "end", "elements"}) {
            yr::RequireKey(b, key, w);
        }
        StructureBeamConfig beam;
        yr::ReadString(b, "name", beam.name, w);
        yr::ReadString(b, "section", beam.section, w);
        yr::ReadArray(b, "start", beam.start, w);
        yr::ReadArray(b, "end", beam.end, w);
        yr::ReadInt(b, "elements", beam.elements, w);
        yr::ReadArray(b, "y_direction", beam.y_direction, w);

        RequireUniqueName(beam.name, names, w, "beam");
        if (beam.name.find('.') != std::string::npos) {
            throw std::runtime_error(w + ".name '" + beam.name +
                                     "' must not contain '.', which separates node references "
                                     "such as beam.first.");
        }
        if (beam.elements < 1) {
            throw std::runtime_error(w + ".elements must be >= 1.");
        }
        const std::array<double, 3> axis = {beam.end[0] - beam.start[0], beam.end[1] - beam.start[1],
                                            beam.end[2] - beam.start[2]};
        if (!(Norm(axis) > 0.0)) {
            throw std::runtime_error(w + ": start and end are the same point (zero-length beam).");
        }
        RequireValidYDirection(axis, beam.y_direction, w);
        cfg.beams.push_back(std::move(beam));
    }
}

void ParseCrossBeams(const YAML::Node& node, StructureConfig& cfg, const std::string& where) {
    const std::string w = where + ": cross_beams";
    yr::CheckKeys(node, {"name_prefix", "section", "between", "elements", "y_direction", "node_set"},
                  w);
    for (const char* key : {"section", "between"}) {
        yr::RequireKey(node, key, w);
    }
    StructureCrossBeamsConfig cb;
    yr::ReadString(node, "name_prefix", cb.name_prefix, w);
    yr::ReadString(node, "section", cb.section, w);
    const YAML::Node between = node["between"];
    if (!between.IsSequence() || between.size() != 2 || !between[0].IsScalar() ||
        !between[1].IsScalar()) {
        throw std::runtime_error(w + ".between must be a list of two beam names.");
    }
    cb.between[0] = between[0].Scalar();
    cb.between[1] = between[1].Scalar();
    if (cb.between[0] == cb.between[1]) {
        throw std::runtime_error(w + ".between must name two different beams.");
    }
    yr::ReadInt(node, "elements", cb.elements, w);
    yr::ReadArray(node, "y_direction", cb.y_direction, w);
    yr::ReadString(node, "node_set", cb.node_set, w);
    if (cb.elements < 2) {
        throw std::runtime_error(w + ".elements must be >= 2 so each cross-beam has a mid node.");
    }
    if (!(Norm(cb.y_direction) > 0.0)) {
        throw std::runtime_error(w + ".y_direction must be a non-zero vector.");
    }
    if (cb.node_set.empty()) {
        throw std::runtime_error(w + ".node_set must not be empty.");
    }
    cfg.cross_beams = std::move(cb);
}

void ParseNodeSets(const YAML::Node& node, StructureConfig& cfg, const std::string& where) {
    yr::RequireSequence(node, where + ": node_sets");
    for (std::size_t i = 0; i < node.size(); ++i) {
        const YAML::Node s = node[i];
        const std::string w = Item(where, "node_sets", i);
        yr::CheckKeys(s, {"name", "nodes"}, w);
        yr::RequireKey(s, "name", w);
        yr::RequireKey(s, "nodes", w);
        StructureNodeSetConfig ns;
        yr::ReadString(s, "name", ns.name, w);
        if (ns.name.empty()) {
            throw std::runtime_error(w + ".name must not be empty.");
        }
        const YAML::Node nodes = s["nodes"];
        if (!nodes.IsSequence() || nodes.size() == 0) {
            throw std::runtime_error(w + ".nodes must be a non-empty list of node references "
                                         "such as [jib.last].");
        }
        for (const auto& ref : nodes) {
            if (!ref.IsScalar()) {
                throw std::runtime_error(w + ".nodes must contain only node references.");
            }
            ns.nodes.push_back(ref.Scalar());
        }
        cfg.node_sets.push_back(std::move(ns));
    }
}

void ParseMates(const YAML::Node& node, StructureConfig& cfg, const std::string& where) {
    yr::RequireSequence(node, where + ": mates");
    std::set<std::string> names;
    for (std::size_t i = 0; i < node.size(); ++i) {
        const YAML::Node m = node[i];
        const std::string w = Item(where, "mates", i);
        yr::CheckKeys(m, {"name", "node", "body", "constrain"}, w);
        for (const char* key : {"name", "node", "body", "constrain"}) {
            yr::RequireKey(m, key, w);
        }
        StructureMateConfig mate;
        yr::ReadString(m, "name", mate.name, w);
        yr::ReadString(m, "node", mate.node, w);
        yr::ReadString(m, "body", mate.body, w);
        ReadConstrain(m, mate.constrain, w);
        RequireUniqueName(mate.name, names, w, "mate");
        if (mate.body.empty()) {
            throw std::runtime_error(w + ".body must name a body from the model YAML.");
        }
        if (std::none_of(mate.constrain.begin(), mate.constrain.end(), [](bool c) { return c; })) {
            throw std::runtime_error(w + ".constrain has no constrained direction, so the mate "
                                         "does nothing. Set at least one entry to true.");
        }
        cfg.mates.push_back(std::move(mate));
    }
}

void ParseAttachmentMaterial(const YAML::Node& node, StructureContactMaterialConfig& mat,
                             const std::string& where) {
    // Young_modulus / Poisson_ratio follow the Chrono model-YAML spelling.
    yr::CheckKeys(node, {"friction", "restitution", "Young_modulus", "Poisson_ratio",
                         "normal_stiffness", "normal_damping", "tangential_stiffness",
                         "tangential_damping"},
                  where);
    yr::ReadDouble(node, "friction", mat.friction, where);
    yr::ReadDouble(node, "restitution", mat.restitution, where);
    yr::ReadDouble(node, "Young_modulus", mat.youngs_modulus, where);
    yr::ReadDouble(node, "Poisson_ratio", mat.poisson_ratio, where);
    yr::ReadDouble(node, "normal_stiffness", mat.normal_stiffness, where);
    yr::ReadDouble(node, "normal_damping", mat.normal_damping, where);
    yr::ReadDouble(node, "tangential_stiffness", mat.tangential_stiffness, where);
    yr::ReadDouble(node, "tangential_damping", mat.tangential_damping, where);
    RequireNonNegative(mat.friction, where + ".friction");
    if (mat.restitution < 0.0 || mat.restitution > 1.0) {
        throw std::runtime_error(where + ".restitution must be in [0, 1].");
    }
    RequirePositive(mat.youngs_modulus, where + ".Young_modulus [Pa]");
    if (mat.poisson_ratio < 0.0 || mat.poisson_ratio >= 0.5) {
        throw std::runtime_error(where + ".Poisson_ratio must be in [0, 0.5).");
    }
    RequireNonNegative(mat.normal_stiffness, where + ".normal_stiffness");
    RequireNonNegative(mat.normal_damping, where + ".normal_damping");
    RequireNonNegative(mat.tangential_stiffness, where + ".tangential_stiffness");
    RequireNonNegative(mat.tangential_damping, where + ".tangential_damping");
}

void ParseRigidAttachments(const YAML::Node& node, StructureConfig& cfg,
                           const std::string& where) {
    yr::RequireSequence(node, where + ": rigid_attachments");
    for (std::size_t i = 0; i < node.size(); ++i) {
        const YAML::Node a = node[i];
        const std::string w = Item(where, "rigid_attachments", i);
        yr::CheckKeys(a, {"name_prefix", "node_set", "mass", "box", "offset", "collide",
                          "collision_family", "opacity", "color", "material"},
                      w);
        for (const char* key : {"node_set", "mass", "box"}) {
            yr::RequireKey(a, key, w);
        }
        StructureRigidAttachmentConfig att;
        yr::ReadString(a, "name_prefix", att.name_prefix, w);
        yr::ReadString(a, "node_set", att.node_set, w);
        yr::ReadDouble(a, "mass", att.mass, w);
        yr::ReadArray(a, "box", att.box, w);
        yr::ReadArray(a, "offset", att.offset, w);
        yr::ReadBool(a, "collide", att.collide, w);
        yr::ReadInt(a, "collision_family", att.collision_family, w);
        yr::ReadDouble(a, "opacity", att.opacity, w);
        yr::ReadArray(a, "color", att.color, w);
        if (a["material"]) {
            ParseAttachmentMaterial(a["material"], att.material, w + ".material");
        }
        if (att.name_prefix.empty()) {
            throw std::runtime_error(w + ".name_prefix must not be empty.");
        }
        RequirePositive(att.mass, w + ".mass [kg]");
        for (std::size_t k = 0; k < 3; ++k) {
            RequirePositive(att.box[k], w + ".box[" + std::to_string(k) + "] [m]");
        }
        if (att.collision_family < 0 || att.collision_family > 15) {
            throw std::runtime_error(w + ".collision_family must be in 0..15 (Chrono families).");
        }
        if (att.opacity < 0.0 || att.opacity > 1.0) {
            throw std::runtime_error(w + ".opacity must be in [0, 1].");
        }
        for (double c : att.color) {
            if (c < 0.0 || c > 1.0) {
                throw std::runtime_error(w + ".color entries must be in [0, 1] (RGB).");
            }
        }
        cfg.rigid_attachments.push_back(std::move(att));
    }
}

void ParseVisualization(const YAML::Node& node, StructureVisualizationConfig& viz,
                        const std::string& where) {
    const std::string w = where + ": visualization";
    yr::CheckKeys(node, {"data_type", "colormap", "range", "beam_resolution",
                         "beam_resolution_section"},
                  w);
    std::string dt = viz.data_type;
    yr::ReadString(node, "data_type", dt, w);
    viz.data_type = yr::ToUpper(dt);
    if (std::find(kDataTypes.begin(), kDataTypes.end(), viz.data_type) == kDataTypes.end()) {
        throw std::runtime_error(w + ".data_type '" + viz.data_type + "' is not supported." +
                                 yr::DidYouMean(viz.data_type, kDataTypes) +
                                 " Supported: " + yr::JoinNames(kDataTypes) + ".");
    }
    std::string cm = viz.colormap;
    yr::ReadString(node, "colormap", cm, w);
    viz.colormap = yr::ToUpper(cm);
    if (std::find(kColormaps.begin(), kColormaps.end(), viz.colormap) == kColormaps.end()) {
        throw std::runtime_error(w + ".colormap '" + viz.colormap + "' is not supported." +
                                 " Supported: " + yr::JoinNames(kColormaps) + ".");
    }
    yr::ReadArray(node, "range", viz.range, w);
    if (!(viz.range[0] < viz.range[1])) {
        throw std::runtime_error(w + ".range must be [min, max] with min < max.");
    }
    yr::ReadInt(node, "beam_resolution", viz.beam_resolution, w);
    yr::ReadInt(node, "beam_resolution_section", viz.beam_resolution_section, w);
    if (viz.beam_resolution < 1 || viz.beam_resolution_section < 3) {
        throw std::runtime_error(w + ": beam_resolution must be >= 1 and "
                                     "beam_resolution_section >= 3.");
    }
}

void ParseDiagnostics(const YAML::Node& node, StructureDiagnosticsConfig& diag,
                      const std::string& where) {
    const std::string w = where + ": diagnostics";
    yr::CheckKeys(node, {"file", "every_n_steps"}, w);
    yr::ReadString(node, "file", diag.file, w);
    yr::ReadInt(node, "every_n_steps", diag.every_n_steps, w);
    if (diag.every_n_steps < 1) {
        throw std::runtime_error(w + ".every_n_steps must be >= 1.");
    }
}

/// Cross-reference checks that need the whole config.
void ValidateReferences(const StructureConfig& cfg, const std::string& where) {
    std::set<std::string> materials;
    for (const auto& m : cfg.materials) {
        materials.insert(m.name);
    }
    std::set<std::string> sections;
    for (std::size_t i = 0; i < cfg.sections.size(); ++i) {
        RequireKnown(cfg.sections[i].material, materials,
                     Item(where, "sections", i) + ".material", "material");
        sections.insert(cfg.sections[i].name);
    }
    std::set<std::string> beams;
    for (std::size_t i = 0; i < cfg.beams.size(); ++i) {
        RequireKnown(cfg.beams[i].section, sections, Item(where, "beams", i) + ".section",
                     "section");
        beams.insert(cfg.beams[i].name);
    }

    std::set<std::string> node_sets;
    if (cfg.cross_beams) {
        const auto& cb = *cfg.cross_beams;
        RequireKnown(cb.section, sections, where + ": cross_beams.section", "section");
        for (const auto& rail : cb.between) {
            RequireKnown(rail, beams, where + ": cross_beams.between", "beam");
        }
        node_sets.insert(cb.node_set);
    }

    // Node references must name a defined beam and an index inside it.
    auto check_ref = [&](const std::string& ref, const std::string& what) {
        StructureNodeRef parsed;
        try {
            parsed = ParseStructureNodeRef(ref);
        } catch (const std::exception& e) {
            throw std::runtime_error(what + ": " + e.what());
        }
        RequireKnown(parsed.beam, beams, what + " '" + ref + "'", "beam");
        if (parsed.kind == StructureNodeRef::Kind::kIndex) {
            const auto it = std::find_if(cfg.beams.begin(), cfg.beams.end(),
                                         [&](const auto& b) { return b.name == parsed.beam; });
            const std::size_t num_nodes = static_cast<std::size_t>(it->elements) + 1;
            if (parsed.index >= num_nodes) {
                throw std::runtime_error(what + " '" + ref + "': beam '" + parsed.beam + "' has " +
                                         std::to_string(num_nodes) + " nodes (indices 0.." +
                                         std::to_string(num_nodes - 1) + ").");
            }
        }
    };
    for (std::size_t i = 0; i < cfg.node_sets.size(); ++i) {
        for (const auto& ref : cfg.node_sets[i].nodes) {
            check_ref(ref, Item(where, "node_sets", i) + ".nodes");
        }
        node_sets.insert(cfg.node_sets[i].name);
    }
    for (std::size_t i = 0; i < cfg.mates.size(); ++i) {
        check_ref(cfg.mates[i].node, Item(where, "mates", i) + ".node");
    }
    for (std::size_t i = 0; i < cfg.rigid_attachments.size(); ++i) {
        RequireKnown(cfg.rigid_attachments[i].node_set, node_sets,
                     Item(where, "rigid_attachments", i) + ".node_set", "node set");
    }
}

}  // namespace

StructureNodeRef ParseStructureNodeRef(const std::string& ref) {
    const auto dot = ref.rfind('.');
    if (dot == std::string::npos || dot == 0 || dot + 1 == ref.size()) {
        throw std::runtime_error("node reference '" + ref +
                                 "' must be of the form beam.first, beam.last, beam.mid or "
                                 "beam.<index>.");
    }
    StructureNodeRef out;
    out.beam = ref.substr(0, dot);
    const std::string qual = ref.substr(dot + 1);
    if (qual == "first") {
        out.kind = StructureNodeRef::Kind::kFirst;
    } else if (qual == "last") {
        out.kind = StructureNodeRef::Kind::kLast;
    } else if (qual == "mid") {
        out.kind = StructureNodeRef::Kind::kMid;
    } else if (std::all_of(qual.begin(), qual.end(),
                           [](unsigned char c) { return std::isdigit(c) != 0; })) {
        out.kind = StructureNodeRef::Kind::kIndex;
        out.index = static_cast<std::size_t>(std::stoull(qual));
    } else {
        throw std::runtime_error("node reference '" + ref + "' has an invalid qualifier '" + qual +
                                 "' (use first, last, mid or a 0-based index).");
    }
    return out;
}

StructureConfig LoadStructureConfigFromYaml(const std::string& path) {
    const std::string where = "structure_file " + std::filesystem::path(path).filename().string();
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("structure_file not found: " + path);
    }
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(where + ": YAML syntax error: " + e.what());
    }
    if (!root.IsMap() || !root["structure"]) {
        throw std::runtime_error(where + " has no top-level 'structure:' block.");
    }
    yr::CheckKeys(root, {"structure"}, where);

    const YAML::Node s = root["structure"];
    const std::string w = where + ": structure";
    yr::CheckKeys(s, {"automatic_gravity", "materials", "sections", "beams", "cross_beams",
                      "node_sets", "mates", "rigid_attachments", "visualization", "diagnostics"},
                  w);
    for (const char* key : {"materials", "sections", "beams"}) {
        yr::RequireKey(s, key, w);
    }

    StructureConfig config;
    yr::ReadBool(s, "automatic_gravity", config.automatic_gravity, w);
    ParseMaterials(s["materials"], config, where);
    ParseSections(s["sections"], config, where);
    ParseBeams(s["beams"], config, where);
    if (s["cross_beams"] && !s["cross_beams"].IsNull()) {
        ParseCrossBeams(s["cross_beams"], config, where);
    }
    if (s["node_sets"] && !s["node_sets"].IsNull()) {
        ParseNodeSets(s["node_sets"], config, where);
    }
    if (s["mates"] && !s["mates"].IsNull()) {
        ParseMates(s["mates"], config, where);
    }
    if (s["rigid_attachments"] && !s["rigid_attachments"].IsNull()) {
        ParseRigidAttachments(s["rigid_attachments"], config, where);
    }
    if (s["visualization"] && !s["visualization"].IsNull()) {
        ParseVisualization(s["visualization"], config.visualization, where);
    }
    if (s["diagnostics"] && !s["diagnostics"].IsNull()) {
        ParseDiagnostics(s["diagnostics"], config.diagnostics, where);
    }

    if (config.materials.empty() || config.sections.empty() || config.beams.empty()) {
        throw std::runtime_error(where + " must define at least one material, section and beam.");
    }
    ValidateReferences(config, where);
    return config;
}

}  // namespace seastack::app
