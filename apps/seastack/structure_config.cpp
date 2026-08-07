/*********************************************************************
 * @file  structure_config.cpp
 * @brief yaml-cpp parser for the FEA structure scenario schema.
 *********************************************************************/

#include "structure_config.h"

#include "yaml_read_helpers.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace seastack::app {

namespace {

using yaml_read::ReadArray;
using yaml_read::ReadBool;
using yaml_read::ReadDouble;
using yaml_read::ReadInt;
using yaml_read::ReadString;
using yaml_read::ToUpper;

/// Read a 6-element boolean constrain mask [x, y, z, rx, ry, rz].
void ReadConstrain(const YAML::Node& node, const char* key, std::array<bool, 6>& out) {
    if (node && node[key] && node[key].IsSequence() && node[key].size() == 6) {
        for (std::size_t i = 0; i < 6; ++i) {
            out[i] = node[key][i].as<bool>();
        }
    }
}

void ParseMaterials(const YAML::Node& node, StructureConfig& cfg) {
    if (!node || !node.IsSequence()) {
        return;
    }
    for (const auto& m : node) {
        StructureMaterialConfig mat;
        ReadString(m, "name", mat.name);
        ReadDouble(m, "density", mat.density);
        ReadDouble(m, "youngs_modulus", mat.youngs_modulus);
        ReadDouble(m, "shear_modulus", mat.shear_modulus);
        cfg.materials.push_back(std::move(mat));
    }
}

void ParseSections(const YAML::Node& node, StructureConfig& cfg) {
    if (!node || !node.IsSequence()) {
        return;
    }
    for (const auto& s : node) {
        StructureSectionConfig sec;
        ReadString(s, "name", sec.name);
        ReadString(s, "type", sec.type);
        ReadString(s, "material", sec.material);
        ReadDouble(s, "width", sec.width);
        ReadDouble(s, "depth", sec.depth);
        ReadDouble(s, "wall", sec.wall);
        cfg.sections.push_back(std::move(sec));
    }
}

void ParseBeams(const YAML::Node& node, StructureConfig& cfg) {
    if (!node || !node.IsSequence()) {
        return;
    }
    for (const auto& b : node) {
        StructureBeamConfig beam;
        ReadString(b, "name", beam.name);
        ReadString(b, "section", beam.section);
        ReadArray(b, "start", beam.start);
        ReadArray(b, "end", beam.end);
        ReadInt(b, "elements", beam.elements);
        ReadArray(b, "y_direction", beam.y_direction);
        cfg.beams.push_back(std::move(beam));
    }
}

void ParseCrossBeams(const YAML::Node& node, StructureConfig& cfg) {
    if (!node) {
        return;
    }
    StructureCrossBeamsConfig cb;
    ReadString(node, "name_prefix", cb.name_prefix);
    ReadString(node, "section", cb.section);
    if (node["between"] && node["between"].IsSequence() && node["between"].size() == 2) {
        cb.between[0] = node["between"][0].as<std::string>();
        cb.between[1] = node["between"][1].as<std::string>();
    }
    ReadInt(node, "elements", cb.elements);
    ReadArray(node, "y_direction", cb.y_direction);
    ReadString(node, "node_set", cb.node_set);
    cfg.cross_beams = std::move(cb);
}

void ParseMates(const YAML::Node& node, StructureConfig& cfg) {
    if (!node || !node.IsSequence()) {
        return;
    }
    for (const auto& m : node) {
        StructureMateConfig mate;
        ReadString(m, "name", mate.name);
        ReadString(m, "node", mate.node);
        ReadString(m, "body", mate.body);
        ReadConstrain(m, "constrain", mate.constrain);
        cfg.mates.push_back(std::move(mate));
    }
}

void ParseAttachmentMaterial(const YAML::Node& node, StructureContactMaterialConfig& mat) {
    if (!node) {
        return;
    }
    ReadDouble(node, "friction", mat.friction);
    ReadDouble(node, "restitution", mat.restitution);
    ReadDouble(node, "Young_modulus", mat.youngs_modulus);
    ReadDouble(node, "Poisson_ratio", mat.poisson_ratio);
    ReadDouble(node, "normal_stiffness", mat.normal_stiffness);
    ReadDouble(node, "normal_damping", mat.normal_damping);
    ReadDouble(node, "tangential_stiffness", mat.tangential_stiffness);
    ReadDouble(node, "tangential_damping", mat.tangential_damping);
}

void ParseRigidAttachments(const YAML::Node& node, StructureConfig& cfg) {
    if (!node || !node.IsSequence()) {
        return;
    }
    for (const auto& a : node) {
        StructureRigidAttachmentConfig att;
        ReadString(a, "name_prefix", att.name_prefix);
        ReadString(a, "node_set", att.node_set);
        ReadDouble(a, "mass", att.mass);
        ReadArray(a, "box", att.box);
        ReadArray(a, "offset", att.offset);
        ReadInt(a, "collision_family", att.collision_family);
        ReadDouble(a, "opacity", att.opacity);
        ReadArray(a, "color", att.color);
        if (a["material"]) {
            ParseAttachmentMaterial(a["material"], att.material);
        }
        cfg.rigid_attachments.push_back(std::move(att));
    }
}

void ParseVisualization(const YAML::Node& node, StructureVisualizationConfig& viz) {
    if (!node) {
        return;
    }
    std::string dt = viz.data_type;
    ReadString(node, "data_type", dt);
    viz.data_type = ToUpper(dt);
    std::string cm = viz.colormap;
    ReadString(node, "colormap", cm);
    viz.colormap = ToUpper(cm);
    ReadArray(node, "range", viz.range);
    ReadInt(node, "beam_resolution", viz.beam_resolution);
    ReadInt(node, "beam_resolution_section", viz.beam_resolution_section);
}

}  // namespace

StructureConfig LoadStructureConfigFromYaml(const std::string& path) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        throw std::runtime_error("Could not load structure YAML '" + path + "': " + e.what());
    }

    const YAML::Node structure_node = root["structure"];
    if (!structure_node) {
        throw std::runtime_error("Structure YAML '" + path +
                                 "' has no top-level 'structure:' block.");
    }

    StructureConfig config;
    ReadBool(structure_node, "automatic_gravity", config.automatic_gravity);
    ParseMaterials(structure_node["materials"], config);
    ParseSections(structure_node["sections"], config);
    ParseBeams(structure_node["beams"], config);
    ParseCrossBeams(structure_node["cross_beams"], config);
    ParseMates(structure_node["mates"], config);
    ParseRigidAttachments(structure_node["rigid_attachments"], config);
    ParseVisualization(structure_node["visualization"], config.visualization);

    if (config.sections.empty()) {
        throw std::runtime_error("Structure YAML '" + path +
                                 "' defines no sections; nothing to build.");
    }
    if (config.beams.empty()) {
        throw std::runtime_error("Structure YAML '" + path +
                                 "' defines no beams; nothing to build.");
    }

    return config;
}

}  // namespace seastack::app
