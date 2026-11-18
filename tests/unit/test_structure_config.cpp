/**
 * @file test_structure_config.cpp
 * @brief Strict parsing of the FEA structure YAML and the setup `checks:` block.
 *
 * Cases:
 *   S1  A complete, valid structure file parses with the expected values
 *       and defaults.
 *   S2  Each invalid input is rejected with an error that names the
 *       offending key or reference (one file per rule).
 *   S3  Node references: first/last/mid/index parse; malformed ones throw.
 *   C1  Setup checks: canonical type, the bridge alias, defaults, and the
 *       rejected forms (unknown type, unknown key, non-positive values).
 */

#include "setup_checks.h"
#include "structure_config.h"
#include "test_macros.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

static TestResults test_results;

namespace {

fs::path TempDir() {
    const fs::path dir = fs::temp_directory_path() / "seastack_test_structure_config";
    fs::create_directories(dir);
    return dir;
}

fs::path WriteFile(const std::string& name, const std::string& text) {
    const fs::path path = TempDir() / name;
    std::ofstream out(path);
    out << text;
    return path;
}

/// A valid one-beam cantilever; `beam_extra` lines go inside the beam,
/// `structure_extra` lines after the mates block.
std::string BaseStructure(const std::string& beam_extra = "",
                          const std::string& structure_extra = "") {
    return std::string(
               "structure:\n"
               "  automatic_gravity: true\n"
               "  materials:\n"
               "    - { name: alu, density: 2700.0, youngs_modulus: 7.0e+10, shear_modulus: 2.63e+10 }\n"
               "  sections:\n"
               "    - { name: rhs, type: BOX, material: alu, width: 0.3, depth: 0.3, wall: 0.01 }\n"
               "  beams:\n"
               "    - name: arm\n"
               "      section: rhs\n"
               "      start: [0.0, 0.0, 1.0]\n"
               "      end: [6.0, 0.0, 1.0]\n"
               "      elements: 6\n") +
           beam_extra +
           "  mates:\n"
           "    - { name: root, node: arm.first, body: ground, constrain: [true, true, true, true, true, true] }\n" +
           structure_extra;
}

/// Parse `text`; return the error message, or "" when it parsed.
std::string ParseError(const std::string& name, const std::string& text) {
    try {
        (void)seastack::app::LoadStructureConfigFromYaml(WriteFile(name, text).string());
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

void ExpectRejected(const std::string& name, const std::string& text,
                    const std::string& expected_fragment) {
    const std::string err = ParseError(name + ".structure.yaml", text);
    TEST_ASSERT(!err.empty(), "S2 " + name + ": rejected");
    TEST_ASSERT(err.find(expected_fragment) != std::string::npos,
                "S2 " + name + ": error mentions '" + expected_fragment + "': " + err);
}

std::string ReplaceOnce(std::string text, const std::string& from, const std::string& to) {
    const auto pos = text.find(from);
    if (pos == std::string::npos) {
        throw std::logic_error("test bug: '" + from + "' not in template");
    }
    return text.replace(pos, from.size(), to);
}

}  // namespace

int main() {
    std::cout << "test_structure_config\n";
    using seastack::app::LoadStructureConfigFromYaml;
    using seastack::app::StructureNodeRef;

    {  // S1
        const std::string text = BaseStructure(
            "      y_direction: [0.0, 0.0, 1.0]\n",
            "  node_sets:\n"
            "    - { name: tip, nodes: [arm.last, arm.3] }\n"
            "  rigid_attachments:\n"
            "    - { name_prefix: plate, node_set: tip, mass: 50.0, box: [0.2, 0.2, 0.02] }\n"
            "  visualization: { data_type: elem_beam_my, colormap: viridis, range: [-1.0, 1.0] }\n"
            "  diagnostics: { file: diag.csv, every_n_steps: 5 }\n");
        const std::string err = ParseError("valid.structure.yaml", text);
        TEST_ASSERT(err.empty(), "S1: valid file parses: " + err);
        if (err.empty()) {
            const auto cfg = LoadStructureConfigFromYaml((TempDir() / "valid.structure.yaml").string());
            TEST_ASSERT(cfg.automatic_gravity, "S1: automatic_gravity");
            TEST_ASSERT(cfg.materials.size() == 1 && cfg.sections.size() == 1 && cfg.beams.size() == 1,
                        "S1: one material, section and beam");
            TEST_NEAR(cfg.materials[0].youngs_modulus, 7.0e10, 1.0, "S1: E [Pa]");
            TEST_ASSERT(!cfg.sections[0].torsion_constant.has_value(),
                        "S1: torsion_constant absent -> default J used at build time");
            TEST_ASSERT(cfg.beams[0].elements == 6, "S1: elements");
            TEST_NEAR(cfg.beams[0].end[0], 6.0, 0.0, "S1: beam end x [m]");
            TEST_ASSERT(cfg.mates.size() == 1 && cfg.mates[0].constrain[5], "S1: mate constrain");
            TEST_ASSERT(cfg.node_sets.size() == 1 && cfg.node_sets[0].nodes.size() == 2,
                        "S1: node set");
            TEST_ASSERT(cfg.rigid_attachments.size() == 1 && cfg.rigid_attachments[0].collide,
                        "S1: rigid attachment, collide defaults to true");
            TEST_ASSERT(cfg.visualization.data_type == "ELEM_BEAM_MY" &&
                            cfg.visualization.colormap == "VIRIDIS",
                        "S1: visualization names are case-insensitive");
            TEST_ASSERT(cfg.diagnostics.every_n_steps == 5, "S1: diagnostics.every_n_steps");
        }
    }

    {  // S2: one rule per file
        const std::string base = BaseStructure();
        ExpectRejected("missing_file_block", "beams: []\n", "structure:");
        ExpectRejected("unknown_top_key", base + "  beamz: []\n", "beamz");
        ExpectRejected("unknown_root_key", base + "extra: 1\n", "extra");
        ExpectRejected("yaml_syntax", "structure: [\n", "YAML syntax error");
        ExpectRejected("missing_E",
                       ReplaceOnce(base, "youngs_modulus: 7.0e+10, ", ""), "youngs_modulus");
        ExpectRejected("negative_density", ReplaceOnce(base, "density: 2700.0", "density: -1.0"),
                       "density");
        ExpectRejected("string_for_number",
                       ReplaceOnce(base, "density: 2700.0", "density: heavy"), "density");
        ExpectRejected("unknown_material_key",
                       ReplaceOnce(base, "shear_modulus: 2.63e+10", "shear_modulus: 2.63e+10, poisson: 0.3"),
                       "poisson");
        ExpectRejected("duplicate_material",
                       ReplaceOnce(base, "  sections:\n",
                                   "    - { name: alu, density: 1.0, youngs_modulus: 1.0, shear_modulus: 1.0 }\n"
                                   "  sections:\n"),
                       "used twice");
        ExpectRejected("section_type", ReplaceOnce(base, "type: BOX", "type: I_BEAM"), "I_BEAM");
        ExpectRejected("solid_wall", ReplaceOnce(base, "wall: 0.01", "wall: 0.15"), "wall");
        ExpectRejected("negative_rayleigh",
                       ReplaceOnce(base, "wall: 0.01", "wall: 0.01, rayleigh_beta: -0.1"),
                       "rayleigh_beta");
        ExpectRejected("zero_torsion",
                       ReplaceOnce(base, "wall: 0.01", "wall: 0.01, torsion_constant: 0.0"),
                       "torsion_constant");
        ExpectRejected("unknown_material_ref", ReplaceOnce(base, "material: alu", "material: steel"),
                       "steel");
        ExpectRejected("unknown_section_ref", ReplaceOnce(base, "section: rhs", "section: rhz"),
                       "Did you mean 'rhs'");
        ExpectRejected("short_start", ReplaceOnce(base, "start: [0.0, 0.0, 1.0]", "start: [0.0, 0.0]"),
                       "start");
        ExpectRejected("zero_length", ReplaceOnce(base, "end: [6.0, 0.0, 1.0]", "end: [0.0, 0.0, 1.0]"),
                       "zero-length");
        ExpectRejected("zero_elements", ReplaceOnce(base, "elements: 6", "elements: 0"), "elements");
        ExpectRejected("ydir_parallel", BaseStructure("      y_direction: [2.0, 0.0, 0.0]\n"),
                       "parallel");
        ExpectRejected("vertical_beam_default_ydir",
                       ReplaceOnce(base, "end: [6.0, 0.0, 1.0]", "end: [0.0, 0.0, 7.0]"), "parallel");
        ExpectRejected("dot_in_beam_name", ReplaceOnce(ReplaceOnce(base, "name: arm", "name: a.rm"),
                                                       "node: arm.first", "node: a.rm.first"),
                       "must not contain '.'");
        ExpectRejected("unknown_beam_key", BaseStructure("      elemnts: 4\n"), "elemnts");
        ExpectRejected("node_out_of_range", ReplaceOnce(base, "node: arm.first", "node: arm.7"),
                       "has 7 nodes");
        ExpectRejected("node_bad_index", ReplaceOnce(base, "node: arm.first", "node: arm.3abc"),
                       "invalid qualifier");
        ExpectRejected("node_unknown_beam", ReplaceOnce(base, "node: arm.first", "node: leg.first"),
                       "leg");
        ExpectRejected("constrain_none",
                       ReplaceOnce(base, "[true, true, true, true, true, true]",
                                   "[false, false, false, false, false, false]"),
                       "no constrained direction");
        ExpectRejected("constrain_length",
                       ReplaceOnce(base, "[true, true, true, true, true, true]", "[true, true, true]"),
                       "6 booleans");
        ExpectRejected("mate_missing_body", ReplaceOnce(base, "body: ground, ", ""), "body");
        ExpectRejected("node_set_empty", base + "  node_sets:\n    - { name: tip, nodes: [] }\n",
                       "non-empty");
        ExpectRejected("attachment_unknown_set",
                       base + "  rigid_attachments:\n"
                              "    - { name_prefix: p, node_set: nowhere, mass: 1.0, box: [1, 1, 1] }\n",
                       "nowhere");
        ExpectRejected("attachment_zero_mass",
                       base + "  node_sets:\n    - { name: tip, nodes: [arm.last] }\n"
                              "  rigid_attachments:\n"
                              "    - { name_prefix: p, node_set: tip, mass: 0.0, box: [1, 1, 1] }\n",
                       "mass");
        ExpectRejected("attachment_family",
                       base + "  node_sets:\n    - { name: tip, nodes: [arm.last] }\n"
                              "  rigid_attachments:\n"
                              "    - { name_prefix: p, node_set: tip, mass: 1.0, box: [1, 1, 1], collision_family: 16 }\n",
                       "collision_family");
        ExpectRejected("viz_data_type", base + "  visualization: { data_type: ELEM_BEAM_MQ }\n",
                       "ELEM_BEAM_MQ");
        ExpectRejected("viz_colormap", base + "  visualization: { colormap: RAINBOW }\n", "RAINBOW");
        ExpectRejected("viz_range", base + "  visualization: { range: [1.0, -1.0] }\n", "min < max");
        ExpectRejected("diag_every", base + "  diagnostics: { file: d.csv, every_n_steps: 0 }\n",
                       "every_n_steps");
        ExpectRejected("cross_same_rail",
                       base + "  cross_beams: { section: rhs, between: [arm, arm], node_set: c }\n",
                       "two different beams");
        ExpectRejected("mates_not_list",
                       ReplaceOnce(base, "  mates:\n    - { name: root, node: arm.first, body: ground, "
                                         "constrain: [true, true, true, true, true, true] }\n",
                                   "  mates: oops\n"),
                       "mates");
    }

    {  // S2: missing file
        bool threw = false;
        try {
            (void)LoadStructureConfigFromYaml((TempDir() / "does_not_exist.yaml").string());
        } catch (const std::runtime_error& e) {
            threw = std::string(e.what()).find("not found") != std::string::npos;
        }
        TEST_ASSERT(threw, "S2: a missing structure_file is reported as not found");
    }

    {  // S3
        using Kind = StructureNodeRef::Kind;
        const auto first = seastack::app::ParseStructureNodeRef("jib.first");
        TEST_ASSERT(first.beam == "jib" && first.kind == Kind::kFirst, "S3: jib.first");
        const auto last = seastack::app::ParseStructureNodeRef("jib.last");
        TEST_ASSERT(last.kind == Kind::kLast, "S3: jib.last");
        const auto mid = seastack::app::ParseStructureNodeRef("jib.mid");
        TEST_ASSERT(mid.kind == Kind::kMid, "S3: jib.mid");
        const auto idx = seastack::app::ParseStructureNodeRef("jib.12");
        TEST_ASSERT(idx.kind == Kind::kIndex && idx.index == 12, "S3: jib.12");
        for (const char* bad : {"jib", ".first", "jib.", "jib.-1", "jib.1.5x", "jib.tip"}) {
            bool threw = false;
            try {
                (void)seastack::app::ParseStructureNodeRef(bad);
            } catch (const std::runtime_error&) {
                threw = true;
            }
            TEST_ASSERT(threw, std::string("S3: '") + bad + "' is rejected");
        }
    }

    {  // C1
        using seastack::app::LoadChecksFromSetupYaml;
        const auto none = LoadChecksFromSetupYaml(WriteFile("none.setup.yaml", "model_file: m.yaml\n"));
        TEST_ASSERT(none.empty(), "C1: no checks block -> no checks");

        const auto canon = LoadChecksFromSetupYaml(WriteFile(
            "canon.setup.yaml",
            "model_file: m.yaml\nchecks:\n  - { type: structure_static_load_path, time_s: 1.5 }\n"));
        TEST_ASSERT(canon.size() == 1 && canon[0].type == seastack::app::kStructureStaticLoadPathCheck,
                    "C1: canonical type");
        if (!canon.empty()) {
            TEST_NEAR(canon[0].time_s, 1.5, 0.0, "C1: time_s [s]");
            TEST_NEAR(canon[0].tolerance_percent, 10.0, 0.0, "C1: default tolerance [%]");
        }

        const auto alias = LoadChecksFromSetupYaml(WriteFile(
            "alias.setup.yaml", "checks:\n  - { type: bridge_static_load_path, tolerance_percent: 2 }\n"));
        TEST_ASSERT(alias.size() == 1 && alias[0].type == seastack::app::kStructureStaticLoadPathCheck,
                    "C1: bridge_static_load_path is an alias");

        auto rejected = [](const std::string& name, const std::string& text, const std::string& frag) {
            try {
                (void)LoadChecksFromSetupYaml(WriteFile(name, text));
            } catch (const std::runtime_error& e) {
                return std::string(e.what()).find(frag) != std::string::npos;
            }
            return false;
        };
        TEST_ASSERT(rejected("c_type.setup.yaml", "checks:\n  - { type: structure_static_loadpath }\n",
                             "Did you mean"),
                    "C1: unknown check type rejected with a suggestion");
        TEST_ASSERT(rejected("c_key.setup.yaml",
                             "checks:\n  - { type: structure_static_load_path, tolerance: 5 }\n",
                             "tolerance"),
                    "C1: unknown key rejected");
        TEST_ASSERT(rejected("c_time.setup.yaml",
                             "checks:\n  - { type: structure_static_load_path, time_s: 0 }\n",
                             "time_s"),
                    "C1: time_s <= 0 rejected");
        TEST_ASSERT(rejected("c_tol.setup.yaml",
                             "checks:\n  - { type: structure_static_load_path, tolerance_percent: -1 }\n",
                             "tolerance_percent"),
                    "C1: tolerance <= 0 rejected");
        TEST_ASSERT(rejected("c_seq.setup.yaml", "checks: structure_static_load_path\n", "checks"),
                    "C1: checks must be a list");
    }

    test_results.Summary();
    return test_results.failed == 0 ? 0 : 1;
}
