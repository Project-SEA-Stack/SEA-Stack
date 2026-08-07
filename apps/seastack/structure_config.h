/*********************************************************************
 * @file  structure_config.h
 * @brief Declarative Chrono::FEA structure scenario, parsed from YAML.
 *
 * A "structure" is an Euler-beam frame (girders + cross-beams), its end
 * mates to existing model bodies, and optional rigid attachments (deck
 * planks) clamped to the frame.  A linkspan (or any grillage of Euler beams)
 * can be described in YAML instead of code.  Nothing here depends on Chrono;
 * StructureSubsystem (structure_subsystem.h) turns this config into live FEA
 * objects.
 *
 * Frame / unit / sign conventions (must stay explicit):
 *   - z up, undisturbed free surface at z = 0, lengths in metres, SI units.
 *   - A beam's `y_direction` sets the section's LOCAL y axis (Chrono's Ydir
 *     for ChBuilderBeamEuler).  With y_direction = [0,0,1] the local y is
 *     vertical, so ChElementBeamEuler resists vertical (wheel) loads with
 *     EIzz and the matching section bending moment is Mz (ELEM_BEAM_MZ) —
 *     NOT My.  Getting this backwards silently mislabels the stress plot.
 *   - Rectangular hollow section second moments: `depth` is the dimension
 *     along the beam's local y (Ydir); `i_strong` (depth cubed) is assigned
 *     to Izz, `i_weak` to Iyy, so Izz is the strong axis for vertical loads.
 *********************************************************************/

#ifndef SEASTACK_APP_STRUCTURE_CONFIG_H
#define SEASTACK_APP_STRUCTURE_CONFIG_H

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace seastack::app {

/// Elastic material for the Euler-beam sections (linear isotropic).
struct StructureMaterialConfig {
    std::string name;
    double density = 2700.0;         ///< kg/m^3
    double youngs_modulus = 7.0e10;  ///< Pa
    double shear_modulus = 2.63e10;  ///< Pa (E / 2(1+nu))
};

/// A rectangular hollow section (box).  `width` is the local-z thickness,
/// `depth` the local-y thickness (see the header note on axis mapping).
struct StructureSectionConfig {
    std::string name;
    std::string type = "BOX";  ///< only BOX is supported at present
    std::string material;      ///< references StructureMaterialConfig::name
    double width = 0.0;        ///< m, section extent across local z
    double depth = 0.0;        ///< m, section extent across local y (Ydir)
    double wall = 0.0;         ///< m, wall thickness of the hollow box
};

/// A single explicit Euler beam built from `start` to `end`.
struct StructureBeamConfig {
    std::string name;
    std::string section;                     ///< references a section name
    std::array<double, 3> start{0, 0, 0};    ///< m, world
    std::array<double, 3> end{0, 0, 0};      ///< m, world
    int elements = 1;                        ///< number of ChElementBeamEuler
    std::array<double, 3> y_direction{0, 0, 1};  ///< local y axis (Ydir)
};

/// Convenience generator: one transverse cross-beam per node station of two
/// parallel "rail" beams (the two girders), each split into `elements`
/// elements so the middle node lands between the rails.  The middle node of
/// every cross-beam is registered under `node_set`, which rigid attachments
/// (planks) then reference.  Builds a cross-beam ladder without listing each
/// member by hand.
struct StructureCrossBeamsConfig {
    std::string name_prefix = "cross";
    std::string section;
    std::array<std::string, 2> between{"", ""};  ///< the two rail beam names
    int elements = 2;                            ///< per cross-beam (>=2 for a mid node)
    std::array<double, 3> y_direction{0, 0, 1};
    std::string node_set = "cross_centres";      ///< tag applied to the mid nodes
};

/// A mate (ChLinkMateGeneric) between one FEA node and an existing model
/// body.  `node` is a node reference of the form `beam.first`, `beam.last`,
/// `beam.mid`, or `beam.N` (0-based index).  Link frames are world-aligned
/// and located at the node's world position, so `constrain` reads directly
/// as [x, y, z, rot_x, rot_y, rot_z] in world axes.
struct StructureMateConfig {
    std::string name;
    std::string node;   ///< node reference (beam.first / .last / .mid / .N)
    std::string body;   ///< existing body name in the Chrono model
    std::array<bool, 6> constrain{false, false, false, false, false, false};
};

/// Optional SMC contact material for a rigid attachment's collision box.
/// Defaults are typical steel-deck contact values so a plank sees the same
/// tire contact properties as the hull decks.
struct StructureContactMaterialConfig {
    double friction = 0.9;
    double restitution = 0.01;
    double youngs_modulus = 2.0e7;
    double poisson_ratio = 0.3;
    double normal_stiffness = 2.0e5;
    double normal_damping = 40.0;
    double tangential_stiffness = 2.0e5;
    double tangential_damping = 20.0;
};

/// Rigid bodies (deck planks) clamped in all 6 DOF to every node in a node
/// set.  One body per node; body inertia is computed from the box extents
/// and `mass` (a thin uniform plate).  `offset` places the body centre
/// relative to its node (e.g. lift the plank onto the top of the frame).
struct StructureRigidAttachmentConfig {
    std::string name_prefix = "plank";
    std::string node_set;                     ///< which node set to attach to
    double mass = 0.0;                         ///< kg, per body
    std::array<double, 3> box{1.0, 1.0, 0.1};  ///< m, collision/visual box extents
    std::array<double, 3> offset{0, 0, 0};     ///< m, body centre relative to node
    int collision_family = 7;
    double opacity = 0.35;
    std::array<double, 3> color{0.75, 0.78, 0.82};
    /// Contact material for the plank. Always applied; an absent `material`
    /// block in YAML leaves these defaults in place.
    StructureContactMaterialConfig material;
};

/// FEA visualisation (ChVisualShapeFEA) coloured by a beam field.
struct StructureVisualizationConfig {
    std::string data_type = "ELEM_BEAM_MZ";  ///< beam field to colour by
    std::string colormap = "JET";
    std::array<double, 2> range{-7.0e4, 7.0e4};  ///< colormap range (field units)
    int beam_resolution = 12;                    ///< colour bands along each member
    int beam_resolution_section = 8;             ///< facets around the section
};

/// The whole FEA structure scenario.
struct StructureConfig {
    bool automatic_gravity = true;
    std::vector<StructureMaterialConfig> materials;
    std::vector<StructureSectionConfig> sections;
    std::vector<StructureBeamConfig> beams;
    std::optional<StructureCrossBeamsConfig> cross_beams;
    std::vector<StructureMateConfig> mates;
    std::vector<StructureRigidAttachmentConfig> rigid_attachments;
    StructureVisualizationConfig visualization;
};

/// Parse a `<case>.structure.yaml` file.  Throws std::runtime_error on a
/// missing file or a malformed / absent top-level `structure:` block.
StructureConfig LoadStructureConfigFromYaml(const std::string& path);

}  // namespace seastack::app

#endif  // SEASTACK_APP_STRUCTURE_CONFIG_H
