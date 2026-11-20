/// @file vsg_beam_lines.h
/// @brief VSG renderer for Chrono::FEA beams (SceneDescription::beam_lines).
#pragma once

#include "vsg_overlay_builder.h"

#include <seastack/viz_core/color_range.h>
#include <seastack/viz_core/scene_model.h>

#include <vsg/all.h>

#include <vector>

namespace seastack::viz {

/// Draws each FEA beam line as its section outline (rectangle or circle)
/// swept through the section samples of the frame, optionally coloured by
/// the magnitude of one section load. Built once, then the vertex, normal
/// and colour buffers are rewritten in place every frame.
class BeamLinesViz {
  public:
    /// Build the meshes from the scene and the first frame with beam data and
    /// add them to `parent_group`. No effect if `frame` has no beams.
    void Initialize(OverlayBuilder* builder, vsg::ref_ptr<vsg::Group> parent_group, const SceneDescription& scene,
                    const VizFrame& frame);

    bool IsInitializedFor(const OverlayBuilder* builder) const { return initialized_ && builder_ == builder; }

    /// Pose the beams from `frame` and colour them by |field| through
    /// `range` (fed the largest |field| of the frame); kNone = plain steel.
    void Update(const SceneDescription& scene, const VizFrame& frame, BeamField field, ColorRange* range);

  private:
    /// One straight edge of the section outline, in the section frame (y, z)
    /// [m], with the outward normal at each end.
    struct ProfileEdge {
        double y0, z0, ny0, nz0;
        double y1, z1, ny1, nz1;
    };

    struct BeamMesh {
        std::vector<ProfileEdge> profile;
        size_t num_rings = 0;
        vsg::ref_ptr<vsg::vec3Array> vertices;
        vsg::ref_ptr<vsg::vec3Array> normals;
        vsg::ref_ptr<vsg::vec4Array> colors;
    };

    static std::vector<ProfileEdge> MakeProfile(const BeamLineDescription& desc);
    /// Rewrite the buffers of `mesh` from `sections`; `colors` has one entry per section.
    static void WriteMesh(const std::vector<BeamSectionState>& sections, const std::vector<vsg::vec4>& colors,
                          BeamMesh* mesh);

    std::vector<BeamMesh> meshes_;
    OverlayBuilder* builder_ = nullptr;
    bool initialized_ = false;
};

/// Readable name and units of a beam field, e.g. "Mz" and "N m".
const char* BeamFieldName(BeamField field);
const char* BeamFieldUnits(BeamField field);

}  // namespace seastack::viz
