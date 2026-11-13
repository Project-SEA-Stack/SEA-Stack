// =============================================================================
// SEA-Stack VSG Overlay Builder
// =============================================================================
// Creates the VSG nodes for SEA-Stack's dynamic overlays (water surface and
// its wireframe) and owns the scene group they attach to. Implemented by the
// Chrono viewer (Chrono's shape builder) and the SEA-Stack viewer (plain VSG),
// so each overlay is written once for both.
// =============================================================================
#pragma once

#include <seastack/viz_core/scene_model.h>

#include <vsg/all.h>

namespace seastack::viz {

/// PBR appearance of an overlay mesh.
struct OverlayMaterial {
    Rgba color{1.0f, 1.0f, 1.0f, 1.0f};  ///< Linear colour; alpha = opacity [0, 1]
    float roughness = 0.5f;               ///< [0, 1]
    float metallic = 0.0f;                ///< [0, 1]
};

class OverlayBuilder {
  public:
    virtual ~OverlayBuilder() = default;

    /// Group that overlay nodes are added to and removed from.
    virtual vsg::ref_ptr<vsg::Group> OverlayScene() = 0;

    /// Compiled PBR mesh node, ready to add to OverlayScene(); null on failure.
    /// Vertices are unshared and in face order: corner k of face f is vertex
    /// 3 f + k. Vertex arrays: [0] positions and [1] normals (vec3),
    /// [2] texture coordinates, [3] colours (vec4). Overlays update the arrays
    /// in place and call dirty().
    virtual vsg::ref_ptr<vsg::Node> CreateOverlayMesh(const TriangleMesh& mesh, const OverlayMaterial& material) = 0;
};

}  // namespace seastack::viz
