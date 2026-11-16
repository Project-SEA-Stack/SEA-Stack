// =============================================================================
// SEA-Stack VSG Overlay Builder
// =============================================================================
// Creates the VSG nodes for SEA-Stack's dynamic overlays (water surface, its
// wireframe, mooring lines) and owns the scene group they attach to. Implemented by the
// Chrono viewer (Chrono's shape builder) and the SEA-Stack viewer (plain VSG),
// so each overlay is written once for both.
// =============================================================================
#pragma once

#include <seastack/viz_core/scene_model.h>
#include <seastack/viz_core/water_colormap.h>

#include <vector>

#include <vsg/all.h>

namespace seastack::viz {

struct DisplayWaveComponent;  // water_display_components.h

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

    /// Water surface node: same vertex layout as CreateOverlayMesh, with the
    /// vertex colours as the water tint (alpha = opacity). A viewer with its
    /// own water shader adds sky reflection and soft edges; by default it is
    /// a plain PBR overlay mesh.
    virtual vsg::ref_ptr<vsg::Node> CreateWaterMesh(const TriangleMesh& mesh, const OverlayMaterial& material) {
        return CreateOverlayMesh(mesh, material);
    }

    /// Water surface displaced on the GPU from linear wave components, or
    /// null if this viewer cannot (the caller then uses CreateWaterMesh).
    /// `flat_grid` is a grid at z = 0 with shared vertices: vertex j nx + i is
    /// column i, row j. The vertices never change; the shape comes from
    /// SetGpuWaterWaves.
    virtual vsg::ref_ptr<vsg::Node> CreateGpuWaterGrid(const TriangleMesh& /*flat_grid*/, int /*nx*/) { return {}; }

    /// Components the GPU water sums now [world frame, SI]. Returns the number
    /// used (the shader has a fixed capacity).
    virtual int SetGpuWaterWaves(const std::vector<DisplayWaveComponent>& /*components*/) { return 0; }

    /// Elevation [m] at which a viewer with its own water shader draws crests
    /// in the full crest colour (either path); ignored otherwise.
    virtual void SetWaterElevationScale(float /*elevation_scale*/) {}

    /// Draw the water in the elevation style (colour by eta over -range_m ..
    /// +range_m in `palette`, contours every contour_m [m], 0 = none) or the shaded one. Returns
    /// false if this viewer has no such style.
    virtual bool SetWaterElevationStyle(bool /*on*/, float /*range_m*/, float /*contour_m*/, WaterPalette /*palette*/) {
        return false;
    }

    /// GPU water grid lines every `cells_per_line` cells; 0 hides them.
    virtual void SetGpuWaterGridLines(float /*cells_per_line*/) {}

    /// Compiled mesh node with shared vertices: vertex i is mesh vertex i.
    /// Same array layout as CreateOverlayMesh; colours start at `color`
    /// (linear, opaque) and carry the per-vertex colour.
    virtual vsg::ref_ptr<vsg::Node> CreateIndexedOverlayMesh(const TriangleMesh& mesh, const Rgba& color) = 0;

    /// Compiled unit-radius sphere [m] drawn under `transform`; move or scale
    /// it by setting transform->matrix. Returns the node to add to the scene.
    virtual vsg::ref_ptr<vsg::Node> CreateSphereOverlay(const OverlayMaterial& material,
                                                        vsg::ref_ptr<vsg::MatrixTransform> transform) = 0;
};

}  // namespace seastack::viz
