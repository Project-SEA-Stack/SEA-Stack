// =============================================================================
// SEA-Stack VSG Water Shader
// =============================================================================
// Shading for the water surface in the SEA-Stack viewer: Fresnel reflection
// of a sky gradient, GGX sun glint, a deep / scatter colour palette by crest
// height, view-angle opacity and soft patch edges.
//
// Two ways to draw the surface:
//  - CreateNode: the shape (vertex positions and normals) comes from the wave
//    model on the CPU; only colour and opacity are done here.
//  - CreateGpuNode: a flat grid displaced in the vertex shader by the linear
//    wave components set with SetWaves (see water_display_components.h), so
//    fine grids cost no CPU time. Normals are analytic.
// =============================================================================
#pragma once

#include <vector>

#include <vsg/all.h>

#include "water_display_components.h"

namespace seastack::viz {

class WaterShader {
  public:
    /// Most components the GPU grid sums; SetWaves drops the rest.
    static constexpr int kMaxGpuComponents = 1024;

    WaterShader();

    /// State group drawing an indexed triangle mesh with the water shader.
    /// Arrays as the overlay meshes: [0] positions, [1] normals (vec3, world
    /// frame), [2] texture coordinates (unused), [3] colours (vec4: water tint,
    /// alpha = opacity seen straight down). `patch_min` / `patch_max` [m] are
    /// the patch corners in x, y; the surface fades out near its edges.
    vsg::ref_ptr<vsg::Node> CreateNode(vsg::ref_ptr<vsg::vec3Array> vertices, vsg::ref_ptr<vsg::vec3Array> normals,
                                       vsg::ref_ptr<vsg::vec2Array> texcoords, vsg::ref_ptr<vsg::vec4Array> colors,
                                       vsg::ref_ptr<vsg::uintArray> indices, const vsg::vec2& patch_min,
                                       const vsg::vec2& patch_max);

    /// State group drawing a flat grid displaced on the GPU. `vertices` are
    /// the grid points at z = 0 [m, world]; `grid_coords` their column and
    /// row (for grid lines). Patch corners as CreateNode.
    vsg::ref_ptr<vsg::Node> CreateGpuNode(vsg::ref_ptr<vsg::vec3Array> vertices,
                                          vsg::ref_ptr<vsg::vec2Array> grid_coords,
                                          vsg::ref_ptr<vsg::uintArray> indices, const vsg::vec2& patch_min,
                                          const vsg::vec2& patch_max);

    /// Components the GPU grid sums now. Returns the number uploaded (at
    /// most kMaxGpuComponents).
    int SetWaves(const std::vector<DisplayWaveComponent>& components);

    /// Elevation [m] at which crests take the full scatter colour (both
    /// paths; e.g. DisplayElevationScale).
    void SetElevationScale(float elevation_scale);

    /// GPU grid lines every `cells_per_line` cells; 0 hides them.
    void SetGridLines(float cells_per_line);

    /// Camera position [m, world]; set every frame before recording.
    void SetEye(const vsg::dvec3& eye);

    /// Key light: unit direction from the surface towards the light (world)
    /// and its linear colour times intensity.
    void SetSun(const vsg::vec3& towards_sun, const vsg::vec3& color);

    /// Sky the water reflects (linear colours): zenith and horizon.
    void SetSky(const vsg::vec3& zenith, const vsg::vec3& horizon);

    /// Linear water body colours: deep (troughs, looking straight down) and
    /// scatter (crests, where light passes through thin water).
    void SetWaterColors(const vsg::vec3& deep, const vsg::vec3& scatter);

  private:
    void SetPatch(const vsg::vec2& patch_min, const vsg::vec2& patch_max);

    /// std140 block of vec4s; see the shader for the layout.
    vsg::ref_ptr<vsg::vec4Array> params_;
    /// GPU grid components, one vec4 (a, kx, ky, psi) each.
    vsg::ref_ptr<vsg::vec4Array> components_;
};

}  // namespace seastack::viz
