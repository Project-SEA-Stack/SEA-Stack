// SEA-Stack VSG Water Surface Visualization
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <seastack/hydro/waves/wave_base.h>

#include <chrono/physics/ChBody.h>

#include <seastack/viz_core/radiated_wave_synthesis.h>
#include <seastack/viz_core/scene_framing.h>
#include <seastack/viz_core/scene_model.h>
#include <seastack/viz_core/water_colormap.h>

#include <vsg/all.h>

#include "vsg_overlay_builder.h"
#include "water_display_components.h"

namespace seastack::viz {

// Forward declaration (defined in vsg_gui_component.h).
struct ViewerSettings;

/// Deformable water surface mesh for visualizing ocean waves.
///
/// Two paths, chosen each frame:
///  - GPU: if the viewer supports it (OverlayBuilder::CreateGpuWaterGrid),
///    the wave model has discrete components and no radiated-wave field is
///    set. A flat grid is displaced in the vertex shader by the same
///    components GetElevation(pos, t, 500) sums; updated every frame.
///  - CPU: otherwise (e.g. tabulated elevation records, the radiated-wave
///    field, the Chrono viewer). Vertex Z from GetElevation plus the
///    radiated field at update_hz.
class AnimatedWaterSurface {
  public:
    /// Largest grid resolution on the CPU path; finer settings use this.
    static constexpr int kMaxCpuResolution = 128;

    AnimatedWaterSurface();
    ~AnimatedWaterSurface();

    // Non-copyable, non-movable (owns GPU resources).
    AnimatedWaterSurface(const AnimatedWaterSurface&) = delete;
    AnimatedWaterSurface& operator=(const AnimatedWaterSurface&) = delete;

    /// Reset state for reuse with a new viewer.
    void Reset();

    /// Initialize mesh and add it to the builder's overlay scene (call once
    /// the viewer is initialized).
    void Initialize(OverlayBuilder* builder, int resolution = 0, const ViewerSettings* settings = nullptr);

    /// Reinitialize with new grid resolution or extent.
    void Reinitialize(int resolution, const ViewerSettings* settings = nullptr);

    /// Update vertex Z from wave elevation at time t.
    void Update(const std::shared_ptr<seastack::hydro::WaveBase>& wave, double t,
                const ViewerSettings* settings = nullptr);

    void SetVisible(bool visible);
    bool IsVisible() const;
    bool IsInitialized() const;
    bool IsInitializedFor(const OverlayBuilder* builder) const;
    int GetGridResolution() const { return current_resolution_; }
    /// True if the surface is displaced on the GPU now.
    bool IsGpu() const { return gpu_; }
    /// Elevation-style legend now (range and contour interval [m]), and
    /// whether the viewer draws that style at all.
    const ElevationLegend& GetElevationLegend() const { return legend_; }
    bool HasElevationStyle() const { return has_elevation_style_; }
    /// Vertices along X and Y (about GetGridResolution()^2 in total).
    WaterGridCounts GetGridCounts() const { return counts_; }
    /// Grid cell size [m] (cells are near-square; the larger side is returned).
    double GetCellSize() const;
    std::string GetStatusString() const;

    /// Radiated-wave field added to the elevation (not owned; null = none).
    /// Set before each Update(); scaled by settings->radiation_visual_scale.
    void SetRadiationField(const RadiatedWaveSynthesizer* field) { radiation_field_ = field; }
    /// True if Update(t) would recompute the surface (always on the GPU path):
    /// compute the radiated field only then.
    bool NeedsUpdate(double t, const ViewerSettings* settings) const;

    void SetWireframeVisible(bool visible);
    bool IsWireframeVisible() const { return wireframe_visible_; }

  private:
    void InitializeInternal(OverlayBuilder* builder, int resolution, const ViewerSettings* settings = nullptr);
    void InitializeWireframe();
    void UpdateWireframe();
    /// Flat grid_ at z = 0 for `resolution` and the current extent; sets counts_.
    void BuildFlatGrid(int resolution);
    void UpdateGpu(const std::shared_ptr<seastack::hydro::WaveBase>& wave, double t, const ViewerSettings* settings);
    /// Legend from the full sea (without the start-up ramp) and the style to draw.
    void UpdateStyle(const std::shared_ptr<seastack::hydro::WaveBase>& wave, double t, const ViewerSettings* settings);

    TriangleMesh grid_;  ///< Flat grid at z = 0: vertex positions [m] and faces
    vsg::ref_ptr<vsg::vec3Array> vsg_vertices_;
    vsg::ref_ptr<vsg::vec3Array> vsg_normals_;
    vsg::ref_ptr<vsg::vec4Array> vsg_colors_;  ///< Per-vertex colors for height shading
    std::vector<vsg::vec3> grid_normals_;      ///< Scratch: accumulated normal per grid vertex
    vsg::ref_ptr<vsg::Node> vsg_node_;
    vsg::ref_ptr<vsg::Group> scene_;
    OverlayBuilder* builder_ = nullptr;
    size_t num_triangles_ = 0;
    int current_resolution_ = 0;    ///< Resolution of the grid drawn (CPU path: at most kMaxCpuResolution)
    int requested_resolution_ = 0;  ///< Resolution asked for; used again when the path changes
    WaterGridCounts counts_;  ///< Vertices along X and Y for current_resolution_ and the extent
    bool prefer_gpu_ = true;  ///< Try the GPU path at the next (re)build
    bool gpu_ = false;        ///< The grid drawn is the GPU one
    bool gpu_capacity_logged_ = false;
    std::vector<DisplayWaveComponent> display_components_;  ///< Scratch for the GPU path
    std::vector<DisplayWaveComponent> legend_components_;   ///< Scratch for the legend
    ElevationLegend legend_;
    bool has_elevation_style_ = false;
    bool initialized_ = false;
    bool visible_ = true;
    mutable int frame_count_ = 0;
    double last_update_time_ = -1.0;  ///< Simulated time of the last surface update [s]
    std::chrono::steady_clock::time_point last_update_wall_{};

    const RadiatedWaveSynthesizer* radiation_field_ = nullptr;

    vsg::ref_ptr<vsg::vec3Array> wireframe_vertices_;
    vsg::ref_ptr<vsg::Node> wireframe_node_;
    bool wireframe_visible_ = false;
    bool wireframe_initialized_ = false;

    // Adaptive height-shading range (smoothed min/max eta values).
    float adaptive_eta_min_ = 0.0f;
    float adaptive_eta_max_ = 0.0f;
    bool adaptive_range_initialized_ = false;

    // Custom grid extent. Values <= 0 use defaults from vsg_config.h.
    double grid_width_ = 0.0;   ///< Grid extent in X direction [m]
    double grid_length_ = 0.0;  ///< Grid extent in Y direction [m]
    double grid_center_x_ = 0.0;  ///< Grid center X coordinate [m]
    double grid_center_y_ = 0.0;  ///< Grid center Y coordinate [m]
};

/// Creates a static water plane (visual only, no collision).
std::shared_ptr<::chrono::ChBody> CreateStaticWaterPlane();

// ---- Shared by both viewers' GUI implementations ----

/// Size the water grid from the model bounds and wavelength when no extent
/// was set (grid_width and grid_length <= 0). Returns true if it changed.
bool AutoSizeWaterGrid(ViewerSettings* settings, const Bounds3& model_bounds,
                       const seastack::hydro::WaveBase* wave);

/// Set settings->grid_resolution from the detail level, the patch extent and
/// the deep-water wavelength of the wave's characteristic period; flags a
/// rebuild (resolution_changed) when it moves enough to matter. Call before
/// Initialize and before ApplyWaterSettings each frame.
void ChooseWaterResolution(const seastack::hydro::WaveBase* wave, ViewerSettings* settings);

/// Apply panel settings: visibility, and a rebuild after a resolution or
/// extent change.
void ApplyWaterSettings(AnimatedWaterSurface* water, ViewerSettings* settings);

/// Panel readouts: grid cell size, and the deep-water wavelength of the
/// wave's characteristic period now (0 for no waves).
void UpdateWaterReadouts(const AnimatedWaterSurface& water, const seastack::hydro::WaveBase* wave,
                         ViewerSettings* settings);
}  // namespace seastack::viz

