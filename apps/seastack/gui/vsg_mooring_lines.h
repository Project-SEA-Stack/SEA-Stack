/// @file vsg_mooring_lines.h
/// @brief VSG mooring-line tube-mesh renderer for SEA-Stack.
#pragma once

#include "vsg_overlay_builder.h"

#include <seastack/core/mooring_viz_data.h>
#include <seastack/viz_core/color_range.h>
#include <seastack/viz_core/scene_framing.h>
#include <seastack/viz_core/scene_model.h>

#include <vsg/all.h>

#include <vector>

namespace seastack::viz {

/// Renders MoorDyn mooring lines as cylindrical tube meshes in the VSG scene.
///
/// Each line is tessellated once during Initialize() and then cheaply
/// repositioned every frame via direct writes into the VSG vertex buffer,
/// followed by a `dirty()` call that triggers GPU re-upload.
class MooringLinesViz {
  public:
    MooringLinesViz();
    ~MooringLinesViz();

    MooringLinesViz(const MooringLinesViz&) = delete;
    MooringLinesViz& operator=(const MooringLinesViz&) = delete;

    /// Build tube meshes from initial line data and add to the VSG scene.
    /// Call once the viewer is initialized.
    ///
    /// @param builder        Overlay node builder of the active viewer (must
    ///                       outlive this object).
    /// @param initial_lines  One entry per mooring line with initial node
    ///                       positions and endpoint types.
    /// @param parent_group   Optional scene-graph group.  When provided,
    ///                       mooring geometry is added here instead of the
    ///                       builder's overlay scene.  Use this to guarantee
    ///                       render order (e.g. render mooring lines before the
    ///                       transparent water surface).
    void Initialize(OverlayBuilder* builder,
                    const std::vector<MooringLineVizData>& initial_lines,
                    vsg::ref_ptr<vsg::Group> parent_group = {});

    /// Reposition tube vertices and (optionally) recolour by tension.
    /// @param color_enabled  When false, colours are reset to the default cable
    ///                       tan and the colour range is left untouched.
    void Update(const std::vector<MooringLineVizData>& lines, bool color_enabled = true);

    bool IsInitialized() const { return initialized_; }
    bool IsInitializedFor(const OverlayBuilder* builder) const {
        return initialized_ && builder_ == builder;
    }

    /// Current tension colour range [N] (for the GUI colour bar).
    float TensionMin() const { return tension_range_.Min(); }
    float TensionMax() const { return tension_range_.Max(); }

    /// Tension colour range [N]: mode (auto/locked/fixed) and fixed limits.
    ColorRange& TensionRange() { return tension_range_; }

    /// Evaluate the Turbo colour map at normalised parameter @p t in [0, 1].
    /// Exposed as a public static so the GUI colour bar can reuse the same
    /// polynomial without duplicating coefficients.
    static vsg::vec4 TurboColormap(float t);

    /// Override cable tube / marker radii [m] before Initialize(). Any argument
    /// that is negative or non-finite is ignored for that component (library
    /// default kept). Values may be set from hydrodynamics YAML (see
    /// `hydrodynamics.moordyn.visualization_{line,endpoint,node_marker}_radius`).
    void SetVisualizationRadii(double line_radius_m,
                               double endpoint_radius_m,
                               double node_marker_radius_m);

  private:
    static constexpr int kSides = 16;  ///< Tube cross-section facets
    static constexpr double kDefaultTubeRadius = 0.15;
    static constexpr double kDefaultEndpointRadius = 0.5;
    static constexpr double kDefaultNodeMarkerRadius = 0.20;

    double tube_radius_ = kDefaultTubeRadius;
    double endpoint_radius_ = kDefaultEndpointRadius;
    double node_marker_radius_ = kDefaultNodeMarkerRadius;

    /// A PBR sphere used to mark an endpoint or intermediate node.
    struct PointMarker {
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        vsg::ref_ptr<vsg::Node> node;
    };

    /// Per-line rendering state: VSG node, extracted vertex/normal/color
    /// buffers (for dynamic updates), and small sphere markers.
    struct LineMesh {
        vsg::ref_ptr<vsg::vec3Array> vertices;
        vsg::ref_ptr<vsg::vec3Array> normals;
        vsg::ref_ptr<vsg::vec4Array> colors;
        vsg::ref_ptr<vsg::Node> node;
        size_t num_nodes = 0;
        PointMarker start_marker;
        PointMarker end_marker;
        std::vector<PointMarker> node_markers;
    };

    void BuildTubeMesh(const MooringLineVizData& line_data, LineMesh& lm);
    void UpdateTubeMesh(const MooringLineVizData& line_data, LineMesh& lm,
                        bool color_enabled, float range_min, float range_max);

    PointMarker CreateSphereMarker(double x, double y, double z,
                                   double radius, int point_type);

    static void UpdateMarkerPosition(PointMarker& marker,
                                     double x, double y, double z,
                                     double radius);

    std::vector<LineMesh> line_meshes_;
    vsg::ref_ptr<vsg::Group> scene_;
    OverlayBuilder* builder_ = nullptr;
    bool initialized_ = false;

    ColorRange tension_range_;
};

// ---- Shared by both viewers' GUI implementations ----

/// Start-up mooring display options (from YAML).
struct MooringVizOptions {
    /// Radii [m] for line / endpoint / node-marker geometry; < 0 keeps the
    /// model-scaled default.
    double line_radius_m = -1.0;
    double endpoint_radius_m = -1.0;
    double node_marker_radius_m = -1.0;
    /// Fixed tension colour range [N]; otherwise the range auto-tracks.
    bool has_tension_range = false;
    float tension_min = 0.0f;
    float tension_max = 0.0f;
};

/// Mooring lines of a frame in the form MooringLinesViz takes.
std::vector<MooringLineVizData> ToMooringVizData(const SceneDescription& scene, const VizFrame& frame);

/// Create the mooring renderer on the first frame with line data (radii
/// scaled to the model, then YAML overrides, then any fixed colour range)
/// and update it from `frame`. No effect for frames without mooring lines.
void DrawMooringLines(MooringLinesViz* viz, OverlayBuilder* builder, vsg::ref_ptr<vsg::Group> parent_group,
                      const SceneDescription& scene, const VizFrame& frame, const Bounds3& model_bounds,
                      const MooringVizOptions& options, bool color_enabled);

/// Renderers for Blender-style TSDA glyphs (separate radii: barrel ≠ rod ≠ coil).
struct TsdaGlyphRenderers {
    MooringLinesViz* segments = nullptr;  ///< Thin hoist / winch cables (SEGMENT)
    MooringLinesViz* barrels = nullptr;   ///< Hydraulic ram barrels (SPRING)
    MooringLinesViz* rods = nullptr;      ///< Piston rods (SPRING)
    MooringLinesViz* springs = nullptr;   ///< Return-spring coils on the exposed rod
};

/// SEA-Stack viewer TSDA glyphs: SEGMENT = thin cable; SPRING = hydraulic ram
/// (barrel + rod + spring coil). Solid colour, no tension mapping. Chrono
/// viewer should skip this and use its native point-point draw instead.
void DrawTsdaCables(const TsdaGlyphRenderers& glyphs, OverlayBuilder* builder,
                    vsg::ref_ptr<vsg::Group> parent_group, const SceneDescription& scene,
                    const VizFrame& frame);

}  // namespace seastack::viz
