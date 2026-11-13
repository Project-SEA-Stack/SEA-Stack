#pragma once

#include <memory>

#include <seastack/config.h>
#include <gui/guihelper.h>
#include <seastack/infra/logging.h>
#include <seastack/hydro/waves/wave_base.h>

#include <chrono/physics/ChSystem.h>

#include <chrono/core/ChCoordsys.h>
#include <chrono/core/ChQuaternion.h>
#include <chrono/core/ChVector3.h>

#include <chrono/assets/ChVisualSystem.h>

#ifdef SEASTACK_HAVE_VSG
    #include "vsg_visual_system.h"
    #include <seastack/adapters/chrono/viz_extractor.h>
    #include <seastack/viz_core/scene_framing.h>
    #include <seastack/viz_core/scene_model.h>
#endif

namespace chrono {
class ChSystem;
}

namespace seastack::viz {

// Base class for private GUI implementation.
class GUIImpl {
  public:
    GUIImpl() {}
    virtual ~GUIImpl() {}
    GUIImpl(const GUIImpl&)            = delete;
    GUIImpl& operator=(const GUIImpl&) = delete;

    virtual void SetViewerOptions(const ViewerOptions& /*options*/) {}
    virtual void Init(UI& ui, ::chrono::ChSystem*, const char* title) {
        seastack::infra::cli::LogWarning(
            "Warning: GUI deactivated. SEA-Stack was built without run-time visualization support.");
    }
    virtual void SetCamera(double /*x*/, double /*y*/, double /*z*/,
                           double /*dirx*/, double /*diry*/, double /*dirz*/) {}
    virtual bool IsRunning(double /*timestep*/) { return true; }
    virtual void SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> /*wave*/) {}
    virtual void SetWaterGridExtent(double /*width*/, double /*length*/,
                                     double /*center_x*/, double /*center_y*/) {}
    virtual void SetMooringLineProvider(MooringVizProvider /*provider*/) {}
    virtual void SetMooringVisualizationRadii(double /*line_radius_m*/,
                                              double /*endpoint_radius_m*/,
                                              double /*node_marker_radius_m*/) {}
};

#ifdef SEASTACK_HAVE_VSG
// Forward declarations (defined in vsg_water_surface.h, vsg_gui_component.h, vsg_mooring_lines.h).
class AnimatedWaterSurface;
class MooringLinesViz;
class OverlayBuilder;
struct ViewerSettings;

class GUIImplVSG : public GUIImpl {
  public:
    GUIImplVSG();
    ~GUIImplVSG();
    GUIImplVSG(const GUIImplVSG&)            = delete;
    GUIImplVSG& operator=(const GUIImplVSG&) = delete;

    virtual void SetViewerOptions(const ViewerOptions& options) override { options_ = options; }
    virtual void Init(UI& ui, ::chrono::ChSystem*, const char* title) override;
    virtual void SetCamera(double x, double y, double z, double dirx, double diry, double dirz) override;
    virtual bool IsRunning(double timestep) override;
    virtual void SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> wave) override;
    virtual void SetWaterGridExtent(double width, double length, double center_x, double center_y) override;
    virtual void SetMooringLineProvider(MooringVizProvider provider) override;
    virtual void SetMooringVisualizationRadii(double line_radius_m,
                                            double endpoint_radius_m,
                                            double node_marker_radius_m) override;

  private:
    /// Ensure water surface is created (animated if wave model set, static otherwise).
    void EnsureWaterSurface();

    /// Apply pending camera requests from the GUI (standard views, fit).
    void HandleCameraRequests();

    /// Update clip planes for the current camera distance and water extent.
    void UpdateClipPlanes();

    std::shared_ptr<SeastackVisualSystemVSG> pVis;
    std::unique_ptr<OverlayBuilder> overlay_builder_;  ///< Water nodes via Chrono's shape builder
    ViewerOptions options_;

    /// World-frame bounds of all body visual shapes at Init() [m]; drives
    /// the default camera, water extent, clip planes and mooring marker sizes.
    Bounds3 model_bounds_;
    std::shared_ptr<seastack::hydro::WaveBase> wave_model_;
    ::chrono::ChSystem* system_ = nullptr;
    /// Scene and per-frame state for the renderers; the only reader of system_
    /// after Init() apart from Chrono's own body drawing.
    std::unique_ptr<seastack::chrono::VizExtractor> extractor_;
    std::unique_ptr<AnimatedWaterSurface> animated_water_;
    std::unique_ptr<ViewerSettings> viewer_settings_;

    MooringVizProvider mooring_provider_;
    std::unique_ptr<MooringLinesViz> mooring_viz_;
    vsg::ref_ptr<vsg::Group> mooring_scene_group_;

    double mooring_viz_line_radius_request_ = -1.0;
    double mooring_viz_endpoint_radius_request_ = -1.0;
    double mooring_viz_node_marker_radius_request_ = -1.0;
};

class SeastackViewer;
class SeastackGuiComponent;

/// GUI backed by SEA-Stack's own viewer (--viewer=seastack). Reads Chrono
/// only through the VizExtractor.
class GUIImplSeastack : public GUIImpl {
  public:
    GUIImplSeastack();
    ~GUIImplSeastack();
    GUIImplSeastack(const GUIImplSeastack&)            = delete;
    GUIImplSeastack& operator=(const GUIImplSeastack&) = delete;

    void SetViewerOptions(const ViewerOptions& options) override { options_ = options; }
    void Init(UI& ui, ::chrono::ChSystem* system, const char* title) override;
    void SetCamera(double x, double y, double z, double dirx, double diry, double dirz) override;
    bool IsRunning(double timestep) override;
    void SetWaveModel(std::shared_ptr<seastack::hydro::WaveBase> wave) override { wave_model_ = std::move(wave); }
    void SetWaterGridExtent(double width, double length, double center_x, double center_y) override;
    void SetMooringLineProvider(MooringVizProvider provider) override;
    void SetMooringVisualizationRadii(double line_radius_m, double endpoint_radius_m,
                                      double node_marker_radius_m) override;

  private:
    void HandleCameraRequests();
    void UpdateClipPlanes();
    /// Render the scene only where the GUI panels leave the window free.
    void UpdateSceneViewport();
    /// Create the water surface on the first frame, then animate it from `frame`.
    void UpdateWater(const VizFrame& frame);
    /// Create the mooring lines on the first frame with line data, then update them.
    void UpdateMooring(const VizFrame& frame);
    /// Body names at their centres of mass (ImGui, inside the GUI frame).
    void DrawBodyLabels();

    ViewerOptions options_;
    VizFrame last_frame_;  ///< Frame shown this render; labels are placed from it
    std::shared_ptr<seastack::hydro::WaveBase> wave_model_;
    std::unique_ptr<AnimatedWaterSurface> water_;
    bool water_tried_ = false;  ///< Water creation attempted (it is not retried)
    /// The start-up framing used the full window; redo it once the panel layout is known.
    bool reframe_pending_ = false;
    Bounds3 model_bounds_;  ///< World bounds of all body visual shapes at Init() [m]
    std::unique_ptr<seastack::chrono::VizExtractor> extractor_;
    std::unique_ptr<SeastackViewer> viewer_;
    std::unique_ptr<ViewerSettings> viewer_settings_;
    std::unique_ptr<SeastackGuiComponent> gui_;
    MooringVizProvider mooring_provider_;
    std::unique_ptr<MooringLinesViz> mooring_viz_;
    /// Blender-style TSDA glyphs: thin SEGMENT cables + SPRING hydraulic rams.
    std::unique_ptr<MooringLinesViz> tsda_segment_viz_;
    std::unique_ptr<MooringLinesViz> tsda_barrel_viz_;
    std::unique_ptr<MooringLinesViz> tsda_rod_viz_;
    std::unique_ptr<MooringLinesViz> tsda_spring_viz_;
    /// Drawn before the water so opaque lines show through the translucent surface.
    vsg::ref_ptr<vsg::Group> mooring_group_;
    double mooring_line_radius_m_ = -1.0;         ///< YAML override [m]; < 0 = model-scaled default
    double mooring_endpoint_radius_m_ = -1.0;     ///< YAML override [m]; < 0 = model-scaled default
    double mooring_node_marker_radius_m_ = -1.0;  ///< YAML override [m]; < 0 = model-scaled default
};
#endif

}  // namespace seastack::viz
