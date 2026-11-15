// =============================================================================
// SEA-Stack viewer: force vectors, trajectories and live plots
// =============================================================================
// Drawn with ImGui (arrows and trails on the background draw list, plots with
// ImPlot) from the recorded frames, so they follow the timeline in playback.
// Visual only: nothing here is fed back into the simulation.
// =============================================================================
#pragma once

#include "vsg_gui_component.h"

#include <seastack/viz_core/frame_history.h>
#include <seastack/viz_core/scene_model.h>

#include <cstddef>

namespace seastack::viz {

class SeastackViewer;

class AnalysisOverlays {
  public:
    /// Force / moment arrows and centre-of-mass trails over the 3D view, with
    /// the arrow legend. Call inside the ImGui frame. `shown` is the frame
    /// drawn; `history` the recorded frames; `model_radius` [m] sets the
    /// arrow length. Updates the scales in `state`.
    void DrawSceneOverlays(const SceneDescription& scene, const FrameHistory& history, const VizFrame& shown,
                           const SeastackViewer& viewer, const ScreenRect& scene_rect, double model_radius,
                           float ui_scale, AnalysisState* state);

    /// Plot window (when state->show_plots): one quantity of one body over a
    /// time window ending at the frame shown.
    void DrawPlots(const SceneDescription& scene, const FrameHistory& history, const VizFrame& shown,
                   const ScreenRect& scene_rect, float ui_scale, AnalysisState* state);

  private:
    /// Update state->force_scale and moment_scale from the loads of the last
    /// 30 s of recorded frames (unless locked).
    void UpdateScales(const SceneDescription& scene, const FrameHistory& history, const VizFrame& shown,
                      double model_radius, AnalysisState* state);

    int plot_body_ = -1;      ///< Scene body index; < 0 = first moving body
    int plot_quantity_ = 0;   ///< See kQuantities in the .cpp
    int plot_source_ = -1;    ///< Force source for loads; -1 = total
    float plot_window_s_ = 60.0f;  ///< Time window [s]
};

}  // namespace seastack::viz
