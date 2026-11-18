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
#include <string>
#include <vector>

namespace seastack::viz {

class SeastackViewer;

/// Arrow colour of force source `source` (index into force_sources), or the
/// text colour for AnalysisState::kTotal. Shared by the arrows, the legend
/// and the panels so a component has one colour everywhere.
ImU32 ForceSourceColor(int source);

/// "12.3 kN"-style label of `value` in `unit`, with an SI prefix.
std::string FormatSi(double value, const char* unit);

class AnalysisOverlays {
  public:
    /// Force / moment arrows and centre-of-mass trails over the 3D view, with
    /// the arrow legend. Call inside the ImGui frame. `shown` is the frame
    /// drawn; `history` the recorded frames; `model_radius` [m] sets the
    /// arrow length. Which bodies get arrows and trails comes from `model`
    /// (hidden bodies get neither). The legend toggles components when
    /// clicked; hovering a legend entry, a panel entry or an arrow
    /// highlights that component. Updates the scales in `state`.
    void DrawSceneOverlays(const SceneDescription& scene, const FrameHistory& history, const VizFrame& shown,
                           const SeastackViewer& viewer, const ScreenRect& scene_rect, double model_radius,
                           float ui_scale, const ModelState& model, AnalysisState* state);

    /// Plot window (when state->show_plots): one quantity of one body over a
    /// time window ending at the frame shown. The plotted body follows
    /// `selected_body` (scene index, -1 = none) when the selection changes.
    void DrawPlots(const SceneDescription& scene, const FrameHistory& history, const VizFrame& shown,
                   const ScreenRect& scene_rect, float ui_scale, int selected_body, AnalysisState* state);

  private:
    /// Update state->force_scale and moment_scale from the loads on bodies
    /// with arrows over the last 30 s of recorded frames (unless locked).
    void UpdateScales(const SceneDescription& scene, const FrameHistory& history, const VizFrame& shown,
                      double model_radius, const std::vector<char>& with_arrows, AnalysisState* state);

    /// Interactive legend at the bottom-left of the scene.
    void DrawLegend(const SceneDescription& scene, const ScreenRect& scene_rect, float ui_scale,
                    AnalysisState* state);

    // Hover from the previous frame (the legend and arrows are drawn after
    // the arrows that use it); AnalysisState::kNone when nothing is hovered.
    int legend_hovered_ = AnalysisState::kNone;
    int arrow_hovered_ = AnalysisState::kNone;

    int last_selected_ = -1;  ///< Selection seen by the last DrawPlots

    int plot_body_ = -1;      ///< Scene body index; < 0 = first moving body
    int plot_quantity_ = 0;   ///< See kQuantities in the .cpp
    int plot_source_ = -1;    ///< Force source for loads; -1 = total
    float plot_window_s_ = 60.0f;  ///< Time window [s]
};

}  // namespace seastack::viz
