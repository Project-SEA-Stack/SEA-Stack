// =============================================================================
// SEA-Stack VSG GUI Component
// =============================================================================
// ImGui overlay component for the VSG visualization backend.
// Provides the play/pause button and other SEA-Stack-specific UI elements.
// =============================================================================
#pragma once

#include <chrono_vsg/ChGuiComponentVSG.h>
#include <chrono_vsg/ChVisualSystemVSG.h>

#include <seastack/viz_core/scene_framing.h>
#include <seastack/viz_core/simulation_pacer.h>
#include <seastack/viz_core/visual_theme.h>

namespace seastack::viz {

class MooringLinesViz;  // forward declaration

/// Runtime-adjustable water visualization settings.
///
/// Allows tuning water appearance without recompiling. Owned by GUIImplVSG
/// and passed by pointer to GUI components and the water surface renderer.
struct ViewerSettings {
    // --- Display Options ---
    bool show_water = true;           ///< Toggle water surface visibility
    bool show_water_grid = false;     ///< Show wireframe overlay on water surface
    bool show_water_status = false;   ///< Show status line in overlay

    // --- Wave Rendering ---
    float wave_visual_scale = 1.0f;   ///< Multiplier for wave elevation display
    int update_hz = 30;               ///< Maximum update rate [Hz], range 10-60

    // --- Grid Resolution ---
    int grid_resolution = 64;         ///< Vertices per side (32, 64, 96, or 128)
    bool resolution_changed = false;  ///< Triggers mesh rebuild when resolution changes

    static constexpr int kResolutionOptions[] = {32, 64, 96, 128};
    static constexpr int kResolutionCount = 4;

    // --- Grid Extent ---
    // Values <= 0 use defaults (100m x 100m centered at origin).
    double grid_width = 0.0;          ///< Grid extent in X direction [m]
    double grid_length = 0.0;         ///< Grid extent in Y direction [m]
    double grid_center_x = 0.0;       ///< Grid center X coordinate [m]
    double grid_center_y = 0.0;       ///< Grid center Y coordinate [m]
    bool grid_extent_changed = false; ///< Triggers mesh rebuild when extent changes

    // --- Radiation Visualization (Approximate) ---
    // Visual-only representation of radiated waves from moving bodies.
    // NOT physically accurate; wavelength derived from dispersion relation,
    // amplitude scales with body velocity and size.
    bool show_radiation_viz = false;       ///< Enable radiated wave visualization
    float radiation_visual_scale = 1.0f;   ///< Visual amplification [0.1x - 5x]

    // --- Mooring Colour Mapping ---
    bool show_mooring_colors = true;       ///< Colour lines by tension magnitude

    // --- Theme ---
    VisualTheme theme = VisualTheme::kDark;
    bool theme_changed = true;  ///< Scene theme needs (re)applying by GUIImplVSG

    // --- Camera ---
    // Requests set by the GUI and consumed by GUIImplVSG on the next frame.
    bool camera_view_requested = false;    ///< Move to camera_view, framing the model
    CameraView camera_view = CameraView::kIsometric;
    bool camera_fit_requested = false;     ///< Re-frame the model, keeping the view direction
};

/// ImGui component for SEA-Stack visualization overlay.
/// Displays the simulation toolbar (play/pause, playback speed, run status)
/// and the viewer settings panel.
class SeastackGuiComponent : public ::chrono::vsg3d::ChGuiComponentVSG {
  public:
    /// Construct the GUI component.
    /// @param vsys Pointer to the VSG visual system (for frame-rate display).
    /// @param button_pressed Reference to the simulation running state (toggled by button).
    /// @param playback_speed Requested playback speed [-]; <= 0 means unlimited.
    /// @param run_status Run progress written by the simulation loop.
    /// @param settings Pointer to viewer settings (owned by GUIImplVSG).
    /// @param mooring_viz Pointer to the mooring renderer (for reading the
    ///                    adaptive scalar range for the colour bar).
    SeastackGuiComponent(::chrono::vsg3d::ChVisualSystemVSG* vsys, bool& button_pressed,
                         double& playback_speed, const RunStatus& run_status,
                         ViewerSettings* settings = nullptr,
                         MooringLinesViz* mooring_viz = nullptr);

    /// Render the ImGui overlay.
    /// Called each frame by the VSG rendering loop.
    void render(vsg::CommandBuffer& cb) override;

  private:
    void RenderToolbar();
    void RenderMooringPanel();
    void RenderColorBar();

    ::chrono::vsg3d::ChVisualSystemVSG* vsys_;
    bool& pressed_;
    double& playback_speed_;
    double last_finite_speed_ = 1.0;  ///< Speed restored when "Max" is switched off [-]
    bool imgui_theme_applied_ = false;
    bool fixed_range_error_ = false;  ///< Last fixed mooring range entry was rejected
    VisualTheme imgui_theme_ = VisualTheme::kDark;  ///< Theme of the current ImGui style
    const RunStatus& run_status_;
    ViewerSettings* settings_;
    MooringLinesViz* mooring_viz_;
};

}  // namespace seastack::viz

