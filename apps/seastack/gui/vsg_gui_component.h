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

#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace seastack::viz {

class MooringLinesViz;  // forward declaration

/// Axis-aligned screen rectangle [px], origin at the window's top-left.
struct ScreenRect {
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
};

/// Timeline of recorded frames (SEA-Stack viewer only). The viewer records
/// frames and fills the range; the transport bar edits the playback fields.
/// Playback is visual only: it shows stored frames and never changes the
/// simulation, which stays paused while a recorded frame is shown.
struct TimelineState {
    // --- Written by the viewer every frame ---
    bool available = false;    ///< The viewer records frames (show the timeline row)
    bool replay = false;       ///< Offline replay from file: there is no live simulation
    double start_time = 0.0;   ///< Oldest recorded frame [s]
    double end_time = 0.0;     ///< Newest recorded frame [s] (the live state when not replaying)
    size_t frame_count = 0;    ///< Recorded frames
    double memory_mb = 0.0;    ///< Approximate memory held by the recording [MB]
    bool interpolated = false; ///< The frame shown is blended between recorded frames
    bool smooth_slow_motion_active = false;  ///< Live view is interpolated between steps
    /// Replay only: the input pose (before the first step) is shown, not a result.
    bool showing_input_pose = false;

    // --- Written by the GUI (and by the viewer when playback reaches the end) ---
    bool playback = false;        ///< Show recorded frames instead of the live state
    double playback_time = 0.0;   ///< Time shown in playback [s]
    bool playing = false;         ///< Playback advances at the playback speed
    int step_request = 0;         ///< -1 / +1 = show the previous / next recorded frame
    bool resume_on_live = false;  ///< Restart stepping when returning to live
    bool smooth_slow_motion = true;  ///< Interpolate the live view when steps are slower than frames
};

/// One TSDA/RSDA parameter as shown in the Parameters panel.
struct ParameterRow {
    std::string id;        ///< Runtime parameter id, e.g. "tsda.PTO.damping_coefficient"
    std::string element;   ///< "tsda", "rsda" or "waves"
    std::string link_name; ///< Link name in the model, e.g. "PTO"; empty for waves
    std::string field;     ///< e.g. "damping_coefficient", "rest_length", "rest_angle", "height"
    std::string link;      ///< Group heading, e.g. "TSDA PTO" or "Waves"
    std::string label;     ///< e.g. "Damping"
    std::string units;     ///< SI units, e.g. "N*s/m"
    std::string source;    ///< "model YAML" or "default"
    std::string owner;     ///< Empty: editable. Otherwise who sets it (field disabled).
    double value = 0.0;          ///< In effect now
    double default_value = 0.0;  ///< At the start of the run
    double min_value = 0.0;      ///< Lowest accepted value
    bool pending = false;        ///< An edit waits for the next step
    double pending_value = 0.0;
    double effective_time = 0.0;  ///< Sim time [s] the last change is fully in effect (wave ramps)
};

/// Parameters panel: live TSDA/RSDA parameters and the changes applied so far
/// (SEA-Stack viewer, live runs). Edits change the simulation from the next
/// step on; every change is logged and can be exported as a schedule.
struct ParametersPanelState {
    // --- Written by the viewer every frame ---
    bool available = false;
    std::vector<ParameterRow> rows;
    std::vector<std::string> event_lines;  ///< Applied changes, oldest first
    std::string schedule_path;             ///< Export target; empty = no output directory
    bool run_finished = false;             ///< Edits can no longer take effect
    double sim_time = 0.0;                 ///< Live simulation time [s]

    // --- Written by the GUI, consumed by the viewer ---
    std::vector<std::pair<std::string, double>> requests;  ///< (id, value) for the next step
    bool export_requested = false;
    std::string status;  ///< Last rejected edit or export result; empty = none
};

/// Force vectors, trajectories and live plots (SEA-Stack viewer). Visual
/// only: everything is read from the recorded frames, so it follows the
/// timeline in playback.
struct AnalysisState {
    // --- Written by the viewer every frame ---
    std::vector<std::string> force_sources;  ///< Empty = no force data (replay, or no hydro model)
    double force_scale = 0.0;   ///< Arrow length per unit force in use [m/N]
    double moment_scale = 0.0;  ///< Arrow length per unit moment in use [m/(N m)]

    // --- Written by the GUI ---
    bool show_forces = false;   ///< Force arrows at each centre of mass
    bool show_moments = false;  ///< Moment arrows (double head, right-hand rule)
    /// Per force source, same order as force_sources; the viewer sizes it (all on).
    std::vector<char> source_shown;
    bool show_total = false;    ///< Also draw the sum of all sources
    bool lock_scale = false;    ///< Keep the arrow scale instead of following the loads
    bool show_trajectories = false;  ///< Centre-of-mass trails of moving bodies
    float trail_seconds = 30.0f;     ///< Trail length [s of simulated time]
    bool show_plots = false;         ///< Plot window
};

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

    // --- Body Annotations (SEA-Stack viewer) ---
    bool show_body_frames = false;  ///< Axis triad at each body reference frame
    bool show_body_labels = false;  ///< Body name at each centre of mass

    // --- Theme ---
    VisualTheme theme = VisualTheme::kDark;
    bool theme_changed = true;  ///< Scene theme needs (re)applying by GUIImplVSG

    // --- Camera ---
    // Requests set by the GUI and consumed by GUIImplVSG on the next frame.
    bool camera_view_requested = false;    ///< Move to camera_view, framing the model
    CameraView camera_view = CameraView::kIsometric;
    bool camera_fit_requested = false;     ///< Re-frame the model, keeping the view direction

    // --- Screenshot ---
    bool screenshot_requested = false;  ///< Save the 3D view as PNG (set by GUI, consumed by the viewer)
    bool capturing_frame = false;       ///< This frame is being saved: hide the view toolbar
    std::string screenshot_status;      ///< Last result shown next to the button; empty = none
    double screenshot_status_time = 0.0;  ///< ImGui time the status was shown first [s]; < 0 = new status

    // --- Timeline (SEA-Stack viewer) ---
    TimelineState timeline;

    // --- Runtime parameters (SEA-Stack viewer) ---
    ParametersPanelState parameters;

    // --- Force vectors, trajectories, plots (SEA-Stack viewer) ---
    AnalysisState analysis;
};

/// Unused file path `<directory>/screenshot_YYYYMMDD_HHMMSS[_n].png`;
/// an empty directory means the working directory. Creates the directory.
std::string NextScreenshotPath(const std::string& directory);

/// ImGui component for SEA-Stack visualization overlay.
/// Docked layout: a transport bar along the bottom (play/pause, playback
/// speed, progress, run status), an inspector panel on the right (viewer
/// settings), and the 3D scene in the remaining rectangle, with camera
/// presets and the tension legend in its corners.
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

    /// Draw the toolbar and panels; call inside an ImGui frame. Used directly
    /// by viewers that are not a Chrono visual system (vsys == nullptr).
    void Draw();

    /// Display scale for panel sizes (e.g. 1.5 at 150 %); 1 by default.
    void SetUiScale(float scale);

    /// Window area not covered by the docked panels [px], as laid out by the
    /// last Draw(); zero size before the first frame. A viewer can render the
    /// 3D scene into this rectangle so no panel covers the model.
    ScreenRect SceneRect() const { return scene_rect_; }

  private:
    void RenderTransportBar(const ImVec2& pos, const ImVec2& size);
    /// Second transport row: scrub slider and frame buttons (when settings_->timeline.available).
    void RenderTimelineRow();
    /// "PLAYBACK" / "REPLAY" / "interpolated" banner at the bottom of the scene.
    void RenderTimelineBanner();
    /// Show recorded frames from time t [s]; pauses stepping, remembering whether to resume.
    void EnterPlayback(double t);
    /// Leave playback and show the live state again; resumes stepping if it was running.
    void GoLive();
    /// Play / Pause action: the simulation when live, the recording in playback.
    void TogglePlay();
    void RenderInspector(const ImVec2& pos, const ImVec2& size);
    void RenderViewToolbar();
    void RenderMooringPanel();
    /// Force vectors, trajectories and the plot window (SEA-Stack viewer).
    void RenderAnalysisPanel();
    /// TSDA/RSDA parameters: edit, reset, owner, and the log of applied changes.
    void RenderParametersPanel();
    /// One slider that sets `field` on every editable member (one row per member).
    /// The value is requested when the slider is released (or a typed value entered).
    void RenderParameterSlider(const std::string& key, const std::vector<const ParameterRow*>& members);
    void RenderColorBar();

    ::chrono::vsg3d::ChVisualSystemVSG* vsys_;
    // Toolbar readouts averaged over short intervals (see RenderToolbar).
    double readout_start_ = 0.0;  ///< ImGui time of the current interval start [s]
    long long readout_steps_sum_ = 0;
    int readout_frames_ = 0;
    double shown_steps_per_frame_ = 0.0;
    double shown_fps_ = 0.0;  ///< Frames drawn per second of the last interval [Hz]
    bool behind_shown_ = false;  ///< "Cannot keep up" warning currently shown
    float status_width_ = 0.0f;  ///< Widest run-status readout so far [px]; it never shrinks
    bool show_inspector_ = true;
    ScreenRect scene_rect_;
    bool& pressed_;
    double& playback_speed_;
    double last_finite_speed_ = 1.0;  ///< Speed restored when "Max" is switched off [-]
    bool imgui_theme_applied_ = false;
    float ui_scale_ = 1.0f;  ///< Display scale applied to panel sizes [-]
    bool fixed_range_error_ = false;  ///< Last fixed mooring range entry was rejected
    /// Parameter slider being dragged (or typed into) and its value; applied on release.
    std::string param_edit_key_;
    double param_edit_value_ = 0.0;
    /// Slider ranges set by the user (right-click), keyed by slider; {min, max} in parameter units.
    std::map<std::string, std::pair<double, double>> param_ranges_;
    VisualTheme imgui_theme_ = VisualTheme::kDark;  ///< Theme of the current ImGui style
    const RunStatus& run_status_;
    ViewerSettings* settings_;
    MooringLinesViz* mooring_viz_;
};

}  // namespace seastack::viz

