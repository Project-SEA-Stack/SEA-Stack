// =============================================================================
// SEA-Stack VSG GUI Component - Implementation
// =============================================================================

// Suppress MSVC warning about IMGUI_API macro redefinition in vsgImGui headers.
// This is a harmless conflict between vsgImGui/imgui.h and vsgImGui/Export.h.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4005)  // macro redefinition
#endif

#include "vsg_gui_component.h"
#include "vsg_analysis_overlays.h"
#include "vsg_mooring_lines.h"
#include "vsg_theme.h"
#include "vsg_water_surface.h"

#include <vsgImGui/imgui.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <seastack/viz_core/frame_series.h>
#include <seastack/viz_core/slider_step.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>

namespace seastack::viz {

namespace {

// Playback speed slider range [simulated s per wall s]. "Max" (unlimited)
// is a separate toggle, stored as a speed of 0.
constexpr float kMinSpeed = 0.05f;
constexpr float kMaxSpeed = 20.0f;

// Docked layout sizes [px at 100 % display scale; multiplied by the UI scale].
constexpr float kInspectorWidth = 340.0f;
constexpr float kOverlayMargin = 10.0f;  // gap between scene overlays and the scene edge

// Docked panels fill a fixed slot of the window; the user cannot move them.
constexpr ImGuiWindowFlags kDockedFlags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                          ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                                          ImGuiWindowFlags_NoBringToFrontOnFocus;

double ClampSpeed(double speed) {
    return std::clamp(speed, static_cast<double>(kMinSpeed), static_cast<double>(kMaxSpeed));
}

/// Collapsing header whose open state is kept in `*open` (saved in the preferences).
bool Section(const char* label, bool* open) {
    ImGui::SetNextItemOpen(*open, ImGuiCond_Always);
    *open = ImGui::CollapsingHeader(label);
    return *open;
}

}  // namespace

SeastackGuiComponent::SeastackGuiComponent(::chrono::vsg3d::ChVisualSystemVSG* vsys, bool& button_pressed,
                                           double& playback_speed, const RunStatus& run_status,
                                           ViewerSettings* settings,
                                           MooringLinesViz* mooring_viz)
    : vsys_(vsys), pressed_(button_pressed), playback_speed_(playback_speed),
      run_status_(run_status), settings_(settings), mooring_viz_(mooring_viz) {
    if (!settings_) return;
    prefs_path_ = DefaultViewerPreferencesPath();
    if (!prefs_path_.empty()) {
        saved_prefs_ = LoadViewerPreferences(prefs_path_);
        ApplyPreferences(saved_prefs_, settings_);
        settings_->resolution_changed = false;  // no water mesh yet; it is built at the loaded size
    }
    start_theme_ = settings_->theme;
}

SeastackGuiComponent::~SeastackGuiComponent() {
    SavePreferencesIfChanged(true);
}

ViewerPreferences PreferencesFromSettings(const ViewerSettings& s) {
    ViewerPreferences p;
    p.theme = s.theme;
    p.water_wireframe = s.show_water_grid;
    p.shadows = s.show_shadows;
    p.water_detail = s.water_detail;
    p.water_style = s.water_style;
    p.water_palette = s.water_palette;
    p.water_contours = s.water_contours;
    p.update_hz = s.update_hz;
    p.smooth_slow_motion = s.timeline.smooth_slow_motion;
    p.mooring_colours = s.show_mooring_colors;
    p.show_inspector = s.inspector.visible;
    p.show_moments = s.analysis.show_moments;
    p.show_total_force = s.analysis.show_total;
    p.trail_seconds = s.analysis.trail_seconds;
    p.open_scene = s.inspector.scene;
    p.open_selected = s.inspector.selected;
    p.open_loads = s.inspector.loads;
    p.open_sea_state = s.inspector.sea_state;
    p.open_connectors = s.inspector.connectors;
    p.open_changes = s.inspector.changes;
    p.open_display = s.inspector.display;
    return p;
}

void ApplyPreferences(const ViewerPreferences& p, ViewerSettings* s) {
    if (!s->theme_from_model && s->theme != p.theme) {
        s->theme = p.theme;
        s->theme_changed = true;
    }
    s->show_water_grid = p.water_wireframe;
    s->show_shadows = p.shadows;
    s->water_style = p.water_style;
    s->water_palette = p.water_palette;
    s->water_contours = p.water_contours;
    if (s->water_detail != p.water_detail) {
        s->water_detail = p.water_detail;
        s->water_detail_changed = true;
    }
    s->update_hz = p.update_hz;
    s->timeline.smooth_slow_motion = p.smooth_slow_motion;
    s->show_mooring_colors = p.mooring_colours;
    s->inspector.visible = p.show_inspector;
    s->analysis.show_moments = p.show_moments;
    s->analysis.show_total = p.show_total_force;
    s->analysis.trail_seconds = p.trail_seconds;
    s->inspector.scene = p.open_scene;
    s->inspector.selected = p.open_selected;
    s->inspector.loads = p.open_loads;
    s->inspector.sea_state = p.open_sea_state;
    s->inspector.connectors = p.open_connectors;
    s->inspector.changes = p.open_changes;
    s->inspector.display = p.open_display;
}

ViewerPreferences SeastackGuiComponent::CurrentPreferences() const {
    ViewerPreferences p = PreferencesFromSettings(*settings_);
    if (settings_->theme_from_model && p.theme == start_theme_) {
        p.theme = saved_prefs_.theme;
    }
    return p;
}

void SeastackGuiComponent::SavePreferencesIfChanged(bool now_or_never) {
    if (!settings_ || prefs_path_.empty()) return;
    const ViewerPreferences p = CurrentPreferences();
    if (p == saved_prefs_) {
        prefs_changed_time_ = -1.0;
        return;
    }
    if (!now_or_never) {
        const double now = ImGui::GetTime();
        if (prefs_changed_time_ < 0.0) prefs_changed_time_ = now;
        if (now - prefs_changed_time_ < 1.0) return;
    }
    if (!SaveViewerPreferences(prefs_path_, p)) {
        std::cerr << "[viz] Could not save viewer preferences to " << prefs_path_ << std::endl;
        prefs_path_.clear();  // do not retry every frame
        return;
    }
    saved_prefs_ = p;
    prefs_changed_time_ = -1.0;
}

void SeastackGuiComponent::RenderTransportBar(const ImVec2& pos, const ImVec2& size) {
    bool unlimited = playback_speed_ <= 0.0;
    if (!unlimited) {
        last_finite_speed_ = playback_speed_;
    }

    // Keyboard shortcuts (ignored while typing in an ImGui widget):
    //   Space = play/pause, [ / ] = half / double speed, \ = back to real time.
    const ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureKeyboard) {
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
            TogglePlay();
        }
        // vsgImGui maps '(' and ')' to ImGuiKey_Left/RightBracket and leaves
        // '[' and ']' unmapped, so use the typed characters (any keyboard layout).
        for (const ImWchar c : io.InputQueueCharacters) {
            if (c == '[') {
                playback_speed_ = ClampSpeed(last_finite_speed_ * 0.5);
                last_finite_speed_ = playback_speed_;
            } else if (c == ']') {
                playback_speed_ = ClampSpeed(last_finite_speed_ * 2.0);
                last_finite_speed_ = playback_speed_;
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Backslash, false)) {
            playback_speed_ = 1.0;
        }
    }

    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("##transport", nullptr,
                 kDockedFlags | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoNav);
    const ImGuiStyle& style = ImGui::GetStyle();
    const float group_gap = 3.0f * style.ItemSpacing.x;
    const bool playback = settings_ && settings_->timeline.available && settings_->timeline.playback;
    if (settings_ && settings_->timeline.available) {
        RenderTimelineRow();
    }
    ImGui::AlignTextToFramePadding();

    // Play / Pause: the button names the action it performs. In playback it
    // plays the recording; the simulation stays paused.
    const bool running = playback ? settings_->timeline.playing : pressed_;
    ImGui::BeginDisabled(!playback && run_status_.finished);
    if (ImGui::Button(running ? "Pause" : "Play", ImVec2(70.0f * ui_scale_, 0))) {
        TogglePlay();
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip(settings_ && settings_->timeline.replay ? "Play the replay (Space)."
                          : playback                ? "Play the recording (Space). The simulation stays paused."
                          : run_status_.finished    ? "Reached the configured end time."
                                                    : "Space");
    }

    // Speed slider (logarithmic, so 0.1x-1x gets as much travel as 1x-10x).
    unlimited = playback_speed_ <= 0.0;
    float slider_speed = static_cast<float>(unlimited ? last_finite_speed_ : playback_speed_);
    ImGui::SameLine(0.0f, group_gap);
    ImGui::SetNextItemWidth(150.0f * ui_scale_);
    ImGui::BeginDisabled(unlimited);
    if (ImGui::SliderFloat("##speed", &slider_speed, kMinSpeed, kMaxSpeed, "%.2fx",
                           ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp)) {
        playback_speed_ = slider_speed;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Playback speed: simulated time per wall-clock time.\n"
                          "Changes only how often the solver is stepped;\n"
                          "the time step and results are unchanged.\n"
                          "Ctrl+click to type a value.  [ / ] = half / double,  \\ = 1x");
    }

    ImGui::SameLine();
    if (ImGui::Button("1x")) {
        playback_speed_ = 1.0;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Real time");
    }

    ImGui::SameLine();
    if (ImGui::Checkbox("Max", &unlimited)) {
        playback_speed_ = unlimited ? 0.0 : last_finite_speed_;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Step as fast as the hardware allows.");
    }

    // Simulated time, in a slot sized for the end time so the progress bar
    // does not shift as the digits change.
    const RunStatus& s = run_status_;
    char time_text[64];
    char time_slot[64];
    if (s.end_time > 0.0) {
        std::snprintf(time_text, sizeof(time_text), "t = %.2f / %.2f s", s.sim_time, s.end_time);
        std::snprintf(time_slot, sizeof(time_slot), "t = %.2f / %.2f s", s.end_time, s.end_time);
    } else {
        std::snprintf(time_text, sizeof(time_text), "t = %.2f s", s.sim_time);
        std::snprintf(time_slot, sizeof(time_slot), "t = 00000.00 s");
    }
    ImGui::SameLine(0.0f, group_gap);
    const float time_x = ImGui::GetCursorPosX();
    ImGui::TextUnformatted(time_text);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Simulated time / end time.  dt = %.4g s", s.time_step);
    }

    // Progress through the run; takes the space the readouts leave free.
    const float toggle_w = settings_ ? ImGui::GetFrameHeight() + style.ItemInnerSpacing.x +
                                           ImGui::CalcTextSize("Inspector").x + group_gap
                                     : 0.0f;
    ImGui::SameLine(time_x + ImGui::CalcTextSize(time_slot).x + style.ItemSpacing.x);
    const float progress_w =
        std::max(60.0f * ui_scale_, ImGui::GetContentRegionAvail().x - status_width_ - toggle_w - group_gap);
    const float progress_h = 6.0f * ui_scale_;
    const float progress = s.end_time > 0.0 ? static_cast<float>(std::clamp(s.sim_time / s.end_time, 0.0, 1.0)) : 0.0f;
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 0.5f * (ImGui::GetFrameHeight() - progress_h));
    ImGui::ProgressBar(progress, ImVec2(progress_w, progress_h), "");

    // State, achieved vs requested speed, wall time, solver load.
    ImGui::SameLine(0.0f, group_gap);
    const float status_x = ImGui::GetCursorScreenPos().x;
    const ImVec4 kWarn(1.0f, 0.65f, 0.2f, 1.0f);
    // The per-frame falling_behind flag toggles almost every frame, so the
    // warning follows the windowed achieved speed with hysteresis instead:
    // on below 95 % of the requested speed, off again above 98 %.
    if (!pressed_ || !s.has_achieved_speed || playback_speed_ <= 0.0) {
        behind_shown_ = false;
    } else if (s.achieved_speed < 0.95 * playback_speed_) {
        behind_shown_ = true;
    } else if (s.achieved_speed > 0.98 * playback_speed_) {
        behind_shown_ = false;
    }
    if (playback) {
        ImGui::TextColored(kWarn, settings_->timeline.replay ? "Replay" : "Playback");
    } else if (s.finished) {
        ImGui::TextColored(kWarn, "Finished");
    } else if (!pressed_) {
        ImGui::TextDisabled("Paused");
    } else if (!s.has_achieved_speed) {
        ImGui::TextDisabled("Running...");
    } else if (behind_shown_) {
        ImGui::TextColored(kWarn, "%.2fx (cannot keep up with %gx)", s.achieved_speed, playback_speed_);
    } else {
        ImGui::Text("%.2fx", s.achieved_speed);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Achieved speed: simulated time / active wall time,\n"
                          "measured over the last %.1f s of stepping.", RunClock::kSpeedWindow);
    }
    ImGui::SameLine(0.0f, group_gap);
    // Steps per frame and frame rate change every frame; show averages
    // refreshed every kReadoutInterval so they can be read.
    constexpr double kReadoutInterval = 0.5;  // [s]
    readout_steps_sum_ += s.steps_last_frame;
    ++readout_frames_;
    const double now = ImGui::GetTime();
    if (now - readout_start_ >= kReadoutInterval) {
        shown_steps_per_frame_ = static_cast<double>(readout_steps_sum_) / readout_frames_;
        shown_fps_ = readout_frames_ / (now - readout_start_);
        readout_steps_sum_ = 0;
        readout_frames_ = 0;
        readout_start_ = now;
    }
    ImGui::TextDisabled("wall %.1f s    %.1f steps/frame    %.0f fps", s.active_wall_time,
                        shown_steps_per_frame_, shown_fps_);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Wall time while playing (pauses excluded). It includes drawing and\n"
                          "the wait that paces the run, so it is not a compute-speed measure.");
    }
    // Grow-only, so the progress bar does not jitter as the readouts change length.
    status_width_ = std::max(status_width_, ImGui::GetItemRectMax().x - status_x);

    if (settings_) {
        ImGui::SameLine(ImGui::GetWindowWidth() - style.WindowPadding.x - (toggle_w - group_gap));
        ImGui::Checkbox("Inspector", &settings_->inspector.visible);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Show or hide the settings panel.");
        }
    }

    ImGui::End();
}

void SeastackGuiComponent::EnterPlayback(double t) {
    TimelineState& tl = settings_->timeline;
    if (!tl.playback) {
        tl.playback = true;
        tl.playing = false;
        tl.resume_on_live = pressed_;
        pressed_ = false;  // physics stays paused while recorded frames are shown
    }
    tl.playback_time = std::clamp(t, tl.start_time, tl.end_time);
}

void SeastackGuiComponent::GoLive() {
    TimelineState& tl = settings_->timeline;
    if (tl.replay || !tl.playback) {
        return;
    }
    tl.playback = false;
    tl.playing = false;
    tl.step_request = 0;
    pressed_ = tl.resume_on_live && !run_status_.finished;
}

void SeastackGuiComponent::TogglePlay() {
    if (settings_ && settings_->timeline.available && settings_->timeline.playback) {
        TimelineState& tl = settings_->timeline;
        if (!tl.playing && tl.replay && tl.playback_time >= tl.end_time) {
            tl.playback_time = tl.start_time;  // play the replay again from the start
        }
        tl.playing = !tl.playing;
        return;
    }
    if (!run_status_.finished) {
        pressed_ = !pressed_;
    }
}

void SeastackGuiComponent::RenderTimelineRow() {
    TimelineState& tl = settings_->timeline;
    const double shown = tl.playback ? tl.playback_time : tl.end_time;
    const bool has_range = tl.frame_count >= 2 && tl.end_time > tl.start_time;

    // Keys: Home = first frame, End = live (or last frame), , / . = step one frame.
    if (has_range && !ImGui::GetIO().WantCaptureKeyboard) {
        if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) {
            EnterPlayback(tl.start_time);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_End, false)) {
            tl.replay ? EnterPlayback(tl.end_time) : GoLive();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Comma)) {
            EnterPlayback(shown);
            tl.playing = false;
            tl.step_request = -1;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Period)) {
            EnterPlayback(shown);
            tl.playing = false;
            tl.step_request = 1;
        }
    }

    ImGui::AlignTextToFramePadding();
    const ImVec2 button_size(ImGui::GetFrameHeight() * 1.4f, 0.0f);
    ImGui::BeginDisabled(!has_range);
    if (ImGui::Button("|<", button_size)) {
        EnterPlayback(tl.start_time);
        tl.playing = false;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip(tl.replay ? "Start: input pose before the first step (Home)" : "First recorded frame (Home)");
    }
    ImGui::SameLine();
    if (ImGui::Button("<", button_size)) {
        EnterPlayback(shown);
        tl.playing = false;
        tl.step_request = -1;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Previous recorded frame ( , )");
    }
    ImGui::SameLine();
    if (ImGui::Button(">", button_size)) {
        EnterPlayback(shown);
        tl.playing = false;
        tl.step_request = 1;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip("Next recorded frame ( . )");
    }

    char info[96];
    std::snprintf(info, sizeof(info), "%zu frames  %.0f MB", tl.frame_count, tl.memory_mb);
    const ImGuiStyle& style = ImGui::GetStyle();
    const float live_w = tl.replay ? 0.0f : ImGui::CalcTextSize("Live").x + 2.0f * style.FramePadding.x +
                                                style.ItemSpacing.x;
    const float info_w = ImGui::CalcTextSize(info).x + style.ItemSpacing.x;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(std::max(80.0f * ui_scale_, ImGui::GetContentRegionAvail().x - live_w - info_w));
    double slider_time = shown;
    double lo = tl.start_time;
    double hi = has_range ? tl.end_time : tl.start_time + 1.0;
    if (ImGui::SliderScalar("##timeline", ImGuiDataType_Double, &slider_time, &lo, &hi, "t = %.2f s")) {
        EnterPlayback(slider_time);
        tl.playing = false;
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip(tl.replay ? "Drag to scrub through the replay."
                                    : "Drag to scrub back through recorded frames.\n"
                                      "The simulation pauses while you look back;\n"
                                      "Live returns to it. Visual only: results are not changed.");
    }
    ImGui::EndDisabled();

    if (!tl.replay) {
        ImGui::SameLine();
        ImGui::BeginDisabled(!tl.playback);
        if (ImGui::Button("Live")) {
            GoLive();
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("Back to the live simulation (End); resumes it if it was running.");
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%s", info);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(tl.replay ? "Output samples loaded from the results file."
                                    : "Frames kept for scrubbing (one per rendered frame in which\n"
                                      "time advanced). The oldest are dropped beyond the memory budget.");
    }
}

void SeastackGuiComponent::RenderTimelineBanner() {
    const TimelineState& tl = settings_->timeline;
    char text[160];
    ImU32 color = IM_COL32(255, 176, 64, 255);
    if (tl.playback) {
        if (tl.replay && tl.showing_input_pose) {
            std::snprintf(text, sizeof(text), "REPLAY   t = %.2f s   -   input pose (before the first step)",
                          tl.playback_time);
        } else if (tl.replay) {
            std::snprintf(text, sizeof(text), "REPLAY   t = %.2f s%s", tl.playback_time,
                          tl.interpolated ? "   (interpolated)" : "");
        } else {
            std::snprintf(text, sizeof(text), "PLAYBACK   t = %.2f s   -   live at t = %.2f s%s", tl.playback_time,
                          tl.end_time, tl.interpolated ? "   (interpolated)" : "");
        }
    } else if (tl.smooth_slow_motion_active) {
        std::snprintf(text, sizeof(text), "Smooth slow motion: interpolated between steps");
        color = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    } else {
        return;
    }
    const float margin = kOverlayMargin * ui_scale_;
    const float pad = 6.0f * ui_scale_;
    const ImVec2 size = ImGui::CalcTextSize(text);
    const ImVec2 p0(scene_rect_.x + 0.5f * (scene_rect_.w - size.x) - pad,
                    scene_rect_.y + scene_rect_.h - margin - size.y - 2.0f * pad);
    const ImVec2 p1(p0.x + size.x + 2.0f * pad, p0.y + size.y + 2.0f * pad);
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->AddRectFilled(p0, p1, ImGui::GetColorU32(ImGuiCol_WindowBg), ImGui::GetStyle().WindowRounding);
    dl->AddRect(p0, p1, color, ImGui::GetStyle().WindowRounding);
    dl->AddText(ImVec2(p0.x + pad, p0.y + pad), color, text);
}

void SeastackGuiComponent::render(vsg::CommandBuffer& /*cb*/) {
    Draw();
}

void SeastackGuiComponent::SetUiScale(float scale) {
    ui_scale_ = scale > 0.0f ? scale : 1.0f;
    imgui_theme_applied_ = false;  // re-apply sizes on the next frame
}

void SeastackGuiComponent::Draw() {
    if (settings_ && (!imgui_theme_applied_ || imgui_theme_ != settings_->theme)) {
        ApplyImGuiTheme(settings_->theme, ui_scale_);
        imgui_theme_ = settings_->theme;
        imgui_theme_applied_ = true;
    }
    if (settings_) {
        settings_->analysis.hovered_source = AnalysisState::kNone;  // set again below while hovered
    }
    if (settings_ && !ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_F, false)) {
        settings_->camera_fit_requested = true;
    }

    // Docked layout: transport bar along the bottom, inspector on the right,
    // and the 3D scene in the rectangle that is left.
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    float bar_h = ImGui::GetFrameHeight() + 2.0f * ImGui::GetStyle().WindowPadding.y;
    if (settings_ && settings_->timeline.available) {
        bar_h += ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.y;
    }
    const float inspector_w = (settings_ && settings_->inspector.visible) ? kInspectorWidth * ui_scale_ : 0.0f;
    scene_rect_.x = vp->WorkPos.x;
    scene_rect_.y = vp->WorkPos.y;
    scene_rect_.w = std::max(vp->WorkSize.x - inspector_w, 1.0f);
    scene_rect_.h = std::max(vp->WorkSize.y - bar_h, 1.0f);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    RenderTransportBar(ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - bar_h), ImVec2(vp->WorkSize.x, bar_h));
    if (inspector_w > 0.0f) {
        RenderInspector(ImVec2(vp->WorkPos.x + vp->WorkSize.x - inspector_w, vp->WorkPos.y),
                        ImVec2(inspector_w, vp->WorkSize.y - bar_h));
    }
    ImGui::PopStyleVar();

    RenderViewToolbar();
    RenderWaterLegend(RenderColorBar());
    if (settings_ && settings_->timeline.available) {
        RenderTimelineBanner();
    }
    SavePreferencesIfChanged(false);
}

// ---------------------------------------------------------------------------

void SeastackGuiComponent::RenderInspector(const ImVec2& pos, const ImVec2& size) {
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Inspector", nullptr, kDockedFlags);

    // Ordered as the work goes: what is shown, the body picked, its loads,
    // then the inputs that change the run, then appearance.
    RenderScenePanel();
    if (!vsys_) {
        RenderSelectionPanel();
        RenderLoadsPanel();
    }
    RenderSeaStatePanel();
    RenderConnectorsPanel();
    RenderChangesPanel();
    RenderDisplayPanel();

    ImGui::End();
}

void SeastackGuiComponent::RenderDisplayPanel() {
    if (Section("Display", &settings_->inspector.display)) {
        bool dark = settings_->theme == VisualTheme::kDark;
        if (ImGui::Checkbox("Dark mode", &dark)) {
            settings_->theme = dark ? VisualTheme::kDark : VisualTheme::kLight;
            settings_->theme_changed = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Dark studio background, or light theme with sky.\n"
                              "Visual only; remembered between runs unless the model sets\n"
                              "simulation.visualization.theme: dark | light");
        }
        if (settings_->timeline.available && !settings_->timeline.replay) {
            ImGui::Checkbox("Smooth slow motion", &settings_->timeline.smooth_slow_motion);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("When each time step lasts several frames, draw poses\n"
                                  "interpolated between the last two steps (the view lags\n"
                                  "one step). Visual only; results are not changed.");
            }
        }
        ImGui::SliderInt("Update Hz", &settings_->update_hz, 10, 60, "%d Hz");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Most water-surface updates per second of wall time.");
        }
        if (vsys_) {
            bool chrono_panel = vsys_->IsBaseGuiVisible();
            if (ImGui::Checkbox("Chrono debug panel", &chrono_panel)) {
                vsys_->SetBaseGuiVisibility(chrono_panel);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Chrono's own panel: body/COM/link frames, labels,\n"
                                  "collision shapes. Its wall clock includes pauses.");
            }
        }
        ImGui::Checkbox("Show Status", &settings_->show_water_status);
        if (settings_->show_water_status) {
            ImGui::TextDisabled("Water: %s | Grid: %d | Hz: %d",
                                settings_->show_water ? "ON" : "OFF",
                                settings_->grid_resolution,
                                settings_->update_hz);
        }
        if (ImGui::SmallButton("Reset viewer preferences")) {
            ApplyPreferences(ViewerPreferences{}, settings_);
            settings_->inspector.display = true;  // keep the section being used open
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Theme, water grid, panel layout and force display are\n"
                              "remembered between runs%s%s.\nThis restores the defaults.",
                              prefs_path_.empty() ? "" : " in\n", prefs_path_.c_str());
        }
    }
}

void SeastackGuiComponent::RenderWaterRows() {
    ImGui::Checkbox("Water", &settings_->show_water);
    ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &settings_->show_water_grid);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Mesh edges on the water surface: shows the wave\n"
                          "shape and how finely the surface is drawn. On GPU water\n"
                          "they fade out where cells are smaller than a pixel.");
    }
    ImGui::SameLine();
    static constexpr WaterDetail kDetails[] = {WaterDetail::kCoarse, WaterDetail::kNormal, WaterDetail::kFine};
    static constexpr const char* kDetailLabels[] = {"Coarse", "Normal", "Fine"};
    ImGui::SetNextItemWidth(ImGui::CalcTextSize("Normal").x + ImGui::GetFrameHeight() +
                            2.0f * ImGui::GetStyle().FramePadding.x);
    if (ImGui::BeginCombo("##detail", kDetailLabels[static_cast<int>(settings_->water_detail)])) {
        for (int i = 0; i < 3; ++i) {
            if (ImGui::Selectable(kDetailLabels[i], kDetails[i] == settings_->water_detail) &&
                kDetails[i] != settings_->water_detail) {
                settings_->water_detail = kDetails[i];
                settings_->water_detail_changed = true;
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Water detail: grid points per wavelength at the\n"
                          "characteristic period (Coarse %.0f, Normal %.0f, Fine %.0f).\n"
                          "The grid follows the sea: it is rebuilt when the period\n"
                          "or the patch changes. Up to %d x %d points on the GPU,\n"
                          "%d x %d when the surface is computed on the CPU.",
                          WaterPointsPerWavelength(WaterDetail::kCoarse),
                          WaterPointsPerWavelength(WaterDetail::kNormal),
                          WaterPointsPerWavelength(WaterDetail::kFine), kMaxWaterGridResolution,
                          kMaxWaterGridResolution, AnimatedWaterSurface::kMaxCpuResolution,
                          AnimatedWaterSurface::kMaxCpuResolution);
    }
    if (settings_->water_cell_m > 0.0) {
        ImGui::Indent();
        const int nx = settings_->water_counts.nx;
        const int ny = settings_->water_counts.ny;
        const char* where = settings_->water_gpu ? "GPU" : "CPU";
        if (settings_->water_wavelength_m > 0.0) {
            const double per_wave = settings_->water_wavelength_m / settings_->water_cell_m;
            // Below about 20 points per wavelength the crests look faceted.
            constexpr double kCoarsePointsPerWave = 20.0;
            if (per_wave < kCoarsePointsPerWave) {
                ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.25f, 1.0f), "%d x %d %s, %.2g m cells, %.0f per wave",
                                   nx, ny, where, settings_->water_cell_m, per_wave);
            } else {
                ImGui::TextDisabled("%d x %d %s, %.2g m cells, %.0f per wave", nx, ny, where,
                                    settings_->water_cell_m, per_wave);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Grid drawn (points along x and y, displaced on the GPU or\n"
                                  "computed on the CPU), cell size, and grid points per\n"
                                  "deep-water wavelength (%.3g m) at the characteristic\n"
                                  "period. Visual only.",
                                  settings_->water_wavelength_m);
            }
        } else {
            ImGui::TextDisabled("%d x %d %s, %.2g m cells", nx, ny, where, settings_->water_cell_m);
        }
        ImGui::Unindent();
    }

    if (settings_->water_style_available) {
        int style = (settings_->water_style == WaterStyle::kElevation) ? 0 : 1;
        ImGui::TextUnformatted("Style");
        ImGui::SameLine();
        ImGui::RadioButton("Elevation", &style, 0);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Colour by surface elevation, with contour lines and a\n"
                              "legend in metres; the still-water level is the bold line.\n"
                              "No reflections, so the sea reads the same from any angle.");
        }
        ImGui::SameLine();
        ImGui::RadioButton("Shaded", &style, 1);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Sky reflection, sun glint and water colours:\n"
                              "a lit look rather than colouring by elevation.");
        }
        settings_->water_style = (style == 0) ? WaterStyle::kElevation : WaterStyle::kShaded;
        if (settings_->water_style == WaterStyle::kElevation) {
            const char* palettes[] = {"Deep", "Viridis", "Balance"};
            int palette = static_cast<int>(settings_->water_palette);
            ImGui::SetNextItemWidth(110.0f * ui_scale_);
            if (ImGui::Combo("Colours", &palette, palettes, IM_ARRAYSIZE(palettes))) {
                settings_->water_palette = static_cast<WaterPalette>(palette);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Deep: navy troughs to pale crests (cmocean).\n"
                                  "Viridis: purple troughs to yellow crests (matplotlib).\n"
                                  "Balance: blue troughs, white still water, red crests (cmocean).");
            }
            ImGui::SameLine();
            ImGui::Checkbox("Contours", &settings_->water_contours);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Lines of equal elevation, at the interval shown in the legend.");
            }
        }
    }

    RenderRadiatedWaveRows();
}

void SeastackGuiComponent::RenderRadiatedWaveRows() {
    static constexpr const char* kMethodHelp[kNumRadiatedWaveMethods] = {
        "No radiated waves.",
        "Haskind if the BEM file has excitation for headings all round,\n"
        "otherwise Approximate.",
        "Point multipoles per body (heave ring, surge / sway dipoles, yaw\n"
        "quadrupole) carrying the power the radiation damping B(w) takes\n"
        "from each motion. Pattern shape and phase are assumed, not computed.\n"
        "Each body is treated alone: coupling between bodies is ignored, so\n"
        "where their waves cancel (RM3 float and spar heaving together) the\n"
        "drawn wave is too high, by 2 to 3 times at 0.5 to 1 rad/s.",
        "Directional patterns from the BEM excitation force through the\n"
        "Haskind relation: the far field of the BEM radiation problem.",
        "Far-field patterns from the BEM solver's Kochin functions.",
        "Elevation computed by the BEM solver near the body.",
    };
    const auto method_index = [](RadiatedWaveMethod m) { return static_cast<int>(m); };
    const RadiatedWaveMethod requested = settings_->radiation_method;
    const RadiatedWaveMethod active = settings_->radiation_active;

    std::string preview = RadiatedWaveMethodName(requested);
    if (requested == RadiatedWaveMethod::kAuto && active != RadiatedWaveMethod::kOff) {
        preview += std::string(" (") + RadiatedWaveMethodName(active) + ")";
    }
    ImGui::TextUnformatted("Radiated waves");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150.0f * ui_scale_);
    if (ImGui::BeginCombo("##radiated_waves", preview.c_str())) {
        for (int i = 0; i < kNumRadiatedWaveMethods; ++i) {
            const auto method = static_cast<RadiatedWaveMethod>(i);
            const MethodAvailability& availability = settings_->radiation_availability[i];
            const bool enabled = availability.available || method == RadiatedWaveMethod::kOff;
            ImGui::BeginDisabled(!enabled);
            if (ImGui::Selectable(RadiatedWaveMethodName(method), method == requested)) {
                settings_->radiation_method = method;
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                if (enabled) {
                    ImGui::SetTooltip("%s", kMethodHelp[i]);
                } else {
                    ImGui::SetTooltip("%s\n\nNot available: %s.", kMethodHelp[i], availability.reason.c_str());
                }
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Waves the bodies' own motion makes, computed from the BEM\n"
                          "data and drawn on top of the incident sea. Radiated waves\n"
                          "only: the scattering (diffraction) of the incident sea by\n"
                          "the bodies is not drawn. Visual only; never affects the\n"
                          "simulation. While on, the water is computed on the CPU\n"
                          "instead of the GPU shader, which is slower on fine grids.");
    }
    if (requested == RadiatedWaveMethod::kOff) {
        return;
    }

    ImGui::Indent();
    if (active == RadiatedWaveMethod::kOff) {
        const MethodAvailability& availability = settings_->radiation_availability[method_index(requested)];
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.25f, 1.0f), "Not drawn: %s.",
                           availability.reason.empty() ? "no field yet" : availability.reason.c_str());
        ImGui::PopTextWrapPos();
    } else {
        if (settings_->radiation_approximate) {
            ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.25f, 1.0f), "Approximate - not a BEM result");
        } else {
            ImGui::TextDisabled("From BEM data");
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s\nRadiated waves only (no diffraction).", settings_->radiation_label.c_str());
        }
        ImGui::SliderFloat("Scale##rad", &settings_->radiation_visual_scale, 0.1f, 5.0f, "%.1fx");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Exaggerate the radiated waves for inspection (1.0x = computed\n"
                              "height). Multiplies the Wave scale. Visual only.");
        }
    }
    ImGui::Unindent();
}

// ---------------------------------------------------------------------------

void SeastackGuiComponent::RenderViewToolbar() {
    if (!settings_) return;

    const float margin = kOverlayMargin * ui_scale_;
    ImGui::SetNextWindowPos(ImVec2(scene_rect_.x + margin, scene_rect_.y + margin));
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("##view_toolbar", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoFocusOnAppearing);

    if (ImGui::Button("Fit")) {
        settings_->camera_fit_requested = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Frame the whole model, keeping the view direction (F).");
    }
    struct ViewButton {
        const char* label;
        CameraView view;
        const char* tooltip;
    };
    constexpr ViewButton kViews[] = {
        {"Iso", CameraView::kIsometric, "Isometric view from the -Y/+X side."},
        {"Front", CameraView::kFront, "Looking along +Y."},
        {"Side", CameraView::kSide, "Looking along -X."},
        {"Top", CameraView::kTop, "Looking down, +X right, +Y up."},
    };
    for (size_t i = 0; i < std::size(kViews); ++i) {
        // Wider gap between Fit and the fixed views.
        ImGui::SameLine(0.0f, i == 0 ? 3.0f * ImGui::GetStyle().ItemSpacing.x : -1.0f);
        if (ImGui::Button(kViews[i].label)) {
            settings_->camera_view = kViews[i].view;
            settings_->camera_view_requested = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s", kViews[i].tooltip);
        }
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------

void SeastackGuiComponent::RenderParameterSlider(const std::string& key,
                                                 const std::vector<const ParameterRow*>& members) {
    ParametersPanelState& panel = settings_->parameters;
    const ParameterRow& first = *members.front();
    auto current = [](const ParameterRow& r) { return r.pending ? r.pending_value : r.value; };

    std::vector<const ParameterRow*> editable;
    for (const ParameterRow* r : members) {
        if (r->owner.empty()) editable.push_back(r);
    }
    const std::vector<const ParameterRow*>& shown_rows = editable.empty() ? members : editable;
    const ParameterRow& ref = *shown_rows.front();

    constexpr double kInf = std::numeric_limits<double>::infinity();
    bool mixed = false;
    bool changed = false;
    bool pending = false;
    double d_lo = kInf, d_hi = -kInf, v_lo = kInf, v_hi = -kInf;  // defaults / current values
    double pos_lo = kInf, pos_hi = 0.0;                             // positive defaults (else values)
    for (const ParameterRow* r : shown_rows) {
        const double v = current(*r);
        mixed = mixed || (v != current(ref));
        changed = changed || (r->value != r->default_value);
        pending = pending || r->pending;
        d_lo = std::min(d_lo, r->default_value);
        d_hi = std::max(d_hi, r->default_value);
        v_lo = std::min(v_lo, v);
        v_hi = std::max(v_hi, v);
        if (r->default_value > 0.0) {
            pos_lo = std::min(pos_lo, r->default_value);
            pos_hi = std::max(pos_hi, r->default_value);
        }
    }
    if (pos_hi == 0.0) {
        for (const ParameterRow* r : shown_rows) {
            if (current(*r) > 0.0) {
                pos_lo = std::min(pos_lo, current(*r));
                pos_hi = std::max(pos_hi, current(*r));
            }
        }
    }

    // Slider range. Coefficients: logarithmic, start value /10 to x10. Rest
    // length: 0 to twice the start value. Rest angle: start value +/- pi rad.
    // With no positive start value there is no scale, so the field is typed.
    constexpr double kPi = 3.14159265358979323846;
    bool slider = true;
    bool logarithmic = false;
    double lo = 0.0, hi = 0.0;
    if (first.field == "rest_angle") {
        lo = std::min(d_lo - kPi, v_lo);
        hi = std::max(d_hi + kPi, v_hi);
    } else if (first.field == "height" || first.field == "height_scale") {
        slider = pos_hi > 0.0;  // calm (0) to three times the start height
        hi = std::max(3.0 * pos_hi, v_hi);
    } else if (first.field == "period") {
        slider = pos_hi > 0.0;  // half to twice the start period; the BEM check refuses what is not covered
        lo = std::min(0.5 * pos_lo, v_lo);
        hi = std::max(2.0 * pos_hi, v_hi);
    } else if (first.field == "direction") {
        lo = std::min(d_lo - 180.0, v_lo);  // [deg]
        hi = std::max(d_hi + 180.0, v_hi);
    } else if (first.field == "rest_length") {
        slider = pos_hi > 0.0;
        hi = std::max(2.0 * pos_hi, v_hi);
    } else {
        slider = pos_hi > 0.0;
        logarithmic = true;
        lo = pos_lo / 10.0;
        if (v_lo > 0.0) lo = std::min(lo, v_lo);
        hi = std::max(10.0 * pos_hi, v_hi);
    }
    const double default_lo = lo, default_hi = hi;
    const bool default_slider = slider;
    const auto custom_range = param_ranges_.find(key);
    if (custom_range != param_ranges_.end()) {
        slider = true;
        lo = custom_range->second.first;
        hi = custom_range->second.second;
    }

    ImGui::PushID(key.c_str());
    const float x0 = ImGui::GetCursorPosX();
    const float label_w = ImGui::CalcTextSize("Rest length").x + ImGui::GetStyle().ItemSpacing.x;
    const float slider_w = 1.3f * ImGui::CalcTextSize("-0.000000e+00").x;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(first.label.c_str());
    ImGui::SameLine(x0 + label_w);

    const bool editing = (param_edit_key_ == key);
    double v = editing ? param_edit_value_ : current(ref);
    ImGui::BeginDisabled(editable.empty() || panel.run_finished);
    // Fill the row, leaving room for the units and the Reset button.
    const ImGuiStyle& style = ImGui::GetStyle();
    const float trailing_w = ImGui::CalcTextSize(first.units.c_str()).x + ImGui::CalcTextSize("Reset").x +
                             2.0f * style.FramePadding.x + 2.0f * style.ItemSpacing.x;
    ImGui::SetNextItemWidth(std::max(slider_w, ImGui::GetContentRegionAvail().x - trailing_w));
    bool changed_now = false;
    // Sea-state sliders step in about 1% of the start height or period (SliderStepFor) and
    // 1 deg; a typed value (Ctrl+click) is kept as entered.
    double drag_step = 0.0;
    const char* value_format = "%.4g";
    char step_format[16];
    if (first.element == "waves") {
        if (first.field == "height" || first.field == "period") {
            const SliderStep s = SliderStepFor(first.default_value);
            drag_step = s.step;
            std::snprintf(step_format, sizeof(step_format), "%%.%df", s.decimals);
            value_format = step_format;
        } else if (first.field == "direction") {
            drag_step = 1.0;
            value_format = "%.0f";
        }
    }
    if (slider) {
        const char* format = (mixed && !editing) ? "mixed" : value_format;
        changed_now = ImGui::SliderScalar("##value", ImGuiDataType_Double, &v, &lo, &hi, format,
                                          logarithmic ? ImGuiSliderFlags_Logarithmic : ImGuiSliderFlags_None);
        if (changed_now && drag_step > 0.0 && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            v = std::clamp(drag_step * std::round(v / drag_step), lo, hi);
        }
    } else {
        changed_now = ImGui::InputDouble("##value", &v, 0.0, 0.0, "%.6g");
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        const double value = (changed_now || !editing) ? v : param_edit_value_;
        for (const ParameterRow* r : editable) {
            if (current(*r) != value) panel.requests.emplace_back(r->id, value);
        }
        param_edit_key_.clear();
    } else if (ImGui::IsItemActive()) {
        param_edit_key_ = key;
        param_edit_value_ = v;
    } else if (editing) {
        param_edit_key_.clear();  // cancelled (Escape)
    }
    ImGui::OpenPopupOnItemClick("range", ImGuiPopupFlags_MouseButtonRight);
    ImGui::EndDisabled();

    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && !editing) {
        const char* how = slider ? "Drag and release to apply from the next time step\n"
                                   "(Ctrl+click to type a value; right-click to change the slider range)."
                                 : "Start value is 0, so there is no scale for a slider:\n"
                                   "type a value and press Enter. It applies from the next time step.\n"
                                   "Right-click to set a slider range.";
        if (editable.empty()) {
            ImGui::SetTooltip("Set by %s; it cannot be changed here.\nValue at start: %.6g %s", first.owner.c_str(),
                              first.default_value, first.units.c_str());
        } else if (panel.run_finished) {
            ImGui::SetTooltip("The run has ended; changes can no longer take effect.");
        } else if (members.size() == 1 && first.element == "waves") {
            const char* what =
                (first.field == "period" || first.field == "direction")
                    ? "The sea is sampled again with the new value and crossfaded in\n"
                      "over two wave periods (excitation and the drawn sea);\n"
                      "the wave height dips mid-fade (to about 0.7 of Hs).\n"
                      "Refused if the BEM data does not cover the new sea,\n"
                      "or while the last period or direction change fades in."
                    : "Scales every wave component (excitation and the drawn sea).\n"
                      "A change ramps in over two wave periods; 0 = calm.";
            if (first.field == "direction") {
                ImGui::SetTooltip("Wave direction [deg]: the direction the waves travel towards,\n"
                                  "counter-clockwise from +x (0 = along +x, 90 = along +y).\n"
                                  "From %s. Start value %.6g.\n%s\n%s\n"
                                  "Every change is logged; Export schedule reruns it.",
                                  first.source.c_str(), first.default_value, how, what);
            } else {
                ImGui::SetTooltip("Wave %s [%s], from %s. Start value %.6g.\n%s\n%s\n"
                                  "Every change is logged; Export schedule reruns it.",
                                  first.label.c_str(), first.units.c_str(), first.source.c_str(),
                                  first.default_value, how, what);
            }
        } else if (members.size() == 1) {
            ImGui::SetTooltip("%s [%s], from %s. Start value %.6g.\n%s\nEvery change is logged; Export schedule reruns it.%s",
                              first.label.c_str(), first.units.c_str(), first.source.c_str(), first.default_value, how,
                              std::isfinite(first.min_value) ? "\nMust be >= 0." : "");
        } else {
            char owned_note[96] = "";
            if (editable.size() < members.size()) {
                std::snprintf(owned_note, sizeof(owned_note), "\n%zu owned links are left unchanged.",
                              members.size() - editable.size());
            }
            ImGui::SetTooltip("%s [%s] of %zu links%s.\n%s\nSets all of them; each link's change is logged.%s",
                              first.label.c_str(), first.units.c_str(), editable.size(),
                              mixed ? " (values differ)" : "", how, owned_note);
        }
    }

    // Slider range only changes what the slider can reach, never the model.
    if (ImGui::BeginPopup("range")) {
        ImGui::Text("Slider range for %s [%s]", first.label.c_str(), first.units.c_str());
        double r_lo = lo, r_hi = hi;
        ImGui::SetNextItemWidth(slider_w);
        const bool lo_edit = ImGui::InputDouble("Min", &r_lo, 0.0, 0.0, "%.6g");
        ImGui::SetNextItemWidth(slider_w);
        const bool hi_edit = ImGui::InputDouble("Max", &r_hi, 0.0, 0.0, "%.6g");
        const double floor = logarithmic ? 0.0 : first.min_value;
        auto valid = [&](double a, double b) {
            const bool above_floor = logarithmic ? a > floor : !(std::isfinite(floor) && a < floor);
            return std::isfinite(a) && std::isfinite(b) && a < b && above_floor;
        };
        if ((lo_edit || hi_edit) && valid(r_lo, r_hi)) param_ranges_[key] = {r_lo, r_hi};
        if (logarithmic) {
            ImGui::TextDisabled("Logarithmic scale: Min must be > 0.");
        } else if (std::isfinite(first.min_value)) {
            ImGui::TextDisabled("Min must be >= %.6g.", first.min_value);
        }
        if (ImGui::Button("Max x10") && valid(lo, 10.0 * hi)) param_ranges_[key] = {lo, 10.0 * hi};
        ImGui::SameLine();
        if (ImGui::Button(logarithmic ? "Min /10" : "Min - span")) {
            const double new_lo = logarithmic ? lo / 10.0 : lo - (hi - lo);
            if (valid(new_lo, hi)) param_ranges_[key] = {new_lo, hi};
        }
        if (custom_range != param_ranges_.end()) {
            if (ImGui::Button("Default range")) param_ranges_.erase(key);
            if (default_slider) {
                ImGui::SameLine();
                ImGui::TextDisabled("(%.4g to %.4g)", default_lo, default_hi);
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    ImGui::SameLine();
    ImGui::TextDisabled("%s", first.units.c_str());
    if (!editable.empty() && !panel.run_finished && changed && !pending) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset")) {
            for (const ParameterRow* r : editable) {
                if (r->value != r->default_value) panel.requests.emplace_back(r->id, r->default_value);
            }
        }
        if (ImGui::IsItemHovered()) {
            if (members.size() == 1) {
                ImGui::SetTooltip("Back to the start value %.6g %s (from the next step).", first.default_value,
                                  first.units.c_str());
            } else {
                ImGui::SetTooltip("Back to each link's start value (from the next step).");
            }
        }
    }
    if (pending) {
        ImGui::SameLine();
        ImGui::TextDisabled("next step");
    } else if (!panel.run_finished && ref.effective_time > panel.sim_time) {
        ImGui::SameLine();
        ImGui::TextDisabled("ramping, %.0f s", std::ceil(ref.effective_time - panel.sim_time));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Still ramping to %.6g %s; fully in effect at t = %.1f s.", ref.value,
                              ref.units.c_str(), ref.effective_time);
        }
    }
    ImGui::PopID();
}

void SeastackGuiComponent::RenderSeaStatePanel() {
    if (!settings_ || !settings_->parameters.available) return;
    const ParametersPanelState& panel = settings_->parameters;
    const ParameterRow* height = nullptr;
    const ParameterRow* period = nullptr;
    bool any_wave = false;
    for (const ParameterRow& row : panel.rows) {
        if (row.element != "waves") continue;
        any_wave = true;
        if (row.field == "height" || row.field == "height_scale") height = &row;
        if (row.field == "period") period = &row;
    }
    if (!any_wave || !Section("Sea state", &settings_->inspector.sea_state)) return;

    for (const ParameterRow& row : panel.rows) {
        if (row.element == "waves") RenderParameterSlider(row.id, {&row});
    }
    // Derived readout. Deep-water dispersion only: in finite depth the
    // wavelength is shorter, so the steepness shown is a lower bound.
    if (period && period->value > 0.0) {
        constexpr double kGravity = 9.81;  // [m/s^2]
        const double wavelength = DeepWaterWavelength(period->value, kGravity);
        if (height && height->field == "height" && wavelength > 0.0) {
            ImGui::TextDisabled("Wavelength %.3g m, %s/wavelength %.3f", wavelength, height->label.c_str(),
                                height->value / wavelength);
        } else {
            ImGui::TextDisabled("Wavelength %.3g m", wavelength);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Deep-water linear wavelength g T^2 / (2 pi) for the period in\n"
                              "effect now, and the steepness from it. In finite depth\n"
                              "the wavelength is shorter. Readout only.");
        }
    }
}

void SeastackGuiComponent::RenderConnectorsPanel() {
    if (!settings_ || !settings_->parameters.available) return;
    const ParametersPanelState& panel = settings_->parameters;

    // Links in model order, each with its rows (fields in registry order).
    struct LinkRows {
        std::string element;
        std::string name;
        std::string heading;
        std::vector<const ParameterRow*> rows;
    };
    std::vector<LinkRows> links;
    for (const ParameterRow& row : panel.rows) {
        if (row.element != "tsda" && row.element != "rsda") continue;
        if (links.empty() || links.back().element != row.element || links.back().name != row.link_name) {
            links.push_back({row.element, row.link_name, row.link, {}});
        }
        links.back().rows.push_back(&row);
    }

    // Sliders for one field across several links: one per field all links have.
    auto render_group_fields = [&](const std::string& key, const std::vector<const LinkRows*>& members) {
        for (size_t f = 0; f < members.front()->rows.size(); ++f) {
            const std::string& field = members.front()->rows[f]->field;
            std::vector<const ParameterRow*> rows;
            for (const LinkRows* link : members) {
                for (const ParameterRow* row : link->rows) {
                    if (row->field == field) rows.push_back(row);
                }
            }
            if (rows.size() == members.size()) {
                RenderParameterSlider(key + "." + field, rows);
            }
        }
    };
    auto render_link = [&](const LinkRows& link) {
        for (const ParameterRow* row : link.rows) {
            RenderParameterSlider(row->id, {row});
        }
    };

    if (links.empty() || !Section("PTO and connectors", &settings_->inspector.connectors)) return;
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Spring-damper links (TSDA, RSDA): PTOs, rams and springs.");
    }
    if (links.size() == 1) {
        ImGui::TextUnformatted(links.front().heading.c_str());
        render_link(links.front());
    } else {
        for (const char* element : {"tsda", "rsda"}) {
            std::vector<const LinkRows*> of_element;
            for (const LinkRows& link : links) {
                if (link.element == element) of_element.push_back(&link);
            }
            if (of_element.empty()) continue;
            const bool tsda = std::string(element) == "tsda";

            // Links sharing a name prefix ("ram_12" for "ram_12_top", ...) form a group.
            std::vector<std::pair<std::string, std::vector<const LinkRows*>>> groups;
            std::vector<const LinkRows*> ungrouped;
            for (const LinkRows* link : of_element) {
                const size_t cut = link->name.find_last_of('_');
                const std::string prefix = (cut == std::string::npos || cut == 0) ? "" : link->name.substr(0, cut);
                size_t count = 0;
                for (const LinkRows* other : of_element) {
                    if (other->name.size() > prefix.size() && other->name.compare(0, prefix.size() + 1, prefix + "_") == 0) {
                        ++count;
                    }
                }
                if (prefix.empty() || count < 2 || count == of_element.size()) {
                    ungrouped.push_back(link);
                    continue;
                }
                auto it = std::find_if(groups.begin(), groups.end(), [&](const auto& g) { return g.first == prefix; });
                if (it == groups.end()) {
                    groups.push_back({prefix, {}});
                    it = std::prev(groups.end());
                }
                it->second.push_back(link);
            }
            std::sort(groups.begin(), groups.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });

            const bool few = of_element.size() <= 3;
            if (of_element.size() >= 2) {
                char heading[64];
                std::snprintf(heading, sizeof(heading), "All %s (%zu)", tsda ? "TSDAs" : "RSDAs", of_element.size());
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::TreeNode(heading)) {
                    render_group_fields(std::string("all.") + element, of_element);
                    ImGui::TreePop();
                }
            }
            for (const auto& [prefix, members] : groups) {
                char heading[96];
                std::snprintf(heading, sizeof(heading), "%s_* (%zu)", prefix.c_str(), members.size());
                if (ImGui::TreeNode(heading)) {
                    render_group_fields(std::string("group.") + element + "." + prefix, members);
                    for (const LinkRows* link : members) {
                        if (ImGui::TreeNode(link->name.c_str())) {
                            render_link(*link);
                            ImGui::TreePop();
                        }
                    }
                    ImGui::TreePop();
                }
            }
            for (const LinkRows* link : ungrouped) {
                ImGui::SetNextItemOpen(few, ImGuiCond_Once);
                if (ImGui::TreeNode(link->heading.c_str())) {
                    render_link(*link);
                    ImGui::TreePop();
                }
            }
        }
    }
}

void SeastackGuiComponent::RenderChangesPanel() {
    if (!settings_ || !settings_->parameters.available || settings_->parameters.rows.empty()) return;
    ParametersPanelState& panel = settings_->parameters;
    char heading[64];
    std::snprintf(heading, sizeof(heading), "Changes (%zu)###changes", panel.event_lines.size());
    if (!Section(heading, &settings_->inspector.changes)) {
        // Rejections must stay visible with the section closed.
        if (!panel.status.empty()) {
            ImGui::PushTextWrapPos(0.0f);
            ImGui::TextDisabled("%s", panel.status.c_str());
            ImGui::PopTextWrapPos();
        }
        return;
    }
    if (panel.event_lines.empty()) {
        ImGui::TextDisabled("No changes yet. Sea-state and connector edits are logged here.");
    } else {
        constexpr size_t kMaxShown = 8;
        const size_t first = panel.event_lines.size() > kMaxShown ? panel.event_lines.size() - kMaxShown : 0;
        if (first > 0) {
            ImGui::TextDisabled("  ... %zu earlier", first);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        for (size_t i = first; i < panel.event_lines.size(); ++i) {
            ImGui::TextWrapped("%s", panel.event_lines[i].c_str());
        }
        ImGui::PopStyleColor();
    }

    ImGui::BeginDisabled(panel.schedule_path.empty() || panel.event_lines.empty());
    if (ImGui::Button("Export schedule")) {
        panel.export_requested = true;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        if (panel.schedule_path.empty()) {
            ImGui::SetTooltip("Needs output_directory in the setup YAML.");
        } else {
            ImGui::SetTooltip("Write the changes to\n%s\nRerun: run_seastack <case> --nogui --schedule <file>.\n"
                              "Also written automatically when the run ends.",
                              panel.schedule_path.c_str());
        }
    }
    if (!panel.status.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("%s", panel.status.c_str());
        ImGui::PopTextWrapPos();
    }
}

// ---------------------------------------------------------------------------

void SeastackGuiComponent::RenderMooringColours() {
    if (!settings_ || !mooring_viz_ || !mooring_viz_->IsInitialized()) return;

    ImGui::Checkbox("Colour by tension", &settings_->show_mooring_colors);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Map tension magnitude to colour\n"
                          "along each mooring line.");
    }

    if (settings_->show_mooring_colors) {
        ImGui::Indent();

        ColorRange& range = mooring_viz_->TensionRange();
        int mode = static_cast<int>(range.Mode());
        bool mode_changed = false;
        mode_changed |= ImGui::RadioButton("Auto", &mode, static_cast<int>(ColorRangeMode::kAuto));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Track peak tension: jumps up to new peaks,\n"
                              "then relaxes slowly.");
        }
        ImGui::SameLine();
        mode_changed |= ImGui::RadioButton("Locked", &mode, static_cast<int>(ColorRangeMode::kLocked));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Freeze the current range for stable comparison.");
        }
        ImGui::SameLine();
        mode_changed |= ImGui::RadioButton("Fixed", &mode, static_cast<int>(ColorRangeMode::kFixed));
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Set min and max yourself. Startup value from\n"
                              "simulation.visualization.mooring_tension_range.");
        }
        if (mode_changed) {
            const auto new_mode = static_cast<ColorRangeMode>(mode);
            if (new_mode == ColorRangeMode::kFixed && !range.HasFixedRange()) {
                range.SetFixedRange(range.Min(), range.Max());  // start from what is shown
            }
            range.SetMode(new_mode);
        }

        ImGui::TextDisabled("Peak now: %.4g N", range.LastPeak());
        if (range.Mode() == ColorRangeMode::kFixed) {
            float limits[2] = {range.FixedMin(), range.FixedMax()};
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.75f);
            if (ImGui::InputFloat2("N##mooring_range", limits, "%.4g",
                                   ImGuiInputTextFlags_EnterReturnsTrue)) {
                fixed_range_error_ = !range.SetFixedRange(limits[0], limits[1]);
            }
            if (fixed_range_error_) {
                ImGui::TextDisabled("Needs min < max; kept previous range.");
            }
        } else {
            ImGui::TextDisabled("Range: %.3g .. %.3g N", range.Min(), range.Max());
            if (range.Mode() == ColorRangeMode::kAuto) {
                ImGui::SameLine();
                if (ImGui::SmallButton("Reset")) {
                    range.ResetAuto();
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Re-snap the range to the current peak tension.");
                }
            }
        }

        ImGui::Unindent();
    }
}

// ---------------------------------------------------------------------------

namespace {

/// Checkbox that sets a whole per-body column: ticked when every counted row
/// is on, drawn as mixed when only some are. Rows with applies[i] == 0 are not
/// counted. Returns true when clicked, with *all_on the new state.
bool ColumnToggle(const char* label, const std::vector<char>& flags, const std::vector<char>* applies,
                  bool* all_on) {
    size_t on = 0;
    size_t count = 0;
    for (size_t i = 0; i < flags.size(); ++i) {
        if (applies && i < applies->size() && !(*applies)[i]) continue;
        ++count;
        on += flags[i] ? 1 : 0;
    }
    // CheckboxFlags draws "mixed" when some but not all of the bits are set.
    int bits = (on > 0 ? 1 : 0) | (count > 0 && on == count ? 2 : 0);
    if (ImGui::CheckboxFlags(label, &bits, 3)) {
        *all_on = (bits == 3);
        return true;
    }
    return false;
}

void SetColumn(std::vector<char>* flags, const std::vector<char>* applies, bool on) {
    for (size_t i = 0; i < flags->size(); ++i) {
        if (applies && i < applies->size() && !(*applies)[i]) continue;
        (*flags)[i] = on ? 1 : 0;
    }
}

/// Small colour swatch matching the arrows of a force source (kTotal = text colour).
void SourceSwatch(int source) {
    const float size = ImGui::GetTextLineHeight();
    ImGui::ColorButton("##swatch", ImGui::ColorConvertU32ToFloat4(ForceSourceColor(source)),
                       ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop,
                       ImVec2(size, size));
}

}  // namespace

void SeastackGuiComponent::RenderScenePanel() {
    if (!settings_ || !Section("Scene", &settings_->inspector.scene)) return;
    ModelState& m = settings_->model;
    const int n = static_cast<int>(m.body_names.size());

    if (!vsys_ && n > 0) {
        const size_t count = m.body_names.size();
        for (std::vector<char>* v : {&m.body_visible, &m.body_label, &m.body_axes, &m.body_forces, &m.body_trail}) {
            if (v->size() != count) v->assign(count, v == &m.body_visible ? 1 : 0);
        }
        std::vector<char> moving(count, 1);
        for (size_t i = 0; i < count && i < m.body_fixed.size(); ++i) moving[i] = m.body_fixed[i] ? 0 : 1;
        if (m.selected >= n) m.selected = -1;

        struct Column {
            const char* name;
            const char* tooltip;
            std::vector<char>* flags;
            const std::vector<char>* applies;
        };
        const std::array<Column, 5> columns = {{
            {"Show", "Draw the body (visual only; the simulation is unchanged).", &m.body_visible, nullptr},
            {"Label", "Body name at its centre of mass.", &m.body_label, nullptr},
            {"Axes", "Body reference-frame axes: x red, y green, z blue.", &m.body_axes, nullptr},
            {"Force", "Force (and moment) arrows at the centre of mass.\nComponents and scale: Loads and motion.",
             &m.body_forces, nullptr},
            {"Trail", "Centre-of-mass path over the recorded frames.\nLength: Loads and motion.", &m.body_trail,
             &moving},
        }};
        constexpr int kMaxRows = 8;  // more bodies scroll inside the table
        const float row_h = ImGui::GetFrameHeightWithSpacing();
        const float table_h = n > kMaxRows ? (kMaxRows + 2) * row_h : 0.0f;
        ImGuiTableFlags flags =
            ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_NoPadOuterX;
        if (n > kMaxRows) flags |= ImGuiTableFlags_ScrollY;
        if (ImGui::BeginTable("##bodies", 6, flags, ImVec2(0.0f, table_h))) {
            ImGui::TableSetupScrollFreeze(0, 2);
            ImGui::TableSetupColumn(columns[0].name);
            ImGui::TableSetupColumn("Body", ImGuiTableColumnFlags_WidthStretch);
            for (size_t c = 1; c < columns.size(); ++c) ImGui::TableSetupColumn(columns[c].name);

            ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
            for (int c = 0; c < 6; ++c) {
                ImGui::TableSetColumnIndex(c);
                const Column* col = c == 0 ? &columns[0] : (c >= 2 ? &columns[static_cast<size_t>(c - 1)] : nullptr);
                ImGui::TableHeader(col ? col->name : "Body");
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", col ? col->tooltip
                                               : "Click to select (also in the 3D view);\ndouble-click to frame it.");
                }
            }

            // First row sets each column for every body.
            ImGui::TableNextRow();
            for (int c = 0; c < 6; ++c) {
                ImGui::TableSetColumnIndex(c);
                if (c == 1) {
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextDisabled("All bodies");
                    continue;
                }
                const Column& col = columns[static_cast<size_t>(c == 0 ? 0 : c - 1)];
                ImGui::PushID(c);
                bool on = false;
                if (ColumnToggle("##all", *col.flags, col.applies, &on)) SetColumn(col.flags, col.applies, on);
                ImGui::PopID();
            }

            for (int i = 0; i < n; ++i) {
                ImGui::PushID(i);
                ImGui::TableNextRow();
                for (int c = 0; c < 6; ++c) {
                    ImGui::TableSetColumnIndex(c);
                    if (c == 1) {
                        std::string label = m.body_names[i].empty() ? "(unnamed)" : m.body_names[i];
                        if (!moving[static_cast<size_t>(i)]) label += " (fixed)";
                        ImGui::AlignTextToFramePadding();
                        if (ImGui::Selectable(label.c_str(), m.selected == i, ImGuiSelectableFlags_AllowDoubleClick)) {
                            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                                m.selected = i;
                                m.frame_selected_requested = true;
                            } else {
                                m.selected = (m.selected == i) ? -1 : i;
                            }
                        }
                        continue;
                    }
                    const Column& col = columns[static_cast<size_t>(c == 0 ? 0 : c - 1)];
                    if (col.applies && !(*col.applies)[static_cast<size_t>(i)]) continue;  // no trail for fixed bodies
                    ImGui::PushID(c);
                    bool on = (*col.flags)[static_cast<size_t>(i)] != 0;
                    if (ImGui::Checkbox("##on", &on)) (*col.flags)[static_cast<size_t>(i)] = on ? 1 : 0;
                    ImGui::PopID();
                }
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        if (ImGui::SmallButton("Show all")) {
            std::fill(m.body_visible.begin(), m.body_visible.end(), static_cast<char>(1));
            m.lines_visible = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Show every body and the mooring lines again (after Isolate).");
        }
    }

    // Lines and cables, then the water.
    if (!vsys_ && m.has_lines) {
        ImGui::Checkbox("Mooring lines and cables", &m.lines_visible);
        if (m.lines_visible) {
            ImGui::Indent();
            RenderMooringColours();
            ImGui::Unindent();
        }
    } else if (mooring_viz_ && mooring_viz_->IsInitialized()) {
        ImGui::TextUnformatted("Mooring lines");
        ImGui::Indent();
        RenderMooringColours();
        ImGui::Unindent();
    }
    if (!vsys_) {
        ImGui::Checkbox("Shadows", &settings_->show_shadows);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Soft shadows from the main light, cast by the bodies onto\n"
                              "themselves and each other (not onto the water).");
        }
    }
    RenderWaterRows();
}

void SeastackGuiComponent::RenderSelectionPanel() {
    ModelState& m = settings_->model;
    AnalysisState& a = settings_->analysis;
    const int n = static_cast<int>(m.body_names.size());
    if (n == 0) return;
    const bool has = m.selected >= 0 && m.selected < n;
    char heading[160];
    std::snprintf(heading, sizeof(heading), "Selected: %s###selected",
                  has ? (m.body_names[m.selected].empty() ? "(unnamed)" : m.body_names[m.selected].c_str()) : "none");
    if (!Section(heading, &settings_->inspector.selected)) return;
    if (!has) {
        ImGui::TextDisabled("Click a body in the 3D view or in Scene.");
        return;
    }
    const size_t sel = static_cast<size_t>(m.selected);

    if (ImGui::Button("Isolate")) {
        for (size_t i = 0; i < m.body_visible.size(); ++i) m.body_visible[i] = (i == sel) ? 1 : 0;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Show only this body (Show all in Scene undoes it).");
    ImGui::SameLine();
    if (ImGui::Button("Frame")) m.frame_selected_requested = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Move the camera to this body (or double-click it).");
    ImGui::SameLine();
    if (ImGui::Button("Plot")) a.show_plots = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Open the plot window for this body.");

    const Vec3& p = m.selected_com;
    const Vec3& r = m.selected_rotation;
    const Vec3& v = m.selected_velocity;
    ImGui::Text("COM       %9.3f %9.3f %9.3f m", p[0], p[1], p[2]);
    ImGui::Text("Rotation  %9.2f %9.2f %9.2f deg", r[0], r[1], r[2]);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Roll, pitch, yaw: Cardan angles, extrinsic X-Y-Z\n"
                          "(about the fixed world axes; as euler_xyz in the HDF5 output).");
    }
    ImGui::Text("Velocity  %9.3f %9.3f %9.3f m/s", v[0], v[1], v[2]);

    // Load breakdown: one bar per component, coloured as its arrows.
    const size_t n_sources = a.force_sources.size();
    if (n_sources == 0 || m.selected_wrenches.size() != n_sources + 1) return;
    ImGui::Spacing();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Loads");
    ImGui::SameLine();
    if (sel < m.body_forces.size()) {
        bool arrows = m.body_forces[sel] != 0;
        if (ImGui::Checkbox("Arrows on this body", &arrows)) m.body_forces[sel] = arrows ? 1 : 0;
    }
    double max_force = 0.0;
    for (const Wrench& w : m.selected_wrenches) max_force = std::max(max_force, Magnitude(w.force));
    const float line_h = ImGui::GetTextLineHeightWithSpacing();
    const float name_w = ImGui::CalcTextSize("Nonlinear Froude-Krylov").x * 0.8f;
    const float value_w = ImGui::CalcTextSize("-000 MN").x;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (size_t k = 0; k <= n_sources; ++k) {
        const int source = k < n_sources ? static_cast<int>(k) : AnalysisState::kTotal;
        const Wrench& w = m.selected_wrenches[k];
        const std::string name = k < n_sources ? a.force_sources[k] : "Total";
        ImGui::PushID(static_cast<int>(k));
        if (ImGui::Selectable("##load", false, ImGuiSelectableFlags_None, ImVec2(0.0f, line_h - 2.0f))) {
            a.plot_force_request = source;
        }
        const bool hovered = ImGui::IsItemHovered();
        const ImVec2 r0 = ImGui::GetItemRectMin();
        const ImVec2 r1 = ImGui::GetItemRectMax();
        const float swatch = ImGui::GetTextLineHeight() * 0.8f;
        const float cy = 0.5f * (r0.y + r1.y);
        const ImU32 color = ForceSourceColor(source);
        dl->AddRectFilled(ImVec2(r0.x, cy - 0.5f * swatch), ImVec2(r0.x + swatch, cy + 0.5f * swatch), color);
        const float text_x = r0.x + swatch + ImGui::GetStyle().ItemSpacing.x;
        dl->PushClipRect(ImVec2(text_x, r0.y), ImVec2(text_x + name_w, r1.y), true);
        dl->AddText(ImVec2(text_x, cy - 0.5f * ImGui::GetTextLineHeight()), ImGui::GetColorU32(ImGuiCol_Text),
                    name.c_str());
        dl->PopClipRect();
        const float bar_x0 = text_x + name_w + ImGui::GetStyle().ItemSpacing.x;
        const float bar_x1 = std::max(bar_x0, r1.x - value_w - ImGui::GetStyle().ItemSpacing.x);
        const double f = Magnitude(w.force);
        const float frac = max_force > 0.0 ? static_cast<float>(f / max_force) : 0.0f;
        const float bar_h = 0.5f * ImGui::GetTextLineHeight();
        dl->AddRectFilled(ImVec2(bar_x0, cy - 0.5f * bar_h), ImVec2(bar_x1, cy + 0.5f * bar_h),
                          ImGui::GetColorU32(ImGuiCol_FrameBg));
        dl->AddRectFilled(ImVec2(bar_x0, cy - 0.5f * bar_h), ImVec2(bar_x0 + frac * (bar_x1 - bar_x0), cy + 0.5f * bar_h),
                          color);
        const std::string value = FormatSi(f, "N");
        dl->AddText(ImVec2(r1.x - ImGui::CalcTextSize(value.c_str()).x, cy - 0.5f * ImGui::GetTextLineHeight()),
                    ImGui::GetColorU32(ImGuiCol_Text), value.c_str());
        if (hovered) {
            a.hovered_source = source;
            ImGui::SetTooltip("%s on %s (world frame)\n"
                              "F  = %10.4g %10.4g %10.4g N\n"
                              "M  = %10.4g %10.4g %10.4g N m (about the centre of mass)\n"
                              "Click to plot it over time.",
                              name.c_str(), m.body_names[sel].c_str(), w.force[0], w.force[1], w.force[2],
                              w.moment[0], w.moment[1], w.moment[2]);
        }
        ImGui::PopID();
    }
}

void SeastackGuiComponent::RenderLoadsPanel() {
    if (!Section("Loads and motion", &settings_->inspector.loads)) return;
    AnalysisState& a = settings_->analysis;
    ModelState& m = settings_->model;
    std::vector<char> moving(m.body_fixed.size(), 1);
    for (size_t i = 0; i < m.body_fixed.size(); ++i) moving[i] = m.body_fixed[i] ? 0 : 1;

    bool on = false;
    if (ColumnToggle("Force arrows", m.body_forces, nullptr, &on)) SetColumn(&m.body_forces, nullptr, on);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Loads the solver applied in the step shown, per hydro\n"
                          "model component (incl. mooring and external forces),\n"
                          "drawn at each centre of mass in the world frame.\n"
                          "Arrow lengths are scaled; the legend gives the scale.\n"
                          "PTO and joint forces are not included.\n"
                          "Per body: the Force column in Scene.");
    }
    ImGui::Indent();
    if (a.force_sources.empty()) {
        const TimelineState& tl = settings_->timeline;
        ImGui::TextDisabled(tl.replay                 ? "No loads recorded in this file."
                            : tl.end_time > tl.start_time ? "No hydro force data for this model."
                                                          : "Components appear once the run starts.");
    } else {
        // Components, each with the colour of its arrows. Hovering one
        // highlights its arrows in the view.
        for (size_t s = 0; s <= a.force_sources.size(); ++s) {
            const bool total = s == a.force_sources.size();
            const int source = total ? AnalysisState::kTotal : static_cast<int>(s);
            ImGui::PushID(static_cast<int>(s));
            SourceSwatch(source);
            ImGui::SameLine();
            if (total) {
                ImGui::Checkbox("Total", &a.show_total);
            } else if (s < a.source_shown.size()) {
                bool shown = a.source_shown[s] != 0;
                if (ImGui::Checkbox(a.force_sources[s].c_str(), &shown)) a.source_shown[s] = shown ? 1 : 0;
            }
            if (ImGui::IsItemHovered()) {
                a.hovered_source = source;
                if (total) ImGui::SetTooltip("Sum of all components, shown or not.");
            }
            ImGui::PopID();
        }
        ImGui::Checkbox("Moments", &a.show_moments);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Moments about the centre of mass as double-headed\n"
                              "arrows (right-hand rule), with their own scale.");
        }
        ImGui::SameLine();
        ImGui::Checkbox("Lock scale", &a.lock_scale);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Keep the current arrow scale. Otherwise the largest\n"
                              "load of the last 30 s sets it.");
        }
        ImGui::TextDisabled("Hover a component to highlight its arrows.");
    }
    ImGui::Unindent();

    if (ColumnToggle("Trails", m.body_trail, &moving, &on)) SetColumn(&m.body_trail, &moving, on);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Path of each moving body's centre of mass over the\n"
                          "recorded frames up to the time shown.\n"
                          "Per body: the Trail column in Scene.");
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::SliderFloat("##trail", &a.trail_seconds, 2.0f, 600.0f, "last %.0f s", ImGuiSliderFlags_Logarithmic);

    ImGui::Checkbox("Plots", &a.show_plots);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Time histories of body motions and loads from the\n"
                          "recorded frames (one point per rendered frame).\n"
                          "Follows the selected body.");
    }
}

float SeastackGuiComponent::RenderColorBar() {
    if (!settings_ || !settings_->show_mooring_colors) return 0.0f;
    if (!mooring_viz_ || !mooring_viz_->IsInitialized()) return 0.0f;

    const float bar_w = 14.0f * ui_scale_;
    const float bar_h = 180.0f * ui_scale_;
    const float margin = kOverlayMargin * ui_scale_;
    const float label_gap = 6.0f * ui_scale_;
    const float pad = 8.0f * ui_scale_;  // legend panel padding
    const float line_h = ImGui::GetTextLineHeight();

    // Top-right corner of the scene area, clear of the inspector.
    const float x0 = scene_rect_.x + scene_rect_.w - margin - pad - bar_w;
    const float y0 = scene_rect_.y + margin + pad + line_h + 2.0f * label_gap;

    ImDrawList* dl = ImGui::GetBackgroundDrawList();

    // Labels first, so the legend panel can be sized around them.
    char max_label[64];
    char min_label[64];
    std::snprintf(max_label, sizeof(max_label), "%.3g N", mooring_viz_->TensionMax());
    std::snprintf(min_label, sizeof(min_label), "%.3g N", mooring_viz_->TensionMin());
    const ColorRangeMode mode = mooring_viz_->TensionRange().Mode();
    const char* title = (mode == ColorRangeMode::kFixed)    ? "Tension (fixed)"
                        : (mode == ColorRangeMode::kLocked) ? "Tension (locked)"
                                                            : "Tension";
    const float label_w = std::max(ImGui::CalcTextSize(max_label).x, ImGui::CalcTextSize(min_label).x);
    const float title_w = ImGui::CalcTextSize(title).x;

    // Legend panel in the theme's window colour, so text stays readable over
    // both the dark background and the light theme's sky.
    const float panel_x0 = std::min(x0 - label_gap - label_w, x0 + bar_w - title_w) - pad;
    const float panel_y0 = y0 - line_h - 2.0f * label_gap - pad;
    const float panel_x1 = x0 + bar_w + pad;
    const float panel_y1 = y0 + bar_h + pad;
    const ImGuiStyle& style = ImGui::GetStyle();
    dl->AddRectFilled(ImVec2(panel_x0, panel_y0), ImVec2(panel_x1, panel_y1),
                      ImGui::GetColorU32(ImGuiCol_WindowBg), style.WindowRounding);
    dl->AddRect(ImVec2(panel_x0, panel_y0), ImVec2(panel_x1, panel_y1), ImGui::GetColorU32(ImGuiCol_Border),
                style.WindowRounding);
    const ImU32 text_color = ImGui::GetColorU32(ImGuiCol_Text);

    // Draw the gradient as a stack of thin horizontal strips.
    constexpr int kStrips = 64;
    const float strip_h = bar_h / kStrips;

    auto turbo_to_im = [](float t) -> ImU32 {
        const auto c = MooringLinesViz::TurboColormap(t);
        return IM_COL32(static_cast<int>(c.r * 255),
                        static_cast<int>(c.g * 255),
                        static_cast<int>(c.b * 255), 255);
    };

    for (int i = 0; i < kStrips; ++i) {
        const float t_top = 1.0f - static_cast<float>(i) / kStrips;
        const float t_bot = 1.0f - static_cast<float>(i + 1) / kStrips;

        const ImU32 col_top = turbo_to_im(t_top);
        const ImU32 col_bot = turbo_to_im(t_bot);
        dl->AddRectFilledMultiColor(
            ImVec2(x0, y0 + i * strip_h),
            ImVec2(x0 + bar_w, y0 + (i + 1) * strip_h),
            col_top, col_top, col_bot, col_bot);
    }

    // Outline.
    dl->AddRect(ImVec2(x0, y0), ImVec2(x0 + bar_w, y0 + bar_h), ImGui::GetColorU32(ImGuiCol_Border));

    // Labels: max at the top of the bar, min at the bottom, title above.
    dl->AddText(ImVec2(x0 - label_gap - ImGui::CalcTextSize(max_label).x, y0), text_color, max_label);
    dl->AddText(ImVec2(x0 - label_gap - ImGui::CalcTextSize(min_label).x, y0 + bar_h - line_h), text_color,
                min_label);
    dl->AddText(ImVec2(x0 + bar_w - title_w, y0 - line_h - label_gap), text_color, title);
    return panel_y1;
}

void SeastackGuiComponent::RenderWaterLegend(float top_y) {
    if (!settings_ || !settings_->show_water || !settings_->water_style_available ||
        settings_->water_style != WaterStyle::kElevation) {
        return;
    }
    const ElevationLegend& legend = settings_->water_legend;
    if (!(legend.range_m > 0.0)) {
        return;
    }

    const float bar_w = 14.0f * ui_scale_;
    const float bar_h = 140.0f * ui_scale_;
    const float margin = kOverlayMargin * ui_scale_;
    const float label_gap = 6.0f * ui_scale_;
    const float tick_w = 4.0f * ui_scale_;
    const float pad = 8.0f * ui_scale_;
    const float line_h = ImGui::GetTextLineHeight();

    // Top-right of the scene, under the tension legend if it is shown.
    const float panel_top = (top_y > 0.0f) ? top_y + margin : scene_rect_.y + margin;
    const float x0 = scene_rect_.x + scene_rect_.w - margin - pad - bar_w;
    const float y0 = panel_top + pad + line_h + 2.0f * label_gap;

    // Labels at +range, 0 and -range; "m" or "cm" so lab-scale seas read well.
    const bool centimetres = legend.range_m < 0.2;
    const double unit = centimetres ? 0.01 : 1.0;
    const char* unit_name = centimetres ? "cm" : "m";
    char top_label[32];
    char mid_label[32];
    char bottom_label[32];
    char contour_label[48];
    std::snprintf(top_label, sizeof(top_label), "+%.3g %s", legend.range_m / unit, unit_name);
    std::snprintf(mid_label, sizeof(mid_label), "0");
    std::snprintf(bottom_label, sizeof(bottom_label), "-%.3g %s", legend.range_m / unit, unit_name);
    std::snprintf(contour_label, sizeof(contour_label), "contours %.3g %s", legend.contour_m / unit, unit_name);
    const char* title = "Elevation";
    const float label_w = std::max(ImGui::CalcTextSize(top_label).x, ImGui::CalcTextSize(bottom_label).x);
    const float footer_w = ImGui::CalcTextSize(contour_label).x;
    const float title_w = ImGui::CalcTextSize(title).x;

    const float panel_x0 =
        std::min({x0 - label_gap - label_w, x0 + bar_w - title_w, x0 + bar_w - footer_w}) - pad;
    const float panel_y0 = panel_top;
    const float panel_x1 = x0 + bar_w + pad;
    const float panel_y1 = y0 + bar_h + label_gap + line_h + pad;
    const ImGuiStyle& style = ImGui::GetStyle();
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->AddRectFilled(ImVec2(panel_x0, panel_y0), ImVec2(panel_x1, panel_y1),
                      ImGui::GetColorU32(ImGuiCol_WindowBg), style.WindowRounding);
    dl->AddRect(ImVec2(panel_x0, panel_y0), ImVec2(panel_x1, panel_y1), ImGui::GetColorU32(ImGuiCol_Border),
                style.WindowRounding);
    const ImU32 text_color = ImGui::GetColorU32(ImGuiCol_Text);

    // The surface's colour map. ImGui colours are linear (sRGB swapchain).
    const WaterPalette palette = settings_->water_palette;
    auto to_im = [palette](float t) -> ImU32 {
        const std::array<float, 3> c = PaletteColor(palette, t);
        auto lin = [](float v) {
            return v <= 0.04045f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f);
        };
        return IM_COL32(static_cast<int>(lin(c[0]) * 255.0f + 0.5f), static_cast<int>(lin(c[1]) * 255.0f + 0.5f),
                        static_cast<int>(lin(c[2]) * 255.0f + 0.5f), 255);
    };
    constexpr int kStrips = 64;
    const float strip_h = bar_h / kStrips;
    for (int i = 0; i < kStrips; ++i) {
        const ImU32 top = to_im(1.0f - static_cast<float>(i) / kStrips);
        const ImU32 bottom = to_im(1.0f - static_cast<float>(i + 1) / kStrips);
        dl->AddRectFilledMultiColor(ImVec2(x0, y0 + i * strip_h), ImVec2(x0 + bar_w, y0 + (i + 1) * strip_h), top,
                                    top, bottom, bottom);
    }
    dl->AddRect(ImVec2(x0, y0), ImVec2(x0 + bar_w, y0 + bar_h), ImGui::GetColorU32(ImGuiCol_Border));

    // Contour ticks, the still-water one across the bar.
    const bool contours = settings_->water_contours && legend.contour_m > 0.0;
    if (contours) {
        const int n = static_cast<int>(std::floor(legend.range_m / legend.contour_m + 1e-9));
        for (int k = -n; k <= n; ++k) {
            const float y = y0 + bar_h * (0.5f - 0.5f * static_cast<float>(k * legend.contour_m / legend.range_m));
            const float x_start = (k == 0) ? x0 : x0 + bar_w - tick_w;
            dl->AddLine(ImVec2(x_start, y), ImVec2(x0 + bar_w, y), IM_COL32(0, 0, 0, 160), 1.0f * ui_scale_);
        }
    }

    dl->AddText(ImVec2(x0 - label_gap - ImGui::CalcTextSize(top_label).x, y0), text_color, top_label);
    dl->AddText(ImVec2(x0 - label_gap - ImGui::CalcTextSize(mid_label).x, y0 + 0.5f * (bar_h - line_h)), text_color,
                mid_label);
    dl->AddText(ImVec2(x0 - label_gap - ImGui::CalcTextSize(bottom_label).x, y0 + bar_h - line_h), text_color,
                bottom_label);
    dl->AddText(ImVec2(x0 + bar_w - title_w, y0 - line_h - label_gap), text_color, title);
    if (contours) {
        dl->AddText(ImVec2(x0 + bar_w - footer_w, y0 + bar_h + label_gap),
                    ImGui::GetColorU32(ImGuiCol_TextDisabled), contour_label);
    }
}

}  // namespace seastack::viz
