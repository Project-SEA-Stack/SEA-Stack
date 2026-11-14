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
#include "vsg_mooring_lines.h"
#include "vsg_theme.h"

#include <vsgImGui/imgui.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iterator>

namespace seastack::viz {

namespace {

// Playback speed slider range [simulated s per wall s]. "Max" (unlimited)
// is a separate toggle, stored as a speed of 0.
constexpr float kMinSpeed = 0.05f;
constexpr float kMaxSpeed = 20.0f;

// Docked layout sizes [px at 100 % display scale; multiplied by the UI scale].
constexpr float kInspectorWidth = 300.0f;
constexpr float kOverlayMargin = 10.0f;  // gap between scene overlays and the scene edge

// Docked panels fill a fixed slot of the window; the user cannot move them.
constexpr ImGuiWindowFlags kDockedFlags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                          ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                                          ImGuiWindowFlags_NoBringToFrontOnFocus;

double ClampSpeed(double speed) {
    return std::clamp(speed, static_cast<double>(kMinSpeed), static_cast<double>(kMaxSpeed));
}

}  // namespace

std::string NextScreenshotPath(const std::string& directory) {
    namespace fs = std::filesystem;
    const fs::path dir = directory.empty() ? fs::current_path() : fs::path(directory);
    std::error_code ec;
    fs::create_directories(dir, ec);

    const std::time_t now = std::time(nullptr);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
    fs::path path = dir / (std::string("screenshot_") + stamp + ".png");
    for (int n = 2; fs::exists(path); ++n) {
        path = dir / (std::string("screenshot_") + stamp + "_" + std::to_string(n) + ".png");
    }
    return path.string();
}

SeastackGuiComponent::SeastackGuiComponent(::chrono::vsg3d::ChVisualSystemVSG* vsys, bool& button_pressed,
                                           double& playback_speed, const RunStatus& run_status,
                                           ViewerSettings* settings,
                                           MooringLinesViz* mooring_viz)
    : vsys_(vsys), pressed_(button_pressed), playback_speed_(playback_speed),
      run_status_(run_status), settings_(settings), mooring_viz_(mooring_viz) {}

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
        if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket, false)) {
            playback_speed_ = ClampSpeed(last_finite_speed_ * 0.5);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_RightBracket, false)) {
            playback_speed_ = ClampSpeed(last_finite_speed_ * 2.0);
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
        ImGui::SetTooltip("Wall time counts only time spent stepping (pauses excluded).");
    }
    // Grow-only, so the progress bar does not jitter as the readouts change length.
    status_width_ = std::max(status_width_, ImGui::GetItemRectMax().x - status_x);

    if (settings_) {
        ImGui::SameLine(ImGui::GetWindowWidth() - style.WindowPadding.x - (toggle_w - group_gap));
        ImGui::Checkbox("Inspector", &show_inspector_);
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
    if (settings_ && !ImGui::GetIO().WantCaptureKeyboard) {
        if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            settings_->camera_fit_requested = true;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F12, false)) {
            settings_->screenshot_requested = true;
        }
    }

    // Docked layout: transport bar along the bottom, inspector on the right,
    // and the 3D scene in the rectangle that is left.
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    float bar_h = ImGui::GetFrameHeight() + 2.0f * ImGui::GetStyle().WindowPadding.y;
    if (settings_ && settings_->timeline.available) {
        bar_h += ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.y;
    }
    const float inspector_w = (settings_ && show_inspector_) ? kInspectorWidth * ui_scale_ : 0.0f;
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
    RenderColorBar();
    if (settings_ && settings_->timeline.available && !settings_->capturing_frame) {
        RenderTimelineBanner();
    }
}

// ---------------------------------------------------------------------------

void SeastackGuiComponent::RenderInspector(const ImVec2& pos, const ImVec2& size) {
    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Inspector", nullptr, kDockedFlags);

    // ---- View ----
    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool dark = settings_->theme == VisualTheme::kDark;
        if (ImGui::Checkbox("Dark mode", &dark)) {
            settings_->theme = dark ? VisualTheme::kDark : VisualTheme::kLight;
            settings_->theme_changed = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Dark studio background, or light theme with sky.\n"
                              "Visual only; set the start-up theme with\n"
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
    }

    // ---- Waves ----
    if (ImGui::CollapsingHeader("Waves", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Show Water", &settings_->show_water);

        ImGui::Checkbox("Show Wireframe", &settings_->show_water_grid);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Show mesh edges on water surface.\n"
                              "Helps visualize wave motion and ripples.");
        }

        {
            const char* resolution_labels[] = {"32x32", "64x64", "96x96", "128x128"};
            int current_idx = 0;
            for (int i = 0; i < ViewerSettings::kResolutionCount; ++i) {
                if (ViewerSettings::kResolutionOptions[i] == settings_->grid_resolution) {
                    current_idx = i;
                    break;
                }
            }
            int prev_idx = current_idx;
            if (ImGui::Combo("Grid Resolution", &current_idx, resolution_labels, ViewerSettings::kResolutionCount)) {
                if (current_idx != prev_idx) {
                    settings_->grid_resolution = ViewerSettings::kResolutionOptions[current_idx];
                    settings_->resolution_changed = true;
                }
            }
        }

        ImGui::Checkbox("Radiation (approx.)", &settings_->show_radiation_viz);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Qualitative visual approximation of radiated waves.\n"
                              "Heights are empirical - not a quantitative result.\n"
                              "Does NOT affect physics - for feedback only.");
        }

        if (settings_->show_radiation_viz) {
            ImGui::Indent();
            ImGui::SliderFloat("Visual Scale##rad", &settings_->radiation_visual_scale, 0.1f, 5.0f, "%.1fx");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Visual amplification for inspection.\n"
                                  "1.0x = baseline, increase to see small ripples.\n"
                                  "Does NOT affect physics.");
            }
            ImGui::TextDisabled("Wavelength/speed derived from wave period");
            ImGui::Unindent();
        }
    }

    // ---- Mooring ----
    RenderMooringPanel();

    // ---- Display ----
    if (ImGui::CollapsingHeader("Display")) {
        ImGui::SliderInt("Update Hz", &settings_->update_hz, 10, 60, "%d Hz");

        if (vsys_) {
            bool chrono_panel = vsys_->IsBaseGuiVisible();
            if (ImGui::Checkbox("Chrono debug panel", &chrono_panel)) {
                vsys_->SetBaseGuiVisibility(chrono_panel);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Chrono's own panel: body/COM/link frames, labels,\n"
                                  "collision shapes. Its wall clock includes pauses.");
            }
        } else {
            ImGui::Checkbox("Body frames", &settings_->show_body_frames);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Axes of each body reference frame:\n"
                                  "x red, y green, z blue.");
            }
            ImGui::Checkbox("Body labels", &settings_->show_body_labels);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Body names, placed at each centre of mass.");
            }
        }

        ImGui::Checkbox("Show Status", &settings_->show_water_status);
        if (settings_->show_water_status) {
            ImGui::TextDisabled("Water: %s | Grid: %d | Hz: %d",
                                settings_->show_water ? "ON" : "OFF",
                                settings_->grid_resolution,
                                settings_->update_hz);
        }
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------

void SeastackGuiComponent::RenderViewToolbar() {
    if (!settings_ || settings_->capturing_frame) return;

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

    ImGui::SameLine(0.0f, 3.0f * ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Button("Screenshot")) {
        settings_->screenshot_requested = true;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Save the 3D view as PNG in the case output folder (F12).");
    }
    constexpr double kStatusSeconds = 4.0;
    if (settings_->screenshot_status_time < 0.0) {
        settings_->screenshot_status_time = ImGui::GetTime();
    }
    if (!settings_->screenshot_status.empty() &&
        ImGui::GetTime() - settings_->screenshot_status_time < kStatusSeconds) {
        ImGui::SameLine();
        ImGui::TextUnformatted(settings_->screenshot_status.c_str());
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------

void SeastackGuiComponent::RenderMooringPanel() {
    if (!settings_ || !mooring_viz_ || !mooring_viz_->IsInitialized()) return;

    if (!ImGui::CollapsingHeader("Mooring", ImGuiTreeNodeFlags_DefaultOpen)) return;

    ImGui::Checkbox("Colour by Tension", &settings_->show_mooring_colors);
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

void SeastackGuiComponent::RenderColorBar() {
    if (!settings_ || !settings_->show_mooring_colors) return;
    if (!mooring_viz_ || !mooring_viz_->IsInitialized()) return;

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
}

}  // namespace seastack::viz
