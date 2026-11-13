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

namespace seastack::viz {

namespace {

// Playback speed slider range [simulated s per wall s]. "Max" (unlimited)
// is a separate toggle, stored as a speed of 0.
constexpr float kMinSpeed = 0.05f;
constexpr float kMaxSpeed = 20.0f;

double ClampSpeed(double speed) {
    return std::clamp(speed, static_cast<double>(kMinSpeed), static_cast<double>(kMaxSpeed));
}

}  // namespace

SeastackGuiComponent::SeastackGuiComponent(::chrono::vsg3d::ChVisualSystemVSG* vsys, bool& button_pressed,
                                           double& playback_speed, const RunStatus& run_status,
                                           ViewerSettings* settings,
                                           MooringLinesViz* mooring_viz)
    : vsys_(vsys), pressed_(button_pressed), playback_speed_(playback_speed),
      run_status_(run_status), settings_(settings), mooring_viz_(mooring_viz) {}

void SeastackGuiComponent::RenderToolbar() {
    bool unlimited = playback_speed_ <= 0.0;
    if (!unlimited) {
        last_finite_speed_ = playback_speed_;
    }

    // Keyboard shortcuts (ignored while typing in an ImGui widget):
    //   Space = play/pause, [ / ] = half / double speed, \ = back to real time.
    const ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureKeyboard) {
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false) && !run_status_.finished) {
            pressed_ = !pressed_;
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

    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar |
                                    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoNav |
                                    ImGuiWindowFlags_AlwaysAutoResize;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->GetCenter().x, viewport->WorkPos.y + 10.0f * ui_scale_),
                            ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    // Grow-only width: the status text changes length every frame (e.g. the
    // "cannot keep up" warning), and a centred auto-sized window would jitter.
    ImGui::SetNextWindowSizeConstraints(ImVec2(toolbar_width_, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("SEA-Stack", nullptr, window_flags);

    // Play / Pause: the button names the action it performs.
    ImGui::BeginDisabled(run_status_.finished);
    if (ImGui::Button(pressed_ ? "Pause" : "Play", ImVec2(70.0f * ui_scale_, 0))) {
        pressed_ = !pressed_;
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::SetTooltip(run_status_.finished ? "Reached the configured end time." : "Space");
    }

    // Speed slider (logarithmic, so 0.1x-1x gets as much travel as 1x-10x).
    unlimited = playback_speed_ <= 0.0;
    float slider_speed = static_cast<float>(unlimited ? last_finite_speed_ : playback_speed_);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(180.0f * ui_scale_);
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

    // Line 1: simulated time and step size.
    const RunStatus& s = run_status_;
    if (s.end_time > 0.0) {
        ImGui::Text("t = %.2f / %.2f s", s.sim_time, s.end_time);
    } else {
        ImGui::Text("t = %.2f s", s.sim_time);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("dt = %.4g s", s.time_step);

    // Line 2: state, achieved vs requested speed, wall time, solver load.
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
    if (s.finished) {
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
    ImGui::SameLine();
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
    ImGui::TextDisabled("| wall %.1f s | %.1f steps/frame | %.0f fps", s.active_wall_time,
                        shown_steps_per_frame_, shown_fps_);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Wall time counts only time spent stepping (pauses excluded).");
    }

    toolbar_width_ = std::max(toolbar_width_, ImGui::GetWindowWidth());
    ImGui::End();
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

    RenderToolbar();

    // Viewer Settings panel (bottom left corner).
    if (settings_) {
        ImGuiWindowFlags settings_flags = 0;
        settings_flags |= ImGuiWindowFlags_AlwaysAutoResize;

        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10.0f * ui_scale_,
                                       viewport->WorkPos.y + viewport->WorkSize.y - 10.0f * ui_scale_),
                                ImGuiCond_FirstUseEver, ImVec2(0.0f, 1.0f));  // Anchor bottom-left

        ImGui::Begin("Viewer Settings", nullptr, settings_flags);

        // ---- View: theme and camera ----
        if (!ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_F, false)) {
            settings_->camera_fit_requested = true;
        }
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
            for (const auto& v : kViews) {
                ImGui::SameLine();
                if (ImGui::Button(v.label)) {
                    settings_->camera_view = v.view;
                    settings_->camera_view_requested = true;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("%s", v.tooltip);
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

    RenderColorBar();
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

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float bar_w = 14.0f * ui_scale_;
    const float bar_h = 180.0f * ui_scale_;
    const float margin = 20.0f * ui_scale_;
    const float label_gap = 6.0f * ui_scale_;
    const float pad = 8.0f * ui_scale_;  // legend panel padding

    const float x0 = vp->WorkPos.x + vp->WorkSize.x - margin - bar_w;
    const float y0 = vp->WorkPos.y + vp->WorkSize.y * 0.5f - bar_h * 0.5f;

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
    const float line_h = ImGui::GetTextLineHeight();

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

