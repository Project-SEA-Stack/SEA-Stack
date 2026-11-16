// =============================================================================
// SEA-Stack viewer: force vectors, trajectories and live plots
// =============================================================================
// Suppress MSVC warning about IMGUI_API macro redefinition in vsgImGui headers.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4005)  // macro redefinition
#endif

#include "vsg_analysis_overlays.h"

#include "seastack_viewer.h"

#include <vsgImGui/imgui.h>
#include <vsgImGui/implot.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <seastack/viz_core/frame_series.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace seastack::viz {

namespace {

constexpr double kRadToDeg = 57.29577951308232;
constexpr double kScaleWindow = 30.0;     // [s] loads that set the automatic arrow scale
constexpr size_t kScaleSamples = 600;     // frames read for the scale
constexpr size_t kTrailSamples = 2000;    // points per trail
constexpr size_t kPlotSamples = 4000;     // points per plotted line
constexpr double kArrowFraction = 0.6;    // largest load = this fraction of the model radius

constexpr float kFadedAlpha = 0.18f;  // arrows of other components while one is highlighted

ImU32 WithAlpha(ImU32 color, float alpha) {
    const auto a = static_cast<ImU32>(std::clamp(alpha, 0.0f, 1.0f) * 255.0f);
    return (color & ~IM_COL32_A_MASK) | (a << IM_COL32_A_SHIFT);
}

/// Distance [px] from p to the segment a-b.
float SegmentDistance(const ImVec2& p, const ImVec2& a, const ImVec2& b) {
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float len2 = dx * dx + dy * dy;
    float t = len2 > 0.0f ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0f;
    t = std::clamp(t, 0.0f, 1.0f);
    const float ex = a.x + t * dx - p.x, ey = a.y + t * dy - p.y;
    return std::sqrt(ex * ex + ey * ey);
}

ImU32 BodyColor(const BodyDescription& body, float alpha) {
    Rgba c{0.9f, 0.9f, 0.9f, 1.0f};
    if (!body.shapes.empty()) {
        c = body.shapes.front().color;
    }
    // Lighten towards white so dark hulls still give a visible trail.
    auto channel = [](float v) { return static_cast<int>(255.0f * (0.35f + 0.65f * std::clamp(v, 0.0f, 1.0f))); };
    return IM_COL32(channel(c[0]), channel(c[1]), channel(c[2]), static_cast<int>(255.0f * alpha));
}

/// Line from a to b with a head at b (and at the middle for a moment).
void DrawArrow(ImDrawList* dl, const ImVec2& a, const ImVec2& b, ImU32 color, float thickness, bool moment) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    dl->AddLine(a, b, color, thickness);
    if (length < 2.0f * thickness) {
        return;
    }
    const float head = std::min(4.0f * thickness, 0.4f * length);
    const ImVec2 u(dx / length, dy / length);
    const ImVec2 n(-u.y, u.x);
    auto head_at = [&](const ImVec2& tip) {
        dl->AddTriangleFilled(tip, ImVec2(tip.x - head * u.x + 0.5f * head * n.x, tip.y - head * u.y + 0.5f * head * n.y),
                              ImVec2(tip.x - head * u.x - 0.5f * head * n.x, tip.y - head * u.y - 0.5f * head * n.y),
                              color);
    };
    head_at(b);
    if (moment) {
        head_at(ImVec2(b.x - 0.8f * head * u.x, b.y - 0.8f * head * u.y));
    }
}

Vec3 Add(const Vec3& p, const Vec3& v, double scale) {
    return {p[0] + scale * v[0], p[1] + scale * v[1], p[2] + scale * v[2]};
}

struct Quantity {
    const char* name;
    const char* axis;  ///< y-axis label with units
    const char* components[3];
    bool load;  ///< Needs force data
};

constexpr std::array<Quantity, 6> kQuantities = {{
    {"Position (centre of mass)", "[m]", {"x", "y", "z"}, false},
    {"Rotation (Cardan XYZ)", "[deg]", {"roll", "pitch", "yaw"}, false},
    {"Velocity (centre of mass)", "[m/s]", {"vx", "vy", "vz"}, false},
    {"Angular velocity (world)", "[deg/s]", {"wx", "wy", "wz"}, false},
    {"Force (world)", "[N]", {"Fx", "Fy", "Fz"}, true},
    {"Moment about centre of mass (world)", "[N m]", {"Mx", "My", "Mz"}, true},
}};

}  // namespace

ImU32 ForceSourceColor(int source) {
    static constexpr std::array<ImU32, 8> kPalette = {
        IM_COL32(86, 180, 233, 255),  IM_COL32(230, 159, 0, 255),   IM_COL32(0, 158, 115, 255),
        IM_COL32(204, 121, 167, 255), IM_COL32(240, 228, 66, 255),  IM_COL32(213, 94, 0, 255),
        IM_COL32(0, 114, 178, 255),   IM_COL32(170, 170, 170, 255),
    };
    if (source < 0) {
        return ImGui::GetColorU32(ImGuiCol_Text);
    }
    return kPalette[static_cast<size_t>(source) % kPalette.size()];
}

std::string FormatSi(double value, const char* unit) {
    static constexpr std::array<const char*, 5> kPrefix = {"", "k", "M", "G", "T"};
    size_t p = 0;
    double v = std::abs(value);
    while (v >= 1000.0 && p + 1 < kPrefix.size()) {
        v /= 1000.0;
        ++p;
    }
    char text[48];
    std::snprintf(text, sizeof(text), "%.3g %s%s", value < 0.0 ? -v : v, kPrefix[p], unit);
    return text;
}

void AnalysisOverlays::UpdateScales(const SceneDescription& scene, const FrameHistory& history,
                                    const VizFrame& shown, double model_radius,
                                    const std::vector<char>& with_arrows, AnalysisState* state) {
    if (state->lock_scale && state->force_scale > 0.0) {
        return;
    }
    const size_t n_sources = scene.force_sources.size();
    double max_force = 0.0;
    double max_moment = 0.0;
    auto include = [&](const VizFrame& frame) {
        if (frame.body_wrenches.size() != n_sources * frame.bodies.size()) {
            return;
        }
        for (size_t b = 0; b < frame.bodies.size(); ++b) {
            if (b >= with_arrows.size() || !with_arrows[b]) {
                continue;
            }
            for (size_t s = 0; s < n_sources; ++s) {
                if (s < state->source_shown.size() && state->source_shown[s]) {
                    const Wrench w = BodyWrench(frame, s, b);
                    max_force = std::max(max_force, Magnitude(w.force));
                    max_moment = std::max(max_moment, Magnitude(w.moment));
                }
            }
            if (state->show_total) {
                const Wrench w = TotalWrench(frame, b);
                max_force = std::max(max_force, Magnitude(w.force));
                max_moment = std::max(max_moment, Magnitude(w.moment));
            }
        }
    };
    for (size_t i : SampleIndices(history, shown.time - kScaleWindow, shown.time, kScaleSamples)) {
        include(history.At(i));
    }
    include(shown);
    const double length = kArrowFraction * std::max(model_radius, 1e-3);
    state->force_scale = max_force > 0.0 ? length / max_force : 0.0;
    state->moment_scale = max_moment > 0.0 ? length / max_moment : 0.0;
}

void AnalysisOverlays::DrawSceneOverlays(const SceneDescription& scene, const FrameHistory& history,
                                         const VizFrame& shown, const SeastackViewer& viewer,
                                         const ScreenRect& scene_rect, double model_radius, float ui_scale,
                                         const ModelState& model, AnalysisState* state) {
    auto on = [](const std::vector<char>& flags, size_t b) { return b < flags.size() && flags[b] != 0; };
    auto visible = [&](size_t b) { return b >= model.body_visible.size() || model.body_visible[b] != 0; };
    state->force_sources = scene.force_sources;
    if (state->source_shown.size() != scene.force_sources.size()) {
        state->source_shown.assign(scene.force_sources.size(), 1);
    }
    if (shown.bodies.size() != scene.bodies.size()) {
        return;
    }
    ImDrawList* dl = ImGui::GetBackgroundDrawList();
    dl->PushClipRect(ImVec2(scene_rect.x, scene_rect.y), ImVec2(scene_rect.x + scene_rect.w, scene_rect.y + scene_rect.h),
                     true);

    // ---- Trajectories ----
    bool any_trail = false;
    for (size_t b = 0; b < scene.bodies.size(); ++b) any_trail = any_trail || on(model.body_trail, b);
    if (any_trail && !history.Empty()) {
        const double t0 = shown.time - static_cast<double>(state->trail_seconds);
        const std::vector<size_t> indices = SampleIndices(history, t0, shown.time, kTrailSamples);
        const float thickness = 2.0f * ui_scale;
        std::vector<ImVec2> points;
        for (size_t b = 0; b < scene.bodies.size(); ++b) {
            if (scene.bodies[b].fixed || !visible(b) || !on(model.body_trail, b)) {
                continue;
            }
            points.clear();
            for (size_t i : indices) {
                const VizFrame& frame = history.At(i);
                float x = 0.0f, y = 0.0f;
                if (b < frame.bodies.size() && viewer.ProjectToWindow(frame.bodies[b].com_position, &x, &y)) {
                    points.emplace_back(x, y);
                }
            }
            float x = 0.0f, y = 0.0f;
            if (viewer.ProjectToWindow(shown.bodies[b].com_position, &x, &y)) {
                points.emplace_back(x, y);
            }
            // Older segments fade out.
            for (size_t k = 1; k < points.size(); ++k) {
                const float age = static_cast<float>(k) / static_cast<float>(points.size());
                dl->AddLine(points[k - 1], points[k], BodyColor(scene.bodies[b], 0.15f + 0.75f * age), thickness);
            }
        }
    }

    // ---- Force and moment arrows ----
    const size_t n_sources = scene.force_sources.size();
    const bool have_forces = n_sources > 0 && shown.body_wrenches.size() == n_sources * shown.bodies.size();
    std::vector<char> with_arrows(scene.bodies.size(), 0);
    bool any_arrows = false;
    for (size_t b = 0; b < scene.bodies.size(); ++b) {
        with_arrows[b] = (visible(b) && on(model.body_forces, b)) ? 1 : 0;
        any_arrows = any_arrows || with_arrows[b];
    }
    if (!have_forces || !any_arrows) {
        legend_hovered_ = arrow_hovered_ = AnalysisState::kNone;
        dl->PopClipRect();
        return;
    }

    // One component is highlighted while hovered in the panel, the legend
    // or the view; the others fade, and its arrows carry their magnitude.
    int highlight = state->hovered_source;
    if (highlight == AnalysisState::kNone) highlight = legend_hovered_;
    if (highlight == AnalysisState::kNone) highlight = arrow_hovered_;

    UpdateScales(scene, history, shown, model_radius, with_arrows, state);
    const float thickness = 2.5f * ui_scale;
    struct DrawnArrow {
        ImVec2 a, b;
        size_t body;
        int source;
        bool moment;
        Vec3 value;
    };
    std::vector<DrawnArrow> drawn;
    std::vector<std::pair<ImVec2, std::string>> tip_labels;
    auto draw = [&](size_t body, int source, const Vec3& v, bool moment) {
        const double scale = moment ? state->moment_scale : state->force_scale;
        if (scale <= 0.0 || Magnitude(v) * scale < 1e-9) {
            return;
        }
        const Vec3& origin = shown.bodies[body].com_position;
        float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
        if (!viewer.ProjectToWindow(origin, &ax, &ay) || !viewer.ProjectToWindow(Add(origin, v, scale), &bx, &by)) {
            return;
        }
        const bool faded = highlight != AnalysisState::kNone && highlight != source;
        const ImU32 color = WithAlpha(ForceSourceColor(source), faded ? kFadedAlpha : 1.0f);
        DrawArrow(dl, ImVec2(ax, ay), ImVec2(bx, by), color, highlight == source ? 1.4f * thickness : thickness,
                  moment);
        drawn.push_back({ImVec2(ax, ay), ImVec2(bx, by), body, source, moment, v});
        if (highlight == source) {
            tip_labels.emplace_back(ImVec2(bx, by), FormatSi(Magnitude(v), moment ? "N m" : "N"));
        }
    };
    for (size_t b = 0; b < shown.bodies.size(); ++b) {
        if (!with_arrows[b]) {
            continue;
        }
        for (size_t s = 0; s < n_sources; ++s) {
            const int source = static_cast<int>(s);
            // A hidden component is previewed while it is highlighted.
            if ((s < state->source_shown.size() && state->source_shown[s]) || highlight == source) {
                const Wrench w = BodyWrench(shown, s, b);
                draw(b, source, w.force, false);
                if (state->show_moments) draw(b, source, w.moment, true);
            }
        }
        if (state->show_total || highlight == AnalysisState::kTotal) {
            const Wrench w = TotalWrench(shown, b);
            draw(b, AnalysisState::kTotal, w.force, false);
            if (state->show_moments) draw(b, AnalysisState::kTotal, w.moment, true);
        }
    }
    for (const auto& [tip, text] : tip_labels) {
        const ImVec2 pos(tip.x + 4.0f * ui_scale, tip.y - ImGui::GetTextLineHeight());
        dl->AddText(ImVec2(pos.x + 1.0f, pos.y + 1.0f), IM_COL32(0, 0, 0, 200), text.c_str());
        dl->AddText(pos, IM_COL32(255, 255, 255, 255), text.c_str());
    }

    // Arrow under the mouse: tooltip with its values; highlights its component next frame.
    arrow_hovered_ = AnalysisState::kNone;
    const ImGuiIO& io = ImGui::GetIO();
    const ImVec2 mouse = io.MousePos;
    const bool mouse_in_scene = mouse.x >= scene_rect.x && mouse.y >= scene_rect.y &&
                                mouse.x <= scene_rect.x + scene_rect.w && mouse.y <= scene_rect.y + scene_rect.h;
    if (!io.WantCaptureMouse && mouse_in_scene && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        const float pick_px = 6.0f * ui_scale;
        const DrawnArrow* nearest = nullptr;
        float best = pick_px;
        for (const DrawnArrow& d : drawn) {
            const float dist = SegmentDistance(mouse, d.a, d.b);
            if (dist < best) {
                best = dist;
                nearest = &d;
            }
        }
        if (nearest) {
            arrow_hovered_ = nearest->source;
            const std::string& body = scene.bodies[nearest->body].name;
            const char* source = nearest->source == AnalysisState::kTotal
                                     ? "Total"
                                     : scene.force_sources[static_cast<size_t>(nearest->source)].c_str();
            const Vec3& v = nearest->value;
            ImGui::SetTooltip("%s on %s: %s %s\n%s = %10.4g %10.4g %10.4g %s (world)", source, body.c_str(),
                              nearest->moment ? "moment" : "force",
                              FormatSi(Magnitude(v), nearest->moment ? "N m" : "N").c_str(),
                              nearest->moment ? "M" : "F", v[0], v[1], v[2], nearest->moment ? "N m" : "N");
        }
    }
    dl->PopClipRect();

    DrawLegend(scene, scene_rect, ui_scale, state);
}

void AnalysisOverlays::DrawLegend(const SceneDescription& scene, const ScreenRect& scene_rect, float ui_scale,
                                  AnalysisState* state) {
    const size_t n_sources = scene.force_sources.size();
    const float margin = 10.0f * ui_scale;
    ImGui::SetNextWindowPos(ImVec2(scene_rect.x + margin, scene_rect.y + scene_rect.h - margin), ImGuiCond_Always,
                            ImVec2(0.0f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.85f);
    legend_hovered_ = AnalysisState::kNone;
    if (!ImGui::Begin("##force_legend", nullptr,
                      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                          ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
                          ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav |
                          ImGuiWindowFlags_NoScrollbar)) {
        ImGui::End();
        return;
    }
    // Each entry toggles its component; hovering highlights its arrows.
    const float swatch = ImGui::GetTextLineHeight();
    for (size_t s = 0; s <= n_sources; ++s) {
        const bool total = s == n_sources;
        const int source = total ? AnalysisState::kTotal : static_cast<int>(s);
        const bool shown = total ? state->show_total : (s < state->source_shown.size() && state->source_shown[s]);
        ImGui::PushID(static_cast<int>(s));
        const ImVec4 color = ImGui::ColorConvertU32ToFloat4(WithAlpha(ForceSourceColor(source), shown ? 1.0f : 0.25f));
        ImGui::ColorButton("##swatch", color,
                           ImGuiColorEditFlags_NoTooltip | ImGuiColorEditFlags_NoPicker | ImGuiColorEditFlags_NoDragDrop,
                           ImVec2(swatch, swatch));
        bool clicked = ImGui::IsItemClicked();
        bool hovered = ImGui::IsItemHovered();
        ImGui::SameLine();
        if (!shown) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        clicked = ImGui::Selectable(total ? "Total" : scene.force_sources[s].c_str(), false,
                                    ImGuiSelectableFlags_None) || clicked;
        hovered = hovered || ImGui::IsItemHovered();
        if (!shown) ImGui::PopStyleColor();
        if (clicked) {
            if (total) {
                state->show_total = !state->show_total;
            } else if (s < state->source_shown.size()) {
                state->source_shown[s] = state->source_shown[s] ? 0 : 1;
            }
        }
        if (hovered) {
            legend_hovered_ = source;
            ImGui::SetTooltip("%s", shown ? "Click to hide" : "Click to show (previewed while hovered)");
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    if (state->force_scale > 0.0) {
        ImGui::TextDisabled("Force: 1 m = %s", FormatSi(1.0 / state->force_scale, "N").c_str());
    }
    if (state->show_moments && state->moment_scale > 0.0) {
        ImGui::TextDisabled("Moment: 1 m = %s", FormatSi(1.0 / state->moment_scale, "N m").c_str());
    }
    ImGui::TextDisabled(state->lock_scale ? "Scale locked" : "Scale: largest load, last 30 s");
    ImGui::End();
}

void AnalysisOverlays::DrawPlots(const SceneDescription& scene, const FrameHistory& history, const VizFrame& shown,
                                 const ScreenRect& scene_rect, float ui_scale, int selected_body,
                                 AnalysisState* state) {
    if (selected_body != last_selected_) {
        last_selected_ = selected_body;
        if (selected_body >= 0) {
            plot_body_ = selected_body;
        }
    }
    if (state->plot_force_request != AnalysisState::kNone) {
        constexpr int kForceQuantity = 4;  // index of "Force (world)" in kQuantities
        plot_quantity_ = kForceQuantity;
        plot_source_ = state->plot_force_request == AnalysisState::kTotal ? -1 : state->plot_force_request;
        if (selected_body >= 0) {
            plot_body_ = selected_body;
        }
        state->plot_force_request = AnalysisState::kNone;
        state->show_plots = true;
    }
    if (!state->show_plots || scene.bodies.empty()) {
        return;
    }
    if (!ImPlot::GetCurrentContext()) {
        ImPlot::CreateContext();
    }
    if (plot_body_ < 0 || plot_body_ >= static_cast<int>(scene.bodies.size())) {
        plot_body_ = 0;
        for (size_t b = 0; b < scene.bodies.size(); ++b) {
            if (!scene.bodies[b].fixed) {
                plot_body_ = static_cast<int>(b);
                break;
            }
        }
    }
    const float margin = 10.0f * ui_scale;
    const ImVec2 size(560.0f * ui_scale, 320.0f * ui_scale);
    // Bottom-right of the scene, clear of the force legend at the bottom-left.
    ImGui::SetNextWindowPos(
        ImVec2(scene_rect.x + scene_rect.w - size.x - margin, scene_rect.y + scene_rect.h - size.y - margin),
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Plots", &state->show_plots, ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return;
    }

    const float combo_w = 150.0f * ui_scale;
    ImGui::SetNextItemWidth(combo_w);
    const std::string& body_name = scene.bodies[static_cast<size_t>(plot_body_)].name;
    if (ImGui::BeginCombo("##body", body_name.empty() ? "(unnamed)" : body_name.c_str())) {
        for (size_t b = 0; b < scene.bodies.size(); ++b) {
            const std::string label = (scene.bodies[b].name.empty() ? "(unnamed)" : scene.bodies[b].name) +
                                      (scene.bodies[b].fixed ? " (fixed)" : "") + "##" + std::to_string(b);
            if (ImGui::Selectable(label.c_str(), static_cast<int>(b) == plot_body_)) {
                plot_body_ = static_cast<int>(b);
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(combo_w * 1.3f);
    if (ImGui::BeginCombo("##quantity", kQuantities[static_cast<size_t>(plot_quantity_)].name)) {
        for (size_t q = 0; q < kQuantities.size(); ++q) {
            if (ImGui::Selectable(kQuantities[q].name, static_cast<int>(q) == plot_quantity_)) {
                plot_quantity_ = static_cast<int>(q);
            }
        }
        ImGui::EndCombo();
    }
    const Quantity& quantity = kQuantities[static_cast<size_t>(plot_quantity_)];
    const size_t n_sources = scene.force_sources.size();
    if (plot_source_ >= static_cast<int>(n_sources)) {
        plot_source_ = -1;
    }
    if (quantity.load) {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(combo_w);
        const char* source_name = plot_source_ < 0 ? "Total" : scene.force_sources[static_cast<size_t>(plot_source_)].c_str();
        if (ImGui::BeginCombo("##source", source_name)) {
            if (ImGui::Selectable("Total", plot_source_ < 0)) {
                plot_source_ = -1;
            }
            for (size_t s = 0; s < n_sources; ++s) {
                if (ImGui::Selectable(scene.force_sources[s].c_str(), static_cast<int>(s) == plot_source_)) {
                    plot_source_ = static_cast<int>(s);
                }
            }
            ImGui::EndCombo();
        }
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(combo_w * 0.8f);
    ImGui::SliderFloat("##window", &plot_window_s_, 5.0f, 3600.0f, "last %.0f s", ImGuiSliderFlags_Logarithmic);

    if (quantity.load && n_sources == 0) {
        ImGui::TextDisabled("No force data (replay, or no hydro model).");
        ImGui::End();
        return;
    }

    // Series from the recorded frames up to the frame shown.
    const size_t body = static_cast<size_t>(plot_body_);
    const double t_end = shown.time;
    const double t_begin = t_end - static_cast<double>(plot_window_s_);
    std::vector<double> t;
    std::array<std::vector<double>, 3> y;
    auto append = [&](const VizFrame& frame) {
        if (body >= frame.bodies.size()) {
            return;
        }
        const BodyState& s = frame.bodies[body];
        Vec3 v{0.0, 0.0, 0.0};
        switch (plot_quantity_) {
            case 0: v = s.com_position; break;
            case 1: v = CardanAnglesXYZ(s.pose.rotation); for (double& c : v) c *= kRadToDeg; break;
            case 2: v = s.linear_velocity; break;
            case 3: v = s.angular_velocity; for (double& c : v) c *= kRadToDeg; break;
            default: {
                if (frame.body_wrenches.size() != n_sources * frame.bodies.size()) {
                    return;  // frame recorded before the first force evaluation
                }
                const Wrench w = plot_source_ < 0 ? TotalWrench(frame, body)
                                                  : BodyWrench(frame, static_cast<size_t>(plot_source_), body);
                v = plot_quantity_ == 4 ? w.force : w.moment;
            }
        }
        t.push_back(frame.time);
        for (int k = 0; k < 3; ++k) {
            y[static_cast<size_t>(k)].push_back(v[static_cast<size_t>(k)]);
        }
    };
    for (size_t i : SampleIndices(history, t_begin, t_end, kPlotSamples)) {
        if (history.At(i).time < t_end) {
            append(history.At(i));
        }
    }
    append(shown);

    if (ImPlot::BeginPlot("##plot", ImVec2(-1.0f, -1.0f))) {
        ImPlot::SetupAxes("t [s]", quantity.axis, ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxisLimits(ImAxis_X1, t_begin, t_end, ImPlotCond_Always);
        for (size_t k = 0; k < 3; ++k) {
            ImPlot::PlotLine(quantity.components[k], t.data(), y[k].data(), static_cast<int>(t.size()));
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}

}  // namespace seastack::viz
