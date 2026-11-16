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

/// Colour of force source s (Total uses the text colour).
ImU32 SourceColor(size_t s) {
    static constexpr std::array<ImU32, 8> kPalette = {
        IM_COL32(86, 180, 233, 255),  IM_COL32(230, 159, 0, 255),   IM_COL32(0, 158, 115, 255),
        IM_COL32(204, 121, 167, 255), IM_COL32(240, 228, 66, 255),  IM_COL32(213, 94, 0, 255),
        IM_COL32(0, 114, 178, 255),   IM_COL32(170, 170, 170, 255),
    };
    return kPalette[s % kPalette.size()];
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

/// "12.3 kN"-style label of value [unit].
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

void AnalysisOverlays::UpdateScales(const SceneDescription& scene, const FrameHistory& history,
                                    const VizFrame& shown, double model_radius,
                                    const std::vector<char>& body_visible, AnalysisState* state) {
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
            if (b < body_visible.size() && !body_visible[b]) {
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
                                         const std::vector<char>& body_visible, AnalysisState* state) {
    auto hidden = [&](size_t b) { return b < body_visible.size() && !body_visible[b]; };
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
    if (state->show_trajectories && !history.Empty()) {
        const double t0 = shown.time - static_cast<double>(state->trail_seconds);
        const std::vector<size_t> indices = SampleIndices(history, t0, shown.time, kTrailSamples);
        const float thickness = 2.0f * ui_scale;
        std::vector<ImVec2> points;
        for (size_t b = 0; b < scene.bodies.size(); ++b) {
            if (scene.bodies[b].fixed || hidden(b)) {
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
    if (state->show_forces && have_forces) {
        UpdateScales(scene, history, shown, model_radius, body_visible, state);
        const float thickness = 2.5f * ui_scale;
        const ImU32 total_color = ImGui::GetColorU32(ImGuiCol_Text);
        auto draw = [&](const Vec3& origin, const Vec3& v, double scale, ImU32 color, bool moment) {
            if (scale <= 0.0 || Magnitude(v) * scale < 1e-9) {
                return;
            }
            float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
            if (viewer.ProjectToWindow(origin, &ax, &ay) && viewer.ProjectToWindow(Add(origin, v, scale), &bx, &by)) {
                DrawArrow(dl, ImVec2(ax, ay), ImVec2(bx, by), color, thickness, moment);
            }
        };
        for (size_t b = 0; b < shown.bodies.size(); ++b) {
            if (hidden(b)) {
                continue;
            }
            const Vec3& com = shown.bodies[b].com_position;
            for (size_t s = 0; s < n_sources; ++s) {
                if (s < state->source_shown.size() && state->source_shown[s]) {
                    const Wrench w = BodyWrench(shown, s, b);
                    draw(com, w.force, state->force_scale, SourceColor(s), false);
                    if (state->show_moments) {
                        draw(com, w.moment, state->moment_scale, SourceColor(s), true);
                    }
                }
            }
            if (state->show_total) {
                const Wrench w = TotalWrench(shown, b);
                draw(com, w.force, state->force_scale, total_color, false);
                if (state->show_moments) {
                    draw(com, w.moment, state->moment_scale, total_color, true);
                }
            }
        }

        // Legend, bottom-left of the scene.
        std::vector<std::pair<std::string, ImU32>> rows;
        for (size_t s = 0; s < n_sources; ++s) {
            if (s < state->source_shown.size() && state->source_shown[s]) {
                rows.emplace_back(scene.force_sources[s], SourceColor(s));
            }
        }
        if (state->show_total) {
            rows.emplace_back("Total", total_color);
        }
        std::vector<std::string> scale_lines;
        if (state->force_scale > 0.0) {
            scale_lines.push_back("Force: 1 m = " + FormatSi(1.0 / state->force_scale, "N"));
        }
        if (state->show_moments && state->moment_scale > 0.0) {
            scale_lines.push_back("Moment: 1 m = " + FormatSi(1.0 / state->moment_scale, "N m"));
        }
        scale_lines.push_back(state->lock_scale ? "Scale locked" : "Scale: largest load, last 30 s");
        const float pad = 8.0f * ui_scale;
        const float line_h = ImGui::GetTextLineHeightWithSpacing();
        const float swatch = 0.7f * ImGui::GetTextLineHeight();
        float width = 0.0f;
        for (const auto& [name, color] : rows) {
            width = std::max(width, swatch + pad + ImGui::CalcTextSize(name.c_str()).x);
        }
        for (const auto& line : scale_lines) {
            width = std::max(width, ImGui::CalcTextSize(line.c_str()).x);
        }
        const float height = line_h * static_cast<float>(rows.size() + scale_lines.size());
        const float margin = 10.0f * ui_scale;
        const ImVec2 p0(scene_rect.x + margin, scene_rect.y + scene_rect.h - margin - height - 2.0f * pad);
        const ImVec2 p1(p0.x + width + 2.0f * pad, p0.y + height + 2.0f * pad);
        dl->AddRectFilled(p0, p1, ImGui::GetColorU32(ImGuiCol_WindowBg), ImGui::GetStyle().WindowRounding);
        dl->AddRect(p0, p1, ImGui::GetColorU32(ImGuiCol_Border), ImGui::GetStyle().WindowRounding);
        float y = p0.y + pad;
        const ImU32 text_color = ImGui::GetColorU32(ImGuiCol_Text);
        for (const auto& [name, color] : rows) {
            const float sy = y + 0.5f * (ImGui::GetTextLineHeight() - swatch);
            dl->AddRectFilled(ImVec2(p0.x + pad, sy), ImVec2(p0.x + pad + swatch, sy + swatch), color);
            dl->AddText(ImVec2(p0.x + 2.0f * pad + swatch, y), text_color, name.c_str());
            y += line_h;
        }
        for (const auto& line : scale_lines) {
            dl->AddText(ImVec2(p0.x + pad, y), ImGui::GetColorU32(ImGuiCol_TextDisabled), line.c_str());
            y += line_h;
        }
    }
    dl->PopClipRect();
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
    ImGui::SetNextWindowPos(ImVec2(scene_rect.x + margin, scene_rect.y + scene_rect.h - size.y - margin),
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
