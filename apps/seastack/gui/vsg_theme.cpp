// =============================================================================
// SEA-Stack VSG Themes - Implementation
// =============================================================================
#include "vsg_theme.h"

#include <vsgImGui/imgui.h>

namespace seastack::viz {

SceneTheme GetSceneTheme(VisualTheme theme) {
    if (theme == VisualTheme::kLight) {
        // Skybox backdrop; light-grey clear colour only shows if the skybox
        // texture failed to load.
        return SceneTheme{{0.78f, 0.82f, 0.87f}, true, 0.22f, 0.40f, 0.20f};
    }
    // Studio Dark: neutral dark slate so bodies, water and mooring colours
    // stand out; a stronger rim light separates silhouettes from the background.
    return SceneTheme{{0.10f, 0.115f, 0.14f}, false, 0.18f, 0.38f, 0.45f};
}

namespace {

/// Shared sizes and rounding for both themes.
void ApplyModernMetrics(ImGuiStyle& style) {
    style.WindowPadding = ImVec2(12.0f, 10.0f);
    style.FramePadding = ImVec2(8.0f, 5.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
    style.WindowRounding = 8.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 6.0f;
    style.PopupRounding = 6.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding = 6.0f;
    style.TabRounding = 6.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;
    style.GrabMinSize = 12.0f;
    style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
}

void ApplyAccent(ImVec4* c, const ImVec4& accent) {
    auto with_alpha = [](ImVec4 v, float a) { return ImVec4(v.x, v.y, v.z, a); };
    c[ImGuiCol_CheckMark] = accent;
    c[ImGuiCol_SliderGrab] = with_alpha(accent, 0.85f);
    c[ImGuiCol_SliderGrabActive] = accent;
    c[ImGuiCol_ButtonHovered] = with_alpha(accent, 0.55f);
    c[ImGuiCol_ButtonActive] = with_alpha(accent, 0.80f);
    c[ImGuiCol_HeaderHovered] = with_alpha(accent, 0.35f);
    c[ImGuiCol_HeaderActive] = with_alpha(accent, 0.55f);
    c[ImGuiCol_FrameBgActive] = with_alpha(accent, 0.35f);
    c[ImGuiCol_TextSelectedBg] = with_alpha(accent, 0.35f);
    c[ImGuiCol_ResizeGripHovered] = with_alpha(accent, 0.60f);
    c[ImGuiCol_ResizeGripActive] = accent;
}

}  // namespace

void ApplyImGuiTheme(VisualTheme theme) {
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* c = style.Colors;

    if (theme == VisualTheme::kLight) {
        ImGui::StyleColorsLight(&style);
        ApplyModernMetrics(style);
        c[ImGuiCol_Text] = ImVec4(0.10f, 0.12f, 0.15f, 1.00f);
        c[ImGuiCol_TextDisabled] = ImVec4(0.42f, 0.45f, 0.50f, 1.00f);
        c[ImGuiCol_WindowBg] = ImVec4(0.97f, 0.97f, 0.98f, 0.94f);
        c[ImGuiCol_PopupBg] = ImVec4(0.99f, 0.99f, 1.00f, 0.98f);
        c[ImGuiCol_Border] = ImVec4(0.00f, 0.00f, 0.00f, 0.10f);
        c[ImGuiCol_FrameBg] = ImVec4(0.89f, 0.90f, 0.92f, 1.00f);
        c[ImGuiCol_FrameBgHovered] = ImVec4(0.84f, 0.86f, 0.89f, 1.00f);
        c[ImGuiCol_Button] = ImVec4(0.88f, 0.89f, 0.91f, 1.00f);
        c[ImGuiCol_Header] = ImVec4(0.90f, 0.91f, 0.93f, 1.00f);
        c[ImGuiCol_TitleBg] = ImVec4(0.92f, 0.93f, 0.95f, 1.00f);
        c[ImGuiCol_TitleBgActive] = ImVec4(0.88f, 0.90f, 0.93f, 1.00f);
        ApplyAccent(c, ImVec4(0.13f, 0.45f, 0.80f, 1.00f));
        return;
    }

    ImGui::StyleColorsDark(&style);
    ApplyModernMetrics(style);
    c[ImGuiCol_Text] = ImVec4(0.90f, 0.92f, 0.95f, 1.00f);
    c[ImGuiCol_TextDisabled] = ImVec4(0.55f, 0.58f, 0.63f, 1.00f);
    c[ImGuiCol_WindowBg] = ImVec4(0.09f, 0.10f, 0.12f, 0.92f);
    c[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.11f, 0.13f, 0.98f);
    c[ImGuiCol_Border] = ImVec4(1.00f, 1.00f, 1.00f, 0.08f);
    c[ImGuiCol_FrameBg] = ImVec4(0.15f, 0.17f, 0.20f, 1.00f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.19f, 0.21f, 0.25f, 1.00f);
    c[ImGuiCol_Button] = ImVec4(0.17f, 0.19f, 0.23f, 1.00f);
    c[ImGuiCol_Header] = ImVec4(0.15f, 0.17f, 0.20f, 1.00f);
    c[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.09f, 0.11f, 1.00f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.11f, 0.12f, 0.15f, 1.00f);
    c[ImGuiCol_Separator] = ImVec4(1.00f, 1.00f, 1.00f, 0.08f);
    ApplyAccent(c, ImVec4(0.25f, 0.62f, 0.95f, 1.00f));
}

}  // namespace seastack::viz
