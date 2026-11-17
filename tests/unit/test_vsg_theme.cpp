// ApplyImGuiTheme: switching themes at a display scale above 100 % must not
// compound the panel sizes (ScaleAllSizes multiplies the current style).

#include "vsg_theme.h"

#include <vsgImGui/imgui.h>

#include "test_macros.h"

#include <vector>

using seastack::viz::ApplyImGuiTheme;
using seastack::viz::VisualTheme;

namespace {

/// The style sizes that ImGuiStyle::ScaleAllSizes multiplies [px].
std::vector<float> Sizes(const ImGuiStyle& s) {
    const ImVec2 pairs[] = {s.WindowPadding,     s.WindowMinSize,        s.FramePadding,
                            s.ItemSpacing,       s.ItemInnerSpacing,     s.CellPadding,
                            s.TouchExtraPadding, s.SeparatorTextPadding, s.DisplayWindowPadding,
                            s.DisplaySafeAreaPadding};
    std::vector<float> sizes = {s.WindowRounding,    s.ChildRounding,     s.PopupRounding,
                                s.FrameRounding,     s.IndentSpacing,     s.ColumnsMinSpacing,
                                s.ScrollbarSize,     s.ScrollbarRounding, s.GrabMinSize,
                                s.GrabRounding,      s.LogSliderDeadzone, s.TabRounding,
                                s.TabMinWidthForCloseButton, s.TabBarOverlineSize, s.MouseCursorScale};
    for (const ImVec2& p : pairs) {
        sizes.push_back(p.x);
        sizes.push_back(p.y);
    }
    return sizes;
}

}  // namespace

int main() {
    TestResults test_results;
    ImGui::CreateContext();
    const ImGuiStyle& style = ImGui::GetStyle();

    ApplyImGuiTheme(VisualTheme::kDark, 1.5f);
    const std::vector<float> dark = Sizes(style);
    const float scrollbar = style.ScrollbarSize;
    const ImVec2 cell_padding = style.CellPadding;

    for (int i = 0; i < 4; ++i) {
        ApplyImGuiTheme(VisualTheme::kLight, 1.5f);
        ApplyImGuiTheme(VisualTheme::kDark, 1.5f);
    }
    TEST_ASSERT(Sizes(style) == dark, "dark style sizes identical after four light/dark switches at 150 %");
    TEST_NEAR(style.ScrollbarSize, scrollbar, 0.0, "scrollbar width does not grow");
    TEST_NEAR(style.CellPadding.x, cell_padding.x, 0.0, "cell padding does not grow");

    const ImGuiStyle defaults;
    TEST_NEAR(scrollbar, 1.5f * defaults.ScrollbarSize, 1e-5, "sizes scaled once by the display scale");

    ApplyImGuiTheme(VisualTheme::kLight, 1.5f);
    const std::vector<float> light = Sizes(style);
    ApplyImGuiTheme(VisualTheme::kLight, 1.5f);
    TEST_ASSERT(Sizes(style) == light, "applying the light theme twice gives identical sizes");

    ImGui::DestroyContext();
    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
