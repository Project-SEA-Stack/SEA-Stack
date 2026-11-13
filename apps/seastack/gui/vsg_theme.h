// =============================================================================
// SEA-Stack VSG Themes
// =============================================================================
// Scene and panel settings for the dark / light viewer themes. Everything here
// can be switched at runtime (background, skybox, light levels, ImGui style).
// =============================================================================
#pragma once

#include <seastack/viz_core/visual_theme.h>

#include <array>

namespace seastack::viz {

/// Runtime-switchable scene settings for one theme.
struct SceneTheme {
    std::array<float, 3> background_srgb;  ///< Clear colour when no skybox (sRGB, 0-1)
    bool show_sky;                         ///< Show the skybox
    float ambient_intensity;               ///< Ambient light [-]
    float fill_intensity;                  ///< Fill light, opposite the key [-]
    float rim_intensity;                   ///< Rim light from behind / above [-]
};

SceneTheme GetSceneTheme(VisualTheme theme);

/// Apply the ImGui panel style for `theme`. Call inside an ImGui frame.
void ApplyImGuiTheme(VisualTheme theme);

}  // namespace seastack::viz
