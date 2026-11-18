// =============================================================================
// SEA-Stack VSG Themes
// =============================================================================
// Scene and panel settings for the dark / light viewer themes. Everything here
// can be switched at runtime (background, sky, light levels, ImGui style).
// =============================================================================
#pragma once

#include <seastack/viz_core/visual_theme.h>

#include <array>

namespace seastack::viz {

/// Runtime-switchable scene settings for one theme.
struct SceneTheme {
    std::array<float, 3> background_srgb;  ///< Clear colour behind the scene (sRGB, 0-1)
    bool show_sky;                         ///< Chrono viewer only: show its cubemap skybox
    float ambient_intensity;               ///< Ambient light [-]
    float fill_intensity;                  ///< Fill light, opposite the key [-]
    float rim_intensity;                   ///< Rim light from behind / above [-]
    /// SEA-Stack viewer: soft light straight down in the sky colour and
    /// straight up in the water colour, so faces turned from the key light
    /// keep their shape (a cheap stand-in for image-based lighting) [-].
    float sky_fill_intensity;
    float sea_bounce_intensity;

    // SEA-Stack viewer: procedural sky, drawn as the backdrop and reflected by
    // the water (sRGB, 0-1).
    std::array<float, 3> sky_zenith_srgb;
    std::array<float, 3> sky_horizon_srgb;
    std::array<float, 3> sky_ground_srgb;  ///< Below the horizon: distant sea and ground
    float sky_sun_haze;                    ///< Glow around the sun [-]; 0 for a studio backdrop
    /// Water body: deep (troughs, looking straight down) and scatter (crests).
    std::array<float, 3> water_deep_srgb;
    std::array<float, 3> water_scatter_srgb;
};

SceneTheme GetSceneTheme(VisualTheme theme);

/// Apply the ImGui panel style for `theme`, with sizes multiplied by
/// `ui_scale` (display scale, e.g. 1.5 at 150 %). Call inside an ImGui frame.
void ApplyImGuiTheme(VisualTheme theme, float ui_scale = 1.0f);

/// UI scale [-] from a desktop DPI setting [px/in]. 96 DPI is 1, so an
/// Xft.dpi of 192 is 2. Values outside 48..384, including non-finite
/// values, return 1 so a bad resource cannot resize the window.
inline float UiScaleFromDpi(float dpi) {
    if (!(dpi >= 48.0f && dpi <= 384.0f)) {
        return 1.0f;
    }
    return dpi / 96.0f;
}

}  // namespace seastack::viz
