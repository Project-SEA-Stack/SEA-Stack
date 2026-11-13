#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace seastack::viz {

/// Overall look of the 3D viewer and its panels. Switchable at runtime from
/// the GUI; the initial value comes from simulation.visualization.theme.
/// Purely visual: never affects the simulation.
enum class VisualTheme {
    kDark,   ///< Studio Dark: dark background, no skybox, dark panels
    kLight,  ///< Light: skybox, light panels
};

inline const char* VisualThemeName(VisualTheme theme) {
    return theme == VisualTheme::kLight ? "light" : "dark";
}

/// Parse "dark" / "light" (case-insensitive). Returns false if unrecognised.
inline bool ParseVisualTheme(std::string text, VisualTheme* theme) {
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (text == "dark") {
        *theme = VisualTheme::kDark;
        return true;
    }
    if (text == "light") {
        *theme = VisualTheme::kLight;
        return true;
    }
    return false;
}

}  // namespace seastack::viz
