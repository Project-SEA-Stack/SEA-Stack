#pragma once

// Viewer preferences remembered between runs (theme, water grid, panel
// layout, force display). Purely visual: never affects the simulation.
//
// Stored per user as plain "key = value" lines; '#' starts a comment.
// Unknown keys and invalid values are ignored (the default is kept), so an
// old or hand-edited file never stops the viewer from starting.

#include <seastack/viz_core/visual_theme.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace seastack::viz {

struct ViewerPreferences {
    VisualTheme theme = VisualTheme::kDark;
    bool water_wireframe = true;
    int water_grid = 64;           ///< About water_grid^2 vertices: 32, 64, 96 or 128
    int update_hz = 30;            ///< Water update rate [Hz], 10-60
    bool smooth_slow_motion = true;
    bool mooring_colours = true;   ///< Colour lines by tension
    bool show_inspector = true;
    bool show_moments = false;
    bool show_total_force = false;
    float trail_seconds = 30.0f;   ///< [s of simulated time], 1-600
    // Inspector sections expanded.
    bool open_scene = true;
    bool open_selected = true;
    bool open_loads = true;
    bool open_sea_state = true;
    bool open_connectors = true;
    bool open_changes = true;
    bool open_display = false;

    bool operator==(const ViewerPreferences& o) const {
        return theme == o.theme && water_wireframe == o.water_wireframe && water_grid == o.water_grid &&
               update_hz == o.update_hz && smooth_slow_motion == o.smooth_slow_motion &&
               mooring_colours == o.mooring_colours && show_inspector == o.show_inspector &&
               show_moments == o.show_moments && show_total_force == o.show_total_force &&
               trail_seconds == o.trail_seconds && open_scene == o.open_scene &&
               open_selected == o.open_selected && open_loads == o.open_loads &&
               open_sea_state == o.open_sea_state && open_connectors == o.open_connectors &&
               open_changes == o.open_changes && open_display == o.open_display;
    }
    bool operator!=(const ViewerPreferences& o) const { return !(*this == o); }
};

namespace preferences_detail {

inline std::string Trim(const std::string& s) {
    const size_t b = s.find_first_not_of(" \t\r");
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(" \t\r");
    return s.substr(b, e - b + 1);
}

inline bool ParseBool(const std::string& v, bool* out) {
    if (v == "true" || v == "1") {
        *out = true;
        return true;
    }
    if (v == "false" || v == "0") {
        *out = false;
        return true;
    }
    return false;
}

inline bool ParseInt(const std::string& v, int lo, int hi, int* out) {
    try {
        size_t used = 0;
        const int x = std::stoi(v, &used);
        if (used != v.size() || x < lo || x > hi) return false;
        *out = x;
        return true;
    } catch (...) {
        return false;
    }
}

inline bool ParseFloat(const std::string& v, float lo, float hi, float* out) {
    try {
        size_t used = 0;
        const float x = std::stof(v, &used);
        if (used != v.size() || !(x >= lo && x <= hi)) return false;
        *out = x;
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace preferences_detail

/// Read preferences from text; starts from the defaults.
inline ViewerPreferences ParseViewerPreferences(const std::string& text) {
    using namespace preferences_detail;
    ViewerPreferences p;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = Trim(line.substr(0, eq));
        const std::string value = Trim(line.substr(eq + 1));
        int grid = 0;
        if (key == "theme") {
            ParseVisualTheme(value, &p.theme);
        } else if (key == "water_wireframe") {
            ParseBool(value, &p.water_wireframe);
        } else if (key == "water_grid") {
            if (ParseInt(value, 32, 128, &grid) && (grid == 32 || grid == 64 || grid == 96 || grid == 128)) {
                p.water_grid = grid;
            }
        } else if (key == "update_hz") {
            ParseInt(value, 10, 60, &p.update_hz);
        } else if (key == "smooth_slow_motion") {
            ParseBool(value, &p.smooth_slow_motion);
        } else if (key == "mooring_colours") {
            ParseBool(value, &p.mooring_colours);
        } else if (key == "show_inspector") {
            ParseBool(value, &p.show_inspector);
        } else if (key == "show_moments") {
            ParseBool(value, &p.show_moments);
        } else if (key == "show_total_force") {
            ParseBool(value, &p.show_total_force);
        } else if (key == "trail_seconds") {
            ParseFloat(value, 1.0f, 600.0f, &p.trail_seconds);
        } else if (key == "open_scene") {
            ParseBool(value, &p.open_scene);
        } else if (key == "open_selected") {
            ParseBool(value, &p.open_selected);
        } else if (key == "open_loads") {
            ParseBool(value, &p.open_loads);
        } else if (key == "open_sea_state") {
            ParseBool(value, &p.open_sea_state);
        } else if (key == "open_connectors") {
            ParseBool(value, &p.open_connectors);
        } else if (key == "open_changes") {
            ParseBool(value, &p.open_changes);
        } else if (key == "open_display") {
            ParseBool(value, &p.open_display);
        }
    }
    return p;
}

/// Preferences as text, one "key = value" line each.
inline std::string FormatViewerPreferences(const ViewerPreferences& p) {
    auto b = [](bool v) { return v ? "true" : "false"; };
    std::ostringstream out;
    out << "# SEA-Stack viewer preferences (visual only). Delete this file to reset.\n"
        << "theme = " << VisualThemeName(p.theme) << "\n"
        << "water_wireframe = " << b(p.water_wireframe) << "\n"
        << "water_grid = " << p.water_grid << "\n"
        << "update_hz = " << p.update_hz << "\n"
        << "smooth_slow_motion = " << b(p.smooth_slow_motion) << "\n"
        << "mooring_colours = " << b(p.mooring_colours) << "\n"
        << "show_inspector = " << b(p.show_inspector) << "\n"
        << "show_moments = " << b(p.show_moments) << "\n"
        << "show_total_force = " << b(p.show_total_force) << "\n"
        << "trail_seconds = " << p.trail_seconds << "\n"
        << "open_scene = " << b(p.open_scene) << "\n"
        << "open_selected = " << b(p.open_selected) << "\n"
        << "open_loads = " << b(p.open_loads) << "\n"
        << "open_sea_state = " << b(p.open_sea_state) << "\n"
        << "open_connectors = " << b(p.open_connectors) << "\n"
        << "open_changes = " << b(p.open_changes) << "\n"
        << "open_display = " << b(p.open_display) << "\n";
    return out.str();
}

/// Per-user preferences file: %APPDATA%\SEA-Stack\viewer.ini on Windows,
/// $XDG_CONFIG_HOME/seastack/viewer.ini or ~/.config/seastack/viewer.ini
/// elsewhere. Empty if no such location is known.
inline std::string DefaultViewerPreferencesPath() {
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    if (appdata && *appdata) return std::string(appdata) + "\\SEA-Stack\\viewer.ini";
    return "";
#else
    const char* xdg = std::getenv("XDG_CONFIG_HOME");
    if (xdg && *xdg) return std::string(xdg) + "/seastack/viewer.ini";
    const char* home = std::getenv("HOME");
    if (home && *home) return std::string(home) + "/.config/seastack/viewer.ini";
    return "";
#endif
}

/// Load from `path`; defaults if the file is missing or unreadable.
inline ViewerPreferences LoadViewerPreferences(const std::string& path) {
    std::ifstream in(path);
    if (!in) return ViewerPreferences{};
    std::ostringstream text;
    text << in.rdbuf();
    return ParseViewerPreferences(text.str());
}

/// Write to `path`, creating its folder. Returns false on failure.
inline bool SaveViewerPreferences(const std::string& path, const ViewerPreferences& p) {
    if (path.empty()) return false;
    std::error_code ec;
    const std::filesystem::path target(path);
    if (target.has_parent_path()) std::filesystem::create_directories(target.parent_path(), ec);
    const std::filesystem::path temp = target.string() + ".tmp";
    {
        std::ofstream out(temp, std::ios::trunc);
        if (!out) return false;
        out << FormatViewerPreferences(p);
        if (!out) return false;
    }
    std::filesystem::rename(temp, target, ec);
    return !ec;
}

}  // namespace seastack::viz
