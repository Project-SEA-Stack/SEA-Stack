// Unit tests for viewer_preferences.h (Chrono-free, VSG-free): parsing,
// formatting, round trip through a file, and tolerance of bad input.

#include <seastack/viz_core/viewer_preferences.h>

#include "test_macros.h"

#include <filesystem>

using namespace seastack::viz;

int main() {
    TestResults test_results;

    // Empty text gives the defaults.
    {
        const ViewerPreferences p = ParseViewerPreferences("");
        TEST_ASSERT(p == ViewerPreferences{}, "empty text gives defaults");
        TEST_ASSERT(p.water_wireframe, "wireframe on by default");
        TEST_ASSERT(p.water_grid == 64, "grid 64 by default");
    }

    // Format then parse returns the same values.
    {
        ViewerPreferences p;
        p.theme = VisualTheme::kLight;
        p.water_wireframe = false;
        p.water_grid = 96;
        p.update_hz = 45;
        p.smooth_slow_motion = false;
        p.mooring_colours = false;
        p.show_inspector = false;
        p.show_moments = true;
        p.show_total_force = true;
        p.trail_seconds = 12.5f;
        p.open_scene = false;
        p.open_display = true;
        p.open_changes = false;
        const ViewerPreferences q = ParseViewerPreferences(FormatViewerPreferences(p));
        TEST_ASSERT(q == p, "format / parse round trip");
    }

    // Comments, spaces and unknown keys are ignored.
    {
        const ViewerPreferences p = ParseViewerPreferences(
            "# comment\n"
            "  theme =  LIGHT  # trailing comment\r\n"
            "future_key = 3\n"
            "no equals sign\n"
            "water_grid=128\n");
        TEST_ASSERT(p.theme == VisualTheme::kLight, "theme parsed case-insensitively");
        TEST_ASSERT(p.water_grid == 128, "grid without spaces");
    }

    // Invalid values keep the default.
    {
        const ViewerPreferences p = ParseViewerPreferences(
            "water_grid = 50\n"
            "update_hz = 500\n"
            "trail_seconds = abc\n"
            "show_moments = maybe\n"
            "water_wireframe = 0\n"
            "theme = neon\n");
        const ViewerPreferences d;
        TEST_ASSERT(p.water_grid == d.water_grid, "grid not in the option list is ignored");
        TEST_ASSERT(p.update_hz == d.update_hz, "update rate out of range is ignored");
        TEST_NEAR(p.trail_seconds, d.trail_seconds, 0.0, "non-numeric trail length is ignored");
        TEST_ASSERT(p.show_moments == d.show_moments, "non-boolean is ignored");
        TEST_ASSERT(!p.water_wireframe, "0 reads as false");
        TEST_ASSERT(p.theme == d.theme, "unknown theme is ignored");
    }

    // File round trip, creating the folder; a missing file gives defaults.
    {
        const std::filesystem::path dir = std::filesystem::temp_directory_path() / "seastack_test_prefs";
        std::filesystem::remove_all(dir);
        const std::string path = (dir / "sub" / "viewer.ini").string();
        TEST_ASSERT(LoadViewerPreferences(path) == ViewerPreferences{}, "missing file gives defaults");
        ViewerPreferences p;
        p.water_grid = 32;
        p.open_loads = false;
        TEST_ASSERT(SaveViewerPreferences(path, p), "save creates the folder");
        TEST_ASSERT(LoadViewerPreferences(path) == p, "load returns what was saved");
        p.water_grid = 128;
        TEST_ASSERT(SaveViewerPreferences(path, p), "save overwrites");
        TEST_ASSERT(LoadViewerPreferences(path).water_grid == 128, "overwritten value read back");
        TEST_ASSERT(!SaveViewerPreferences("", p), "empty path is refused");
        std::filesystem::remove_all(dir);
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
