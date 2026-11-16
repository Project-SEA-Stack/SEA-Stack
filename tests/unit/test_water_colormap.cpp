// Unit tests for water_colormap.h (Chrono-free, VSG-free): 1-2-5 rounding,
// the elevation legend, and the colour maps' ends and lightness order.

#include <seastack/viz_core/water_colormap.h>

#include <initializer_list>

#include "test_macros.h"

using namespace seastack::viz;

namespace {

/// Relative luminance of an sRGB colour (Rec. 709 weights on linear values).
double Luminance(const std::array<float, 3>& srgb) {
    auto lin = [](double v) { return v <= 0.04045 ? v / 12.92 : std::pow((v + 0.055) / 1.055, 2.4); };
    return 0.2126 * lin(srgb[0]) + 0.7152 * lin(srgb[1]) + 0.0722 * lin(srgb[2]);
}

}  // namespace

int main() {
    TestResults test_results;

    // 1-2-5 rounding.
    {
        TEST_NEAR(NiceNumber(1.3), 1.0, 1e-12, "1.3 -> 1");
        TEST_NEAR(NiceNumber(1.5), 2.0, 1e-12, "1.5 -> 2");
        TEST_NEAR(NiceNumber(4.0), 5.0, 1e-12, "4 -> 5");
        TEST_NEAR(NiceNumber(8.0), 10.0, 1e-12, "8 -> 10");
        TEST_NEAR(NiceNumber(0.03), 0.02, 1e-12, "0.03 -> 0.02");
        TEST_NEAR(NiceNumber(0.0), 0.0, 0.0, "0 -> 0");
        TEST_NEAR(NiceCeil(2.0), 2.0, 1e-12, "2 stays 2");
        TEST_NEAR(NiceCeil(2.1), 2.5, 1e-12, "2.1 -> 2.5");
        TEST_NEAR(NiceCeil(2.6), 5.0, 1e-12, "2.6 -> 5");
        TEST_NEAR(NiceCeil(0.07), 0.1, 1e-12, "0.07 -> 0.1");
        TEST_NEAR(NiceCeil(-1.0), 0.0, 0.0, "negative -> 0");
    }

    // Legend: range about 3 sigma (or the amplitude of a regular wave),
    // contours every quarter of the range or so.
    {
        const ElevationLegend full = ComputeElevationLegend(1.5, 40.0);  // Hs = 3 m
        TEST_NEAR(full.range_m, 2.5, 1e-12, "Hs 3 m: range +-2.5 m");
        TEST_NEAR(full.contour_m, 0.5, 1e-12, "Hs 3 m: 0.5 m contours");
        const ElevationLegend lab = ComputeElevationLegend(0.015, 0.4);  // Hs = 3 cm
        TEST_NEAR(lab.range_m, 0.025, 1e-12, "Hs 3 cm: range +-2.5 cm (scale invariant)");
        TEST_NEAR(lab.contour_m, 0.005, 1e-12, "Hs 3 cm: 5 mm contours");
        const double a = 1.0;  // regular wave
        const ElevationLegend regular = ComputeElevationLegend(std::sqrt(2.0) * a, a);
        TEST_NEAR(regular.range_m, 1.0, 1e-12, "regular wave: range = amplitude");
        const ElevationLegend still = ComputeElevationLegend(0.0, 0.0);
        TEST_ASSERT(still.range_m == 0.0 && still.contour_m == 0.0, "still water: no legend");
    }

    // Colour maps: ends are the first and last stops; t is clamped; the
    // sequential maps rise in lightness (readable in grey scale), the
    // diverging one is lightest at still water.
    for (WaterPalette palette : {WaterPalette::kDeep, WaterPalette::kViridis, WaterPalette::kBalance}) {
        const ColormapStops& stops = PaletteStops(palette);
        const auto lo = PaletteColor(palette, 0.0f);
        const auto hi = PaletteColor(palette, 1.0f);
        TEST_NEAR(lo[2], stops.front()[2], 1e-6, "t = 0 is the first stop");
        TEST_NEAR(hi[1], stops.back()[1], 1e-6, "t = 1 is the last stop");
        TEST_NEAR(PaletteColor(palette, -1.0f)[0], lo[0], 1e-6, "t clamped below");
        TEST_NEAR(PaletteColor(palette, 2.0f)[0], hi[0], 1e-6, "t clamped above");
        bool ordered = true;
        double previous = -1.0;
        for (int i = 0; i <= 100; ++i) {
            const float t = static_cast<float>(i) / 100.0f;
            const double y = Luminance(PaletteColor(palette, t));
            const bool rising = (palette != WaterPalette::kBalance) || t <= 0.5f;
            ordered = ordered && (i == 0 || (rising ? y > previous : y < previous));
            previous = y;
        }
        TEST_ASSERT(ordered, "lightness ordered with elevation");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
