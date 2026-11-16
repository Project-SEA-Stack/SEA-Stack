#pragma once

// Water surface "Elevation" style: colour by free-surface elevation eta with
// optional contour lines, and the legend that goes with it. Visual only.
//
// The legend spans eta = -range .. +range, mapped to t = 0 .. 1 of the
// chosen colour map; contours are every `contour` m.

#include <algorithm>
#include <array>
#include <cmath>

namespace seastack::viz {

/// How the water surface is drawn (viewers with a water shader).
enum class WaterStyle {
    kElevation,  ///< Colour by eta: for reading the sea
    kShaded,  ///< Reflections, glint, deep / crest colours: lit water look
};

/// Colour maps for the elevation style, all perceptually ordered.
enum class WaterPalette {
    kDeep,     ///< Navy troughs to pale green-yellow crests (after cmocean "deep")
    kViridis,  ///< Purple troughs through teal to yellow crests (matplotlib "viridis")
    kBalance,  ///< Diverging: blue troughs, near-white still water, red crests (after cmocean "balance")
};

inline constexpr int kColormapStops = 5;
using ColormapStops = std::array<std::array<float, 3>, kColormapStops>;

/// Colour map stops, sRGB 0-1, evenly spaced from t = 0 (eta = -range) to
/// t = 1 (eta = +range). Five-stop approximations of the published maps; the
/// shader and the legend both use these.
inline const ColormapStops& PaletteStops(WaterPalette palette) {
    // The darkest end is lifted from cmocean's near-black so troughs stay
    // visible against the dark theme.
    static constexpr ColormapStops kDeep = {{
        {0.19f, 0.17f, 0.36f},
        {0.25f, 0.33f, 0.58f},
        {0.25f, 0.56f, 0.64f},
        {0.42f, 0.78f, 0.65f},
        {0.86f, 0.96f, 0.74f},
    }};
    static constexpr ColormapStops kViridis = {{
        {0.267f, 0.005f, 0.329f},
        {0.229f, 0.322f, 0.546f},
        {0.128f, 0.567f, 0.551f},
        {0.369f, 0.789f, 0.383f},
        {0.993f, 0.906f, 0.144f},
    }};
    static constexpr ColormapStops kBalance = {{
        {0.10f, 0.15f, 0.40f},
        {0.28f, 0.50f, 0.80f},
        {0.92f, 0.91f, 0.90f},
        {0.80f, 0.36f, 0.26f},
        {0.45f, 0.08f, 0.12f},
    }};
    switch (palette) {
        case WaterPalette::kViridis:
            return kViridis;
        case WaterPalette::kBalance:
            return kBalance;
        case WaterPalette::kDeep:
        default:
            return kDeep;
    }
}

/// sRGB colour at t in [0, 1] (clamped): piecewise linear between the stops.
inline std::array<float, 3> PaletteColor(WaterPalette palette, float t) {
    const ColormapStops& stops = PaletteStops(palette);
    constexpr int kSegments = kColormapStops - 1;
    const float x = std::clamp(t, 0.0f, 1.0f) * kSegments;
    const int i = std::min(static_cast<int>(x), kSegments - 1);
    const float f = x - static_cast<float>(i);
    std::array<float, 3> c{};
    for (int k = 0; k < 3; ++k) {
        c[k] = stops[i][k] + f * (stops[i + 1][k] - stops[i][k]);
    }
    return c;
}

/// The 1-2-5 number (x 10^n) nearest to x > 0 on a log scale; 0 otherwise.
inline double NiceNumber(double x) {
    if (!(x > 0.0) || !std::isfinite(x)) {
        return 0.0;
    }
    const double decade = std::pow(10.0, std::floor(std::log10(x)));
    const double m = x / decade;  // [1, 10)
    // Geometric midpoints between 1, 2, 5 and 10.
    if (m < std::sqrt(2.0)) return decade;
    if (m < std::sqrt(10.0)) return 2.0 * decade;
    if (m < std::sqrt(50.0)) return 5.0 * decade;
    return 10.0 * decade;
}

/// The smallest of 1, 2, 2.5, 5 (x 10^n) >= x > 0; 0 otherwise.
inline double NiceCeil(double x) {
    if (!(x > 0.0) || !std::isfinite(x)) {
        return 0.0;
    }
    const double decade = std::pow(10.0, std::floor(std::log10(x)));
    const double m = x / decade;
    constexpr double kTolerance = 1e-9;
    if (m <= 1.0 + kTolerance) return decade;
    if (m <= 2.0 + kTolerance) return 2.0 * decade;
    if (m <= 2.5 + kTolerance) return 2.5 * decade;
    if (m <= 5.0 + kTolerance) return 5.0 * decade;
    return 10.0 * decade;
}

struct ElevationLegend {
    double range_m = 0.0;    ///< Colour map spans eta = -range .. +range [m]
    double contour_m = 0.0;  ///< Contour interval [m]; 0 = none
};

/// Legend for a sea with elevation scale `scale_m` = sqrt(2 sum a_i^2) [m]
/// (Hs / 2 for a spectral sea, sqrt(2) a for a regular wave) and largest
/// possible |eta| `peak_m` = sum |a_i| [m]. The range is the round number at
/// or above min(3 sigma, peak), with sigma = scale / 2 the standard deviation
/// of eta: 3 sigma = 0.75 Hs for a spectral sea, and the amplitude itself for
/// a regular wave. Higher crests saturate the colour map. About four or five
/// contours each side of zero.
inline ElevationLegend ComputeElevationLegend(double scale_m, double peak_m) {
    ElevationLegend legend;
    legend.range_m = NiceCeil(std::min(1.5 * scale_m, peak_m));
    legend.contour_m = NiceNumber(legend.range_m / 4.0);
    return legend;
}

}  // namespace seastack::viz
