#pragma once

#include <algorithm>
#include <cmath>

namespace seastack::viz {

/// Drag step of a slider and the decimals that show it.
struct SliderStep {
    double step = 0.1;  ///< In the units of the value
    int decimals = 1;   ///< Digits after the decimal point, enough to show one step
};

/// Step for a slider that starts at `start_value`: about 1% of it, rounded to
/// 1, 2 or 5 times a power of ten (Hs = 0.05 m gives 0.0005 m, Tp = 8 s gives
/// 0.1 s), so model-scale and full-scale values are equally usable. A zero or
/// non-finite start value gives 0.1 with one decimal.
inline SliderStep SliderStepFor(double start_value) {
    const double magnitude = std::abs(start_value);
    if (!std::isfinite(magnitude) || magnitude == 0.0) {
        return {};
    }
    const double target = 0.01 * magnitude;
    const double decade = std::pow(10.0, std::floor(std::log10(target)));
    const double mantissa = target / decade;  // [1, 10)
    const double multiple = mantissa < 1.5 ? 1.0 : mantissa < 3.5 ? 2.0 : mantissa < 7.5 ? 5.0 : 10.0;
    SliderStep out;
    out.step = multiple * decade;
    // 1e-9: log10 of an exact power of ten may round just below the integer.
    out.decimals = std::max(0, static_cast<int>(-std::floor(std::log10(out.step) + 1e-9)));
    return out;
}

}  // namespace seastack::viz
