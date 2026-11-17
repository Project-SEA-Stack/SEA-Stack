// SliderStepFor: sea-state slider steps follow the scale of the start value,
// so a model-scale sea (Hs of a few centimetres) can be set as finely as a
// full-scale one.

#include <seastack/viz_core/slider_step.h>

#include "test_macros.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

using seastack::viz::SliderStep;
using seastack::viz::SliderStepFor;

namespace {

std::string Shown(double value, const SliderStep& s) {
    char format[16];
    char text[64];
    std::snprintf(format, sizeof(format), "%%.%df", s.decimals);
    std::snprintf(text, sizeof(text), format, value);
    return text;
}

}  // namespace

int main() {
    TestResults test_results;

    // Model scale: Hs = 0.05 m must step in 0.0005 m or finer and show it.
    const SliderStep tank = SliderStepFor(0.05);
    TEST_ASSERT(tank.step <= 0.0005 + 1e-15, "Hs = 0.05 m: step 0.0005 m or finer");
    TEST_NEAR(tank.step, 0.0005, 1e-15, "Hs = 0.05 m: step 0.0005 m");
    TEST_ASSERT(Shown(0.0505, tank) == "0.0505", "Hs = 0.05 m: one step is visible (" + Shown(0.0505, tank) + ")");

    // Full scale keeps a usable step: Tp = 8 s -> 0.1 s (as before), Hs = 2 m -> 0.02 m.
    TEST_NEAR(SliderStepFor(8.0).step, 0.1, 1e-15, "Tp = 8 s: 0.1 s");
    TEST_ASSERT(SliderStepFor(8.0).decimals == 1, "Tp = 8 s: one decimal");
    TEST_NEAR(SliderStepFor(2.0).step, 0.02, 1e-15, "Hs = 2 m: 0.02 m");
    TEST_ASSERT(SliderStepFor(2.0).decimals == 2, "Hs = 2 m: two decimals");
    TEST_NEAR(SliderStepFor(10.0).step, 0.1, 1e-15, "T = 10 s: exact power of ten");
    TEST_ASSERT(SliderStepFor(10.0).decimals == 1, "T = 10 s: one decimal");
    TEST_NEAR(SliderStepFor(1.2).step, 0.01, 1e-15, "Tp = 1.2 s (tank): 0.01 s");
    TEST_NEAR(SliderStepFor(400.0).step, 5.0, 1e-12, "400: 5");
    TEST_ASSERT(SliderStepFor(400.0).decimals == 0, "400: no decimals");

    // Every step is 1, 2 or 5 times a power of ten, within 0.5x to 2x of 1%.
    bool nice = true;
    for (double v = 1e-4; v < 1e4; v *= 1.37) {
        const SliderStep s = SliderStepFor(v);
        const double mantissa = s.step / std::pow(10.0, std::floor(std::log10(s.step) + 1e-9));
        const bool one_two_five = std::abs(mantissa - 1.0) < 1e-9 || std::abs(mantissa - 2.0) < 1e-9 ||
                                  std::abs(mantissa - 5.0) < 1e-9;
        nice = nice && one_two_five && s.step >= 0.005 * v && s.step <= 0.02 * v &&
               Shown(v + s.step, s) != Shown(v, s);
    }
    TEST_ASSERT(nice, "steps are 1-2-5 x 10^k, about 1% of the value, and visible");

    // Fallback.
    TEST_NEAR(SliderStepFor(0.0).step, 0.1, 0.0, "zero start value: 0.1");
    TEST_NEAR(SliderStepFor(std::numeric_limits<double>::quiet_NaN()).step, 0.1, 0.0, "NaN: 0.1");
    TEST_NEAR(SliderStepFor(-3.0).step, 0.02, 1e-15, "negative start value uses its magnitude");

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
