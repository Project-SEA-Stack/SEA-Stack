// Unit tests for water_display_components.h: the components the GPU water
// shader sums must reproduce the visualization elevation
// LinearDirectionalWaveField::GetElevation(pos, t, max_components) (same
// selection, crossfade weights, scale), and the float evaluation the shader
// does must stay close to it at large t.

#include "water_display_components.h"

#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include "test_macros.h"

#include <cmath>
#include <random>
#include <vector>

using seastack::hydro::LinearDirectionalWaveField;
using seastack::hydro::NoWave;
using seastack::hydro::WaveComponent;
using namespace seastack::viz;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;

/// Irregular directional sea with `n` components. Amplitudes are drawn from a
/// few levels so many tie, which exercises the stable selection order.
std::vector<WaveComponent> Sea(int n, double omega_scale, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::vector<WaveComponent> sea;
    for (int i = 0; i < n; ++i) {
        WaveComponent c;
        c.omega = omega_scale * (0.3 + 1.5 * i / n);
        c.k = c.omega * c.omega / kG;  // deep water
        c.direction = -0.6 + 1.2 * uniform(rng);
        c.amplitude = 0.01 * (1 + static_cast<int>(4.0 * uniform(rng)));
        c.phase = 2.0 * kPi * uniform(rng);
        sea.push_back(c);
    }
    return sea;
}

/// Display components evaluated in double: isolates the selection and
/// folding from float round-off in the shader's sum.
double EvaluateInDouble(const std::vector<DisplayWaveComponent>& comps, double x, double y) {
    double eta = 0.0;
    for (const DisplayWaveComponent& c : comps) {
        eta += static_cast<double>(c.amplitude) *
               std::cos(static_cast<double>(c.kx) * x + static_cast<double>(c.ky) * y + static_cast<double>(c.phase));
    }
    return eta;
}

/// Largest |difference| over a grid covering a 320 m x 120 m patch.
struct Errors {
    double selection = 0.0;  ///< Double evaluation vs GetElevation [m]
    double shader = 0.0;     ///< Float evaluation vs GetElevation [m]
};
Errors Compare(const LinearDirectionalWaveField& wave, double t, int max_per_set, double scale) {
    std::vector<DisplayWaveComponent> comps;
    CollectDisplayWaveComponents(wave, t, max_per_set, scale, &comps);
    Errors e;
    for (double x = -160.0; x <= 160.0; x += 23.0) {
        for (double y = -60.0; y <= 60.0; y += 17.0) {
            const double reference = scale * wave.GetElevation(Eigen::Vector3d(x, y, 0.0), t, max_per_set);
            e.selection = std::max(e.selection, std::abs(EvaluateInDouble(comps, x, y) - reference));
            const float shader = EvaluateDisplayElevation(comps, static_cast<float>(x), static_cast<float>(y));
            e.shader = std::max(e.shader, std::abs(static_cast<double>(shader) - reference));
        }
    }
    return e;
}

}  // namespace

int main() {
    TestResults test_results;

    // No discrete components: not available on the GPU path.
    {
        NoWave none;
        std::vector<DisplayWaveComponent> comps(3);
        TEST_ASSERT(!CollectDisplayWaveComponents(none, 1.0, 500, 1.0, &comps), "no components -> false");
        TEST_ASSERT(comps.empty(), "output cleared");
    }

    // Large irregular sea, top-500 selection, at small and large t.
    {
        LinearDirectionalWaveField wave(Sea(800, 1.0, 7), 0.0);
        for (double t : {0.0, 37.3, 5000.0, 86400.0}) {
            std::vector<DisplayWaveComponent> comps;
            TEST_ASSERT(CollectDisplayWaveComponents(wave, t, 500, 1.0, &comps), "components available");
            TEST_ASSERT(comps.size() == 500u, "500 components kept");
            const Errors e = Compare(wave, t, 500, 1.0);
            TEST_NEAR(e.selection, 0.0, 1e-5, "selection and phase folding match GetElevation(pos, t, 500)");
            TEST_NEAR(e.shader, 0.0, 5e-3, "float evaluation within 5 mm");
        }
        std::vector<DisplayWaveComponent> comps;
        CollectDisplayWaveComponents(wave, 86400.0, 500, 1.0, &comps);
        bool reduced = true;
        for (const DisplayWaveComponent& c : comps) {
            reduced = reduced && c.phase >= 0.0f && c.phase < 6.2832f;
        }
        TEST_ASSERT(reduced, "phases reduced to [0, 2 pi) at t = 1 day");
    }

    // All components (max <= 0) and a scale factor.
    {
        LinearDirectionalWaveField wave(Sea(120, 1.0, 3), 0.0);
        const Errors e = Compare(wave, 123.4, 0, 0.37);
        TEST_NEAR(e.selection, 0.0, 1e-6, "all components, scaled");
        std::vector<DisplayWaveComponent> comps;
        CollectDisplayWaveComponents(wave, 0.0, 0, 1.0, &comps);
        TEST_ASSERT(comps.size() == 120u, "max <= 0 keeps all");
    }

    // Live period change: during the crossfade both sets are drawn with their
    // weights; afterwards only the new set (the old weight is 0).
    {
        LinearDirectionalWaveField wave(Sea(600, 1.0, 11), 0.0);
        wave.CrossfadeTo(Sea(600, 1.2, 11), 100.0, 20.0);
        std::vector<DisplayWaveComponent> comps;
        CollectDisplayWaveComponents(wave, 110.0, 500, 1.0, &comps);
        TEST_ASSERT(comps.size() == 1000u, "both sets drawn mid-crossfade");
        const Errors mid = Compare(wave, 110.0, 500, 1.0);
        TEST_NEAR(mid.selection, 0.0, 1e-5, "mid-crossfade matches");
        CollectDisplayWaveComponents(wave, 200.0, 500, 1.0, &comps);
        TEST_ASSERT(comps.size() == 500u, "only the new set after the crossfade");
        const Errors after = Compare(wave, 200.0, 500, 1.0);
        TEST_NEAR(after.selection, 0.0, 1e-5, "after crossfade matches");
    }

    // Crest shading scale: sqrt(2) a for a regular wave.
    {
        std::vector<DisplayWaveComponent> one(1);
        one[0].amplitude = 1.5f;
        TEST_NEAR(DisplayElevationScale(one), std::sqrt(2.0) * 1.5, 1e-6, "regular-wave shading scale");
        TEST_NEAR(DisplayElevationScale({}), 0.0, 0.0, "no components -> 0");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
