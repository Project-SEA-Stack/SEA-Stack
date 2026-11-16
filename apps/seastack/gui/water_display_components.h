// =============================================================================
// SEA-Stack Water Display Components
// =============================================================================
// The linear wave components the GPU water shader sums, prepared on the CPU
// each frame so the GPU only evaluates
//   eta(x, y) = sum_i a_i cos(kx_i x + ky_i y + psi_i)
// in float. Everything time dependent is folded in here, in double:
//   a_i   = scale * w_l(t) * A_i   (crossfade weight w_l of the component's set,
//                                   scale = visual scale x excitation ramp x e(t))
//   psi_i = (phi_i - omega_i t) mod 2 pi
// Reducing the phase in double keeps float precision at large t.
//
// The selection matches LinearDirectionalWaveField::GetElevationForVisualization:
// per component set, the max_per_set largest amplitudes (stable order for
// ties); sets with zero weight are skipped. Visual only.
// =============================================================================
#pragma once

#include <seastack/hydro/waves/wave_base.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

namespace seastack::viz {

/// One component as uploaded to the GPU (float, world frame, SI).
struct DisplayWaveComponent {
    float amplitude = 0.0f;  ///< a_i [m]
    float kx = 0.0f;         ///< [rad/m]
    float ky = 0.0f;         ///< [rad/m]
    float phase = 0.0f;      ///< psi_i in [0, 2 pi) [rad]
};

/// Fill `out` with the displayed components at time `t` [s]. `scale` [-]
/// multiplies every amplitude; max_per_set <= 0 keeps all. Returns false
/// (and clears out) if the wave model has no discrete components, e.g. a
/// tabulated elevation record.
inline bool CollectDisplayWaveComponents(const seastack::hydro::WaveBase& wave, double t, int max_per_set,
                                         double scale, std::vector<DisplayWaveComponent>* out) {
    out->clear();
    const std::vector<seastack::hydro::WaveComponent>* components = wave.GetWaveComponents();
    if (!components) {
        return false;
    }
    constexpr double kTurn = 6.283185307179586476925286766559;

    struct Set {
        std::size_t begin, end;
        double weight;
    };
    std::vector<Set> sets;
    const auto& layers = wave.GetWaveLayers();
    if (layers.empty()) {
        sets.push_back({0, components->size(), 1.0});
    } else {
        for (const auto& layer : layers) {
            sets.push_back({layer.begin, layer.end, layer.weight.Value(t)});
        }
    }

    std::vector<std::size_t> order;
    for (const Set& set : sets) {
        if (set.weight == 0.0 || set.end <= set.begin) {
            continue;
        }
        order.resize(set.end - set.begin);
        std::iota(order.begin(), order.end(), set.begin);
        std::stable_sort(order.begin(), order.end(), [components](std::size_t a, std::size_t b) {
            return (*components)[a].amplitude > (*components)[b].amplitude;
        });
        const std::size_t n = (max_per_set <= 0) ? order.size()
                                                 : std::min(order.size(), static_cast<std::size_t>(max_per_set));
        for (std::size_t j = 0; j < n; ++j) {
            const seastack::hydro::WaveComponent& c = (*components)[order[j]];
            double psi = std::fmod(c.phase - c.omega * t, kTurn);
            if (psi < 0.0) psi += kTurn;
            DisplayWaveComponent d;
            d.amplitude = static_cast<float>(scale * set.weight * c.amplitude);
            d.kx = static_cast<float>(c.k * std::cos(c.direction));
            d.ky = static_cast<float>(c.k * std::sin(c.direction));
            d.phase = static_cast<float>(psi);
            out->push_back(d);
        }
    }
    return true;
}

/// eta(x, y) [m] from display components, in float as the shader computes it.
inline float EvaluateDisplayElevation(const std::vector<DisplayWaveComponent>& components, float x, float y) {
    float eta = 0.0f;
    for (const DisplayWaveComponent& c : components) {
        eta += c.amplitude * std::cos(c.kx * x + c.ky * y + c.phase);
    }
    return eta;
}

/// Elevation scale for crest / trough shading [m]: sqrt(2 sum a_i^2), which
/// is Hs / 2 for a spectral sea and sqrt(2) a for a regular wave.
inline float DisplayElevationScale(const std::vector<DisplayWaveComponent>& components) {
    double sum = 0.0;
    for (const DisplayWaveComponent& c : components) {
        sum += static_cast<double>(c.amplitude) * c.amplitude;
    }
    return static_cast<float>(std::sqrt(2.0 * sum));
}

}  // namespace seastack::viz
