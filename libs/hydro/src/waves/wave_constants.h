/**
 * @file wave_constants.h
 * @brief Internal numerical constants shared across wave computation modules.
 *
 * These thresholds control the transition between asymptotic forms in
 * finite-depth wave theory and the definition of "effectively infinite" depth.
 */

#ifndef SEASTACK_HYDRO_WAVE_CONSTANTS_H
#define SEASTACK_HYDRO_WAVE_CONSTANTS_H

namespace seastack::hydro::wave_detail {

/// Depth values above this [m] are treated as infinite (deep water).
inline constexpr double kEffectiveInfiniteDepth = 1000.0;

/// sinh(kh) below this threshold triggers shallow-water asymptotic forms
/// to avoid division-by-near-zero.
inline constexpr double kShallowWaterKhThreshold = 1e-8;

/// tanh(kh) ~= 1 to machine precision beyond this threshold.
inline constexpr double kDeepWaterKhThreshold = 89.4;

}  // namespace seastack::hydro::wave_detail

#endif  // SEASTACK_HYDRO_WAVE_CONSTANTS_H
