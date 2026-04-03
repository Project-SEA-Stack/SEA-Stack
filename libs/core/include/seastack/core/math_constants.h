/*********************************************************************
 * @file  math_constants.h
 * @brief Project-wide mathematical constants.
 *
 * Prefer the seastack::kPi / kPiOver2 constexpr constants in new code.
 * They live in namespace seastack to avoid collisions with identically
 * named locals inside Eigen and other third-party headers.
 *
 * The M_PI / M_PI_2 macros are retained for compatibility with
 * third-party headers that expect them.
 *********************************************************************/

#ifndef SEASTACK_CORE_MATH_CONSTANTS_H
#define SEASTACK_CORE_MATH_CONSTANTS_H

namespace seastack {

inline constexpr double kPi      = 3.14159265358979323846;
inline constexpr double kPiOver2 = 1.57079632679489661923;
inline constexpr double kTwoPi   = 2.0 * kPi;

/// Radians to degrees (180 / pi)
inline constexpr double kRadToDeg = 180.0 / kPi;

}  // namespace seastack

// Retain M_PI / M_PI_2 macros for third-party compatibility.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#endif  // SEASTACK_CORE_MATH_CONSTANTS_H
