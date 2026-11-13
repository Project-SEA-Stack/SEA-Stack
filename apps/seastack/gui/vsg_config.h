// SEA-Stack VSG Configuration Constants
#pragma once

#include <seastack/core/math_constants.h>

#include <cmath>

namespace seastack::viz {
namespace vsg_config {

// Painted Metal (Industrial Yellow)
inline constexpr float kPaintedMetalR = 0.92f;
inline constexpr float kPaintedMetalG = 0.72f;
inline constexpr float kPaintedMetalB = 0.12f;

inline constexpr float kSpecular = 0.25f;
inline constexpr float kSpecularExponent = 64.0f;
inline constexpr float kRoughness = 0.55f;  // satin painted steel
inline constexpr float kMetallic = 0.0f;    // paint is a dielectric

// Water Surface
inline constexpr float kWaterR = 0.01f;
inline constexpr float kWaterG = 0.20f;
inline constexpr float kWaterB = 0.35f;
inline constexpr float kWaterOpacity = 0.55f;
inline constexpr float kWaterSpecular = 0.6f;
inline constexpr float kWaterRoughness = 0.05f;
inline constexpr float kWaterMetallic = 0.0f;

// Water Plane (Static)
inline constexpr double kWaterPlaneSize = 100.0;
inline constexpr double kWaterPlaneThickness = 0.02;
inline constexpr double kWaterPlaneZ = -0.05;

// Water Grid (Animated)
inline constexpr double kWaterGridSize = 100.0;
inline constexpr int kWaterGridResolution = 100;
inline constexpr double kWaterGridSpacing = kWaterGridSize / (kWaterGridResolution - 1);

// Window and camera
inline constexpr int kWindowWidth = 1280;
inline constexpr int kWindowHeight = 720;
inline constexpr double kCameraFovDeg = 40.0;  ///< Vertical field of view [deg]

// Lighting (Key + Fill)
inline constexpr float kKeyIntensity = 1.0f;
inline constexpr double kKeyAzimuth = 0.4 * M_PI;
inline constexpr double kKeyElevation = M_PI / 5.0;
inline constexpr float kFillIntensity = 0.35f;
inline constexpr double kFillAzimuth = kKeyAzimuth + M_PI;
inline constexpr double kFillElevation = M_PI / 8.0;
inline constexpr float kRimIntensity = 0.30f;
inline constexpr double kRimAzimuth = kKeyAzimuth + 0.6 * M_PI;
inline constexpr double kRimElevation = 0.3 * M_PI;  // 54 deg
inline constexpr const char* kFillLightName = "seastack fill light";
inline constexpr const char* kRimLightName = "seastack rim light";

// Wireframe
inline constexpr bool kEnableWireframe = false;

// Debug
inline constexpr bool kDebugWaveSurface = false;
inline constexpr int kDebugPrintEveryNFrames = 60;
inline constexpr bool kDebugSpikeVertex = false;
inline constexpr bool kForceWaterSurface = false;

}  // namespace vsg_config
}  // namespace seastack::viz

