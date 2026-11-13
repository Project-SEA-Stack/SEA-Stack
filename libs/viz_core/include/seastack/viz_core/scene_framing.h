#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace seastack::viz {

/// Model-scaled defaults for the 3D view: camera framing, water-surface extent,
/// clip planes and mooring-line marker sizes, all derived from the model's
/// bounding box so that lab-scale and full-scale models look sensible without
/// manual tuning.
///
/// Conventions: world frame, Z up, metres. Renderer-independent so it can be
/// unit-tested without Chrono or VSG.

using Vec3 = std::array<double, 3>;

/// Axis-aligned bounding box in the world frame [m].
/// Default-constructed boxes are empty (min > max) until extended.
struct Bounds3 {
    Vec3 min{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()};
    Vec3 max{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest()};

    bool IsValid() const { return min[0] <= max[0] && min[1] <= max[1] && min[2] <= max[2]; }

    void Extend(const Vec3& p) {
        for (int i = 0; i < 3; ++i) {
            min[i] = std::min(min[i], p[i]);
            max[i] = std::max(max[i], p[i]);
        }
    }
    void Extend(const Bounds3& b) {
        if (b.IsValid()) {
            Extend(b.min);
            Extend(b.max);
        }
    }

    Vec3 Center() const { return {0.5 * (min[0] + max[0]), 0.5 * (min[1] + max[1]), 0.5 * (min[2] + max[2])}; }
    Vec3 Size() const { return {max[0] - min[0], max[1] - min[1], max[2] - min[2]}; }
    /// Radius of the sphere enclosing the box [m].
    double Radius() const {
        const Vec3 s = Size();
        return 0.5 * std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    }
};

/// Smallest model length used for scaling, so a degenerate (point-like)
/// model still gets a usable view [m].
inline constexpr double kMinModelLength = 0.1;

/// Characteristic horizontal model length [m]: the larger plan-view side of the
/// bounds, floored at kMinModelLength.
inline double ModelLengthScale(const Bounds3& b) {
    if (!b.IsValid()) {
        return kMinModelLength;
    }
    const Vec3 s = b.Size();
    return std::max({s[0], s[1], kMinModelLength});
}

/// Deep-water linear wavelength L = g T^2 / (2 pi) [m] for period T [s].
/// Used only to size the visual water patch; returns 0 for T <= 0.
inline double DeepWaterWavelength(double period_s, double gravity = 9.81) {
    if (!(period_s > 0.0) || !(gravity > 0.0)) {
        return 0.0;
    }
    return gravity * period_s * period_s / (2.0 * 3.14159265358979323846);
}

// -----------------------------------------------------------------------------
// Camera framing
// -----------------------------------------------------------------------------

/// Standard camera views. Directions are from the target towards the camera.
enum class CameraView {
    kIsometric,  ///< From the -Y/+X quadrant, 25 deg above the horizon
    kFront,      ///< From -Y, looking along +Y (horizontal)
    kSide,       ///< From +X, looking along -X (horizontal)
    kTop,        ///< From +Z, looking down (+Y up on screen)
};

struct CameraPose {
    Vec3 eye{0.0, -1.0, 0.0};
    Vec3 target{0.0, 0.0, 0.0};
    Vec3 up{0.0, 0.0, 1.0};
};

/// Unit vector from target to camera for a standard view.
inline Vec3 ViewDirection(CameraView view) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    switch (view) {
        case CameraView::kFront:
            return {0.0, -1.0, 0.0};
        case CameraView::kSide:
            return {1.0, 0.0, 0.0};
        case CameraView::kTop:
            return {0.0, 0.0, 1.0};
        case CameraView::kIsometric:
        default: {
            const double az = 35.0 * kDegToRad;  // rotation from -Y towards +X
            const double el = 25.0 * kDegToRad;  // elevation above the horizon
            return {std::cos(el) * std::sin(az), -std::cos(el) * std::cos(az), std::sin(el)};
        }
    }
}

/// Fraction of the view kept free around the model when framing.
inline constexpr double kFramingMargin = 1.15;

/// Camera distance [m] at which a sphere of radius `radius` fills the view
/// with kFramingMargin of headroom, for vertical field of view `vfov_deg` and
/// viewport aspect ratio (width / height).
inline double FramingDistance(double radius, double vfov_deg, double aspect) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double half_v = 0.5 * vfov_deg * kDegToRad;
    const double half_h = std::atan(std::tan(half_v) * std::max(aspect, 1e-3));
    const double half = std::max(std::min(half_v, half_h), 1e-3);
    return kFramingMargin * std::max(radius, 0.5 * kMinModelLength) / std::sin(half);
}

/// Frame `bounds` so the whole box is visible, viewing from `direction`
/// (from target towards camera; normalized internally).
inline CameraPose FrameBounds(const Bounds3& bounds, const Vec3& direction, double vfov_deg, double aspect) {
    CameraPose pose;
    if (!bounds.IsValid()) {
        return pose;
    }
    Vec3 d = direction;
    const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (!(n > 0.0)) {
        d = ViewDirection(CameraView::kIsometric);
    } else {
        d = {d[0] / n, d[1] / n, d[2] / n};
    }
    const double dist = FramingDistance(bounds.Radius(), vfov_deg, aspect);
    pose.target = bounds.Center();
    pose.eye = {pose.target[0] + dist * d[0], pose.target[1] + dist * d[1], pose.target[2] + dist * d[2]};
    // Z up, except looking straight down where Z is the line of sight.
    pose.up = (std::abs(d[2]) > 0.999) ? Vec3{0.0, 1.0, 0.0} : Vec3{0.0, 0.0, 1.0};
    return pose;
}

inline CameraPose FrameBounds(const Bounds3& bounds, CameraView view, double vfov_deg, double aspect) {
    return FrameBounds(bounds, ViewDirection(view), vfov_deg, aspect);
}

/// Near / far clip distances [m].
struct ClipPlanes {
    double near_m = 0.05;
    double far_m = 500.0;
};

/// Clip planes for a scene framed at `view_distance` [m] with a water patch
/// of side `water_extent` [m]. Near scales with the view so lab-scale models
/// are not clipped; far covers the water patch from well outside the framing
/// distance (allowing for zooming out).
inline ClipPlanes ComputeClipPlanes(double view_distance, double water_extent) {
    ClipPlanes c;
    c.near_m = std::max(1e-3, 0.005 * view_distance);
    c.far_m = std::max(10.0 * (view_distance + water_extent), 100.0 * c.near_m);
    return c;
}

// -----------------------------------------------------------------------------
// Water patch and mooring markers
// -----------------------------------------------------------------------------

/// Plan-view water patch [m], centred on the model.
struct WaterExtent {
    double width = 100.0;   ///< Along X [m]
    double length = 100.0;  ///< Along Y [m]
    double center_x = 0.0;
    double center_y = 0.0;
};

/// Water patch side as multiples of the model length and of the wavelength.
inline constexpr double kWaterModelFactor = 2.5;
inline constexpr double kWaterWavelengthFactor = 1.5;

/// Square water patch large enough to show the model with some sea around it
/// and at least kWaterWavelengthFactor wavelengths (if a wavelength is given).
inline WaterExtent ComputeWaterExtent(const Bounds3& bounds, double wavelength_m) {
    WaterExtent w;
    const double side = std::max(kWaterModelFactor * ModelLengthScale(bounds),
                                 kWaterWavelengthFactor * std::max(wavelength_m, 0.0));
    w.width = side;
    w.length = side;
    if (bounds.IsValid()) {
        const Vec3 c = bounds.Center();
        w.center_x = c[0];
        w.center_y = c[1];
    }
    return w;
}

/// Mooring-line visual radii [m].
struct MooringVizRadii {
    double line = 0.15;
    double endpoint = 0.5;
    double node_marker = 0.2;
};

/// Default mooring marker radii proportional to the model length; chosen so a
/// 20 m device (RM3 float) keeps the previous fixed defaults
/// (0.15 / 0.5 / 0.2 m).
inline MooringVizRadii DefaultMooringVizRadii(double model_length_m) {
    const double L = std::max(model_length_m, kMinModelLength);
    MooringVizRadii r;
    r.line = 0.0075 * L;
    r.endpoint = 0.025 * L;
    r.node_marker = 0.01 * L;
    return r;
}

}  // namespace seastack::viz
