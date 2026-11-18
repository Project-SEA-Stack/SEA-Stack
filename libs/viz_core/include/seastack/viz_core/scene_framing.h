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

/// Mean plan-view model size [m]: the geometric mean of the plan sides, so a
/// long slender model (e.g. a 180 m x 3 m attenuator) is not sized from its
/// length alone. Used for mooring radii and the still-water patch. Equals
/// ModelLengthScale for a square plan.
inline double MeanPlanLength(const Bounds3& b) {
    if (!b.IsValid()) {
        return kMinModelLength;
    }
    const Vec3 s = b.Size();
    return std::sqrt(std::max(s[0], kMinModelLength) * std::max(s[1], kMinModelLength));
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
inline constexpr double kWaterModelFactor = 1.75;
inline constexpr double kWaterWavelengthFactor = 1.5;
/// Still-water minimum side as a multiple of MeanPlanLength, so a small body
/// with no wavelength to size from still sits on a sizeable patch.
inline constexpr double kStillWaterPlanFactor = 5.0;
/// Shorter patch side as a minimum fraction of the longer one, so a slender
/// model does not sit on a thin strip of water.
inline constexpr double kWaterMinAspect = 0.4;

/// Water patch that covers the model with some sea around it and, along each
/// side, at least kWaterWavelengthFactor wavelengths. Each side follows the
/// model's plan size along that axis, so a slender model gets a rectangle
/// (no shorter than kWaterMinAspect of the long side) and the grid points are
/// not spent on empty sea beside it. With no wavelength (still water,
/// wavelength_m <= 0) each side is at least kStillWaterPlanFactor mean plan sizes.
inline WaterExtent ComputeWaterExtent(const Bounds3& bounds, double wavelength_m) {
    WaterExtent w;
    Vec3 size{kMinModelLength, kMinModelLength, kMinModelLength};
    if (bounds.IsValid()) {
        size = bounds.Size();
    }
    const double wave_side = kWaterWavelengthFactor * std::max(wavelength_m, 0.0);
    w.width = std::max(kWaterModelFactor * std::max(size[0], kMinModelLength), wave_side);
    w.length = std::max(kWaterModelFactor * std::max(size[1], kMinModelLength), wave_side);
    if (!(wavelength_m > 0.0)) {
        const double still_side = kStillWaterPlanFactor * MeanPlanLength(bounds);
        w.width = std::max(w.width, still_side);
        w.length = std::max(w.length, still_side);
    }
    const double min_side = kWaterMinAspect * std::max(w.width, w.length);
    w.width = std::max(w.width, min_side);
    w.length = std::max(w.length, min_side);
    if (bounds.IsValid()) {
        const Vec3 c = bounds.Center();
        w.center_x = c[0];
        w.center_y = c[1];
    }
    return w;
}

/// Grid vertices along X and Y of a water patch.
struct WaterGridCounts {
    int nx = 2;
    int ny = 2;
};

/// Vertex counts for a `width` x `length` [m] patch with about resolution^2
/// vertices in total and near-square cells, so a rectangular patch costs the
/// same as a square one at that resolution but has finer cells. A square
/// patch gives resolution x resolution.
inline WaterGridCounts ComputeWaterGridCounts(double width, double length, int resolution) {
    const int n = std::max(resolution, 2);
    WaterGridCounts c{n, n};
    if (!(width > 0.0) || !(length > 0.0)) {
        return c;
    }
    const double cell = std::sqrt(width * length) / (n - 1);
    c.nx = std::max(2, static_cast<int>(std::lround(width / cell)) + 1);
    c.ny = std::max(2, static_cast<int>(std::lround(length / cell)) + 1);
    return c;
}

/// How finely the water surface is drawn, as grid points per deep-water
/// wavelength at the sea's characteristic period. Shorter components of an
/// irregular sea get fewer points (a quarter as many at twice the peak frequency).
enum class WaterDetail { kCoarse, kNormal, kFine };

inline double WaterPointsPerWavelength(WaterDetail detail) {
    switch (detail) {
        case WaterDetail::kCoarse:
            return 24.0;
        case WaterDetail::kFine:
            return 96.0;
        case WaterDetail::kNormal:
        default:
            return 48.0;
    }
}

inline constexpr int kMinWaterGridResolution = 32;
inline constexpr int kMaxWaterGridResolution = 512;
/// Resolution without a wavelength (no waves): the surface is flat.
inline constexpr int kStillWaterGridResolution = 64;

/// Grid resolution (as ComputeWaterGridCounts) with about `points_per_wave`
/// points per `wavelength` [m] on a `width` x `length` [m] patch: rounded up
/// to a multiple of 16, within [kMinWaterGridResolution, max_resolution].
/// kStillWaterGridResolution (clamped the same way) if there is no wavelength.
inline int ChooseWaterGridResolution(double width, double length, double wavelength, double points_per_wave,
                                     int max_resolution = kMaxWaterGridResolution) {
    const int hi = std::max(max_resolution, kMinWaterGridResolution);
    if (!(width > 0.0) || !(length > 0.0) || !(wavelength > 0.0) || !(points_per_wave > 0.0)) {
        return std::clamp(kStillWaterGridResolution, kMinWaterGridResolution, hi);
    }
    // ComputeWaterGridCounts: cell = sqrt(width length) / (resolution - 1).
    const double cell = wavelength / points_per_wave;
    const double wanted = std::sqrt(width * length) / cell + 1.0;
    if (!(wanted < static_cast<double>(hi))) {
        return hi;
    }
    const int rounded = 16 * static_cast<int>(std::ceil(wanted / 16.0));
    return std::clamp(rounded, kMinWaterGridResolution, hi);
}

/// Mooring-line visual radii [m].
struct MooringVizRadii {
    double line = 0.15;
    double endpoint = 0.5;
    double node_marker = 0.2;
};

/// Default mooring radii proportional to the model length (see
/// MeanPlanLength); a 20 m device gets 0.15 m lines, 0.5 m endpoints and
/// 0.2 m node markers. Node markers show MoorDyn's discretisation, not
/// hardware; a radius of 0 hides them.
inline MooringVizRadii DefaultMooringVizRadii(double model_length_m) {
    const double L = std::max(model_length_m, kMinModelLength);
    MooringVizRadii r;
    r.line = 0.0075 * L;
    r.endpoint = 0.025 * L;
    r.node_marker = 0.01 * L;
    return r;
}

}  // namespace seastack::viz
