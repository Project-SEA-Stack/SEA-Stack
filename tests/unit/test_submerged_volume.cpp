/*********************************************************************
 * @file  test_submerged_volume.cpp
 * @brief Unit tests for MeshSubmergedVolume (clipped divergence-theorem volume).
 *
 * Checks volume and centre of buoyancy against analytical values for a box
 * and a sphere, including waterplanes away from z = 0 and meshes whose cut
 * faces list an above-water vertex first. With the waterplane at z = 0 the
 * cap triangles contribute zero signed volume, so only a non-zero waterplane
 * exercises the cap-edge orientation.
 *
 * Self-contained: meshes are built in memory, no data files or Chrono.
 *********************************************************************/

#include <seastack/core/math_constants.h>
#include <seastack/hydro/geometry/submerged_volume.h>

#include "test_mesh_shapes.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using seastack::hydro::geometry::MeshSubmergedVolume;
using seastack::hydro::geometry::SubmergedVolumeResult;
using seastack::test::MakeBox;
using seastack::test::MakeUvSphere;
using seastack::test::RotateFaceIndices;

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool condition, const std::string& label) {
    if (condition) {
        ++g_pass;
    } else {
        ++g_fail;
        std::cerr << "FAIL: " << label << "\n";
    }
}

static void CheckNear(double actual, double expected, double tol, const std::string& label) {
    if (std::abs(actual - expected) <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::cerr << "FAIL: " << label << " (expected " << expected << ", got " << actual
                  << ", tol=" << tol << ")\n";
    }
}

static Eigen::Affine3d Translation(double x, double y, double z) {
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translate(Eigen::Vector3d(x, y, z));
    return t;
}

/// Box 2 x 3 x 4 m centred at the origin, cut by waterplanes at several
/// heights, for every cyclic ordering of the face vertices.
static void TestBoxAnyWaterplaneAnyFaceOrder() {
    const double lx = 2.0, ly = 3.0, lz = 4.0;
    for (int shift = 0; shift < 3; ++shift) {
        MeshSubmergedVolume calc(RotateFaceIndices(MakeBox(lx, ly, lz), shift));
        for (double wz : {-1.3, 0.0, 0.7, 1.9}) {
            const SubmergedVolumeResult r = calc.Compute(Eigen::Affine3d::Identity(), wz);
            const double depth = wz + 0.5 * lz;
            const std::string tag =
                "box shift=" + std::to_string(shift) + " wz=" + std::to_string(wz);
            CheckNear(r.volume, lx * ly * depth, 1e-12, tag + " volume");
            CheckNear(r.centroid.z(), -0.5 * lz + 0.5 * depth, 1e-12, tag + " cob z");
            CheckNear(r.centroid.x(), 0.0, 1e-12, tag + " cob x");
            CheckNear(r.centroid.y(), 0.0, 1e-12, tag + " cob y");
        }
    }
}

/// Moving the body and the waterplane by the same vertical offset must not
/// change the submerged volume (the old cap orientation broke this).
static void TestTranslationInvariance() {
    for (int shift = 0; shift < 3; ++shift) {
        MeshSubmergedVolume calc(RotateFaceIndices(MakeUvSphere(5.0, 24, 32), shift));
        const SubmergedVolumeResult ref = calc.Compute(Eigen::Affine3d::Identity(), 0.4);
        for (double dz : {-3.0, 2.5, 10.0}) {
            const SubmergedVolumeResult r = calc.Compute(Translation(1.5, -2.0, dz), 0.4 + dz);
            const std::string tag = "sphere shift=" + std::to_string(shift) +
                                    " dz=" + std::to_string(dz);
            CheckNear(r.volume, ref.volume, 1e-9 * ref.volume, tag + " volume");
            CheckNear(r.centroid.z() - dz, ref.centroid.z(), 1e-9, tag + " cob z");
            CheckNear(r.centroid.x() - 1.5, ref.centroid.x(), 1e-9, tag + " cob x");
        }
    }
}

/// Fine sphere: spherical-cap volume and centroid against analytical values.
static void TestSphereCapAnalytic() {
    const double radius = 5.0;
    MeshSubmergedVolume calc(MakeUvSphere(radius, 90, 120));
    for (double wz : {-2.0, 0.0, 1.0, 3.5}) {
        const double h = radius + wz;  // submerged cap height
        const double v_exact = seastack::kPi * h * h * (3.0 * radius - h) / 3.0;
        // Cap centroid measured from the sphere centre (negative = below).
        const double zc_exact = -3.0 * (2.0 * radius - h) * (2.0 * radius - h) /
                                (4.0 * (3.0 * radius - h));
        const SubmergedVolumeResult r = calc.Compute(Eigen::Affine3d::Identity(), wz);
        const std::string tag = "sphere cap wz=" + std::to_string(wz);
        CheckNear(r.volume / v_exact, 1.0, 2e-3, tag + " volume ratio");
        CheckNear(r.centroid.z(), zc_exact, 2e-2, tag + " cob z");
    }
}

static void TestFullyEmergedAndSubmerged() {
    MeshSubmergedVolume calc(MakeBox(1.0, 1.0, 1.0));
    const SubmergedVolumeResult above = calc.Compute(Translation(0, 0, 5.0), 0.0);
    CheckNear(above.volume, 0.0, 0.0, "emerged volume is zero");

    const SubmergedVolumeResult below = calc.Compute(Translation(0, 0, -5.0), 0.0);
    CheckNear(below.volume, 1.0, 1e-12, "submerged volume equals mesh volume");
    CheckNear(below.centroid.z(), -5.0, 1e-12, "submerged cob at box centre");

    const SubmergedVolumeResult all = calc.Compute(
        Eigen::Affine3d::Identity(), std::numeric_limits<double>::infinity());
    CheckNear(all.volume, 1.0, 1e-12, "waterplane at +inf gives mesh volume");
}

/// Box rotated 90 deg about x behaves like a box with y and z swapped.
static void TestRotatedBox() {
    MeshSubmergedVolume calc(MakeBox(2.0, 3.0, 4.0));
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.rotate(Eigen::AngleAxisd(0.5 * seastack::kPi, Eigen::Vector3d::UnitX()));
    const SubmergedVolumeResult r = calc.Compute(pose, 0.5);
    // Rotated height is 3 m (former y extent): depth = 0.5 + 1.5 = 2.0.
    CheckNear(r.volume, 2.0 * 4.0 * 2.0, 1e-9, "rotated box volume");
    CheckNear(r.centroid.z(), -1.5 + 1.0, 1e-9, "rotated box cob z");
}

static void TestOpenMeshRejected() {
    auto mesh = MakeBox(1.0, 1.0, 1.0);
    mesh.faces.conservativeResize(mesh.faces.rows() - 2, 3);  // drop the +x face
    bool threw = false;
    try {
        MeshSubmergedVolume calc(mesh);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    Check(threw, "open mesh rejected at construction");
}

int main() {
    TestBoxAnyWaterplaneAnyFaceOrder();
    TestTranslationInvariance();
    TestSphereCapAnalytic();
    TestFullyEmergedAndSubmerged();
    TestRotatedBox();
    TestOpenMeshRejected();

    std::cout << "\n========================================\n"
              << "Results: " << g_pass << " passed, " << g_fail << " failed\n"
              << "========================================\n";
    return (g_fail == 0) ? 0 : 1;
}
