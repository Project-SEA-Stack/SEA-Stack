/*********************************************************************
 * @file  test_wetted_surface_pressure.cpp
 * @brief Unit tests for WettedSurfacePressureIntegrator.
 *
 * Pass criteria from docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9:
 *   U3 (integrator part)  still water: force and moment equal rho g V at the
 *                         centre of buoyancy from MeshSubmergedVolume, <= 1e-9 relative
 *   U7                    box under a sloping plane with p = rho g (eta - z):
 *                         force and moment vs closed form, <= 1e-9 relative
 * plus closed-surface and dry-body identities and input validation.
 *
 * Self-contained: meshes are built in memory, no data files or Chrono.
 *********************************************************************/

#include <seastack/hydro/geometry/submerged_volume.h>
#include <seastack/hydro/geometry/wetted_surface_pressure.h>

#include "test_macros.h"
#include "test_mesh_shapes.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>

using seastack::hydro::geometry::MeshSubmergedVolume;
using seastack::hydro::geometry::PressureIntegral;
using seastack::hydro::geometry::TriangleMesh;
using seastack::hydro::geometry::WettedSurfacePressureIntegrator;
using seastack::test::MakeBox;
using seastack::test::MakeUvSphere;
using seastack::test::RotateFaceIndices;

namespace {

constexpr double kRhoG = 1025.0 * 9.81;

Eigen::Affine3d Pose(double x, double y, double z, double roll, double pitch, double yaw) {
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translate(Eigen::Vector3d(x, y, z));
    t.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    return t;
}

/// Integrate p = rho g (eta - z) with eta = sx x + sy y + c (planar surface).
PressureIntegral IntegratePlanar(const WettedSurfacePressureIntegrator& integ, const Eigen::Affine3d& pose,
                                 double sx, double sy, double c, const Eigen::Vector3d& ref) {
    const Eigen::Matrix3Xd v = integ.TransformVertices(pose);
    const int n = integ.NumVertices();
    Eigen::VectorXd d(n), p(n);
    for (int i = 0; i < n; ++i) {
        const double eta = sx * v(0, i) + sy * v(1, i) + c;
        d[i] = v(2, i) - eta;
        p[i] = -kRhoG * d[i];
    }
    return integ.Integrate(v, d, p, Eigen::VectorXd::Zero(n), ref);
}

}  // namespace

int main() {
    TestResults test_results;

    // U3 (integrator part): still water vs MeshSubmergedVolume, several poses and face orders.
    {
        const double r = 5.0;
        const double f_scale = kRhoG * 4.0 / 3.0 * std::acos(-1.0) * r * r * r;
        for (int shift = 0; shift < 3; ++shift) {
            const TriangleMesh mesh = RotateFaceIndices(MakeUvSphere(r, 16, 32, Eigen::Vector3d(0.0, 0.0, 2.0)), shift);
            const WettedSurfacePressureIntegrator integ(mesh);
            const MeshSubmergedVolume volume(mesh);
            for (const auto& pose : {Pose(0.0, 0.0, -2.0, 0.0, 0.0, 0.0), Pose(3.0, -7.0, -4.5, 0.3, -0.2, 1.1),
                                     Pose(-1.0, 2.0, 1.0, -0.6, 0.4, -2.0)}) {
                const Eigen::Vector3d ref = pose.translation();
                const PressureIntegral res = IntegratePlanar(integ, pose, 0.0, 0.0, 0.0, ref);
                const auto vol = volume.Compute(pose, 0.0);
                const Eigen::Vector3d f_ref(0.0, 0.0, kRhoG * vol.volume);
                const Eigen::Vector3d m_ref = (vol.centroid - ref).cross(f_ref);
                const std::string tag = " (face shift " + std::to_string(shift) + ")";
                TEST_NEAR((res.force - f_ref).norm(), 0.0, 1e-9 * f_scale, "U3 still-water force" + tag);
                TEST_NEAR((res.moment - m_ref).norm(), 0.0, 1e-9 * f_scale * r, "U3 still-water moment" + tag);
            }
        }
    }

    // U7: box under a sloping plane, hydrostatic pressure. Closed form for an
    // axis-aligned L x W box centred at (x0, y0), bottom zb, surface
    // eta = sx x + sy y + c, written in local u = x - x0, v = y - y0:
    //   V = L W (c' - zb), c' = c + sx x0 + sy y0
    //   u_B = sx L^3 W / (12 V), v_B = sy W^3 L / (12 V)
    //   z_B = [sx^2 L^3 W / 12 + sy^2 W^3 L / 12 + c'^2 L W - zb^2 L W] / (2 V)
    //   F = rho g V (-sx, -sy, 1) at r_B (the surface is a zero-pressure plane).
    {
        const double L = 4.0, W = 3.0, H = 2.0;
        const double x0 = 1.5, y0 = -2.0, zc = -0.5;
        const double zb = zc - 0.5 * H;
        const double sx = 0.1, sy = -0.05, c = -0.2 - sx * x0 - sy * y0;  // c' = -0.2
        const double cp = c + sx * x0 + sy * y0;
        const double vol = L * W * (cp - zb);
        const Eigen::Vector3d r_b(x0 + sx * L * L * L * W / (12.0 * vol), y0 + sy * W * W * W * L / (12.0 * vol),
                                  (sx * sx * L * L * L * W / 12.0 + sy * sy * W * W * W * L / 12.0 + cp * cp * L * W -
                                   zb * zb * L * W) / (2.0 * vol));
        const Eigen::Vector3d f_ref = kRhoG * vol * Eigen::Vector3d(-sx, -sy, 1.0);
        const Eigen::Vector3d ref(0.3, 0.7, -1.1);
        const Eigen::Vector3d m_ref = (r_b - ref).cross(f_ref);
        for (int shift = 0; shift < 3; ++shift) {
            const WettedSurfacePressureIntegrator integ(RotateFaceIndices(MakeBox(L, W, H), shift));
            const PressureIntegral res = IntegratePlanar(integ, Pose(x0, y0, zc, 0.0, 0.0, 0.0), sx, sy, c, ref);
            const std::string tag = " (face shift " + std::to_string(shift) + ")";
            TEST_NEAR((res.force - f_ref).norm(), 0.0, 1e-9 * f_ref.norm(), "U7 sloping plane force" + tag);
            TEST_NEAR((res.moment - m_ref).norm(), 0.0, 1e-9 * f_ref.norm() * L, "U7 sloping plane moment" + tag);
        }
    }

    // Flat plane: wetted area of a box = bottom + perimeter x draft.
    {
        const WettedSurfacePressureIntegrator integ(MakeBox(4.0, 3.0, 2.0));
        const PressureIntegral res = IntegratePlanar(integ, Pose(0.0, 0.0, -0.4, 0.0, 0.0, 0.0), 0.0, 0.0, 0.0,
                                                     Eigen::Vector3d::Zero());
        TEST_NEAR(res.wetted_area, 4.0 * 3.0 + 2.0 * (4.0 + 3.0) * 1.4, 1e-12, "box wetted area");
    }

    // Closed surface under uniform pressure: zero force and moment.
    {
        const WettedSurfacePressureIntegrator integ(MakeUvSphere(2.0, 12, 24));
        const Eigen::Affine3d pose = Pose(1.0, 1.0, -10.0, 0.2, 0.1, 0.3);
        const Eigen::Matrix3Xd v = integ.TransformVertices(pose);
        const int n = integ.NumVertices();
        const PressureIntegral res = integ.Integrate(v, Eigen::VectorXd::Constant(n, -1.0),
                                                     Eigen::VectorXd::Constant(n, 5.0e4), Eigen::VectorXd::Zero(n),
                                                     Eigen::Vector3d(4.0, -3.0, 2.0));
        TEST_NEAR(res.force.norm(), 0.0, 1e-9 * 5.0e4 * 50.0, "uniform pressure: zero force");
        TEST_NEAR(res.moment.norm(), 0.0, 1e-9 * 5.0e4 * 50.0 * 10.0, "uniform pressure: zero moment");
    }

    // Fully dry body: nothing integrated.
    {
        const WettedSurfacePressureIntegrator integ(MakeBox(1.0, 1.0, 1.0));
        const PressureIntegral res = IntegratePlanar(integ, Pose(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), 0.0, 0.0, 0.0,
                                                     Eigen::Vector3d::Zero());
        TEST_ASSERT(res.force.isZero(0.0) && res.moment.isZero(0.0) && res.wetted_area == 0.0, "dry body: zero");
    }

    // Input validation.
    {
        bool threw = false;
        TriangleMesh open = MakeBox(1.0, 1.0, 1.0);
        open.faces.conservativeResize(open.faces.rows() - 1, 3);
        try {
            const WettedSurfacePressureIntegrator integ(open);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        TEST_ASSERT(threw, "open mesh rejected");

        threw = false;
        const WettedSurfacePressureIntegrator integ(MakeBox(1.0, 1.0, 1.0));
        try {
            (void)integ.Integrate(integ.TransformVertices(Eigen::Affine3d::Identity()), Eigen::VectorXd::Zero(3),
                                  Eigen::VectorXd::Zero(8), Eigen::VectorXd::Zero(8), Eigen::Vector3d::Zero());
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        TEST_ASSERT(threw, "wrong array size rejected");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
