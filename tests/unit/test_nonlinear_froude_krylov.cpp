/*********************************************************************
 * @file  test_nonlinear_froude_krylov.cpp
 * @brief Unit tests for NonlinearFroudeKrylovComponent.
 *
 * Pass criteria from docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9:
 *   U3   still water: equals NonlinearHydrostaticsComponent, box and sphere,
 *        several poses, both pressure models: <= 1e-9 relative
 *   U4   frames: translated + yawed body and wave give the rotated force;
 *        body rotation via orientation_rpy equals a pre-rotated mesh: <= 1e-9
 *   U5   fixed body off the sphere equator: F(a) - F(0) scales as a^2,
 *        log-log slope 2 +/- 0.1 for a = 1e-3 .. 1e-1 m, both models
 *   U6   mesh linear FK heave on a truncated vertical cylinder vs
 *        rho g a e^{-kd} pi R^2 2 J1(kR)/(kR): <= 1% on the finest mesh
 *   U8   mesh convergence of the dynamic pressure force on a sphere, three
 *        resolutions: error falls monotonically, observed order >= 1.5
 *   U9a  long wave: wall-sided box, hydrostatic model, F_z - F_z,still vs
 *        rho g int(eta) dA: <= 1e-9; Airy vs hydrostatic linear FK heave on
 *        a sphere in deep water: <= 1% at T = 120 s, difference ~ k
 * plus ramp shapes, diagnostics and input validation.
 *
 * Self-contained: meshes and waves are built in memory, no data files or Chrono.
 *********************************************************************/

#include <seastack/hydro/force_components/nonlinear_froude_krylov_component.h>
#include <seastack/hydro/force_components/nonlinear_hydrostatics_component.h>
#include <seastack/hydro/geometry/submerged_volume.h>
#include <seastack/hydro/waves/eta_table_wave_field.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include "test_macros.h"
#include "test_mesh_shapes.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using seastack::hydro::BodyForces;
using seastack::hydro::BodyState;
using seastack::hydro::EtaTableWaveField;
using seastack::hydro::FroudeKrylovPressureModel;
using seastack::hydro::LinearDirectionalWaveField;
using seastack::hydro::NonlinearFroudeKrylovComponent;
using seastack::hydro::NonlinearHydrostaticsComponent;
using seastack::hydro::SystemState;
using seastack::hydro::WaveBase;
using seastack::hydro::WaveComponent;
using seastack::hydro::geometry::ISubmergedVolumeCalculator;
using seastack::hydro::geometry::MeshSubmergedVolume;
using seastack::hydro::geometry::TriangleMesh;
using seastack::test::MakeBox;
using seastack::test::MakeUvSphere;
using seastack::test::MakeVerticalCylinder;

namespace {

constexpr double kRho  = 1025.0;
constexpr double kG    = 9.81;
constexpr double kRhoG = kRho * kG;
const double kPi       = std::acos(-1.0);
const Eigen::Vector3d kGravity(0.0, 0.0, -kG);

using Vec6     = Eigen::Matrix<double, 6, 1>;
using Model    = FroudeKrylovPressureModel;
using RampKind = NonlinearFroudeKrylovComponent::RampShape;

const std::vector<Model> kModels = {Model::kAiryWheeler, Model::kHydrostatic};

std::string ModelName(Model m) { return m == Model::kAiryWheeler ? "airy_wheeler" : "hydrostatic"; }

/// Deep-water wave component (k = w^2 / g).
WaveComponent DeepComponent(double amplitude, double omega, double direction, double phase) {
    WaveComponent c;
    c.amplitude = amplitude;
    c.omega     = omega;
    c.k         = omega * omega / kG;
    c.direction = direction;
    c.phase     = phase;
    return c;
}

std::shared_ptr<WaveBase> DeepWave(std::vector<WaveComponent> comps) {
    return std::make_shared<LinearDirectionalWaveField>(std::move(comps), 0.0);
}

/// Single-body component, no ramp.
NonlinearFroudeKrylovComponent MakeComponent(const TriangleMesh& mesh, const Eigen::Vector3d& bem_cg, Model model,
                                             std::shared_ptr<WaveBase> wave) {
    std::vector<NonlinearFroudeKrylovComponent::BodySetup> bodies;
    bodies.push_back({0, mesh, model, bem_cg});
    return NonlinearFroudeKrylovComponent(std::move(bodies), std::move(wave), kRho, kGravity, 0.0,
                                          RampKind::kCosine);
}

BodyState State(const Eigen::Vector3d& position, const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero()) {
    BodyState s;
    s.position        = position;
    s.orientation_rpy = rpy;
    return s;
}

/// Total component output (force; moment about the CG) for one body.
Vec6 Evaluate(seastack::hydro::IHydroForceComponent& component, const BodyState& body, double time) {
    SystemState state;
    state.bodies = {body};
    BodyForces forces(1);
    component.Compute(state, time, forces);
    Vec6 out;
    out << forces[0].force, forces[0].moment;
    return out;
}

Eigen::Matrix3d Rpy(const Eigen::Vector3d& rpy) {
    return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

TriangleMesh RotateMesh(TriangleMesh mesh, const Eigen::Matrix3d& rotation) {
    mesh.vertices = (mesh.vertices * rotation.transpose()).eval();
    return mesh;
}

/// Least-squares slope of log|y| against log x.
double LogLogSlope(const std::vector<double>& x, const std::vector<double>& y) {
    const int n = static_cast<int>(x.size());
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (int i = 0; i < n; ++i) {
        const double lx = std::log(x[i]), ly = std::log(std::abs(y[i]));
        sx += lx;
        sy += ly;
        sxx += lx * lx;
        sxy += lx * ly;
    }
    return (n * sxy - sx * sy) / (n * sxx - sx * sx);
}

}  // namespace

int main() {
    TestResults test_results;

    // U3: still water equals NonlinearHydrostaticsComponent (no wave, and a wave at r(t) = 0).
    {
        struct Case {
            std::string name;
            TriangleMesh mesh;
            double scale;  // force scale rho g V_total [N]
            double length; // moment arm scale [m]
        };
        const std::vector<Case> cases = {
            {"sphere", MakeUvSphere(5.0, 16, 32), kRhoG * 4.0 / 3.0 * kPi * 125.0, 5.0},
            {"box", MakeBox(6.0, 4.0, 3.0, Eigen::Vector3d(0.5, -0.3, 0.2)), kRhoG * 72.0, 4.0}};
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> poses = {
            {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
            {{3.0, -7.0, -1.2}, {0.3, -0.2, 1.1}},
            {{-1.0, 2.0, 0.8}, {-0.6, 0.4, -2.0}}};
        for (const auto& c : cases) {
            std::vector<std::unique_ptr<ISubmergedVolumeCalculator>> calc;
            calc.push_back(std::make_unique<MeshSubmergedVolume>(c.mesh));
            NonlinearHydrostaticsComponent nl_hs(std::move(calc), kRho, kGravity);
            for (Model model : kModels) {
                for (bool with_wave : {false, true}) {
                    std::vector<NonlinearFroudeKrylovComponent::BodySetup> bodies;
                    bodies.push_back({0, c.mesh, model, Eigen::Vector3d::Zero()});
                    // With a wave and a 10 s ramp, r(0) = 0: still water at t = 0.
                    NonlinearFroudeKrylovComponent fk(
                        std::move(bodies), with_wave ? DeepWave({DeepComponent(1.0, 0.8, 0.3, 0.5)}) : nullptr,
                        kRho, kGravity, 10.0, RampKind::kCosine);
                    for (const auto& [pos, rpy] : poses) {
                        const Vec6 ref = Evaluate(nl_hs, State(pos, rpy), 0.0);
                        const Vec6 got = Evaluate(fk, State(pos, rpy), 0.0);
                        const std::string tag = " (" + c.name + ", " + ModelName(model) +
                                                (with_wave ? ", wave at r = 0)" : ", no wave)");
                        TEST_NEAR((got.head<3>() - ref.head<3>()).norm(), 0.0, 1e-9 * c.scale, "U3 force" + tag);
                        TEST_NEAR((got.tail<3>() - ref.tail<3>()).norm(), 0.0, 1e-9 * c.scale * c.length,
                                  "U3 moment" + tag);
                    }
                }
            }
        }
    }

    // U4a: translate by x0 and yaw by psi the mesh, BEM position, body position and
    // wave (directions + psi, phases shifted so the body sees the same wave).
    // The force and moment must rotate by R = Rz(psi).
    {
        const TriangleMesh mesh = MakeBox(6.0, 4.0, 3.0, Eigen::Vector3d(0.5, -0.3, 0.2));
        const Eigen::Vector3d bem_cg(1.0, 2.0, -0.1);
        const Eigen::Vector3d pos = bem_cg + Eigen::Vector3d(0.4, -0.2, -0.3);
        const std::vector<WaveComponent> comps = {DeepComponent(0.8, 0.7, 0.2, 0.3),
                                                  DeepComponent(0.5, 1.1, 1.0, 1.7)};
        const double psi = 0.7;
        const Eigen::Vector3d x0(30.0, -12.0, 0.0);
        const Eigen::Matrix3d rot = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        std::vector<WaveComponent> comps_moved = comps;
        for (auto& c : comps_moved) {
            c.direction += psi;
            c.phase -= c.k * (std::cos(c.direction) * x0.x() + std::sin(c.direction) * x0.y());
        }

        for (Model model : kModels) {
            auto ref_fk = MakeComponent(mesh, bem_cg, model, DeepWave(comps));
            auto moved_fk = MakeComponent(RotateMesh(mesh, rot), x0 + rot * bem_cg, model, DeepWave(comps_moved));
            for (double t : {0.0, 3.3, 7.9}) {
                const Vec6 ref = Evaluate(ref_fk, State(pos), t);
                const Vec6 got = Evaluate(moved_fk, State(x0 + rot * pos), t);
                const double scale = ref.head<3>().norm();
                const std::string tag = " (" + ModelName(model) + ", t = " + std::to_string(t) + ")";
                TEST_NEAR((got.head<3>() - rot * ref.head<3>()).norm(), 0.0, 1e-9 * scale, "U4 translated+yawed force" + tag);
                TEST_NEAR((got.tail<3>() - rot * ref.tail<3>()).norm(), 0.0, 1e-9 * scale * 6.0,
                          "U4 translated+yawed moment" + tag);
            }
        }
    }

    // U4b: body rotation through orientation_rpy (R = Rz Ry Rx) equals the same
    // world geometry given as a pre-rotated mesh at zero orientation.
    {
        const TriangleMesh mesh = MakeBox(6.0, 4.0, 3.0, Eigen::Vector3d(0.5, -0.3, 0.2));
        const Eigen::Vector3d rpy(0.2, -0.15, 0.6);
        const Eigen::Vector3d pos(0.3, -0.4, -0.2);
        auto wave = DeepWave({DeepComponent(0.8, 0.7, 0.2, 0.3), DeepComponent(0.5, 1.1, 1.0, 1.7)});
        for (Model model : kModels) {
            auto rotated_by_pose = MakeComponent(mesh, Eigen::Vector3d::Zero(), model, wave);
            auto rotated_mesh = MakeComponent(RotateMesh(mesh, Rpy(rpy)), Eigen::Vector3d::Zero(), model, wave);
            (void)Evaluate(rotated_by_pose, State(pos, rpy), 2.5);
            (void)Evaluate(rotated_mesh, State(pos), 2.5);
            const Vec6 got = rotated_by_pose.GetLastPressureForce(0);
            const Vec6 ref = rotated_mesh.GetLastPressureForce(0);
            const double scale = ref.head<3>().norm();
            TEST_NEAR((got.head<3>() - ref.head<3>()).norm(), 0.0, 1e-9 * scale,
                      "U4 orientation_rpy force (" + ModelName(model) + ")");
            TEST_NEAR((got.tail<3>() - ref.tail<3>()).norm(), 0.0, 1e-9 * scale * 6.0,
                      "U4 orientation_rpy moment (" + ModelName(model) + ")");
        }
    }

    // U5: fixed body, sphere centre 1 m above the mean surface (waterline off the
    // equator): the nonlinear correction F(a) - F(0) scales as a^2.
    {
        const TriangleMesh mesh = MakeUvSphere(5.0, 24, 48);
        const Eigen::Vector3d bem_cg(0.0, 0.0, 1.0);
        const std::vector<double> amplitudes = {1e-3, 1e-2, 1e-1};
        for (Model model : kModels) {
            auto still = MakeComponent(mesh, bem_cg, model, nullptr);
            const Vec6 f0 = Evaluate(still, State(bem_cg), 2.0);
            std::vector<double> heave, surge;
            for (double a : amplitudes) {
                auto fk = MakeComponent(mesh, bem_cg, model, DeepWave({DeepComponent(a, 0.8, 0.3, 0.4)}));
                const Vec6 c = Evaluate(fk, State(bem_cg), 2.0) - f0;
                heave.push_back(c[2]);
                surge.push_back(c[0]);
            }
            const double slope_z = LogLogSlope(amplitudes, heave);
            const double slope_x = LogLogSlope(amplitudes, surge);
            std::cout << "  U5 " << ModelName(model) << ": slope heave " << slope_z << ", surge " << slope_x << "\n";
            TEST_NEAR(slope_z, 2.0, 0.1, "U5 heave correction ~ a^2 (" + ModelName(model) + ")");
            TEST_NEAR(slope_x, 2.0, 0.1, "U5 surge correction ~ a^2 (" + ModelName(model) + ")");
        }
    }

    // U6: mesh linear FK heave on a truncated vertical cylinder (R = 5 m, draft
    // 4 m), deep water. Only the flat bottom carries vertical pressure:
    //   F_z = rho g a e^{-kd} pi R^2 2 J1(kR) / (kR)   at theta = 0.
    {
        const double radius = 5.0, draft = 4.0, omega = 0.8, k = omega * omega / kG;
        const double expected = kRhoG * std::exp(-k * draft) * kPi * radius * radius * 2.0 *
                                std::cyl_bessel_j(1.0, k * radius) / (k * radius);
        double err_prev = 1.0;
        bool monotone = true;
        for (int n_theta : {32, 64, 128}) {
            auto fk = MakeComponent(MakeVerticalCylinder(radius, -draft, 3.0, n_theta, 8), Eigen::Vector3d::Zero(),
                                    Model::kAiryWheeler, DeepWave({DeepComponent(1.0, omega, 0.0, 0.0)}));
            (void)Evaluate(fk, State(Eigen::Vector3d::Zero()), 0.0);
            const double err = std::abs(fk.GetLastLinearFroudeKrylovForce(0)[2] / expected - 1.0);
            std::cout << "  U6 n_theta = " << n_theta << ": relative error " << err << "\n";
            monotone = monotone && err < err_prev;
            err_prev = err;
        }
        TEST_ASSERT(monotone, "U6 cylinder linear FK error falls with refinement");
        TEST_NEAR(err_prev, 0.0, 0.01, "U6 cylinder linear FK heave within 1% (n_theta = 128)");
    }

    // U8: mesh convergence of the dynamic pressure force F_p(a) - F_p(0) on a
    // half-submerged sphere, against a 4x finer mesh.
    {
        const double radius = 5.0, t = 1.0;
        auto wave = DeepWave({DeepComponent(1.5, 0.8, 0.0, 0.0)});
        auto dynamic_force = [&](int n_lat) {
            const TriangleMesh mesh = MakeUvSphere(radius, n_lat, 2 * n_lat);
            auto still = MakeComponent(mesh, Eigen::Vector3d::Zero(), Model::kAiryWheeler, nullptr);
            auto fk = MakeComponent(mesh, Eigen::Vector3d::Zero(), Model::kAiryWheeler, wave);
            (void)Evaluate(still, State(Eigen::Vector3d::Zero()), t);
            (void)Evaluate(fk, State(Eigen::Vector3d::Zero()), t);
            return Vec6(fk.GetLastPressureForce(0) - still.GetLastPressureForce(0));
        };
        const Vec6 ref = dynamic_force(128);
        std::vector<double> err_z, err_x;
        for (int n_lat : {8, 16, 32}) {
            const Vec6 f = dynamic_force(n_lat);
            err_z.push_back(std::abs(f[2] - ref[2]));
            err_x.push_back(std::abs(f[0] - ref[0]));
        }
        for (int i = 0; i < 2; ++i) {
            const double order_z = std::log2(err_z[i] / err_z[i + 1]);
            const double order_x = std::log2(err_x[i] / err_x[i + 1]);
            std::cout << "  U8 refinement " << i + 1 << ": observed order heave " << order_z << ", surge "
                      << order_x << "\n";
            TEST_ASSERT(order_z >= 1.5, "U8 heave observed order >= 1.5, refinement " + std::to_string(i + 1));
            TEST_ASSERT(order_x >= 1.5, "U8 surge observed order >= 1.5, refinement " + std::to_string(i + 1));
        }
    }

    // U9a (wall-sided): hydrostatic model, small box in a long deep-water wave.
    // Walls carry no vertical force, so F_z - F_z,still = rho g int_bottom eta dA
    // exactly, up to the linear interpolation of eta over the bottom
    // ((kL)^2 / 24 ~ 1e-10 relative here).
    {
        const double L = 0.2, W = 0.2, a = 0.02, omega = 2.0 * kPi / 120.0, k = omega * omega / kG;
        const double phase = 0.4, t = 17.0;
        const TriangleMesh box = MakeBox(L, W, 0.2);
        auto still = MakeComponent(box, Eigen::Vector3d::Zero(), Model::kHydrostatic, nullptr);
        auto fk = MakeComponent(box, Eigen::Vector3d::Zero(), Model::kHydrostatic,
                                DeepWave({DeepComponent(a, omega, 0.0, phase)}));
        (void)Evaluate(still, State(Eigen::Vector3d::Zero()), t);
        (void)Evaluate(fk, State(Eigen::Vector3d::Zero()), t);
        const double got = fk.GetLastPressureForce(0)[2] - still.GetLastPressureForce(0)[2];
        // int_{-L/2}^{L/2} cos(k x - w t + phi) dx = (2/k) sin(kL/2) cos(-w t + phi)
        const double expected = kRhoG * a * W * (2.0 / k) * std::sin(0.5 * k * L) * std::cos(phase - omega * t);
        TEST_NEAR(got, expected, 1e-9 * kRhoG * a * L * W, "U9a wall-sided box: F_z - F_still = rho g int(eta) dA");
    }

    // U9a (k -> 0): Airy vs hydrostatic mesh linear FK heave on a half-submerged
    // sphere in deep water; the relative difference is first order in k.
    {
        const TriangleMesh mesh = MakeUvSphere(5.0, 24, 48);
        auto linear_fk_heave = [&](Model model, double period) {
            auto fk = MakeComponent(mesh, Eigen::Vector3d::Zero(), model,
                                    DeepWave({DeepComponent(1.0, 2.0 * kPi / period, 0.0, 0.0)}));
            (void)Evaluate(fk, State(Eigen::Vector3d::Zero()), 0.0);
            return fk.GetLastLinearFroudeKrylovForce(0)[2];
        };
        std::vector<double> rel_diff;
        for (double period : {120.0, 60.0}) {
            const double hs = linear_fk_heave(Model::kHydrostatic, period);
            rel_diff.push_back(std::abs(linear_fk_heave(Model::kAiryWheeler, period) / hs - 1.0));
        }
        // Halving T multiplies k by 4.
        const double order = std::log(rel_diff[1] / rel_diff[0]) / std::log(4.0);
        std::cout << "  U9a Airy vs hydrostatic heave: " << rel_diff[0] << " (T = 120 s), " << rel_diff[1]
                  << " (T = 60 s), order in k " << order << "\n";
        TEST_NEAR(rel_diff[0], 0.0, 0.01, "U9a Airy and hydrostatic linear FK heave within 1% at T = 120 s");
        TEST_NEAR(order, 1.0, 0.1, "U9a Airy minus hydrostatic difference is first order in k");
    }

    // Ramp shapes.
    {
        std::vector<NonlinearFroudeKrylovComponent::BodySetup> b1, b2;
        b1.push_back({0, MakeBox(1.0, 1.0, 1.0), Model::kAiryWheeler, Eigen::Vector3d::Zero()});
        b2.push_back({0, MakeBox(1.0, 1.0, 1.0), Model::kAiryWheeler, Eigen::Vector3d::Zero()});
        const NonlinearFroudeKrylovComponent cosine(std::move(b1), nullptr, kRho, kGravity, 20.0, RampKind::kCosine);
        const NonlinearFroudeKrylovComponent linear(std::move(b2), nullptr, kRho, kGravity, 20.0, RampKind::kLinear);
        TEST_NEAR(cosine.RampFactor(-1.0), 0.0, 0.0, "ramp: zero before t = 0");
        TEST_NEAR(cosine.RampFactor(5.0), 0.5 * (1.0 - std::cos(kPi * 0.25)), 1e-15, "ramp: cosine shape");
        TEST_NEAR(linear.RampFactor(5.0), 0.25, 1e-15, "ramp: linear shape");
        TEST_NEAR(cosine.RampFactor(25.0), 1.0, 0.0, "ramp: one after T_r");
    }

    // Diagnostics: half-submerged sphere wetted area ~ 2 pi R^2 in still water.
    {
        auto fk = MakeComponent(MakeUvSphere(5.0, 64, 128), Eigen::Vector3d::Zero(), Model::kAiryWheeler, nullptr);
        (void)Evaluate(fk, State(Eigen::Vector3d::Zero()), 0.0);
        TEST_NEAR(fk.GetLastWettedArea(0), 2.0 * kPi * 25.0, 0.002 * 2.0 * kPi * 25.0,
                  "wetted area of a half-submerged sphere");
        TEST_NEAR(fk.GetLastLinearFroudeKrylovForce(0).norm(), 0.0, 0.0, "no linear FK in still water");
        TEST_NEAR(fk.GetLastWettedArea(3), 0.0, 0.0, "diagnostics for an unknown body are zero");
    }

    // Input validation.
    {
        bool threw = false;
        try {
            NonlinearFroudeKrylovComponent fk({}, nullptr, kRho, kGravity, 0.0, RampKind::kCosine);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        TEST_ASSERT(threw, "no bodies rejected");

        threw = false;
        try {
            auto table = std::make_shared<EtaTableWaveField>(std::vector<double>{0.0, 1.0, 2.0},
                                                             std::vector<double>{0.0, 0.1, 0.0}, 0.0);
            auto fk = MakeComponent(MakeBox(1.0, 1.0, 1.0), Eigen::Vector3d::Zero(), Model::kAiryWheeler, table);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        TEST_ASSERT(threw, "wave without discrete components rejected");

        threw = false;
        std::vector<NonlinearFroudeKrylovComponent::BodySetup> bodies;
        bodies.push_back({2, MakeBox(1.0, 1.0, 1.0), Model::kAiryWheeler, Eigen::Vector3d::Zero()});
        NonlinearFroudeKrylovComponent fk(std::move(bodies), nullptr, kRho, kGravity, 0.0, RampKind::kCosine);
        try {
            (void)Evaluate(fk, State(Eigen::Vector3d::Zero()), 0.0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        TEST_ASSERT(threw, "body index outside the system rejected");
    }

    test_results.Summary();
    return test_results.failed > 0 ? 1 : 0;
}
