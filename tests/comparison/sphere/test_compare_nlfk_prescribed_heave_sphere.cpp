/**
 * C3 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): pressure force on
 * bodies in prescribed heave, for comparison with the Giorgi & Ringwood
 * nonlinear Froude-Krylov model (evaluated by nlfk_reference.py).
 *
 * Bodies, CG frame:
 *   sphere    IEA sphere mesh (sphere.obj), R = 5 m, centre 2 m above the CG,
 *             mean CG at z = -2 m
 *   cylinder  R = 3 m, bottom 4 m below and top 3 m above the CG, mean CG at 0
 * Motion: z_cg(t) = z_mean + 1.5 sin(0.37 w t), as in `nlfk_reference.py
 * gr-compare`. Deep-water regular waves, T = 12, 20, 30 s (kR <= 0.15 for the
 * sphere), a = 0.5 and 1.5 m, rho = 1000 kg/m^3, g = 9.81 m/s^2, no ramp.
 *
 * Output per case: "time z_cg Fz" with Fz the vertical pressure force
 * (static + incident-wave pressure on the instantaneous wetted surface) [N].
 * The header also records the mesh size and the mean wall time per Compute()
 * call [us] (information only; not checked).
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/hydro/force_components/nonlinear_froude_krylov_component.h>
#include <seastack/hydro/geometry/mesh_io.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include "../../unit/test_mesh_shapes.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

using seastack::hydro::NonlinearFroudeKrylovComponent;

constexpr double kRho = 1000.0;
constexpr double kG = 9.81;
constexpr double kHeave = 1.5;      // prescribed heave amplitude [m]
constexpr int kSamples = 81;        // over two wave periods
constexpr int kTimingSweeps = 50;   // repeats of the 81 samples for the timing

struct Body {
    const char* name;
    seastack::hydro::geometry::TriangleMesh mesh;  // CG frame
    double z_mean;                                 // mean CG height [m]
};

bool RunCase(const Body& body, double period, double amplitude, const std::string& path) {
    const double omega = 2.0 * std::acos(-1.0) / period;  // 2 pi / T
    seastack::hydro::WaveComponent c;
    c.amplitude = amplitude;
    c.omega = omega;
    c.k = omega * omega / kG;
    c.direction = 0.0;
    c.phase = 0.0;
    auto wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
        std::vector<seastack::hydro::WaveComponent>{c}, 0.0);

    std::vector<NonlinearFroudeKrylovComponent::BodySetup> setups;
    setups.push_back({0, body.mesh, seastack::hydro::FroudeKrylovPressureModel::kAiryWheeler,
                      Eigen::Vector3d(0.0, 0.0, body.z_mean)});
    NonlinearFroudeKrylovComponent fk(std::move(setups), wave, kRho, Eigen::Vector3d(0.0, 0.0, -kG), 0.0,
                                      NonlinearFroudeKrylovComponent::RampShape::kCosine);

    seastack::hydro::SystemState state;
    state.bodies.resize(1);

    // Wall time per Compute() call over repeated sweeps of the same samples, for
    // the cost comparison with nlfk_reference.py. One wave component on a mesh
    // of a few thousand vertices is below the OpenMP work threshold, so this is
    // single-threaded.
    double compute_us_per_call = 0.0;
    {
        seastack::hydro::BodyForces scratch(1);
        const auto start = std::chrono::steady_clock::now();
        for (int sweep = 0; sweep < kTimingSweeps; ++sweep) {
            for (int i = 0; i < kSamples; ++i) {
                const double t = 2.0 * period * i / (kSamples - 1);
                state.bodies[0].position =
                    Eigen::Vector3d(0.0, 0.0, body.z_mean + kHeave * std::sin(0.37 * omega * t));
                fk.Compute(state, t, scratch);
            }
        }
        const std::chrono::duration<double, std::micro> elapsed = std::chrono::steady_clock::now() - start;
        compute_us_per_call = elapsed.count() / (kTimingSweeps * kSamples);
    }

    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: cannot write " << path << "\n";
        return false;
    }
    out << std::setprecision(12) << "# body " << body.name << "\n# period_s " << period << "\n# wave_amplitude_m "
        << amplitude << "\n# heave_amplitude_m " << kHeave << "\n# z_mean_m " << body.z_mean
        << "\n# mesh_vertices " << body.mesh.num_vertices() << "\n# mesh_triangles " << body.mesh.num_faces()
        << "\n# compute_us_per_call " << std::setprecision(4) << compute_us_per_call
        << std::setprecision(12) << "\n# Time(s)  z_cg(m)  Fz(N)\n";
    for (int i = 0; i < kSamples; ++i) {
        const double t = 2.0 * period * i / (kSamples - 1);
        state.bodies[0].position = Eigen::Vector3d(0.0, 0.0, body.z_mean + kHeave * std::sin(0.37 * omega * t));
        seastack::hydro::BodyForces forces(1);
        fk.Compute(state, t, forces);
        out << std::setprecision(10) << t << " " << state.bodies[0].position.z() << " "
            << std::setprecision(12) << fk.GetLastPressureForce(0)[2] << "\n";
    }
    return true;
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    const std::filesystem::path data(seastack::chrono::GetDataDir());
    const auto obj = (data / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    // sphere.obj is in the BEM frame (centre at the origin); the CG is 2 m below.
    auto sphere = seastack::hydro::geometry::LoadObjMesh(obj);
    sphere.vertices.col(2).array() += 2.0;
    const std::vector<Body> bodies = {
        {"sphere", sphere, -2.0},
        {"cylinder", seastack::test::MakeVerticalCylinder(3.0, -4.0, 3.0, 128, 28), 0.0}};

    for (const auto& body : bodies) {
        for (double period : {12.0, 20.0, 30.0}) {
            for (double amplitude : {0.5, 1.5}) {
                char name[96];
                std::snprintf(name, sizeof(name), "%s_T%02d_a%.1f.txt", body.name, static_cast<int>(period),
                              amplitude);
                std::string file(name);
                file[file.find('.')] = 'p';
                std::cout << file << "\n";
                if (!RunCase(body, period, amplitude, out_dir + "/" + file)) return 1;
            }
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
