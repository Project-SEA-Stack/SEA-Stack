/**
 * Internal method comparison: linear vs nonlinear hydrostatics on the IEA
 * sphere in regular waves (heave only, no PTO).
 *
 * The same case is run with
 *   A) linear hydrostatics (BEM stiffness from H5)
 *   B) nonlinear hydrostatics (submerged volume of the OBJ mesh below the
 *      calm waterplane z = 0)
 * at several wave amplitudes. Wave excitation is linear in both runs.
 *
 * At small amplitude the two must agree (compare_*.py checks this). At larger
 * amplitude they diverge: B uses a calm-water buoyancy while the excitation
 * still contains the linear Froude-Krylov force on the mean hull, so B is only
 * valid while the relative motion stays well below the draft and freeboard.
 *
 * Conventions: eta = a cos(w t) at x = 0; heave z positive up [m], measured
 * from the equilibrium CG height read from the H5 file.
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro_io/h5_reader.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include <chrono/utils/ChConstants.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using namespace chrono;
using seastack::chrono::HydroSystem;

namespace {

constexpr double kMass = 261.8e3;         // [kg]
constexpr double kWavePeriod = 8.0;       // [s]
constexpr double kTimeStep = 0.01;        // [s]
constexpr double kRampDuration = 20.0;    // [s]
constexpr double kSteadyPeriods = 15.0;   // wave periods simulated after the ramp

struct AmplitudeCase {
    double amplitude;  // [m]
    const char* tag;
};
const AmplitudeCase kCases[] = {{0.1, "a0p1"}, {1.0, "a1p0"}, {2.5, "a2p5"}};

void ConfigureSystem(ChSystemNSC& system) {
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    hht->SetAlpha(-0.2);
    hht->SetMaxIters(50);
    hht->SetRelTolerance(1e-6);
    hht->SetAbsTolerances(1e-6, 1e2);
    hht->SetStepControl(false);
}

bool RunCase(const std::string& h5, const std::string& mesh, bool nonlinear, double amplitude,
             double z_eq, const std::string& path) {
    ChSystemNSC system;
    ConfigureSystem(system);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0, 0, -10));
    ground->SetFixed(true);
    system.AddBody(ground);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, z_eq));
    body->SetMass(kMass);
    system.AddBody(body);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false, ChFramed(ChVector3d(0, 0, z_eq)),
                          ChFramed(ChVector3d(0, 0, -10)));
    system.AddLink(prismatic);

    seastack::hydro::SeaStateDefinition sea_state;
    sea_state.type = "regular";
    sea_state.amplitude = amplitude;
    sea_state.omega = CH_2PI / kWavePeriod;
    auto wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
        seastack::hydro::ComponentSampler::Build(sea_state), sea_state.depth);
    wave->SetRampDuration(kRampDuration);

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    HydroSystem hydro(bodies, h5);
    if (nonlinear) {
        hydro.EnableNonlinearHydrostatics();
        hydro.SetBodyMeshFiles({mesh});
    }
    hydro.AddWaves(wave);

    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: cannot write " << path << "\n";
        return false;
    }
    out << "# wave_amplitude_m " << amplitude << "\n";
    out << "# omega_rad_s " << std::setprecision(12) << sea_state.omega << "\n";
    out << "# Time(s)  Heave(m)\n";
    const double duration = kRampDuration + kSteadyPeriods * kWavePeriod;
    while (system.GetChTime() < duration - kTimeStep / 2.0) {
        system.DoStepDynamics(kTimeStep);
        out << std::fixed << std::setprecision(4) << system.GetChTime() << " "
            << std::setprecision(9) << body->GetPos().z() - z_eq << "\n";
    }
    return true;
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    const auto h5 =
        (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();
    const auto mesh =
        (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    const auto data = seastack::hydro_io::H5FileInfo(h5, 1).ReadH5Data();
    const double z_eq = data.GetCGVector(0)[2];

    for (const auto& c : kCases) {
        for (bool nonlinear : {false, true}) {
            const char* method = nonlinear ? "nonlinear" : "linear";
            std::cout << "a = " << c.amplitude << " m, " << method << "...\n";
            const auto t0 = std::chrono::high_resolution_clock::now();
            const std::string path = out_dir + "/" + c.tag + "_" + method + "_heave.txt";
            if (!RunCase(h5, mesh, nonlinear, c.amplitude, z_eq, path)) return 1;
            const auto t1 = std::chrono::high_resolution_clock::now();
            std::cout << "  done in "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " ms\n";
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
