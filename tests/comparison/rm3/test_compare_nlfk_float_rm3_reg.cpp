/**
 * C5 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): RM3 in small regular
 * waves (steepness H / lambda = 0.0005), two heave DOFs with a linear PTO
 * damper, constant-coefficient radiation, at w = 0.4 to 1.2 rad/s in steps of
 * 0.1 rad/s (wave periods 5.2 to 15.7 s, inside the rm3.h5 BEM range).
 *
 * Runs: all linear, and float with nonlinear hydrostatics + NL FK (Airy +
 * Wheeler) while the plate stays linear (per-body toggle). At this steepness
 * the two must agree (compare_*.py).
 *
 * Layout and parameters as in test_compare_radiation_methods_rm3_reg.cpp.
 * Conventions: eta = a cos(w t) at x = 0; heave positive up [m] from the
 * initial position.
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include <chrono/utils/ChConstants.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace chrono;
using seastack::chrono::HydroSystem;

namespace {

constexpr double kFloatMass = 725834.0;    // [kg]
constexpr double kPlateMass = 886691.0;    // [kg]
constexpr double kFloatZ = -0.72;          // [m] float CG
constexpr double kPlateZ = -21.29;         // [m] plate CG
constexpr double kPtoDamping = 1.2e6;      // [N s/m]
constexpr double kSteepness = 0.0005;      // H / lambda
constexpr double kTimeStep = 0.01;         // [s]
constexpr double kRampDuration = 20.0;     // [s]
constexpr double kG = 9.81;

/// Constant-coefficient transients decay with B(w); allow 400 s after the ramp.
double Duration(double omega) { return kRampDuration + std::max(30.0 * CH_2PI / omega, 400.0); }

void ConfigureSystem(ChSystemNSC& system) {
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -kG));
    system.SetSolverType(ChSolver::Type::SPARSE_LU);
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    hht->SetAlpha(-0.2);
    hht->SetMaxIters(50);
    hht->SetRelTolerance(1e-6);
    hht->SetAbsTolerances(1e-6, 1e2);
    hht->SetStepControl(false);
}

bool RunCase(const std::string& h5, const std::string& float_mesh, bool nlfk, double omega, double amplitude,
             const std::string& path) {
    ChSystemNSC system;
    ConfigureSystem(system);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0, 0, kPlateZ));
    ground->SetFixed(true);
    system.AddBody(ground);

    auto float_body = chrono_types::make_shared<ChBody>();
    float_body->SetName("body1");
    float_body->SetPos(ChVector3d(0, 0, kFloatZ));
    float_body->SetMass(kFloatMass);
    float_body->SetInertiaXX(ChVector3d(20907301.0, 21306090.66, 37085481.11));
    system.AddBody(float_body);

    auto plate_body = chrono_types::make_shared<ChBody>();
    plate_body->SetName("body2");
    plate_body->SetPos(ChVector3d(0, 0, kPlateZ));
    plate_body->SetMass(kPlateMass);
    plate_body->SetInertiaXX(ChVector3d(94419614.57, 94407091.24, 28542224.82));
    system.AddBody(plate_body);

    auto plate_ground = chrono_types::make_shared<ChLinkLockPrismatic>();
    plate_ground->Initialize(plate_body, ground, false, ChFramed(ChVector3d(0, 0, kPlateZ)),
                             ChFramed(ChVector3d(0, 0, kPlateZ)));
    system.AddLink(plate_ground);

    auto float_plate = chrono_types::make_shared<ChLinkLockPrismatic>();
    float_plate->Initialize(float_body, plate_body, false, ChFramed(ChVector3d(0, 0, kFloatZ)),
                            ChFramed(ChVector3d(0, 0, kPlateZ)));
    system.AddLink(float_plate);

    auto pto = chrono_types::make_shared<ChLinkTSDA>();
    pto->Initialize(float_body, plate_body, false, ChVector3d(0, 0, kFloatZ), ChVector3d(0, 0, kPlateZ));
    pto->SetSpringCoefficient(0.0);
    pto->SetDampingCoefficient(kPtoDamping);
    system.AddLink(pto);

    seastack::hydro::SeaStateDefinition sea_state;
    sea_state.type = "regular";
    sea_state.amplitude = amplitude;
    sea_state.omega = omega;
    auto wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
        seastack::hydro::ComponentSampler::Build(sea_state), sea_state.depth);
    wave->SetRampDuration(kRampDuration);

    std::vector<std::shared_ptr<ChBody>> bodies = {float_body, plate_body};
    HydroSystem hydro(bodies, h5);
    hydro.SetRadiationMethod(seastack::hydro::RadiationMethod::kConstantCoefficients);
    if (nlfk) {
        seastack::hydro::HydroModelBuilder::BodyHydrostaticsConfig float_cfg, plate_cfg;
        float_cfg.model = seastack::hydro::HydrostaticsModel::kNonlinear;
        float_cfg.mesh_file = float_mesh;
        float_cfg.froude_krylov = seastack::hydro::FroudeKrylovModel::kNonlinear;
        hydro.SetBodyHydrostatics({float_cfg, plate_cfg});
    }
    hydro.AddWaves(wave);

    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: cannot write " << path << "\n";
        return false;
    }
    out << std::setprecision(12) << "# omega_rad_s " << omega << "\n# wave_amplitude_m " << amplitude << "\n";
    out << "# Time(s)  FloatHeave(m)  PlateHeave(m)\n";
    const double z1 = float_body->GetPos().z();
    const double z2 = plate_body->GetPos().z();
    const double duration = Duration(omega);
    while (system.GetChTime() < duration - kTimeStep / 2.0) {
        system.DoStepDynamics(kTimeStep);
        out << std::fixed << std::setprecision(4) << system.GetChTime() << " " << std::setprecision(9)
            << float_body->GetPos().z() - z1 << " " << plate_body->GetPos().z() - z2 << "\n";
    }
    return true;
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    const std::filesystem::path data(seastack::chrono::GetDataDir());
    const auto h5 = (data / "demos" / "rm3" / "hydroData" / "rm3.h5").lexically_normal().generic_string();
    const auto float_mesh = (data / "demos" / "rm3" / "geometry" / "float.obj").lexically_normal().generic_string();

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    for (double omega : {0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2}) {
        // Deep-water wavelength: the default sea state is deep water.
        const double amplitude = 0.5 * kSteepness * 2.0 * CH_PI * kG / (omega * omega);
        for (bool nlfk : {false, true}) {
            char name[64];
            std::snprintf(name, sizeof(name), "w%.1f_%s.txt", omega, nlfk ? "float_nlfk" : "linear");
            std::string file(name);
            file[file.find('.')] = 'p';
            std::cout << file << "...\n";
            if (!RunCase(h5, float_mesh, nlfk, omega, amplitude, out_dir + "/" + file)) return 1;
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
