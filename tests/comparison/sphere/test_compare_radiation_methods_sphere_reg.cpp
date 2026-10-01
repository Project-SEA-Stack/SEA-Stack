/**
 * Internal method comparison: radiation methods on the IEA sphere in regular
 * waves (heave only, linear PTO damper).
 *
 * At five BEM grid frequencies the same case is run with
 *   A) RIRF convolution (default)
 *   B) state-space radiation
 *   C) constant coefficients A(w), B(w)
 * and the heave time series are written for compare_*.py, which fits the
 * steady-state response and compares it with the single-DOF frequency-domain
 * solution  Z = a X3 / (C33 - w^2 (M + A33(w)) + i w (B33(w) + Bpto)).
 *
 * The dimensional coefficients used by that solution are written to
 * reference.txt (from HydroData: A = rho*A_file, B = rho*w*B_file,
 * C = rho*g*C_file, X = rho*g*|X_file| exp(i phase)).
 *
 * Conventions: eta = a cos(w t) at x = 0; F_exc = Re{a X exp(i w t)};
 * heave z positive up [m], measured from the initial (equilibrium) position.
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>
#include <seastack/hydro_io/h5_reader.h>

#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChLinkLock.h>
#include <chrono/physics/ChLinkTSDA.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/timestepper/ChTimestepperHHT.h>
#include <chrono/utils/ChConstants.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace chrono;
using seastack::chrono::HydroSystem;
using seastack::hydro::RadiationMethod;

namespace {

constexpr double kMass = 261.8e3;          // [kg]
constexpr double kPtoDamping = 1.0e5;      // [N s/m]
constexpr double kWaveAmplitude = 0.5;     // [m]
constexpr double kTimeStep = 0.01;         // [s]
constexpr double kRampDuration = 20.0;     // [s]
// BEM grid indices into sphere.h5 (w = 0.05 (k+1) rad/s): 0.5, 1.0, 1.5, 2.0, 2.5.
const int kGridIndices[] = {9, 19, 29, 39, 49};

struct Method {
    RadiationMethod method;
    const char* tag;
};
const Method kMethods[] = {{RadiationMethod::kRirfConvolution, "rirf"},
                           {RadiationMethod::kStateSpace, "state_space"},
                           {RadiationMethod::kConstantCoefficients, "constant_coefficients"}};

/// Steady state needs the start-up transient to decay. With constant
/// coefficients it decays with B(w) at the wave frequency (damping ratio
/// ~0.02 at 2.5 rad/s without PTO), so allow at least 400 s after the ramp.
double Duration(double omega) {
    const double period = CH_2PI / omega;
    return kRampDuration + std::max(30.0 * period, 400.0);
}

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

bool RunCase(const std::string& h5, RadiationMethod method, double omega, const std::string& path) {
    ChSystemNSC system;
    ConfigureSystem(system);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0, 0, -5));
    ground->SetFixed(true);
    system.AddBody(ground);

    auto body = chrono_types::make_shared<ChBody>();
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, 0));
    body->SetMass(kMass);
    system.AddBody(body);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false, ChFramed(ChVector3d(0, 0, 0)),
                          ChFramed(ChVector3d(0, 0, -5)));
    system.AddLink(prismatic);

    auto pto = chrono_types::make_shared<ChLinkTSDA>();
    pto->Initialize(body, ground, false, ChVector3d(0, 0, 0), ChVector3d(0, 0, -5));
    pto->SetSpringCoefficient(0.0);
    pto->SetDampingCoefficient(kPtoDamping);
    system.AddLink(pto);

    seastack::hydro::SeaStateDefinition sea_state;
    sea_state.type = "regular";
    sea_state.amplitude = kWaveAmplitude;
    sea_state.omega = omega;
    auto wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
        seastack::hydro::ComponentSampler::Build(sea_state), sea_state.depth);
    wave->SetRampDuration(kRampDuration);

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    HydroSystem hydro(bodies, h5);
    hydro.SetRadiationMethod(method);
    hydro.AddWaves(wave);

    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: cannot write " << path << "\n";
        return false;
    }
    out << "# omega_rad_s " << std::setprecision(12) << omega << "\n";
    out << "# Time(s)  Heave(m)  HeaveVelocity(m/s)\n";
    const double z0 = body->GetPos().z();
    const double duration = Duration(omega);
    while (system.GetChTime() < duration - kTimeStep / 2.0) {
        system.DoStepDynamics(kTimeStep);
        out << std::fixed << std::setprecision(4) << system.GetChTime() << " "
            << std::setprecision(9) << body->GetPos().z() - z0 << " " << body->GetPosDt().z()
            << "\n";
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

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    const auto data = seastack::hydro_io::H5FileInfo(h5, 1).ReadH5Data();
    const Eigen::VectorXd omegas = data.GetBemFrequencies();
    const auto& exc = data.GetRegularWaveInfos()[0];

    std::ofstream ref(out_dir + "/reference.txt");
    ref << "# Single-DOF heave reference coefficients (dimensional, heading 0)\n"
        << "# mass_kg " << kMass << "\n# pto_damping_Ns_m " << kPtoDamping << "\n"
        << "# wave_amplitude_m " << kWaveAmplitude << "\n"
        << "# k omega A33 B33 C33 X3_re X3_im (all BEM grid frequencies)\n";
    for (int k = 0; k < omegas.size(); ++k) {
        const double w = omegas[k];
        const double mag = exc.excitation_mag_matrix(2, 0, k);
        const double phase = exc.excitation_phase_matrix(2, 0, k);
        ref << k << " " << std::setprecision(12) << w << " "
            << data.GetAddedMassAtFrequencyIndex(k)(2, 2) << " "
            << data.GetRadiationDampingAtFrequencyIndex(k)(2, 2) << " "
            << data.GetHydrostaticStiffnessVal(0, 2, 2) << " " << mag * std::cos(phase) << " "
            << mag * std::sin(phase) << "\n";
    }
    ref.close();

    for (int k : kGridIndices) {
        for (const auto& m : kMethods) {
            const double w = omegas[k];
            std::cout << "w = " << w << " rad/s, " << m.tag << "...\n";
            const auto t0 = std::chrono::high_resolution_clock::now();
            const std::string path = out_dir + "/" + m.tag + "_k" + std::to_string(k) + ".txt";
            if (!RunCase(h5, m.method, w, path)) return 1;
            const auto t1 = std::chrono::high_resolution_clock::now();
            std::cout << "  done in "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " ms\n";
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
