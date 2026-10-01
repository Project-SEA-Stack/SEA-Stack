/**
 * Internal method comparison: radiation methods on RM3 in regular waves
 * (two bodies, heave only, linear PTO damper between float and plate).
 *
 * Layout: the plate (body2) slides on a vertical prismatic joint to ground and
 * the float (body1) slides on a vertical prismatic joint to the plate, so the
 * motion is exactly two heave DOFs. At five BEM grid frequencies the same case
 * is run with
 *   A) RIRF convolution (default)
 *   B) state-space radiation (default fit options)
 *   C) constant coefficients A(w), B(w) (full coupled 12x12 matrices)
 * and the heave time series are written for compare_*.py, which fits the
 * steady-state response and compares it with the coupled 2x2 frequency-domain
 * solution  Z = [C - w^2 (M + A(w)) + i w (B(w) + B_pto)]^-1 a X.
 *
 * The dimensional heave sub-matrices used by that solution are written to
 * reference.txt (from HydroData: A = rho*A_file, B = rho*w*B_file,
 * C = rho*g*C_file, X = rho*g*|X_file| exp(i phase)). Index 2 is float heave,
 * index 8 is plate heave in the 12x12 matrices.
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

constexpr double kFloatMass = 725834.0;    // [kg]
constexpr double kPlateMass = 886691.0;    // [kg]
constexpr double kFloatZ = -0.72;          // [m] float CG
constexpr double kPlateZ = -21.29;         // [m] plate CG
constexpr double kPtoDamping = 1.2e6;      // [N s/m]
constexpr double kWaveAmplitude = 0.5;     // [m]
constexpr double kTimeStep = 0.01;         // [s]
constexpr double kRampDuration = 20.0;     // [s]
// BEM grid indices into rm3.h5 (w = 0.02 (k+1) rad/s): 0.4, 0.6, 0.8, 1.0, 1.2.
const int kGridIndices[] = {19, 29, 39, 49, 59};

struct Method {
    RadiationMethod method;
    const char* tag;
};
const Method kMethods[] = {{RadiationMethod::kRirfConvolution, "rirf"},
                           {RadiationMethod::kStateSpace, "state_space"},
                           {RadiationMethod::kConstantCoefficients, "constant_coefficients"}};

/// With constant coefficients the start-up transient decays with B(w) at the
/// wave frequency, so allow at least 400 s after the ramp for steady state.
double Duration(double omega) {
    const double period = CH_2PI / omega;
    return kRampDuration + std::max(30.0 * period, 400.0);
}

void ConfigureSystem(ChSystemNSC& system) {
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    // The ground-plate-float prismatic chain can diverge with unpreconditioned
    // GMRES; use a direct solver.
    system.SetSolverType(ChSolver::Type::SPARSE_LU);
    system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht = std::static_pointer_cast<ChTimestepperHHT>(system.GetTimestepper());
    hht->SetAlpha(-0.2);
    hht->SetMaxIters(50);
    hht->SetRelTolerance(1e-6);
    hht->SetAbsTolerances(1e-6, 1e2);
    hht->SetStepControl(false);
}

bool RunCase(const std::string& h5, const Method& method, double omega, const std::string& path) {
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
    pto->Initialize(float_body, plate_body, false, ChVector3d(0, 0, kFloatZ),
                    ChVector3d(0, 0, kPlateZ));
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

    std::vector<std::shared_ptr<ChBody>> bodies = {float_body, plate_body};
    HydroSystem hydro(bodies, h5);
    hydro.SetRadiationMethod(method.method);
    hydro.AddWaves(wave);

    std::ofstream out(path);
    if (!out) {
        std::cerr << "Error: cannot write " << path << "\n";
        return false;
    }
    out << "# omega_rad_s " << std::setprecision(12) << omega << "\n";
    out << "# Time(s)  FloatHeave(m)  PlateHeave(m)  FloatHeaveVel(m/s)  PlateHeaveVel(m/s)\n";
    const double z1 = float_body->GetPos().z();
    const double z2 = plate_body->GetPos().z();
    const double duration = Duration(omega);
    while (system.GetChTime() < duration - kTimeStep / 2.0) {
        system.DoStepDynamics(kTimeStep);
        out << std::fixed << std::setprecision(4) << system.GetChTime() << " "
            << std::setprecision(9) << float_body->GetPos().z() - z1 << " "
            << plate_body->GetPos().z() - z2 << " " << float_body->GetPosDt().z() << " "
            << plate_body->GetPosDt().z() << "\n";
    }
    return true;
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    const auto h5 =
        (DATADIR / "demos" / "rm3" / "hydroData" / "rm3.h5").lexically_normal().generic_string();
    if (!std::filesystem::exists(h5)) {
        std::cerr << "ERROR: rm3.h5 not found at " << h5 << "\n";
        return 1;
    }

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    const auto data = seastack::hydro_io::H5FileInfo(h5, 2).ReadH5Data();
    const Eigen::VectorXd omegas = data.GetBemFrequencies();
    const auto& exc_float = data.GetRegularWaveInfos()[0];
    const auto& exc_plate = data.GetRegularWaveInfos()[1];

    std::ofstream ref(out_dir + "/reference.txt");
    ref << "# Coupled two-body heave reference coefficients (dimensional, heading 0)\n"
        << "# float_mass_kg " << kFloatMass << "\n# plate_mass_kg " << kPlateMass << "\n"
        << "# pto_damping_Ns_m " << kPtoDamping << "\n"
        << "# wave_amplitude_m " << kWaveAmplitude << "\n"
        << "# k omega A22 A28 A82 A88 B22 B28 B82 B88 C_float C_plate"
        << " X_float_re X_float_im X_plate_re X_plate_im (all BEM grid frequencies)\n";
    for (int k = 0; k < omegas.size(); ++k) {
        const Eigen::MatrixXd a = data.GetAddedMassAtFrequencyIndex(k);
        const Eigen::MatrixXd b = data.GetRadiationDampingAtFrequencyIndex(k);
        const double m1 = exc_float.excitation_mag_matrix(2, 0, k);
        const double p1 = exc_float.excitation_phase_matrix(2, 0, k);
        const double m2 = exc_plate.excitation_mag_matrix(2, 0, k);
        const double p2 = exc_plate.excitation_phase_matrix(2, 0, k);
        ref << k << " " << std::setprecision(12) << omegas[k] << " " << a(2, 2) << " " << a(2, 8)
            << " " << a(8, 2) << " " << a(8, 8) << " " << b(2, 2) << " " << b(2, 8) << " "
            << b(8, 2) << " " << b(8, 8) << " " << data.GetHydrostaticStiffnessVal(0, 2, 2) << " "
            << data.GetHydrostaticStiffnessVal(1, 2, 2) << " " << m1 * std::cos(p1) << " "
            << m1 * std::sin(p1) << " " << m2 * std::cos(p2) << " " << m2 * std::sin(p2) << "\n";
    }
    ref.close();

    for (int k : kGridIndices) {
        for (const auto& m : kMethods) {
            const double w = omegas[k];
            std::cout << "w = " << w << " rad/s, " << m.tag << "...\n";
            const auto t0 = std::chrono::high_resolution_clock::now();
            const std::string path = out_dir + "/" + m.tag + "_k" + std::to_string(k) + ".txt";
            if (!RunCase(h5, m, w, path)) return 1;
            const auto t1 = std::chrono::high_resolution_clock::now();
            std::cout << "  done in "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " ms\n";
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
