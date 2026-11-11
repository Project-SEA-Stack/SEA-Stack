/**
 * Internal method comparison: IRF-convolution vs frequency-domain excitation
 * for an oblique long-crested sea on the OSWEC (BEM headings 0..90 deg).
 *
 * The IRF path convolves the elevation at the origin, which carries no
 * direction, so it must use the excitation kernel for the sea's heading.
 * Runs three cases with the same JONSWAP sea and writes flap pitch [deg]:
 *   irf_0.txt   IRF, direction 0 deg
 *   irf_30.txt  IRF, direction 30 deg
 *   fd_30.txt   frequency domain, direction 30 deg
 * The Python check requires irf_30 ~ fd_30, and irf_30 clearly different
 * from irf_0 (the heading must not be ignored).
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/core/math_constants.h>
#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

namespace {

struct RunResult {
    std::vector<double> time;
    std::vector<double> pitch_deg;
};

RunResult RunOswec(const std::filesystem::path& data_dir, double direction_deg, ExcitationMethod method,
                   const std::string& label) {
    const auto flap_mesh = (data_dir / "demos" / "oswec" / "geometry" / "flap.obj").lexically_normal().generic_string();
    const auto base_mesh = (data_dir / "demos" / "oswec" / "geometry" / "base.obj").lexically_normal().generic_string();
    const auto h5        = (data_dir / "demos" / "oswec" / "hydroData" / "oswec.h5").lexically_normal().generic_string();

    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    system.SetSolverType(ChSolver::Type::GMRES);
    const double timestep = 0.015;
    const double duration = seastack::chrono::GetSimDuration(200.0, 400.0);

    auto flap = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 1000, false, true, false);
    system.Add(flap);
    flap->SetName("body1");
    flap->SetPos(ChVector3d(0, 0, -3.9));
    flap->SetMass(127000.0);
    flap->SetInertiaXX(ChVector3d(1.85e6, 1.85e6, 1.85e6));

    auto base = chrono_types::make_shared<ChBodyEasyMesh>(base_mesh, 1000, false, true, false);
    system.Add(base);
    base->SetName("body2");
    base->SetPos(ChVector3d(0, 0, -10.15));
    base->SetMass(999);
    base->SetInertiaXX(ChVector3d(1, 1, 1));

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -10.15));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto anchor = chrono_types::make_shared<ChLinkMateGeneric>();
    anchor->Initialize(base, ground, false, base->GetVisualModelFrame(), base->GetVisualModelFrame());
    anchor->SetConstrainedCoords(true, true, true, true, true, true);
    system.Add(anchor);

    // Hinge about +Y at z = -8.9 m, with a linear rotary damper (PTO).
    const ChQuaternion<> hinge_rot = QuatFromAngleX(CH_PI / 2.0);
    auto hinge                     = chrono_types::make_shared<ChLinkLockRevolute>();
    hinge->Initialize(base, flap, ChFramed(ChVector3d(0.0, 0.0, -8.9), hinge_rot));
    system.AddLink(hinge);
    auto pto = chrono_types::make_shared<ChLinkRSDA>();
    pto->Initialize(flap, base, ChFramed(ChVector3d(0.0, 0.0, -8.9), hinge_rot));
    pto->SetSpringCoefficient(0.0);
    pto->SetDampingCoefficient(12.0e6);
    system.AddLink(pto);

    SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    SeaStatePartition partition;
    partition.spectrum.type               = "jonswap";
    partition.spectrum.Hs                 = 2.5;
    partition.spectrum.Tp                 = 8.0;
    partition.spectrum.gamma              = 1.0;
    partition.spreading.type              = "none";
    partition.spreading.mean_direction_deg = direction_deg;
    sea_state.partitions.push_back(partition);
    sea_state.omega_min = 2.0 * M_PI * 0.001;
    sea_state.omega_max = 2.0 * M_PI * 1.0;
    sea_state.n_omega   = 1000;
    sea_state.seed      = 1;

    auto wave_field = std::make_shared<LinearDirectionalWaveField>(ComponentSampler::Build(sea_state), sea_state.depth);
    wave_field->SetRampDuration(30.0);

    std::vector<std::shared_ptr<ChBody>> bodies = {flap, base};
    HydroSystem hydro(bodies, h5);
    hydro.SetExcitationMethod(method);
    hydro.AddWaves(wave_field);

    RunResult result;
    const auto start = std::chrono::high_resolution_clock::now();
    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(timestep);
        result.time.push_back(system.GetChTime());
        result.pitch_deg.push_back(flap->GetRot().GetCardanAnglesXYZ().y() * 180.0 / CH_PI);
    }
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
    std::cout << "  " << label << " completed in " << ms << " ms\n";
    return result;
}

bool WriteResults(const RunResult& r, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "Error: cannot write " << path << "\n";
        return false;
    }
    f << "# Time(s)  FlapPitch(deg)\n";
    for (size_t i = 0; i < r.time.size(); ++i) {
        f << std::fixed << std::setprecision(6) << r.time[i] << " " << r.pitch_deg[i] << "\n";
    }
    return true;
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    const std::filesystem::path data_path(seastack::chrono::GetDataDir());

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    const auto irf_0  = RunOswec(data_path, 0.0, ExcitationMethod::kIrfConvolution, "IRF 0 deg");
    const auto irf_30 = RunOswec(data_path, 30.0, ExcitationMethod::kIrfConvolution, "IRF 30 deg");
    const auto fd_30  = RunOswec(data_path, 30.0, ExcitationMethod::kFrequencyDomain, "FD 30 deg");

    const bool ok = WriteResults(irf_0, out_dir + "/irf_0.txt") && WriteResults(irf_30, out_dir + "/irf_30.txt") &&
                    WriteResults(fd_30, out_dir + "/fd_30.txt");
    return ok ? 0 : 1;
}
