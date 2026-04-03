/**
 * Internal method comparison: linear vs nonlinear hydrostatics
 * on sphere free decay in heave.
 *
 * Runs the same sphere decay scenario for two amplitudes (1 m and 5 m drop):
 *   A) Linear hydrostatics (BEM stiffness matrix from H5)
 *   B) Nonlinear hydrostatics (instantaneous submerged volume from OBJ mesh)
 *
 * Heave is recorded as displacement from equilibrium (z - z_eq). At 1 m amplitude
 * the two methods should nearly agree; at 5 m the nonlinear effects are visible.
 */

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro_io/h5_reader.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

struct RunResult {
    std::vector<double> time;
    std::vector<double> heave;
};

static RunResult run_linear(const std::string& h5, double z_eq, double z0,
                            double duration) {
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.005;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -5));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto body = chrono_types::make_shared<ChBody>();
    system.Add(body);
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, z0));
    body->SetMass(261.8e3);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false,
                          ChFramed(ChVector3d(0, 0, z0)),
                          ChFramed(ChVector3d(0, 0, -5)));
    system.AddLink(prismatic);

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    HydroSystem hydro(bodies, h5);

    RunResult result;
    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(timestep);
        result.time.push_back(system.GetChTime());
        result.heave.push_back(body->GetPos().z() - z_eq);
    }

    return result;
}

static RunResult run_nonlinear(const std::string& mesh,
                               const std::string& h5,
                               double z_eq, double z0,
                               double duration) {
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.005;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -5));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto body = chrono_types::make_shared<ChBody>();
    system.Add(body);
    body->SetName("body1");
    body->SetPos(ChVector3d(0, 0, z0));
    body->SetMass(261.8e3);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(body, ground, false,
                          ChFramed(ChVector3d(0, 0, z0)),
                          ChFramed(ChVector3d(0, 0, -5)));
    system.AddLink(prismatic);

    std::vector<std::shared_ptr<ChBody>> bodies = {body};
    HydroSystem hydro(bodies, h5);
    hydro.EnableNonlinearHydrostatics();
    hydro.SetBodyMeshFiles({mesh});

    RunResult result;
    while (system.GetChTime() <= duration) {
        system.DoStepDynamics(timestep);
        result.time.push_back(system.GetChTime());
        result.heave.push_back(body->GetPos().z() - z_eq);
    }

    return result;
}

static void write_results(const std::vector<double>& time,
                          const std::vector<double>& signal,
                          const std::string& path,
                          const std::string& header) {
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "Error: cannot write " << path << "\n";
        return;
    }
    f << header << "\n";
    for (size_t i = 0; i < time.size(); ++i) {
        f << std::left << std::setw(14) << std::fixed << std::setprecision(6) << time[i]
          << std::right << std::setw(14) << std::fixed << std::setprecision(6) << signal[i]
          << "\n";
    }
}

int main(int argc, char* argv[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto mesh = (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj")
                    .lexically_normal().generic_string();
    auto h5 = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5")
                  .lexically_normal().generic_string();

    auto h5data = seastack::hydro_io::H5FileInfo(h5, 1).ReadH5Data();
    double z_eq = h5data.GetCGVector(0)[2];

    double duration = seastack::chrono::GetSimDuration(30.0, 60.0);

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);

    struct AmplitudeCase { double amp; const char* tag; };
    AmplitudeCase cases[] = {{1.0, "amp1m"}, {5.0, "amp5m"}};

    for (const auto& c : cases) {
        double z0 = z_eq + c.amp;
        std::cout << "Amplitude " << c.tag << " (z0=" << z0 << " m)...\n";

        std::cout << "  Running linear...\n";
        auto t0 = std::chrono::high_resolution_clock::now();
        auto linear = run_linear(h5, z_eq, z0, duration);
        auto t1 = std::chrono::high_resolution_clock::now();
        std::cout << "    Linear completed in "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                  << " ms\n";

        std::cout << "  Running nonlinear...\n";
        t0 = std::chrono::high_resolution_clock::now();
        auto nonlinear = run_nonlinear(mesh, h5, z_eq, z0, duration);
        t1 = std::chrono::high_resolution_clock::now();
        std::cout << "    Nonlinear completed in "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                  << " ms\n";

        std::string tag(c.tag);
        write_results(linear.time, linear.heave,
                      out_dir + "/" + tag + "_linear_heave.txt",
                      "# Time(s)       Heave(m)");
        write_results(nonlinear.time, nonlinear.heave,
                      out_dir + "/" + tag + "_nonlinear_heave.txt",
                      "# Time(s)       Heave(m)");
    }

    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
