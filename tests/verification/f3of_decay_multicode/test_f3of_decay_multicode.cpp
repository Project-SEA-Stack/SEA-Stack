#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemSMC.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

struct F3ofResult {
    std::vector<double> time;
    std::vector<double> base_surge;
    std::vector<double> base_pitch;
    std::vector<double> fore_pitch;
    std::vector<double> aft_pitch;
};

void write_output(const std::string& filepath, const F3ofResult& r) {
    std::ofstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "Error: Could not open " << filepath << std::endl;
        return;
    }
    f << std::left << std::setw(10) << "Time (s)"
      << std::right << std::setw(16) << "Base Surge (m)"
      << std::right << std::setw(16) << "Base Pitch (rad)"
      << std::right << std::setw(16) << "Flap Fore (rad)"
      << std::right << std::setw(16) << "Flap Aft (rad)" << std::endl;
    for (size_t i = 0; i < r.time.size(); ++i)
        f << std::left << std::setw(10) << std::setprecision(4) << std::fixed << r.time[i]
          << std::right << std::setw(16) << std::setprecision(6) << std::fixed << r.base_surge[i]
          << std::right << std::setw(16) << std::setprecision(6) << std::fixed << r.base_pitch[i]
          << std::right << std::setw(16) << std::setprecision(6) << std::fixed << r.fore_pitch[i]
          << std::right << std::setw(16) << std::setprecision(6) << std::fixed << r.aft_pitch[i]
          << std::endl;
    f.close();
}

F3ofResult run_decay_c1(const std::string& base_mesh, const std::string& flap_mesh,
                   const std::string& h5fname) {
    std::cout << "\n--- Decay C1: Surge decay (flaps locked) ---" << std::endl;
    F3ofResult result;

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.02;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double dur = 400.0;  // verification suite runs full 400s

    auto base_body = chrono_types::make_shared<ChBodyEasyMesh>(base_mesh, 0, false, true, false);
    system.Add(base_body);
    base_body->SetName("body1");
    base_body->SetMass(1089825.0);
    base_body->SetInertiaXX(ChVector3d(100000000.0, 76300000.0, 100000000.0));

    auto flapFore = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
    system.Add(flapFore);
    flapFore->SetName("body2");
    flapFore->SetMass(179250.0);
    flapFore->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

    auto flapAft = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
    system.Add(flapAft);
    flapAft->SetName("body3");
    flapAft->SetMass(179250.0);
    flapAft->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

    // Decay C1: base displaced 5m in surge, flaps locked
    base_body->SetPos(ChVector3d(5.0, 0.0, -9.0));
    flapFore->SetPos(ChVector3d(5.0 + -12.5, 0.0, -9.0 + 3.5));
    flapAft->SetPos(ChVector3d(5.0 + 12.5, 0.0, -9.0 + 3.5));

    ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
    auto revFore = chrono_types::make_shared<ChLinkLockRevolute>();
    revFore->Initialize(base_body, flapFore,
                        ChFramed(ChVector3d(5.0 - 12.5, 0.0, -9.0), revoluteRot));
    system.AddLink(revFore);
    auto revAft = chrono_types::make_shared<ChLinkLockRevolute>();
    revAft->Initialize(base_body, flapAft,
                       ChFramed(ChVector3d(5.0 + 12.5, 0.0, -9.0), revoluteRot));
    system.AddLink(revAft);
    revFore->Lock(true);
    revAft->Lock(true);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -9.0));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(ground, base_body,
                          ChFramed(ChVector3d(0.0, 0.0, -9.0), QuatFromAngleY(CH_PI_2)));
    system.AddLink(prismatic);

    auto pto = chrono_types::make_shared<ChLinkTSDA>();
    pto->Initialize(ground, base_body, true,
                    ChVector3d(0.0, 0.0, 0.0), ChVector3d(0.0, 0.0, 0.0));
    pto->SetSpringCoefficient(1e5);
    pto->SetRestLength(0.0);
    system.AddLink(pto);

    auto no_waves = std::make_shared<NoWave>();
    std::vector<std::shared_ptr<ChBody>> bodies = {base_body, flapFore, flapAft};
    HydroSystem hydro(bodies, h5fname, no_waves);

    auto start = std::chrono::high_resolution_clock::now();
    while (system.GetChTime() <= dur) {
        system.DoStepDynamics(timestep);
        result.time.push_back(system.GetChTime());
        result.base_surge.push_back(base_body->GetPos().x());
        result.base_pitch.push_back(base_body->GetRot().GetCardanAnglesXYZ().y());
        result.fore_pitch.push_back(flapFore->GetRot().GetCardanAnglesXYZ().y());
        result.aft_pitch.push_back(flapAft->GetRot().GetCardanAnglesXYZ().y());
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Decay C1 completed in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms" << std::endl;
    return result;
}

F3ofResult run_decay_c3(const std::string& base_mesh, const std::string& flap_mesh,
                   const std::string& h5fname, const std::string& diagnostics_out_dir) {
    std::cout << "\n--- Decay C3: Flap fore decay (base fixed) ---" << std::endl;
    F3ofResult result;

    ChSystemSMC system;
    system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
    double timestep = 0.02;
    system.SetSolverType(ChSolver::Type::SPARSE_QR);
    double dur = 400.0;  // verification suite runs full 400s

    auto base_body = chrono_types::make_shared<ChBodyEasyMesh>(base_mesh, 0, false, true, false);
    system.Add(base_body);
    base_body->SetName("body1");
    base_body->SetMass(1089825.0);
    base_body->SetInertiaXX(ChVector3d(100000000.0, 76300000.0, 100000000.0));

    auto flapFore = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
    system.Add(flapFore);
    flapFore->SetName("body2");
    flapFore->SetMass(179250.0);
    flapFore->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

    auto flapAft = chrono_types::make_shared<ChBodyEasyMesh>(flap_mesh, 0, false, true, false);
    system.Add(flapAft);
    flapAft->SetName("body3");
    flapAft->SetMass(179250.0);
    flapAft->SetInertiaXX(ChVector3d(100000000.0, 1300000.0, 100000000.0));

    // Decay C3: base fixed, fore flap rotated 10 deg
    base_body->SetPos(ChVector3d(0.0, 0.0, -9.0));
    double fore_ang = CH_PI / 18.0;
    flapFore->SetRot(QuatFromAngleY(fore_ang));
    flapFore->SetPos(ChVector3d(-12.5 + 3.5 * std::cos(CH_PI / 2.0 - fore_ang), 0.0,
                                -9.0 + 3.5 * std::sin(CH_PI / 2.0 - fore_ang)));
    flapAft->SetPos(ChVector3d(12.5 + 3.5 * std::cos(CH_PI / 2.0 - fore_ang), 0.0,
                               -9.0 + 3.5 * std::sin(CH_PI / 2.0 - fore_ang)));

    ChQuaternion<> revoluteRot = QuatFromAngleX(CH_PI / 2.0);
    auto revFore = chrono_types::make_shared<ChLinkLockRevolute>();
    revFore->Initialize(base_body, flapFore,
                        ChFramed(ChVector3d(-12.5, 0.0, -9.0), revoluteRot));
    system.AddLink(revFore);
    auto revAft = chrono_types::make_shared<ChLinkLockRevolute>();
    revAft->Initialize(base_body, flapAft,
                       ChFramed(ChVector3d(12.5, 0.0, -9.0), revoluteRot));
    system.AddLink(revAft);

    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetPos(ChVector3d(0, 0, -12.0));
    ground->SetTag(-1);
    ground->SetFixed(true);
    ground->EnableCollision(false);

    auto anchor = chrono_types::make_shared<ChLinkMateGeneric>();
    anchor->Initialize(base_body, ground, false,
                       base_body->GetVisualModelFrame(),
                       base_body->GetVisualModelFrame());
    system.Add(anchor);
    anchor->SetConstrainedCoords(true, true, true, true, true, true);

    auto no_waves = std::make_shared<NoWave>();
    std::vector<std::shared_ptr<ChBody>> bodies = {base_body, flapFore, flapAft};
    HydroSystem hydro(bodies, h5fname, no_waves);
    hydro.EnableRirfSmoothing();  // reduces tail/irregular-frequency artefacts
    hydro.SetDiagnosticsOutputDirectory(diagnostics_out_dir);

    auto start = std::chrono::high_resolution_clock::now();
    while (system.GetChTime() <= dur) {
        system.DoStepDynamics(timestep);
        result.time.push_back(system.GetChTime());
        result.base_surge.push_back(base_body->GetPos().x());
        result.base_pitch.push_back(base_body->GetRot().GetCardanAnglesXYZ().y());
        result.fore_pitch.push_back(flapFore->GetRot().GetCardanAnglesXYZ().y());
        result.aft_pitch.push_back(flapAft->GetRot().GetCardanAnglesXYZ().y());
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Decay C3 completed in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms" << std::endl;
    return result;
}

int main(int argc, char* argv[]) {
    std::cout << "=== F3OF DECAY MULTICODE VERIFICATION ===" << std::endl;
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

    auto base_mesh = (DATADIR / "demos" / "f3of" / "geometry" / "base.obj").lexically_normal().generic_string();
    auto flap_mesh = (DATADIR / "demos" / "f3of" / "geometry" / "flap.obj").lexically_normal().generic_string();
    auto h5fname   = (DATADIR / "demos" / "f3of" / "hydroData" / "f3of.h5").lexically_normal().generic_string();

    if (!std::filesystem::exists(base_mesh)) {
        std::cerr << "ERROR: base.obj not found at " << base_mesh << std::endl;
        return 1;
    }
    if (!std::filesystem::exists(h5fname)) {
        std::cerr << "ERROR: f3of.h5 not found at " << h5fname << std::endl;
        return 1;
    }

    std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(std::filesystem::path(out_dir));

    std::string base_path = out_dir + "/" + RESULTS_FILE_NAME;

    auto dt1 = run_decay_c1(base_mesh, flap_mesh, h5fname);
    write_output(base_path + "_decay_c1.txt", dt1);

    auto dt3 = run_decay_c3(base_mesh, flap_mesh, h5fname, out_dir);
    write_output(base_path + "_decay_c3.txt", dt3);

    std::cout << "\nAll decay tests completed (Decay C1, Decay C3)." << std::endl;
    return 0;
}
