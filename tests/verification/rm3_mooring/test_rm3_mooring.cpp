#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/eta_table_wave_field.h>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#endif

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>
#include <chrono/timestepper/ChTimestepperImplicit.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;
using seastack::mooring::MoorDynConfig;

int main(int argc, char* argv[]) {
    std::cout << "=== RM3 MOORING VERIFICATION TEST ===" << std::endl;
    std::cout << "Chrono version: " << CHRONO_VERSION << "\n\n";

    try {
        std::string data_dir;
        if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

        std::filesystem::path DATADIR(seastack::chrono::GetDataDir());

        auto body1_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "float_cog.obj").lexically_normal().generic_string();
        auto body2_mesh = (DATADIR / "demos" / "rm3" / "geometry" / "plate_cog.obj").lexically_normal().generic_string();
        auto h5_file    = (DATADIR / "demos" / "rm3" / "hydroData" / "rm3.h5").lexically_normal().generic_string();
        auto eta_file   = (DATADIR / "verification" / "rm3_mooring" / "inputs" / "eta_rm3_mooring.txt").lexically_normal().generic_string();
        auto moordyn_file = (DATADIR / "demos" / "rm3" / "mooring" / "lines_rm3.txt").lexically_normal().generic_string();

        for (const auto& [label, path] : std::vector<std::pair<std::string, std::string>>{
                 {"float mesh", body1_mesh}, {"plate mesh", body2_mesh},
                 {"h5 data", h5_file}, {"eta file", eta_file}, {"MoorDyn input", moordyn_file}}) {
            if (!std::filesystem::exists(path)) {
                std::cerr << "ERROR: " << label << " not found: " << path << std::endl;
                return 1;
            }
            std::cout << "  " << label << ": " << path << std::endl;
        }

        // System / solver (matching WEC-Sim: HHT integrator, 0.01s timestep)
        ChSystemNSC system;
        system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
        double timestep = 0.01;
        system.SetTimestepperType(ChTimestepper::Type::HHT);
        system.SetSolverType(ChSolver::Type::GMRES);

        if (auto integrator = std::dynamic_pointer_cast<ChTimestepperImplicit>(system.GetTimestepper())) {
            integrator->SetStepControl(false);
            integrator->SetMaxIters(50);
        }

        double simulationDuration = seastack::chrono::GetSimDuration(180.0, 400.0);

        std::vector<double> time_vector;
        std::vector<double> float_heave;
        std::vector<double> plate_heave;

        // Float body (body1) -- CG at z = -0.72 m (WEC-Sim equilibrium)
        auto float_body1 = chrono_types::make_shared<ChBodyEasyMesh>(
            body1_mesh, 0, false, true, false);
        system.Add(float_body1);
        float_body1->SetName("body1");
        float_body1->SetPos(ChVector3d(0, 0, -0.72));
        float_body1->SetMass(725834);
        float_body1->SetInertiaXX(ChVector3d(20907301.0, 21306090.66, 37085481.11));

        // Plate body (body2) -- CG at -21.29 + initial displacement -0.21 = -21.5 m
        auto plate_body2 = chrono_types::make_shared<ChBodyEasyMesh>(
            body2_mesh, 0, false, true, false);
        system.Add(plate_body2);
        plate_body2->SetName("body2");
        plate_body2->SetPos(ChVector3d(0, 0, -21.5));
        plate_body2->SetMass(886691);
        plate_body2->SetInertiaXX(ChVector3d(94419614.57, 94407091.24, 28542224.82));

        // Prismatic joint (heave DOF between float and plate)
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic->Initialize(float_body1, plate_body2, false,
                              ChFramed(ChVector3d(0, 0, -0.72)),
                              ChFramed(ChVector3d(0, 0, -21.5)));
        system.AddLink(prismatic);

        // PTO: 1,200,000 N-s/m damping, 0 stiffness (matching WEC-Sim)
        auto pto = chrono_types::make_shared<ChLinkTSDA>();
        pto->Initialize(float_body1, plate_body2, false,
                        ChVector3d(0, 0, -0.72), ChVector3d(0, 0, -21.5));
        pto->SetDampingCoefficient(1200000.0);
        pto->SetSpringCoefficient(0.0);
        system.AddLink(pto);

        // Hydrodynamic forces
        std::vector<std::shared_ptr<ChBody>> bodies;
        bodies.push_back(float_body1);
        bodies.push_back(plate_body2);

        // Wave input: direct eta table (no DFT decomposition).
        // Uses EtaTableWaveField so ExcitationIrfComponent falls to its
        // slow path and convolves directly with the original imported eta.
        auto waves = std::make_shared<EtaTableWaveField>(eta_file, 70.0);
        waves->SetRampDuration(40.0);  // WEC-Sim rampTime = 40 s

        HydroSystem hydro_forces(bodies, h5_file);
        hydro_forces.SetExcitationTruncationTime(20.0);  // WEC-Sim CITime = 20 s

#ifdef SEASTACK_HAVE_MOORDYN
        {
            MoorDynConfig md_cfg;
            md_cfg.enabled = true;
            md_cfg.input_file = moordyn_file;
            md_cfg.coupled_body_indices = {1};  // plate = body index 1
            hydro_forces.SetMoorDynConfig(md_cfg);
            std::cout << "MoorDyn enabled, coupled to body2 (plate)" << std::endl;
        }
#else
        std::cerr << "WARNING: Built without SEASTACK_HAVE_MOORDYN -- mooring forces disabled" << std::endl;
#endif

        hydro_forces.AddWaves(waves);

        auto start = std::chrono::high_resolution_clock::now();

        std::vector<double> eta_vector;
        Eigen::Vector3d origin(0, 0, 0);

        while (system.GetChTime() <= simulationDuration) {
            system.DoStepDynamics(timestep);

            time_vector.push_back(system.GetChTime());
            float_heave.push_back(float_body1->GetPos().z());
            plate_heave.push_back(plate_body2->GetPos().z());
            eta_vector.push_back(waves->GetElevation(origin, system.GetChTime()));
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Simulation completed in " << duration_ms << " ms ("
                  << time_vector.size() << " steps)" << std::endl;

        // Save results
        std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
        std::filesystem::create_directories(std::filesystem::path(out_dir));

        std::string out_path = out_dir + "/" + RESULTS_FILE_NAME + ".txt";
        std::ofstream outputFile(out_path);
        if (!outputFile.is_open()) {
            std::cerr << "Error: Could not open " << out_path << std::endl;
            return 1;
        }

        outputFile << std::left << std::setw(12) << "Time(s)"
                   << std::right << std::setw(16) << "FloatHeaveZ(m)"
                   << std::right << std::setw(16) << "PlateHeaveZ(m)" << "\n";

        for (size_t i = 0; i < time_vector.size(); ++i) {
            outputFile << std::left   << std::setw(12) << std::fixed << std::setprecision(4) << time_vector[i]
                       << std::right  << std::setw(16) << std::fixed << std::setprecision(8) << float_heave[i]
                       << std::right  << std::setw(16) << std::fixed << std::setprecision(8) << plate_heave[i]
                       << "\n";
        }
        outputFile.close();
        std::cout << "Results saved to " << out_path << std::endl;

        // Write reconstructed wave elevation for diagnostic comparison
        std::string eta_path = out_dir + "/" + RESULTS_FILE_NAME + "_eta.txt";
        std::ofstream etaFile(eta_path);
        if (etaFile.is_open()) {
            etaFile << std::left << std::setw(12) << "Time(s)"
                    << std::right << std::setw(16) << "Elevation(m)" << "\n";
            for (size_t i = 0; i < time_vector.size(); ++i) {
                etaFile << std::left  << std::setw(12) << std::fixed << std::setprecision(4) << time_vector[i]
                        << std::right << std::setw(16) << std::fixed << std::setprecision(8) << eta_vector[i]
                        << "\n";
            }
            etaFile.close();
            std::cout << "Eta saved to " << eta_path << std::endl;
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << std::endl;
        return 1;
    }
}
