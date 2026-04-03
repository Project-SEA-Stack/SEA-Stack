/*********************************************************************
 * @file  main.cpp
 * @brief Standalone hydro example — no Chrono dependency.
 *
 * Demonstrates the HydroModelBuilder API for constructing a
 * Chrono-free hydrodynamic force model from a BEMIO HDF5 file.
 *
 * This is the recommended workflow for coupling SEA-Stack hydro
 * to an external solver.  For Chrono-integrated and richer usage
 * patterns, see the demos under demos/ in the repository.
 *********************************************************************/

#include <seastack/hydro_io/h5_reader.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/core/math_constants.h>
#include <seastack/core/system_state.h>
#include <seastack/core/types.h>

#include <H5Cpp.h>
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: standalone_hydro <path_to_h5_file>\n";
        return 1;
    }

    const std::string h5_path = argv[1];
    constexpr int num_bodies = 1;
    constexpr double dt = 0.01;
    constexpr double t_end = 2.0;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "SEA-Stack Standalone Hydro Example\n";
    std::cout << "==================================\n";
    std::cout << "  H5 file: " << h5_path << "\n\n";

    seastack::hydro::HydroData hydro_data;
    try {
        // ── Step 1: Load hydrodynamic data ──────────────────────────────
        hydro_data = seastack::hydro_io::H5FileInfo(h5_path, num_bodies).ReadH5Data();
    } catch (const H5::Exception& e) {
        std::cerr << "HDF5 error: " << e.getFuncName() << " — " << e.getDetailMsg() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Unexpected error loading HDF5 (non-standard exception).\n";
        return 1;
    }

    std::cout << "  Loaded H5 data: " << num_bodies << " body(ies)\n";
    std::cout << "  rho = " << hydro_data.GetSimulationInfo().rho
              << " kg/m3, g = " << hydro_data.GetSimulationInfo().g << " m/s2\n";
    std::cout << "  water depth = " << hydro_data.GetSimulationInfo().water_depth << " m\n\n";

    // ── Step 2: Build the hydro model ───────────────────────────────
    using namespace seastack::hydro;

    SeaStateDefinition sea_state;
    sea_state.type = "regular";
    sea_state.amplitude = 0.5;                     // H/2 [m]
    sea_state.omega = 2.0 * M_PI / 8.0;           // T = 8 s

    HydroModel model = HydroModelBuilder()
        .FromHydroData(std::move(hydro_data))
        .WithSeaState(sea_state)
        .EnableHydrostatics()
        .EnableRadiation()
        .EnableExcitation()
        .Build();

    std::cout << "  Force components: hydrostatics, radiation (RIRF), excitation (regular wave)\n";
    std::cout << "  Wave: A = " << sea_state.amplitude
              << " m, omega = " << sea_state.omega << " rad/s\n\n";

    // ── Step 3: Evaluate forces over a short time window ────────────
    std::cout << std::setw(8) << "time"
              << std::setw(14) << "Fz [N]"
              << std::setw(14) << "My [N.m]" << "\n";

    // Set up initial state at equilibrium (CG position from the H5 data)
    const auto& cg = model.GetData().GetCGVector(0);
    SystemState state;
    state.bodies.resize(num_bodies);
    state.bodies[0].position        = Eigen::Vector3d(cg[0], cg[1], cg[2]);
    state.bodies[0].orientation_rpy = Eigen::Vector3d::Zero();
    state.bodies[0].linear_velocity = Eigen::Vector3d::Zero();
    state.bodies[0].angular_velocity = Eigen::Vector3d::Zero();

    int step = 0;
    for (double t = 0.0; t <= t_end; t += dt) {
        BodyForces forces = model.Evaluate(state, t);

        if (step % 10 == 0) {
            double Fz = forces[0].force.z();
            double My = forces[0].moment.y();
            std::cout << std::setw(8) << t
                      << std::setw(14) << Fz
                      << std::setw(14) << My << "\n";
        }
        ++step;
    }

    std::cout << "\nStandalone hydro evaluation complete (" << step << " steps).\n";
    return 0;
}

// For Chrono-integrated hydro, waves, and other patterns, see demos/ in the repository.
