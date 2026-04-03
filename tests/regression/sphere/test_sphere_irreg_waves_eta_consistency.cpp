/**
 * @file test_sphere_irreg_waves_eta_consistency.cpp
 * @brief Validates spectrum synthesis vs tabulated free-surface elevation for irregular waves.
 *
 * 1. Simulate with waves from spectrum parameters (LinearDirectionalWaveField +
 *    IRF excitation fast path from discrete components).
 * 2. Export eta(t) to a file, then simulate with EtaTableWaveField (IRF excitation
 *    slow path: interpolated eta at IRF lags).
 *
 * Both runs must agree within tolerance; a large mismatch indicates inconsistent
 * excitation or insufficient eta history in the table for early-time convolution.
 */

#include <chrono>
#include <cmath>
#include <seastack/core/math_constants.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChSystemNSC.h>

#include <seastack/adapters/chrono/helper.h>
#include <seastack/adapters/chrono/hydro_system.h>
#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/eta_table_wave_field.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro/waves/wave_component.h>

using namespace chrono;
using namespace seastack::hydro;
using seastack::chrono::HydroSystem;

// Shared simulation parameters
const double TIMESTEP      = 0.015;
const double WAVE_HEIGHT   = 2.0;
const double WAVE_PERIOD   = 12.0;
const int SEED             = 42;   // Fixed seed for reproducibility

int main(int argc, char* argv[]) {
    const double DURATION = seastack::chrono::GetSimDuration(60.0, 120.0);
    std::cout << "=== IRREGULAR WAVES ETA CONSISTENCY TEST ===" << std::endl;
    std::cout << "Validates spectrum synthesis vs tabulated eta (IRF convolution).\n" << std::endl;

    // Initialize environment
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;

    std::filesystem::path DATADIR(seastack::chrono::GetDataDir());
    auto meshfname =
        (DATADIR / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();
    auto h5fname = (DATADIR / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();

    // Output directory setup
    std::string out_dir = seastack::chrono::GetTestOutDir();
    std::filesystem::create_directories(out_dir + "/" + RESULTS_DIR_NAME);
    std::string eta_file = out_dir + "/" + RESULTS_DIR_NAME + "/temp_eta.txt";

    std::vector<double> heave_spectrum;
    std::vector<double> heave_eta;

    seastack::hydro::SeaStateDefinition sea_state;
    sea_state.type = "irregular";
    seastack::hydro::SeaStatePartition partition;
    partition.spectrum.type = "jonswap";
    partition.spectrum.Hs = WAVE_HEIGHT;
    partition.spectrum.Tp = WAVE_PERIOD;
    partition.spectrum.gamma = 1.0;
    sea_state.partitions.push_back(partition);
    sea_state.omega_min = 2.0 * M_PI * 0.001;
    sea_state.omega_max = 2.0 * M_PI * 1.0;
    sea_state.n_omega = 1000;
    sea_state.seed = SEED;

    // ========== PHASE 1: Run with spectrum-generated waves ==========
    std::cout << "Phase 1: Running simulation with spectrum-generated waves..." << std::endl;
    {
        ChSystemNSC system;
        system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
        system.SetSolverType(ChSolver::Type::SPARSE_QR);

        // Ground
        auto ground = chrono_types::make_shared<ChBody>();
        system.AddBody(ground);
        ground->SetPos(ChVector3d(0, 0, -5));
        ground->SetFixed(true);
        ground->EnableCollision(false);

        // Sphere body
        auto sphereBody = chrono_types::make_shared<ChBodyEasyMesh>(meshfname, 1000, false, true, false);
        system.Add(sphereBody);
        sphereBody->SetName("body1");
        sphereBody->SetPos(ChVector3d(0, 0, -2));
        sphereBody->SetMass(261.8e3);

        // Prismatic joint (heave only)
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic->Initialize(sphereBody, ground, false, ChFramed(ChVector3d(0, 0, -2)),
                              ChFramed(ChVector3d(0, 0, -5)));
        system.AddLink(prismatic);

        // Spring (zero stiffness/damping)
        auto spring = chrono_types::make_shared<ChLinkTSDA>();
        spring->Initialize(sphereBody, ground, false, ChVector3d(0, 0, -2), ChVector3d(0, 0, -5));
        spring->SetSpringCoefficient(0.0);
        spring->SetDampingCoefficient(0.0);
        system.AddLink(spring);

        // Create spectrum-based waves
        auto components = seastack::hydro::ComponentSampler::Build(sea_state);
        auto spectrum_wave = std::make_shared<seastack::hydro::LinearDirectionalWaveField>(
            std::move(components), sea_state.depth);

        // Setup hydro forces (wave kinematics + ExcitationComponent for FD cases)
        std::vector<std::shared_ptr<ChBody>> bodies = {sphereBody};
        HydroSystem hydro_forces(bodies, h5fname);
        hydro_forces.AddWaves(spectrum_wave);

        // Free-surface time series with padding before t=0 and after t=DURATION so
        // IRF convolution (phase 2) has eta history for lagged times. Fine dt for
        // smooth interpolation in EtaTableWaveField.
        const double eta_dt = 0.001;
        // Pre-padding must cover max excitation IRF lag: EtaTableWaveField returns
        // 0 outside the table, while phase 1 evaluates analytic eta for all times.
        const double eta_pad = std::max(DURATION, 150.0);
        auto [fse_time, fse_elevation] = spectrum_wave->ComputeElevationTimeSeries(-eta_pad, DURATION + eta_pad, eta_dt);

        std::cout << "  Generated " << fse_time.size() << " free surface elevation samples." << std::endl;
        if (!fse_time.empty()) {
            std::cout << "  Time range: [" << fse_time.front() << ", " << fse_time.back() << "]" << std::endl;
        }

        // Export eta data to file
        std::cout << "  Exporting eta data to: " << eta_file << std::endl;
        {
            std::ofstream eta_out(eta_file);
            if (!eta_out.is_open()) {
                std::cerr << "ERROR: Could not create eta file: " << eta_file << std::endl;
                return 1;
            }
            eta_out << std::setprecision(17);
            for (size_t i = 0; i < fse_time.size(); ++i) {
                eta_out << fse_time[i] << ":" << fse_elevation[i] << "\n";
            }
            eta_out.close();
        }

        // Run simulation
        while (system.GetChTime() <= DURATION) {
            system.DoStepDynamics(TIMESTEP);
            heave_spectrum.push_back(sphereBody->GetPos().z());
        }
        std::cout << "  Completed. " << heave_spectrum.size() << " timesteps." << std::endl;
    }

    // ========== PHASE 2: Run with eta-file-imported waves ==========
    std::cout << "\nPhase 2: Running simulation with eta-file-imported waves..." << std::endl;
    {
        ChSystemNSC system;
        system.SetGravitationalAcceleration(ChVector3d(0.0, 0.0, -9.81));
        system.SetSolverType(ChSolver::Type::SPARSE_QR);

        // Ground
        auto ground = chrono_types::make_shared<ChBody>();
        system.AddBody(ground);
        ground->SetPos(ChVector3d(0, 0, -5));
        ground->SetFixed(true);
        ground->EnableCollision(false);

        // Sphere body
        auto sphereBody = chrono_types::make_shared<ChBodyEasyMesh>(meshfname, 1000, false, true, false);
        system.Add(sphereBody);
        sphereBody->SetName("body1");
        sphereBody->SetPos(ChVector3d(0, 0, -2));
        sphereBody->SetMass(261.8e3);

        // Prismatic joint (heave only)
        auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
        prismatic->Initialize(sphereBody, ground, false, ChFramed(ChVector3d(0, 0, -2)),
                              ChFramed(ChVector3d(0, 0, -5)));
        system.AddLink(prismatic);

        // Spring (zero stiffness/damping)
        auto spring = chrono_types::make_shared<ChLinkTSDA>();
        spring->Initialize(sphereBody, ground, false, ChVector3d(0, 0, -2), ChVector3d(0, 0, -5));
        spring->SetSpringCoefficient(0.0);
        spring->SetDampingCoefficient(0.0);
        system.AddLink(spring);

        // Tabulated eta (same depth metadata as phase 1); IRF uses slow path with
        // linear interpolation — matches exported elevation from phase 1.
        auto eta_waves = std::make_shared<seastack::hydro::EtaTableWaveField>(
            eta_file, sea_state.depth);

        // Setup hydro forces
        std::vector<std::shared_ptr<ChBody>> bodies = {sphereBody};
        HydroSystem hydro_forces(bodies, h5fname);
        hydro_forces.AddWaves(eta_waves);

        // Run simulation
        while (system.GetChTime() <= DURATION) {
            system.DoStepDynamics(TIMESTEP);
            heave_eta.push_back(sphereBody->GetPos().z());
        }
        std::cout << "  Completed. " << heave_eta.size() << " timesteps." << std::endl;
    }

    // ========== PHASE 3: Compare results ==========
    std::cout << "\nPhase 3: Comparing results..." << std::endl;

    if (heave_spectrum.size() != heave_eta.size()) {
        std::cerr << "ERROR: Different number of timesteps! Spectrum: " << heave_spectrum.size()
                  << ", Eta: " << heave_eta.size() << std::endl;
        return 1;
    }

    double max_diff     = 0.0;
    double sum_diff_sq  = 0.0;
    size_t max_diff_idx = 0;

    for (size_t i = 0; i < heave_spectrum.size(); ++i) {
        double diff = std::abs(heave_spectrum[i] - heave_eta[i]);
        sum_diff_sq += diff * diff;
        if (diff > max_diff) {
            max_diff     = diff;
            max_diff_idx = i;
        }
    }

    double rms_diff    = std::sqrt(sum_diff_sq / heave_spectrum.size());
    double time_at_max = max_diff_idx * TIMESTEP;

    std::cout << "  Max difference: " << max_diff << " m (at t=" << time_at_max << "s)" << std::endl;
    std::cout << "  RMS difference: " << rms_diff << " m" << std::endl;

    // ========== PHASE 4: Save results ==========
    std::string results_file = out_dir + "/" + RESULTS_DIR_NAME + "/" + RESULTS_FILE_NAME + ".txt";
    std::ofstream out(results_file);
    if (out.is_open()) {
        out << std::setprecision(10);
        out << "Time (s)    Heave_Spectrum (m)    Heave_Eta (m)    Difference (m)\n";
        out << "--------    ------------------    -------------    --------------\n";
        for (size_t i = 0; i < heave_spectrum.size(); ++i) {
            double t = i * TIMESTEP;
            out << std::setw(10) << std::fixed << std::setprecision(3) << t << std::setw(20) << std::fixed
                << std::setprecision(8) << heave_spectrum[i] << std::setw(18) << std::fixed << std::setprecision(8)
                << heave_eta[i] << std::setw(18) << std::scientific << std::setprecision(4)
                << (heave_spectrum[i] - heave_eta[i]) << "\n";
        }
        out.close();
        std::cout << "\n  Results saved to: " << results_file << std::endl;
    }

    // ========== PASS/FAIL determination ==========
    const double tolerance = 1e-6;  // 1 micrometer tolerance
    bool passed            = (max_diff < tolerance);

    std::cout << "\n=== TEST " << (passed ? "PASSED" : "FAILED") << " ===" << std::endl;
    if (!passed) {
        std::cerr << "Spectrum synthesis and tabulated eta produced different results!" << std::endl;
        std::cerr << "Max difference " << max_diff << " exceeds tolerance " << tolerance << std::endl;
        return 1;
    }

    std::cout << "Spectrum synthesis and tabulated eta produce identical results." << std::endl;
    return 0;
}
