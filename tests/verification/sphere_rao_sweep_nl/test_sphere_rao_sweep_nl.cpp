/**
 * IEA OES Task 10 sphere, regular waves at steepness S = H / (g T^2) = 0.01
 * (Wendt et al. 2019), free body and optimum PTO damping, ten wave periods.
 * External verification of nonlinear Froude-Krylov
 * (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 10, sphere_rao_sweep_nl).
 *
 * Runs linear, NL FK (Airy + Wheeler) and NL FK (long-wave) with RIRF
 * radiation. compare_*.py fits the steady heave RAO and compares it with the
 * MARIN and NREL weakly nonlinear results.
 *
 * Output: one file per (configuration, period, model) in the results
 * directory, format as nlfk_sphere_heave.h; plus RESULTS_FILE_NAME.txt listing
 * the files written.
 */

#include "../../comparison/sphere/nlfk_sphere_heave.h"

#include <seastack/adapters/chrono/helper.h>
#include <seastack/hydro_io/h5_reader.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

constexpr double kSteepness = 0.01;  // S = H / (g T^2)
constexpr double kG = 9.81;          // [m/s^2]
constexpr double kRampDuration = 30.0;
constexpr double kSteadyPeriods = 25.0;  // fit uses the last 10

constexpr double kPeriods[] = {3.0, 4.0, 4.4, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0};  // [s]
// Optimum resistive PTO damping sqrt(B^2 + (w (m + A) - K / w)^2) from sphere.h5
// [N s/m]; same values as tests/regression/sphere/test_sphere_reg_waves.cpp.
constexpr double kOptimumDamping[] = {398736.034, 118149.758, 90080.857,  161048.558, 322292.419,
                                      479668.979, 633979.761, 784083.286, 932117.647, 1077123.445};

const nlfk_test::HydroModel kModels[] = {nlfk_test::HydroModel::kLinear, nlfk_test::HydroModel::kNlfkAiryWheeler,
                                         nlfk_test::HydroModel::kNlfkHydrostatic};

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    const std::filesystem::path data(seastack::chrono::GetDataDir());
    const auto h5 = (data / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();
    const auto mesh = (data / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    std::ofstream index(out_dir + "/" + RESULTS_FILE_NAME + ".txt");
    const double z_eq = seastack::hydro_io::H5FileInfo(h5, 1).ReadH5Data().GetCGVector(0)[2];

    for (const char* config : {"free", "pto"}) {
        for (int i = 0; i < 10; ++i) {
            const double period = kPeriods[i];
            nlfk_test::HeaveCase c;
            c.omega = 2.0 * std::acos(-1.0) / period;
            c.amplitude = 0.5 * kSteepness * kG * period * period;  // a = H / 2
            c.pto_damping = (std::string(config) == "pto") ? kOptimumDamping[i] : 0.0;
            c.constant_coefficients = false;  // 4.4 s is not on the sphere.h5 frequency grid
            c.ramp = kRampDuration;
            c.t_end = kRampDuration + kSteadyPeriods * period;
            for (auto model : kModels) {
                char name[96];
                std::snprintf(name, sizeof(name), "%s_T%04.1f_%s.txt", config, period, nlfk_test::Name(model));
                std::cout << name << "...\n";
                if (!nlfk_test::RunSphereHeave(h5, mesh, model, c, z_eq, out_dir + "/" + name)) return 1;
                index << name << "\n";
            }
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
