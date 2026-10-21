/**
 * C1 and C6 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): IEA sphere,
 * heave only, free body, constant-coefficient radiation.
 *
 *   C1  steepness H / lambda = 0.0005 at w = 0.6, 0.8, 1.0, 1.25 rad/s:
 *       linear, NL FK (Airy + Wheeler), NL FK (long-wave). At this steepness
 *       NL FK must reproduce the linear RAO.
 *   C6  a = 1 m at w = 0.6, 0.8, 1.0, 1.25, 1.5 rad/s, all four models
 *       (informational sweep).
 */

#include "nlfk_sphere_heave.h"

#include <seastack/adapters/chrono/helper.h>
#include <seastack/hydro_io/h5_reader.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

constexpr double kG = 9.81;
constexpr double kDepth = 200.0;     // sphere.h5 water depth [m]
constexpr double kSteepness = 0.0005;

/// Wavelength [m] from w^2 = g k tanh(k h).
double Wavelength(double omega) {
    double k = omega * omega / kG;
    for (int i = 0; i < 100; ++i) {
        const double t = std::tanh(k * kDepth);
        const double dk = (omega * omega - kG * k * t) / (-kG * t - kG * k * kDepth * (1.0 - t * t));
        k -= dk;
        if (std::abs(dk) < 1e-14 * k) break;
    }
    return 2.0 * std::acos(-1.0) / k;
}

std::string OmegaTag(double omega) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "w%.2f", omega);
    std::string s(buf);
    for (auto& ch : s) {
        if (ch == '.') ch = 'p';
    }
    return s;
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    const std::filesystem::path data(seastack::chrono::GetDataDir());
    const auto h5 = (data / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();
    const auto mesh = (data / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    const double z_eq = seastack::hydro_io::H5FileInfo(h5, 1).ReadH5Data().GetCGVector(0)[2];

    auto run = [&](const std::string& prefix, double omega, double amplitude, nlfk_test::HydroModel model) {
        nlfk_test::HeaveCase c;
        c.omega = omega;
        c.amplitude = amplitude;
        const std::string path = out_dir + "/" + prefix + "_" + OmegaTag(omega) + "_" + nlfk_test::Name(model) +
                                 "_heave.txt";
        std::cout << prefix << " w = " << omega << ", a = " << amplitude << " m, " << nlfk_test::Name(model)
                  << "...\n";
        return nlfk_test::RunSphereHeave(h5, mesh, model, c, z_eq, path);
    };

    using nlfk_test::HydroModel;
    for (double omega : {0.6, 0.8, 1.0, 1.25}) {
        const double amplitude = 0.5 * kSteepness * Wavelength(omega);
        for (auto model : {HydroModel::kLinear, HydroModel::kNlfkAiryWheeler, HydroModel::kNlfkHydrostatic}) {
            if (!run("c1", omega, amplitude, model)) return 1;
        }
    }
    for (double omega : {0.6, 0.8, 1.0, 1.25, 1.5}) {
        for (auto model : nlfk_test::kAllModels) {
            if (!run("c6", omega, 1.0, model)) return 1;
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
