/**
 * C2 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): IEA sphere in
 * regular waves, heave only, constant-coefficient radiation, against the
 * independent Python reference (tests/verification/utilities/nlfk_reference.py,
 * mesh-free sphere). The cases match the stage 2a reference table:
 *   free body, w = 0.8 rad/s, a = 0.1, 1.0, 2.5 m
 *   PTO 6.1e5 N s/m, w = 0.8 rad/s, a = 1.0, 2.5 m
 *   free body, w = 1.25 rad/s, a = 0.5, 1.0 m
 * Each case runs the four models: linear, calm-water nonlinear hydrostatics,
 * NL FK (Airy + Wheeler) and NL FK (long-wave). compare_*.py checks the
 * steady heave amplitude and phase.
 */

#include "nlfk_sphere_heave.h"

#include <seastack/adapters/chrono/helper.h>
#include <seastack/hydro_io/h5_reader.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

struct Case {
    const char* tag;
    nlfk_test::HeaveCase heave;
};

nlfk_test::HeaveCase Make(double amplitude, double omega, double pto) {
    nlfk_test::HeaveCase c;
    c.amplitude = amplitude;
    c.omega = omega;
    c.pto_damping = pto;
    return c;
}

const Case kCases[] = {
    {"free_w0p8_a0p1", Make(0.1, 0.8, 0.0)},   {"free_w0p8_a1p0", Make(1.0, 0.8, 0.0)},
    {"free_w0p8_a2p5", Make(2.5, 0.8, 0.0)},   {"pto_w0p8_a1p0", Make(1.0, 0.8, 6.1e5)},
    {"pto_w0p8_a2p5", Make(2.5, 0.8, 6.1e5)},  {"free_w1p25_a0p5", Make(0.5, 1.25, 0.0)},
    {"free_w1p25_a1p0", Make(1.0, 1.25, 0.0)},
};

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

    for (const auto& c : kCases) {
        for (auto model : nlfk_test::kAllModels) {
            std::cout << c.tag << ", " << nlfk_test::Name(model) << "...\n";
            const auto t0 = std::chrono::steady_clock::now();
            const std::string path = out_dir + "/" + c.tag + "_" + nlfk_test::Name(model) + "_heave.txt";
            if (!nlfk_test::RunSphereHeave(h5, mesh, model, c.heave, z_eq, path)) return 1;
            const auto t1 = std::chrono::steady_clock::now();
            std::cout << "  done in " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                      << " ms\n";
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
