/**
 * C7 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): still-water heave
 * decay of the IEA sphere from 1 m and 5 m, RIRF radiation, 40 s, with the
 * three nonlinear variants: calm-water nonlinear hydrostatics, NL FK (Airy +
 * Wheeler) and NL FK (long-wave). In still water all three integrate the same
 * static pressure, so compare_*.py checks they agree.
 */

#include "nlfk_sphere_heave.h"

#include <seastack/adapters/chrono/helper.h>
#include <seastack/hydro_io/h5_reader.h>

#include <filesystem>
#include <iostream>
#include <string>

int main(int /*argc*/, char* /*argv*/[]) {
    std::string data_dir;
    if (!seastack::chrono::SetInitialEnvironment(data_dir)) return 1;
    const std::filesystem::path data(seastack::chrono::GetDataDir());
    const auto h5 = (data / "demos" / "sphere" / "hydroData" / "sphere.h5").lexically_normal().generic_string();
    const auto mesh = (data / "demos" / "sphere" / "geometry" / "sphere.obj").lexically_normal().generic_string();

    const std::string out_dir = seastack::chrono::GetTestOutDir() + "/" + RESULTS_DIR_NAME;
    std::filesystem::create_directories(out_dir);
    const double z_eq = seastack::hydro_io::H5FileInfo(h5, 1).ReadH5Data().GetCGVector(0)[2];

    using nlfk_test::HydroModel;
    for (double offset : {1.0, 5.0}) {
        nlfk_test::HeaveCase c;
        c.amplitude = 0.0;  // still water
        c.constant_coefficients = false;
        c.t_end = 40.0;
        c.z0_offset = offset;
        for (auto model : {HydroModel::kCalmWaterNonlinear, HydroModel::kNlfkAiryWheeler,
                           HydroModel::kNlfkHydrostatic}) {
            const std::string tag = "decay_" + std::to_string(static_cast<int>(offset)) + "m";
            std::cout << tag << ", " << nlfk_test::Name(model) << "...\n";
            const std::string path = out_dir + "/" + tag + "_" + nlfk_test::Name(model) + "_heave.txt";
            if (!nlfk_test::RunSphereHeave(h5, mesh, model, c, z_eq, path)) return 1;
        }
    }
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
