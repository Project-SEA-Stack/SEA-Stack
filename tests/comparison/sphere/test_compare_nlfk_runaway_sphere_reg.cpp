/**
 * C4 (docs/theory/NONLINEAR_FROUDE_KRYLOV.md, section 9): the runaway case
 * behind the original report. IEA sphere, free heave, H = 10 m, T = 8 s,
 * RIRF radiation, 80 s, all four hydrostatic / Froude-Krylov models.
 *
 * Calm-water nonlinear hydrostatics with linear excitation drives the body
 * out of the water; NL FK must keep the heave within +/-(draft + amplitude).
 * The log is written to run.log with one "NLFK_RUN <model>" marker per run,
 * so compare_*.py can check which runs warned.
 */

#include "nlfk_sphere_heave.h"

#include <seastack/adapters/chrono/helper.h>
#include <seastack/hydro_io/h5_reader.h>
#include <seastack/infra/logging.h>

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

    const std::string log_path = out_dir + "/run.log";
    std::filesystem::remove(log_path);
    seastack::infra::LoggingConfig log_cfg;
    log_cfg.log_file_path = log_path;
    log_cfg.enable_file_output = true;
    log_cfg.enable_colors = false;
    if (seastack::infra::IsInitialized()) {
        seastack::infra::UpdateLoggingConfig(log_cfg);
    } else if (!seastack::infra::Initialize(log_cfg)) {
        std::cerr << "Error: cannot initialise logging to " << log_path << "\n";
        return 1;
    }

    nlfk_test::HeaveCase c;
    c.amplitude = 5.0;
    c.omega = 2.0 * std::acos(-1.0) / 8.0;
    c.constant_coefficients = false;
    c.t_end = 80.0;
    for (auto model : nlfk_test::kAllModels) {
        LOG_INFO("NLFK_RUN " << nlfk_test::Name(model));
        std::cout << "H = 10 m, T = 8 s, " << nlfk_test::Name(model) << "...\n";
        const std::string path = out_dir + "/runaway_" + nlfk_test::Name(model) + "_heave.txt";
        if (!nlfk_test::RunSphereHeave(h5, mesh, model, c, z_eq, path)) return 1;
    }
    seastack::infra::Shutdown();
    std::cout << "Results written to " << out_dir << "\n";
    return 0;
}
