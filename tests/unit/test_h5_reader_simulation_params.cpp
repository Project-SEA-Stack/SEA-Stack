/*********************************************************************
 * @file  test_h5_reader_simulation_params.cpp
 * @brief Regression tests for BEMIO HDF5 simulation_parameters scalars.
 *
 * Ensures fixed-length string water_depth ("infinite") is read as +inf
 * regardless of HDF5 PredType identity (release bundles vs dev HDF5).
 *********************************************************************/

#include <seastack/hydro_io/h5_reader.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#ifndef SEASTACK_TEST_RM3_H5_FILE
#error "SEASTACK_TEST_RM3_H5_FILE must be set by CMake (path to rm3.h5)."
#endif
#ifndef SEASTACK_TEST_WIGLEY_H5_FILE
#error "SEASTACK_TEST_WIGLEY_H5_FILE must be set by CMake (path to wigley_directional.h5)."
#endif

namespace fs = std::filesystem;

static int g_pass = 0;
static int g_fail = 0;

static void Check(bool ok, const std::string& label) {
    if (ok) {
        ++g_pass;
    } else {
        ++g_fail;
        std::cerr << "FAIL: " << label << "\n";
    }
}

int main() {
    const std::string rm3_path  = SEASTACK_TEST_RM3_H5_FILE;
    const std::string wigley_path = SEASTACK_TEST_WIGLEY_H5_FILE;

    Check(fs::exists(rm3_path), "rm3.h5 exists at CMake-provided path");
    Check(fs::exists(wigley_path), "wigley_directional.h5 exists at CMake-provided path");
    if (g_fail > 0) {
        return EXIT_FAILURE;
    }

    {
        seastack::hydro_io::H5FileInfo reader(rm3_path, 2);
        seastack::hydro::HydroData data = reader.ReadH5Data();
        const auto& sim = data.GetSimulationInfo();
        const double wd = sim.water_depth;
        Check(std::isinf(wd) && wd > 0.0, "rm3 water_depth is positive infinity");
        Check(std::abs(sim.rho - 1000.0) < 1e-9, "rm3 rho is 1000 kg/m^3");
        Check(std::abs(sim.g - 9.81) < 1e-9, "rm3 g is 9.81 m/s^2");
    }

    {
        seastack::hydro_io::H5FileInfo reader(wigley_path, 1);
        seastack::hydro::HydroData data = reader.ReadH5Data();
        const double wd = data.GetSimulationInfo().water_depth;
        Check(std::isfinite(wd) && wd > 0.0, "wigley water_depth is finite and positive");
    }

    if (g_fail > 0) {
        return EXIT_FAILURE;
    }
    std::cout << "OK: test_h5_reader_simulation_params (" << g_pass << " checks)\n";
    return EXIT_SUCCESS;
}
