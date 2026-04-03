/*********************************************************************
 * @file  test_rm3_wave_yaml_depth_precedence.cpp
 * @brief Ensures explicit YAML-style sea-state depth is not replaced by
 *        H5 simulation_parameters/water_depth when ApplySimulationEnvironment(..., false)
 *        and HydroSystem-style UpdateEnvironment(g, GetWaterDepth()) are used.
 *********************************************************************/

#include <seastack/hydro/waves/component_sampler.h>
#include <seastack/hydro/waves/linear_directional_wave_field.h>
#include <seastack/hydro_io/h5_reader.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#ifndef SEASTACK_TEST_RM3_H5_FILE
#error "SEASTACK_TEST_RM3_H5_FILE must be set by CMake (path to rm3.h5)."
#endif

namespace fs = std::filesystem;

static int Fail(const char* msg) {
    std::cerr << "FAIL: " << msg << "\n";
    return EXIT_FAILURE;
}

int main() {
    const std::string rm3_path = SEASTACK_TEST_RM3_H5_FILE;
    if (!fs::exists(rm3_path)) {
        return Fail("rm3.h5 missing at CMake path");
    }

    seastack::hydro_io::H5FileInfo reader(rm3_path, 2);
    seastack::hydro::HydroData data = reader.ReadH5Data();

    seastack::hydro::SeaStateDefinition def;
    def.type  = "irregular";
    def.depth = 50.0;
    def.g     = 9.81;
    seastack::hydro::SeaStatePartition part;
    part.spectrum.type                 = "jonswap";
    part.spectrum.Hs                   = 2.0;
    part.spectrum.Tp                 = 8.0;
    part.spectrum.gamma              = 3.3;
    part.spreading.type              = "none";
    part.spreading.mean_direction_deg = 0.0;
    def.partitions.push_back(part);
    def.n_omega = 16;
    def.seed    = 42;

    std::vector<seastack::hydro::WaveComponent> components =
        seastack::hydro::ComponentSampler::Build(def);
    seastack::hydro::LinearDirectionalWaveField field(std::move(components), def.depth);
    field.SetNumBodies(2);
    field.ApplySimulationEnvironment(data.GetSimulationInfo(), false);

    constexpr double kYamlDepth = 50.0;
    if (std::abs(field.GetWaterDepth() - kYamlDepth) > 1.0e-9) {
        return Fail("water depth after ApplySimulationEnvironment(..., false) should match YAML (50 m)");
    }

    const double g_file = data.GetSimulationInfo().g;
    field.UpdateEnvironment(g_file, field.GetWaterDepth());
    if (std::abs(field.GetWaterDepth() - kYamlDepth) > 1.0e-9) {
        return Fail("water depth after UpdateEnvironment(g, GetWaterDepth()) should stay 50 m");
    }

    std::cout << "OK: test_rm3_wave_yaml_depth_precedence\n";
    return EXIT_SUCCESS;
}
