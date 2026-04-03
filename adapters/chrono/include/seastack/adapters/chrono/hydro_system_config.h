/*********************************************************************
 * @file  hydro_system_config.h
 * @brief HydroSystemConfig: value type for all hydrodynamic settings.
 *
 * MAIN TYPES:
 *   - HydroSystemConfig: Accumulates configuration for HydroModelBuilder.
 *
 * ROLE: Separates configuration accumulation from force-callback plumbing
 * and model orchestration. Maps 1:1 to HydroModelBuilder settings.
 *********************************************************************/

#ifndef SEASTACK_ADAPTERS_CHRONO_HYDRO_SYSTEM_CONFIG_H
#define SEASTACK_ADAPTERS_CHRONO_HYDRO_SYSTEM_CONFIG_H

#include <seastack/config.h>

#include <seastack/hydro/excitation_types.h>
#include <seastack/hydro/radiation_types.h>
#include <seastack/hydro/hydro_model_builder.h>

#include <array>
#include <string>
#include <vector>

#ifdef SEASTACK_HAVE_MOORDYN
#include <seastack/mooring/moordyn_config.h>
#endif

namespace seastack::chrono {

/// Value type accumulating all hydrodynamic configuration settings.
///
/// Can be constructed independently and validated. Corresponds 1:1 to
/// HydroModelBuilder settings, eliminating the dual configuration surface.
struct HydroSystemConfig {
    // Excitation
    double excitation_truncation_time = 0.0;
    seastack::hydro::ExcitationMethod excitation_method =
        seastack::hydro::ExcitationMethod::kAuto;
    seastack::hydro::ExcitationInterpolation excitation_interpolation =
        seastack::hydro::ExcitationInterpolation::kCartesian;

    // Radiation
    double radiation_truncation_time = 0.0;
    seastack::hydro::RadiationMethod radiation_method =
        seastack::hydro::RadiationMethod::kRirfConvolution;
    seastack::hydro::StateSpaceOptions state_space_opts;
    bool output_kernel_fit = false;
    seastack::hydro::RadiationKernelProcessing kernel_processing;

    // Diagnostics
    std::string diagnostics_output_dir;

    // Damping
    std::vector<std::array<double, 6>> linear_damping;
    std::vector<std::array<double, 6>> quadratic_damping;

    // Hydrostatics
    std::vector<seastack::hydro::HydroModelBuilder::BodyHydrostaticsConfig>
        body_hydrostatics;
    bool legacy_enable_nonlinear = false;
    std::vector<std::string> legacy_body_mesh_paths;

    // Profiling
    bool profiling_enabled = false;

#ifdef SEASTACK_HAVE_MOORDYN
    seastack::mooring::MoorDynConfig moordyn_config;
#endif
};

}  // namespace seastack::chrono

#endif  // SEASTACK_ADAPTERS_CHRONO_HYDRO_SYSTEM_CONFIG_H
