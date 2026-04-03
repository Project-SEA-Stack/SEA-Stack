# SEAStack::Hydro — Chrono-free hydrodynamic physics library
# Allowed deps: Core, Eigen, OpenMP
# Forbidden: Chrono, HDF5, VSG, MoorDyn

add_library(seastack_hydro STATIC
    src/hydro_data.cpp
    src/core/hydro_forces.cpp
    src/waves/wave_base.cpp
    src/waves/component_sampler.cpp
    src/waves/linear_wave_kinematics.cpp
    src/waves/linear_directional_wave_field.cpp
    src/waves/eta_table_wave_field.cpp
    src/waves/wave_utilities.cpp
    src/radiation/radiation_rirf_processing.cpp
    src/radiation/radiation_rirf_convolution.cpp
    src/radiation/radiation_ss_model.cpp
    src/radiation/radiation_ss_fitter.cpp
    src/geometry/triangle_mesh.cpp
    src/geometry/mesh_io.cpp
    src/geometry/submerged_volume.cpp
    src/force_components/hydrostatics_component.cpp
    src/force_components/nonlinear_hydrostatics_component.cpp
    src/force_components/radiation_component.cpp
    src/force_components/radiation_ss_component.cpp
    src/force_components/excitation_component.cpp
    src/force_components/excitation_irf_component.cpp
    src/force_components/damping_component.cpp
    src/excitation_transfer.cpp
    src/wave_factory.cpp
    src/hydro_model_builder.cpp
    src/config/yaml_parser.cpp
    src/config/config_loader.cpp
    # mooring_component excluded — moves to libs/mooring in Phase 4
)
add_library(SEAStack::Hydro ALIAS seastack_hydro)

target_include_directories(seastack_hydro
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(seastack_hydro
    PUBLIC SEAStack::Core
    PRIVATE SEAStack::Infra
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(seastack_hydro PRIVATE OpenMP::OpenMP_CXX)
endif()

target_compile_features(seastack_hydro PUBLIC cxx_std_17)
