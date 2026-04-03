# Unit tests — standalone executables (most need no repo data files).
# HydroIO tests require HDF5; test_h5_reader_simulation_params reads data/demos/run_seastack/.../*.h5.

function(build_unit_test TEST_NAME)
    set(TEST_SRC ${TEST_NAME}.cpp)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_compile_features(${TEST_NAME} PRIVATE cxx_std_17)
    seastack_apply_compile_settings(${TEST_NAME})
    add_test(
        NAME ${TEST_NAME}
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_NAME}>"
        COMMAND $<TARGET_FILE:${TEST_NAME}>
    )
    set_tests_properties(${TEST_NAME} PROPERTIES LABELS "unit" COST 1)
    # On Windows, prepend DLL search paths so Chrono/HDF5 DLLs are found (same as legacy project)
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND DEFINED TEST_ENVIRONMENT)
        set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT "${TEST_ENVIRONMENT}")
    endif()
endfunction()

# Radiation state-space tests (Chrono-free, HDF5-free)
# These need access to private headers in libs/hydro/src
build_unit_test(test_radiation_ss_model)
target_link_libraries(test_radiation_ss_model PRIVATE SEAStack::Hydro)
target_include_directories(test_radiation_ss_model PRIVATE
    "${CMAKE_SOURCE_DIR}/libs/hydro/src"
)
set_tests_properties(test_radiation_ss_model PROPERTIES LABELS "unit;chrono-free")

build_unit_test(test_radiation_ss_fitter)
target_link_libraries(test_radiation_ss_fitter PRIVATE SEAStack::Hydro)
target_include_directories(test_radiation_ss_fitter PRIVATE
    "${CMAKE_SOURCE_DIR}/libs/hydro/src"
)
set_tests_properties(test_radiation_ss_fitter PROPERTIES LABELS "unit;chrono-free")

# Directional waves test (requires HDF5 via HydroIO, but no Chrono)
if(TARGET seastack_hydro_io)
    build_unit_test(test_directional_waves)
    target_link_libraries(test_directional_waves PRIVATE SEAStack::HydroIO)
    set_tests_properties(test_directional_waves PROPERTIES LABELS "unit;chrono-free")

    # BEMIO string scalar water_depth (rm3) + float scalar (wigley) — needs repo data files
    set(_seastack_test_rm3_h5 "${CMAKE_SOURCE_DIR}/data/demos/run_seastack/rm3/assets/hydroData/rm3.h5")
    set(_seastack_test_wigley_h5
        "${CMAKE_SOURCE_DIR}/data/demos/run_seastack/wigley/assets/hydroData/wigley_directional.h5")
    build_unit_test(test_h5_reader_simulation_params)
    target_link_libraries(test_h5_reader_simulation_params PRIVATE SEAStack::HydroIO)
    target_compile_definitions(test_h5_reader_simulation_params PRIVATE
        "SEASTACK_TEST_RM3_H5_FILE=\"${_seastack_test_rm3_h5}\""
        "SEASTACK_TEST_WIGLEY_H5_FILE=\"${_seastack_test_wigley_h5}\"")
    set_tests_properties(test_h5_reader_simulation_params PROPERTIES LABELS "unit;chrono-free")

    build_unit_test(test_rm3_wave_yaml_depth_precedence)
    target_link_libraries(test_rm3_wave_yaml_depth_precedence PRIVATE SEAStack::HydroIO)
    target_compile_definitions(test_rm3_wave_yaml_depth_precedence PRIVATE
        "SEASTACK_TEST_RM3_H5_FILE=\"${_seastack_test_rm3_h5}\"")
    set_tests_properties(test_rm3_wave_yaml_depth_precedence PROPERTIES LABELS "unit;chrono-free")
else()
    message(STATUS "Skipping test_directional_waves (requires SEASTACK_ENABLE_HYDRO_IO)")
endif()

# YAML parser tests (Chrono-free, HDF5-free)
build_unit_test(test_yaml_parser)
target_link_libraries(test_yaml_parser PRIVATE SEAStack::Hydro)
set_tests_properties(test_yaml_parser PROPERTIES LABELS "unit;chrono-free")

# Excitation ramp factor for GUI visualization (Chrono-free, HDF5-free)
build_unit_test(test_excitation_ramp_visualization)
target_link_libraries(test_excitation_ramp_visualization PRIVATE SEAStack::Hydro)
set_tests_properties(test_excitation_ramp_visualization PROPERTIES LABELS "unit;chrono-free")

# PTO tests (Chrono-free, HDF5-free)
build_unit_test(test_linear_pto)
target_link_libraries(test_linear_pto PRIVATE SEAStack::PTO)
set_tests_properties(test_linear_pto PROPERTIES LABELS "unit;chrono-free")

build_unit_test(test_hydraulic_components)
target_link_libraries(test_hydraulic_components PRIVATE SEAStack::PTO)
set_tests_properties(test_hydraulic_components PROPERTIES LABELS "unit;chrono-free")

# Control tests (Chrono-free, HDF5-free)
build_unit_test(test_pi_controller)
target_link_libraries(test_pi_controller PRIVATE SEAStack::Control)
set_tests_properties(test_pi_controller PROPERTIES LABELS "unit;chrono-free")

# Hydraulic PTO assembly test (Chrono-free, HDF5-free)
build_unit_test(test_rectified_hydraulic_pto)
target_link_libraries(test_rectified_hydraulic_pto PRIVATE SEAStack::PTO)
set_tests_properties(test_rectified_hydraulic_pto PROPERTIES LABELS "unit;chrono-free")

# Doctor diagnostics tests (Chrono-free, uses Infra only).
# Compiles doctor.cpp directly to avoid pulling the full app lib (and Chrono).
add_executable(test_doctor
    test_doctor.cpp
    "${CMAKE_SOURCE_DIR}/apps/seastack/doctor.cpp"
)
target_compile_features(test_doctor PRIVATE cxx_std_17)
seastack_apply_compile_settings(test_doctor)
target_link_libraries(test_doctor PRIVATE SEAStack::Infra)
target_include_directories(test_doctor PRIVATE
    "${CMAKE_SOURCE_DIR}/apps/seastack"
    "${CMAKE_BINARY_DIR}"
)
add_test(
    NAME test_doctor
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_doctor>"
    COMMAND $<TARGET_FILE:test_doctor>
)
set_tests_properties(test_doctor PROPERTIES LABELS "unit;chrono-free" COST 1)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND DEFINED TEST_ENVIRONMENT)
    set_tests_properties(test_doctor PROPERTIES ENVIRONMENT "${TEST_ENVIRONMENT}")
endif()

# Excitation ramp accumulation test (Chrono-free, HDF5-free)
# Regression test for the excitation ramp bug where ComputeFromTFData()
# applied the ramp multiplier to the entire force vector, zeroing buoyancy.
build_unit_test(test_excitation_ramp_accumulation)
target_link_libraries(test_excitation_ramp_accumulation PRIVATE SEAStack::Hydro)
set_tests_properties(test_excitation_ramp_accumulation PROPERTIES LABELS "unit;chrono-free")

# Added mass determinism test (requires Chrono adapter)
if(TARGET seastack_chrono_adapter)
    build_unit_test(test_added_mass_determinism)
    target_link_libraries(test_added_mass_determinism PRIVATE SEAStack::ChronoAdapter)
    set_tests_properties(test_added_mass_determinism PROPERTIES LABELS "unit;chrono")
else()
    message(STATUS "Skipping test_added_mass_determinism (requires SEASTACK_ENABLE_CHRONO)")
endif()

# Adapter-layer unit tests (ChronoForceAttacher caching/divergence, HydroSystemConfig)
if(TARGET seastack_chrono_adapter)
    build_unit_test(test_adapter_layer)
    target_link_libraries(test_adapter_layer PRIVATE SEAStack::ChronoAdapter)
    set_tests_properties(test_adapter_layer PROPERTIES LABELS "unit;chrono")
else()
    message(STATUS "Skipping test_adapter_layer (requires SEASTACK_ENABLE_CHRONO)")
endif()

# Export redesign tests (H5Writer compression/F32, HydroForces per-component, ExportConfig)
if(TARGET seastack_hydro_io AND TARGET seastack_chrono_adapter)
    build_unit_test(test_export_redesign)
    # ChronoAdapter already PUBLIC-links Hydro, HydroIO (when built), Infra, PTO, Core, Chrono.
    target_link_libraries(test_export_redesign PRIVATE SEAStack::ChronoAdapter)
    set_tests_properties(test_export_redesign PROPERTIES LABELS "unit")
else()
    message(STATUS "Skipping test_export_redesign (requires SEASTACK_ENABLE_HYDRO_IO + SEASTACK_ENABLE_CHRONO)")
endif()
