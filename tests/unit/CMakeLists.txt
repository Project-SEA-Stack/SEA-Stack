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

    # Nonlinear Froude-Krylov builder wiring: per-body toggle and build-time errors
    build_unit_test(test_nonlinear_froude_krylov_builder)
    target_link_libraries(test_nonlinear_froude_krylov_builder PRIVATE SEAStack::HydroIO)
    target_compile_definitions(test_nonlinear_froude_krylov_builder PRIVATE
        "SEASTACK_TEST_RM3_H5_FILE=\"${_seastack_test_rm3_h5}\""
        "SEASTACK_TEST_RM3_FLOAT_OBJ=\"${CMAKE_SOURCE_DIR}/data/demos/run_seastack/rm3/assets/geometry/float.obj\"")
    set_tests_properties(test_nonlinear_froude_krylov_builder PROPERTIES LABELS "unit;chrono-free")
else()
    message(STATUS "Skipping test_directional_waves (requires SEASTACK_ENABLE_HYDRO_IO)")
endif()

# YAML parser tests (Chrono-free, HDF5-free)
build_unit_test(test_yaml_parser)
target_link_libraries(test_yaml_parser PRIVATE SEAStack::Hydro)
target_compile_definitions(test_yaml_parser PRIVATE "SEASTACK_TEST_DATA_DIR=\"${CMAKE_SOURCE_DIR}/data\"")
set_tests_properties(test_yaml_parser PROPERTIES LABELS "unit;chrono-free")

# Mesh submerged volume (nonlinear hydrostatics geometry; Chrono-free, HDF5-free)
build_unit_test(test_submerged_volume)
target_link_libraries(test_submerged_volume PRIVATE SEAStack::Hydro)
set_tests_properties(test_submerged_volume PROPERTIES LABELS "unit;chrono-free")

# Wetted-surface pressure integration for nonlinear Froude-Krylov (Chrono-free, HDF5-free)
build_unit_test(test_wetted_surface_pressure)
target_link_libraries(test_wetted_surface_pressure PRIVATE SEAStack::Hydro)
set_tests_properties(test_wetted_surface_pressure PROPERTIES LABELS "unit;chrono-free")

# Incident-wave dynamic pressure for nonlinear Froude-Krylov (Chrono-free, HDF5-free)
build_unit_test(test_wave_pressure)
target_link_libraries(test_wave_pressure PRIVATE SEAStack::Hydro)
set_tests_properties(test_wave_pressure PROPERTIES LABELS "unit;chrono-free")

# Nonlinear Froude-Krylov component: frames, a^2 scaling, convergence (Chrono-free, HDF5-free)
build_unit_test(test_nonlinear_froude_krylov)
target_link_libraries(test_nonlinear_froude_krylov PRIVATE SEAStack::Hydro)
set_tests_properties(test_nonlinear_froude_krylov PROPERTIES LABELS "unit;chrono-free")

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

# MoorDyn orientation-convention test (Chrono-free, header-only: no MoorDyn link)
build_unit_test(test_moordyn_euler_convention)
target_link_libraries(test_moordyn_euler_convention PRIVATE SEAStack::Core)
target_include_directories(test_moordyn_euler_convention PRIVATE
    "${CMAKE_SOURCE_DIR}/libs/mooring/include"
)
set_tests_properties(test_moordyn_euler_convention PROPERTIES LABELS "unit;chrono-free")

# Control tests (Chrono-free, HDF5-free)
build_unit_test(test_pi_controller)
target_link_libraries(test_pi_controller PRIVATE SEAStack::Control)
set_tests_properties(test_pi_controller PROPERTIES LABELS "unit;chrono-free")

# Hydraulic PTO assembly test (Chrono-free, HDF5-free)
build_unit_test(test_rectified_hydraulic_pto)
target_link_libraries(test_rectified_hydraulic_pto PRIVATE SEAStack::PTO)
set_tests_properties(test_rectified_hydraulic_pto PROPERTIES LABELS "unit;chrono-free")

# External force modules (optional)
if(TARGET seastack_external)
    add_executable(mock_external_module mock_external_module.cpp)
    target_compile_features(mock_external_module PRIVATE cxx_std_17)
    seastack_apply_compile_settings(mock_external_module)
    if(WIN32)
        target_link_libraries(mock_external_module PRIVATE ws2_32)
    endif()

    build_unit_test(test_ipc_external_force_model)
    target_link_libraries(test_ipc_external_force_model PRIVATE SEAStack::External)
    add_dependencies(test_ipc_external_force_model mock_external_module)
    set_tests_properties(test_ipc_external_force_model PROPERTIES LABELS "unit;chrono-free")

    build_unit_test(test_external_pto_model)
    target_link_libraries(test_external_pto_model PRIVATE SEAStack::External)
    set_tests_properties(test_external_pto_model PROPERTIES LABELS "unit;chrono-free")

    if(TARGET seastack_chrono_adapter AND SEASTACK_ENABLE_CHRONO)
        build_unit_test(test_external_pto_functors)
        target_link_libraries(test_external_pto_functors PRIVATE SEAStack::ChronoAdapter)
        target_include_directories(test_external_pto_functors PRIVATE "${CMAKE_BINARY_DIR}")
        set_tests_properties(test_external_pto_functors PROPERTIES LABELS "unit")
    endif()

    if(TARGET external_pto_example)
        add_test(
            NAME test_example_external_pto_mock
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:external_pto_example>"
            COMMAND $<TARGET_FILE:external_pto_example>
                --mock $<TARGET_FILE:mock_external_module>
        )
        set_tests_properties(test_example_external_pto_mock PROPERTIES
            LABELS "unit;chrono-free" COST 1)
        add_dependencies(external_pto_example mock_external_module)
    endif()

    # Prescribed-input golden tests for the three Python example modules
    # (linear / adaptive / hydraulic). Pure Python, no IPC/Chrono/RM3 run:
    # each case is checked against an independent oracle.
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_FOUND)
        add_test(
            NAME test_external_pto_examples_golden
            COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_SOURCE_DIR}/examples/external_pto/verify_examples.py"
        )
        set_tests_properties(test_external_pto_examples_golden PROPERTIES
            LABELS "unit;chrono-free" COST 1)

        add_test(
            NAME test_external_pto_python_api
            COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_SOURCE_DIR}/examples/external_pto/test_pto_python_api.py"
        )
        set_tests_properties(test_external_pto_python_api PROPERTIES
            LABELS "unit;chrono-free" COST 1)

        # IPC-level equivalence: the same prescribed trace through the full IPC
        # transport must match a direct in-process run of each module. Verifies
        # framing / JSON / config passing / C++ time-caching (no Chrono).
        if(TARGET external_pto_example)
            set(_ext_pto_cmp
                "${CMAKE_SOURCE_DIR}/examples/external_pto/compare_ipc_replay.py")
            set(_ext_pto_rm3 "${CMAKE_SOURCE_DIR}/data/demos/run_seastack/rm3")
            foreach(_case
                    "linear;external_pto;linear_damper_pto.py;LinearDamperPTO"
                    "adaptive;external_pto_adaptive;adaptive_damping_pto.py;AdaptiveDampingPTO"
                    "hydraulic;external_pto_hydraulic;hydraulic_accumulator_pto.py;HydraulicAccumulatorPTO")
                list(GET _case 0 _name)
                list(GET _case 1 _dir)
                list(GET _case 2 _script)
                list(GET _case 3 _class)
                add_test(
                    NAME test_external_pto_ipc_${_name}
                    COMMAND ${Python3_EXECUTABLE} "${_ext_pto_cmp}"
                        --exe $<TARGET_FILE:external_pto_example>
                        --module "${_ext_pto_rm3}/${_dir}/${_script}"
                        --class ${_class}
                )
                set_tests_properties(test_external_pto_ipc_${_name} PROPERTIES
                    LABELS "unit;chrono-free" COST 2
                    ENVIRONMENT "PYTHON=${Python3_EXECUTABLE}")
            endforeach()
        endif()

        # Focused Chrono regression: run the three RM3 external-PTO demos through
        # run_seastack. Linear is an exact-equivalence check vs a native Chrono
        # LinearPTO twin (generated on the fly); adaptive/hydraulic use aggregate
        # and cross-case ordering checks (no committed time-history baselines).
        # Skips (77) if h5py is unavailable.
        if(TARGET run_seastack AND SEASTACK_ENABLE_CHRONO)
            add_test(
                NAME test_external_pto_rm3_regression
                COMMAND ${Python3_EXECUTABLE}
                    "${CMAKE_SOURCE_DIR}/examples/external_pto/verify_rm3_regression.py"
                    --exe $<TARGET_FILE:run_seastack>
            )
            # Four ~600 s irregular-wave Chrono runs (3 demos + native twin).
            set_tests_properties(test_external_pto_rm3_regression PROPERTIES
                LABELS "external;chrono" COST 120 SKIP_RETURN_CODE 77)
            # On Windows, run_seastack needs Chrono/HDF5/VSG DLLs (VSG is loaded
            # even with --nogui) plus a `python` interpreter (the demos spawn
            # `python <module>.py`). We pass these directories to the test script
            # via a '|'-separated variable rather than on PATH, so the script's
            # own interpreter (which imports h5py) keeps a clean DLL search path
            # and only the run_seastack subprocess sees them.
            if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
                get_filename_component(_ext_pto_pydir "${Python3_EXECUTABLE}" DIRECTORY)
                set(_ext_pto_run_dirs
                    "$<TARGET_FILE_DIR:run_seastack>" "${_ext_pto_pydir}")
                if(TARGET Chrono::Chrono_core)
                    list(APPEND _ext_pto_run_dirs "$<TARGET_FILE_DIR:Chrono::Chrono_core>")
                endif()
                if(TARGET hdf5::hdf5-shared)
                    list(APPEND _ext_pto_run_dirs "$<TARGET_FILE_DIR:hdf5::hdf5-shared>")
                elseif(TARGET HDF5::HDF5)
                    list(APPEND _ext_pto_run_dirs "$<TARGET_FILE_DIR:HDF5::HDF5>")
                endif()
                if(VSG_DLL_DIR)
                    list(APPEND _ext_pto_run_dirs "${VSG_DLL_DIR}")
                endif()
                if(SEASTACK_ENABLE_MOORING AND TARGET moordyn)
                    list(APPEND _ext_pto_run_dirs "$<TARGET_FILE_DIR:moordyn>")
                endif()
                list(JOIN _ext_pto_run_dirs "|" _ext_pto_run_path)
                set_tests_properties(test_external_pto_rm3_regression PROPERTIES
                    ENVIRONMENT "SEASTACK_RUN_DLL_DIRS=${_ext_pto_run_path}")
            endif()
        endif()
    else()
        message(STATUS "Skipping test_external_pto_examples_golden (Python3 not found)")
    endif()
endif()

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

# Excitation TF interpolation on uniform vs non-uniform omega grids (Chrono-free, HDF5-free)
build_unit_test(test_excitation_nonuniform_grid)
target_link_libraries(test_excitation_nonuniform_grid PRIVATE SEAStack::Hydro)
set_tests_properties(test_excitation_nonuniform_grid PROPERTIES LABELS "unit;chrono-free")

# Two-TU Eigen allocator ABI: Core-style TU vs Chrono-facing TU must share the
# same Eigen heap protocol (allocate in one, destroy in the other, both ways).
# OBJECT libraries keep Chrono INTERFACE flags off the Core TU.
if(TARGET Chrono::Chrono_core AND TARGET SEAStack::Core)
    add_library(test_eigen_allocator_abi_core_tu OBJECT
        test_eigen_allocator_abi_core_tu.cpp)
    target_include_directories(test_eigen_allocator_abi_core_tu PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_eigen_allocator_abi_core_tu PUBLIC SEAStack::Core)
    target_compile_features(test_eigen_allocator_abi_core_tu PUBLIC cxx_std_17)
    seastack_apply_compile_settings(test_eigen_allocator_abi_core_tu)

    add_library(test_eigen_allocator_abi_chrono_tu OBJECT
        test_eigen_allocator_abi_chrono_tu.cpp)
    target_include_directories(test_eigen_allocator_abi_chrono_tu PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_eigen_allocator_abi_chrono_tu PUBLIC
        SEAStack::Core Chrono::Chrono_core)
    target_compile_features(test_eigen_allocator_abi_chrono_tu PUBLIC cxx_std_17)
    seastack_apply_compile_settings(test_eigen_allocator_abi_chrono_tu)

    add_executable(test_eigen_allocator_abi test_eigen_allocator_abi_main.cpp)
    target_include_directories(test_eigen_allocator_abi PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}")
    # Linking the OBJECT libs pulls in their .o files and usage requirements.
    # Chrono INTERFACE flags apply to the Chrono OBJECT TU and to main.cpp, not
    # to the Core OBJECT TU (compiled only against SEAStack::Core).
    target_link_libraries(test_eigen_allocator_abi PRIVATE
        test_eigen_allocator_abi_core_tu
        test_eigen_allocator_abi_chrono_tu
        Chrono::Chrono_core)
    target_compile_features(test_eigen_allocator_abi PRIVATE cxx_std_17)
    seastack_apply_compile_settings(test_eigen_allocator_abi)
    add_test(
        NAME test_eigen_allocator_abi
        WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_eigen_allocator_abi>"
        COMMAND $<TARGET_FILE:test_eigen_allocator_abi>)
    set_tests_properties(test_eigen_allocator_abi PROPERTIES LABELS "unit;chrono" COST 1)
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND DEFINED TEST_ENVIRONMENT)
        set_tests_properties(test_eigen_allocator_abi PROPERTIES ENVIRONMENT "${TEST_ENVIRONMENT}")
    endif()
endif()

# Constant-coefficient (regular-wave) radiation: H5 scaling, matching, builder,
# component, and Chrono added-mass replacement. Reads bundled sphere/rm3 H5 files.
if(TARGET seastack_hydro_io AND TARGET seastack_chrono_adapter)
    build_unit_test(test_radiation_constant_coefficients)
    target_link_libraries(test_radiation_constant_coefficients PRIVATE SEAStack::ChronoAdapter)
    target_compile_definitions(test_radiation_constant_coefficients PRIVATE
        "SEASTACK_TEST_SPHERE_H5_FILE=\"${CMAKE_SOURCE_DIR}/data/demos/run_seastack/iea_sphere/assets/hydroData/sphere.h5\""
        "SEASTACK_TEST_RM3_H5_FILE=\"${CMAKE_SOURCE_DIR}/data/demos/run_seastack/rm3/assets/hydroData/rm3.h5\"")
    set_tests_properties(test_radiation_constant_coefficients PROPERTIES LABELS "unit;chrono")

    # Block-style hull damping in hydro YAML reaches the simulation (TALOS-style:
    # one body, regular waves, constant_coefficients).
    build_unit_test(test_yaml_damping_regression)
    target_link_libraries(test_yaml_damping_regression PRIVATE SEAStack::ChronoAdapter)
    target_compile_definitions(test_yaml_damping_regression PRIVATE
        "SEASTACK_TEST_SPHERE_H5_FILE=\"${CMAKE_SOURCE_DIR}/data/demos/run_seastack/iea_sphere/assets/hydroData/sphere.h5\"")
    set_tests_properties(test_yaml_damping_regression PROPERTIES LABELS "unit;chrono")
else()
    message(STATUS "Skipping test_radiation_constant_coefficients and test_yaml_damping_regression (requires SEASTACK_ENABLE_HYDRO_IO + SEASTACK_ENABLE_CHRONO)")
endif()

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
