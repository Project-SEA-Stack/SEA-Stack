# apps/seastack — Standard SEA-Stack simulation application
# Depends: ChronoAdapter, Infra, and optionally Chrono VSG for visualization

# GUI sources (always compiled; visualization backend selected at build time)
set(GUI_SOURCES
    gui/guihelper.cpp
)

if(SEASTACK_ENABLE_VSG AND TARGET Chrono::Chrono_vsg)
    list(APPEND GUI_SOURCES
        gui/guihelperVSG.cpp
        gui/vsg_water_surface.cpp
        gui/vsg_mooring_lines.cpp
        gui/vsg_materials.cpp
        gui/vsg_lighting.cpp
        gui/vsg_gui_component.cpp
        gui/vsg_radiation_surface.cpp
    )
    if(SEASTACK_ENABLE_VEHICLE AND TARGET Chrono::Chrono_vehicle)
        list(APPEND GUI_SOURCES gui/guihelper_vehicle.cpp)
    endif()
endif()

# Runner library (not an executable itself)
add_library(seastack_app_lib STATIC
    run_from_yaml.cpp
    single_run.cpp
    campaign_runner.cpp
    cell_io.cpp
    app_init.cpp
    misc_options.cpp
    doctor.cpp
    ${GUI_SOURCES}
)

if(SEASTACK_ENABLE_EXTERNAL)
    target_sources(seastack_app_lib PRIVATE external_pto_yaml.cpp)
endif()

# SPH (Chrono::FSI) runner: only compiled when SPH support is enabled.
if(SEASTACK_ENABLE_SPH)
    target_sources(seastack_app_lib PRIVATE sph_run.cpp coupled_run.cpp)
endif()

target_include_directories(seastack_app_lib
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/gui"
)

# ChronoAdapter already PUBLIC-links Hydro, HydroIO (when built), Infra, PTO, Core, Chrono.
target_link_libraries(seastack_app_lib
    PUBLIC
        SEAStack::ChronoAdapter
        Chrono::Chrono_parsers
)

if(SEASTACK_ENABLE_EXTERNAL AND TARGET seastack_external)
    target_link_libraries(seastack_app_lib PUBLIC SEAStack::External)
    # external_pto_yaml.cpp uses yaml-cpp directly
    if(TARGET yaml-cpp::yaml-cpp)
        target_link_libraries(seastack_app_lib PRIVATE yaml-cpp::yaml-cpp)
    elseif(TARGET Chrono::yaml-cpp)
        target_link_libraries(seastack_app_lib PRIVATE Chrono::yaml-cpp)
    endif()
endif()

# Campaign summary HDF5 (power_matrix_summary.h5): define for this target's TUs only.
# Link-time dependency on HydroIO comes transitively from ChronoAdapter.
if(TARGET seastack_hydro_io)
    target_compile_definitions(seastack_app_lib PRIVATE SEASTACK_HAVE_HYDRO_IO)
endif()

if(SEASTACK_ENABLE_VSG AND TARGET Chrono::Chrono_vsg)
    target_link_libraries(seastack_app_lib PUBLIC Chrono::Chrono_vsg)
endif()

# SPH (Chrono::FSI): link the FSI coupling layer and the GPU SPH fluid solver.
# SEASTACK_HAVE_SPH gates the sph_run.cpp translation unit and the dispatch in
# run_from_yaml.cpp.
if(SEASTACK_ENABLE_SPH)
    target_compile_definitions(seastack_app_lib PUBLIC SEASTACK_HAVE_SPH)
    if(TARGET Chrono::Chrono_fsi)
        target_link_libraries(seastack_app_lib PUBLIC Chrono::Chrono_fsi)
    endif()
    if(TARGET Chrono::Chrono_fsisph)
        target_link_libraries(seastack_app_lib PUBLIC Chrono::Chrono_fsisph)
    endif()
    if(SEASTACK_ENABLE_VSG AND TARGET Chrono::Chrono_fsisph_vsg)
        target_link_libraries(seastack_app_lib PUBLIC Chrono::Chrono_fsisph_vsg)
    endif()
endif()

if(SEASTACK_ENABLE_VEHICLE AND TARGET Chrono::Chrono_vehicle)
    target_link_libraries(seastack_app_lib PUBLIC Chrono::Chrono_vehicle)
    if(TARGET Chrono::Chrono_vehicle_vsg)
        target_link_libraries(seastack_app_lib PUBLIC Chrono::Chrono_vehicle_vsg)
    endif()
endif()

target_compile_features(seastack_app_lib PUBLIC ${SEASTACK_CXX_STANDARD})

# Generated headers (version.h, config.h)
target_include_directories(seastack_app_lib PRIVATE "${CMAKE_BINARY_DIR}")

# Main executable
add_executable(run_seastack run_seastack.cpp)
target_link_libraries(run_seastack PRIVATE seastack_app_lib)
target_compile_features(run_seastack PRIVATE ${SEASTACK_CXX_STANDARD})

# Release ZIP layout: third-party dylibs install to ../lib; resolve at runtime.
if(APPLE)
    set_target_properties(run_seastack PROPERTIES INSTALL_RPATH "@loader_path/../lib")
elseif(UNIX)
    set_target_properties(run_seastack PROPERTIES INSTALL_RPATH "\$ORIGIN/../lib")
endif()

# POST_BUILD must run in this directory (same as add_executable(run_seastack)); macro from SeaStackWindowsPythonRuntime.cmake.
if(WIN32)
    seastack_windows_python_runtime_postbuild(run_seastack)
endif()
