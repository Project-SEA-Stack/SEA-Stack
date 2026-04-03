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

# Campaign summary HDF5 (power_matrix_summary.h5): define for this target's TUs only.
# Link-time dependency on HydroIO comes transitively from ChronoAdapter.
if(TARGET seastack_hydro_io)
    target_compile_definitions(seastack_app_lib PRIVATE SEASTACK_HAVE_HYDRO_IO)
endif()

if(SEASTACK_ENABLE_VSG AND TARGET Chrono::Chrono_vsg)
    target_link_libraries(seastack_app_lib PUBLIC Chrono::Chrono_vsg)
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
