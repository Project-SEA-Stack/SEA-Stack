# SEAStack::ChronoAdapter — Chrono coupling layer
# Allowed deps: Core, Hydro, HydroIO, PTO, Chrono
# Forbidden: VSG (unless viz-specific sub-target)

add_library(seastack_chrono_adapter STATIC
    src/hydro_system.cpp
    src/chrono_force_attacher.cpp
    src/chrono_hydro_coupler.cpp
    src/chrono_state_utils.cpp
    src/setup_from_yaml.cpp
    src/simulation_export.cpp
    src/helper.cpp
    src/pto_chrono_adapter.cpp
)
add_library(SEAStack::ChronoAdapter ALIAS seastack_chrono_adapter)

target_include_directories(seastack_chrono_adapter
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# Chrono MBS YAML registers TSDA/RSDA force functors without copying damping onto the link;
# simulation_export reads damping_coefficient from model YAML when GetDampingCoefficient()==0.
#
# Prefer Chrono's bundled yaml-cpp when available so that the yaml-cpp headers
# (which come from Chrono's chrono_thirdparty/yaml-cpp/include via Chrono::Chrono_core)
# match the linked library. Mixing a system libyaml-cpp.so with Chrono's headers can
# produce link errors against internal symbols (e.g. YAML::FpToString, YAML::Emitter::Write).
if(TARGET Chrono::yaml-cpp)
    set(_seastack_yaml_cpp_target Chrono::yaml-cpp)
else()
    find_package(yaml-cpp CONFIG QUIET)
    if(NOT yaml-cpp_FOUND)
        find_package(yaml-cpp QUIET)
    endif()
    if(TARGET yaml-cpp::yaml-cpp)
        set(_seastack_yaml_cpp_target yaml-cpp::yaml-cpp)
    elseif(TARGET yaml-cpp)
        set(_seastack_yaml_cpp_target yaml-cpp)
    else()
        message(FATAL_ERROR "yaml-cpp not found (required for Chrono adapter).")
    endif()
endif()

target_link_libraries(seastack_chrono_adapter
    PUBLIC
        SEAStack::Core
        SEAStack::Hydro
        SEAStack::Infra
        SEAStack::PTO
        Chrono::Chrono_core
        OpenMP::OpenMP_CXX
    PRIVATE
        ${_seastack_yaml_cpp_target}
)

if(TARGET seastack_hydro_io)
    target_link_libraries(seastack_chrono_adapter PUBLIC SEAStack::HydroIO)
    target_compile_definitions(seastack_chrono_adapter PRIVATE SEASTACK_HAVE_HYDRO_IO)
endif()

if(TARGET seastack_mooring)
    target_link_libraries(seastack_chrono_adapter PUBLIC SEAStack::Mooring)
endif()

# External PTO rich functors; SEASTACK_HAVE_EXTERNAL comes from seastack/config.h
# when SEASTACK_ENABLE_EXTERNAL=ON.
if(TARGET seastack_external)
    target_link_libraries(seastack_chrono_adapter PUBLIC SEAStack::External)
endif()

target_compile_features(seastack_chrono_adapter PUBLIC ${SEASTACK_CXX_STANDARD})

# Generated headers (version.h, config.h) — PUBLIC because hydro_system.h includes seastack/config.h
target_include_directories(seastack_chrono_adapter PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>")
