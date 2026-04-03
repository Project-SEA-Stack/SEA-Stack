# SEAStack::Mooring — MoorDyn mooring line module
# Allowed deps: Core, MoorDyn
# Forbidden: Chrono, HDF5, VSG, Hydro (uses Core types, not hydro-specific)

add_library(seastack_mooring STATIC
    src/mooring_component.cpp
    src/moordyn_wrapper.cpp
)
add_library(SEAStack::Mooring ALIAS seastack_mooring)

target_include_directories(seastack_mooring
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(seastack_mooring
    PUBLIC SEAStack::Core
    PUBLIC SEAStack::Infra
)

# MoorDyn — external dependency
# MoorDyn's CMakeLists.txt uses CMAKE_SOURCE_DIR (upstream defect) which
# resolves to SEA-Stack's root when used via add_subdirectory().
# Workaround: copy the files MoorDyn expects at CMAKE_SOURCE_DIR.
# When MoorDyn upstream switches to PROJECT_SOURCE_DIR these copies can go.
if(TARGET MoorDyn::moordyn)
    target_link_libraries(seastack_mooring PRIVATE MoorDyn::moordyn)
else()
    if(EXISTS "${CMAKE_SOURCE_DIR}/extern/MoorDyn")
        if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/MoorDynConfig.cmake")
            file(COPY "${CMAKE_SOURCE_DIR}/extern/MoorDyn/cmake/MoorDynConfig.cmake"
                 DESTINATION "${CMAKE_SOURCE_DIR}/cmake/")
        endif()
        # TEMPORARY WORKAROUND: MoorDyn CPack expects LICENSE.txt at
        # CMAKE_SOURCE_DIR.  This is NOT the SEA-Stack project license
        # (that is /LICENSE).  The authoritative MoorDyn license copy lives
        # at third_party/licenses/MoorDyn-LICENSE.txt.  This auto-generated
        # file is gitignored and can be removed once MoorDyn upstream fixes
        # the CMAKE_SOURCE_DIR defect.
        if(NOT EXISTS "${CMAKE_SOURCE_DIR}/LICENSE.txt")
            file(COPY "${CMAKE_SOURCE_DIR}/extern/MoorDyn/LICENSE.txt"
                 DESTINATION "${CMAKE_SOURCE_DIR}/")
        endif()

        set(EXTERNAL_EIGEN OFF CACHE BOOL "" FORCE)
        set(PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
        set(FORTRAN_WRAPPER OFF CACHE BOOL "" FORCE)
        set(MATLAB_WRAPPER OFF CACHE BOOL "" FORCE)
        set(RUST_WRAPPER OFF CACHE BOOL "" FORCE)
        set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)

        set(_build_testing_saved ${BUILD_TESTING})
        set(BUILD_TESTING OFF)
        add_subdirectory("${CMAKE_SOURCE_DIR}/extern/MoorDyn"
                         "${CMAKE_BINARY_DIR}/extern/MoorDyn")
        set(BUILD_TESTING ${_build_testing_saved})

        if(TARGET moordyn)
            set_target_properties(moordyn PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "")
            target_link_libraries(seastack_mooring PRIVATE moordyn)
        endif()
    else()
        message(WARNING "MoorDyn not found. Mooring module will compile but not link.")
    endif()
endif()

# mooring_viz_data.h is in libs/core (lightweight, no deps).
# Mooring gets it transitively via SEAStack::Core.

target_compile_features(seastack_mooring PUBLIC cxx_std_17)
