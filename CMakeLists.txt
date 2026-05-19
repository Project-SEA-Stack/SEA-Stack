cmake_minimum_required(VERSION 3.21)

# CMP0091: MSVC runtime library selection (align with Chrono/HDF5 to avoid CRT mismatch)
cmake_policy(SET CMP0091 NEW)
# CMP0074: Prefer <PackageName>_ROOT for find_package()
cmake_policy(SET CMP0074 NEW)

project(SEAStack
    VERSION 1.0.0
    DESCRIPTION "Modular offshore system simulation platform"
    LANGUAGES CXX
)

# Version suffix for prerelease tags (e.g., "-beta.1", "-rc.1", or empty for releases).
# Keep this in sync with the current release branch name (release/v<PROJECT_VERSION><SUFFIX>).
set(SEASTACK_VERSION_SUFFIX "-beta.1")
set(SEASTACK_VERSION_FULL "${PROJECT_VERSION}${SEASTACK_VERSION_SUFFIX}")

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "In-source builds not allowed.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Route all install rules without an explicit COMPONENT to the "sdk"
# component so they are excluded from the runtime-only release package.
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME sdk)

# MSVC: use /I for system includes so third-party headers work
if(MSVC)
    set(CMAKE_INCLUDE_SYSTEM_FLAG_C "/I")
    set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "/I")
endif()

# Helper function to apply SEAStack-specific compile settings to a target
# (prevents leaking to third-party code like MoorDyn)
function(seastack_apply_compile_settings target)
    get_target_property(_type ${target} TYPE)
    if(_type STREQUAL "INTERFACE_LIBRARY")
        return()
    endif()

    # Windows/Eigen alignment and MSVC warnings (CRT/ABI compatibility)
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        if(MSVC AND MSVC_VERSION GREATER_EQUAL 1915)
            target_compile_definitions(${target} PRIVATE ENABLE_EXTENDED_ALIGNED_STORAGE)
        endif()
    endif()
    if(MSVC)
        target_compile_definitions(${target} PRIVATE
            _CRT_SECURE_NO_DEPRECATE
            _SCL_SECURE_NO_DEPRECATE
            NOMINMAX
        )
        target_compile_options(${target} PRIVATE /wd4275 /wd4251)
        # Chrono's exported interface adds /arch:AVX2, which changes Eigen's alignment
        # from 16 to 32 bytes. This switches Eigen's allocator from malloc/free to
        # handmade_aligned_malloc/free. If any static library using Eigen is compiled
        # WITHOUT AVX2, Eigen objects allocated there (via malloc) will be freed in
        # AVX2-compiled code (via handmade_aligned_free), causing heap corruption
        # (STATUS_HEAP_CORRUPTION / 0xc0000374). Setting AVX2 project-wide ensures
        # all translation units use the same Eigen allocator.
        target_compile_options(${target} PRIVATE /arch:AVX2)
    endif()
endfunction()

# Output layout
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Per-config subdirs for multi-config generators (Visual Studio) so exes/DLLs go to bin/Release, bin/Debug
if(CMAKE_CONFIGURATION_TYPES)
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER "${cfg}" cfg_uc)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg_uc} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${cfg})
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${cfg_uc} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${cfg})
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${cfg_uc} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${cfg})
    endforeach()
elseif(CMAKE_BUILD_TYPE)
    string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type_uc)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${build_type_uc} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${build_type_uc} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${build_type_uc} ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE})
endif()

# ── Options ──────────────────────────────────────────────────────────────────
option(SEASTACK_ENABLE_CHRONO    "Build Chrono adapter and apps"           ON)
option(SEASTACK_ENABLE_HYDRO_IO  "Build HDF5-based hydro IO"              ON)
option(SEASTACK_ENABLE_MOORING   "Build MoorDyn mooring module"           OFF)
option(SEASTACK_ENABLE_VSG       "Enable VSG visualization in seastack"   OFF)
option(SEASTACK_ENABLE_TESTS     "Build tests"                            ON)
option(SEASTACK_ENABLE_DEMOS     "Build demo executables"                 OFF)
option(SEASTACK_ENABLE_APPS      "Build application executables"          ON)

# ── Find Eigen (direct dependency, not via Chrono) ───────────────────────────
# Eigen can come from: system install, vcpkg, or Chrono's bundled copy.
# If Chrono is enabled, its include dirs typically provide Eigen already.
# For Chrono-free builds, find_package(Eigen3) is required.
# ── Eigen ────────────────────────────────────────────────────────────────────
# If EIGEN3_INCLUDE_DIR is set, create the Eigen3::Eigen target directly
# (avoids version-compat issues between Eigen 5.x and the 3.3 check in
# Chrono's FindEigen3.cmake). Otherwise fall back to find_package.
if(EIGEN3_INCLUDE_DIR)
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
    endif()
else()
    find_package(Eigen3 REQUIRED NO_MODULE)
endif()

# ── OpenMP ───────────────────────────────────────────────────────────────────
# Find OpenMP early so the target exists when Chrono's imports reference it.
# AppleClang does not ship OpenMP; LLVM libomp from Homebrew is the usual provider.
if(APPLE AND NOT OpenMP_ROOT)
    foreach(_seastack_libomp
            "$ENV{HOMEBREW_PREFIX}/opt/libomp"
            "/opt/homebrew/opt/libomp"
            "/usr/local/opt/libomp")
        if(EXISTS "${_seastack_libomp}/include/omp.h")
            set(OpenMP_ROOT "${_seastack_libomp}" CACHE PATH "LLVM OpenMP (libomp) for FindOpenMP")
            break()
        endif()
    endforeach()
endif()

find_package(OpenMP COMPONENTS CXX)

if(SEASTACK_ENABLE_CHRONO)
    if(NOT Chrono_DIR)
        message("Specify Chrono_DIR for Chrono adapter builds.")
        set(Chrono_DIR "" CACHE PATH "Directory containing Chrono CMake package config (ChronoConfig.cmake or chrono-config.cmake)")
        return()
    endif()

    # HDF5 config-mode pre-load BEFORE creating shims
    find_package(HDF5 CONFIG QUIET)

    # Chrono exports may reference hdf5_tools targets that don't exist in all
    # HDF5 installations.  Create empty shims only when necessary so that
    # ChronoTargets.cmake loads without errors.
    if(NOT TARGET hdf5::hdf5_tools-shared)
        add_library(hdf5::hdf5_tools-shared INTERFACE IMPORTED)
    endif()
    if(NOT TARGET hdf5::hdf5_tools-static)
        add_library(hdf5::hdf5_tools-static INTERFACE IMPORTED)
    endif()

    # Chrono's import targets reference vsg::vsg / vsgImGui / vsgXchange.
    # When VSG is ON, find_package(Chrono COMPONENTS VSG) will call
    # find_package(vsg) etc. and create the real targets.  When VSG is OFF,
    # provide empty shims so ChronoTargets.cmake loads without errors.
    if(NOT SEASTACK_ENABLE_VSG)
        foreach(_vsg_target vsg::vsg vsgXchange::vsgXchange vsgImGui::vsgImGui)
            if(NOT TARGET ${_vsg_target})
                add_library(${_vsg_target} INTERFACE IMPORTED)
            endif()
        endforeach()
    endif()

    set(_chrono_components Parsers)
    if(SEASTACK_ENABLE_VSG)
        list(APPEND _chrono_components VSG)
    endif()
    find_package(Chrono CONFIG REQUIRED COMPONENTS ${_chrono_components})

    if(DEFINED Chrono_CONFIG)
        message(STATUS "Chrono package config file: ${Chrono_CONFIG}")
    endif()
    if(DEFINED Chrono_VERSION)
        message(STATUS "Chrono version: ${Chrono_VERSION}")
    endif()

    if(NOT OpenMP_CXX_FOUND)
        message(FATAL_ERROR
            "OpenMP (C++) not found. SEA-Stack links OpenMP for the Chrono adapter and hydrodynamics.\n"
            "  macOS: brew install libomp\n"
            "  Then reconfigure CMake. If FindOpenMP still fails, pass:\n"
            "    -DOpenMP_ROOT=/opt/homebrew/opt/libomp\n"
            "  (or /usr/local/opt/libomp on Intel Homebrew).")
    endif()

    # If Chrono set HDF5_DIR, retry the find
    if(NOT HDF5_FOUND AND DEFINED CACHE{HDF5_DIR})
        if("${HDF5_DIR}" MATCHES "NOTFOUND")
            unset(HDF5_DIR CACHE)
            find_package(HDF5 CONFIG QUIET)
        endif()
    endif()

    # Use same C++ standard as Chrono for ABI compatibility
    if(DEFINED CHRONO_CXX_STANDARD)
        set(SEASTACK_CXX_STANDARD ${CHRONO_CXX_STANDARD})
        message(STATUS "C++ standard (from Chrono): ${SEASTACK_CXX_STANDARD}")
    else()
        set(SEASTACK_CXX_STANDARD cxx_std_17)
        message(STATUS "C++ standard: ${SEASTACK_CXX_STANDARD}")
    endif()

    # Copy Chrono DLLs (core, parsers, yaml-cpp, optionally vsg) to the
    # output bin directory at build time.  This uses the function exported
    # by chrono-config.cmake and works for both script and IDE builds.
    if(WIN32)
        add_CHRONO_DLLS_copy_command()
        include("${CMAKE_SOURCE_DIR}/cmake/SeaStackWindowsPythonRuntime.cmake")
    endif()
endif()

# ── Data directory (copy source data into build tree) ──
set(SEASTACK_BUILD_DATA "${CMAKE_BINARY_DIR}/data")
file(COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${SEASTACK_BUILD_DATA})
message(STATUS "Data directory copied to: ${SEASTACK_BUILD_DATA}/")

# Provide model data at workflow-neutral paths for C++ demos and tests.
# The canonical files live under demos/run_seastack/*/assets/; these copies
# avoid coupling C++ demo code to the run_seastack directory layout.
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/rm3/assets/geometry
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/rm3)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/rm3/assets/hydroData
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/rm3)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/rm3/assets/mooring
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/rm3)

file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/f3of/assets/geometry
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/f3of)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/f3of/assets/hydroData
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/f3of)

file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/oswec/assets/geometry
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/oswec)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/oswec/assets/hydroData
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/oswec)

file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/iea_sphere/assets/geometry
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/sphere)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/iea_sphere/assets/hydroData
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/sphere)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/iea_sphere/assets/eta
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/sphere)

file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/5sa/assets/geometry
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/5sa)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/5sa/assets/hydroData
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/5sa)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/5sa/spreading/mooring
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/5sa)

file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/trimaran/assets/geometry
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/trimaran)
file(COPY ${SEASTACK_BUILD_DATA}/demos/run_seastack/trimaran/assets/hydroData
     DESTINATION ${SEASTACK_BUILD_DATA}/demos/trimaran)

# Copy Chrono runtime data (skybox, colormaps, vsg, logo) into build tree so
# SetInitialEnvironment() finds HC_DATA_DIR/chrono/ at runtime.
if(SEASTACK_ENABLE_CHRONO AND EXISTS "${CHRONO_DATA_DIR}")
    if(EXISTS "${CHRONO_DATA_DIR}/colormaps")
        file(COPY ${CHRONO_DATA_DIR}/colormaps DESTINATION ${SEASTACK_BUILD_DATA}/chrono)
        # Chrono 9 ChColormap has a static-init double-prefix bug: paths
        # baked at static init with the default "../data/" prefix get
        # re-prefixed by GetChronoDataFile() at load time.  When the Chrono
        # data path is ".../build/data/chrono/", the effective lookup becomes
        # ".../build/data/chrono/../data/colormaps/..." = ".../build/data/data/colormaps/...".
        # Mirror colormaps there so both the correct and the buggy path resolve.
        file(COPY ${CHRONO_DATA_DIR}/colormaps DESTINATION ${SEASTACK_BUILD_DATA}/data)
    endif()
    if(EXISTS "${CHRONO_DATA_DIR}/skybox")
        file(COPY ${CHRONO_DATA_DIR}/skybox DESTINATION ${SEASTACK_BUILD_DATA}/chrono)
    endif()
    if(EXISTS "${CHRONO_DATA_DIR}/vsg")
        # Copy only the VSG runtime data SEAStack needs (skip Chrono demo
        # models like forklift/ and sample textures like statue_1920.jpg).
        foreach(_vsg_subdir fonts shaders)
            if(EXISTS "${CHRONO_DATA_DIR}/vsg/${_vsg_subdir}")
                file(COPY "${CHRONO_DATA_DIR}/vsg/${_vsg_subdir}"
                     DESTINATION "${SEASTACK_BUILD_DATA}/chrono/vsg")
            endif()
        endforeach()
        # Sky models (top-level .vsgb files only, not subdirectories)
        file(GLOB _vsg_sky_models "${CHRONO_DATA_DIR}/vsg/models/*.vsgb")
        foreach(_f ${_vsg_sky_models})
            file(COPY "${_f}" DESTINATION "${SEASTACK_BUILD_DATA}/chrono/vsg/models")
        endforeach()
        # Textures (skip demo textures)
        file(GLOB _vsg_textures "${CHRONO_DATA_DIR}/vsg/textures/*")
        foreach(_f ${_vsg_textures})
            get_filename_component(_fname "${_f}" NAME)
            if(NOT _fname STREQUAL "statue_1920.jpg")
                file(COPY "${_f}" DESTINATION "${SEASTACK_BUILD_DATA}/chrono/vsg/textures")
            endif()
        endforeach()
        # imgui config
        if(EXISTS "${CHRONO_DATA_DIR}/vsg/imgui.ini")
            file(COPY "${CHRONO_DATA_DIR}/vsg/imgui.ini"
                 DESTINATION "${SEASTACK_BUILD_DATA}/chrono/vsg")
        endif()
    endif()
    if(EXISTS "${CHRONO_DATA_DIR}/logo_chrono_alpha.png")
        file(COPY ${CHRONO_DATA_DIR}/logo_chrono_alpha.png DESTINATION ${SEASTACK_BUILD_DATA}/chrono)
    endif()
endif()

# ── Generated config headers ─────────────────────────────────────────────────
if(SEASTACK_ENABLE_VSG)
    set(SEASTACK_HAS_VSG "#define SEASTACK_HAVE_VSG")
else()
    set(SEASTACK_HAS_VSG "/* #undef SEASTACK_HAVE_VSG */")
endif()
if(SEASTACK_ENABLE_MOORING)
    set(SEASTACK_HAS_MOORDYN "#define SEASTACK_HAVE_MOORDYN")
else()
    set(SEASTACK_HAS_MOORDYN "/* #undef SEASTACK_HAVE_MOORDYN */")
endif()
set(SEASTACK_DATA_DIR "#define SEASTACK_DATA_DIR_PATH \"${SEASTACK_BUILD_DATA}\"")
if(SEASTACK_ENABLE_CHRONO AND DEFINED CHRONO_DATA_DIR)
    set(SEASTACK_CHRONO_DATA_DIR "#define SEASTACK_CHRONO_DATA_DIR_PATH \"${CHRONO_DATA_DIR}\"")
else()
    set(SEASTACK_CHRONO_DATA_DIR "/* Chrono not enabled */")
endif()

configure_file(${CMAKE_SOURCE_DIR}/cmake/version.h.in
               ${CMAKE_BINARY_DIR}/seastack/version.h @ONLY)
configure_file(${CMAKE_SOURCE_DIR}/cmake/config.h.in
               ${CMAKE_BINARY_DIR}/seastack/config.h @ONLY)

# Generated headers will be added to specific targets that need them
# (seastack_infra, seastack_chrono_adapter, seastack_app_lib, run_seastack)

# ── Always-built libraries ───────────────────────────────────────────────────
add_subdirectory(libs/core)
add_subdirectory(libs/hydro)
add_subdirectory(libs/infrastructure)
add_subdirectory(libs/pto)
add_subdirectory(libs/control)

if(SEASTACK_ENABLE_HYDRO_IO)
    add_subdirectory(libs/hydro_io)
endif()

if(SEASTACK_ENABLE_MOORING)
    add_subdirectory(libs/mooring)
endif()

# ── Chrono-dependent targets ─────────────────────────────────────────────────
if(SEASTACK_ENABLE_CHRONO)
    add_subdirectory(adapters/chrono)

    if(SEASTACK_ENABLE_APPS)
        add_subdirectory(apps/seastack)
    endif()
endif()

# ── Demos ─────────────────────────────────────────────────────────────────────
if(SEASTACK_ENABLE_CHRONO AND SEASTACK_ENABLE_DEMOS)
    add_subdirectory(demos)
endif()

# ── Examples ─────────────────────────────────────────────────────────────────
add_subdirectory(examples/standalone_controller)
if(SEASTACK_ENABLE_HYDRO_IO)
    add_subdirectory(examples/standalone_hydro)
endif()

# ── Apply compile settings to all SEAStack targets ──────────────────────────
# (prevents leaking to third-party code like MoorDyn)
foreach(_target
    seastack_core
    seastack_hydro
    seastack_infra
    seastack_pto
    seastack_control
)
    if(TARGET ${_target})
        seastack_apply_compile_settings(${_target})
    endif()
endforeach()

if(TARGET seastack_hydro_io)
    seastack_apply_compile_settings(seastack_hydro_io)
endif()
if(TARGET seastack_mooring)
    seastack_apply_compile_settings(seastack_mooring)
endif()
if(TARGET seastack_chrono_adapter)
    seastack_apply_compile_settings(seastack_chrono_adapter)
endif()
if(TARGET seastack_app_lib)
    seastack_apply_compile_settings(seastack_app_lib)
endif()
if(TARGET run_seastack)
    seastack_apply_compile_settings(run_seastack)
endif()
if(TARGET standalone_controller)
    seastack_apply_compile_settings(standalone_controller)
endif()
if(TARGET standalone_hydro)
    seastack_apply_compile_settings(standalone_hydro)
endif()

# ── Tests ────────────────────────────────────────────────────────────────────
# Helper (Windows): set TEST_ENVIRONMENT so ctest finds Chrono/HDF5 DLLs
function(seastack_configure_test_environment)
    set(DLL_DIRS "")
    set(SEASTACK_DLL_DIRS "${CMAKE_BINARY_DIR}/bin/Release;${CMAKE_BINARY_DIR}/bin/RelWithDebInfo;${CMAKE_BINARY_DIR}/bin/MinSizeRel;${CMAKE_BINARY_DIR}/bin/Debug")
    list(APPEND DLL_DIRS ${SEASTACK_DLL_DIRS})
    if(TARGET Chrono::Chrono_core)
        get_target_property(_dll Chrono::Chrono_core IMPORTED_LOCATION_RELEASE)
        if(_dll)
            get_filename_component(_dir "${_dll}" DIRECTORY)
            list(APPEND DLL_DIRS "${_dir}")
        endif()
        get_target_property(_dll Chrono::Chrono_core IMPORTED_LOCATION_DEBUG)
        if(_dll)
            get_filename_component(_dir "${_dll}" DIRECTORY)
            list(APPEND DLL_DIRS "${_dir}")
        endif()
    endif()
    if(TARGET hdf5::hdf5-shared)
        get_target_property(_dll hdf5::hdf5-shared IMPORTED_LOCATION_RELEASE)
        if(_dll)
            get_filename_component(_dir "${_dll}" DIRECTORY)
            list(APPEND DLL_DIRS "${_dir}")
        endif()
    elseif(TARGET HDF5::HDF5)
        get_target_property(_dll HDF5::HDF5 IMPORTED_LOCATION_RELEASE)
        if(_dll)
            get_filename_component(_dir "${_dll}" DIRECTORY)
            list(APPEND DLL_DIRS "${_dir}")
        endif()
    endif()
    if(SEASTACK_ENABLE_MOORING AND TARGET moordyn)
        list(APPEND DLL_DIRS "$<TARGET_FILE_DIR:moordyn>")
    endif()
    set(TEST_ENVIRONMENT "PATH=${DLL_DIRS};$ENV{PATH}" PARENT_SCOPE)
endfunction()

if(SEASTACK_ENABLE_TESTS)
    enable_testing()
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND SEASTACK_ENABLE_CHRONO)
        seastack_configure_test_environment()
    endif()
    add_subdirectory(tests)
endif()

# ── Installation & Packaging ──────────────────────────────────────────────────

# ---------- Library install & export ----------

# Always-built library targets
set(_seastack_lib_targets seastack_core seastack_hydro seastack_infra seastack_pto seastack_control)

if(TARGET seastack_hydro_io)
    list(APPEND _seastack_lib_targets seastack_hydro_io)
endif()
if(TARGET seastack_mooring)
    list(APPEND _seastack_lib_targets seastack_mooring)
endif()
if(TARGET seastack_chrono_adapter)
    list(APPEND _seastack_lib_targets seastack_chrono_adapter)
endif()

# Imported target names for find_package consumers (NAMESPACE SEAStack:: + EXPORT_NAME).
# Build tree uses ALIAS targets SEAStack::Infra etc.; without EXPORT_NAME the install
# export would emit SEAStack::seastack_infra instead.
set_target_properties(seastack_core PROPERTIES EXPORT_NAME Core)
set_target_properties(seastack_hydro PROPERTIES EXPORT_NAME Hydro)
set_target_properties(seastack_infra PROPERTIES EXPORT_NAME Infra)
set_target_properties(seastack_pto PROPERTIES EXPORT_NAME PTO)
set_target_properties(seastack_control PROPERTIES EXPORT_NAME Control)
if(TARGET seastack_hydro_io)
    set_target_properties(seastack_hydro_io PROPERTIES EXPORT_NAME HydroIO)
endif()
if(TARGET seastack_mooring)
    set_target_properties(seastack_mooring PROPERTIES EXPORT_NAME Mooring)
endif()
if(TARGET seastack_chrono_adapter)
    set_target_properties(seastack_chrono_adapter PROPERTIES EXPORT_NAME ChronoAdapter)
endif()

install(TARGETS ${_seastack_lib_targets}
    EXPORT SEAStackTargets
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin
    INCLUDES DESTINATION include
)

# Install public headers for each module
install(DIRECTORY libs/core/include/          DESTINATION include)
install(DIRECTORY libs/hydro/include/         DESTINATION include)
install(DIRECTORY libs/infrastructure/include/ DESTINATION include)
install(DIRECTORY libs/pto/include/           DESTINATION include)
install(DIRECTORY libs/control/include/       DESTINATION include)
if(TARGET seastack_hydro_io)
    install(DIRECTORY libs/hydro_io/include/ DESTINATION include)
endif()
if(TARGET seastack_mooring)
    install(DIRECTORY libs/mooring/include/ DESTINATION include)
endif()
if(TARGET seastack_chrono_adapter)
    install(DIRECTORY adapters/chrono/include/ DESTINATION include)
endif()

# Install generated headers
install(FILES
    ${CMAKE_BINARY_DIR}/seastack/config.h
    ${CMAKE_BINARY_DIR}/seastack/version.h
    DESTINATION include/seastack
)

# Export targets file
install(EXPORT SEAStackTargets
    FILE SEAStackTargets.cmake
    NAMESPACE SEAStack::
    DESTINATION lib/cmake/SEAStack
)

# Generate and install package config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/SEAStackConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Package config: flags must always be ON or OFF (never empty), or if() in SEAStackConfig.cmake
# becomes `if()` and find_dependency for MoorDyn/Chrono is skipped while exports still reference them.
set(SEASTACK_INSTALL_HYDRO_IO OFF)
set(SEASTACK_INSTALL_MOORING OFF)
set(SEASTACK_INSTALL_CHRONO_ADAPTER OFF)
if(TARGET seastack_hydro_io)
    set(SEASTACK_INSTALL_HYDRO_IO ON)
endif()
if(TARGET seastack_mooring)
    set(SEASTACK_INSTALL_MOORING ON)
endif()
if(TARGET seastack_chrono_adapter)
    set(SEASTACK_INSTALL_CHRONO_ADAPTER ON)
endif()
configure_package_config_file(
    "${CMAKE_SOURCE_DIR}/cmake/SEAStackConfig.cmake.in"
    "${CMAKE_BINARY_DIR}/SEAStackConfig.cmake"
    INSTALL_DESTINATION lib/cmake/SEAStack
    PATH_VARS SEASTACK_INSTALL_HYDRO_IO
)
install(FILES
    "${CMAKE_BINARY_DIR}/SEAStackConfig.cmake"
    "${CMAKE_BINARY_DIR}/SEAStackConfigVersion.cmake"
    DESTINATION lib/cmake/SEAStack
)

# ---------- Executables ----------

# Install main CLI executable (non-Windows: bundle transitive shared libs into lib/)
if(TARGET run_seastack)
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        install(TARGETS run_seastack CONFIGURATIONS Release DESTINATION bin COMPONENT runtime)
    else()
        # RUNTIME_DEPENDENCIES must follow targets immediately (CMake signature).
        # macOS requires FRAMEWORK DESTINATION when using RUNTIME_DEPENDENCIES.
        # COMPONENT must be per destination; otherwise the executable lands in sdk only.
        set(_seastack_dep_exclude
            "^/System/Library/"
            "^/usr/lib/"
            "^/lib/"
            "^/usr/libexec/"
            # Transitive Chrono/SWIG stacks may reference Python; SEA-Stack does not ship it.
            "Python3\\.framework"
            "Python\\.framework")
        if(APPLE)
            install(TARGETS run_seastack
                RUNTIME_DEPENDENCIES
                    POST_EXCLUDE_REGEXES ${_seastack_dep_exclude}
                CONFIGURATIONS Release
                RUNTIME DESTINATION bin COMPONENT runtime
                LIBRARY DESTINATION lib COMPONENT runtime
                FRAMEWORK DESTINATION lib COMPONENT runtime)
        else()
            install(TARGETS run_seastack
                RUNTIME_DEPENDENCIES
                    POST_EXCLUDE_REGEXES ${_seastack_dep_exclude}
                CONFIGURATIONS Release
                RUNTIME DESTINATION bin COMPONENT runtime
                LIBRARY DESTINATION lib COMPONENT runtime)
        endif()
        unset(_seastack_dep_exclude)
    endif()
endif()

# Chrono-free example (always built when examples are enabled). SDK only — not in runtime ZIP.
if(TARGET standalone_controller)
    install(TARGETS standalone_controller CONFIGURATIONS Release DESTINATION bin COMPONENT sdk)
endif()

# Reference C++ demo (short sphere decay) — requires Chrono + SEASTACK_ENABLE_DEMOS at configure time.
if(TARGET demo_sphere_decay)
    install(TARGETS demo_sphere_decay CONFIGURATIONS Release DESTINATION bin COMPONENT sdk)
endif()

# On Windows, install third-party DLLs into bin/
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # Chrono DLLs -- glob all DLLs from the Chrono bin directory to catch
    # transitive dependencies (yaml-cpp, etc.) not listed in CHRONO_TARGETS.
    if(TARGET Chrono::Chrono_core)
        get_target_property(_chrono_core_dll Chrono::Chrono_core IMPORTED_LOCATION_RELEASE)
        if(_chrono_core_dll)
            get_filename_component(_chrono_dll_dir "${_chrono_core_dll}" DIRECTORY)
            file(GLOB _chrono_dlls "${_chrono_dll_dir}/*.dll")
            # Drop MSVC/ucrt runtime DLLs (handled by InstallRequiredSystemLibraries)
            list(FILTER _chrono_dlls EXCLUDE REGEX "(concrt|msvcp|vcruntime|ucrtbase|vcomp)[0-9_]*\\.dll$")
            # Drop HDF5 DLLs (installed from their own imported targets below).
            # Keep szip/zlib -- they are HDF5 compression deps not covered by HDF5 targets.
            list(FILTER _chrono_dlls EXCLUDE REGEX "hdf5[_a-z]*\\.dll$")
            # Drop Chrono modules not used by SEAStack
            list(FILTER _chrono_dlls EXCLUDE REGEX "ChronoModels_robot\\.dll$")
            foreach(_dll ${_chrono_dlls})
                install(FILES "${_dll}" DESTINATION bin COMPONENT runtime)
            endforeach()
        endif()
    endif()

    # libpython (pythonNN.dll) when Chrono_parsers imports it — not shipped next to Chrono DLLs.
    if(DEFINED SEASTACK_WINDOWS_PYTHON_DLL AND SEASTACK_WINDOWS_PYTHON_DLL)
        install(FILES "${SEASTACK_WINDOWS_PYTHON_DLL}" DESTINATION bin COMPONENT runtime)
    endif()
    if(DEFINED SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS AND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS)
        foreach(_seastack_py_extra IN LISTS SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS)
            if(EXISTS "${_seastack_py_extra}")
                install(FILES "${_seastack_py_extra}" DESTINATION bin COMPONENT runtime)
            endif()
        endforeach()
    endif()

    # HDF5 DLLs (only when using shared HDF5)
    if(TARGET hdf5::hdf5-shared)
        foreach(_hdf5_tgt hdf5::hdf5-shared hdf5::hdf5_cpp-shared hdf5::hdf5_hl-shared hdf5::hdf5_hl_cpp-shared)
            if(TARGET ${_hdf5_tgt})
                get_target_property(_hdf5_dll ${_hdf5_tgt} IMPORTED_LOCATION_RELEASE)
                if(_hdf5_dll AND EXISTS "${_hdf5_dll}")
                    install(FILES "${_hdf5_dll}" DESTINATION bin COMPONENT runtime)
                endif()
            endif()
        endforeach()
        # vcpkg-style HDF5: zlib1.dll / szip.dll sit next to hdf5.dll but are not
        # separate imported targets; install them so staged installs and CPack run.
        get_target_property(_hdf5_main_dll hdf5::hdf5-shared IMPORTED_LOCATION_RELEASE)
        if(_hdf5_main_dll AND EXISTS "${_hdf5_main_dll}")
            get_filename_component(_hdf5_peer_dir "${_hdf5_main_dll}" DIRECTORY)
            foreach(_hdf5_peer_dll IN ITEMS zlib1.dll szip.dll)
                set(_hdf5_peer "${_hdf5_peer_dir}/${_hdf5_peer_dll}")
                if(EXISTS "${_hdf5_peer}")
                    install(FILES "${_hdf5_peer}" DESTINATION bin COMPONENT runtime)
                endif()
            endforeach()
        endif()
    endif()

    # VSG DLLs -- always installed because Chrono_parsers.dll has a transitive
    # runtime dependency on Chrono_vsg.dll -> vsg-*.dll / vsgImGui.dll / vsgXchange.dll.
    # Chrono usually sets VSG_DLL_DIR from vsg::vsg. Fallback: derive from vsg_DIR layout
    # (vcpkg uses .../<triplet>/share/vsg -> bin is two levels above share; CMake installs
    # often use .../lib/cmake/vsg -> prefix/bin is two levels above lib/cmake).
    if(NOT DEFINED VSG_DLL_DIR OR VSG_DLL_DIR STREQUAL "")
        if(DEFINED vsg_DIR)
            get_filename_component(_vsg_up1 "${vsg_DIR}" DIRECTORY)
            get_filename_component(_vsg_up1_name "${_vsg_up1}" NAME)
            if(_vsg_up1_name STREQUAL "share")
                get_filename_component(_vsg_root "${_vsg_up1}" DIRECTORY)
            else()
                get_filename_component(_vsg_root "${_vsg_up1}" DIRECTORY)
                get_filename_component(_vsg_root "${_vsg_root}" DIRECTORY)
            endif()
            set(VSG_DLL_DIR "${_vsg_root}/bin")
        endif()
    endif()
    if(DEFINED VSG_DLL_DIR AND EXISTS "${VSG_DLL_DIR}")
        file(GLOB _vsg_dlls "${VSG_DLL_DIR}/*.dll")
        # Exclude debug-build DLLs (*_d.dll convention used by vsg, draco)
        list(FILTER _vsg_dlls EXCLUDE REGEX "_d\\.dll$")
        # Exclude debug-build DLLs (appended-d convention used by glslang/SPIRV)
        set(_vsg_debug_dlls
            glslangd.dll
            glslang-default-resource-limitsd.dll
            SPIRVd.dll
            SPIRV-Tools-sharedd.dll
            SPVRemapperd.dll)
        foreach(_dbg ${_vsg_debug_dlls})
            list(REMOVE_ITEM _vsg_dlls "${VSG_DLL_DIR}/${_dbg}")
        endforeach()
        if(_vsg_dlls)
            install(FILES ${_vsg_dlls} DESTINATION bin COMPONENT runtime)
        endif()
    endif()

endif()

# MoorDyn shared library -- explicit runtime component for CPack (Windows: bin;
# macOS/Linux: lib next to bundled Chrono/VSG deps).
if(SEASTACK_ENABLE_MOORING AND TARGET moordyn)
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        install(TARGETS moordyn CONFIGURATIONS Release
                RUNTIME DESTINATION bin COMPONENT runtime)
    else()
        install(TARGETS moordyn CONFIGURATIONS Release
                LIBRARY DESTINATION lib COMPONENT runtime)
    endif()
endif()

# Chrono runtime data (skybox, colormaps, vsg, logo)
install(DIRECTORY ${SEASTACK_BUILD_DATA}/chrono/ DESTINATION data/chrono COMPONENT runtime
        OPTIONAL)

# Demo case data for run_seastack: ship at package root demos/ (short CLI paths).
# Each case uses ../assets/geometry and ../assets/hydroData relative to its YAML
# directory, so a second copy under data/demos/ is not installed (that duplicated
# large meshes/HDF5 and bloated the runtime ZIP). C++ tests and local builds still
# use flattened paths under ${SEASTACK_BUILD_DATA}/demos/<model>/ from file(COPY)
# above; the packaged app only needs demos/.
# Wigley cases stay in the source tree for developers but are omitted from the
# distribution package (trimaran covers a richer multi-hull Wigley-style example).
install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/demos/run_seastack/
        DESTINATION demos COMPONENT runtime
        REGEX "^wigley"              EXCLUDE
        # Omit redundant cases from the release ZIP (full tree remains in the source repo).
        REGEX "rm3/regular_waves"    EXCLUDE
        REGEX "f3of/decay_dt1"     EXCLUDE
        REGEX "f3of/decay_dt2"     EXCLUDE
        PATTERN "compare_rirfs.py"   EXCLUDE
        PATTERN "*.pyc"              EXCLUDE
        PATTERN "__pycache__"        EXCLUDE
        PATTERN "*.nc"               EXCLUDE
        PATTERN "*.nemoh"            EXCLUDE
        PATTERN "meshes"             EXCLUDE
        PATTERN ".gitignore"         EXCLUDE)

# MSVC runtime DLLs (msvcp, vcruntime, ucrt) and OpenMP runtime
set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT runtime)
if(MSVC AND OpenMP_CXX_FOUND)
    get_filename_component(_msvc_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
    find_file(_vcomp_dll "vcomp140.dll"
        PATHS "${_msvc_dir}" "${_msvc_dir}/../../../redist/x64" "C:/Windows/System32"
        NO_DEFAULT_PATH)
    if(_vcomp_dll)
        list(APPEND CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS "${_vcomp_dll}")
    else()
        message(WARNING "vcomp140.dll not found -- OpenMP runtime will be missing from package")
    endif()
endif()
include(InstallRequiredSystemLibraries)

# Quick start for end users (package root)
if(EXISTS "${CMAKE_SOURCE_DIR}/docs/build/QUICKSTART_RELEASE.txt")
    install(FILES "${CMAKE_SOURCE_DIR}/docs/build/QUICKSTART_RELEASE.txt"
            DESTINATION . COMPONENT runtime
            RENAME QUICKSTART.txt)
endif()

# Project license
install(FILES "${CMAKE_SOURCE_DIR}/LICENSE"
        DESTINATION . COMPONENT runtime)

# Third-party license notices
if(EXISTS "${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.txt")
    install(FILES "${CMAKE_SOURCE_DIR}/THIRD_PARTY_NOTICES.txt"
            DESTINATION . COMPONENT runtime)
endif()

# Test runner (users can verify the installation produces correct results)
set(_test_runner_dir "${CMAKE_SOURCE_DIR}/tests/regression/run_seastack")
install(FILES
    "${_test_runner_dir}/run_tests.py"
    "${_test_runner_dir}/run_simulation.py"
    "${_test_runner_dir}/compare_results.py"
    "${_test_runner_dir}/compare_template.py"
    "${_test_runner_dir}/generate_plots.py"
    "${_test_runner_dir}/requirements.txt"
    "${_test_runner_dir}/README.md"
    DESTINATION tests COMPONENT runtime)
if(WIN32)
    install(FILES "${_test_runner_dir}/RUN-TESTS.ps1"
            DESTINATION tests COMPONENT runtime)
else()
    install(FILES "${_test_runner_dir}/RUN-TESTS.sh"
            DESTINATION tests COMPONENT runtime)
endif()

# Packaged compare_template.py imports plot_helpers from tests/utilities (flat install layout).
install(FILES "${CMAKE_SOURCE_DIR}/tests/utilities/plot_helpers.py"
        DESTINATION tests/utilities COMPONENT runtime)

# RUNTIME_DEPENDENCIES copies dylibs into lib/ but leaves absolute LC_LOAD_DYLIB on the exe; point those at the bundle.
# (Linux would need patchelf/chrpath; RPATH $ORIGIN/../lib on the exe covers many layouts.)
if(TARGET run_seastack AND APPLE)
    install(CODE "
        set(_seastack_pfx \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}\")
        set(_seastack_exe \"\${_seastack_pfx}/bin/run_seastack\")
        if(EXISTS \"\${_seastack_exe}\")
            execute_process(COMMAND /usr/bin/otool -L \"\${_seastack_exe}\"
                OUTPUT_VARIABLE _seastack_otool
                OUTPUT_STRIP_TRAILING_WHITESPACE)
            string(REPLACE \"\\n\" \";\" _seastack_lines \"\${_seastack_otool}\")
            foreach(_seastack_line IN LISTS _seastack_lines)
                if(_seastack_line MATCHES \"^[ \\t]+(/[^ ]+\\.dylib)\")
                    set(_seastack_old \"\${CMAKE_MATCH_1}\")
                    if(_seastack_old MATCHES \"^/usr/lib/\" OR _seastack_old MATCHES \"^/System/\")
                        continue()
                    endif()
                    get_filename_component(_seastack_base \"\${_seastack_old}\" NAME)
                    set(_seastack_new \"@loader_path/../lib/\${_seastack_base}\")
                    set(_seastack_cand \"\${_seastack_pfx}/lib/\${_seastack_base}\")
                    if(EXISTS \"\${_seastack_cand}\")
                        execute_process(COMMAND /usr/bin/install_name_tool
                            -change \"\${_seastack_old}\" \"\${_seastack_new}\" \"\${_seastack_exe}\"
                            RESULT_VARIABLE _seastack_inr)
                    endif()
                endif()
            endforeach()
        endif()
    " COMPONENT runtime)
endif()

set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "SEAStack")
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)
set(CPACK_COMPONENTS_ALL runtime)
if(WIN32)
    set(_SEASTACK_CPACK_SUFFIX "win64")
elseif(APPLE)
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _seastack_proc)
    if(_seastack_proc MATCHES "arm64|aarch64")
        set(_SEASTACK_CPACK_SUFFIX "darwin-arm64")
    else()
        set(_SEASTACK_CPACK_SUFFIX "darwin-x64")
    endif()
else()
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _seastack_proc)
    if(_seastack_proc MATCHES "x86_64|amd64")
        set(_SEASTACK_CPACK_SUFFIX "linux-x64")
    else()
        set(_SEASTACK_CPACK_SUFFIX "linux-${CMAKE_SYSTEM_PROCESSOR}")
    endif()
endif()
set(CPACK_ARCHIVE_RUNTIME_FILE_NAME "${CPACK_PACKAGE_NAME}-${SEASTACK_VERSION_FULL}-${_SEASTACK_CPACK_SUFFIX}")
include(CPack)
