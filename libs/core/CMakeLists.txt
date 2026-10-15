# SEAStack::Core — header-only stable contract layer
# Allowed deps: C++ stdlib, Eigen
# Forbidden: Chrono, HDF5, OpenMP, MoorDyn, VSG

add_library(seastack_core INTERFACE)
add_library(SEAStack::Core ALIAS seastack_core)

target_include_directories(seastack_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# Eigen — always provided by top-level CMake (either from find_package or EIGEN3_INCLUDE_DIR)
if(TARGET Eigen3::Eigen)
    target_link_libraries(seastack_core INTERFACE Eigen3::Eigen)
else()
    message(FATAL_ERROR "Eigen3::Eigen target not found. Top-level CMake should ensure Eigen3 is available.")
endif()

target_compile_features(seastack_core INTERFACE cxx_std_17)

# Eigen ABI contract. Every translation unit that shares Eigen objects with
# SEA-Stack libraries must use the same EIGEN_MAX_ALIGN_BYTES /
# EIGEN_MAX_STATIC_ALIGN_BYTES (and therefore the same Eigen heap protocol).
#
# MSVC: Chrono exports /arch:AVX2 (32-byte alignment; see
# seastack_apply_compile_settings). That makes Eigen use its handmade aligned
# allocator instead of plain malloc/free. A consumer compiled without AVX2
# frees those blocks with the wrong allocator and corrupts the heap. Export
# the settings so Chrono-free find_package(SEAStack) consumers inherit them.
#
# Non-MSVC (Linux/macOS): pin the portable 16/16 contract that matches the
# official Chrono 10 recipe in docs/build/BUILD_CHRONO.md (CH_USE_SIMD=OFF and
# the same Eigen definitions when building Chrono). Do not use -march=native
# here; a Chrono built with native SIMD keeps a 32-byte DEFAULT alignment even
# if MAX_ALIGN is forced to 16, and that reintroduces the allocator split.
if(MSVC)
    target_compile_options(seastack_core INTERFACE $<$<COMPILE_LANGUAGE:CXX>:/arch:AVX2>)
    target_compile_definitions(seastack_core INTERFACE _ENABLE_EXTENDED_ALIGNED_STORAGE)
else()
    target_compile_definitions(seastack_core INTERFACE
        EIGEN_MAX_ALIGN_BYTES=16
        EIGEN_MAX_STATIC_ALIGN_BYTES=16
    )
endif()
