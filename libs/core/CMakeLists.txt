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

# Eigen ABI contract (MSVC). Every translation unit that shares Eigen objects
# with SEA-Stack libraries must use the same EIGEN_MAX_ALIGN_BYTES. SEA-Stack
# is compiled with /arch:AVX2 (32-byte alignment, matching Chrono's exported
# flags; see seastack_apply_compile_settings), which makes Eigen use its own
# aligned allocator instead of plain malloc/free. A consumer compiled without
# it frees those blocks with the wrong allocator and corrupts the heap. Export
# the settings so Chrono-free find_package(SEAStack) consumers inherit them.
if(MSVC)
    target_compile_options(seastack_core INTERFACE $<$<COMPILE_LANGUAGE:CXX>:/arch:AVX2>)
    target_compile_definitions(seastack_core INTERFACE ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()
