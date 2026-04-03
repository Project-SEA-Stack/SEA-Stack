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
