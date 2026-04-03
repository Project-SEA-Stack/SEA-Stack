# SEAStack::Control — solver-agnostic control library
# Allowed deps: C++ stdlib only (scalar interfaces, no Eigen)
# Forbidden: Chrono, HDF5, Hydro, VSG, OpenMP, Core/Eigen

# Header-only for now (only IController interface, no concrete implementations yet)
add_library(seastack_control INTERFACE)
add_library(SEAStack::Control ALIAS seastack_control)

target_include_directories(seastack_control INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
