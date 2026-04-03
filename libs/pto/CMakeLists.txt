# SEAStack::PTO — solver-agnostic PTO models
# Allowed deps: C++ stdlib, SEAStack::Control (header-only), SEAStack::Infra (logging)
# Forbidden: Chrono, HDF5, Hydro, VSG, OpenMP, Core/Eigen

add_library(seastack_pto STATIC
    src/linear_pto.cpp
    src/hydraulic/hydraulic_cylinder.cpp
    src/hydraulic/hydraulic_accumulator.cpp
    src/hydraulic/hydraulic_motor.cpp
    src/hydraulic/rectified_hydraulic_pto.cpp
)
add_library(SEAStack::PTO ALIAS seastack_pto)

target_include_directories(seastack_pto
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

# RectifiedHydraulicPTO uses LOG_WARNING via <seastack/infra/logging.h>
target_link_libraries(seastack_pto
    PUBLIC SEAStack::Control
    PRIVATE SEAStack::Infra
)

target_compile_features(seastack_pto PUBLIC cxx_std_17)
