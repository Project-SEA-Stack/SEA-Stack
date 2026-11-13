# SEAStack::VizCore — visualization support that does not depend on a renderer
# Allowed deps: C++ stdlib only
# Forbidden: Chrono, HDF5, VSG, Eigen, OpenMP
#
# Header-only for now: simulation pacing and run-time bookkeeping used by the
# interactive GUI loop. Kept free of Chrono/VSG so it is unit-testable and can
# later be shared by a SEA-Stack-owned viewer and recorded-result playback.

add_library(seastack_viz_core INTERFACE)
add_library(SEAStack::VizCore ALIAS seastack_viz_core)

target_include_directories(seastack_viz_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_compile_features(seastack_viz_core INTERFACE cxx_std_17)
