# SEAStack::Infra — Logging, YAML discovery, and general-purpose utilities
# Allowed deps: C++ stdlib only
# Forbidden: Chrono, HDF5, Eigen, OpenMP, VSG

add_library(seastack_infra STATIC
    src/logging.cpp
    src/logger_backend.cpp
    src/yaml_discovery.cpp
)
add_library(SEAStack::Infra ALIAS seastack_infra)

target_include_directories(seastack_infra
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_compile_features(seastack_infra PUBLIC cxx_std_17)

# Provide version and build-type macros so the logger can report them.
target_compile_definitions(seastack_infra PRIVATE
    SEASTACK_BUILD_TYPE="$<CONFIG>"
)
target_include_directories(seastack_infra PRIVATE "${CMAKE_BINARY_DIR}")
