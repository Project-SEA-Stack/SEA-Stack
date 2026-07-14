# SEAStack::External — out-of-process force modules (IPC + bridges)
# Allowed deps: C++ stdlib, SEAStack::PTO (ExternalPtoModel), SEAStack::Core (ExternalForceComponent)
# Forbidden: Chrono, HDF5, OpenMP

add_library(seastack_external STATIC
    src/protocol.cpp
    src/ipc_external_force_model.cpp
    src/external_pto_model.cpp
    src/external_force_component.cpp
)
add_library(SEAStack::External ALIAS seastack_external)

target_include_directories(seastack_external
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(seastack_external
    PUBLIC
        SEAStack::PTO
        SEAStack::Core
)

if(WIN32)
    target_link_libraries(seastack_external PRIVATE ws2_32)
endif()

target_compile_features(seastack_external PUBLIC cxx_std_17)
