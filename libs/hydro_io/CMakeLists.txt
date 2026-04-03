# SEAStack::HydroIO — HDF5 reader/writer for hydrodynamic data
# Allowed deps: Hydro, HDF5, Infra (logging, PRIVATE)
# Forbidden: Chrono, VSG

add_library(seastack_hydro_io STATIC
    src/h5_reader.cpp
    src/h5_writer.cpp
)
add_library(SEAStack::HydroIO ALIAS seastack_hydro_io)

target_include_directories(seastack_hydro_io
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(seastack_hydro_io
    PUBLIC SEAStack::Hydro
    PRIVATE SEAStack::Infra
)

# HDF5 discovery: try config mode first, then module mode
find_package(HDF5 CONFIG QUIET)
if(NOT HDF5_FOUND)
    find_package(HDF5 REQUIRED COMPONENTS CXX)
endif()

# Match legacy HDF5 link pattern: link core + HL (and optional tools) for ABI/CRT consistency
if(TARGET hdf5::hdf5-static)
    target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5-static)
    if(TARGET hdf5::hdf5_cpp-static)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_cpp-static)
    endif()
    if(TARGET hdf5::hdf5_hl-static)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_hl-static)
    endif()
    if(TARGET hdf5::hdf5_hl_cpp-static)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_hl_cpp-static)
    endif()
    if(TARGET hdf5::hdf5_tools-static)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_tools-static)
    endif()
elseif(TARGET hdf5::hdf5-shared)
    target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5-shared)
    if(TARGET hdf5::hdf5_cpp-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_cpp-shared)
    endif()
    if(TARGET hdf5::hdf5_hl-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_hl-shared)
    endif()
    if(TARGET hdf5::hdf5_hl_cpp-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_hl_cpp-shared)
    endif()
    if(TARGET hdf5::hdf5_tools-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5::hdf5_tools-shared)
    endif()
elseif(TARGET hdf5-shared)
    # Upstream HDF5Config.cmake (e.g. Homebrew) exports unprefixed hdf5-shared / hdf5_cpp-shared.
    target_link_libraries(seastack_hydro_io PUBLIC hdf5-shared)
    if(TARGET hdf5_cpp-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5_cpp-shared)
    endif()
    if(TARGET hdf5_hl-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5_hl-shared)
    endif()
    if(TARGET hdf5_hl_cpp-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5_hl_cpp-shared)
    endif()
    if(TARGET hdf5_tools-shared)
        target_link_libraries(seastack_hydro_io PUBLIC hdf5_tools-shared)
    endif()
elseif(TARGET HDF5::HDF5)
    target_link_libraries(seastack_hydro_io PUBLIC HDF5::HDF5)
else()
    target_include_directories(seastack_hydro_io PUBLIC ${HDF5_INCLUDE_DIRS})
    target_link_libraries(seastack_hydro_io PUBLIC ${HDF5_CXX_LIBRARIES} ${HDF5_LIBRARIES})
endif()

target_compile_features(seastack_hydro_io PUBLIC cxx_std_17)
