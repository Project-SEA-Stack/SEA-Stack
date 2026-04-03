include(${CMAKE_SOURCE_DIR}/cmake/SeaStackTestNaming.cmake)

add_subdirectory(unit)

if(SEASTACK_ENABLE_CHRONO AND TARGET seastack_chrono_adapter AND SEASTACK_ENABLE_APPS)
    add_subdirectory(regression)
    add_subdirectory(verification)
    add_subdirectory(comparison)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/benchmark/CMakeLists.txt)
        add_subdirectory(benchmark)
    endif()
endif()
