# Coupled seakeeping + SPH sloshing examples.
# Only configured when SEASTACK_ENABLE_SPH is ON (guarded by the parent CMake).

# ── Coupled POC: wigley hull (potential-flow BEM) + SPH deck-sloshing tank ──
add_executable(wigley_tank_sloshing wigley_tank_sloshing.cpp)
# ChronoAdapter PUBLIC-links Hydro, HydroIO, Infra, Core and Chrono; we add the
# YAML MBS parser and the Chrono FSI/SPH libraries on top.
target_link_libraries(wigley_tank_sloshing PRIVATE
    SEAStack::ChronoAdapter
    Chrono::Chrono_parsers
    Chrono::Chrono_fsi
    Chrono::Chrono_fsisph)
if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(wigley_tank_sloshing PRIVATE yaml-cpp::yaml-cpp)
elseif(TARGET Chrono::yaml-cpp)
    target_link_libraries(wigley_tank_sloshing PRIVATE Chrono::yaml-cpp)
endif()
target_compile_features(wigley_tank_sloshing PRIVATE cxx_std_17)
