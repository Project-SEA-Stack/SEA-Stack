# Standalone hydro example — proves Hydro + HydroIO compile without Chrono
add_executable(standalone_hydro main.cpp)
# HydroIO -> Hydro (PUBLIC) -> Core; Hydro/HydroIO pull Infra via PRIVATE static deps.
target_link_libraries(standalone_hydro PRIVATE SEAStack::HydroIO)
target_compile_features(standalone_hydro PRIVATE cxx_std_17)
