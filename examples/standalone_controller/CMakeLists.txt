# Standalone controller example — proves PTO + Control compile without Chrono
add_executable(standalone_controller main.cpp)
target_link_libraries(standalone_controller PRIVATE SEAStack::PTO SEAStack::Control)
target_compile_features(standalone_controller PRIVATE cxx_std_17)
