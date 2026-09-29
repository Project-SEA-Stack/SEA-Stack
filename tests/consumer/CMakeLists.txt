# Standalone downstream project: configured against an installed SEA-Stack SDK,
# never added to the main build. Driven by scripts/windows/run_consumer_check.ps1
# and scripts/unix/run_consumer_check.sh.
cmake_minimum_required(VERSION 3.21)
project(seastack_consumer_check LANGUAGES CXX)

find_package(SEAStack REQUIRED)

add_executable(consumer_check main.cpp)
target_link_libraries(consumer_check PRIVATE SEAStack::HydroIO)
target_compile_features(consumer_check PRIVATE cxx_std_17)
