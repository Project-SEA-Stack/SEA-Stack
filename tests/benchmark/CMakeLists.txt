find_package(Python3 COMPONENTS Interpreter REQUIRED)

# ═══════════════════════════════════════════════════════════════════════════════
# Options
# ═══════════════════════════════════════════════════════════════════════════════

option(SEASTACK_BENCHMARK_SANITIZER
    "Enable sanitizers for benchmark builds (AddressSanitizer on GCC/Clang,
     /fsanitize=address on MSVC). Useful for debugging teardown crashes
     and memory issues in parallel execution." OFF)

# ═══════════════════════════════════════════════════════════════════════════════
# Git info: capture commit hash at configure time
# ═══════════════════════════════════════════════════════════════════════════════
execute_process(
    COMMAND git rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE SEASTACK_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE GIT_RESULT
)
if(NOT GIT_RESULT EQUAL 0)
    set(SEASTACK_GIT_COMMIT "unknown")
endif()

execute_process(
    COMMAND git diff --quiet HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE GIT_DIRTY_RESULT
    ERROR_QUIET
)
if(GIT_DIRTY_RESULT EQUAL 0)
    set(SEASTACK_GIT_DIRTY "false")
else()
    set(SEASTACK_GIT_DIRTY "true")
endif()

# ═══════════════════════════════════════════════════════════════════════════════
# build_benchmark(TEST_GROUP TEST_NAME [COST <cost>])
#
# Registers a performance benchmark test with:
#   - Full metadata compile definitions (git, compiler, build type, version)
#   - RESOURCE_LOCK to enforce sequential CTest-level execution
#   - FIXTURES_SETUP so the report step runs after all benchmarks
#   - COST hint for CTest scheduling (higher cost → scheduled earlier)
#   - No thread suppression: each benchmark uses full internal parallelism
# ═══════════════════════════════════════════════════════════════════════════════
function(build_benchmark TEST_GROUP TEST_NAME)

   cmake_parse_arguments(PARSE_ARGV 2 ARG "" "COST" "")
   if(NOT ARG_COST)
       set(ARG_COST 100)
   endif()

   message(STATUS "...${TEST_NAME} (benchmark)")

   seastack_benchmark_case_id("${TEST_GROUP}" "${TEST_NAME}" BENCH_CASE_ID)
   seastack_ctest_name(benchmark "${TEST_GROUP}" "${BENCH_CASE_ID}" RUN TEST_CTEST_NAME)

   set(TEST_SRC  ${TEST_GROUP}/test_${TEST_NAME}.cpp)
   set(TEST_EXE  test_${TEST_NAME})

   add_executable(${TEST_EXE} ${TEST_SRC})
   source_group("" FILES ${TEST_SRC})

   target_include_directories(${TEST_EXE} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

   target_compile_definitions(${TEST_EXE}
       PRIVATE
       "RESULTS_DIR_NAME=\"benchmark/${TEST_GROUP}\""
       "RESULTS_FILE_NAME=\"results_${TEST_NAME}\""
       "SEASTACK_BENCH_BUILD_TYPE=\"$<CONFIG>\""
       "SEASTACK_BENCH_COMPILER_ID=\"${CMAKE_CXX_COMPILER_ID}\""
       "SEASTACK_BENCH_COMPILER_VERSION=\"${CMAKE_CXX_COMPILER_VERSION}\""
       "SEASTACK_BENCH_GIT_COMMIT=\"${SEASTACK_GIT_COMMIT}\""
       "SEASTACK_BENCH_GIT_DIRTY=${SEASTACK_GIT_DIRTY}"
       "SEASTACK_BENCH_SEASTACK_VERSION=\"${SEASTACK_VERSION_FULL}\""
       "SEASTACK_BENCH_SYSTEM_NAME=\"${CMAKE_SYSTEM_NAME}\""
       "SEASTACK_BENCH_SYSTEM_VERSION=\"${CMAKE_SYSTEM_VERSION}\""
   )

   if(MSVC)
       set_target_properties(${TEST_EXE} PROPERTIES
           VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>")
   endif()

   target_link_libraries(${TEST_EXE} PRIVATE SEAStack::ChronoAdapter)
   if(TARGET OpenMP::OpenMP_CXX)
       target_link_libraries(${TEST_EXE} PRIVATE OpenMP::OpenMP_CXX)
   endif()
   target_compile_features(${TEST_EXE} PRIVATE ${SEASTACK_CXX_STANDARD})

   # Sanitizer support for debugging teardown crashes
   if(SEASTACK_BENCHMARK_SANITIZER)
       if(MSVC)
           target_compile_options(${TEST_EXE} PRIVATE /fsanitize=address)
       else()
           target_compile_options(${TEST_EXE} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
           target_link_options(${TEST_EXE} PRIVATE -fsanitize=address)
       endif()
   endif()

   add_test(
       NAME ${TEST_CTEST_NAME}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>"
       COMMAND $<TARGET_FILE:${TEST_EXE}> --no_gui --no_plot)

   # Set worker environment: only data directory, no thread suppression.
   # Each benchmark uses full internal parallelism (OpenMP, MKL, etc.)
   # since benchmarks run sequentially at the CTest level.
   set(WORKER_ENV "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}")

   set_tests_properties(
       ${TEST_CTEST_NAME}
       PROPERTIES
       LABELS "benchmark;${TEST_GROUP}"
       ENVIRONMENT "${WORKER_ENV}"
       RESOURCE_LOCK "benchmark_exclusive"
       FIXTURES_SETUP "benchmark_results"
       COST ${ARG_COST}
   )

endfunction()

# ===========================================================================
# Register benchmark tests
# ===========================================================================

message(STATUS "Add SEA-Stack benchmark tests")

# Tier 1: Fast baselines
build_benchmark(sphere  bench_sphere_decay_conv  COST 120)
build_benchmark(sphere  bench_sphere_decay_ss    COST 100)
build_benchmark(sphere  bench_sphere_irreg_conv  COST 350)

# Tier 2: Method comparison pair
build_benchmark(sphere  bench_sphere_irreg_ss    COST 250)

# Tier 4: Batch throughput
build_benchmark(oswec   bench_oswec_reg_batch    COST 600)

# MoorDyn-dependent: RM3
if(SEASTACK_ENABLE_MOORING)
    build_benchmark(rm3     bench_rm3_mooring_irreg  COST 350)
endif()

# Tier 3: Multi-body / coupling (no MoorDyn)
build_benchmark(f3of    bench_f3of_decay_c3           COST 200)

# MoorDyn-dependent: 5SA spreading
if(SEASTACK_ENABLE_MOORING)
    build_benchmark(5sa     bench_5sa_spreading      COST 250)
    target_include_directories(test_bench_5sa_spreading PRIVATE ${CMAKE_SOURCE_DIR}/demos/5sa)
endif()

# ═══════════════════════════════════════════════════════════════════════════════
# Benchmark Report Generation
#
# Runs after all benchmark executables via FIXTURES_REQUIRED.
# Auto-compares against the latest promoted baseline in data/benchmarks/history/.
# ═══════════════════════════════════════════════════════════════════════════════

set(BENCHMARK_DOCS_DIR ${CMAKE_BINARY_DIR}/bin/Release/results/tests/benchmark/report)
file(MAKE_DIRECTORY ${BENCHMARK_DOCS_DIR})

add_test(
    NAME benchmark_report_generation
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/utilities/generate_benchmark_report.py
        --build-dir ${CMAKE_BINARY_DIR}
        --output-dir ${BENCHMARK_DOCS_DIR}
        --history-dir ${CMAKE_SOURCE_DIR}/data/benchmarks/history
)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
    set_tests_properties(
        benchmark_report_generation
        PROPERTIES
        LABELS "benchmark;report;documentation"
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
        FIXTURES_REQUIRED "benchmark_results"
        RESOURCE_LOCK "benchmark_exclusive"
    )
else()
    set_tests_properties(
        benchmark_report_generation
        PROPERTIES
        LABELS "benchmark;report;documentation"
        ENVIRONMENT "PATH=${Python3_ROOT_DIR}"
        FIXTURES_REQUIRED "benchmark_results"
        RESOURCE_LOCK "benchmark_exclusive"
    )
endif()
