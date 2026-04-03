find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Matches tests/regression/utilities/compare_template.py SKIP_EXIT_CODE; CTest reports Skipped.
set(SEASTACK_SKIP_MISSING_PYTHON_DEPS 77)

# ---------------------------------------------------------------------------
# build_comparison_test(TEST_GROUP TEST_NAME)
# CTest: test_comparison_<family>_<case_id> and ..._comparison
# ---------------------------------------------------------------------------
function(build_comparison_test TEST_GROUP TEST_NAME)

   message(STATUS "...${TEST_NAME} (comparison)")

   seastack_regression_case_id("${TEST_GROUP}" "${TEST_NAME}" CASE_ID)
   seastack_ctest_name(comparison "${TEST_GROUP}" "${CASE_ID}" RUN TEST_CTEST_RUN)
   seastack_ctest_name(comparison "${TEST_GROUP}" "${CASE_ID}" COMPARISON TEST_CTEST_CMP)
   seastack_fixture_comparison("${TEST_GROUP}" "${CASE_ID}" FIXTURE_FILE)

   set(TEST_SRC  ${TEST_GROUP}/test_${TEST_NAME}.cpp)
   set(TEST_EXE  test_${TEST_NAME})
   set(COMPARE_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_GROUP}/compare_${TEST_NAME}.py)

   set(FILE_RST results/tests/comparison/${TEST_GROUP}/${TEST_NAME})

   add_executable(${TEST_EXE} ${TEST_SRC})
   source_group("" FILES ${TEST_SRC})

   target_compile_definitions(${TEST_EXE}
       PRIVATE
       "RESULTS_DIR_NAME=\"comparison/${TEST_GROUP}/${TEST_NAME}\""
       "RESULTS_FILE_NAME=\"\""
   )

   if(MSVC)
       set_target_properties(${TEST_EXE} PROPERTIES
           VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>")
   endif()

   target_link_libraries(${TEST_EXE} PRIVATE SEAStack::ChronoAdapter)
   target_compile_features(${TEST_EXE} PRIVATE ${SEASTACK_CXX_STANDARD})

   add_test(
       NAME ${TEST_CTEST_RUN}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>"
       COMMAND $<TARGET_FILE:${TEST_EXE}> --no_gui --no_plot)

   set(TEST_COST 90)
   if("${TEST_NAME}" MATCHES "decay")
       set(TEST_COST 60)
   elseif("${TEST_NAME}" MATCHES "irreg")
       set(TEST_COST 150)
   endif()

   set_tests_properties(
       ${TEST_CTEST_RUN}
       PROPERTIES
       LABELS "comparison;${TEST_GROUP};execute"
       FIXTURES_SETUP ${FIXTURE_FILE}
       ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
       COST ${TEST_COST}
   )

   add_test(
       NAME ${TEST_CTEST_CMP}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>"
       COMMAND ${Python3_EXECUTABLE} ${COMPARE_SCRIPT}
               $<TARGET_FILE_DIR:${TEST_EXE}>/${FILE_RST}
   )

   if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
       set_tests_properties(
           ${TEST_CTEST_CMP}
           PROPERTIES
           LABELS "comparison;${TEST_GROUP};analysis"
           FIXTURES_REQUIRED ${FIXTURE_FILE}
           ENVIRONMENT_MODIFICATION
               "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
           ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   else()
       set_tests_properties(
           ${TEST_CTEST_CMP}
           PROPERTIES
           LABELS "comparison;${TEST_GROUP};analysis"
           FIXTURES_REQUIRED ${FIXTURE_FILE}
           ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   endif()

endfunction()

# ===========================================================================
# Register comparison tests
# ===========================================================================

message(STATUS "Add SEA-Stack comparison tests")

build_comparison_test(sphere  compare_excitation_irf_vs_fd_sphere_irreg)
build_comparison_test(sphere  compare_eta_dft_vs_convolution_sphere_irreg)
build_comparison_test(sphere  compare_linear_vs_nonlinear_hs_sphere_decay)

set(_rm3_topo_src rm3/test_validate_rm3_mesh_topology.cpp)
set(_rm3_topo_exe test_validate_rm3_mesh_topology)
seastack_ctest_name(comparison rm3 validate_rm3_mesh_topology RUN CTEST_RM3_TOPO)
add_executable(${_rm3_topo_exe} ${_rm3_topo_src})
target_link_libraries(${_rm3_topo_exe} PRIVATE SEAStack::ChronoAdapter)
target_compile_features(${_rm3_topo_exe} PRIVATE ${SEASTACK_CXX_STANDARD})
if(MSVC)
    set_target_properties(${_rm3_topo_exe} PROPERTIES
        VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_rm3_topo_exe}>")
endif()
add_test(
    NAME ${CTEST_RM3_TOPO}
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:${_rm3_topo_exe}>"
    COMMAND $<TARGET_FILE:${_rm3_topo_exe}>)
set_tests_properties(${CTEST_RM3_TOPO} PROPERTIES
    LABELS "comparison;rm3;execute"
    ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
    COST 10)

build_comparison_test(rm3  compare_linear_vs_hydraulic_pto_rm3_irreg)

# Report output uses $<CONFIG> for multi-config generators (Python creates the directory).

find_program(PANDOC_EXECUTABLE pandoc)
if(PANDOC_EXECUTABLE)
    set(_COMPARISON_REPORT_PDF_FLAG "--pdf")
    message(STATUS "  pandoc found -- comparison report will include PDF")
else()
    set(_COMPARISON_REPORT_PDF_FLAG "")
    message(STATUS "  pandoc not found -- comparison report will be Markdown only")
endif()

add_test(
    NAME comparison_report_generation
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/utilities/generate_comparison_report.py
        --build-dir ${CMAKE_BINARY_DIR}
        --output-dir ${CMAKE_BINARY_DIR}/bin/$<CONFIG>/results/tests/comparison/report
        --config $<CONFIG>
        ${_COMPARISON_REPORT_PDF_FLAG}
)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
    set_tests_properties(
        comparison_report_generation
        PROPERTIES
        LABELS "comparison;report;documentation"
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
        ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
    )
else()
    set_tests_properties(
        comparison_report_generation
        PROPERTIES
        LABELS "comparison;report;documentation"
        ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
    )
endif()
