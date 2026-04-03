find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Matches tests/regression/utilities/compare_template.py SKIP_EXIT_CODE; CTest reports Skipped.
set(SEASTACK_SKIP_MISSING_PYTHON_DEPS 77)

# ---------------------------------------------------------------------------
# build_verification_test(TEST_GROUP TEST_NAME)
#
# Registers a verification test (SEA-Stack vs external reference code).
# CTest names: test_verification_<family>_<case_id> and ..._verification
# Paths still use TEST_GROUP (verification dataset directory).
# ---------------------------------------------------------------------------
function(build_verification_test TEST_GROUP TEST_NAME)

   message(STATUS "...${TEST_NAME} (verification)")

   seastack_verification_split_group("${TEST_GROUP}" VF VC)
   seastack_ctest_name(verification "${VF}" "${VC}" RUN TEST_CTEST_RUN)
   seastack_ctest_name(verification "${VF}" "${VC}" VERIFICATION TEST_CTEST_VERIFY)
   seastack_fixture_verification("${VF}" "${VC}" FIXTURE_FILE)

   set(TEST_SRC  ${TEST_GROUP}/test_${TEST_NAME}.cpp)
   set(TEST_EXE  test_${TEST_NAME})
   set(COMPARE_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_GROUP}/compare_${TEST_NAME}.py)

   set(FILE_REF_DIR ${SEASTACK_BUILD_DATA}/verification/${TEST_GROUP}/normalized)
   set(FILE_RST     results/tests/verification/${TEST_GROUP}/results_${TEST_NAME}.txt)

   add_executable(${TEST_EXE} ${TEST_SRC})
   source_group("" FILES ${TEST_SRC})

   target_compile_definitions(${TEST_EXE}
       PRIVATE
       "REFERENCE_DIR=\"${FILE_REF_DIR}\""
       "RESULTS_DIR_NAME=\"verification/${TEST_GROUP}\""
       "RESULTS_FILE_NAME=\"results_${TEST_NAME}\""
   )

   if(MSVC)
       set_target_properties(${TEST_EXE} PROPERTIES
           VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>")
   endif()

   target_link_libraries(${TEST_EXE} PRIVATE SEAStack::ChronoAdapter)

   add_test(
       NAME ${TEST_CTEST_RUN}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>"
       COMMAND $<TARGET_FILE:${TEST_EXE}> --no_gui --no_plot)

   set(TEST_COST 60)
   if("${TEST_NAME}" MATCHES "decay")
       set(TEST_COST 40)
   elseif("${TEST_NAME}" MATCHES "mooring")
       set(TEST_COST 150)
   endif()

   set_tests_properties(
       ${TEST_CTEST_RUN}
       PROPERTIES
       LABELS "verification;${TEST_GROUP};execute"
       FIXTURES_SETUP ${FIXTURE_FILE}
       ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
       COST ${TEST_COST}
   )

   add_test(
       NAME ${TEST_CTEST_VERIFY}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>"
       COMMAND ${Python3_EXECUTABLE} ${COMPARE_SCRIPT}
               ${FILE_REF_DIR}
               $<TARGET_FILE_DIR:${TEST_EXE}>/${FILE_RST}
   )

   if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
       set_tests_properties(
           ${TEST_CTEST_VERIFY}
           PROPERTIES
           LABELS "verification;${TEST_GROUP};reference"
           FIXTURES_REQUIRED ${FIXTURE_FILE}
           ENVIRONMENT_MODIFICATION
               "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
           ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   else()
       set_tests_properties(
           ${TEST_CTEST_VERIFY}
           PROPERTIES
           LABELS "verification;${TEST_GROUP};reference"
           FIXTURES_REQUIRED ${FIXTURE_FILE}
           ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   endif()

endfunction()

# ---------------------------------------------------------------------------
# build_rao_verification_test(VER_GROUP VER_NAME REG_TEST_NAME REG_MODEL_DIR)
# ---------------------------------------------------------------------------
function(build_rao_verification_test VER_GROUP VER_NAME REG_TEST_NAME REG_MODEL_DIR)

   message(STATUS "...${VER_NAME} (verification, RAO compare + regression data)")

   seastack_verification_split_group("${VER_GROUP}" VF VC)
   seastack_ctest_name(verification "${VF}" "${VC}" VERIFICATION TEST_CTEST_VERIFY)

   seastack_regression_case_id("${REG_MODEL_DIR}" "${REG_TEST_NAME}" REG_CASE_ID)
   seastack_fixture_regression("${REG_MODEL_DIR}" "${REG_CASE_ID}" REG_FIXTURE)

   # Verification-owned full-period sweep: same executable as regression *_full tests.
   # FIXTURES_SETUP must live here (not on regression-labeled tests) so
   # `ctest -L verification` does not auto-schedule regression tests as fixture deps.
   set(TEST_CTEST_REG_DATA "test_verification_${VER_NAME}_regression_data")
   if(REG_MODEL_DIR STREQUAL "sphere")
       set(RAO_REG_DATA_COST 500)
   else()
       set(RAO_REG_DATA_COST 800)
   endif()
   add_test(
       NAME ${TEST_CTEST_REG_DATA}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_${REG_TEST_NAME}>"
       COMMAND $<TARGET_FILE:test_${REG_TEST_NAME}> --no_gui --no_plot)
   set_tests_properties(${TEST_CTEST_REG_DATA} PROPERTIES
       LABELS "verification;${VER_GROUP};execute;rao_regression_data"
       FIXTURES_SETUP "${REG_FIXTURE}"
       ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
       COST ${RAO_REG_DATA_COST})

   set(COMPARE_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${VER_GROUP}/compare_${VER_NAME}.py)

   set(FILE_REF_DIR ${SEASTACK_BUILD_DATA}/verification/${VER_GROUP}/normalized)
   set(REG_INPUT_DIR $<TARGET_FILE_DIR:test_${REG_TEST_NAME}>/results/tests/regression/${REG_MODEL_DIR})
   set(VER_OUTPUT_DIR $<TARGET_FILE_DIR:test_${REG_TEST_NAME}>/results/tests/verification/${VER_GROUP})

   add_test(
       NAME ${TEST_CTEST_VERIFY}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_${REG_TEST_NAME}>"
       COMMAND ${Python3_EXECUTABLE} ${COMPARE_SCRIPT}
               ${FILE_REF_DIR}
               ${REG_INPUT_DIR}
               ${VER_OUTPUT_DIR}
   )

   if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
       set_tests_properties(
           ${TEST_CTEST_VERIFY}
           PROPERTIES
           LABELS "verification;${VER_GROUP};reference"
           FIXTURES_REQUIRED ${REG_FIXTURE}
           ENVIRONMENT_MODIFICATION
               "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
           ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   else()
       set_tests_properties(
           ${TEST_CTEST_VERIFY}
           PROPERTIES
           LABELS "verification;${VER_GROUP};reference"
           FIXTURES_REQUIRED ${REG_FIXTURE}
           ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   endif()

endfunction()

# ===========================================================================
# Register verification tests
# ===========================================================================

message(STATUS "Add SEA-Stack verification tests (ordered: sphere decay/rao, oswec decay/rao, rm3 mooring, f3of decay)")

build_verification_test(sphere_decay_multicode  sphere_decay_multicode)
build_rao_verification_test(sphere_rao_sweep  sphere_rao_sweep  sphere_reg_waves_full  sphere)
build_verification_test(oswec_decay_wecsim      oswec_decay_wecsim)
build_rao_verification_test(oswec_rao_sweep   oswec_rao_sweep   oswec_reg_waves_full   oswec)
if(SEASTACK_ENABLE_MOORING)
    build_verification_test(rm3_mooring  rm3_mooring)
else()
    message(STATUS "  rm3_mooring skipped (requires SEASTACK_ENABLE_MOORING)")
endif()
build_verification_test(f3of_decay_multicode    f3of_decay_multicode)

# Reference-step CTest names (must match seastack_ctest_name rules above).
set(ALL_VERIFICATION_DEPENDS
    test_verification_sphere_decay_multicode_verification
    test_verification_sphere_rao_sweep_verification
    test_verification_oswec_decay_wecsim_verification
    test_verification_oswec_rao_sweep_verification
)
if(SEASTACK_ENABLE_MOORING)
    list(APPEND ALL_VERIFICATION_DEPENDS test_verification_rm3_mooring_verification)
endif()
list(APPEND ALL_VERIFICATION_DEPENDS test_verification_f3of_decay_multicode_verification)

# ── Verification Report Generation ─────────────────────────────────────
# Output path uses $<CONFIG> so multi-config generators match the active build type.
# Python creates the directory; do not use file(MAKE_DIRECTORY) with generator expressions.

find_program(PANDOC_EXECUTABLE pandoc)
if(PANDOC_EXECUTABLE)
    set(_VERIFICATION_REPORT_PDF_FLAG "--pdf")
    message(STATUS "  pandoc found -- verification report will include PDF")
else()
    set(_VERIFICATION_REPORT_PDF_FLAG "")
    message(STATUS "  pandoc not found -- verification report will be Markdown only")
endif()

add_test(
    NAME verification_report_generation
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/utilities/generate_verification_report.py
        --build-dir ${CMAKE_BINARY_DIR}
        --output-dir ${CMAKE_BINARY_DIR}/bin/$<CONFIG>/results/tests/verification/report
        --config $<CONFIG>
        ${_VERIFICATION_REPORT_PDF_FLAG}
)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
    set_tests_properties(
        verification_report_generation
        PROPERTIES
        LABELS "verification;report;documentation"
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
        ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
        DEPENDS "${ALL_VERIFICATION_DEPENDS}"
    )
else()
    set_tests_properties(
        verification_report_generation
        PROPERTIES
        LABELS "verification;report;documentation"
        ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
        DEPENDS "${ALL_VERIFICATION_DEPENDS}"
    )
endif()
