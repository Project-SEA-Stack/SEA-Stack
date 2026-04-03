find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Matches tests/regression/utilities/compare_template.py SKIP_EXIT_CODE; CTest reports Skipped.
set(SEASTACK_SKIP_MISSING_PYTHON_DEPS 77)

function(build_regression_test TEST_GROUP TEST_NAME MULTIPLE)
   if(ARGC GREATER 3)
       set(TEST_CATEGORY ${ARGV3})
   else()
       set(TEST_CATEGORY "regression")
   endif()

   message(STATUS "...${TEST_NAME} [${TEST_CATEGORY}]")

   seastack_regression_case_id("${TEST_GROUP}" "${TEST_NAME}" CASE_ID)
   seastack_ctest_name(regression "${TEST_GROUP}" "${CASE_ID}" RUN TEST_CTEST_NAME)
   seastack_ctest_name(regression "${TEST_GROUP}" "${CASE_ID}" REFERENCE TEST_REF_CTEST_NAME)
   seastack_fixture_regression("${TEST_GROUP}" "${CASE_ID}" FIXTURE_FILE)

   set(TEST_SRC ${TEST_GROUP}/test_${TEST_NAME}.cpp)
   set(TEST_EXE test_${TEST_NAME})
   set(COMPARE_SCRIPT ${CMAKE_SOURCE_DIR}/tests/regression/${TEST_GROUP}/compare_${TEST_NAME}.py)
   if(MULTIPLE)
     set(FILE_REF ${SEASTACK_BUILD_DATA}/reference_data/${TEST_GROUP}/ss_ref_${TEST_NAME}_1.txt)
     set(FILE_RST results/tests/regression/${TEST_GROUP}/results_${TEST_NAME}_1.txt)
   else()
     set(FILE_REF ${SEASTACK_BUILD_DATA}/reference_data/${TEST_GROUP}/ss_ref_${TEST_NAME}.txt)
     set(FILE_RST results/tests/regression/${TEST_GROUP}/results_${TEST_NAME}.txt)
   endif()

   add_executable(${TEST_EXE} ${TEST_SRC})
   source_group("" FILES ${TEST_SRC})

   target_compile_definitions(${TEST_EXE}
	   PRIVATE
       "REFERENCE_FILE_NAME=\"${FILE_REF}\""
       "RESULTS_DIR_NAME=\"regression/${TEST_GROUP}\""
       "RESULTS_FILE_NAME=\"results_${TEST_NAME}\""
   )

   if(MSVC)
       set_target_properties(${TEST_EXE} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>")
   endif()

   target_link_libraries(${TEST_EXE} PRIVATE SEAStack::ChronoAdapter)
   target_compile_features(${TEST_EXE} PRIVATE ${SEASTACK_CXX_STANDARD})

   add_test(
       NAME ${TEST_CTEST_NAME}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>"
       COMMAND $<TARGET_FILE:${TEST_EXE}> --no_gui --no_plot)

   set(TEST_COST 30)
   if("${TEST_NAME}" MATCHES "reg_waves")
       set(TEST_COST 50)
   elseif("${TEST_NAME}" MATCHES "irreg_waves")
       set(TEST_COST 120)
   elseif("${TEST_NAME}" MATCHES "decay")
       set(TEST_COST 30)
   endif()

   set_tests_properties(
       ${TEST_CTEST_NAME}
       PROPERTIES
       LABELS "regression;${TEST_GROUP};execute;${TEST_CATEGORY}"
       FIXTURES_SETUP ${FIXTURE_FILE}
       ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
       COST ${TEST_COST}
   )

   add_test(
       NAME ${TEST_REF_CTEST_NAME}
       WORKING_DIRECTORY "$<TARGET_FILE_DIR:${TEST_EXE}>"
       COMMAND ${Python3_EXECUTABLE} ${COMPARE_SCRIPT} ${FILE_REF} $<TARGET_FILE_DIR:${TEST_EXE}>/${FILE_RST}
   )

   if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
       set_tests_properties(
           ${TEST_REF_CTEST_NAME}
           PROPERTIES
           LABELS "regression;${TEST_GROUP};reference"
           FIXTURES_REQUIRED ${FIXTURE_FILE}
           ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
           ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   else()
       set_tests_properties(
           ${TEST_REF_CTEST_NAME}
           PROPERTIES
           LABELS "regression;${TEST_GROUP};reference"
           FIXTURES_REQUIRED ${FIXTURE_FILE}
           ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
           SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
           COST 5
       )
   endif()

endfunction()

option(SEASTACK_CORE_REGRESSION_SUBSET
    "Run only a representative subset of regular-wave conditions in core regression. \
Turn OFF to run the full sweep (e.g. for verification or local analysis)." ON)

message(STATUS "Add SEA-Stack regression tests")

build_regression_test(sphere  sphere_decay            FALSE)
build_regression_test(sphere  sphere_decay_ss         FALSE)
build_regression_test(sphere  sphere_reg_waves        TRUE)
build_regression_test(sphere  sphere_irreg_waves      FALSE)
build_regression_test(sphere  sphere_irreg_waves_ss   FALSE)
build_regression_test(sphere  sphere_irreg_waves_eta  FALSE)
build_regression_test(sphere  sphere_irreg_waves_eta_consistency  FALSE  consistency)
build_regression_test(oswec   oswec_decay             FALSE)
build_regression_test(oswec   oswec_decay_ss          FALSE)
build_regression_test(oswec   oswec_irreg_waves       FALSE)
build_regression_test(oswec   oswec_irreg_waves_ss    FALSE)
build_regression_test(oswec   oswec_reg_waves         TRUE)
build_regression_test(rm3     rm3_decay               FALSE)
build_regression_test(rm3     rm3_reg_waves           FALSE)
build_regression_test(f3of    f3of_decay_c1                FALSE)
build_regression_test(f3of    f3of_decay_c2                FALSE)
build_regression_test(f3of    f3of_decay_c3                FALSE)

if(SEASTACK_CORE_REGRESSION_SUBSET)
    target_compile_definitions(test_sphere_reg_waves PRIVATE "REGRESSION_CONDITION_SUBSET=3,7")
    target_compile_definitions(test_oswec_reg_waves  PRIVATE "REGRESSION_CONDITION_SUBSET=3,12")
    message(STATUS "  Core regression subset ON: sphere_reg_waves={3,7}, oswec_reg_waves={3,12}")

    foreach(CONDITION_NUM 3 7)
        set(CASE_ID "reg_waves_c${CONDITION_NUM}")
        seastack_ctest_name(regression sphere "${CASE_ID}" RUN CONDITION_CTEST_NAME)
        seastack_ctest_name(regression sphere "${CASE_ID}" REFERENCE CONDITION_REF_CTEST_NAME)
        seastack_fixture_regression_condition(sphere reg_waves ${CONDITION_NUM} FIXTURE_FILE)
        set(FILE_REF ${SEASTACK_BUILD_DATA}/reference_data/sphere/ss_ref_sphere_reg_waves_${CONDITION_NUM}.txt)
        set(FILE_RST results/tests/regression/sphere/results_sphere_reg_waves_${CONDITION_NUM}.txt)

        add_test(
            NAME ${CONDITION_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_sphere_reg_waves>"
            COMMAND $<TARGET_FILE:test_sphere_reg_waves> --condition ${CONDITION_NUM} --no_gui --no_plot)

        set_tests_properties(
            ${CONDITION_CTEST_NAME}
            PROPERTIES
            LABELS "regression;sphere;execute;sphere_reg_waves_per_condition;core"
            FIXTURES_SETUP ${FIXTURE_FILE}
            ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
        )

        add_test(
            NAME ${CONDITION_REF_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_sphere_reg_waves>"
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/regression/sphere/compare_sphere_reg_waves.py ${FILE_REF} $<TARGET_FILE_DIR:test_sphere_reg_waves>/${FILE_RST} --condition ${CONDITION_NUM}
        )

        if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;sphere;reference;sphere_reg_waves_per_condition;core"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
                ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        else()
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;sphere;reference;sphere_reg_waves_per_condition;core"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        endif()
    endforeach()

    foreach(CONDITION_NUM 3 12)
        set(CASE_ID "reg_waves_c${CONDITION_NUM}")
        seastack_ctest_name(regression oswec "${CASE_ID}" RUN CONDITION_CTEST_NAME)
        seastack_ctest_name(regression oswec "${CASE_ID}" REFERENCE CONDITION_REF_CTEST_NAME)
        seastack_fixture_regression_condition(oswec reg_waves ${CONDITION_NUM} FIXTURE_FILE)
        set(FILE_REF ${SEASTACK_BUILD_DATA}/reference_data/oswec/ss_ref_oswec_reg_waves_${CONDITION_NUM}.txt)
        set(FILE_RST results/tests/regression/oswec/results_oswec_reg_waves_${CONDITION_NUM}.txt)

        add_test(
            NAME ${CONDITION_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_oswec_reg_waves>"
            COMMAND $<TARGET_FILE:test_oswec_reg_waves> --condition ${CONDITION_NUM} --no_gui --no_plot)

        set_tests_properties(
            ${CONDITION_CTEST_NAME}
            PROPERTIES
            LABELS "regression;oswec;execute;oswec_reg_waves_per_condition;core"
            FIXTURES_SETUP ${FIXTURE_FILE}
            ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
        )

        add_test(
            NAME ${CONDITION_REF_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_oswec_reg_waves>"
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/regression/oswec/compare_oswec_reg_waves.py ${FILE_REF} $<TARGET_FILE_DIR:test_oswec_reg_waves>/${FILE_RST} --condition ${CONDITION_NUM}
        )

        if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;oswec;reference;oswec_reg_waves_per_condition;core"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
                ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        else()
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;oswec;reference;oswec_reg_waves_per_condition;core"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        endif()
    endforeach()
else()
    foreach(CONDITION_NUM RANGE 1 10)
        set(CASE_ID "reg_waves_c${CONDITION_NUM}")
        seastack_ctest_name(regression sphere "${CASE_ID}" RUN CONDITION_CTEST_NAME)
        seastack_ctest_name(regression sphere "${CASE_ID}" REFERENCE CONDITION_REF_CTEST_NAME)
        seastack_fixture_regression_condition(sphere reg_waves ${CONDITION_NUM} FIXTURE_FILE)
        set(FILE_REF ${SEASTACK_BUILD_DATA}/reference_data/sphere/ss_ref_sphere_reg_waves_${CONDITION_NUM}.txt)
        set(FILE_RST results/tests/regression/sphere/results_sphere_reg_waves_${CONDITION_NUM}.txt)

        add_test(
            NAME ${CONDITION_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_sphere_reg_waves>"
            COMMAND $<TARGET_FILE:test_sphere_reg_waves> --condition ${CONDITION_NUM} --no_gui --no_plot)

        set_tests_properties(
            ${CONDITION_CTEST_NAME}
            PROPERTIES
            LABELS "regression;sphere;execute;sphere_reg_waves_per_condition"
            FIXTURES_SETUP ${FIXTURE_FILE}
            ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
        )

        add_test(
            NAME ${CONDITION_REF_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_sphere_reg_waves>"
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/regression/sphere/compare_sphere_reg_waves.py ${FILE_REF} $<TARGET_FILE_DIR:test_sphere_reg_waves>/${FILE_RST} --condition ${CONDITION_NUM}
        )

        if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;sphere;reference;sphere_reg_waves_per_condition"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
                ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        else()
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;sphere;reference;sphere_reg_waves_per_condition"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        endif()
    endforeach()

    foreach(CONDITION_NUM RANGE 1 16)
        set(CASE_ID "reg_waves_c${CONDITION_NUM}")
        seastack_ctest_name(regression oswec "${CASE_ID}" RUN CONDITION_CTEST_NAME)
        seastack_ctest_name(regression oswec "${CASE_ID}" REFERENCE CONDITION_REF_CTEST_NAME)
        seastack_fixture_regression_condition(oswec reg_waves ${CONDITION_NUM} FIXTURE_FILE)
        set(FILE_REF ${SEASTACK_BUILD_DATA}/reference_data/oswec/ss_ref_oswec_reg_waves_${CONDITION_NUM}.txt)
        set(FILE_RST results/tests/regression/oswec/results_oswec_reg_waves_${CONDITION_NUM}.txt)

        add_test(
            NAME ${CONDITION_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_oswec_reg_waves>"
            COMMAND $<TARGET_FILE:test_oswec_reg_waves> --condition ${CONDITION_NUM} --no_gui --no_plot)

        set_tests_properties(
            ${CONDITION_CTEST_NAME}
            PROPERTIES
            LABELS "regression;oswec;execute;oswec_reg_waves_per_condition"
            FIXTURES_SETUP ${FIXTURE_FILE}
            ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
        )

        add_test(
            NAME ${CONDITION_REF_CTEST_NAME}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_oswec_reg_waves>"
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/regression/oswec/compare_oswec_reg_waves.py ${FILE_REF} $<TARGET_FILE_DIR:test_oswec_reg_waves>/${FILE_RST} --condition ${CONDITION_NUM}
        )

        if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;oswec;reference;oswec_reg_waves_per_condition"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${Python3_ROOT_DIR};PATH=path_list_prepend:${Python3_ROOT_DIR}/Library/bin;PATH=path_list_prepend:${Python3_ROOT_DIR}/DLLs"
                ENVIRONMENT "SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        else()
            set_tests_properties(
                ${CONDITION_REF_CTEST_NAME}
                PROPERTIES
                LABELS "regression;oswec;reference;oswec_reg_waves_per_condition"
                FIXTURES_REQUIRED ${FIXTURE_FILE}
                ENVIRONMENT "PATH=${Python3_ROOT_DIR};SEASTACK_BUILD_DIR=${CMAKE_BINARY_DIR}"
                SKIP_RETURN_CODE ${SEASTACK_SKIP_MISSING_PYTHON_DEPS}
            )
        endif()
    endforeach()
endif()

seastack_regression_case_id(sphere sphere_reg_waves_full RW_FULL_CASE)
seastack_ctest_name(regression sphere "${RW_FULL_CASE}" RUN CTEST_SPHERE_REG_FULL)
seastack_fixture_regression(sphere "${RW_FULL_CASE}" FIXTURE_SPHERE_REG_FULL)

add_executable(test_sphere_reg_waves_full sphere/test_sphere_reg_waves.cpp)
target_compile_definitions(test_sphere_reg_waves_full PRIVATE
    "REFERENCE_FILE_NAME=\"${SEASTACK_BUILD_DATA}/reference_data/sphere/ss_ref_sphere_reg_waves_1.txt\""
    "RESULTS_DIR_NAME=\"regression/sphere\""
    "RESULTS_FILE_NAME=\"results_sphere_reg_waves\""
)
target_link_libraries(test_sphere_reg_waves_full PRIVATE SEAStack::ChronoAdapter)
target_compile_features(test_sphere_reg_waves_full PRIVATE ${SEASTACK_CXX_STANDARD})
if(MSVC)
    set_target_properties(test_sphere_reg_waves_full PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_sphere_reg_waves_full>")
endif()
add_test(NAME ${CTEST_SPHERE_REG_FULL} WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_sphere_reg_waves_full>"
    COMMAND $<TARGET_FILE:test_sphere_reg_waves_full> --no_gui --no_plot)
# FIXTURES_SETUP for RAO inputs lives on verification-owned tests (see
# build_rao_verification_test) so ctest -L verification does not pull in
# this regression-labeled test as a fixture prerequisite.
set_tests_properties(${CTEST_SPHERE_REG_FULL} PROPERTIES
    LABELS "regression;sphere;execute;reg_waves_full"
    ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
    COST 500)

seastack_regression_case_id(oswec oswec_reg_waves_full OW_FULL_CASE)
seastack_ctest_name(regression oswec "${OW_FULL_CASE}" RUN CTEST_OSWEC_REG_FULL)
seastack_fixture_regression(oswec "${OW_FULL_CASE}" FIXTURE_OSWEC_REG_FULL)

add_executable(test_oswec_reg_waves_full oswec/test_oswec_reg_waves.cpp)
target_compile_definitions(test_oswec_reg_waves_full PRIVATE
    "REFERENCE_FILE_NAME=\"${SEASTACK_BUILD_DATA}/reference_data/oswec/ss_ref_oswec_reg_waves_1.txt\""
    "RESULTS_DIR_NAME=\"regression/oswec\""
    "RESULTS_FILE_NAME=\"results_oswec_reg_waves\""
)
target_link_libraries(test_oswec_reg_waves_full PRIVATE SEAStack::ChronoAdapter)
target_compile_features(test_oswec_reg_waves_full PRIVATE ${SEASTACK_CXX_STANDARD})
if(MSVC)
    set_target_properties(test_oswec_reg_waves_full PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_oswec_reg_waves_full>")
endif()
add_test(NAME ${CTEST_OSWEC_REG_FULL} WORKING_DIRECTORY "$<TARGET_FILE_DIR:test_oswec_reg_waves_full>"
    COMMAND $<TARGET_FILE:test_oswec_reg_waves_full> --no_gui --no_plot)
set_tests_properties(${CTEST_OSWEC_REG_FULL} PROPERTIES
    LABELS "regression;oswec;execute;reg_waves_full"
    ENVIRONMENT "SEASTACK_DATA_DIR=${SEASTACK_BUILD_DATA}"
    COST 800)
