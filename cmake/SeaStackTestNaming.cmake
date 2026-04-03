# SeaStackTestNaming.cmake
#
# Canonical CTest identity for SEA-Stack integration tests:
#   test_<suite>_<family>_<case_id>[_reference|_verification|_comparison]
#
# Executable target names stay historical (e.g. test_sphere_decay, test_bench_*);
# only CTest NAME values and fixture strings use this scheme.
#
# case_id: strip "${family}_" prefix from the legacy TEST_NAME when present
#          (e.g. sphere + sphere_reg_waves -> reg_waves).

cmake_minimum_required(VERSION 3.18)

function(seastack_regression_case_id FAMILY TEST_NAME OUT_VAR)
    set(prefix "${FAMILY}_")
    string(LENGTH "${prefix}" pl)
    string(LENGTH "${TEST_NAME}" tl)
    if(tl GREATER_EQUAL pl)
        string(SUBSTRING "${TEST_NAME}" 0 ${pl} head)
        if(head STREQUAL prefix)
            math(EXPR rest_start "${pl}")
            string(SUBSTRING "${TEST_NAME}" ${rest_start} -1 rest)
            set(${OUT_VAR} "${rest}" PARENT_SCOPE)
            return()
        endif()
    endif()
    set(${OUT_VAR} "${TEST_NAME}" PARENT_SCOPE)
endfunction()

# VER_GROUP is a path/dataset slug like sphere_decay_multicode -> family sphere, case decay_multicode
function(seastack_verification_split_group VER_GROUP OUT_FAMILY OUT_CASE)
    string(FIND "${VER_GROUP}" "_" pos)
    if(pos LESS 0)
        set(${OUT_FAMILY} "" PARENT_SCOPE)
        set(${OUT_CASE} "${VER_GROUP}" PARENT_SCOPE)
        return()
    endif()
    string(SUBSTRING "${VER_GROUP}" 0 ${pos} fam)
    math(EXPR pos1 "${pos}+1")
    string(SUBSTRING "${VER_GROUP}" ${pos1} -1 rest)
    set(${OUT_FAMILY} "${fam}" PARENT_SCOPE)
    set(${OUT_CASE} "${rest}" PARENT_SCOPE)
endfunction()

# bench_sphere_decay_conv + family sphere -> decay_conv
function(seastack_benchmark_case_id FAMILY BENCH_NAME OUT_VAR)
    set(prefix "bench_${FAMILY}_")
    string(LENGTH "${prefix}" pl)
    string(LENGTH "${BENCH_NAME}" tl)
    if(tl GREATER_EQUAL pl)
        string(SUBSTRING "${BENCH_NAME}" 0 ${pl} head)
        if(head STREQUAL prefix)
            math(EXPR rest_start "${pl}")
            string(SUBSTRING "${BENCH_NAME}" ${rest_start} -1 rest)
            set(${OUT_VAR} "${rest}" PARENT_SCOPE)
            return()
        endif()
    endif()
    set(${OUT_VAR} "${BENCH_NAME}" PARENT_SCOPE)
endfunction()

# ROLE: RUN | REFERENCE | VERIFICATION | COMPARISON
function(seastack_ctest_name SUITE FAMILY CASE_ID ROLE OUT_VAR)
    if(ROLE STREQUAL "RUN")
        set(_n "test_${SUITE}_${FAMILY}_${CASE_ID}")
    elseif(ROLE STREQUAL "REFERENCE")
        set(_n "test_${SUITE}_${FAMILY}_${CASE_ID}_reference")
    elseif(ROLE STREQUAL "VERIFICATION")
        set(_n "test_${SUITE}_${FAMILY}_${CASE_ID}_verification")
    elseif(ROLE STREQUAL "COMPARISON")
        set(_n "test_${SUITE}_${FAMILY}_${CASE_ID}_comparison")
    else()
        message(FATAL_ERROR "seastack_ctest_name: unknown ROLE '${ROLE}'")
    endif()
    set(${OUT_VAR} "${_n}" PARENT_SCOPE)
endfunction()

function(seastack_fixture_regression FAMILY CASE_ID OUT_VAR)
    set(${OUT_VAR} "fixture_regression_${FAMILY}_${CASE_ID}" PARENT_SCOPE)
endfunction()

function(seastack_fixture_regression_condition FAMILY CASE_BASE CONDITION_NUM OUT_VAR)
    set(${OUT_VAR} "fixture_regression_${FAMILY}_${CASE_BASE}_c${CONDITION_NUM}" PARENT_SCOPE)
endfunction()

function(seastack_fixture_verification FAMILY CASE_ID OUT_VAR)
    set(${OUT_VAR} "fixture_verification_${FAMILY}_${CASE_ID}" PARENT_SCOPE)
endfunction()

function(seastack_fixture_comparison FAMILY CASE_ID OUT_VAR)
    set(${OUT_VAR} "fixture_comparison_${FAMILY}_${CASE_ID}" PARENT_SCOPE)
endfunction()
