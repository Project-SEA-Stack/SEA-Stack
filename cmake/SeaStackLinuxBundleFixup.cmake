# Linux install-time fixup: make bin/run_seastack and every bundled lib/*.so*
# relocatable via package-relative RUNPATH only.
# Run via install(SCRIPT) after RUNTIME_DEPENDENCIES stages third-party libs.
#
# Prefer:
#   bin/run_seastack  → RUNPATH $ORIGIN/../lib
#   lib/*.so*         → RUNPATH $ORIGIN
#
# Absolute host Chrono/VSG/build-tree RUNPATHs left on imported .so files would
# otherwise win over $ORIGIN and pull libraries from the build machine.
#
# Uses patchelf --set-rpath: Chrono/VSG libraries often ship with no DT_RUNPATH,
# and CMake's file(RPATH_SET) cannot create one from scratch. patchelf is a
# required packaging-host tool for Linux ZIP builds.

set(_fx_prefix "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
set(_fx_lib "${_fx_prefix}/lib")
set(_fx_exe "${_fx_prefix}/bin/run_seastack")

function(_fx_fail msg)
    message(FATAL_ERROR "Linux bundle fixup: ${msg}")
endfunction()

if(NOT EXISTS "${_fx_exe}")
    _fx_fail("${_fx_exe} not found")
endif()
if(NOT IS_DIRECTORY "${_fx_lib}")
    _fx_fail("${_fx_lib} not found")
endif()

find_program(_fx_patchelf NAMES patchelf)
if(NOT _fx_patchelf)
    _fx_fail(
        "patchelf not found on PATH. Linux packaging needs patchelf to set "
        "RUNPATH on bundled Chrono/VSG libraries that ship with no DT_RUNPATH "
        "(install patchelf, then re-run cmake --install / cpack).")
endif()

function(_fx_write_rpath file new_rpath)
    execute_process(
        COMMAND "${_fx_patchelf}" --set-rpath "${new_rpath}" "${file}"
        RESULT_VARIABLE _res
        OUTPUT_VARIABLE _out
        ERROR_VARIABLE _err)
    if(NOT _res EQUAL 0)
        _fx_fail("patchelf --set-rpath '${new_rpath}' failed for ${file}: ${_err}${_out}")
    endif()
endfunction()

# Collect ELF shared libraries (versioned names; skip symlinks).
file(GLOB _fx_candidates LIST_DIRECTORIES false "${_fx_lib}/*.so" "${_fx_lib}/*.so.*")
set(_fx_libs "")
foreach(_f IN LISTS _fx_candidates)
    if(NOT IS_SYMLINK "${_f}")
        list(APPEND _fx_libs "${_f}")
    endif()
endforeach()
list(REMOVE_DUPLICATES _fx_libs)

_fx_write_rpath("${_fx_exe}" "\$ORIGIN/../lib")
foreach(_f IN LISTS _fx_libs)
    _fx_write_rpath("${_f}" "\$ORIGIN")
endforeach()

# Verify with readelf: every RPATH/RUNPATH entry must be $ORIGIN or $ORIGIN/...
function(_fx_assert_relocatable file)
    execute_process(
        COMMAND readelf -d "${file}"
        RESULT_VARIABLE _res
        OUTPUT_VARIABLE _out
        ERROR_VARIABLE _err)
    if(NOT _res EQUAL 0)
        _fx_fail("readelf -d failed for ${file}: ${_err}")
    endif()
    set(_found_any FALSE)
    set(_bad "")
    string(REPLACE "\n" ";" _lines "${_out}")
    foreach(_line IN LISTS _lines)
        if(_line MATCHES "\\((RPATH|RUNPATH)\\)[^[]*\\[([^]]+)\\]")
            set(_found_any TRUE)
            set(_val "${CMAKE_MATCH_2}")
            string(REPLACE ":" ";" _entries "${_val}")
            foreach(_e IN LISTS _entries)
                if(_e STREQUAL "")
                    continue()
                endif()
                if(NOT _e MATCHES "^\\$ORIGIN(/.*)?$")
                    list(APPEND _bad "${_e}")
                endif()
            endforeach()
        endif()
    endforeach()
    if(NOT _found_any)
        _fx_fail("${file}: missing RPATH/RUNPATH after fixup")
    endif()
    if(_bad)
        list(JOIN _bad ", " _bad_joined)
        _fx_fail("${file}: non-package-relative RPATH/RUNPATH entry: ${_bad_joined}")
    endif()
endfunction()

_fx_assert_relocatable("${_fx_exe}")
foreach(_f IN LISTS _fx_libs)
    _fx_assert_relocatable("${_f}")
endforeach()

list(LENGTH _fx_libs _fx_count)
message(STATUS "Linux bundle fixup: run_seastack + ${_fx_count} shared libs use package-relative RUNPATH (via ${_fx_patchelf})")
