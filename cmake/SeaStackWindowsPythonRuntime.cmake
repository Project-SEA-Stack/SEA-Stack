# Windows: locate libpython (pythonNN.dll) and peers required at load time by Chrono_parsers, for install() and POST_BUILD.
# Include only under if(WIN32) after find_package(Chrono COMPONENTS ... Parsers) and add_CHRONO_DLLS_copy_command().
# Outputs: SEASTACK_WINDOWS_PYTHON_DLL, SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS (parent scope).

# Chrono_parsers may import python3xx.dll (Parsers embedded Python / Python C API); it is not in Chrono's bin/.
# Use the same discovery for install() and for POST_BUILD copies to bin/<Config> (single source of truth).
find_package(Python3 COMPONENTS Interpreter QUIET)
set(SEASTACK_WINDOWS_PYTHON_DLL "")
set(SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS "")
set(_seastack_py_exe_dir "")
set(_seastack_py_search_roots "")
if(Python3_Interpreter_FOUND)
    get_filename_component(_seastack_py_exe_dir "${Python3_EXECUTABLE}" DIRECTORY)
    list(APPEND _seastack_py_search_roots "${_seastack_py_exe_dir}")
endif()
if(DEFINED Python3_ROOT_DIR AND NOT "${Python3_ROOT_DIR}" STREQUAL "" AND EXISTS "${Python3_ROOT_DIR}")
    list(APPEND _seastack_py_search_roots "${Python3_ROOT_DIR}")
endif()
if(_seastack_py_search_roots)
    list(REMOVE_DUPLICATES _seastack_py_search_roots)
endif()

# Prefer the exact pythonNN.dll that Chrono_parsers imports, not only the DLL name implied by FindPython3.
set(_seastack_required_python_dll "")
if(MSVC AND TARGET Chrono::Chrono_parsers)
    set(_seastack_chrono_parsers_dll "")
    foreach(_cfg RELEASE RELWITHDEBINFO MINSIZEREL DEBUG)
        string(TOUPPER "${_cfg}" _cfg_u)
        get_target_property(_loc Chrono::Chrono_parsers IMPORTED_LOCATION_${_cfg_u})
        if(_loc AND NOT _loc STREQUAL "Chrono::Chrono_parsers-NOTFOUND" AND EXISTS "${_loc}")
            set(_seastack_chrono_parsers_dll "${_loc}")
            break()
        endif()
    endforeach()
    if(NOT _seastack_chrono_parsers_dll)
        get_target_property(_loc Chrono::Chrono_parsers IMPORTED_LOCATION)
        if(_loc AND NOT _loc STREQUAL "Chrono::Chrono_parsers-NOTFOUND" AND EXISTS "${_loc}")
            set(_seastack_chrono_parsers_dll "${_loc}")
        endif()
    endif()
    if(_seastack_chrono_parsers_dll)
        get_filename_component(_seastack_dumpbin_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
        find_program(SEASTACK_DUMPBIN dumpbin HINTS "${_seastack_dumpbin_dir}")
        if(NOT SEASTACK_DUMPBIN)
            find_program(SEASTACK_DUMPBIN dumpbin)
        endif()
        if(SEASTACK_DUMPBIN)
            execute_process(
                COMMAND "${SEASTACK_DUMPBIN}" /nologo /dependents "${_seastack_chrono_parsers_dll}"
                OUTPUT_VARIABLE _seastack_db_out
                ERROR_VARIABLE _seastack_db_err
            )
            string(CONCAT _seastack_db_all "${_seastack_db_out}" "${_seastack_db_err}")
            string(TOLOWER "${_seastack_db_all}" _seastack_db_lc)
            string(REGEX MATCH "python[0-9][0-9]+\\.dll" _seastack_dm "${_seastack_db_lc}")
            if(_seastack_dm)
                set(_seastack_required_python_dll "${_seastack_dm}")
                message(STATUS "Chrono_parsers.dll load-time Python import: ${_seastack_required_python_dll}")
            endif()
        endif()
    endif()
endif()

if(_seastack_required_python_dll AND _seastack_py_search_roots)
    foreach(_root ${_seastack_py_search_roots})
        foreach(_sub IN ITEMS "" "DLLs" "Library/bin")
            if(_sub STREQUAL "")
                set(_cand "${_root}/${_seastack_required_python_dll}")
            else()
                set(_cand "${_root}/${_sub}/${_seastack_required_python_dll}")
            endif()
            if(EXISTS "${_cand}")
                set(SEASTACK_WINDOWS_PYTHON_DLL "${_cand}")
                break()
            endif()
        endforeach()
        if(SEASTACK_WINDOWS_PYTHON_DLL)
            break()
        endif()
    endforeach()
    if(NOT SEASTACK_WINDOWS_PYTHON_DLL)
        message(WARNING
            "Chrono_parsers imports ${_seastack_required_python_dll} but that file was not found under Python search roots "
            "(${_seastack_py_search_roots}). Set PythonRoot/Python3_ROOT_DIR to the same Python environment Chrono's Parsers "
            "module was built against, then reconfigure (clear CMake cache if needed). Packaged run_seastack may fail (0xC0000135).")
    endif()
elseif(Python3_Interpreter_FOUND)
    set(_seastack_py_dll_name "python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}.dll")
    foreach(_root ${_seastack_py_search_roots})
        foreach(_sub IN ITEMS "" "DLLs" "Library/bin")
            if(_sub STREQUAL "")
                set(_cand "${_root}/${_seastack_py_dll_name}")
            else()
                set(_cand "${_root}/${_sub}/${_seastack_py_dll_name}")
            endif()
            if(EXISTS "${_cand}")
                set(SEASTACK_WINDOWS_PYTHON_DLL "${_cand}")
                break()
            endif()
        endforeach()
        if(SEASTACK_WINDOWS_PYTHON_DLL)
            break()
        endif()
    endforeach()
    if(NOT SEASTACK_WINDOWS_PYTHON_DLL)
        message(WARNING
            "Python interpreter found (${Python3_EXECUTABLE}) but ${_seastack_py_dll_name} not under env root, DLLs/, or Library/bin/. "
            "Packaged/naked run_seastack.exe may fail (0xC0000135) if Chrono_parsers links Python. "
            "Use the same PythonRoot in build-config.json as the Python used when building Chrono's Parsers module, or rebuild "
            "Chrono with Parsers not linked to Python if your upstream build allows it.")
    endif()
elseif(_seastack_required_python_dll)
    message(WARNING
        "Chrono_parsers imports ${_seastack_required_python_dll} but Python3 was not found at configure time. "
        "Add Python to PATH or pass -DPython3_ROOT_DIR (see build-config.json PythonRoot).")
endif()

# Stable ABI shim and conda peers (zlib/OpenSSL): beside interpreter, beside resolved pythonNN.dll, and Library/bin.
if(Python3_Interpreter_FOUND OR SEASTACK_WINDOWS_PYTHON_DLL)
    if(Python3_Interpreter_FOUND)
        if(EXISTS "${_seastack_py_exe_dir}/python3.dll")
            list(APPEND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS "${_seastack_py_exe_dir}/python3.dll")
        endif()
        set(_seastack_conda_libbin "${_seastack_py_exe_dir}/Library/bin")
        if(EXISTS "${_seastack_conda_libbin}/python3.dll")
            list(APPEND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS "${_seastack_conda_libbin}/python3.dll")
        endif()
        if(EXISTS "${_seastack_conda_libbin}")
            if(EXISTS "${_seastack_conda_libbin}/zlib.dll")
                list(APPEND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS "${_seastack_conda_libbin}/zlib.dll")
            endif()
            file(GLOB _seastack_conda_ssl
                "${_seastack_conda_libbin}/libssl-*.dll"
                "${_seastack_conda_libbin}/libcrypto-*.dll")
            list(APPEND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS ${_seastack_conda_ssl})
        endif()
    endif()
    if(SEASTACK_WINDOWS_PYTHON_DLL)
        get_filename_component(_seastack_py_dll_dir "${SEASTACK_WINDOWS_PYTHON_DLL}" DIRECTORY)
        if(EXISTS "${_seastack_py_dll_dir}/python3.dll")
            list(APPEND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS "${_seastack_py_dll_dir}/python3.dll")
        endif()
        if(EXISTS "${_seastack_py_dll_dir}/zlib.dll")
            list(APPEND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS "${_seastack_py_dll_dir}/zlib.dll")
        endif()
        file(GLOB _seastack_peer_ssl
            "${_seastack_py_dll_dir}/libssl-*.dll"
            "${_seastack_py_dll_dir}/libcrypto-*.dll")
        list(APPEND SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS ${_seastack_peer_ssl})
    endif()
    if(SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS)
        list(REMOVE_DUPLICATES SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS)
        message(STATUS "Python extra DLLs for packaging (zlib/OpenSSL etc.): ${SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS}")
    endif()
    if(SEASTACK_WINDOWS_PYTHON_DLL)
        message(STATUS "Python runtime DLL for packaging: ${SEASTACK_WINDOWS_PYTHON_DLL}")
    endif()
endif()

if(NOT Python3_Interpreter_FOUND AND NOT _seastack_required_python_dll)
    message(WARNING
        "Python3 interpreter not found at configure time. If Chrono_parsers.dll imports python*.dll, "
        "add Python to PATH or pass -DPython3_ROOT_DIR (see build-config.json PythonRoot) so the matching DLL can be installed.")
endif()

# Call after add_executable(run_seastack): copies the same DLL set as install(FILES ...).
macro(seastack_windows_python_runtime_postbuild _target)
    if(WIN32 AND TARGET "${_target}")
        if(SEASTACK_WINDOWS_PYTHON_DLL)
            add_custom_command(TARGET "${_target}" POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${SEASTACK_WINDOWS_PYTHON_DLL}"
                    "$<TARGET_FILE_DIR:${_target}>"
                COMMENT "SEA-Stack: copy Python runtime DLL for Chrono_parsers")
        endif()
        foreach(_seastack_wpyx IN LISTS SEASTACK_WINDOWS_PYTHON_EXTRA_DLLS)
            if(EXISTS "${_seastack_wpyx}")
                add_custom_command(TARGET "${_target}" POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${_seastack_wpyx}"
                        "$<TARGET_FILE_DIR:${_target}>"
                    COMMENT "SEA-Stack: copy Python-related DLL")
            endif()
        endforeach()
    endif()
endmacro()
