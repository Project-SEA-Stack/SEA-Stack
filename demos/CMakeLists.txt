function(build_demo DEMO_GROUP DEMO_NAME)

    message(STATUS "...${DEMO_NAME}")

    set(DEMO_SRC ${DEMO_GROUP}/demo_${DEMO_NAME}.cpp)
    set(DEMO_EXE demo_${DEMO_NAME})

    add_executable(${DEMO_EXE} ${DEMO_SRC})
    source_group("" FILES ${DEMO_SRC})
    seastack_apply_compile_settings(${DEMO_EXE})

    target_compile_definitions(${DEMO_EXE} 
         PRIVATE
        "RESULTS_DIR_NAME=\"${DEMO_GROUP}\""
    )

    if(MSVC)
        set_target_properties(${DEMO_EXE} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${DEMO_EXE}>")
    endif()

    target_link_libraries(${DEMO_EXE} PRIVATE seastack_app_lib)

endfunction()

# Build demos
# ===========

message(STATUS "Add SEA-Stack demos")

build_demo(sphere    sphere_decay)
build_demo(sphere    sphere_decay_ss)
build_demo(oswec     oswec_irreg_waves)
build_demo(f3of      f3of_irreg_waves)

build_demo(rm3       rm3_hydraulic_pto)

build_demo(trimaran  trimaran_hydro)
build_demo(trimaran  trimaran_rigid)
build_demo(trimaran  trimaran_fea)

if(SEASTACK_ENABLE_MOORING)
    build_demo(rm3   rm3_irreg_mooring)
    build_demo(5sa   5sa_spreading)
    build_demo(5sa   5sa_bimodal)
endif()
