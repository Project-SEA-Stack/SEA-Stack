# External PTO example — ExternalPtoModel + IPC (optional Chrono for package builds)
add_executable(external_pto_example main.cpp)
target_link_libraries(external_pto_example PRIVATE SEAStack::External)
target_compile_features(external_pto_example PRIVATE cxx_std_17)
target_include_directories(external_pto_example PRIVATE "${CMAKE_BINARY_DIR}")

# Canonical example modules live in the RM3 demo directories (they ship in the
# release ZIP under demos/). Copy them next to the executable, alongside the IPC
# helper, so `external_pto_example --python <script>` and the replay harness work
# from the build tree without extra path setup.
set(_ext_pto_demo_dir "${CMAKE_SOURCE_DIR}/data/demos/run_seastack/rm3")
add_custom_command(TARGET external_pto_example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/libs/external/python/seastack_external.py"
        "$<TARGET_FILE_DIR:external_pto_example>/seastack_external.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_ext_pto_demo_dir}/external_pto/linear_damper_pto.py"
        "$<TARGET_FILE_DIR:external_pto_example>/linear_damper_pto.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_ext_pto_demo_dir}/external_pto_adaptive/adaptive_damping_pto.py"
        "$<TARGET_FILE_DIR:external_pto_example>/adaptive_damping_pto.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${_ext_pto_demo_dir}/external_pto_hydraulic/hydraulic_accumulator_pto.py"
        "$<TARGET_FILE_DIR:external_pto_example>/hydraulic_accumulator_pto.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/replay_harness.py"
        "$<TARGET_FILE_DIR:external_pto_example>/replay_harness.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/verify_examples.py"
        "$<TARGET_FILE_DIR:external_pto_example>/verify_examples.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/compare_ipc_replay.py"
        "$<TARGET_FILE_DIR:external_pto_example>/compare_ipc_replay.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/plot_verification.py"
        "$<TARGET_FILE_DIR:external_pto_example>/plot_verification.py"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/run_visual_verification.py"
        "$<TARGET_FILE_DIR:external_pto_example>/run_visual_verification.py"
)
