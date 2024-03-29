if(BUILD_JSD_TESTS)

    include(CTest)
    message("${Green}Building JSD tests, disable with BUILD_JSD_TESTS CMake option ${ColorReset}")

    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_BINARY_DIR}/include
        )
    set(jsd_test_libs jsd-lib m)

    #########  Unit Tests #########
    add_executable(jsd_alloc_free_test unit/jsd_alloc_free_test.c)
    target_link_libraries(jsd_alloc_free_test ${jsd_test_libs})
    add_test(NAME jsd_alloc_free_test COMMAND jsd_alloc_free_test)

    add_executable(jsd_soem_init_close_test unit/jsd_soem_init_close_test.c)
    target_link_libraries(jsd_soem_init_close_test ${jsd_test_libs})
    add_test(NAME jsd_soem_init_close_test COMMAND jsd_soem_init_close_test)

    add_executable(jsd_error_cirq_test unit/jsd_error_cirq_test.c)
    target_link_libraries(jsd_error_cirq_test ${jsd_test_libs})
    add_test(NAME jsd_error_cirq_test COMMAND jsd_error_cirq_test)

    add_executable(jsd_epd_lc_to_do_test unit/jsd_epd_lc_to_do_test.c)
    target_link_libraries(jsd_epd_lc_to_do_test ${jsd_test_libs})
    add_test(NAME jsd_epd_lc_to_do_test COMMAND jsd_epd_lc_to_do_test)

    ######### Device Tests #########
    add_executable(jsd_minimal_example_el3602 device/jsd_minimal_example_el3602.c)
    target_link_libraries(jsd_minimal_example_el3602 ${jsd_test_libs})


    add_executable(jsd_device_test_template
        device/jsd_device_test_template.c
        jsd_test_utils.c)
    target_link_libraries(jsd_device_test_template ${jsd_test_libs})

    add_executable(jsd_el1008_test
        device/jsd_el1008_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_el1008_test ${jsd_test_libs})

    add_executable(jsd_el2124_test
        device/jsd_el2124_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_el2124_test ${jsd_test_libs})

    add_executable(jsd_el2809_test
        device/jsd_el2809_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_el2809_test ${jsd_test_libs})

    add_executable(jsd_el3602_test
        device/jsd_el3602_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_el3602_test ${jsd_test_libs})

    add_executable(jsd_el3202_test
            device/jsd_el3202_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_el3202_test ${jsd_test_libs})

    add_executable(jsd_el3208_test
        device/jsd_el3208_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_el3208_test ${jsd_test_libs})

    add_executable(jsd_egd_test
        device/jsd_egd_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_egd_test ${jsd_test_libs})

    add_executable(jsd_egd_csp_sine_test
        device/jsd_egd_csp_sine_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_egd_csp_sine_test ${jsd_test_libs})

    add_executable(jsd_beckhoff_latency_test
        device/jsd_beckhoff_latency_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_beckhoff_latency_test ${jsd_test_libs})

    add_executable(jsd_el3356_test
        device/jsd_el3356_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_el3356_test ${jsd_test_libs})

    add_executable(jsd_jed0101_test
        device/jsd_jed0101_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_jed0101_test ${jsd_test_libs})

    add_executable(jsd_jed0200_test
        device/jsd_jed0200_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_jed0200_test ${jsd_test_libs})

    add_executable(jsd_ati_fts_test
        device/jsd_ati_fts_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_ati_fts_test ${jsd_test_libs})

    add_executable(jsd_el3104_test
            device/jsd_el3104_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_el3104_test ${jsd_test_libs})

    add_executable(jsd_el3318_test
            device/jsd_el3318_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_el3318_test ${jsd_test_libs})

    add_executable(jsd_el3162_test
            device/jsd_el3162_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_el3162_test ${jsd_test_libs})

    add_executable(jsd_el4102_test
            device/jsd_el4102_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_el4102_test ${jsd_test_libs})

    add_executable(jsd_ild1900_test
            device/jsd_ild1900_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_ild1900_test ${jsd_test_libs})

    add_executable(jsd_epd_nominal_csp_sine_test
            device/jsd_epd_nominal_csp_sine_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_nominal_csp_sine_test ${jsd_test_libs})

    add_executable(jsd_epd_nominal_csv_sine_test
            device/jsd_epd_nominal_csv_sine_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_nominal_csv_sine_test ${jsd_test_libs})

    add_executable(jsd_epd_nominal_cst_test
            device/jsd_epd_nominal_cst_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_nominal_cst_test ${jsd_test_libs})

    add_executable(jsd_epd_nominal_prof_pos_test
            device/jsd_epd_nominal_prof_pos_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_nominal_prof_pos_test ${jsd_test_libs})

    add_executable(jsd_epd_nominal_prof_vel_test
            device/jsd_epd_nominal_prof_vel_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_nominal_prof_vel_test ${jsd_test_libs})

    add_executable(jsd_epd_nominal_prof_torque_test
            device/jsd_epd_nominal_prof_torque_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_nominal_prof_torque_test ${jsd_test_libs})

    add_executable(jsd_epd_nominal_network_test
            device/jsd_epd_nominal_network_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_nominal_network_test ${jsd_test_libs})

    add_executable(jsd_epd_sil_test
            device/jsd_epd_sil_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_sil_test ${jsd_test_libs})

    add_executable(jsd_epd_sil_csp_sine_test
            device/jsd_epd_sil_csp_sine_test.c
            jsd_test_utils.c)
    target_link_libraries(jsd_epd_sil_csp_sine_test ${jsd_test_libs})

    find_program(VALGRIND_EXECUTABLE NAMES valgrind)
    if(VALGRIND_EXECUTABLE)
        set(CTEST_MEMORYCHECK_COMMAND ${VALGRIND_EXECUTABLE})
        set(MEMORYCHECK_COMMAND_OPTIONS
            "--trace-children=yes --leak-check=full --track-origins=yes --error-exitcode=1")
        add_custom_target(
            memcheck
            ${CMAKE_CTEST_COMMAND} -D ExperimentalMemCheck \${ARGS}
            WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        )
        if(CMAKE_TEST_COMMAND)
            message(STATUS "CMAKE_TEST_COMMAND=${CMAKE_TEST_COMMAND}")
        endif()
        message(STATUS "VALGRIND_EXECUTABLE=${VALGRIND_EXECUTABLE}")
    else()
        message(WARNING "Memcheck target will not be built, valgrind was not found on this machine")
    endif()

else(BUILD_JSD_TESTS)

    message("${Yellow}Not building JSD tests, enable with BUILD_JSD_TESTS CMake option ${ColorReset}")

endif(BUILD_JSD_TESTS)
