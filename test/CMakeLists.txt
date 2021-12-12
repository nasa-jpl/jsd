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

    ######### Device Tests #########
    add_executable(jsd_minimal_example_el3602 device/jsd_minimal_example_el3602.c)
    target_link_libraries(jsd_minimal_example_el3602 ${jsd_test_libs})


    add_executable(jsd_device_test_template
        device/jsd_device_test_template.c
        jsd_test_utils.c)
    target_link_libraries(jsd_device_test_template ${jsd_test_libs})

    add_executable(jsd_el2124_test
        device/jsd_el2124_test.c
        jsd_test_utils.c)
    target_link_libraries(jsd_el2124_test ${jsd_test_libs})

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
    else()
        message(WARNING "Memcheck target will not be built, valgrind was not found on this machine")
    endif()

else(BUILD_JSD_TESTS)

    message("${Yellow}Not building JSD tests, enable with BUILD_JSD_TESTS CMake option ${ColorReset}")

endif(BUILD_JSD_TESTS)
