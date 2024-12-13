add_library(jsd-lib STATIC
    jsd.c
    jsd_sdo.c
    jsd_error_cirq.c
    jsd_common_device_types.c
    jsd_elmo_common.c
    jsd_epd_common.c

    # Devices
    jsd_el3602.c
    jsd_el3208.c
    jsd_el2124.c
    jsd_el2809.c
    jsd_el2798.c
    jsd_el2828.c
    jsd_egd.c
    jsd_el3356.c
    jsd_jed0101.c
    jsd_jed0200.c
    jsd_timer.c
    jsd_ati_fts.c
    jsd_el3104.c
    jsd_el1008.c
    jsd_el3202.c
    jsd_el3318.c
    jsd_el3162.c
    jsd_el4102.c
    jsd_el5042.c
    jsd_ild1900.c
    jsd_epd_nominal.c
    jsd_epd_sil.c
    )

message(STATUS "SOEM INCLUDE DIRS: ${SOEM_INCLUDE_DIRS}")

target_include_directories(
    jsd-lib PUBLIC
    ${CMAKE_BINARY_DIR}/include
    ${SOEM_INCLUDE_DIRS}
    )

find_package(Threads REQUIRED)
target_link_libraries(jsd-lib
    PUBLIC soem
    PRIVATE Threads::Threads
    PRIVATE rt
    )
