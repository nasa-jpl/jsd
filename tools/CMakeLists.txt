add_executable(jsd_slaveinfo jsd_slaveinfo.c)
target_link_libraries(jsd_slaveinfo jsd-lib)

add_executable(jsd_egd_tlc_tty jsd_egd_tlc_tty.c)
target_link_libraries(jsd_egd_tlc_tty jsd-lib -lreadline)
message(STATUS "${Green}Building jsd_egd_tlc_tty utility${ColorReset}")

