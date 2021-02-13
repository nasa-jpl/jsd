add_executable(jsd_slaveinfo jsd_slaveinfo.c)
target_link_libraries(jsd_slaveinfo jsd-lib)

find_package(readline)
if(READLINE_FOUND)
    include_directories(${READLINE_INCLUDE_DIRS})
    add_executable(jsd_egd_tlc_tty jsd_egd_tlc_tty.c)
    target_link_libraries(jsd_egd_tlc_tty jsd-lib ${READLINE_LIBRARIES})
    message(STATUS "${Green}Building jsd_egd_tlc_tty utility${ColorReset}")
else(READLINE_FOUND)
    message(STATUS "${Yellow}Could not find readline, not building jsd_egd_tlc_tty utility${ColorReset}")
endif(READLINE_FOUND)

