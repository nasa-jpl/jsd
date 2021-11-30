cmake_minimum_required(VERSION 3.11)
project(jsd
    DESCRIPTION "JPL SOEM Drivers"
    VERSION 1.5.1
    LANGUAGES C CXX
    )

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(
    -Wall
    -Wextra
    -Werror
    -Wno-missing-field-initializers
    )

if(DISABLE_DEBUG_PRINTS)
    remove_definitions(-DDEBUG)
else(DISABLE_DEBUG_PRINTS)
    add_definitions(-DDEBUG)
endif(DISABLE_DEBUG_PRINTS)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# for pretty printing
string(ASCII 27 Esc)
set(ColorReset "${Esc}[m")
set(Yellow "${Esc}[33m")
set(Green "${Esc}[32m")

# install project headers to binary directory
file(GLOB_RECURSE includes RELATIVE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hh
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
    )
foreach(include ${includes})
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${include}"
        "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${include}"
        COPYONLY
        )
endforeach()

####### Dependencies ########

remove_definitions(-DEC_VER1)
add_definitions(-DEC_VER2)

include(FetchContent)
FetchContent_Declare(soem
  GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM.git
    GIT_TAG f9db4f2b23455e83d9871568e3a69ab124e0c1a0
    )
FetchContent_MakeAvailable(soem)

set(SOEM_INCLUDE_DIRS
    ${soem_SOURCE_DIR}/soem
    ${soem_SOURCE_DIR}/osal
    )
if(WIN32)
    set(OS "win32")
elseif(UNIX AND NOT APPLE)
    set(OS "linux")
elseif(APPLE)
    set(OS "macosx")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
    list(APPEND SOEM_INCLUDE_DIRS ${soem_SOURCE_DIR}/oshw/${OS}/${ARCH})
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
    set(OS "rtems")
endif()

list(APPEND SOEM_INCLUDE_DIRS
    ${soem_SOURCE_DIR}/osal/${OS}
    ${soem_SOURCE_DIR}/oshw/${OS}
    )

add_subdirectory(src)
add_subdirectory(tools)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    enable_testing()
    add_subdirectory(test)
else()
    message(STATUS "Not building jsd tests when built as a submodule")
endif()

######### Install Rules #########
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_INSTALL_PREFIX /opt/jsd)
    install(DIRECTORY ${CMAKE_BINARY_DIR}/include/jsd DESTINATION include)
    install(DIRECTORY ${CMAKE_BINARY_DIR}/bin/ DESTINATION bin)
    install(TARGETS jsd-lib DESTINATION lib)
    install(TARGETS jsd_slaveinfo DESTINATION bin)
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)

    ######### "doc" target #########
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_SOURCE_DIR}/.doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/.doxyfile)
        add_custom_target ( doc
            COMMAND ${DOXYGEN_EXECUTABLE} "${CMAKE_CURRENT_BINARY_DIR}/.doxyfile"
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Creating Doxygen Documentation"
            VERBATIM
            )
    endif()

    ######### "format" target #########
    find_program(FOUND_CLANG_FORMAT clang-format)
    if(FOUND_CLANG_FORMAT)
        FILE(GLOB_RECURSE C_FILES
            "${CMAKE_CURRENT_LIST_DIR}/src/*.[ch]"
            "${CMAKE_CURRENT_LIST_DIR}/tools/*.[ch]"
            "${CMAKE_CURRENT_LIST_DIR}/test/*.[ch]")
        add_custom_target( format
            COMMAND clang-format -i -style=file ${C_FILES}
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
            COMMENT "Formatting all .c/.h files recursively under: ${CMAKE_CURRENT_LIST_DIR}"
            VERBATIM
            )
    endif()

endif()
