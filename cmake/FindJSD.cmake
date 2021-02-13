# This module defines
#  SOEM_FOUND, true if found
#  JSD_LIBRARIES expression containing libraries
#  JSD_INCLUDE_DIRS, expression containig headers

# Searches /usr/local, /opt/JSD, or $JSD_INSTALL_DIR for installed JSD

include(FindPackageHandleStandardArgs)
if(NOT JSD_FOUND)

  find_package(SOEM REQUIRED)


  find_library(JSD_LIBRARIES 
    NAME jsd-lib
    PATHS 
    /usr/local
    /opt/jsd/lib
    ${JSD_INSTALL_DIR}
    PATH_SUFFIXES lib)


  if(${CMAKE_VERSION} VERSION_GREATER "3.5.0")
    find_package(Threads REQUIRED)
    list(APPEND JSD_LIBRARIES ${SOEM_LIBRARIES} Threads::Threads rt)
  else(${CMAKE_VERSION} VERSION_GREATER "3.5.0")
    list(APPEND JSD_LIBRARIES ${SOEM_LIBRARIES} pthread rt)
  endif(${CMAKE_VERSION} VERSION_GREATER "3.5.0")

  find_path(JSD_INCLUDE_DIRS 
    NAME jsd/jsd.h 
    PATHS
    /usr/local
    /opt/jsd/include
    /opt/
    ${JSD_INSTALL_DIR}
    PATH_SUFFIXES include)

  list(APPEND JSD_INCLUDE_DIRS ${SOEM_INCLUDE_DIRS})

  set(JSD_VERSION 1.3.0)

  find_package_handle_standard_args(JSD 
    REQUIRED_VARS JSD_LIBRARIES JSD_INCLUDE_DIRS
    VERSION_VAR JSD_VERSION
    FAIL_MESSAGE "Find JSD unsuccessful")

  if(JSD_FOUND)
    message (STATUS "JSD_INCLUDE_DIRS = ${JSD_INCLUDE_DIRS}")
    message (STATUS "JSD_LIBRARIES = ${JSD_LIBRARIES}")
  else(JSD_FOUND)
    if(JSD_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find JSD!")
    endif(JSD_FIND_REQUIRED)
  endif(JSD_FOUND)

endif(NOT JSD_FOUND)
