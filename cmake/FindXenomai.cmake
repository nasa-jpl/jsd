###############################################################################
#
# CMake script for finding the Xenomai Libraries and skins
#
# Copyright 2015-2018 Leopold Palomo-Avellaneda <leopold.palomo@upc.edu>
# Copyright 2016-2018 Institute of Industrial and Control Engineering (IOC)
#                     Universitat Politecnica de Catalunya
#                     BarcelonaTech
# Redistribution and use is allowed according to the terms of the 3-clause BSD
# license.
#
#
# Input variables:
# 
# Cache variables (not intended to be used in CMakeLists.txt files)
# Input variables:
# 
# - ${Xenomai_ROOT_DIR} (optional): Used as a hint to find the Xenomai root dir
# - $ENV{XENOMAI_ROOT_DIR} (optional): Used as a hint to find the Xenomai root dir
#
# Output Variables:
#
# - Xenomai_FOUND: Boolean that indicates if the package was found. Default is Native
# - Xenomai_SKINS: List of the available skins. 
#   The availabe skins are for 2.6: NATIVE POSIX PSOS RTDM UITRON VRTX VXWORKS
#   The availabe skins are for 3.x: POSIX ALCHEMY RTDM PSOS VXWORKS SMOKEY COBALT NATIVE
#   NATIVE for compatibility purpose    
# - Xenomai_*_FOUND variables below for individual skins.
# - Xenomai_VERSION: major.minor.patch Xenomai version string
# - Xenomai_XENO_CONFIG: Path to xeno-config program
# 
# For each available skin, theses Var are set:
#   - Xenomai_${SKIN}_FOUND: Boolean that indicates if the skin was found
#   - Xenomai_${SKIN}_DEFINITIONS
#   - Xenomai_${SKIN}_INCLUDE_DIRS
#   - Xenomai_${SKIN}_LIBRARY_DIRS
#   - Xenomai_${SKIN}_LIBRARIES
#   - Xenomai_${SKIN}_LDFLAGS
#   - Xenomai_${SKIN}_DL_FLAGS # Direct Linker Flags
# From Xenomai 3.0.2. If you want to build a shared library
#   - Xenomai_${SKIN}_LDFLAGS_LIBRARIES
#   - Xenomai_${SKIN}_DL_FLAGS_LIBRARIES
#
# Specific var for compatibility with Johns Hopkins University (JHU) FindXenomai
#
# - Individual library variables:
#   - Xenomai_LIBRARY_XENOMAI
#   - Xenomai_LIBRARY_NATIVE
#   - Xenomai_LIBRARY_PTHREAD_RT
#   - Xenomai_LIBRARY_RTDM
#   - Xenomai_LIBRARY_RTDK ( this will be empty, deprecated after Xenomai 2.6.0)
#
#
#
# Example usage:
#
# find_package(Xenomai 2.6.4 REQUIRED POSIX)
# message(STATUS "Xenomai found with theses skins: ${Xenomai_SKINS}")
# # You have some sources xeno-example ${XENO_EXAMPLE_SRC}
# 
# include_directories(${Xenomai_POSIX_INCLUDE_DIR})
# add_executable(demo_xeno ${XENO_EXAMPLE_SRC})
# target_link_libraries(demo_xeno ${Xenomai_POSIX_LIBRARY_DIRS} ${Xenomai_POSIX_LIBRARIES})
# set_target_properties(demo_xeno PROPERTIES
#                  LINK_FLAGS ${Xenomai_POSIX_LDFLAGS})
# target_compile_definitions(demo_xeno PUBLIC ${Xenomai_POSIX_DEFINITIONS})


###############################################################################
# Returns the definitions and includes from a cflags output
# 
# _list cflags var
# _includes return var with the Includes
# _definitions return var with the Definitions 
function(PARSE_CFLAGS _list _includes _definitions)

   separate_arguments(${_list})
   foreach(_item ${${_list}})
      if("${_item}" MATCHES "^-I")
         string(SUBSTRING ${_item} 2 -1 _item_t)
         set(_includes_t ${_includes_t} ${_item_t})
      else("${_item}" MATCHES "^-I")
         set(_definitions_t ${_definitions_t} ${_item})
      endif("${_item}" MATCHES "^-I")
   endforeach(_item)
   
   string (REPLACE ";" " " _definitions_k "${_definitions_t}")
   
   set(${_includes} ${_includes_t} PARENT_SCOPE )
   set(${_definitions} ${_definitions_k} PARENT_SCOPE)

   #message(STATUS "Calling Macro ${${_list}} and returning ${_includes_k} and ${_definitions_k}")
endfunction(PARSE_CFLAGS)
###############################################################################


###############################################################################
# Retrieve information from a ldflags output
# 
# _list ldflags var
# _libraries return var with the libraries
# _ldflags return var with the ldflags value
# _libdir return var with the libdir value
# _dlflags return var with the direct output

function(PARSE_LDFLAGS _list _libraries _ldflags _libdir _dlflags)
   separate_arguments(${_list})
   foreach(_item ${${_list}})
      if("${_item}" MATCHES "^-l")
         string(SUBSTRING ${_item} 2 -1 _item_t)   
         set(_libraries_t ${_libraries_t} ${_item_t})
      elseif("${_item}" MATCHES "^-L")
         set(_libdir_t ${_libdir_t} ${_item})
      else("${_item}" MATCHES "^-L")
         set(_ldflags_t ${_ldflags_t} ${_item})
      endif("${_item}" MATCHES "^-l")
   endforeach(_item ${${_list}})
   
   string (REPLACE ";" " " _ldflags_k "${_ldflags_t}")
   string (REPLACE ";" " " _libdir_k "${_libdir_t}")
   string (REPLACE ";" " " _dlflags_t "${${_list}}")
   
   set(${_libraries} ${_libraries_t} PARENT_SCOPE )
   set(${_ldflags} ${_ldflags_k} PARENT_SCOPE)
   set(${_libdir} ${_libdir_k} PARENT_SCOPE)
   set(${_dlflags} ${_dlflags_t} PARENT_SCOPE) 

   #message(STATUS "Calling Macro ${${_list}} and returning ${_libraries_k} ${_ldflags_k} ${_libdir_k}")
endfunction(PARSE_LDFLAGS)
###############################################################################

include(FindPackageHandleStandardArgs)

if( Xenomai_FIND_COMPONENTS )
  # if components specified in find_package(), make sure each of those pieces were found
  set(_XENOMAI_FOUND_REQUIRED_VARS )
  foreach( component ${Xenomai_FIND_COMPONENTS} )
    string( TOUPPER ${component} _COMPONENT )
    set(_XENOMAI_FOUND_REQUIRED_VARS ${_XENOMAI_FOUND_REQUIRED_VARS} Xenomai_${_COMPONENT}_INCLUDE_DIRS  Xenomai_${_COMPONENT}_LIBRARIES)
  endforeach()
else()
  # if no components specified, we'll make a default set of required variables to say Qt is found
  set(_XENOMAI_FOUND_REQUIRED_VARS Xenomai_XENO_CONFIG Xenomai_VERSION Xenomai_NATIVE_INCLUDE_DIRS Xenomai_NATIVE_LIBRARIES)
endif()

# Get hint from environment variable (if any)
if(NOT $ENV{XENOMAI_ROOT_DIR} STREQUAL "")
  set(XENOMAI_ROOT_DIR $ENV{XENOMAI_ROOT_DIR} CACHE PATH "Xenomai base directory location (optional, used for nonstandard installation paths)" FORCE)
  mark_as_advanced(XENOMAI_ROOT_DIR)
endif()

if(DEFINED ENV{SDKTARGETSYSROOT} AND NOT DEFINED Xenomai_ROOT_DIR)
  message("Found $SDKTARGETSYSROOT = $ENV{SDKTARGETSYSROOT}, using as Xenomai_ROOT_DIR and settings DESTDIR")
  set(Xenomai_ROOT_DIR "$ENV{SDKTARGETSYSROOT}")
  SET(ENV{DESTDIR} $ENV{SDKTARGETSYSROOT})
endif()
# set the search paths
set( Xenomai_SEARCH_PATH /usr/local /usr $ENV{XENOMAI_ROOT_DIR} ${Xenomai_ROOT_DIR} ${Xenomai_ROOT_DIR}/usr/)

# searching kernel headers 
# Find kernel headers
find_package(KernelHeaders)

find_path(Xenomai_KERNEL_INCLUDE_DIR version.h
   PATHS ${KERNELHEADERS_DIR}/include/xenomai/)
         
if(Xenomai_KERNEL_INCLUDE_DIR)
	message(STATUS "Xenomai Kernel Headers located in ${Xenomai_KERNEL_INCLUDE_DIR}")
endif()


# Find xeno-config
find_program(Xenomai_XENO_CONFIG NAMES xeno-config  PATHS ${Xenomai_SEARCH_PATH}/bin NO_DEFAULT_PATH)

if(Xenomai_XENO_CONFIG)
   set(Xenomai_FOUND ${Xenomai_XENO_CONFIG})
   execute_process(COMMAND ${Xenomai_XENO_CONFIG} --version OUTPUT_VARIABLE Xenomai_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE )
   message(STATUS "xeno-config is found. Xenomai should be ok and version found is ${Xenomai_VERSION}")

   if(${Xenomai_VERSION} VERSION_GREATER "3.0.2") 
       message(STATUS "xeno-config has the --auto-init-solib option")
   endif()
   if(${Xenomai_VERSION} VERSION_LESS "3.0.0")
      set(SKINS native posix psos rtdm uitron vrtx vxworks)
   else(${Xenomai_VERSION} VERSION_LESS "3.0.0")
      set(SKINS posix vxworks psos alchemy rtdm smokey cobalt native)
      # From xeno-config manual
      # --native and --skin=native are accepted for backward compatibility purpose. They are stricly equivalent to passing --alchemy --compat.
      #     Likewise, passing --rtdm or --skin=rtdm is stricly equivalent to passing --posix, enabling POSIX I/O routines to be wrapped to their
      #     respective Xenomai implementation.
   endif()

   foreach(_skin ${SKINS})
      #message(STATUS "Processing skin ${_skin} ........")
      string(TOUPPER ${_skin} SKINU)
      
      execute_process(COMMAND ${Xenomai_XENO_CONFIG}  --skin "${_skin}" --cflags OUTPUT_VARIABLE XENO_${SKINU}_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
      PARSE_CFLAGS("XENO_${SKINU}_CFLAGS" "Xenomai_${SKINU}_INCLUDE_DIRS" "Xenomai_${SKINU}_DEFINITIONS")
      #message(STATUS "Xenomai_${SKINU}_INCLUDE_DIRS = ${Xenomai_${SKINU}_INCLUDE_DIRS}")
      #message(STATUS "Xenomai_${SKINU}_DEFINITIONS = ${Xenomai_${SKINU}_DEFINITIONS}")

      execute_process(COMMAND ${Xenomai_XENO_CONFIG} --skin ${_skin} --ldflags OUTPUT_VARIABLE XENO_${SKINU}_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
      PARSE_LDFLAGS("XENO_${SKINU}_LDFLAGS" "Xenomai_${SKINU}_LIBRARIES" "Xenomai_${SKINU}_LDFLAGS" "Xenomai_${SKINU}_LIBRARY_DIRS" "Xenomai_${SKINU}_DL_FLAGS")
      #message(STATUS "Xenomai_${SKINU}_LIBRARIES = ${Xenomai_${SKINU}_LIBRARIES}")
      #message(STATUS "Xenomai_${SKINU}_DIRS = ${Xenomai_${SKINU}_LIBRARY_DIRS}")
      #message(STATUS "Xenomai_${SKINU}_LDFLAGS = ${Xenomai_${SKINU}_LDFLAGS}")

      if(${Xenomai_VERSION} VERSION_GREATER "3.0.2") 
        #message(STATUS "xeno-config has the --auto-init-solib option")
	execute_process(COMMAND ${Xenomai_XENO_CONFIG} --skin ${_skin} --auto-init-solib --ldflags OUTPUT_VARIABLE XENO_${SKINU}_LDFLAGS_LIBRARIES OUTPUT_STRIP_TRAILING_WHITESPACE)
        PARSE_LDFLAGS("XENO_${SKINU}_LDFLAGS_LIBRARIES" "Xenomai_${SKINU}_LIBRARIES" "Xenomai_${SKINU}_LDFLAGS_LIBRARIES" "Xenomai_${SKINU}_LIBRARY_DIRS" "Xenomai_${SKINU}_DL_FLAGS_LIBRARIES")
      endif()
      if(Xenomai_${SKINU}_LIBRARIES AND Xenomai_${SKINU}_INCLUDE_DIRS)
         set(Xenomai_${SKINU}_FOUND TRUE)
         set(Xenomai_SKINS ${Xenomai_SKINS} ${_skin})
         #message(STATUS "Xenomai ${_skin} skin found")
         set(Xenomai_${SKINU}_FOUND ${Xenomai_${SKINU}_FOUND} CACHE STRING "Xenomai ${SKINU} Found" FORCE)
         set(Xenomai_${SKINU}_DEFINITIONS ${Xenomai_${SKINU}_DEFINITIONS} CACHE STRING "Xenomai ${SKINU} skin definitions" FORCE)
         set(Xenomai_${SKINU}_INCLUDE_DIRS ${Xenomai_${SKINU}_INCLUDE_DIRS} CACHE STRING "Xenomai ${SKINU} include directories" FORCE)
         set(Xenomai_${SKINU}_LIBRARY_DIRS ${Xenomai_${SKINU}_LIBRARY_DIRS} CACHE STRING "Xenomai ${SKINU} library directories" FORCE)
         set(Xenomai_${SKINU}_LIBRARIES ${Xenomai_${SKINU}_LIBRARIES}  CACHE STRING "Xenomai ${SKINU} libraries" FORCE)
         set(Xenomai_${SKINU}_LDFLAGS ${Xenomai_${SKINU}_LDFLAGS} CACHE STRING "Xenomai ${SKINU} ldflags" FORCE)
	 set(Xenomai_${SKINU}_DL_FLAGS ${Xenomai_${SKINU}_DL_FLAGS} CACHE STRING "Xenomai ${SKINU} Direct linker flags" FORCE)	 
 	 if(${Xenomai_VERSION} VERSION_GREATER "3.0.2") 
            set(Xenomai_${SKINU}_LDFLAGS_LIBRARIES ${Xenomai_${SKINU}_LDFLAGS_LIBRARIES} CACHE STRING "Xenomai ${SKINU} ldflags for libraries" FORCE)
	    set(Xenomai_${SKINU}_DL_FLAGS_LIBRARIES ${Xenomai_${SKINU}_DL_FLAGS_LIBRARIES} CACHE STRING "Xenomai ${SKINU} Direct linker flags for libraries" FORCE)
         endif()
      endif()
   endforeach(_skin ${SKINS})
   set(Xenomai_KERNEL_INCLUDE_DIR ${Xenomai_KERNEL_INCLUDE_DIR} CACHE STRING "Xenomai Kernel Headers directory " FORCE)
  
   find_library( Xenomai_LIBRARY_NATIVE     native     ${Xenomai_ROOT_DIR}/lib )
   find_library( Xenomai_LIBRARY_XENOMAI    xenomai    ${Xenomai_ROOT_DIR}/lib )
   find_library( Xenomai_LIBRARY_PTHREAD_RT pthread_rt ${Xenomai_ROOT_DIR}/lib )
   find_library( Xenomai_LIBRARY_RTDM       rtdm       ${Xenomai_ROOT_DIR}/lib )

   # In 2.6.0 RTDK was merged into the main xenomai library
   if(Xenomai_VERSION VERSION_GREATER 2.6.0)
      set(Xenomai_LIBRARY_RTDK_FOUND ${Xenomai_LIBRARY_XENOMAI_FOUND})
      set(Xenomai_LIBRARY_RTDK ${Xenomai_LIBRARY_XENOMAI})
   else()
      find_library( Xenomai_LIBRARY_RTDK rtdk ${Xenomai_ROOT_DIR}/lib )
   endif()
 
   find_package_handle_standard_args(Xenomai REQUIRED_VARS ${_XENOMAI_FOUND_REQUIRED_VARS} 
                                  VERSION_VAR Xenomai_VERSION
                                  HANDLE_COMPONENTS )
else(Xenomai_XENO_CONFIG)
   message(FATAL_ERROR "This program needs xeno-config")
endif()

