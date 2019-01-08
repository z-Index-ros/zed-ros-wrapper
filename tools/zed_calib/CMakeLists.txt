cmake_minimum_required(VERSION 2.8.7)

project(zed_calib)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

find_package(ZED 2.3)
checkPackage("ZED" "ZED SDK not found, install it from:\n https://www.stereolabs.com/developers/")

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX
    SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDA)
checkPackage("CUDA" "CUDA not found, install it from:\n https://developer.nvidia.com/cuda-downloads")

find_package(OpenMP)
checkPackage("OpenMP" "OpenMP not found, please install it to improve performances: 'sudo apt install libomp-dev'")
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin COMPONENTS
  roscpp
  rosconsole
  std_msgs
  dynamic_reconfigure
  message_generation  
)

checkPackage("roscpp" "")
checkPackage("rosconsole" "")
checkPackage("std_msgs" "")
checkPackage("dynamic_reconfigure" "")
checkPackage("message_generation" "")

add_message_files( FILES
    calib_stamped.msg
)

add_service_files( FILES
    reset_means.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/calib.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    std_msgs
    dynamic_reconfigure       
    message_runtime
)

###############################################################################
# SOURCES

set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_calib_node.cpp
             ${CMAKE_CURRENT_SOURCE_DIR}/src/calib/src/zedcalib.cpp
)

###############################################################################

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/calib/include/
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE

if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})   
endif()


add_definitions(-std=c++11)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
  )

add_executable(zed_calib_node ${NODE_SRC})
target_link_libraries(zed_calib_node ${LINK_LIBRARIES})
add_dependencies(zed_calib_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INSTALL

install(TARGETS
  zed_calib_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY
  launch
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
