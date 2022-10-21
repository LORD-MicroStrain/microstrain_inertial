cmake_minimum_required(VERSION 3.5)
project(microstrain_inertial_driver)

# C++ 14 required
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Force some options for this module and the MIP SDK
set(BUILD_SHARED_LIBS_TEMP "${BUILD_SHARED_LIBS}")
set(BUILD_EXAMPLES_TEMP "${BUILD_EXAMPLES}")
set(BUILD_TESTING_TEMP "${BUILD_TESTING}")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# Locate the common code and messages
set(COMMON_NAME "microstrain_inertial_driver_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/${COMMON_NAME}")
set(COMMON_SRC_DIR "${COMMON_DIR}/src")
set(COMMON_INC_DIR "${COMMON_DIR}/include/${COMMON_NAME}")
set(MIP_SDK_DIR "${COMMON_DIR}/mip_sdk")
set(MIP_SDK_SRC_DIR "${MIP_SDK_DIR}/src")

# Make sure that the submodule has been properly initialized
if(NOT EXISTS "${COMMON_DIR}/src")
  message(STATUS "Initializing ${COMMON_DIR} submodule. This should only happen once.")

  # Make sure we can find the git executable
  find_package(Git)
  if(NOT Git_FOUND)
    message(FATAL_ERROR "Unable to initialize submodule because we could not find the git executable. Please clone this repo using 'git clone --recursive'")
  endif()

  # Initialize and update the submodule
  execute_process(
    WORKING_DIRECTORY "${${PROJECT_NAME}_SOURCE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} submodule update --init --recursive
  )
endif()

# Include the MIP SDK source
add_subdirectory("${MIP_SDK_DIR}" EXCLUDE_FROM_ALL)
include_directories("${MIP_SDK_SRC_DIR}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-implicit-function-declaration -Wno-incompatible-pointer-types -fno-builtin-memcpy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-builtin-memcpy")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(microstrain_inertial_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
ament_export_include_directories()
ament_export_dependencies(
    roscpp
    rosidl_default_runtime
    cmake_modules
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    nmea_msgs
    mavros_msgs
    message_runtime
		microstrain_inertial_msgs
)

###########
## Build ##
###########

# Add the catkin includes
include_directories(${catkin_INCLUDE_DIRS} include ${COMMON_DIR}/include)

# Only lint the ROS2 code as the ROS1 code is linted through ROS1 and having two competing standards would make things messy quick
set(NODE_SRC_FILES
  src/microstrain_inertial_driver_node.cpp
)

ament_package()

set(LIB_SRC_FILES
  ${COMMON_SRC_DIR}/subscribers.cpp
  ${COMMON_SRC_DIR}/publishers.cpp
  ${COMMON_SRC_DIR}/node_common.cpp
  ${COMMON_SRC_DIR}/services.cpp
  ${COMMON_SRC_DIR}/config.cpp

  ${COMMON_SRC_DIR}/utils/mappings/mip_publisher_mapping.cpp

  ${COMMON_SRC_DIR}/utils/mip/mip_device_wrapper.cpp
  ${COMMON_SRC_DIR}/utils/mip/mip_device_serial.cpp

  src/microstrain_inertial_driver.cpp
)
set(LIB_INC_FILES
  ${COMMON_INC_DIR}/subscribers.h
  ${COMMON_INC_DIR}/publishers.h
  ${COMMON_INC_DIR}/node_common.h
  ${COMMON_INC_DIR}/services.h
  ${COMMON_INC_DIR}/config.h
  ${COMMON_INC_DIR}/utils/ros_compat.h

  ${COMMON_INC_DIR}/utils/mappings/mip_mapping.h
  ${COMMON_INC_DIR}/utils/mappings/mip_publisher_mapping.h

  ${COMMON_INC_DIR}/utils/mip/mip_device_wrapper.h
  ${COMMON_INC_DIR}/utils/mip/mip_device_serial.h

  include/${PROJECT_NAME}/microstrain_inertial_driver.h
)
add_library(${PROJECT_NAME} ${LIB_SRC_FILES} ${LIB_INC_FILES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  microstrain_inertial_msgs
)

# Executables
add_executable(${PROJECT_NAME}_node ${NODE_SRC_FILES} ${NODE_INC_FILES})

# Annoying, but the ROS types don't match up with the MIP types for floats and doubles, so ignore those warnings for now
set_source_files_properties(${COMMON_SRC_DIR}/services.cpp PROPERTIES COMPILE_OPTIONS "-Wno-narrowing")

# Let the code know if it is being compiled with ROS1 or ROS2
if(DEFINED ENV{ROS_VERSION})
  add_definitions(-DMICROSTRAIN_ROS_VERSION=$ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is not set.")
endif()

# Some ROS version specific defines
if(DEFINED ENV{ROS_DISTRO})
  if ("$ENV{ROS_DISTRO}" STREQUAL "rolling")
    add_definitions(-DMICROSTRAIN_ROLLING=1)
  elseif ("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_definitions(-DMICROSTRAIN_HUMBLE=1)
  elseif ("$ENV{ROS_DISTRO}" STREQUAL "galactic")
    add_definitions(-DMICROSTRAIN_GALACTIC=1)
  elseif ("$ENV{ROS_DISTRO}" STREQUAL "foxy")
    add_definitions(-DMICROSTRAIN_FOXY=1)
  else()
    add_definitions(-DMICROSTRAIN_ROLLING=1)  # By default assume this is a newer ROS version than we support, so rolling define is the safest
  endif()
endif()

# Linking
target_link_libraries(${PROJECT_NAME}
  mip
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${COMMON_DIR}/config
  DESTINATION share/${PROJECT_NAME}/${COMMON_NAME}
)

# Reset the forced options
set(BUILD_SHARED_LIBS OFF CACHE BOOL "${BUILD_SHARED_LIBS_TEMP}" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "${BUILD_EXAMPLES_TEMP}" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "${BUILD_TESTING_TEMP}" FORCE)