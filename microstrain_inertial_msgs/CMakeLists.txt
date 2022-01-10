cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)
project(microstrain_inertial_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Locate the common code and messages
set(COMMON_NAME "microstrain_inertial_msgs_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/${COMMON_NAME}")

# Make sure that the submodule has been properly initialized
if(NOT EXISTS "${COMMON_DIR}/msg")
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

# Generate Service Files
set(SRV_DIR "${COMMON_DIR}/srv")
FILE(GLOB SRV_FILES RELATIVE "${SRV_DIR}"
    "${SRV_DIR}/*.srv")
list(TRANSFORM SRV_FILES PREPEND ${COMMON_DIR}/srv:)

# Generate Message Files
set(MSG_DIR "${COMMON_DIR}/msg")
FILE(GLOB MSG_FILES RELATIVE "${MSG_DIR}"
    "${MSG_DIR}/*.msg")
list(TRANSFORM MSG_FILES PREPEND ${COMMON_DIR}/msg:)

rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} ${SRV_FILES} DEPENDENCIES std_msgs geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()