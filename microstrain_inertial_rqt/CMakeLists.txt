cmake_minimum_required(VERSION 3.0.2)
project(microstrain_inertial_rqt)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  rqt_gui
  rqt_gui_py
  std_msgs
  nav_msgs
  geometry_msgs
  microstrain_inertial_msgs
)

# Locate the common code and messages
set(COMMON_NAME "microstrain_inertial_rqt_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/${COMMON_NAME}")

# Make sure that the submodule has been properly initialized
if(NOT EXISTS "${COMMON_DIR}/resource")
  message(STATUS "Initializing ${COMMON_DIR} submodule. This should only happen once.")

  # Make sure we can find the git executable
  find_package(Git)
  if(NOT Git_FOUND)
    message(FATAL_ERROR "Unable to initialize submodule because we could not find the git executable. Please clone this repo using 'git clone --recursive'")
  endif()

  # Initialize and update the submodule  )
  execute_process(
    WORKING_DIRECTORY "${${PROJECT_NAME}_SOURCE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} submodule update --recursive --init
  )
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS
    rospy
    tf
    rqt_gui
    rqt_gui_py
    std_msgs
    nav_msgs
    geometry_msgs
    microstrain_inertial_msgs
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY
${COMMON_NAME}/resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${COMMON_NAME}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
${COMMON_NAME}/plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${COMMON_NAME}
)
