cmake_minimum_required(VERSION 3.0.2)
project(arduino_mega_ctrl)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  rosserial_python
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arduino_mega_ctrl
#  CATKIN_DEPENDS rospy std_msgs rosserial_python
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/move_straight_5s.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)