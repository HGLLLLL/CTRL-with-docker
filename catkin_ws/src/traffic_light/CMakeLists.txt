cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/traffic_light_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
