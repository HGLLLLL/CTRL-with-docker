cmake_minimum_required(VERSION 3.0.2)
project(lane_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS rospy sensor_msgs cv_bridge message_runtime
#  DEPENDS system_lib
)

add_message_files(
  FILES
  LaneData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安裝 Python scripts 到 package 執行路徑
catkin_install_python(PROGRAMS
  scripts/lane_detect.py
  scripts/lane_controller.py
  scripts/lane_controller_fuzzy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)