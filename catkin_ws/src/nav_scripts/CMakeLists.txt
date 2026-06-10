cmake_minimum_required(VERSION 3.0.2)
project(nav_scripts)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 安裝 Python 節點到 package 執行路徑，安裝後可用 rosrun nav_scripts <檔名> 呼叫
catkin_install_python(PROGRAMS
  scripts/lidar_filter_node.py
  scripts/odometry_node.py
  scripts/lidar_odom_nav_node.py
  scripts/pure_odom_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
