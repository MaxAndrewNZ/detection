cmake_minimum_required(VERSION 2.8.3)
project(yolo_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs  
  tf2_py
  tf2_ros
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
 )
