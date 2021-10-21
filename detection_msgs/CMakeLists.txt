cmake_minimum_required(VERSION 2.8.3)
project(detection_msgs)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

file(GLOB msg_files "msg/*.msg")
foreach (file ${msg_files})
    get_filename_component(file_short ${file} NAME)
    list(APPEND msg_base_files ${file_short})
endforeach ()

add_message_files(DIRECTORY msg FILES ${msg_base_files})
message(STATUS "+++ Messages: ${msg_base_files}")

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs
 )
