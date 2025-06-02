cmake_minimum_required(VERSION 3.0.2)
project(com3528_individual)

## Compile as C++11, supported in ROS Noetic
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
)

## Declare ROS messages
add_message_files(
  FILES
  FaceData.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs message_runtime cv_bridge
)

include_directories(
  scripts/face_id_left_eye_node.py
  scripts/face_id_right_eye_node.py
  scripts/face_centering_node.py
  ${catkin_INCLUDE_DIRS}
)
