cmake_minimum_required(VERSION 3.0.2)
project(com3528_individual)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts (ROS nodes)
catkin_install_python(PROGRAMS
  scripts/startup_search_node.py
  scripts/authentication_node.py
  scripts/test_node.py
  # scripts/registration.py
  # Add more ROS node scripts here as needed
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
