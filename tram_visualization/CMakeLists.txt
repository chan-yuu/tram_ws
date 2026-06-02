cmake_minimum_required(VERSION 3.0.2)
project(tram_visualization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  tram_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs visualization_msgs tf tram_msgs
)

install(PROGRAMS scripts/viz_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
