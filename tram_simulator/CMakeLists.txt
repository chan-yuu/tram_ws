cmake_minimum_required(VERSION 3.0.2)
project(tram_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf
  tram_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs std_srvs tf tram_msgs
)

install(PROGRAMS scripts/sim_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
