cmake_minimum_required(VERSION 3.0.2)
project(Depth-Anything)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/depth_anything_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
