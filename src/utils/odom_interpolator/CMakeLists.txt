cmake_minimum_required(VERSION 3.0.2)
project(odom_interpolator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odom_interpolator_node src/odom_interpolator.cpp)
target_link_libraries(odom_interpolator_node ${catkin_LIBRARIES})

