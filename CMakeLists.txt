cmake_minimum_required(VERSION 3.0.2)
project(sonify_it)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

