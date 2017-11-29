cmake_minimum_required(VERSION 2.8.3)
project(marauders_map)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(include  ${catkin_INCLUDE_DIRS})
