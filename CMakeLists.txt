cmake_minimum_required(VERSION 2.8.3)
project(marauders_map)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pathPlanner src/main.cpp src/pathPlanner.cpp src/obsDetector.cpp)

target_link_libraries(pathPlanner
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")

  add_rostest_gtest(
    pathPlannerTest
    test/pathPlannerTest.launch
    test/main.cpp
    test/pathPlannerTest.cpp
    test/obsDetectorTest.cpp
    src/pathPlanner.cpp
    src/obsDetector.cpp
  )

  target_link_libraries(
    pathPlannerTest
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    pathPlannerTest
    pathPlanner
    ${catkin_EXPORTED_TARGETS}
  )
endif()
