cmake_minimum_required(VERSION 2.8.3)
project(motoman_basic_movement_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  moveit_ros_planning_interface
  tf
)

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
  roscpp
  cmake_modules
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motoman_basic_movement_test src/motoman_basic_movement_test.cpp)
target_link_libraries(motoman_basic_movement_test ${catkin_LIBRARIES})


