cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_obstacle_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  sensor_msgs
  geometry_msgs
  move_base_msgs
)

## Declare a catkin package
catkin_package()

## Build target library
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlebot_obstacle_avoidance src/obstacle_avoidance.cpp src/main.cpp)
target_link_libraries(turtlebot_obstacle_avoidance ${catkin_LIBRARIES})