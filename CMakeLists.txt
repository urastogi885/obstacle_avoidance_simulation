cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	genmsg
	tf
)

## Declare ROS messages and services
add_service_files(
	FILES 
	modifyMessage.srv
)

## Generate added messages and services
generate_messages(
	DEPENDENCIES
	std_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_gencpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_gencpp)

## Enable catkin test and add files for the same
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
  modifyMessageTest
  test/modifyMessageTest.launch
  test/modifyMessageTest.cpp
  )

  target_link_libraries(
  modifyMessageTest
  ${catkin_LIBRARIES}
  )

  add_dependencies(
  modifyMessageTest
  talker
  listener
  ${catkin_EXPORTED_TARGETS}
  )
endif()