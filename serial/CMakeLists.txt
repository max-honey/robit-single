cmake_minimum_required(VERSION 2.8.3)
project(serial)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11 -Wno-unused-parameter")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
)

add_message_files(
 FILES
 shootcmd.msg
 speedcmd.msg
 feedback.msg
)
add_service_files(
  FILES
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asd
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serialport src/serialport.cpp)
target_link_libraries(serialport ${catkin_LIBRARIES})
add_dependencies(serialport serial_gencpp)

add_executable(serialtest src/serialtest.cpp)
target_link_libraries(serialtest ${catkin_LIBRARIES})
add_dependencies(serialtest serial_gencpp)

