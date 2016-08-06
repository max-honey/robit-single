cmake_minimum_required(VERSION 2.8.3)
project(movectrl CXX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11 -Wno-unused-parameter")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  serial
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ControlerState.msg
  MoveCmd.msg
  SelfPos.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetState.srv
  SetSpeed.srv
  SetBallCatcher.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp serial std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(movectrl
  src/movectrl_main.cpp
  src/controller.cpp
)
target_link_libraries(movectrl
  ${catkin_LIBRARIES}
)
add_dependencies(movectrl movectrl_gencpp serial_gencpp)

add_executable(movectrl_circle_test
  test/movectrl_circle_test.cpp
)
target_link_libraries(movectrl_circle_test
  ${catkin_LIBRARIES}
)
add_dependencies(movectrl_circle_test movectrl_gencpp)

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_movectrl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
