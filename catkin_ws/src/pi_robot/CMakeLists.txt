cmake_minimum_required(VERSION 2.8.3)
project(pi_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  #sensor_msgs
  #geometry_msgs
  #message_generation 
)

catkin_python_setup()

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  #sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  #message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
