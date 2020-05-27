cmake_minimum_required(VERSION 2.8.3)
project(pi_ai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ObjectDetection.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetObjectDetections.srv
)

catkin_python_setup()

generate_messages(
  #DEPENDENCIES
  #sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

