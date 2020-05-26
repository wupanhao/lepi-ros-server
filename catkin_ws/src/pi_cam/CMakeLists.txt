cmake_minimum_required(VERSION 2.8.3)
project(pi_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation 
)

#catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ApriltagPose.msg
  FaceDetection.msg
  LineDetection.msg
  ObjectDetection.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetApriltagDetections.srv
  GetPredictions.srv
  GetFaceDetections.srv
  GetLineDetection.srv
  GetFrame.srv
  SetPid.srv
  SetColorThreshold.srv
  GetColorThreshold.srv
  GetObjectDetections.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

