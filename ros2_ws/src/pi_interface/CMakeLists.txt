cmake_minimum_required(VERSION 3.5)
project(pi_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorSetSpeed.msg"
  "msg/MotorSetPosition.msg"
  "msg/ButtonEvent.msg"
  "msg/MotorInfo.msg"
  "msg/ServoInfo.msg"
  "msg/Sensor3Axes.msg"
  "msg/SensorStatusChange.msg"
  "msg/SensorValueChange.msg"
  "msg/NineAxisValue.msg"
  "msg/NineAxisValueChange.msg"
  "msg/U8Int32.msg"
  "msg/ApriltagPose.msg"
  "msg/FaceDetection.msg"
  "msg/LineDetection.msg"
  "msg/ObjectDetection.msg"
  "srv/MotorGetPosition.srv"
  "srv/GetInt32.srv"
  "srv/SetInt32.srv"
  "srv/GetMotorsInfo.srv"
  "srv/GetServosInfo.srv"
  "srv/SetServoPosition.srv"
  "srv/SetServoParam.srv"
  "srv/SensorGet3Axes.srv"
  "srv/GetPowerState.srv"
  "srv/GetSensorInfo.srv"
  "srv/GetString.srv"
  "srv/SetString.srv"
  "srv/SetOffset.srv"
  "srv/GetStrings.srv"
  "srv/GetCommandDetection.srv"
  "srv/GetApriltagDetections.srv"
  "srv/GetPredictions.srv"
  "srv/GetFaceDetections.srv"
  "srv/GetLineDetection.srv"
  "srv/GetFrame.srv"
  "srv/GetCompressedFrame.srv"
  "srv/SetPid.srv"
  "srv/SetRoi.srv"
  "srv/SetColorThreshold.srv"
  "srv/GetColorThreshold.srv"
  "srv/GetObjectDetections.srv"
  "srv/AddProc.srv"
  DEPENDENCIES sensor_msgs
)

# add_executable(talker src/publisher_member_function.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
#   talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()