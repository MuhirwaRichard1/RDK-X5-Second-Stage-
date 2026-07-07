cmake_minimum_required(VERSION 3.8)
project(navbot_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sectors.msg"
  "msg/GridOverlay.msg"
  "msg/Detections.msg"
  DEPENDENCIES std_msgs
)

ament_package()
