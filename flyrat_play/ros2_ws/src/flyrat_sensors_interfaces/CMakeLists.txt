cmake_minimum_required(VERSION 3.16)

project(flyrat_sensors_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TofZone.msg"
  "msg/TofFrame.msg"
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()
