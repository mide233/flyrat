cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat_pilot_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(Threads REQUIRED)

# The wire protocol is header-only.
add_executable(pilot_bridge_node src/pilot_bridge_node.cpp)
target_include_directories(pilot_bridge_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/include"
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(pilot_bridge_node PRIVATE
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${OpenCV_LIBS}
  Threads::Threads)
target_compile_options(pilot_bridge_node PRIVATE -Wall -Wextra)

install(TARGETS pilot_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/configs/camera/uvc_640x480.yaml"
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
