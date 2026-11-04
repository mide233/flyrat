cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. scripts/export_compile_commands.sh merges the per-project databases.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Pull the SLAM facade and the vendored ORB-SLAM3 straight from the source
# tree. The core library is a plain CMake project and stays ROS-free.
set(FLYRAT_BUILD_SLAM ON CACHE BOOL "" FORCE)
set(FLYRAT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLYRAT_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat"
  "${CMAKE_CURRENT_BINARY_DIR}/flyrat")

add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node PRIVATE
  flyrat::slam
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  message_filters::message_filters
  ${OpenCV_LIBS})
target_compile_options(slam_node PRIVATE -Wall -Wextra)

install(TARGETS slam_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/configs/orb_slam3/uvc_640x480_imu.yaml"
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
