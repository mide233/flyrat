cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. scripts/export_compile_commands.sh merges the per-project databases.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat_odometry)

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

# TIO-Former needs libtorch and the exported TorchScript model.  Keep it opt-in
# so the default sensor/odometry workspace builds on machines without libtorch.
option(FLYRAT_BUILD_TIO "Build the TIO-Former ToF-inertial odometry node" OFF)
if(FLYRAT_BUILD_TIO)
  find_package(Torch REQUIRED)
  find_package(flyrat_sensors_interfaces REQUIRED)
endif()

# Pull the odometry interface straight from the source tree. The core library
# is a plain CMake project and stays ROS-free.
set(FLYRAT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLYRAT_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat"
  "${CMAKE_CURRENT_BINARY_DIR}/flyrat")

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node PRIVATE
  flyrat::odometry
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_ros::tf2_ros
  message_filters::message_filters
  ${OpenCV_LIBS})
target_compile_options(odometry_node PRIVATE -Wall -Wextra)

install(TARGETS odometry_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/configs/camera/uvc_640x480.yaml"
  DESTINATION share/${PROJECT_NAME}/config)

if(FLYRAT_BUILD_TIO)
  add_executable(tio_odometry_node src/tio_odometry_node.cpp)
  target_link_libraries(tio_odometry_node PRIVATE
    flyrat::tio
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${flyrat_sensors_interfaces_TARGETS}
    tf2_ros::tf2_ros
    message_filters::message_filters)
  target_compile_options(tio_odometry_node PRIVATE -Wall -Wextra)

  install(TARGETS tio_odometry_node RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/models/tio_former_window64.pt"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/models/tio_former_window64.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/models/tio_former_window64_camera.pt"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/models/tio_former_window64_camera.json"
    DESTINATION share/${PROJECT_NAME}/models)
endif()

ament_package()
