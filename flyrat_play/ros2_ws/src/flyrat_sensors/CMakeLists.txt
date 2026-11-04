cmake_minimum_required(VERSION 3.16)

project(flyrat_sensors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flyrat_sensors_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# The wire protocol is a header-only part of libflyrat. Do not link the whole
# library here: the receiver runs on the development machine and only needs the
# serialization definitions.
add_executable(sensor_receiver_node src/sensor_receiver_node.cpp)
target_include_directories(sensor_receiver_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat/include"
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(sensor_receiver_node PRIVATE
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${flyrat_sensors_interfaces_TARGETS}
  ${OpenCV_LIBS})
target_compile_options(sensor_receiver_node PRIVATE -Wall -Wextra)

install(TARGETS sensor_receiver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/adb_forward.sh
  DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
