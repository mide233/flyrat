cmake_minimum_required(VERSION 3.16)
project(flyrat_msp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Plain C++ library wrapping the SITL links, so non-ROS tools (and tests) can
# reuse the MSP client and the virtual RC receiver.
add_library(flyrat_msp_client
  src/msp_client.cpp
  src/rc_receiver.cpp)
target_include_directories(flyrat_msp_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(flyrat_msp_client PRIVATE -Wall -Wextra)

add_executable(msp_node src/msp_node.cpp)
target_link_libraries(msp_node PRIVATE
  flyrat_msp_client
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})
target_compile_options(msp_node PRIVATE -Wall -Wextra)

install(TARGETS flyrat_msp_client msp_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(flyrat_msp_client)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs std_msgs std_srvs)

ament_package()
