cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. .script/export_compile_commands.sh merges the per-project databases into
# one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# libflyrat owns the ROS-free transport (MSP client + UDP RC receiver); this
# package is only the ROS 2 adapter around it.
add_subdirectory(
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat"
  "${CMAKE_CURRENT_BINARY_DIR}/flyrat")

add_executable(msp_node src/msp_node.cpp)
target_link_libraries(msp_node PRIVATE
  flyrat::flyrat
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})
target_compile_options(msp_node PRIVATE -Wall -Wextra)

install(TARGETS msp_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp geometry_msgs sensor_msgs std_msgs std_srvs)

ament_package()
