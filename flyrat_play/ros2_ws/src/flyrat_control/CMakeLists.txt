cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. scripts/export_compile_commands.sh merges the per-project databases
# into one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Pull the ROS-free libflyrat core straight from the source tree. It is a plain
# CMake project so it can also be built and tested without ROS.
set(FLYRAT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../../libflyrat"
  "${CMAKE_CURRENT_BINARY_DIR}/flyrat")

add_executable(control_node src/control_node.cpp)
target_link_libraries(control_node PRIVATE
  flyrat::flyrat
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS})
target_compile_options(control_node PRIVATE -Wall -Wextra)

install(TARGETS control_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
