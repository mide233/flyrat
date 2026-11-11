# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json for clangd; .script/export_compile_commands.sh
# merges the per-project databases into one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat_pilot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The pilot does not use the MPC/attitude core and cross builds should not need
# OSQP, tests or tools.
set(FLYRAT_BUILD_CORE OFF CACHE BOOL "" FORCE)
set(FLYRAT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(FLYRAT_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
option(FLYRAT_STATIC_DEPS
  "Link the static OpenCV/glog/yaml-cpp found in CMAKE_SYSROOT" OFF)

# flyrat_pilot is the ROS-free program that runs on the real vehicle. It links
# the odometry, hardware and MSP layers straight from libflyrat, so it shares
# the exact same code path as the simulator without depending on ROS 2, Gazebo
# or colcon.
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../libflyrat"
                 "${CMAKE_CURRENT_BINARY_DIR}/libflyrat")

if(NOT FLYRAT_STATIC_DEPS)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
  find_package(yaml-cpp REQUIRED)
else()
  # Re-run the package lookup in this directory so the imported targets are
  # visible here; the parent scope only sees their transitive link interface.
  find_package(OpenCV CONFIG REQUIRED COMPONENTS
    core imgproc imgcodecs highgui calib3d video)
  set(OpenCV_INCLUDE_DIRS "${FLYRAT_OPENCV_INCLUDE_DIRS}")
endif()
find_package(Threads REQUIRED)

add_executable(flyrat_pilot
  src/main.cpp
  src/slam/imu_converter.cpp
  src/slam/pose_stream.cpp)
target_include_directories(flyrat_pilot PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flyrat_pilot PRIVATE
  flyrat::odometry
  flyrat::flyrat_msp_interact
  flyrat::flyrat_hardware
  ${OpenCV_LIBS}
  yaml-cpp
  Threads::Threads)
target_compile_options(flyrat_pilot PRIVATE -Wall -Wextra)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  target_compile_options(flyrat_pilot PRIVATE -mcpu=cortex-a55 -mtune=cortex-a55)
endif()

add_executable(flyrat_sensor_forwarder
  src/sensors/sensor_forwarder.cpp
  src/sensors/sensor_forwarder_main.cpp)
target_link_libraries(flyrat_sensor_forwarder PRIVATE
  flyrat::flyrat_msp_interact
  flyrat::flyrat_hardware)
target_compile_options(flyrat_sensor_forwarder PRIVATE -Wall -Wextra)
