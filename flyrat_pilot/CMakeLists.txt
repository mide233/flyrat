# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json for clangd; .script/export_compile_commands.sh
# merges the per-project databases into one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat_pilot VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# flyrat_pilot is the ROS-free program that runs on the real vehicle. It links
# the shared core (algorithms + MSP/UDP transport) straight from libflyrat, so
# it shares the exact same code path as the simulator without depending on
# ROS 2, Gazebo or colcon.
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../libflyrat"
                 "${CMAKE_CURRENT_BINARY_DIR}/libflyrat")

add_executable(flyrat_pilot src/main.cpp)
target_link_libraries(flyrat_pilot PRIVATE flyrat::flyrat)
target_compile_options(flyrat_pilot PRIVATE -Wall -Wextra)
