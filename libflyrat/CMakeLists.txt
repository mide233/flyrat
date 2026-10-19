# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. .script/export_compile_commands.sh merges the per-project databases
# into one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat VERSION 0.1.0 LANGUAGES CXX)

# libflyrat is the ROS-free core shared by the simulator and the real machine:
# the autopilot algorithms plus the Betaflight MSP / UDP transport. It builds as
# a standalone CMake project (cmake -B build && cmake --build build && ctest
# --test-dir build) and can also be dropped into a ROS 2 package via
# add_subdirectory().
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(FLYRAT_STANDALONE ON)
else()
  set(FLYRAT_STANDALONE OFF)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(flyrat
  src/attitude_hold.cpp
  src/msp_client.cpp
  src/rc_receiver.cpp)
add_library(flyrat::flyrat ALIAS flyrat)

target_include_directories(flyrat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flyrat PUBLIC Threads::Threads)
target_compile_options(flyrat PRIVATE -Wall -Wextra)

install(TARGETS flyrat
  EXPORT flyrat_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
if(FLYRAT_STANDALONE)
  install(EXPORT flyrat_targets
    NAMESPACE flyrat:: DESTINATION lib/cmake/flyrat)
endif()

# Tests are only built when this project owns the build (standalone) or when a
# parent explicitly enables them. Downstream ROS packages skip them by default.
if(FLYRAT_STANDALONE)
  set(FLYRAT_BUILD_TESTS ON)
endif()
option(FLYRAT_BUILD_TESTS "Build libflyrat unit tests" OFF)

if(FLYRAT_BUILD_TESTS)
  enable_testing()
  add_executable(test_attitude_hold tests/test_attitude_hold.cpp)
  target_link_libraries(test_attitude_hold PRIVATE flyrat)
  target_compile_options(test_attitude_hold PRIVATE -Wall -Wextra)
  add_test(NAME test_attitude_hold COMMAND test_attitude_hold)
endif()
