# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. scripts/export_compile_commands.sh merges the per-project databases
# into one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat_autopilot VERSION 0.1.0 LANGUAGES CXX)

# The autopilot core is deliberately ROS-free. It builds as a standalone CMake
# project (cmake -B build && cmake --build build && ctest --test-dir build) and
# can also be dropped into a ROS 2 package via add_subdirectory().
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(FLYRAT_AUTOPILOT_STANDALONE ON)
else()
  set(FLYRAT_AUTOPILOT_STANDALONE OFF)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(flyrat_autopilot
  src/attitude_hold.cpp)
add_library(flyrat::autopilot ALIAS flyrat_autopilot)

target_include_directories(flyrat_autopilot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(flyrat_autopilot PRIVATE -Wall -Wextra)

install(TARGETS flyrat_autopilot
  EXPORT flyrat_autopilot_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
if(FLYRAT_AUTOPILOT_STANDALONE)
  install(EXPORT flyrat_autopilot_targets
    NAMESPACE flyrat:: DESTINATION lib/cmake/flyrat_autopilot)
endif()

# Tests are only built when this project owns the build (standalone) or when a
# parent explicitly enables them. Downstream ROS packages skip them by default.
if(FLYRAT_AUTOPILOT_STANDALONE)
  set(FLYRAT_AUTOPILOT_BUILD_TESTS ON)
endif()
option(FLYRAT_AUTOPILOT_BUILD_TESTS "Build autopilot unit tests" OFF)

if(FLYRAT_AUTOPILOT_BUILD_TESTS)
  enable_testing()
  add_executable(test_attitude_hold tests/test_attitude_hold.cpp)
  target_link_libraries(test_attitude_hold PRIVATE flyrat_autopilot)
  target_compile_options(test_attitude_hold PRIVATE -Wall -Wextra)
  add_test(NAME test_attitude_hold COMMAND test_attitude_hold)
endif()
