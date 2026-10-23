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

# Eigen for the manifold maths, OSQP for the QP the MPC solves every cycle.
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(osqp CONFIG QUIET)
if(NOT osqp_FOUND)
  # Fall back to a plain system install (libosqp-dev, Homebrew, or a build
  # pointed at by OSQP_ROOT). Keeps libflyrat usable outside a ROS workspace.
  find_path(FLYRAT_OSQP_INCLUDE_DIR osqp/osqp.h)
  find_library(FLYRAT_OSQP_LIBRARY NAMES osqp)
  if(NOT FLYRAT_OSQP_INCLUDE_DIR OR NOT FLYRAT_OSQP_LIBRARY)
    message(FATAL_ERROR "OSQP not found: install libosqp-dev or set OSQP_ROOT")
  endif()
endif()

add_library(flyrat
  src/attitude_hold.cpp
  src/local_estimator.cpp
  src/msp_client.cpp
    src/msp_telemetry.cpp
  src/ommpc.cpp
  src/rc_receiver.cpp)
add_library(flyrat::flyrat ALIAS flyrat)

target_include_directories(flyrat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
if(osqp_FOUND)
  target_link_libraries(flyrat PUBLIC Threads::Threads Eigen3::Eigen osqp::osqp)
else()
  target_include_directories(flyrat PUBLIC ${FLYRAT_OSQP_INCLUDE_DIR})
  target_link_libraries(flyrat PUBLIC Threads::Threads Eigen3::Eigen ${FLYRAT_OSQP_LIBRARY})
endif()
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

  add_executable(test_ommpc tests/test_ommpc.cpp)
  target_link_libraries(test_ommpc PRIVATE flyrat)
  target_compile_options(test_ommpc PRIVATE -Wall -Wextra)
  add_test(NAME test_ommpc COMMAND test_ommpc)

  add_executable(test_local_estimator tests/test_local_estimator.cpp)
  target_link_libraries(test_local_estimator PRIVATE flyrat)
  target_compile_options(test_local_estimator PRIVATE -Wall -Wextra)
  add_test(NAME test_local_estimator COMMAND test_local_estimator)
endif()
