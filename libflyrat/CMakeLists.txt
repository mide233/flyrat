# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)

# Emit compile_commands.json so clangd/editors can resolve this translation
# unit. .script/export_compile_commands.sh merges the per-project databases
# into one file at the repository root.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(flyrat VERSION 0.1.0 LANGUAGES C CXX)

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

# ST VL53L5CX user-space driver (ULD). The output is trimmed to the blocks the
# flight stack needs: distance, target status, target count and temperature.
# Four sensors on one 750 kHz bus cannot afford the full 532-byte result block.
add_library(flyrat_vl53l5cx STATIC
  third_party/vl53l5cx/src/vl53l5cx_api.c
  third_party/vl53l5cx/src/vl53l5cx_plugin_detection_thresholds.c
  third_party/vl53l5cx/src/vl53l5cx_plugin_motion_indicator.c
  third_party/vl53l5cx/src/vl53l5cx_plugin_xtalk.c
  third_party/vl53l5cx/platform/platform.c)
target_include_directories(flyrat_vl53l5cx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/vl53l5cx/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/vl53l5cx/platform>)
# The VL53L5CX output-selection macros live in platform/platform.h, not here.
# They change the layout of VL53L5CX_ResultsData, so they must be identical for
# the C driver and every C++ consumer; defining them per-target broke that.

# Hardware access lives in its own target so the protocol layer can depend on
# the header-only Transport interface without pulling in a particular backend.
add_library(flyrat_hardware
  src/hardware/mjpeg_decoder.cpp
  src/hardware/serial_transport.cpp
  src/hardware/stereo_camera.cpp
  src/hardware/stereo_jpeg_source.cpp
  src/hardware/tcp_transport.cpp
  src/hardware/tof.cpp)
add_library(flyrat::flyrat_hardware ALIAS flyrat_hardware)
target_include_directories(flyrat_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flyrat_hardware PUBLIC Threads::Threads)
target_link_libraries(flyrat_hardware PRIVATE flyrat_vl53l5cx ${CMAKE_DL_LIBS})
target_compile_options(flyrat_hardware PRIVATE -Wall -Wextra)

# MSP protocol and scheduling. It only depends on the header-only Transport
# interface, so it can be tested with a fake transport and stays free of
# Eigen/OSQP.
add_library(flyrat_msp_interact
  src/msp_frame.cpp
  src/msp_parse.cpp
  src/msp_interact.cpp)
add_library(flyrat::flyrat_msp_interact ALIAS flyrat_msp_interact)
target_include_directories(flyrat_msp_interact PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(flyrat_msp_interact PUBLIC Threads::Threads)
target_compile_options(flyrat_msp_interact PRIVATE -Wall -Wextra)

# The IMU facade reads MspInteract snapshots, so hardware depends on the
# protocol layer (not the other way around).
target_link_libraries(flyrat_hardware PUBLIC flyrat_msp_interact)

install(TARGETS flyrat flyrat_hardware flyrat_msp_interact flyrat_vl53l5cx
  EXPORT flyrat_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES third_party/vl53l5cx/LICENSE DESTINATION share/doc/flyrat)
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
if(FLYRAT_STANDALONE)
  option(FLYRAT_BUILD_TOOLS "Build libflyrat command line tools" ON)
else()
  option(FLYRAT_BUILD_TOOLS "Build libflyrat command line tools" OFF)
endif()

if(FLYRAT_BUILD_TOOLS)
  add_executable(msp_interact_bench tools/msp_interact_bench.cpp)
  target_link_libraries(msp_interact_bench PRIVATE flyrat_msp_interact flyrat_hardware)
  target_compile_options(msp_interact_bench PRIVATE -Wall -Wextra)

  add_executable(tof_bench tools/tof_bench.cpp)
  target_link_libraries(tof_bench PRIVATE flyrat_hardware)
  target_compile_options(tof_bench PRIVATE -Wall -Wextra)

  add_executable(stereo_bench tools/stereo_bench.cpp)
  target_link_libraries(stereo_bench PRIVATE flyrat_hardware)
  target_compile_options(stereo_bench PRIVATE -Wall -Wextra)
endif()

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

  add_executable(test_transport tests/test_transport.cpp)
  target_link_libraries(test_transport PRIVATE flyrat_hardware)
  target_compile_options(test_transport PRIVATE -Wall -Wextra)
  # openpty() lives in libutil on glibc < 2.34.
  find_library(FLYRAT_UTIL_LIBRARY util)
  if(FLYRAT_UTIL_LIBRARY)
    target_link_libraries(test_transport PRIVATE ${FLYRAT_UTIL_LIBRARY})
  endif()
  add_test(NAME test_transport COMMAND test_transport)

  add_executable(test_msp_framing tests/test_msp_framing.cpp)
  target_link_libraries(test_msp_framing PRIVATE flyrat_msp_interact)
  target_compile_options(test_msp_framing PRIVATE -Wall -Wextra)
  add_test(NAME test_msp_framing COMMAND test_msp_framing)

  add_executable(test_msp_interact tests/test_msp_interact.cpp)
  target_link_libraries(test_msp_interact PRIVATE flyrat_msp_interact)
  target_compile_options(test_msp_interact PRIVATE -Wall -Wextra)
  add_test(NAME test_msp_interact COMMAND test_msp_interact)

  add_executable(test_hardware_headers tests/test_hardware_headers.cpp)
  target_link_libraries(test_hardware_headers PRIVATE flyrat_hardware)
  target_compile_options(test_hardware_headers PRIVATE -Wall -Wextra)
  add_test(NAME test_hardware_headers COMMAND test_hardware_headers)

  add_executable(test_sensor_protocol tests/test_sensor_protocol.cpp)
  target_include_directories(test_sensor_protocol PRIVATE include)
  target_compile_options(test_sensor_protocol PRIVATE -Wall -Wextra)
  add_test(NAME test_sensor_protocol COMMAND test_sensor_protocol)
endif()
